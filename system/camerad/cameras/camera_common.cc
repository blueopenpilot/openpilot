#include "system/camerad/cameras/camera_common.h"

#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <chrono>
#include <thread>

#include "libyuv.h"
#include <jpeglib.h>

#include "system/camerad/imgproc/utils.h"
#include "common/clutil.h"
#include "common/modeldata.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "system/hardware/hw.h"
#include "msm_media_info.h"

#ifdef QCOM2
#include "CL/cl_ext_qcom.h"
#include "system/camerad/cameras/camera_qcom2.h"
#else
#include "system/camerad/test/camera_test.h"
#endif

ExitHandler do_exit;

class Debayer {
public:
  Debayer(cl_device_id device_id, cl_context context, const CameraBuf *b, const CameraState *s, int buf_width, int uv_offset) {
    char args[4096];
    const CameraInfo *ci = &s->ci;
    hdr_ = ci->hdr;
    snprintf(args, sizeof(args),
             "-cl-fast-relaxed-math -cl-denorms-are-zero "
             "-DFRAME_WIDTH=%d -DFRAME_HEIGHT=%d -DFRAME_STRIDE=%d -DFRAME_OFFSET=%d "
             "-DRGB_WIDTH=%d -DRGB_HEIGHT=%d -DRGB_STRIDE=%d -DYUV_STRIDE=%d -DUV_OFFSET=%d "
             "-DBAYER_FLIP=%d -DHDR=%d -DCAM_NUM=%d%s",
             ci->frame_width, ci->frame_height, ci->frame_stride, ci->frame_offset,
             b->rgb_width, b->rgb_height, b->rgb_stride, buf_width, uv_offset,
             ci->bayer_flip, ci->hdr, s->camera_num, s->camera_num==1 ? " -DVIGNETTING" : "");
    const char *cl_file = "cameras/real_debayer.cl";
    cl_program prg_debayer = cl_program_from_file(context, device_id, cl_file, args);
    krnl_ = CL_CHECK_ERR(clCreateKernel(prg_debayer, "debayer10", &err));
    CL_CHECK(clReleaseProgram(prg_debayer));
  }

  void queue(cl_command_queue q, cl_mem cam_buf_cl, cl_mem buf_cl, int width, int height, cl_event *debayer_event) {
    CL_CHECK(clSetKernelArg(krnl_, 0, sizeof(cl_mem), &cam_buf_cl));
    CL_CHECK(clSetKernelArg(krnl_, 1, sizeof(cl_mem), &buf_cl));

    const size_t globalWorkSize[] = {size_t(width / 2), size_t(height / 2)};
    const int debayer_local_worksize = 16;
    const size_t localWorkSize[] = {debayer_local_worksize, debayer_local_worksize};
    CL_CHECK(clEnqueueNDRangeKernel(q, krnl_, 2, NULL, globalWorkSize, localWorkSize, 0, 0, debayer_event));
  }

  ~Debayer() {
    CL_CHECK(clReleaseKernel(krnl_));
  }

private:
  cl_kernel krnl_;
  bool hdr_;
};

void CameraBuf::init(cl_device_id device_id, cl_context context, CameraState *s, VisionIpcServer * v, int frame_cnt, VisionStreamType init_rgb_type, VisionStreamType init_yuv_type) {
  vipc_server = v;
  this->rgb_type = init_rgb_type;
  this->yuv_type = init_yuv_type;

  const CameraInfo *ci = &s->ci;
  camera_state = s;
  frame_buf_count = frame_cnt;

  // RAW frame
  const int frame_size = (ci->frame_height + ci->extra_height) * ci->frame_stride;
  camera_bufs = std::make_unique<VisionBuf[]>(frame_buf_count);
  camera_bufs_metadata = std::make_unique<FrameMetadata[]>(frame_buf_count);

  for (int i = 0; i < frame_buf_count; i++) {
    camera_bufs[i].allocate(frame_size);
    camera_bufs[i].init_cl(device_id, context);
  }
  LOGD("allocated %d CL buffers", frame_buf_count);

  rgb_width = ci->frame_width;
  rgb_height = ci->frame_height;

  yuv_transform = get_model_yuv_transform(ci->bayer);

  vipc_server->create_buffers(rgb_type, UI_BUF_COUNT, true, rgb_width, rgb_height);
  rgb_stride = vipc_server->get_buffer(rgb_type)->stride;
  LOGD("created %d UI vipc buffers with size %dx%d", UI_BUF_COUNT, rgb_width, rgb_height);

  int nv12_width = VENUS_Y_STRIDE(COLOR_FMT_NV12, rgb_width);
  int nv12_height = VENUS_Y_SCANLINES(COLOR_FMT_NV12, rgb_height);
  assert(nv12_width == VENUS_UV_STRIDE(COLOR_FMT_NV12, rgb_width));
  assert(nv12_height/2 == VENUS_UV_SCANLINES(COLOR_FMT_NV12, rgb_height));
  size_t nv12_size = 2346 * nv12_width;  // comes from v4l2_format.fmt.pix_mp.plane_fmt[0].sizeimage
  size_t nv12_uv_offset = nv12_width * nv12_height;
  vipc_server->create_buffers_with_sizes(yuv_type, YUV_BUFFER_COUNT, false, rgb_width, rgb_height, nv12_size, nv12_width, nv12_uv_offset);
  LOGD("created %d YUV vipc buffers with size %dx%d", YUV_BUFFER_COUNT, nv12_width, nv12_height);

  if (ci->bayer) {
    debayer = new Debayer(device_id, context, this, s, nv12_width, nv12_uv_offset);
  }
  rgb2yuv = std::make_unique<Rgb2Yuv>(context, device_id, rgb_width, rgb_height, rgb_stride);

#ifdef __APPLE__
  q = CL_CHECK_ERR(clCreateCommandQueue(context, device_id, 0, &err));
#else
  const cl_queue_properties props[] = {0};  //CL_QUEUE_PRIORITY_KHR, CL_QUEUE_PRIORITY_HIGH_KHR, 0};
  q = CL_CHECK_ERR(clCreateCommandQueueWithProperties(context, device_id, props, &err));
#endif
}

CameraBuf::~CameraBuf() {
  for (int i = 0; i < frame_buf_count; i++) {
    camera_bufs[i].free();
  }
  if (debayer) delete debayer;
  if (q) CL_CHECK(clReleaseCommandQueue(q));
}

bool CameraBuf::acquire() {
  if (!safe_queue.try_pop(cur_buf_idx, 50)) return false;

  if (camera_bufs_metadata[cur_buf_idx].frame_id == -1) {
    LOGE("no frame data? wtf");
    release();
    return false;
  }

  cur_frame_data = camera_bufs_metadata[cur_buf_idx];
  cur_rgb_buf = vipc_server->get_buffer(rgb_type);
  cur_yuv_buf = vipc_server->get_buffer(yuv_type);
  cl_mem camrabuf_cl = camera_bufs[cur_buf_idx].buf_cl;
  cl_event event;

  double start_time = millis_since_boot();

  cur_camera_buf = &camera_bufs[cur_buf_idx];

  if (debayer) {
    debayer->queue(q, camrabuf_cl, cur_yuv_buf->buf_cl, rgb_width, rgb_height, &event);
  } else {
    assert(rgb_stride == camera_state->ci.frame_stride);
    rgb2yuv->queue(q, camrabuf_cl, cur_rgb_buf->buf_cl);
  }

  clWaitForEvents(1, &event);
  CL_CHECK(clReleaseEvent(event));

  cur_frame_data.processing_time = (millis_since_boot() - start_time) / 1000.0;

  VisionIpcBufExtra extra = {
                        cur_frame_data.frame_id,
                        cur_frame_data.timestamp_sof,
                        cur_frame_data.timestamp_eof,
  };
  cur_yuv_buf->set_frame_id(cur_frame_data.frame_id);
  vipc_server->send(cur_yuv_buf, &extra);

  return true;
}

void CameraBuf::release() {
  // Empty
}

void CameraBuf::queue(size_t buf_idx) {
  safe_queue.push(buf_idx);
}

// common functions

void fill_frame_data(cereal::FrameData::Builder &framed, const FrameMetadata &frame_data) {
  framed.setFrameId(frame_data.frame_id);
  framed.setTimestampEof(frame_data.timestamp_eof);
  framed.setTimestampSof(frame_data.timestamp_sof);
  framed.setFrameLength(frame_data.frame_length);
  framed.setIntegLines(frame_data.integ_lines);
  framed.setGain(frame_data.gain);
  framed.setHighConversionGain(frame_data.high_conversion_gain);
  framed.setMeasuredGreyFraction(frame_data.measured_grey_fraction);
  framed.setTargetGreyFraction(frame_data.target_grey_fraction);
  framed.setLensPos(frame_data.lens_pos);
  framed.setLensErr(frame_data.lens_err);
  framed.setLensTruePos(frame_data.lens_true_pos);
  framed.setProcessingTime(frame_data.processing_time);
}

kj::Array<uint8_t> get_frame_image(const CameraBuf *b) {
  static const int x_min = util::getenv("XMIN", 0);
  static const int y_min = util::getenv("YMIN", 0);
  static const int env_xmax = util::getenv("XMAX", -1);
  static const int env_ymax = util::getenv("YMAX", -1);
  static const int scale = util::getenv("SCALE", 1);

  assert(b->cur_rgb_buf);

  const int x_max = env_xmax != -1 ? env_xmax : b->rgb_width - 1;
  const int y_max = env_ymax != -1 ? env_ymax : b->rgb_height - 1;
  const int new_width = (x_max - x_min + 1) / scale;
  const int new_height = (y_max - y_min + 1) / scale;
  const uint8_t *dat = (const uint8_t *)b->cur_rgb_buf->addr;

  kj::Array<uint8_t> frame_image = kj::heapArray<uint8_t>(new_width*new_height*3);
  uint8_t *resized_dat = frame_image.begin();
  int goff = x_min*3 + y_min*b->rgb_stride;
  for (int r=0;r<new_height;r++) {
    for (int c=0;c<new_width;c++) {
      memcpy(&resized_dat[(r*new_width+c)*3], &dat[goff+r*b->rgb_stride*scale+c*3*scale], 3*sizeof(uint8_t));
    }
  }
  return kj::mv(frame_image);
}

kj::Array<uint8_t> get_raw_frame_image(const CameraBuf *b) {
  const uint8_t *dat = (const uint8_t *)b->cur_camera_buf->addr;

  kj::Array<uint8_t> frame_image = kj::heapArray<uint8_t>(b->cur_camera_buf->len);
  uint8_t *resized_dat = frame_image.begin();

  memcpy(resized_dat, dat, b->cur_camera_buf->len);

  return kj::mv(frame_image);
}

static kj::Array<capnp::byte> yuv420_to_jpeg(const CameraBuf *b, int thumbnail_width, int thumbnail_height) {
  int downscale = b->cur_yuv_buf->width / thumbnail_width;
  assert(downscale * thumbnail_height == b->cur_yuv_buf->height);
  int in_stride = b->cur_yuv_buf->stride;

  // make the buffer big enough. jpeg_write_raw_data requires 16-pixels aligned height to be used.
  std::unique_ptr<uint8[]> buf(new uint8_t[(thumbnail_width * ((thumbnail_height + 15) & ~15) * 3) / 2]);
  uint8_t *y_plane = buf.get();
  uint8_t *u_plane = y_plane + thumbnail_width * thumbnail_height;
  uint8_t *v_plane = u_plane + (thumbnail_width * thumbnail_height) / 4;
  {
    // subsampled conversion from nv12 to yuv
    for (int hy = 0; hy < thumbnail_height/2; hy++) {
      for (int hx = 0; hx < thumbnail_width/2; hx++) {
        int ix = hx * downscale + (downscale-1)/2;
        int iy = hy * downscale + (downscale-1)/2;
        y_plane[(hy*2 + 0)*thumbnail_width + (hx*2 + 0)] = b->cur_yuv_buf->y[(iy*2 + 0) * in_stride + ix*2 + 0];
        y_plane[(hy*2 + 0)*thumbnail_width + (hx*2 + 1)] = b->cur_yuv_buf->y[(iy*2 + 0) * in_stride + ix*2 + 1];
        y_plane[(hy*2 + 1)*thumbnail_width + (hx*2 + 0)] = b->cur_yuv_buf->y[(iy*2 + 1) * in_stride + ix*2 + 0];
        y_plane[(hy*2 + 1)*thumbnail_width + (hx*2 + 1)] = b->cur_yuv_buf->y[(iy*2 + 1) * in_stride + ix*2 + 1];
        u_plane[hy*thumbnail_width/2 + hx] = b->cur_yuv_buf->uv[iy*in_stride + ix*2 + 0];
        v_plane[hy*thumbnail_width/2 + hx] = b->cur_yuv_buf->uv[iy*in_stride + ix*2 + 1];
      }
    }
  }

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  uint8_t *thumbnail_buffer = nullptr;
  size_t thumbnail_len = 0;
  jpeg_mem_dest(&cinfo, &thumbnail_buffer, &thumbnail_len);

  cinfo.image_width = thumbnail_width;
  cinfo.image_height = thumbnail_height;
  cinfo.input_components = 3;

  jpeg_set_defaults(&cinfo);
  jpeg_set_colorspace(&cinfo, JCS_YCbCr);
  // configure sampling factors for yuv420.
  cinfo.comp_info[0].h_samp_factor = 2;  // Y
  cinfo.comp_info[0].v_samp_factor = 2;
  cinfo.comp_info[1].h_samp_factor = 1;  // U
  cinfo.comp_info[1].v_samp_factor = 1;
  cinfo.comp_info[2].h_samp_factor = 1;  // V
  cinfo.comp_info[2].v_samp_factor = 1;
  cinfo.raw_data_in = TRUE;

  jpeg_set_quality(&cinfo, 50, TRUE);
  jpeg_start_compress(&cinfo, TRUE);

  JSAMPROW y[16], u[8], v[8];
  JSAMPARRAY planes[3]{y, u, v};

  for (int line = 0; line < cinfo.image_height; line += 16) {
    for (int i = 0; i < 16; ++i) {
      y[i] = y_plane + (line + i) * cinfo.image_width;
      if (i % 2 == 0) {
        int offset = (cinfo.image_width / 2) * ((i + line) / 2);
        u[i / 2] = u_plane + offset;
        v[i / 2] = v_plane + offset;
      }
    }
    jpeg_write_raw_data(&cinfo, planes, 16);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  kj::Array<capnp::byte> dat = kj::heapArray<capnp::byte>(thumbnail_buffer, thumbnail_len);
  free(thumbnail_buffer);
  return dat;
}

static void publish_thumbnail(PubMaster *pm, const CameraBuf *b) {
  auto thumbnail = yuv420_to_jpeg(b, b->rgb_width / 4, b->rgb_height / 4);
  if (thumbnail.size() == 0) return;

  MessageBuilder msg;
  auto thumbnaild = msg.initEvent().initThumbnail();
  thumbnaild.setFrameId(b->cur_frame_data.frame_id);
  thumbnaild.setTimestampEof(b->cur_frame_data.timestamp_eof);
  thumbnaild.setThumbnail(thumbnail);

  pm->send("thumbnail", msg);
}

float set_exposure_target(const CameraBuf *b, int x_start, int x_end, int x_skip, int y_start, int y_end, int y_skip) {
  int lum_med;
  uint32_t lum_binning[256] = {0};
  const uint8_t *pix_ptr = b->cur_yuv_buf->y;

  unsigned int lum_total = 0;
  for (int y = y_start; y < y_end; y += y_skip) {
    for (int x = x_start; x < x_end; x += x_skip) {
      uint8_t lum = pix_ptr[(y * b->rgb_width) + x];
      lum_binning[lum]++;
      lum_total += 1;
    }
  }


  // Find mean lumimance value
  unsigned int lum_cur = 0;
  for (lum_med = 255; lum_med >= 0; lum_med--) {
    lum_cur += lum_binning[lum_med];

    if (lum_cur >= lum_total / 2) {
      break;
    }
  }

  return lum_med / 256.0;
}

void *processing_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback) {
  const char *thread_name = nullptr;
  if (cs == &cameras->road_cam) {
    thread_name = "RoadCamera";
  } else if (cs == &cameras->driver_cam) {
    thread_name = "DriverCamera";
  } else {
    thread_name = "WideRoadCamera";
  }
  util::set_thread_name(thread_name);

  uint32_t cnt = 0;
  while (!do_exit) {
    if (!cs->buf.acquire()) continue;

    callback(cameras, cs, cnt);

    if (cs == &(cameras->road_cam) && cameras->pm && cnt % 100 == 3) {
      // this takes 10ms???
      publish_thumbnail(cameras->pm, &(cs->buf));
    }
    cs->buf.release();
    ++cnt;
  }
  return NULL;
}

std::thread start_process_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback) {
  return std::thread(processing_thread, cameras, cs, callback);
}

void camerad_thread() {
  cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
#ifdef QCOM2
  const cl_context_properties props[] = {CL_CONTEXT_PRIORITY_HINT_QCOM, CL_PRIORITY_HINT_HIGH_QCOM, 0};
  cl_context context = CL_CHECK_ERR(clCreateContext(props, 1, &device_id, NULL, NULL, &err));
#else
  cl_context context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));
#endif

  MultiCameraState cameras = {};
  VisionIpcServer vipc_server("camerad", device_id, context);

  cameras_init(&vipc_server, &cameras, device_id, context);
  cameras_open(&cameras);

  vipc_server.start_listener();

  cameras_run(&cameras);

  CL_CHECK(clReleaseContext(context));
}

int open_v4l_by_name_and_index(const char name[], int index, int flags) {
  for (int v4l_index = 0; /**/; ++v4l_index) {
    std::string v4l_name = util::read_file(util::string_format("/sys/class/video4linux/v4l-subdev%d/name", v4l_index));
    if (v4l_name.empty()) return -1;
    if (v4l_name.find(name) == 0) {
      if (index == 0) {
        return HANDLE_EINTR(open(util::string_format("/dev/v4l-subdev%d", v4l_index).c_str(), flags));
      }
      index--;
    }
  }
}
