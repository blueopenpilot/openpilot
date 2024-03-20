/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "selfdrive/ui/qt/onroad/annotated_camera.h"

#include <QPainter>
#include <algorithm>
#include <cmath>

#include "common/swaglog.h"
#include "selfdrive/ui/qt/onroad/buttons.h"
#include "selfdrive/ui/qt/util.h"

// Window that shows camera view and variety of info drawn on top
AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  experimental_btn = new ExperimentalButton(this);
  main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);

  monitor_off_btn = new MonitorOffButton(this);
  main_layout->addWidget(monitor_off_btn, 0, Qt::AlignTop | Qt::AlignRight);

  vag_hud_btn = new VagHudButton(this);
  main_layout->addWidget(vag_hud_btn, 0, Qt::AlignTop | Qt::AlignRight);
  QObject::connect(vag_hud_btn, &VagHudButton::openVagHud, this, &AnnotatedCameraWidget::openVagHud);

  vag_settings_btn = new VagSettingsButton(this);
  main_layout->addWidget(vag_settings_btn, 0, Qt::AlignTop | Qt::AlignRight);
  QObject::connect(vag_settings_btn, &VagSettingsButton::openVagSettings, this, &AnnotatedCameraWidget::openVagSettings);


  map_settings_btn = new MapSettingsButton(this);
  main_layout->addWidget(map_settings_btn, 0, Qt::AlignBottom | Qt::AlignRight);

  vag_osd = new VagOsd(this);


  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});
  //vag_hud_img = loadPixmap("../assets/images/img_hud.png", {img_size + 5, img_size + 5});
  //monitor_off_img = loadPixmap("../assets/images/img_monitor_off.png", {img_size + 5, img_size + 5});
}

void AnnotatedCameraWidget::updateState(const UIState &s) {
  const int SET_SPEED_NA = 255;
  const SubMaster &sm = *(s.sm);

  const bool cs_alive = sm.alive("controlsState");
  const bool nav_alive = sm.alive("navInstruction") && sm["navInstruction"].getValid();
  const auto cs = sm["controlsState"].getControlsState();
  const auto car_state = sm["carState"].getCarState();
  const auto nav_instruction = sm["navInstruction"].getNavInstruction();

  // Handle older routes where vCruiseCluster is not set
  float v_cruise = cs.getVCruiseCluster() == 0.0 ? cs.getVCruise() : cs.getVCruiseCluster();
  setSpeed = cs_alive ? v_cruise : SET_SPEED_NA;
  is_cruise_set = setSpeed > 0 && (int)setSpeed != SET_SPEED_NA;
  if (is_cruise_set && !s.scene.is_metric) {
    setSpeed *= KM_TO_MILE;
  }

  // Handle older routes where vEgoCluster is not set
  v_ego_cluster_seen = v_ego_cluster_seen || car_state.getVEgoCluster() != 0.0;
  float v_ego = v_ego_cluster_seen ? car_state.getVEgoCluster() : car_state.getVEgo();
  speed = cs_alive ? std::max<float>(0.0, v_ego) : 0.0;
  speed *= s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH;

  auto speed_limit_sign = nav_instruction.getSpeedLimitSign();
  speedLimit = nav_alive ? nav_instruction.getSpeedLimit() : 0.0;
  speedLimit *= (s.scene.is_metric ? MS_TO_KPH : MS_TO_MPH);

  has_us_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::MUTCD);
  has_eu_speed_limit = (nav_alive && speed_limit_sign == cereal::NavInstruction::SpeedLimitSign::VIENNA);
  is_metric = s.scene.is_metric;
  speedUnit =  s.scene.is_metric ? tr("km/h") : tr("mph");
  hideBottomIcons = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  status = s.status;

  // update engageability/experimental mode button
  experimental_btn->updateState(s);

  // update DM icon
  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  dmActive = dm_state.getIsActiveMode();
  rightHandDM = dm_state.getIsRHD();
  // DM icon transition
  dm_fade_state = std::clamp(dm_fade_state+0.2*(0.5-dmActive), 0.0, 1.0);

  // hide map settings button for alerts and flip for right hand DM
  if (map_settings_btn->isEnabled()) {
    map_settings_btn->setVisible(!hideBottomIcons);
    main_layout->setAlignment(map_settings_btn, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignBottom);
  }
}

void AnnotatedCameraWidget::drawHud(QPainter &p) {
  p.save();

  // Header gradient
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), UI_HEADER_HEIGHT, bg);

  QString speedLimitStr = (speedLimit > 1) ? QString::number(std::nearbyint(speedLimit)) : "-";
  QString speedStr = QString::number(std::nearbyint(speed));
  QString setSpeedStr = is_cruise_set ? QString::number(std::nearbyint(setSpeed)) : "-";

  // Draw outer box + border to contain set speed and speed limit
  const int sign_margin = 12;
  const int us_sign_height = 186;
  const int eu_sign_size = 176;

  const QSize default_size = {172, 204};
  QSize set_speed_size = default_size;
  if (is_metric || has_eu_speed_limit) set_speed_size.rwidth() = 200;
  if (has_us_speed_limit && speedLimitStr.size() >= 3) set_speed_size.rwidth() = 223;

  if (has_us_speed_limit) set_speed_size.rheight() += us_sign_height + sign_margin;
  else if (has_eu_speed_limit) set_speed_size.rheight() += eu_sign_size + sign_margin;

  int top_radius = 32;
  int bottom_radius = has_eu_speed_limit ? 100 : 32;

  QRect set_speed_rect(QPoint(60 + (default_size.width() - set_speed_size.width()) / 2 - 20, 25), set_speed_size);
  p.setPen(QPen(whiteColor(75), 6));
  p.setBrush(blackColor(166));
  drawRoundedRect(p, set_speed_rect, top_radius, top_radius, bottom_radius, bottom_radius);

  // Draw MAX
  QColor max_color = QColor(0x80, 0xd8, 0xa6, 0xff);
  QColor set_speed_color = whiteColor();
  if (is_cruise_set) {
    if (status == STATUS_DISENGAGED) {
      max_color = whiteColor();
    } else if (status == STATUS_OVERRIDE) {
      max_color = QColor(0x91, 0x9b, 0x95, 0xff);
    } else if (speedLimit > 0) {
      auto interp_color = [=](QColor c1, QColor c2, QColor c3) {
        return speedLimit > 0 ? interpColor(setSpeed, {speedLimit + 5, speedLimit + 15, speedLimit + 25}, {c1, c2, c3}) : c1;
      };
      max_color = interp_color(max_color, QColor(0xff, 0xe4, 0xbf), QColor(0xff, 0xbf, 0xbf));
      set_speed_color = interp_color(set_speed_color, QColor(0xff, 0x95, 0x00), QColor(0xff, 0x00, 0x00));
    }
  } else {
    max_color = QColor(0xa6, 0xa6, 0xa6, 0xff);
    set_speed_color = QColor(0x72, 0x72, 0x72, 0xff);
  }
  p.setFont(InterFont(40, QFont::DemiBold));
  p.setPen(max_color);
  p.drawText(set_speed_rect.adjusted(0, 27, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("MAX"));
  p.setFont(InterFont(90, QFont::Bold));
  p.setPen(set_speed_color);
  p.drawText(set_speed_rect.adjusted(0, 77, 0, 0), Qt::AlignTop | Qt::AlignHCenter, setSpeedStr);

  const QRect sign_rect = set_speed_rect.adjusted(sign_margin, default_size.height(), -sign_margin, -sign_margin);
  // US/Canada (MUTCD style) sign
  if (has_us_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawRoundedRect(sign_rect, 24, 24);
    p.setPen(QPen(blackColor(), 6));
    p.drawRoundedRect(sign_rect.adjusted(9, 9, -9, -9), 16, 16);

    p.setFont(InterFont(28, QFont::DemiBold));
    p.drawText(sign_rect.adjusted(0, 22, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("SPEED"));
    p.drawText(sign_rect.adjusted(0, 51, 0, 0), Qt::AlignTop | Qt::AlignHCenter, tr("LIMIT"));
    p.setFont(InterFont(70, QFont::Bold));
    p.drawText(sign_rect.adjusted(0, 85, 0, 0), Qt::AlignTop | Qt::AlignHCenter, speedLimitStr);
  }

  // EU (Vienna style) sign
  if (has_eu_speed_limit) {
    p.setPen(Qt::NoPen);
    p.setBrush(whiteColor());
    p.drawEllipse(sign_rect);
    p.setPen(QPen(Qt::red, 20));
    p.drawEllipse(sign_rect.adjusted(16, 16, -16, -16));

    p.setFont(InterFont((speedLimitStr.size() >= 3) ? 60 : 70, QFont::Bold));
    p.setPen(blackColor());
    p.drawText(sign_rect, Qt::AlignCenter, speedLimitStr);
  }

  // current speed
  p.setFont(InterFont(176, QFont::Bold));
  drawText(p, rect().center().x(), 210, speedStr);
  p.setFont(InterFont(66));
  drawText(p, rect().center().x(), 290, speedUnit, 200);

  p.restore();
}

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

#define COLOR_BLOCK_SIZE 100
void AnnotatedCameraWidget::drawOsdText(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align) {
  QFont font;
#ifdef QCOM
  font.setPointSize(font_size);
#elif QCOM2
  font.setPointSize(font_size*4);
#endif
  p.setFont(font);
  p.setPen(color);
  p.drawText(x, y, w, h, align, font_string);
}


#define COLOR_INFOBOX_BACKGROUND_PURPLE QColor(126, 78, 153, 150) //Purple
#define COLOR_INFOBOX_BACKGROUND_BLUE QColor(0x17, 0x33, 0x49, 0xc8) //Blue

#define INFO_1_X 0
#define INFO_1_Y 250
#define INFO_1_W 350
#define INFO_1_H 100
#define INFO_2_X 0
#define INFO_2_Y INFO_1_Y + INFO_1_H
#define INFO_2_W 350
#define INFO_2_H 100
#define INFO_3_X 0
#define INFO_3_Y INFO_2_Y + INFO_2_H
#define INFO_3_W 200
#define INFO_3_H 100
#define INFO_4_X 0
#define INFO_4_Y INFO_3_Y + INFO_3_H
#define INFO_4_W 350
#define INFO_4_H 100
#define INFO_5_X 0
#define INFO_5_Y INFO_4_Y + INFO_4_H
#define INFO_5_W 350
#define INFO_5_H 100
#define INFO_6_X 0
#define INFO_6_Y INFO_5_Y + INFO_5_H
#define INFO_6_W 350
#define INFO_6_H 100
#define INFO_7_X 0
#define INFO_7_Y INFO_6_Y + INFO_6_H
#define INFO_7_W 350
#define INFO_7_H 100

void AnnotatedCameraWidget::drawOsdInfobox(QPainter &p) {
  UIState *s = uiState();
  const bool isVagParamFromCerealEnabled = (*(uiState())->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagParamFromCerealEnabled();
  bool isVagDevelopOnRoadUi = false;
  if(isVagParamFromCerealEnabled) {
    isVagDevelopOnRoadUi = (*(uiState())->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagDevelopOnRoadUi();
  } else {
    try {
      isVagDevelopOnRoadUi = Params().getBool("IsVagDevelopOnRoadUi");
    } catch (std::exception &e) {
      printf("[BOP][%s][%s][%d] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
      isVagDevelopOnRoadUi = false;
    }
  }
  QColor infoboxBackgroupColor = COLOR_INFOBOX_BACKGROUND_BLUE;
  if(isVagDevelopOnRoadUi) {
    infoboxBackgroupColor = COLOR_INFOBOX_BACKGROUND_PURPLE;
  }

  //----- Steer angle/FLKA -----
  QPainterPath steeringAngleDegPath;
  steeringAngleDegPath.moveTo(INFO_1_X, INFO_1_Y);
  steeringAngleDegPath.lineTo(INFO_1_X, INFO_1_Y + INFO_1_H);
  steeringAngleDegPath.lineTo(INFO_1_W + INFO_1_H / 2, INFO_1_Y + INFO_1_H);
  steeringAngleDegPath.lineTo(INFO_1_W + INFO_1_H / 2, INFO_1_Y + INFO_1_H /2);
  steeringAngleDegPath.lineTo(INFO_1_W, INFO_1_Y);
  steeringAngleDegPath.closeSubpath();
  p.fillPath(steeringAngleDegPath, infoboxBackgroupColor);

  const float steeringAngleDeg = (*s->sm)["carState"].getCarState().getSteeringAngleDeg();
  const bool steeringPressed = (*s->sm)["carState"].getCarState().getSteeringPressed();
  const bool availableVagFlka = (*s->sm)["carControl"].getCarControl().getVagCarControl().getAvailableVagFlka();
  const bool controlsAllowed = (*s->sm)["pandaStates"].getPandaStates()[0].getControlsAllowed();
  char steeringAngleDegBuffer[100];

  if(steeringPressed) {
    snprintf(steeringAngleDegBuffer, sizeof(steeringAngleDegBuffer), " User: %1.2f", steeringAngleDeg);
    drawOsdText(p, INFO_1_X, INFO_1_Y, INFO_1_W, INFO_1_H, steeringAngleDegBuffer, 10, COLOR_YELLOW, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));
  } else if(availableVagFlka && controlsAllowed) {
    snprintf(steeringAngleDegBuffer, sizeof(steeringAngleDegBuffer), " FLKA: %1.2f", steeringAngleDeg);
    drawOsdText(p, INFO_1_X, INFO_1_Y, INFO_1_W, INFO_1_H, steeringAngleDegBuffer, 10, COLOR_GREEN, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));
  } else {
    snprintf(steeringAngleDegBuffer, sizeof(steeringAngleDegBuffer), " Steer: %1.2f", steeringAngleDeg);
    drawOsdText(p, INFO_1_X, INFO_1_Y, INFO_1_W, INFO_1_H, steeringAngleDegBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));
  }
  //----- Gas/Brake -----
  QPainterPath gasBrakePath;
  gasBrakePath.moveTo(INFO_2_X, INFO_2_Y);
  gasBrakePath.lineTo(INFO_2_X, INFO_2_Y + INFO_2_H);
  gasBrakePath.lineTo(INFO_2_W + INFO_2_H / 2, INFO_2_Y + INFO_2_H);
  gasBrakePath.lineTo(INFO_2_W + INFO_2_H / 2, INFO_2_Y + INFO_2_H / 2);
  gasBrakePath.lineTo(INFO_2_W, INFO_2_Y);
  gasBrakePath.closeSubpath();
  p.fillPath(gasBrakePath, infoboxBackgroupColor);

  const int gas = (int)((*s->sm)["carState"].getCarState().getGas() * 100);
  const int brake = (int)((*s->sm)["carState"].getCarState().getBrake() * 200);

  if(gas > 0) {
    char gasBrakeBuffer[100];
    snprintf(gasBrakeBuffer, sizeof(gasBrakeBuffer), " Gas: %d%%", gas);
    drawOsdText(p, INFO_2_X, INFO_2_Y, INFO_2_W, INFO_2_H, gasBrakeBuffer, 10, COLOR_GREEN, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));
  } else if(brake > 0) {
    char gasBrakeBuffer[100];
    snprintf(gasBrakeBuffer, sizeof(gasBrakeBuffer), " Brake: %d%%", brake);
    drawOsdText(p, INFO_2_X, INFO_2_Y, INFO_2_W, INFO_2_H, gasBrakeBuffer, 10, COLOR_RED, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));
  } else {
    char gasBrakeBuffer[100];
    snprintf(gasBrakeBuffer, sizeof(gasBrakeBuffer), " Gas/Brake:");
    drawOsdText(p, INFO_2_X, INFO_2_Y, INFO_2_W, INFO_2_H, gasBrakeBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));
  }

  //----- Voltage -----
  float voltage = 0;
  const int voltageInt = (int)(*s->sm)["peripheralState"].getPeripheralState().getVoltage();
  voltage = (float)voltageInt/1000;
  QPainterPath voltagePath;
  voltagePath.moveTo(INFO_3_X, INFO_3_Y);
  voltagePath.lineTo(INFO_3_X, INFO_3_Y + INFO_3_H);
  voltagePath.lineTo(INFO_3_W + INFO_3_H / 2, INFO_3_Y + INFO_3_H);
  voltagePath.lineTo(INFO_3_W + INFO_3_H / 2, INFO_3_Y + INFO_3_H / 2);
  voltagePath.lineTo(INFO_3_W,  INFO_3_Y);
  voltagePath.closeSubpath();
  p.fillPath(voltagePath, infoboxBackgroupColor);
  char voltageBuffer[100];
  snprintf(voltageBuffer, sizeof(voltageBuffer), " %2.1f V", voltage);
  drawOsdText(p, INFO_3_X, INFO_3_Y, INFO_3_W, INFO_3_H, voltageBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));

  //----- Gearbox Sumpf Temperature -----
  QPainterPath gearboxSumpfTemperaturePath;
  gearboxSumpfTemperaturePath.moveTo(INFO_4_X, INFO_4_Y);
  gearboxSumpfTemperaturePath.lineTo(INFO_4_X, INFO_4_Y + INFO_4_H);
  gearboxSumpfTemperaturePath.lineTo(INFO_4_W + INFO_4_H / 2, INFO_4_Y + INFO_4_H);
  gearboxSumpfTemperaturePath.lineTo(INFO_4_W + INFO_4_H / 2, INFO_4_Y + INFO_4_H / 2);
  gearboxSumpfTemperaturePath.lineTo(INFO_4_W, INFO_4_Y);
  gearboxSumpfTemperaturePath.closeSubpath();
  p.fillPath(gearboxSumpfTemperaturePath, infoboxBackgroupColor);

  const int geSumpftemperatur = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getGeSumpftemperatur();

  char gearboxSumpfTemperatureBuffer[100];
  snprintf(gearboxSumpfTemperatureBuffer, sizeof(gearboxSumpfTemperatureBuffer), " G.S.T: %3d", geSumpftemperatur);
  drawOsdText(p, INFO_4_X, INFO_4_Y, INFO_4_W, INFO_4_H, gearboxSumpfTemperatureBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));

  //----- Engine Coolant Temperature -----
  QPainterPath engineCoolantTemperaturePath;
  engineCoolantTemperaturePath.moveTo(INFO_5_X, INFO_5_Y);
  engineCoolantTemperaturePath.lineTo(INFO_5_X, INFO_5_Y + INFO_5_H);
  engineCoolantTemperaturePath.lineTo(INFO_5_W + INFO_5_H / 2, INFO_5_Y + INFO_5_H);
  engineCoolantTemperaturePath.lineTo(INFO_5_W + INFO_5_H / 2, INFO_5_Y + INFO_5_H / 2);
  engineCoolantTemperaturePath.lineTo(INFO_5_W, INFO_5_Y);
  engineCoolantTemperaturePath.closeSubpath();
  p.fillPath(engineCoolantTemperaturePath, infoboxBackgroupColor);

  const float moItmKuehlmittelTemp = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getObdEngCoolTemp();

  char engineCoolantTemperatureBuffer[100];
  snprintf(engineCoolantTemperatureBuffer, sizeof(engineCoolantTemperatureBuffer), " E.C.T: %3.0f ", moItmKuehlmittelTemp);
  drawOsdText(p, INFO_5_X, INFO_5_Y, INFO_5_W, INFO_5_H, engineCoolantTemperatureBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));

  //----- Engine Oil Temperature -----
  QPainterPath engineOilTemperaturePath;
  engineOilTemperaturePath.moveTo(INFO_6_X, INFO_6_Y);
  engineOilTemperaturePath.lineTo(INFO_6_X, INFO_6_Y + INFO_6_H);
  engineOilTemperaturePath.lineTo(INFO_6_W + INFO_6_H / 2, INFO_6_Y + INFO_6_H);
  engineOilTemperaturePath.lineTo(INFO_6_W + INFO_6_H / 2, INFO_6_Y + INFO_6_H / 2);
  engineOilTemperaturePath.lineTo(INFO_6_W, INFO_6_Y);
  engineOilTemperaturePath.closeSubpath();
  p.fillPath(engineOilTemperaturePath, infoboxBackgroupColor);

  const int moOelTemp = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoOelTemp();

  char engineOilTemperatureBuffer[100];
  snprintf(engineOilTemperatureBuffer, sizeof(engineOilTemperatureBuffer), " E.O.T: %3d ", moOelTemp);
  drawOsdText(p, INFO_6_X, INFO_6_Y, INFO_6_W, INFO_6_H, engineOilTemperatureBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));

//----- Engine InAir Temperature -----
  QPainterPath engineInAirTemperaturePath;
  engineInAirTemperaturePath.moveTo(INFO_7_X, INFO_7_Y);
  engineInAirTemperaturePath.lineTo(INFO_7_X, INFO_7_Y + INFO_7_H);
  engineInAirTemperaturePath.lineTo(INFO_7_W + INFO_7_H / 2, INFO_7_Y + INFO_7_H);
  engineInAirTemperaturePath.lineTo(INFO_7_W + INFO_7_H / 2, INFO_7_Y + INFO_7_H / 2);
  engineInAirTemperaturePath.lineTo(INFO_7_W, INFO_7_Y);
  engineInAirTemperaturePath.closeSubpath();
  p.fillPath(engineInAirTemperaturePath, infoboxBackgroupColor);

  const int moAnsaugluftTemp = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoAnsaugluftTemp();

  char engineInAirTemperatureBuffer[100];
  snprintf(engineInAirTemperatureBuffer, sizeof(engineInAirTemperatureBuffer), " E.I.T: %3d ", moAnsaugluftTemp);
  drawOsdText(p, INFO_7_X, INFO_7_Y, INFO_7_W, INFO_7_H, engineInAirTemperatureBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_VCENTER+ALIGN_LEFT));
}


void AnnotatedCameraWidget::drawOsdCircle(QPainter &p) {
  //printf("[BOP][%s][%s][%d] \n", __FILE__, __FUNCTION__, __LINE__);
  UIState *s = uiState();

  const bool isVagFulltimeLkaEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagFulltimeLkaEnabled();
  //const bool isVagFulltimeLkaEnableWithBlinker = (*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagFulltimeLkaEnableWithBlinker();
  //const bool isVagFulltimeLkaEnableWithBrake = (*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagFulltimeLkaEnableWithBrake();
  const bool available = (*s->sm)["carState"].getCarState().getCruiseState().getAvailable();
  const bool steeringPressed = (*s->sm)["carState"].getCarState().getSteeringPressed();
  //const bool controlsAllowed = (*s->sm)["pandaStates"].getPandaStates()[0].getControlsAllowed();
  const bool ignitionLine = (*s->sm)["pandaStates"].getPandaStates()[0].getIgnitionLine();
  //printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnabled);
  //printf("[BOP][%s][%s][%d] available=%d \n", __FILE__, __FUNCTION__, __LINE__, available);
  //printf("[BOP][%s][%s][%d] steeringPressed=%d \n", __FILE__, __FUNCTION__, __LINE__, steeringPressed);
  //printf("[BOP][%s][%s][%d] controlsAllowed=%d \n", __FILE__, __FUNCTION__, __LINE__, controlsAllowed);
  //printf("[BOP][%s][%s][%d] ignitionLine=%d \n", __FILE__, __FUNCTION__, __LINE__, ignitionLine);

  int x = 860;
  int y = 0;
  QPixmap steering_img = QPixmap("../assets/img_chffr_wheel.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);

  p.setPen(Qt::NoPen);
  if(isVagFulltimeLkaEnabled) {
    if(available) {
      if(!ignitionLine) {
        p.setBrush(COLOR_RED_ALPHA(0xF1));
      } else {
        if(steeringPressed) {
          p.setBrush(COLOR_YELLOW_ALPHA(0xF1));
        } else {
          p.setBrush(QColor(0x17, 0x86, 0x44, 0xF1));
        }
      }
    } else {
      p.setBrush(QColor(0x17, 0x33, 0x49, 0xC8));
    }
  } else {
    p.setBrush(COLOR_BLACK_ALPHA(0xF1));
  }

  p.drawEllipse(x - radius - 220, y + 30, radius + 10, radius + 10);
  p.setOpacity(1.0);
  p.drawPixmap(x - 400 + 18, y + 30 + 18, img_size + 10, img_size + 10, steering_img);
}

void AnnotatedCameraWidget::drawOsd(QPainter &p) {
  UIState *s = uiState();
  
  const bool isVagInfoBoxEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagInfoBoxEnabled();
  if(isVagInfoBoxEnabled) {
    drawOsdInfobox(p);
  }

  drawOsdCircle(p);

}

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, 0);
  if (sm["controlsState"].getControlsState().getExperimentalMode()) {
    // The first half of track_vertices are the points for the right side of the path
    // and the indices match the positions of accel from uiPlan
    const auto &acceleration = sm["uiPlan"].getUiPlan().getAccel();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration.size());

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      if (scene.track_vertices[i].y() < 0 || scene.track_vertices[i].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[i].y()) / height();

      // speed up: 120, slow down: 0
      float path_hue = fmax(fmin(60 + acceleration[i] * 35, 120), 0);
      // FIXME: painter.drawPolygon can be slow if hue is not rounded
      path_hue = int(path_hue * 100 + 0.5) / 100;

      float saturation = fmin(fabs(acceleration[i] * 1.5), 1);
      float lightness = util::map_val(saturation, 0.0f, 1.0f, 0.95f, 0.62f);  // lighter when grey
      float alpha = util::map_val(lin_grad_point, 0.75f / 2.f, 0.75f, 0.4f, 0.0f);  // matches previous alpha fade
      bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360., saturation, lightness, alpha));

      // Skip a point, unless next is last
      i += (i + 2) < max_len ? 1 : 0;
    }

  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }

  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  painter.restore();
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  //int offset = UI_BORDER_SIZE + btn_size / 2;
  //int x = rightHandDM ? width() - offset : offset;
  //int y = height() - offset;
  //int x = 1580;
  int x = 340;
  int y = 120;
  float opacity = dmActive ? 0.65 : 0.2;
  drawIcon(painter, QPoint(x, y), dm_img, blackColor(70), opacity);
  //drawIcon(painter, QPoint(width()-350, y), vag_hud_img, blackColor(70), 1);
  //drawIcon(painter, QPoint(width()-600, y), monitor_off_img, blackColor(70), 1);


  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                      0.545 + 0.4 * s->engaged(),
                                      0.545 - 0.285 * s->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  painter.restore();
}

void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd) {
  painter.save();

  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));

  painter.restore();
}

void AnnotatedCameraWidget::paintGL() {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();

  // draw camera frame
  {
    std::lock_guard lk(frame_lock);

    if (frames.empty()) {
      if (skip_frame_count > 0) {
        skip_frame_count--;
        qDebug() << "skipping frame, not ready";
        return;
      }
    } else {
      // skip drawing up to this many frames if we're
      // missing camera frames. this smooths out the
      // transitions from the narrow and wide cameras
      skip_frame_count = 5;
    }

    // Wide or narrow cam dependent on speed
    bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
    if (has_wide_cam) {
      float v_ego = sm["carState"].getCarState().getVEgo();
      if ((v_ego < 10) || available_streams.size() == 1) {
        wide_cam_requested = true;
      } else if (v_ego > 15) {
        wide_cam_requested = false;
      }
      wide_cam_requested = wide_cam_requested && sm["controlsState"].getControlsState().getExperimentalMode();
      // for replay of old routes, never go to widecam
      wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
    }
    CameraWidget::setStreamType(wide_cam_requested ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);

    s->scene.wide_cam = CameraWidget::getStreamType() == VISION_STREAM_WIDE_ROAD;
    if (s->scene.calibration_valid) {
      auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
      CameraWidget::updateCalibration(calib);
    } else {
      CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
    }
    CameraWidget::setFrameId(model.getFrameId());
    CameraWidget::paintGL();
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  const bool isVagParamFromCerealEnabled = (*(uiState())->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagParamFromCerealEnabled();
  bool isVagDevelopOnRoadUi = false;
  bool isVagLeadCarEnabled = false;
  if(isVagParamFromCerealEnabled) {
    isVagDevelopOnRoadUi = (*(uiState())->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagDevelopOnRoadUi();
  } else {
    try {
      isVagDevelopOnRoadUi = Params().getBool("IsVagDevelopOnRoadUi");
    } catch (std::exception &e) {
      printf("[BOP][%s][%s][%d] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
      isVagDevelopOnRoadUi = false;
    }
  }
  if(isVagParamFromCerealEnabled) {
    isVagLeadCarEnabled = (*(uiState())->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagLeadCarEnabled();
  } else {
    try {
      isVagLeadCarEnabled = Params().getBool("IsVagLeadCarEnabled");
    } catch (std::exception &e) {
      printf("[BOP][%s][%s][%d] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
      isVagLeadCarEnabled = false;
    }
  }
  if (s->scene.world_objects_visible || isVagDevelopOnRoadUi) {
    update_model(s, model, sm["uiPlan"].getUiPlan());
    drawLaneLines(painter, s);

    if (s->scene.longitudinal_control && sm.rcv_frame("radarState") > s->scene.started_frame) {
      auto radar_state = sm["radarState"].getRadarState();
      update_leads(s, radar_state, model.getPosition());
      auto lead_one = radar_state.getLeadOne();
      auto lead_two = radar_state.getLeadTwo();
      if (lead_one.getStatus()) {
        drawLead(painter, lead_one, s->scene.lead_vertices[0]);
      }
      if (lead_two.getStatus() && (std::abs(lead_one.getDRel() - lead_two.getDRel()) > 3.0)) {
        drawLead(painter, lead_two, s->scene.lead_vertices[1]);
      }
    }
  }


  if (isVagDevelopOnRoadUi) {
    drawDriverState(painter, s);
  } else {
    // DMoji
    if (!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
      update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, rightHandDM);
      drawDriverState(painter, s);
    }
  }

  drawHud(painter);
  drawOsd(painter);
  vag_osd->drawOsd(painter);

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  pm->send("uiDebug", msg);
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  ui_update_params(uiState());
  prev_draw_t = millis_since_boot();
}
