import math
import numpy as np
import time
import wave


from cereal import car, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper
from openpilot.common.retry import retry
from openpilot.common.swaglog import cloudlog

from openpilot.system import micd

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1
CONTROLS_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 30 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

AudibleAlert = car.CarControl.HUDControl.AudibleAlert


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("prompt.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("prompt.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("prompt_distracted.wav", None, MAX_VOLUME),

  AudibleAlert.warningSoft: ("warning_soft.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("warning_immediate.wav", None, MAX_VOLUME),

  #VAG
  AudibleAlert.leftBlindspot: ("LeftBlindspot.wav", 1, MAX_VOLUME),
  AudibleAlert.rightBlindspot: ("RightBlindspot.wav", 1, MAX_VOLUME),
  AudibleAlert.leftBlinker: ("LeftBlinker.wav", 1, MAX_VOLUME),
  AudibleAlert.rightBlinker: ("RightBlinker.wav", 1, MAX_VOLUME),
  AudibleAlert.leadCarGoing: ("LearCarGoing.wav", 1, MAX_VOLUME),
  AudibleAlert.noLeadCarWarning: ("NoLeadCar.wav", 1, MAX_VOLUME),
}

def check_controls_timeout_alert(sm):
  controls_missing = time.monotonic() - sm.recv_time['controlsState']

  if controls_missing > CONTROLS_TIMEOUT:
    if sm['controlsState'].enabled and (controls_missing - CONTROLS_TIMEOUT) < 10:
      return True

  return False


class Soundd:
  def __init__(self):
    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.controls_timeout_alert = False

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

    self.leftBlinkerSoundPlayed = False
    self.rightBlinkerSoundPlayed = False
    self.leftBlindspotInfoSoundPlayed = False
    self.rightBlindspotInfoSoundPlayed = False
    self.leftBlindspotWarningSoundPlayed = False
    self.rightBlindspotWarningSoundPlayed = False
    self.leadCarGoingSoundPlayed = False
    self.noLeadCarWarningSoundPlayed = False

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      wavefile = wave.open(BASEDIR + "/selfdrive/assets/sounds/" + filename, 'r')

      #assert wavefile.getnchannels() == 1
      assert wavefile.getsampwidth() == 2
      assert wavefile.getframerate() == SAMPLE_RATE

      length = wavefile.getnframes()
      self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      num_loops = sound_list[self.current_alert][1]
      sound_data = self.loaded_sounds[self.current_alert]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write

    return ret * self.current_volume

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    current_alert_played_once = self.current_alert == AudibleAlert.none or self.current_sound_frame > len(self.loaded_sounds[self.current_alert])
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
      self.current_alert = new_alert
      self.current_sound_frame = 0

  def get_audible_alert(self, sm):
    if sm.updated['controlsState']:
      new_alert = sm['controlsState'].alertSound.raw
      self.update_alert(new_alert)
    elif check_controls_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      self.controls_timeout_alert = True
    elif self.controls_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.controls_timeout_alert = False
    if sm.updated['carState']:
      self.check_vag_sound(sm)

  def check_vag_sound(self, sm):
    # ===== blinker =====
    # ----- left blinker -----
    if sm['vagParam'].vagParamGeneral.isVagLeftBlinkerSoundEnabled:
      if sm['carState'].leftBlinker:
        if not self.leftBlinkerSoundPlayed:
          self.update_alert(AudibleAlert.leftBlinker)
        self.leftBlinkerSoundPlayed = True
      else:
        self.leftBlinkerSoundPlayed = False

    # ----- right blinker -----
    if sm['vagParam'].vagParamGeneral.isVagRightBlinkerSoundEnabled:
      if sm['carState'].rightBlinker:
        if not self.rightBlinkerSoundPlayed:
          self.update_alert(AudibleAlert.rightBlinker)
        self.rightBlinkerSoundPlayed = True
      else:
        self.rightBlinkerSoundPlayed = False

    # ===== blindspot info =====
    if sm['vagParam'].vagParamFeature.isVagBlindspotEnabled and sm['vagParam'].vagParamFeature.isVagBlindspotInfoSoundEnabled:
      # ----- left blindspot warning -----
      if sm['carState'].leftBlindspot:
        if not self.leftBlindspotInfoSoundPlayed:
          self.update_alert(AudibleAlert.leftBlindspot)
        self.leftBlindspotInfoSoundPlayed = True
      else:
        self.leftBlindspotInfoSoundPlayed = False

      # ----- right blindspot info -----
      if sm['carState'].rightBlindspot:
        if not self.rightBlindspotInfoSoundPlayed:
          self.update_alert(AudibleAlert.rightBlindspot)
        self.rightBlindspotInfoSoundPlayed = True
      else:
        self.rightBlindspotInfoSoundPlayed = False

    # ===== blindspot warning =====
    if sm['vagParam'].vagParamFeature.isVagBlindspotEnabled and sm['vagParam'].vagParamFeature.isVagBlindspotWarningSoundEnabled:
      # ----- left blindspot warning -----
      if sm['carState'].vagCarState.leftBlindspotWarning:
        if not self.leftBlindspotWarningSoundPlayed:
          self.update_alert(AudibleAlert.leftBlindspot)
        self.leftBlindspotWarningSoundPlayed = True
      else:
        self.leftBlindspotWarningSoundPlayed = False

      # ----- right blindspot warning -----
      if sm['carState'].vagCarState.rightBlindspotWarning:
        if not self.rightBlindspotWarningSoundPlayed:
          self.update_alert(AudibleAlert.rightBlindspot)
        self.rightBlindspotWarningSoundPlayed = True
      else:
        self.rightBlindspotWarningSoundPlayed = False

    # ===== lead car going remind =====
    if sm['vagParam'].vagParamFeature.isVagLeadCarGoingRemindEnabled and \
        sm['vagParam'].vagParamFeature.isVagLeadCarGoingRemindSoundEnabled:
      if sm['vagControl'].leadCarGoingTrigged:
        if not self.leadCarGoingSoundPlayed:
          self.update_alert(AudibleAlert.leadCarGoing)
          self.leadCarGoingSoundPlayed = True
      else:
        self.leadCarGoingSoundPlayed = False

    # ===== no lead car =====
    if sm['vagParam'].vagParamFeature.isVagNoLeadCarEnabled and \
        sm['vagParam'].vagParamFeature.isVagNoLeadCarWarningSoundEnabled and \
        (not sm['vagParam'].vagParamOp.experimentalLongitudinalEnabled) and \
        (not sm['vagParam'].vagParamOp.experimentalMode):
      accEnable = sm['carState'].cruiseState.enabled
      accAbstandsindex = sm["carState"].vagCarState.vagUiField.accAbstandsindex
      if accEnable and (accAbstandsindex == 0):
        if not self.noLeadCarWarningSoundPlayed:
          self.update_alert(AudibleAlert.noLeadCarWarning)
          self.noLeadCarWarningSoundPlayed = True
      else:
        self.noLeadCarWarningSoundPlayed = False

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(10, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=7, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd

    sm = messaging.SubMaster(['controlsState', 'microphone', 'carState', 'vagParam', 'vagControl'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)

        if sm.updated['microphone'] and self.current_alert == AudibleAlert.none: # only update volume filter when not playing alert
          self.spl_filter_weighted.update(sm["microphone"].soundPressureWeightedDb)
          if sm['vagParam'].vagParamSetting.isVagManualSoundVolumeEnable:
            self.current_volume = float(sm['vagParam'].vagParamSetting.vagSoundVolume/10)
          else:
            self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x))*float(sm['vagParam'].vagParamSetting.vagSoundVolume/10)

        self.get_audible_alert(sm)

        rk.keep_time()

        assert stream.active


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
