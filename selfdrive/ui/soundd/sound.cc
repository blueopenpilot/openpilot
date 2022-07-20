/*
 * Copyright (c) 2020-2022 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "selfdrive/ui/soundd/sound.h"

#include <cmath>

#include <QAudio>
#include <QAudioDeviceInfo>
#include <QDebug>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/util.h"

// TODO: detect when we can't play sounds
// TODO: detect when we can't display the UI

//----- Blindspot -----
static bool LeftBlindspotWarningSoundPlayed = false;
static bool RightBlindspotWarningSoundPlayed = false;
//----- Lead car going -----
static bool LeadCarGoingRemindSoundPlayed = false;
static bool FirstSetMaunalSoundVolume = true;
static int PreviousAccAbstandsindex = -1;
static int LeadCarGoingCount = 0;
//----- No lead car -----
static bool NoLeadCarWarningSoundPlayed = false;
//----- Speed Camera -----
#if 0
static bool SpeedLimitWarningSoundPlayed = false;
#endif
//----- Manual sound volume -----
static int MaunalSoundVolume = 10;



#if 0
Sound::Sound(QObject *parent) : sm({"carState", "controlsState", "deviceState", "driverMonitoringState", "speedCamera"}) {
#else
Sound::Sound(QObject *parent) : sm({"carState", "controlsState", "deviceState", "driverMonitoringState"}) {
#endif
  qInfo() << "default audio device: " << QAudioDeviceInfo::defaultOutputDevice().deviceName();

  for (auto &[alert, fn, loops] : sound_list) {
    QSoundEffect *s = new QSoundEffect(this);
    QObject::connect(s, &QSoundEffect::statusChanged, [=]() {
      assert(s->status() != QSoundEffect::Error);
    });
    s->setVolume(Hardware::MIN_VOLUME);
    s->setSource(QUrl::fromLocalFile("../../assets/sounds/" + fn));
    sounds[alert] = {s, loops};
  }

  QTimer *timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &Sound::update);
  timer->start(1000 / UI_FREQ);
};

void Sound::update() {
  const bool started_prev = sm["deviceState"].getDeviceState().getStarted();
  sm.update(0);

  const bool started = sm["deviceState"].getDeviceState().getStarted();
  if (started && !started_prev) {
    started_frame = sm.frame;
  }

  // no sounds while offroad
  // also no sounds if nothing is alive in case thermald crashes while offroad
  const bool crashed = (sm.frame - std::max(sm.rcv_frame("deviceState"), sm.rcv_frame("controlsState"))) > 10*UI_FREQ;
  if (!started || crashed) {
    setAlert({});
    return;
  }

  // scale volume with speed
  if (sm.updated("carState")) {
    float volume = util::map_val(sm["carState"].getCarState().getVEgo(), 11.f, 20.f, 0.f, 1.0f);
    volume = QAudio::convertVolume(volume, QAudio::LogarithmicVolumeScale, QAudio::LinearVolumeScale);
    volume = util::map_val(volume, 0.f, 1.f, Hardware::MIN_VOLUME, Hardware::MAX_VOLUME);
    for (auto &[s, loops] : sounds) {
      //Pon Manual sound volume
      bool IsVagManualSoundVolumeEnable = false;
      try {
        IsVagManualSoundVolumeEnable = Params().getBool("IsVagManualSoundVolumeEnable");
      } catch (std::exception &e) {
        printf("[BOP][%s][%d][%s()][IsVagManualSoundVolumeEnable] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
        IsVagManualSoundVolumeEnable = false;
      }
      if(IsVagManualSoundVolumeEnable) {
        s->setVolume(MaunalSoundVolume*0.1);
      } else {
        s->setVolume((std::round(100 * volume) / 100)*(MaunalSoundVolume*0.1));
      }
    }
  }

  setAlert(Alert::get(sm, started_frame));

  //----- Blindspot -----
  bool IsVagBlindspotEnabled = false;
  try {
    IsVagBlindspotEnabled = Params().getBool("IsVagBlindspotEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagBlindspotEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagBlindspotEnabled = false;
  }
  bool IsVagBlindspotWarningSoundEnabled = false;
  try {
    IsVagBlindspotWarningSoundEnabled = Params().getBool("IsVagBlindspotWarningSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagBlindspotWarningSoundEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagBlindspotWarningSoundEnabled = false;
  }
  if(IsVagBlindspotEnabled && IsVagBlindspotWarningSoundEnabled) {
    if (sm.updated("carState")) {
      const bool leftBlindspotWarning = sm["carState"].getCarState().getLeftBlindspotWarning();
      const bool rightBlindspotWarning = sm["carState"].getCarState().getRightBlindspotWarning();
      if (leftBlindspotWarning) {
        if(!LeftBlindspotWarningSoundPlayed) {
          playSound(AudibleAlert::LEFT_BLINDSPOT);
        }
        LeftBlindspotWarningSoundPlayed = true;
      } else {
        LeftBlindspotWarningSoundPlayed = false;
      }
      if (rightBlindspotWarning) {
        if(!RightBlindspotWarningSoundPlayed) {
          playSound(AudibleAlert::RIGHT_BLINDSPOT);
        }
        RightBlindspotWarningSoundPlayed = true;
      } else {
        RightBlindspotWarningSoundPlayed = false;
      }
    }
  }

  //----- Lead car going -----
  bool IsVagLeadCarGoingRemindEnabled = false;
  try {
    IsVagLeadCarGoingRemindEnabled = Params().getBool("IsVagLeadCarGoingRemindEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagLeadCarGoingRemindEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagLeadCarGoingRemindEnabled = false;
  }
  bool IsVagLeadCarGoingRemindSoundEnabled = false;
  try {
    IsVagLeadCarGoingRemindSoundEnabled = Params().getBool("IsVagLeadCarGoingRemindSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagLeadCarGoingRemindSoundEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagLeadCarGoingRemindSoundEnabled = false;
  }
  if(IsVagLeadCarGoingRemindEnabled && IsVagLeadCarGoingRemindSoundEnabled) {
    if(sm.updated("carState")) {
      const bool accEnable = (bool) sm["carState"].getCarState().getCruiseState().getEnabled();
      const bool accAvailable = (bool) sm["carState"].getCarState().getCruiseState().getAvailable();
      const int accAbstandsindex = (int) sm["carState"].getCarState().getVagAcc().getAccAbstandsindex();
      const int vEgo = (int) sm["carState"].getCarState().getVEgo();
      const bool gasPressed = (bool) sm["carState"].getCarState().getGasPressed();
      const int gearShifter = (int) sm["carState"].getCarState().getGearShifter();

      if(vEgo != 0) {
        LeadCarGoingCount = 0;
      }

      if(accAbstandsindex == PreviousAccAbstandsindex + 1) {
        LeadCarGoingCount++;
      } else if (accAbstandsindex == PreviousAccAbstandsindex) {
      } else {
        LeadCarGoingCount = 0;
      }

      if(accAvailable && \
         !accEnable && \
         !gasPressed && \
         vEgo == 0 && \
         LeadCarGoingCount > 2  && \
         (gearShifter == 2 || gearShifter == 5 || gearShifter == 8 || gearShifter == 9) && \
         accAbstandsindex > 6) {
        if(!LeadCarGoingRemindSoundPlayed) {
          playSound(AudibleAlert::LEAD_CAR_GOING);
        }
        LeadCarGoingRemindSoundPlayed = true;
      } else {
        LeadCarGoingRemindSoundPlayed = false;
      }
      PreviousAccAbstandsindex = accAbstandsindex;
    }
  }

  //----- No lead car -----
  bool IsVagNoLeadCarEnabled = false;
  try {
    IsVagNoLeadCarEnabled = Params().getBool("IsVagNoLeadCarEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagNoLeadCarEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagNoLeadCarEnabled = false;
  }
  bool IsVagNoLeadCarWarningSoundEnabled = false;
  try {
    IsVagNoLeadCarWarningSoundEnabled = Params().getBool("IsVagNoLeadCarWarningSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagNoLeadCarWarningSoundEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagNoLeadCarWarningSoundEnabled = false;
  }
  if(IsVagNoLeadCarEnabled && IsVagNoLeadCarWarningSoundEnabled) {
    if (sm.updated("carState")) {
      const bool accEnable = (bool) sm["carState"].getCarState().getCruiseState().getEnabled();
      const int accAbstandsindex = (int) sm["carState"].getCarState().getVagAcc().getAccAbstandsindex();
      //const bool steeringPressed = (bool) sm["carState"].getCarState().getSteeringPressed();
      //const bool gasPressed = (bool) sm["carState"].getCarState().getGasPressed();
      //const bool isDmActive = (bool) sm["driverMonitoringState"].getDriverMonitoringState().getIsActiveMode()
      if (accEnable && accAbstandsindex == 0) {
        if(!NoLeadCarWarningSoundPlayed) {
          playSound(AudibleAlert::NO_LEAD_CAR_WARNING);
        }
        NoLeadCarWarningSoundPlayed = true;
      } else {
        NoLeadCarWarningSoundPlayed = false;
      }
    }
  }

#if 0
  //----- Speed Camera -----
  bool IsVagSpeedCameraEnabled = false;
  try {
    IsVagSpeedCameraEnabled = Params().getBool("IsVagSpeedCameraEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagSpeedCameraEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagSpeedCameraEnabled = false;
  }
  bool IsVagSpeedLimitSoundEnabled = false;
  try {
    IsVagSpeedLimitSoundEnabled = Params().getBool("IsVagSpeedLimitSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagSpeedLimitSoundEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagSpeedLimitSoundEnabled = false;
  }
  if(IsVagSpeedCameraEnabled && IsVagSpeedLimitSoundEnabled) {
    if (sm.updated("speedCamera")) {
      const bool speedCameraDetected = sm["speedCamera"].getSpeedCamera().getSpeedCameraDetected();
      const float vehicleDistance = sm["speedCamera"].getSpeedCamera().getSpeedCameraMapPosition().getVehicleDistance();
      const float speedLimitation = sm["speedCamera"].getSpeedCamera().getSpeedCameraMapPosition().getSpeedLimitation();
      if(speedCameraDetected && vehicleDistance < 0.5) {
        if(!SpeedLimitWarningSoundPlayed) {
          if(speedLimitation == 30) {
            playSound(AudibleAlert::SPEED_LIMIT30_KM);
          } else if(speedLimitation == 40) {
            playSound(AudibleAlert::SPEED_LIMIT40_KM);
          } else if(speedLimitation == 50) {
            playSound(AudibleAlert::SPEED_LIMIT50_KM);
          } else if(speedLimitation == 60) {
            playSound(AudibleAlert::SPEED_LIMIT60_KM);
          } else if(speedLimitation == 70) {
            playSound(AudibleAlert::SPEED_LIMIT70_KM);
          } else if(speedLimitation == 80) {
            playSound(AudibleAlert::SPEED_LIMIT80_KM);
          } else if(speedLimitation == 90) {
            playSound(AudibleAlert::SPEED_LIMIT90_KM);
          } else if(speedLimitation == 100) {
            playSound(AudibleAlert::SPEED_LIMIT100_KM);
          } else if(speedLimitation == 110) {
            playSound(AudibleAlert::SPEED_LIMIT110_KM);
          }
          SpeedLimitWarningSoundPlayed = true;
        }
      } else {
        SpeedLimitWarningSoundPlayed = false;
      }
    }
  }
#endif

  //Pon Manual sound volume
  int VagSoundVolume = 10;
  try {
    VagSoundVolume = std::stoi(Params().get("VagSoundVolume"));
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][VagSoundVolume] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    VagSoundVolume = 10;
  }
  if(MaunalSoundVolume!=VagSoundVolume) {
    MaunalSoundVolume = VagSoundVolume;
    if(!FirstSetMaunalSoundVolume) {
      playSound(AudibleAlert::PROMPT);
    }
    FirstSetMaunalSoundVolume = false;
  }
}

void Sound::setAlert(const Alert &alert) {
  if (!current_alert.equal(alert)) {
    current_alert = alert;
    // stop sounds
    for (auto &[s, loops] : sounds) {
      // Only stop repeating sounds
      if (s->loopsRemaining() > 1 || s->loopsRemaining() == QSoundEffect::Infinite) {
        s->stop();
      }
    }

    // play sound
    if (alert.sound != AudibleAlert::NONE) {
      auto &[s, loops] = sounds[alert.sound];
      s->setLoopCount(loops);
      s->play();
    }
  }
}

//Pon play warning sound
void Sound::playSound(AudibleAlert audibleAlert) {
  // stop sounds
  for (auto &[s, loops] : sounds) {
    // Only stop repeating sounds
    if (s->loopsRemaining() == QSoundEffect::Infinite) {
      s->stop();
    }
  }

  // play sound
  if (audibleAlert != AudibleAlert::NONE) {
    auto &[s, loops] = sounds[audibleAlert];
    s->setLoopCount(loops);
    s->play();
  }
}