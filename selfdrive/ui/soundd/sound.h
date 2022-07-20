/*
 * Copyright (c) 2020-2022 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include <QMap>
#include <QSoundEffect>
#include <QString>

#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/ui.h"

const std::tuple<AudibleAlert, QString, int> sound_list[] = {
  // AudibleAlert, file name, loop count
  {AudibleAlert::ENGAGE, "engage.wav", 0},
  {AudibleAlert::DISENGAGE, "disengage.wav", 0},
  {AudibleAlert::REFUSE, "refuse.wav", 0},

  {AudibleAlert::PROMPT, "prompt.wav", 0},
  {AudibleAlert::PROMPT_REPEAT, "prompt.wav", QSoundEffect::Infinite},
  {AudibleAlert::PROMPT_DISTRACTED, "prompt_distracted.wav", QSoundEffect::Infinite},

  {AudibleAlert::WARNING_SOFT, "warning_soft.wav", QSoundEffect::Infinite},
  {AudibleAlert::WARNING_IMMEDIATE, "warning_immediate.wav", QSoundEffect::Infinite},

  {AudibleAlert::LEFT_BLINDSPOT, "LeftBlindspot.wav", 0},
  {AudibleAlert::RIGHT_BLINDSPOT, "RightBlindspot.wav", 0},
  {AudibleAlert::LEAD_CAR_GOING, "LearCarGoing.wav", 0},
  {AudibleAlert::NO_LEAD_CAR_WARNING, "NoLeadCar.wav", 0},
  {AudibleAlert::LEFT_CUT_IN, "LeftCutIn.wav", 0},
  {AudibleAlert::RIGHT_CUT_IN, "RightCutIn.wav", 0},
  {AudibleAlert::SPEED_LIMIT30_KM, "SpeedLimit30km.wav", 0},
  {AudibleAlert::SPEED_LIMIT40_KM, "SpeedLimit40km.wav", 0},
  {AudibleAlert::SPEED_LIMIT50_KM, "SpeedLimit50km.wav", 0},
  {AudibleAlert::SPEED_LIMIT60_KM, "SpeedLimit60km.wav", 0},
  {AudibleAlert::SPEED_LIMIT70_KM, "SpeedLimit70km.wav", 0},
  {AudibleAlert::SPEED_LIMIT80_KM, "SpeedLimit80km.wav", 0},
  {AudibleAlert::SPEED_LIMIT90_KM, "SpeedLimit90km.wav", 0},
  {AudibleAlert::SPEED_LIMIT100_KM, "SpeedLimit100km.wav", 0},
  {AudibleAlert::SPEED_LIMIT110_KM, "SpeedLimit110km.wav", 0},
};

class Sound : public QObject {
public:
  explicit Sound(QObject *parent = 0);

protected:
  void update();
  void setAlert(const Alert &alert);
  void playSound(AudibleAlert audibleAlert);

  Alert current_alert = {};
  QMap<AudibleAlert, QPair<QSoundEffect *, int>> sounds;
  SubMaster sm;
  uint64_t started_frame;
};
