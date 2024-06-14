/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#pragma once

#include <QTimer>
#include <QWidget>

#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "cereal/gen/cpp/custom.capnp.h"
#include "cereal/messaging/messaging.h"

class VagParam : public QObject{
  Q_OBJECT

public:
  explicit VagParam(QObject* parent = nullptr);
  void update();
  void printInfo();
  static VagParam* getInstance();

private:
  PubMaster* pm;
  QTimer* mFirstSyncTimer;
  QTimer* mUpdateTimer;
  // ===== OP toggle =====
  bool mExperimentalLongitudinalEnabled;
  bool mExperimentalMode;

  // ===== General =====
  bool mIsVagDisableDriverMonitorAlert;
  bool mIsVagLeftBlinkerSoundEnabled;
  bool mIsVagRightBlinkerSoundEnabled;
  bool mIsVagDevelopOnRoadUi;
  bool mIsVagPandaJungleEnabled;
  bool mIsVagDevelopModeEnabled;
  bool mIsVagRunningProcessLogEnabled;
  bool mIsVagParamFromCerealEnabled;

  // ===== OSD =====
  bool mIsVagDebugBlinkerTest;
  bool mIsVagDebugBlindspotInfoTest;
  bool mIsVagDebugBlindspotWarningTest;
  bool mIsVagDebugBrakeLightTest;
  bool mIsVagDebugLeadCarGoingRemindTest;
  bool mIsVagDebugNoLeadCarWarningTest;

  // ===== Test =====
  bool mIsVagDebugOsdTestTextEnabled;
  bool mIsVagDebugItem1Enabled;
  bool mIsVagDebugItem2Enabled;
  bool mIsVagDebugItem3Enabled;
  bool mIsVagDebugItem4Enabled;
  bool mIsVagDebugItem5Enabled;

  // ===== Setting =====
  bool mIsVagManualSoundVolumeEnable;
  int mVagSoundVolume;
  bool mIsVagManualOsdBacklightEnable;
  int mVagOsdBacklight;
  bool mIsVagInfoBoxEnabled;
  bool mIsVagBlinkerEnabled;
  bool mIsVagBrakeLightEnabled;
  bool mIsVagLeadCarEnabled;

  // ===== Feature =====
  // ----- Blindspot -----
  bool mIsVagBlindspotEnabled;
  bool mIsVagBlindspotInfoSoundEnabled;
  bool mIsVagBlindspotInfoVibratorEnabled;
  bool mIsVagBlindspotWarningSoundEnabled;
  bool mIsVagBlindspotWarningVibratorEnabled;
  bool mIsVagBlindspotVibratorWithFlka;
  // ----- FLKA -----
  bool mIsVagFulltimeLkaEnabled;
  bool mIsVagFulltimeLkaEnableWithBlinker;
  bool mIsVagFulltimeLkaEnableWithBrake;
  bool mIsVagFulltimeLkaEnableWithAssistant;
  // ----- Lead car going -----
  bool mIsVagLeadCarGoingRemindEnabled;
  bool mIsVagLeadCarGoingRemindSoundEnabled;
  // ----- No lead car -----
  bool mIsVagNoLeadCarEnabled;
  bool mIsVagNoLeadCarWarningSoundEnabled;
  // ----- Force disable startstop -----
  bool mIsVagForceDisableStartstop;
  // ------ Dynamic DCC -----
  bool mIsVagDynamicDccEnabled;
  // ----- Driving Mode -----
  bool mIsVagDrivingModeEnabled;
  cereal::VagCarControl::VagDrivingMode mVagDrivingMode;

  // ===== Warning =====
  bool mIsVagWarningEngineTurboPressure;
  bool mIsVagWarningEngineOilPressure;
  bool mIsVagWarningEngineInAirTemperature;
  bool mIsVagWarningEngineCoolantTemperature;
  bool mIsVagWarningEngineOilTemperature;
  bool mIsVagWarningCoolantTemperature;
  bool mIsVagWarningInAirPressure;
  bool mIsVagWarningGearOilTemperature;
  bool mIsVagWarningBrakePressure;
  bool mIsVagWarningIndoorTemperature;
  bool mIsVagWarningOutdoorTemperature;




  // ##### param sync function #####
  void readFromFileParam();
  void writeToCeralParam();

public slots:
  // ##### param sync function #####
  void firstSync();

  // ===== OP toggle =====
  void setExperimentalLongitudinalEnabled(bool experimentalLongitudinalEnabled);
  void setExperimentalMode(bool experimentalMode);

  // ===== General =====
  void setIsVagDisableDriverMonitorAlert(bool isVagDisableDriverMonitorAlert);
  void setIsVagLeftBlinkerSoundEnabled(bool isVagLeftBlinkerSoundEnabled);
  void setIsVagRightBlinkerSoundEnabled(bool isVagRightBlinkerSoundEnabled);
  void setIsVagDevelopOnRoadUi(bool isVagDevelopOnRoadUi);
  void setIsVagPandaJungleEnabled(bool isVagPandaJungleEnabled);
  void setIsVagDevelopModeEnabled(bool isVagDevelopModeEnabled);
  void setIsVagRunningProcessLogEnabled(bool isVagRunningProcessLogEnabled);
  void setIsVagParamFromCerealEnabled(bool isVagParamFromCerealEnabled);

  // ===== OSD =====
  void setIsVagDebugBlinkerTest(bool isVagDebugBlinkerTest);
  void setIsVagDebugBlindspotInfoTest(bool isVagDebugBlindspotInfoTest);
  void setIsVagDebugBlindspotWarningTest(bool isVagDebugBlindspotWarningTest);
  void setIsVagDebugBrakeLightTest(bool isVagDebugBrakeLightTest);
  void setIsVagDebugLeadCarGoingRemindTest(bool isVagDebugLeadCarGoingRemindTest);
  void setIsVagDebugNoLeadCarWarningTest(bool isVagDebugNoLeadCarWarningTest);

  // ===== Test =====
  void setIsVagDebugOsdTestTextEnabled(bool isVagDebugOsdTestTextEnabled);
  void setIsVagDebugItem1Enabled(bool isVagDebugItem1Enabled);
  void setIsVagDebugItem2Enabled(bool isVagDebugItem2Enabled);
  void setIsVagDebugItem3Enabled(bool isVagDebugItem3Enabled);
  void setIsVagDebugItem4Enabled(bool isVagDebugItem4Enabled);
  void setIsVagDebugItem5Enabled(bool isVagDebugItem5Enabled);

  // ===== Setting =====
  void setIsVagManualSoundVolumeEnable(bool isVagManualSoundVolumeEnable);
  void setVagSoundVolume(int vagSoundVolume);
  void setIsVagManualOsdBacklightEnable(bool isVagManualOsdBacklightEnable);
  void setVagOsdBacklight(int vagOsdBacklight);
  void setIsVagInfoBoxEnabled(bool isVagInfoBoxEnabled);
  void setIsVagBlinkerEnabled(bool isVagBlinkerEnabled);
  void setIsVagBrakeLightEnabled(bool isVagBrakeLightEnabled);
  void setIsVagLeadCarEnabled(bool isVagLeadCarEnabled);

  // ===== Feature =====
  // ----- Blindspot -----
  void setIsVagBlindspotEnabled(bool isVagBlindspotEnabled);
  void setIsVagBlindspotInfoSoundEnabled(bool isVagBlindspotInfoSoundEnabled);
  void setIsVagBlindspotInfoVibratorEnabled(bool isVagBlindspotInfoVibratorEnabled);
  void setIsVagBlindspotWarningSoundEnabled(bool isVagBlindspotWarningSoundEnabled);
  void setIsVagBlindspotWarningVibratorEnabled(bool isVagBlindspotWarningVibratorEnabled);
  void setIsVagBlindspotVibratorWithFlka(bool isVagBlindspotVibratorWithFlka);
  // ----- FLKA -----
  void setIsVagFulltimeLkaEnabled(bool isVagFulltimeLkaEnabled);
  void setIsVagFulltimeLkaEnableWithBlinker(bool isVagFulltimeLkaEnableWithBlinker);
  void setIsVagFulltimeLkaEnableWithBrake(bool isVagFulltimeLkaEnableWithBrake);
  void setIsVagFulltimeLkaEnableWithAssistant(bool isVagFulltimeLkaEnableWithAssistant);
  // ----- Lead car going -----
  void setIsVagLeadCarGoingRemindEnabled(bool isVagLeadCarGoingRemindEnabled);
  void setIsVagLeadCarGoingRemindSoundEnabled(bool isVagLeadCarGoingRemindSoundEnabled);
  // ----- No lead car -----
  void setIsVagNoLeadCarEnabled(bool isVagNoLeadCarEnabled);
  void setIsVagNoLeadCarWarningSoundEnabled(bool isVagNoLeadCarWarningSoundEnabled);
  // ----- Force disable startstop -----
  void setIsVagForceDisableStartstop(bool isVagForceDisableStartstop);
  // ------ Dynamic DCC -----
  void setIsVagDynamicDccEnabled(bool isVagDynamicDccEnabled);
  // ----- Driving Mode -----
  void setIsVagDrivingModeEnabled(bool isVagDrivingModeEnabled);
  void setVagDrivingMode(int vagDrivingMode);

  // ===== Warning =====
  void setIsVagWarningEngineTurboPressure(bool isVagWarningEngineTurboPressure);
  void setIsVagWarningEngineOilPressure(bool isVagWarningEngineOilPressure);
  void setIsVagWarningEngineInAirTemperature(bool isVagWarningEngineInAirTemperature);
  void setIsVagWarningEngineCoolantTemperature(bool isVagWarningEngineCoolantTemperature);
  void setIsVagWarningEngineOilTemperature(bool isVagWarningEngineOilTemperature);
  void setIsVagWarningCoolantTemperature(bool isVagWarningCoolantTemperature);
  void setIsVagWarningInAirPressure(bool isVagWarningInAirPressure);
  void setIsVagWarningGearOilTemperature(bool isVagWarningGearOilTemperature);
  void setIsVagWarningBrakePressure(bool isVagWarningBrakePressure);
  void setIsVagWarningIndoorTemperature(bool isVagWarningIndoorTemperature);
  void setIsVagWarningOutdoorTemperature(bool isVagWarningOutdoorTemperature);

signals:

};
