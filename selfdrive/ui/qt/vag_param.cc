/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "common/params.h"
#include "selfdrive/ui/qt/vag_param.h"
#include "selfdrive/ui/ui.h"

static VagParam* gVagParam=NULL;

VagParam::VagParam(QObject* parent) {
  pm = new PubMaster({"vagParam"});

  mFirstSyncTimer = new QTimer(this);
  QObject::connect(mFirstSyncTimer, &QTimer::timeout, this, &VagParam::firstSync);
  mFirstSyncTimer->start(1000);

  mUpdateTimer = new QTimer(this);
  QObject::connect(mUpdateTimer, &QTimer::timeout, this, &VagParam::update);
  mUpdateTimer->start(50);
}

VagParam* VagParam::getInstance() {
  if (gVagParam==NULL) {
    gVagParam = new VagParam(NULL);
  }
  return gVagParam;
}

void VagParam::firstSync() {
  UIState *s = uiState();
  bool isVagParamFromCerealEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagParamFromCerealEnabled();
  if(!isVagParamFromCerealEnabled || !mIsVagParamFromCerealEnabled) {
    readFromFileParam();
    printf("[BOP][%s][%s][%d] writeToCeralParam \n", __FILE__, __FUNCTION__, __LINE__);
    writeToCeralParam();
    isVagParamFromCerealEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagParamFromCerealEnabled();
  }
  if(isVagParamFromCerealEnabled && mIsVagParamFromCerealEnabled) {
    printf("[BOP][%s][%s][%d] isVagParamFromCerealEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagParamFromCerealEnabled);
    printf("[BOP][%s][%s][%d] mIsVagParamFromCerealEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, mIsVagParamFromCerealEnabled);
    printInfo();
    mFirstSyncTimer->stop();
  }
}

void VagParam::update() {
  //readFromFileParam();
  writeToCeralParam();
}

void VagParam::printInfo() {
  UIState *s = uiState();
  // ===== OP toggle =====
  bool experimentalLongitudinalEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOp().getExperimentalLongitudinalEnabled();
  bool experimentalMode = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOp().getExperimentalMode();

  // ===== General =====
  bool isVagDisableDriverMonitorAlert = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagDisableDriverMonitorAlert();
  bool isVagLeftBlinkerSoundEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagLeftBlinkerSoundEnabled();
  bool isVagRightBlinkerSoundEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagRightBlinkerSoundEnabled();
  bool isVagDevelopOnRoadUi = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagDevelopOnRoadUi();
  bool isVagPandaJungleEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagPandaJungleEnabled();
  bool isVagDevelopModeEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagDevelopModeEnabled();
  bool isVagRunningProcessLogEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagRunningProcessLogEnabled();
  bool isVagParamFromCerealEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagParamFromCerealEnabled();

  // ===== OSD =====
  bool isVagDebugBlinkerTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBlinkerTest();
  bool isVagDebugBlindspotInfoTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBlindspotInfoTest();
  bool isVagDebugBlindspotWarningTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBlindspotWarningTest();
  bool isVagDebugBrakeLightTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBrakeLightTest();
  bool isVagDebugLeadCarGoingRemindTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugLeadCarGoingRemindTest();
  bool isVagDebugNoLeadCarWarningTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugNoLeadCarWarningTest();

  // ===== Test =====
  bool isVagDebugOsdTestTextEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamTest().getIsVagDebugOsdTestTextEnabled();
  bool isVagDebugItem1Enabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamTest().getIsVagDebugItem1Enabled();
  bool isVagDebugItem2Enabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamTest().getIsVagDebugItem2Enabled();
  bool isVagDebugItem3Enabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamTest().getIsVagDebugItem3Enabled();
  bool isVagDebugItem4Enabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamTest().getIsVagDebugItem4Enabled();
  bool isVagDebugItem5Enabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamTest().getIsVagDebugItem5Enabled();

  // ===== Setting =====
  bool isVagManualSoundVolumeEnable = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagManualSoundVolumeEnable();
  int vagSoundVolume = (int)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getVagSoundVolume();
  bool isVagManualOsdBacklightEnable = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagManualOsdBacklightEnable();
  int vagOsdBacklight = (int)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getVagOsdBacklight();
  bool isVagInfoBoxEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagInfoBoxEnabled();
  bool isVagBlinkerEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagBlinkerEnabled();
  bool isVagBrakeLightEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagBrakeLightEnabled();
  bool isVagLeadCarEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagLeadCarEnabled();

  // ===== Feature =====
  // ----- Blindspot -----
  bool isVagBlindspotEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagBlindspotEnabled();
  bool isVagBlindspotInfoSoundEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagBlindspotInfoSoundEnabled();
  bool isVagBlindspotInfoVibratorEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagBlindspotInfoVibratorEnabled();
  bool isVagBlindspotWarningSoundEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagBlindspotWarningSoundEnabled();
  bool isVagBlindspotWarningVibratorEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagBlindspotWarningVibratorEnabled();
  bool isVagBlindspotVibratorWithFlka = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagBlindspotVibratorWithFlka();
  // ----- FLKA -----
  bool isVagFulltimeLkaEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagFulltimeLkaEnabled();
  bool isVagFulltimeLkaEnableWithBlinker = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagFulltimeLkaEnableWithBlinker();
  bool isVagFulltimeLkaEnableWithBrake = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagFulltimeLkaEnableWithBrake();
  bool isVagFulltimeLkaEnableWithAssistant = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagFulltimeLkaEnableWithAssistant();
  // ----- Lead car going -----
  bool isVagLeadCarGoingRemindEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagLeadCarGoingRemindEnabled();
  bool isVagLeadCarGoingRemindSoundEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagLeadCarGoingRemindSoundEnabled();
  // ----- No lead car -----
  bool isVagNoLeadCarEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagNoLeadCarEnabled();
  bool isVagNoLeadCarWarningSoundEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagNoLeadCarWarningSoundEnabled();
  // ----- Force disable startstop -----
  bool isVagForceDisableStartstop = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagForceDisableStartstop();
  // ----- Driving Mode -----
  bool isVagDrivingModeEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagDrivingModeEnabled();
  cereal::VagCarControl::VagDrivingMode vagDrivingMode = (cereal::VagCarControl::VagDrivingMode)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getVagDrivingMode();
  // ------ Dynamic DCC -----
  bool isVagDynamicDccEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagDynamicDccEnabled();

  // ===== Warning =====
  bool isVagWarningEngineTurboPressure = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningEngineTurboPressure();
  bool isVagWarningEngineOilPressure = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningEngineOilPressure();
  bool isVagWarningEngineInAirTemperature = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningEngineInAirTemperature();
  bool isVagWarningEngineCoolantTemperature = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningEngineCoolantTemperature();
  bool isVagWarningEngineOilTemperature = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningEngineOilTemperature();
  bool isVagWarningCoolantTemperature = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningCoolantTemperature();
  bool isVagWarningInAirPressure = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningInAirPressure();
  bool isVagWarningGearOilTemperature = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningGearOilTemperature();
  bool isVagWarningBrakePressure = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningBrakePressure();
  bool isVagWarningIndoorTemperature = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningIndoorTemperature();
  bool isVagWarningOutdoorTemperature = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamWarning().getIsVagWarningOutdoorTemperature();

  // ===== OP toggle =====
  printf("[BOP][%s][%s][%d] experimentalLongitudinalEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, experimentalLongitudinalEnabled, mExperimentalLongitudinalEnabled);
  printf("[BOP][%s][%s][%d] experimentalMode[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, experimentalMode, mExperimentalMode);

  // ===== General =====
  printf("[BOP][%s][%s][%d] isVagDisableDriverMonitorAlert[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDisableDriverMonitorAlert, mIsVagDisableDriverMonitorAlert);
  printf("[BOP][%s][%s][%d] isVagLeftBlinkerSoundEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagLeftBlinkerSoundEnabled, mIsVagLeftBlinkerSoundEnabled);
  printf("[BOP][%s][%s][%d] isVagRightBlinkerSoundEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagRightBlinkerSoundEnabled, mIsVagRightBlinkerSoundEnabled);
  printf("[BOP][%s][%s][%d] isVagDevelopOnRoadUi[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDevelopOnRoadUi, mIsVagDevelopOnRoadUi);
  printf("[BOP][%s][%s][%d] isVagPandaJungleEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagPandaJungleEnabled, mIsVagPandaJungleEnabled);
  printf("[BOP][%s][%s][%d] isVagDevelopModeEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDevelopModeEnabled, mIsVagDevelopModeEnabled);
  printf("[BOP][%s][%s][%d] isVagRunningProcessLogEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagRunningProcessLogEnabled, mIsVagRunningProcessLogEnabled);
  printf("[BOP][%s][%s][%d] isVagParamFromCerealEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagParamFromCerealEnabled, mIsVagParamFromCerealEnabled);

  // ===== OSD =====
  printf("[BOP][%s][%s][%d] isVagDebugBlinkerTest[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBlinkerTest, mIsVagDebugBlinkerTest);
  printf("[BOP][%s][%s][%d] isVagDebugBlindspotInfoTest[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBlindspotInfoTest, mIsVagDebugBlindspotInfoTest);
  printf("[BOP][%s][%s][%d] isVagDebugBlindspotWarningTest[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBlindspotWarningTest, mIsVagDebugBlindspotWarningTest);
  printf("[BOP][%s][%s][%d] isVagDebugBrakeLightTest[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBrakeLightTest, mIsVagDebugBrakeLightTest);
  printf("[BOP][%s][%s][%d] isVagDebugLeadCarGoingRemindTest[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugLeadCarGoingRemindTest, mIsVagDebugLeadCarGoingRemindTest);
  printf("[BOP][%s][%s][%d] isVagDebugNoLeadCarWarningTest[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugNoLeadCarWarningTest, mIsVagDebugNoLeadCarWarningTest);

  // ===== Test =====
  printf("[BOP][%s][%s][%d] isVagDebugOsdTestTextEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugOsdTestTextEnabled, mIsVagDebugOsdTestTextEnabled);
  printf("[BOP][%s][%s][%d] isVagDebugItem1Enabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem1Enabled, mIsVagDebugItem1Enabled);
  printf("[BOP][%s][%s][%d] isVagDebugItem2Enabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem2Enabled, mIsVagDebugItem2Enabled);
  printf("[BOP][%s][%s][%d] isVagDebugItem3Enabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem3Enabled, mIsVagDebugItem3Enabled);
  printf("[BOP][%s][%s][%d] isVagDebugItem4Enabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem4Enabled, mIsVagDebugItem4Enabled);
  printf("[BOP][%s][%s][%d] isVagDebugItem5Enabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem5Enabled, mIsVagDebugItem5Enabled);

  // ===== Setting =====
  printf("[BOP][%s][%s][%d] isVagManualSoundVolumeEnable[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagManualSoundVolumeEnable, mIsVagManualSoundVolumeEnable);
  printf("[BOP][%s][%s][%d] vagSoundVolume[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, vagSoundVolume, mVagSoundVolume);
  printf("[BOP][%s][%s][%d] isVagManualOsdBacklightEnable[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagManualOsdBacklightEnable, mIsVagManualOsdBacklightEnable);
  printf("[BOP][%s][%s][%d] vagOsdBacklight[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, vagOsdBacklight, mVagOsdBacklight);
  printf("[BOP][%s][%s][%d] isVagInfoBoxEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagInfoBoxEnabled, mIsVagInfoBoxEnabled);
  printf("[BOP][%s][%s][%d] isVagBlinkerEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBlinkerEnabled, mIsVagBlinkerEnabled);
  printf("[BOP][%s][%s][%d] isVagBrakeLightEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBrakeLightEnabled, mIsVagBrakeLightEnabled);
  printf("[BOP][%s][%s][%d] isVagLeadCarEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagLeadCarEnabled, mIsVagLeadCarEnabled);

  // ===== Feature =====
  // ----- Blindspot -----
  printf("[BOP][%s][%s][%d] isVagBlindspotEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotEnabled, mIsVagBlindspotEnabled);
  printf("[BOP][%s][%s][%d] isVagBlindspotInfoSoundEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotInfoSoundEnabled, mIsVagBlindspotInfoSoundEnabled);
  printf("[BOP][%s][%s][%d] isVagBlindspotInfoVibratorEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotInfoVibratorEnabled, mIsVagBlindspotInfoVibratorEnabled);
  printf("[BOP][%s][%s][%d] isVagBlindspotWarningSoundEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotWarningSoundEnabled, mIsVagBlindspotWarningSoundEnabled);
  printf("[BOP][%s][%s][%d] isVagBlindspotWarningVibratorEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotWarningVibratorEnabled, mIsVagBlindspotWarningVibratorEnabled);
  printf("[BOP][%s][%s][%d] isVagBlindspotVibratorWithFlka[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotVibratorWithFlka, mIsVagBlindspotVibratorWithFlka);
  // ----- FLKA -----
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnabled, mIsVagFulltimeLkaEnabled);
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnableWithBlinker[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnableWithBlinker, mIsVagFulltimeLkaEnableWithBlinker);
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnableWithBrake[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnableWithBrake, mIsVagFulltimeLkaEnableWithBrake);
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnableWithAssistant[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnableWithAssistant, mIsVagFulltimeLkaEnableWithAssistant);
  // ----- Lead car going -----
  printf("[BOP][%s][%s][%d] isVagLeadCarGoingRemindEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagLeadCarGoingRemindEnabled, mIsVagLeadCarGoingRemindEnabled);
  printf("[BOP][%s][%s][%d] isVagLeadCarGoingRemindSoundEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagLeadCarGoingRemindSoundEnabled, mIsVagLeadCarGoingRemindSoundEnabled);
  // ----- No lead car -----
  printf("[BOP][%s][%s][%d] isVagNoLeadCarEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagNoLeadCarEnabled, mIsVagNoLeadCarEnabled);
  printf("[BOP][%s][%s][%d] isVagNoLeadCarWarningSoundEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagNoLeadCarWarningSoundEnabled, mIsVagNoLeadCarWarningSoundEnabled);
  // ----- Force disable startstop -----
  printf("[BOP][%s][%s][%d] isVagForceDisableStartstop[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagForceDisableStartstop, mIsVagForceDisableStartstop);
  // ------ Dynamic DCC -----
  printf("[BOP][%s][%s][%d] isVagDynamicDccEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDynamicDccEnabled, mIsVagDynamicDccEnabled);
  // ----- Driving Mode -----
  printf("[BOP][%s][%s][%d] isVagDrivingModeEnabled[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagDrivingModeEnabled, mIsVagDrivingModeEnabled);
  printf("[BOP][%s][%s][%d] vagDrivingMode[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, (int)vagDrivingMode, (int)mVagDrivingMode);

  // ===== Warning =====
  printf("[BOP][%s][%s][%d] isVagWarningEngineTurboPressure[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineTurboPressure, mIsVagWarningEngineTurboPressure);
  printf("[BOP][%s][%s][%d] isVagWarningEngineOilPressure[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineOilPressure, mIsVagWarningEngineOilPressure);
  printf("[BOP][%s][%s][%d] isVagWarningEngineInAirTemperature[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineInAirTemperature, mIsVagWarningEngineInAirTemperature);
  printf("[BOP][%s][%s][%d] isVagWarningEngineCoolantTemperature[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineCoolantTemperature, mIsVagWarningEngineCoolantTemperature);
  printf("[BOP][%s][%s][%d] isVagWarningEngineOilTemperature[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineOilTemperature, mIsVagWarningEngineOilTemperature);
  printf("[BOP][%s][%s][%d] isVagWarningCoolantTemperature[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningCoolantTemperature, mIsVagWarningCoolantTemperature);
  printf("[BOP][%s][%s][%d] isVagWarningInAirPressure[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningInAirPressure, mIsVagWarningInAirPressure);
  printf("[BOP][%s][%s][%d] isVagWarningGearOilTemperature[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningGearOilTemperature, mIsVagWarningGearOilTemperature);
  printf("[BOP][%s][%s][%d] isVagWarningBrakePressure[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningBrakePressure, mIsVagWarningBrakePressure);
  printf("[BOP][%s][%s][%d] isVagWarningIndoorTemperature[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningIndoorTemperature, mIsVagWarningIndoorTemperature);
  printf("[BOP][%s][%s][%d] isVagWarningOutdoorTemperature[%d, %d] \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningOutdoorTemperature, mIsVagWarningOutdoorTemperature);
}

void VagParam::readFromFileParam() {
  // ===== OP toggle =====
  try {
    mExperimentalLongitudinalEnabled = Params().getBool("ExperimentalLongitudinalEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][ExperimentalLongitudinalEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mExperimentalLongitudinalEnabled = false;
  }
  try {
    mExperimentalMode = Params().getBool("ExperimentalMode");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][ExperimentalMode] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mExperimentalMode = false;
  }

  // ===== General =====
  try {
    mIsVagDisableDriverMonitorAlert = Params().getBool("IsVagDisableDriverMonitorAlert");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDisableDriverMonitorAlert] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDisableDriverMonitorAlert = false;
  }
  try {
    mIsVagLeftBlinkerSoundEnabled = Params().getBool("IsVagLeftBlinkerSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagLeftBlinkerSoundEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagLeftBlinkerSoundEnabled = false;
  }
  try {
    mIsVagRightBlinkerSoundEnabled = Params().getBool("IsVagRightBlinkerSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagRightBlinkerSoundEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagRightBlinkerSoundEnabled = false;
  }
  try {
    mIsVagDevelopOnRoadUi = Params().getBool("IsVagDevelopOnRoadUi");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDevelopOnRoadUi] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDevelopOnRoadUi = false;
  }
  try {
    mIsVagPandaJungleEnabled = Params().getBool("IsVagPandaJungleEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagPandaJungleEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagPandaJungleEnabled = false;
  }
  try {
    mIsVagDevelopModeEnabled = Params().getBool("IsVagDevelopModeEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDevelopModeEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDevelopModeEnabled = false;
  }
  try {
    mIsVagRunningProcessLogEnabled = Params().getBool("IsVagRunningProcessLogEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagRunningProcessLogEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagRunningProcessLogEnabled = false;
  }
  try {
    mIsVagParamFromCerealEnabled = Params().getBool("IsVagParamFromCerealEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagParamFromCerealEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagParamFromCerealEnabled = false;
  }

  // ===== OSD =====
  try {
    mIsVagDebugBlinkerTest = Params().getBool("IsVagDebugBlinkerTest");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugBlinkerTest] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugBlinkerTest = false;
  }
  try {
    mIsVagDebugBlindspotInfoTest = Params().getBool("IsVagDebugBlindspotInfoTest");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugBlindspotInfoTest] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugBlindspotInfoTest = false;
  }
  try {
    mIsVagDebugBlindspotWarningTest = Params().getBool("IsVagDebugBlindspotWarningTest");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugBlindspotWarningTest] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugBlindspotWarningTest = false;
  }
  try {
    mIsVagDebugBrakeLightTest = Params().getBool("IsVagDebugBrakeLightTest");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugBrakeLightTest] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugBrakeLightTest = false;
  }
  try {
    mIsVagDebugLeadCarGoingRemindTest = Params().getBool("IsVagDebugLeadCarGoingRemindTest");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugLeadCarGoingRemindTest] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugLeadCarGoingRemindTest = false;
  }
  try {
    mIsVagDebugNoLeadCarWarningTest = Params().getBool("IsVagDebugNoLeadCarWarningTest");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugNoLeadCarWarningTest] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugNoLeadCarWarningTest = false;
  }

  // ===== Test =====
  try {
    mIsVagDebugOsdTestTextEnabled = Params().getBool("IsVagDebugOsdTestTextEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugOsdTestTextEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugOsdTestTextEnabled = false;
  }
  try {
    mIsVagDebugItem1Enabled = Params().getBool("IsVagDebugItem1Enabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugItem1Enabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugItem1Enabled = false;
  }
  try {
    mIsVagDebugItem2Enabled = Params().getBool("IsVagDebugItem2Enabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugItem2Enabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugItem2Enabled = false;
  }
  try {
    mIsVagDebugItem3Enabled = Params().getBool("IsVagDebugItem3Enabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugItem3Enabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugItem3Enabled = false;
  }
  try {
    mIsVagDebugItem4Enabled = Params().getBool("IsVagDebugItem4Enabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugItem4Enabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugItem4Enabled = false;
  }
  try {
    mIsVagDebugItem5Enabled = Params().getBool("IsVagDebugItem5Enabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDebugItem5Enabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDebugItem5Enabled = false;
  }

  // ===== Setting =====
  try {
    mIsVagManualSoundVolumeEnable = Params().getBool("IsVagManualSoundVolumeEnable");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagManualSoundVolumeEnable] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagManualSoundVolumeEnable = false;
  }
  try {
    QString VagSoundVolume = QString::fromStdString(Params().get("VagSoundVolume"));
    mVagSoundVolume = VagSoundVolume.toInt();
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][VagSoundVolume] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mVagSoundVolume = 10;
  }
  try {
    mIsVagManualOsdBacklightEnable = Params().getBool("IsVagManualOsdBacklightEnable");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagManualOsdBacklightEnable] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagManualOsdBacklightEnable = false;
  }
  try {
    QString VagOsdBacklight = QString::fromStdString(Params().get("VagOsdBacklight"));
    mVagOsdBacklight = VagOsdBacklight.toInt();
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][VagOsdBacklight] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mVagOsdBacklight = 5;
  }
  try {
    mIsVagInfoBoxEnabled = Params().getBool("IsVagInfoBoxEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagInfoBoxEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagInfoBoxEnabled = false;
  }
  try {
    mIsVagBlinkerEnabled = Params().getBool("IsVagBlinkerEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlinkerEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBlinkerEnabled = false;
  }
  try {
    mIsVagBrakeLightEnabled = Params().getBool("IsVagBrakeLightEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBrakeLightEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBrakeLightEnabled = false;
  }
  try {
    mIsVagLeadCarEnabled = Params().getBool("IsVagLeadCarEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagLeadCarEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagLeadCarEnabled = false;
  }

  // ===== Feature =====
  // ----- Blindspot -----
  try {
    mIsVagBlindspotEnabled = Params().getBool("IsVagBlindspotEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlindspotEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBlindspotEnabled = false;
  }
  try {
    mIsVagBlindspotInfoSoundEnabled = Params().getBool("IsVagBlindspotInfoSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlindspotInfoSoundEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBlindspotInfoSoundEnabled = false;
  }
  try {
    mIsVagBlindspotInfoVibratorEnabled = Params().getBool("IsVagBlindspotInfoVibratorEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlindspotInfoVibratorEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBlindspotInfoVibratorEnabled = false;
  }
  try {
    mIsVagBlindspotWarningSoundEnabled = Params().getBool("IsVagBlindspotWarningSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlindspotWarningSoundEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBlindspotWarningSoundEnabled = false;
  }
  try {
    mIsVagBlindspotWarningVibratorEnabled = Params().getBool("IsVagBlindspotWarningVibratorEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlindspotWarningVibratorEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBlindspotWarningVibratorEnabled = false;
  }
  try {
    mIsVagBlindspotVibratorWithFlka = Params().getBool("IsVagBlindspotVibratorWithFlka");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlindspotVibratorWithFlka] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagBlindspotVibratorWithFlka = false;
  }
  // ----- FLKA -----
  try {
    mIsVagFulltimeLkaEnabled = Params().getBool("IsVagFulltimeLkaEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagFulltimeLkaEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagFulltimeLkaEnabled = false;
  }
  try {
    mIsVagFulltimeLkaEnableWithBlinker = Params().getBool("IsVagFulltimeLkaEnableWithBlinker");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagFulltimeLkaEnableWithBlinker] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagFulltimeLkaEnableWithBlinker = false;
  }
  try {
    mIsVagFulltimeLkaEnableWithBrake = Params().getBool("IsVagFulltimeLkaEnableWithBrake");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagFulltimeLkaEnableWithBrake] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagFulltimeLkaEnableWithBrake = false;
  }
  try {
    mIsVagFulltimeLkaEnableWithAssistant = Params().getBool("IsVagFulltimeLkaEnableWithAssistant");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagFulltimeLkaEnableWithAssistant] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagFulltimeLkaEnableWithAssistant = false;
  }
  // ----- Lead car going -----
  try {
    mIsVagLeadCarGoingRemindEnabled = Params().getBool("IsVagLeadCarGoingRemindEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagLeadCarGoingRemindEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagLeadCarGoingRemindEnabled = false;
  }
  try {
    mIsVagLeadCarGoingRemindSoundEnabled = Params().getBool("IsVagLeadCarGoingRemindSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagLeadCarGoingRemindSoundEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagLeadCarGoingRemindSoundEnabled = false;
  }
  // ----- No lead car -----
  try {
    mIsVagNoLeadCarEnabled = Params().getBool("IsVagNoLeadCarEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagNoLeadCarEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagNoLeadCarEnabled = false;
  }
  try {
    mIsVagNoLeadCarWarningSoundEnabled = Params().getBool("IsVagNoLeadCarWarningSoundEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagNoLeadCarWarningSoundEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagNoLeadCarWarningSoundEnabled = false;
  }
  // ----- Force disable startstop -----
  try {
    mIsVagForceDisableStartstop = Params().getBool("IsVagForceDisableStartstop");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagForceDisableStartstop] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagForceDisableStartstop = false;
  }
  // ------ Dynamic DCC -----
  try {
    mIsVagDynamicDccEnabled = Params().getBool("IsVagDynamicDccEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDynamicDccEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDynamicDccEnabled = false;
  }
  // ----- Driving Mode -----
  try {
    mIsVagDrivingModeEnabled = Params().getBool("IsVagDrivingModeEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagDrivingModeEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagDrivingModeEnabled = false;
  }
  try {
    std::string stringVagDrivingMode = Params().get("VagDrivingMode");
    int vagDrivingMode = std::stoi(stringVagDrivingMode);
    switch(vagDrivingMode) {
      case 0:
        mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::ECO;
        break;
      case 1:
        mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::NORMAL;
        break;
      case 2:
        mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::SPORT;
        break;
      case 3:
        mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::RACE;
        break;
      case 4:
        mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::SNOW;
        break;
      default:
        mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::NORMAL;
        break;
    }
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][VagDrivingMode] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::NORMAL;
  }

  // ===== Warning =====
  try {
    mIsVagWarningEngineTurboPressure = Params().getBool("IsVagWarningEngineTurboPressure");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningEngineTurboPressure] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningEngineTurboPressure = false;
  }
  try {
    mIsVagWarningEngineOilPressure = Params().getBool("IsVagWarningEngineOilPressure");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningEngineOilPressure] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningEngineOilPressure = false;
  }
  try {
    mIsVagWarningEngineInAirTemperature = Params().getBool("IsVagWarningEngineInAirTemperature");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningEngineInAirTemperature] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningEngineInAirTemperature = false;
  }
  try {
    mIsVagWarningEngineCoolantTemperature = Params().getBool("IsVagWarningEngineCoolantTemperature");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningEngineCoolantTemperature] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningEngineCoolantTemperature = false;
  }
  try {
    mIsVagWarningEngineOilTemperature = Params().getBool("IsVagWarningEngineOilTemperature");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningEngineOilTemperature] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningEngineOilTemperature = false;
  }
  try {
    mIsVagWarningCoolantTemperature = Params().getBool("IsVagWarningCoolantTemperature");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningCoolantTemperature] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningCoolantTemperature = false;
  }
  try {
    mIsVagWarningInAirPressure = Params().getBool("IsVagWarningInAirPressure");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningInAirPressure] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningInAirPressure = false;
  }
  try {
    mIsVagWarningGearOilTemperature = Params().getBool("IsVagWarningGearOilTemperature");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningGearOilTemperature] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningGearOilTemperature = false;
  }
  try {
    mIsVagWarningBrakePressure = Params().getBool("IsVagWarningBrakePressure");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningBrakePressure] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningBrakePressure = false;
  }
  try {
    mIsVagWarningIndoorTemperature = Params().getBool("IsVagWarningIndoorTemperature");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningIndoorTemperature] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningIndoorTemperature = false;
  }
  try {
    mIsVagWarningOutdoorTemperature = Params().getBool("IsVagWarningOutdoorTemperature");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagWarningOutdoorTemperature] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    mIsVagWarningOutdoorTemperature = false;
  }
}

void VagParam::writeToCeralParam() {
  MessageBuilder msg;
  auto cerealVagParam = msg.initEvent().initVagParam();
  // ===== OP toggle =====
  cerealVagParam.getVagParamOp().setExperimentalLongitudinalEnabled(mExperimentalLongitudinalEnabled);
  cerealVagParam.getVagParamOp().setExperimentalMode(mExperimentalMode);

  // ===== General =====
  cerealVagParam.getVagParamGeneral().setIsVagDisableDriverMonitorAlert(mIsVagDisableDriverMonitorAlert);
  cerealVagParam.getVagParamGeneral().setIsVagLeftBlinkerSoundEnabled(mIsVagLeftBlinkerSoundEnabled);
  cerealVagParam.getVagParamGeneral().setIsVagRightBlinkerSoundEnabled(mIsVagRightBlinkerSoundEnabled);
  cerealVagParam.getVagParamGeneral().setIsVagDevelopOnRoadUi(mIsVagDevelopOnRoadUi);
  cerealVagParam.getVagParamGeneral().setIsVagPandaJungleEnabled(mIsVagPandaJungleEnabled);
  cerealVagParam.getVagParamGeneral().setIsVagDevelopModeEnabled(mIsVagDevelopModeEnabled);
  cerealVagParam.getVagParamGeneral().setIsVagRunningProcessLogEnabled(mIsVagRunningProcessLogEnabled);
  cerealVagParam.getVagParamGeneral().setIsVagParamFromCerealEnabled(mIsVagParamFromCerealEnabled);

  // ===== OSD =====
  cerealVagParam.getVagParamOsd().setIsVagDebugBlinkerTest(mIsVagDebugBlinkerTest);
  cerealVagParam.getVagParamOsd().setIsVagDebugBlindspotInfoTest(mIsVagDebugBlindspotInfoTest);
  cerealVagParam.getVagParamOsd().setIsVagDebugBlindspotWarningTest(mIsVagDebugBlindspotWarningTest);
  cerealVagParam.getVagParamOsd().setIsVagDebugBrakeLightTest(mIsVagDebugBrakeLightTest);
  cerealVagParam.getVagParamOsd().setIsVagDebugLeadCarGoingRemindTest(mIsVagDebugLeadCarGoingRemindTest);
  cerealVagParam.getVagParamOsd().setIsVagDebugNoLeadCarWarningTest(mIsVagDebugNoLeadCarWarningTest);

  // ===== Test =====
  cerealVagParam.getVagParamTest().setIsVagDebugOsdTestTextEnabled(mIsVagDebugOsdTestTextEnabled);
  cerealVagParam.getVagParamTest().setIsVagDebugItem1Enabled(mIsVagDebugItem1Enabled);
  cerealVagParam.getVagParamTest().setIsVagDebugItem2Enabled(mIsVagDebugItem2Enabled);
  cerealVagParam.getVagParamTest().setIsVagDebugItem3Enabled(mIsVagDebugItem3Enabled);
  cerealVagParam.getVagParamTest().setIsVagDebugItem4Enabled(mIsVagDebugItem4Enabled);
  cerealVagParam.getVagParamTest().setIsVagDebugItem5Enabled(mIsVagDebugItem5Enabled);

  // ===== Setting =====
  cerealVagParam.getVagParamSetting().setIsVagManualSoundVolumeEnable(mIsVagManualSoundVolumeEnable);
  cerealVagParam.getVagParamSetting().setVagSoundVolume(mVagSoundVolume);
  cerealVagParam.getVagParamSetting().setIsVagManualOsdBacklightEnable(mIsVagManualOsdBacklightEnable);
  cerealVagParam.getVagParamSetting().setVagOsdBacklight(mVagOsdBacklight);
  cerealVagParam.getVagParamSetting().setIsVagInfoBoxEnabled(mIsVagInfoBoxEnabled);
  cerealVagParam.getVagParamSetting().setIsVagBlinkerEnabled(mIsVagBlinkerEnabled);
  cerealVagParam.getVagParamSetting().setIsVagBrakeLightEnabled(mIsVagBrakeLightEnabled);
  cerealVagParam.getVagParamSetting().setIsVagLeadCarEnabled(mIsVagLeadCarEnabled);

  // ===== Feature =====
  // ----- Blindspot -----
  cerealVagParam.getVagParamFeature().setIsVagBlindspotEnabled(mIsVagBlindspotEnabled);
  cerealVagParam.getVagParamFeature().setIsVagBlindspotInfoSoundEnabled(mIsVagBlindspotInfoSoundEnabled);
  cerealVagParam.getVagParamFeature().setIsVagBlindspotInfoVibratorEnabled(mIsVagBlindspotInfoVibratorEnabled);
  cerealVagParam.getVagParamFeature().setIsVagBlindspotWarningSoundEnabled(mIsVagBlindspotWarningSoundEnabled);
  cerealVagParam.getVagParamFeature().setIsVagBlindspotWarningVibratorEnabled(mIsVagBlindspotWarningVibratorEnabled);
  cerealVagParam.getVagParamFeature().setIsVagBlindspotVibratorWithFlka(mIsVagBlindspotVibratorWithFlka);
  // ----- FLKA -----
  cerealVagParam.getVagParamFeature().setIsVagFulltimeLkaEnabled(mIsVagFulltimeLkaEnabled);
  cerealVagParam.getVagParamFeature().setIsVagFulltimeLkaEnableWithBlinker(mIsVagFulltimeLkaEnableWithBlinker);
  cerealVagParam.getVagParamFeature().setIsVagFulltimeLkaEnableWithBrake(mIsVagFulltimeLkaEnableWithBrake);
  cerealVagParam.getVagParamFeature().setIsVagFulltimeLkaEnableWithAssistant(mIsVagFulltimeLkaEnableWithAssistant);
  // ----- Lead car going -----
  cerealVagParam.getVagParamFeature().setIsVagLeadCarGoingRemindEnabled(mIsVagLeadCarGoingRemindEnabled);
  cerealVagParam.getVagParamFeature().setIsVagLeadCarGoingRemindSoundEnabled(mIsVagLeadCarGoingRemindSoundEnabled);
  // ----- No lead car -----
  cerealVagParam.getVagParamFeature().setIsVagNoLeadCarEnabled(mIsVagNoLeadCarEnabled);
  cerealVagParam.getVagParamFeature().setIsVagNoLeadCarWarningSoundEnabled(mIsVagNoLeadCarWarningSoundEnabled);
  // ----- Force disable startstop -----
  cerealVagParam.getVagParamFeature().setIsVagForceDisableStartstop(mIsVagForceDisableStartstop);
  // ----- Driving Mode -----
  cerealVagParam.getVagParamFeature().setIsVagDrivingModeEnabled(mIsVagDrivingModeEnabled);
  cerealVagParam.getVagParamFeature().setVagDrivingMode(mVagDrivingMode);
  // ------ Dynamic DCC -----
  cerealVagParam.getVagParamFeature().setIsVagDynamicDccEnabled(mIsVagDynamicDccEnabled);

  // ===== Warning =====
  cerealVagParam.getVagParamWarning().setIsVagWarningEngineTurboPressure(mIsVagWarningEngineTurboPressure);
  cerealVagParam.getVagParamWarning().setIsVagWarningEngineOilPressure(mIsVagWarningEngineOilPressure);
  cerealVagParam.getVagParamWarning().setIsVagWarningEngineInAirTemperature(mIsVagWarningEngineInAirTemperature);
  cerealVagParam.getVagParamWarning().setIsVagWarningEngineCoolantTemperature(mIsVagWarningEngineCoolantTemperature);
  cerealVagParam.getVagParamWarning().setIsVagWarningEngineOilTemperature(mIsVagWarningEngineOilTemperature);
  cerealVagParam.getVagParamWarning().setIsVagWarningCoolantTemperature(mIsVagWarningCoolantTemperature);
  cerealVagParam.getVagParamWarning().setIsVagWarningInAirPressure(mIsVagWarningInAirPressure);
  cerealVagParam.getVagParamWarning().setIsVagWarningGearOilTemperature(mIsVagWarningGearOilTemperature);
  cerealVagParam.getVagParamWarning().setIsVagWarningBrakePressure(mIsVagWarningBrakePressure);
 
  pm->send("vagParam", msg);

  //std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //printInfo();
}

// ===== OP toggle =====
void VagParam::setExperimentalLongitudinalEnabled(bool experimentalLongitudinalEnabled) {
  mExperimentalLongitudinalEnabled = experimentalLongitudinalEnabled;
  printf("[BOP][%s][%s][%d] experimentalLongitudinalEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, experimentalLongitudinalEnabled);
  writeToCeralParam();
}

void VagParam::setExperimentalMode(bool experimentalMode) {
  mExperimentalMode = experimentalMode;
  printf("[BOP][%s][%s][%d] experimentalMode=%d \n", __FILE__, __FUNCTION__, __LINE__, experimentalMode);
  writeToCeralParam();
}

// ===== General =====
void VagParam::setIsVagDisableDriverMonitorAlert(bool isVagDisableDriverMonitorAlert) {
  mIsVagDisableDriverMonitorAlert = isVagDisableDriverMonitorAlert;
  printf("[BOP][%s][%s][%d] isVagDisableDriverMonitorAlert=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDisableDriverMonitorAlert);
  writeToCeralParam();
}

void VagParam::setIsVagDevelopOnRoadUi(bool isVagDevelopOnRoadUi) {
  mIsVagDevelopOnRoadUi = isVagDevelopOnRoadUi;
  printf("[BOP][%s][%s][%d] isVagDevelopOnRoadUi=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDevelopOnRoadUi);
  writeToCeralParam();
}

void VagParam::setIsVagLeftBlinkerSoundEnabled(bool isVagLeftBlinkerSoundEnabled) {
  mIsVagLeftBlinkerSoundEnabled = isVagLeftBlinkerSoundEnabled;
  printf("[BOP][%s][%s][%d] isVagLeftBlinkerSoundEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagLeftBlinkerSoundEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagRightBlinkerSoundEnabled(bool isVagRightBlinkerSoundEnabled) {
  mIsVagRightBlinkerSoundEnabled = isVagRightBlinkerSoundEnabled;
  printf("[BOP][%s][%s][%d] isVagRightBlinkerSoundEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagRightBlinkerSoundEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagPandaJungleEnabled(bool isVagPandaJungleEnabled) {
  mIsVagPandaJungleEnabled = isVagPandaJungleEnabled;
  printf("[BOP][%s][%s][%d] isVagPandaJungleEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagPandaJungleEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagDevelopModeEnabled(bool isVagDevelopModeEnabled) {
  mIsVagDevelopModeEnabled = isVagDevelopModeEnabled;
  printf("[BOP][%s][%s][%d] isVagDevelopModeEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDevelopModeEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagRunningProcessLogEnabled(bool isVagRunningProcessLogEnabled) {
  mIsVagRunningProcessLogEnabled = isVagRunningProcessLogEnabled;
  printf("[BOP][%s][%s][%d] isVagRunningProcessLogEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagRunningProcessLogEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagParamFromCerealEnabled(bool isVagParamFromCerealEnabled) {
  mIsVagParamFromCerealEnabled = isVagParamFromCerealEnabled;
  printf("[BOP][%s][%s][%d] isVagParamFromCerealEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagParamFromCerealEnabled);
  writeToCeralParam();
}

// ===== OSD =====
void VagParam::setIsVagDebugBlinkerTest(bool isVagDebugBlinkerTest) {
  mIsVagDebugBlinkerTest = isVagDebugBlinkerTest;
  printf("[BOP][%s][%s][%d] isVagDebugBlinkerTest=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBlinkerTest);
  writeToCeralParam();
}

void VagParam::setIsVagDebugBlindspotInfoTest(bool isVagDebugBlindspotInfoTest) {
  mIsVagDebugBlindspotInfoTest = isVagDebugBlindspotInfoTest;
  printf("[BOP][%s][%s][%d] isVagDebugBlindspotInfoTest=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBlindspotInfoTest);
  writeToCeralParam();
}

void VagParam::setIsVagDebugBlindspotWarningTest(bool isVagDebugBlindspotWarningTest) {
  mIsVagDebugBlindspotWarningTest = isVagDebugBlindspotWarningTest;
  printf("[BOP][%s][%s][%d] isVagDebugBlindspotWarningTest=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBlindspotWarningTest);
  writeToCeralParam();
}

void VagParam::setIsVagDebugBrakeLightTest(bool isVagDebugBrakeLightTest) {
  mIsVagDebugBrakeLightTest = isVagDebugBrakeLightTest;
  printf("[BOP][%s][%s][%d] isVagDebugBrakeLightTest=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugBrakeLightTest);
  writeToCeralParam();
}

void VagParam::setIsVagDebugLeadCarGoingRemindTest(bool isVagDebugLeadCarGoingRemindTest) {
  mIsVagDebugLeadCarGoingRemindTest = isVagDebugLeadCarGoingRemindTest;
  printf("[BOP][%s][%s][%d] isVagDebugLeadCarGoingRemindTest=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugLeadCarGoingRemindTest);
  writeToCeralParam();
}

void VagParam::setIsVagDebugNoLeadCarWarningTest(bool isVagDebugNoLeadCarWarningTest) {
  mIsVagDebugNoLeadCarWarningTest = isVagDebugNoLeadCarWarningTest;
  printf("[BOP][%s][%s][%d] isVagDebugNoLeadCarWarningTest=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugNoLeadCarWarningTest);
  writeToCeralParam();
}

// ===== Test =====
void VagParam::setIsVagDebugOsdTestTextEnabled(bool isVagDebugOsdTestTextEnabled) {
  mIsVagDebugOsdTestTextEnabled = isVagDebugOsdTestTextEnabled;
  printf("[BOP][%s][%s][%d] isVagDebugOsdTestTextEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugOsdTestTextEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagDebugItem1Enabled(bool isVagDebugItem1Enabled) {
  mIsVagDebugItem1Enabled = isVagDebugItem1Enabled;
  printf("[BOP][%s][%s][%d] isVagDebugItem1Enabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem1Enabled);
  writeToCeralParam();
}

void VagParam::setIsVagDebugItem2Enabled(bool isVagDebugItem2Enabled) {
  mIsVagDebugItem2Enabled = isVagDebugItem2Enabled;
  printf("[BOP][%s][%s][%d] isVagDebugItem2Enabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem2Enabled);
  writeToCeralParam();
}

void VagParam::setIsVagDebugItem3Enabled(bool isVagDebugItem3Enabled) {
  mIsVagDebugItem3Enabled = isVagDebugItem3Enabled;
  printf("[BOP][%s][%s][%d] isVagDebugItem3Enabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem3Enabled);
  writeToCeralParam();
}

void VagParam::setIsVagDebugItem4Enabled(bool isVagDebugItem4Enabled) {
  mIsVagDebugItem4Enabled = isVagDebugItem4Enabled;
  printf("[BOP][%s][%s][%d] isVagDebugItem4Enabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem4Enabled);
  writeToCeralParam();
}

void VagParam::setIsVagDebugItem5Enabled(bool isVagDebugItem5Enabled) {
  mIsVagDebugItem5Enabled = isVagDebugItem5Enabled;
  printf("[BOP][%s][%s][%d] isVagDebugItem5Enabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDebugItem5Enabled);
  writeToCeralParam();
}

// ===== Setting =====
void VagParam::setIsVagManualSoundVolumeEnable(bool isVagManualSoundVolumeEnable) {
  mIsVagManualSoundVolumeEnable = isVagManualSoundVolumeEnable;
  printf("[BOP][%s][%s][%d] isVagManualSoundVolumeEnable=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagManualSoundVolumeEnable);
  writeToCeralParam();
}

void VagParam::setVagSoundVolume(int vagSoundVolume) {
  mVagSoundVolume = vagSoundVolume;
  printf("[BOP][%s][%s][%d] vagSoundVolume=%d \n", __FILE__, __FUNCTION__, __LINE__, vagSoundVolume);
  writeToCeralParam();
}

void VagParam::setIsVagManualOsdBacklightEnable(bool isVagManualOsdBacklightEnable) {
  mIsVagManualOsdBacklightEnable = isVagManualOsdBacklightEnable;
  printf("[BOP][%s][%s][%d] isVagManualOsdBacklightEnable=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagManualOsdBacklightEnable);
  writeToCeralParam();
}

void VagParam::setVagOsdBacklight(int vagOsdBacklight) {
  mVagOsdBacklight = vagOsdBacklight;
  printf("[BOP][%s][%s][%d] vagOsdBacklight=%d \n", __FILE__, __FUNCTION__, __LINE__, vagOsdBacklight);
  writeToCeralParam();
}

void VagParam::setIsVagInfoBoxEnabled(bool isVagInfoBoxEnabled) {
  mIsVagInfoBoxEnabled = isVagInfoBoxEnabled;
  printf("[BOP][%s][%s][%d] isVagInfoBoxEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagInfoBoxEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagBlinkerEnabled(bool isVagBlinkerEnabled) {
  mIsVagBlinkerEnabled = isVagBlinkerEnabled;
  printf("[BOP][%s][%s][%d] isVagBlinkerEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBlinkerEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagBrakeLightEnabled(bool isVagBrakeLightEnabled) {
  mIsVagBrakeLightEnabled = isVagBrakeLightEnabled;
  printf("[BOP][%s][%s][%d] isVagBrakeLightEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBrakeLightEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagLeadCarEnabled(bool isVagLeadCarEnabled) {
  mIsVagLeadCarEnabled = isVagLeadCarEnabled;
  printf("[BOP][%s][%s][%d] isVagLeadCarEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagLeadCarEnabled);
  writeToCeralParam();
}

// ===== Feature =====
// ----- Blindspot -----
void VagParam::setIsVagBlindspotEnabled(bool isVagBlindspotEnabled) {
  mIsVagBlindspotEnabled = isVagBlindspotEnabled;
  printf("[BOP][%s][%s][%d] isVagBlindspotEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagBlindspotInfoSoundEnabled(bool isVagBlindspotInfoSoundEnabled) {
  mIsVagBlindspotInfoSoundEnabled = isVagBlindspotInfoSoundEnabled;
  printf("[BOP][%s][%s][%d] isVagBlindspotInfoSoundEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotInfoSoundEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagBlindspotInfoVibratorEnabled(bool isVagBlindspotInfoVibratorEnabled) {
  mIsVagBlindspotInfoVibratorEnabled = isVagBlindspotInfoVibratorEnabled;
  printf("[BOP][%s][%s][%d] isVagBlindspotInfoVibratorEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotInfoVibratorEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagBlindspotWarningSoundEnabled(bool isVagBlindspotWarningSoundEnabled) {
  mIsVagBlindspotWarningSoundEnabled = isVagBlindspotWarningSoundEnabled;
  printf("[BOP][%s][%s][%d] isVagBlindspotWarningSoundEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotWarningSoundEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagBlindspotWarningVibratorEnabled(bool isVagBlindspotWarningVibratorEnabled) {
  mIsVagBlindspotWarningVibratorEnabled = isVagBlindspotWarningVibratorEnabled;
  printf("[BOP][%s][%s][%d] isVagBlindspotWarningVibratorEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotWarningVibratorEnabled);
  writeToCeralParam();
}
void VagParam::setIsVagBlindspotVibratorWithFlka(bool isVagBlindspotVibratorWithFlka) {
  mIsVagBlindspotVibratorWithFlka = isVagBlindspotVibratorWithFlka;
  printf("[BOP][%s][%s][%d] isVagBlindspotVibratorWithFlka=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagBlindspotVibratorWithFlka);
  writeToCeralParam();
}

// ----- FLKA -----
void VagParam::setIsVagFulltimeLkaEnabled(bool isVagFulltimeLkaEnabled) {
  mIsVagFulltimeLkaEnabled = isVagFulltimeLkaEnabled;
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagFulltimeLkaEnableWithBlinker(bool isVagFulltimeLkaEnableWithBlinker) {
  mIsVagFulltimeLkaEnableWithBlinker = isVagFulltimeLkaEnableWithBlinker;
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnableWithBlinker=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnableWithBlinker);
  writeToCeralParam();
}

void VagParam::setIsVagFulltimeLkaEnableWithBrake(bool isVagFulltimeLkaEnableWithBrake) {
  mIsVagFulltimeLkaEnableWithBrake = isVagFulltimeLkaEnableWithBrake;
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnableWithBrake=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnableWithBrake);
  writeToCeralParam();
}

void VagParam::setIsVagFulltimeLkaEnableWithAssistant(bool isVagFulltimeLkaEnableWithAssistant) {
  mIsVagFulltimeLkaEnableWithAssistant = isVagFulltimeLkaEnableWithAssistant;
  printf("[BOP][%s][%s][%d] isVagFulltimeLkaEnableWithAssistant=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagFulltimeLkaEnableWithAssistant);
  writeToCeralParam();
}

// ----- Lead car going -----
void VagParam::setIsVagLeadCarGoingRemindEnabled(bool isVagLeadCarGoingRemindEnabled) {
  mIsVagLeadCarGoingRemindEnabled = isVagLeadCarGoingRemindEnabled;
  printf("[BOP][%s][%s][%d] isVagLeadCarGoingRemindEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagLeadCarGoingRemindEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagLeadCarGoingRemindSoundEnabled(bool isVagLeadCarGoingRemindSoundEnabled) {
  mIsVagLeadCarGoingRemindSoundEnabled = isVagLeadCarGoingRemindSoundEnabled;
  printf("[BOP][%s][%s][%d] mIsVagLeadCarGoingRemindSoundEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, mIsVagLeadCarGoingRemindSoundEnabled);
  writeToCeralParam();
}

// ----- No lead car -----
void VagParam::setIsVagNoLeadCarEnabled(bool isVagNoLeadCarEnabled) {
  mIsVagNoLeadCarEnabled = isVagNoLeadCarEnabled;
  printf("[BOP][%s][%s][%d] isVagNoLeadCarEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagNoLeadCarEnabled);
  writeToCeralParam();
}

void VagParam::setIsVagNoLeadCarWarningSoundEnabled(bool isVagNoLeadCarWarningSoundEnabled) {
  mIsVagNoLeadCarWarningSoundEnabled = isVagNoLeadCarWarningSoundEnabled;
  printf("[BOP][%s][%s][%d] isVagNoLeadCarWarningSoundEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagNoLeadCarWarningSoundEnabled);
  writeToCeralParam();
}

// ----- Force disable startstop -----
void VagParam::setIsVagForceDisableStartstop(bool isVagForceDisableStartstop) {
  mIsVagForceDisableStartstop = isVagForceDisableStartstop;
  printf("[BOP][%s][%s][%d] isVagForceDisableStartstop=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagForceDisableStartstop);
  writeToCeralParam();
}

// ----- Driving Mode -----
void VagParam::setIsVagDynamicDccEnabled(bool isVagDynamicDccEnabled) {
  mIsVagDynamicDccEnabled = isVagDynamicDccEnabled;
  printf("[BOP][%s][%s][%d] isVagDynamicDccEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDynamicDccEnabled);
  writeToCeralParam();
}

// ----- Driving Mode -----
void VagParam::setIsVagDrivingModeEnabled(bool isVagDrivingModeEnabled) {
  mIsVagDrivingModeEnabled = isVagDrivingModeEnabled;
  printf("[BOP][%s][%s][%d] isVagDrivingModeEnabled=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagDrivingModeEnabled);
  writeToCeralParam();
}

void VagParam::setVagDrivingMode(int vagDrivingMode) {
  printf("[PONTEST][%s][%s][%d] vagDrivingMode=%d \n", __FILE__, __FUNCTION__, __LINE__, (int)vagDrivingMode);
  switch(vagDrivingMode) {
    case 0:
      mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::ECO;
      break;
    case 1:
      mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::NORMAL;
      break;
    case 2:
      mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::SPORT;
      break;
    case 3:
      mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::RACE;
      break;
    case 4:
      mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::SNOW;
      break;
    default:
      mVagDrivingMode = cereal::VagCarControl::VagDrivingMode::NORMAL;
      break;
  }
  printf("[BOP][%s][%s][%d] mVagDrivingMode=%hu \n", __FILE__, __FUNCTION__, __LINE__, mVagDrivingMode);
  writeToCeralParam();
}

// ===== Warning =====
void VagParam::setIsVagWarningEngineTurboPressure(bool isVagWarningEngineTurboPressure) {
  mIsVagWarningEngineTurboPressure = isVagWarningEngineTurboPressure;
  printf("[BOP][%s][%s][%d] isVagWarningEngineTurboPressure=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineTurboPressure);
  writeToCeralParam();
}

void VagParam::setIsVagWarningEngineOilPressure(bool isVagWarningEngineOilPressure) {
  mIsVagWarningEngineOilPressure = isVagWarningEngineOilPressure;
  printf("[BOP][%s][%s][%d] isVagWarningEngineOilPressure=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineOilPressure);
  writeToCeralParam();
}

void VagParam::setIsVagWarningEngineInAirTemperature(bool isVagWarningEngineInAirTemperature) {
  mIsVagWarningEngineInAirTemperature = isVagWarningEngineInAirTemperature;
  printf("[BOP][%s][%s][%d] isVagWarningEngineInAirTemperature=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineInAirTemperature);
  writeToCeralParam();
}

void VagParam::setIsVagWarningEngineCoolantTemperature(bool isVagWarningEngineCoolantTemperature) {
  mIsVagWarningEngineCoolantTemperature = isVagWarningEngineCoolantTemperature;
  printf("[BOP][%s][%s][%d] isVagWarningEngineCoolantTemperature=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineCoolantTemperature);
  writeToCeralParam();
}

void VagParam::setIsVagWarningEngineOilTemperature(bool isVagWarningEngineOilTemperature) {
  mIsVagWarningEngineOilTemperature = isVagWarningEngineOilTemperature;
  printf("[BOP][%s][%s][%d] isVagWarningEngineOilTemperature=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningEngineOilTemperature);
  writeToCeralParam();
}

void VagParam::setIsVagWarningCoolantTemperature(bool isVagWarningCoolantTemperature) {
  mIsVagWarningCoolantTemperature = isVagWarningCoolantTemperature;
  printf("[BOP][%s][%s][%d] isVagWarningCoolantTemperature=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningCoolantTemperature);
  writeToCeralParam();
}

void VagParam::setIsVagWarningInAirPressure(bool isVagWarningInAirPressure) {
  mIsVagWarningInAirPressure = isVagWarningInAirPressure;
  printf("[BOP][%s][%s][%d] isVagWarningInAirPressure=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningInAirPressure);
  writeToCeralParam();
}

void VagParam::setIsVagWarningGearOilTemperature(bool isVagWarningGearOilTemperature) {
  mIsVagWarningGearOilTemperature = isVagWarningGearOilTemperature;
  printf("[BOP][%s][%s][%d] isVagWarningGearOilTemperature=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningGearOilTemperature);
  writeToCeralParam();
}

void VagParam::setIsVagWarningBrakePressure(bool isVagWarningBrakePressure) {
  mIsVagWarningBrakePressure = isVagWarningBrakePressure;
  printf("[BOP][%s][%s][%d] isVagWarningBrakePressure=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningBrakePressure);
  writeToCeralParam();
}

void VagParam::setIsVagWarningIndoorTemperature(bool isVagWarningIndoorTemperature) {
  mIsVagWarningIndoorTemperature = isVagWarningIndoorTemperature;
  printf("[BOP][%s][%s][%d] isVagWarningIndoorTemperature=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningIndoorTemperature);
  writeToCeralParam();
}

void VagParam::setIsVagWarningOutdoorTemperature(bool isVagWarningOutdoorTemperature) {
  mIsVagWarningOutdoorTemperature = isVagWarningOutdoorTemperature;
  printf("[BOP][%s][%s][%d] isVagWarningOutdoorTemperature=%d \n", __FILE__, __FUNCTION__, __LINE__, isVagWarningOutdoorTemperature);
  writeToCeralParam();
}

