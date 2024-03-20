/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "selfdrive/ui/qt/vag_debug.h"

#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"


VagGeneralPanel::VagGeneralPanel(QWidget* parent) : ListWidget(parent) {
  mVagParam = VagParam::getInstance();
  mVBoxList = new QVBoxLayout();
  mVBoxList->setSpacing(30);
  addItem(mVBoxList);

  mParamControlIsVagDisableDriverMonitorAlert = new ParamControl("IsVagDisableDriverMonitorAlert",
                                            tr("Disable driver monitor alert"),
                                            tr("Disable driver monitor alert"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDisableDriverMonitorAlert, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDisableDriverMonitorAlert(bool)));
  mVBoxList->addWidget(mParamControlIsVagDisableDriverMonitorAlert);

  mParamControlIsVagLeftBlinkerSoundEnabled = new ParamControl("IsVagLeftBlinkerSoundEnabled",
                                            tr("Enable left blinker sound"),
                                            tr("Enable left blinker sound"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagLeftBlinkerSoundEnabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagLeftBlinkerSoundEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagLeftBlinkerSoundEnabled);

  mParamControlIsVagRightBlinkerSoundEnabled = new ParamControl("IsVagRightBlinkerSoundEnabled",
                                            tr("Enable right blinker sound"),
                                            tr("Enable right blinker sound"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagRightBlinkerSoundEnabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagRightBlinkerSoundEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagRightBlinkerSoundEnabled);

  mParamControlIsVagDevelopOnRoadUi = new ParamControl("IsVagDevelopOnRoadUi",
                                            tr("Enable on road screen for develop"),
                                            tr("Enable on road screen for develop"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDevelopOnRoadUi, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDevelopOnRoadUi(bool)));
  mVBoxList->addWidget(mParamControlIsVagDevelopOnRoadUi);

  mParamControlIsVagPandaJungleEnabled = new ParamControl("IsVagPandaJungleEnabled",
                                            tr("Enable panda jungle develop mode"),
                                            tr("IsVagPandaJungleEnabled"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagPandaJungleEnabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagPandaJungleEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagPandaJungleEnabled);

  mParamControlIsVagDevelopModeEnabled = new ParamControl("IsVagDevelopModeEnabled",
                                            tr("Enable develop mode"),
                                            tr("Enable develop mode"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDevelopModeEnabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDevelopModeEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDevelopModeEnabled);

  mParamControlIsVagRunningProcessLogEnabled = new ParamControl("IsVagRunningProcessLogEnabled",
                                            tr("Enable running process log"),
                                            tr("Enable running process log"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagRunningProcessLogEnabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagRunningProcessLogEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagRunningProcessLogEnabled);

  auto resetCalibBtn = new ButtonControl("Reset Calibration", "RESET", " ");
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm(tr("Are you sure you want to reset calibration?"), tr("Reset") , this)) {
      Params().remove("CalibrationParams");
    }
  });
  addItem(resetCalibBtn);
}

VagOsdPanel::VagOsdPanel(QWidget* parent) : ListWidget(parent) {
  mVagParam = VagParam::getInstance();
  mVBoxList = new QVBoxLayout();
  mVBoxList->setSpacing(30);
  addItem(mVBoxList);

#if 0
  // ----- InfoBox Test -----
  mParamControlIsVagDebugInfoBoxTest = new ParamControl("IsVagDebugInfoBoxTest",
                                            tr("Info Box OSD test"),
                                            tr("Info Box OSD test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugInfoBoxTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugInfoBoxTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugInfoBoxTest);

  // ----- Infobar Test -----
  mParamControlIsVagDebugInfobarTest = new ParamControl("IsVagDebugInfobarTest",
                                            tr("Infobar OSD test"),
                                            tr("Infobar OSD test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugInfobarTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugInfobarTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugInfobarTest);
#endif

  // ----- Blinker Test -----
  mParamControlIsVagDebugBlinkerTest = new ParamControl("IsVagDebugBlinkerTest",
                                            tr("Blinker OSD test"),
                                            tr("Blinker OSD test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugBlinkerTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugBlinkerTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugBlinkerTest);

  // ----- Blindspot Info Test -----
  mParamControlIsVagDebugBlindspotInfoTest = new ParamControl("IsVagDebugBlindspotInfoTest",
                                            tr("Blindspot Info OSD test"),
                                            tr("Blindspot Info OSD test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugBlindspotInfoTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugBlindspotInfoTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugBlindspotInfoTest);

  // ----- Blindspot Warning Test -----
  mParamControlIsVagDebugBlindspotWarningTest = new ParamControl("IsVagDebugBlindspotWarningTest",
                                            tr("Blindspot Warning OSD test"),
                                            tr("Blindspot Warning OSD test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugBlindspotWarningTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugBlindspotWarningTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugBlindspotWarningTest);

  // ----- Brake Light Test -----
  mParamControlIsVagDebugBrakeLightTest = new ParamControl("IsVagDebugBrakeLightTest",
                                            tr("Brake Light test"),
                                            tr("Brake Light test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugBrakeLightTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugBrakeLightTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugBrakeLightTest);

  // ----- Lead Car Going Remind Test -----
  mParamControlIsVagDebugLeadCarGoingRemindTest = new ParamControl("IsVagDebugLeadCarGoingRemindTest",
                                            tr("Lear Car Going Remind test"),
                                            tr("Lear Car Going Remind test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugLeadCarGoingRemindTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugLeadCarGoingRemindTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugLeadCarGoingRemindTest);

  // ----- No Lead Car Warning Test -----
  mParamControlIsVagDebugNoLeadCarWarningTest = new ParamControl("IsVagDebugNoLeadCarWarningTest",
                                            tr("No Lead Car Warning test"),
                                            tr("No Lead Car Warning test"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugNoLeadCarWarningTest, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugNoLeadCarWarningTest(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugNoLeadCarWarningTest);

}

VagTestPanel::VagTestPanel(QWidget* parent) : ListWidget(parent) {
  mVagParam = VagParam::getInstance();
  mVBoxList = new QVBoxLayout();
  mVBoxList->setSpacing(30);
  addItem(mVBoxList);

  // ----- OSD Test Text -----
  mParamControlIsVagDebugOsdTestTextEnabled = new ParamControl("IsVagDebugOsdTestTextEnabled",
                                            tr("Enable OSD test text"),
                                            tr("Enable OSD test text"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugOsdTestTextEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagDebugOsdTestTextEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugOsdTestTextEnabled);

  // ----- Debug Item 1 -----
  mParamControlIsVagDebugItem1Enabled = new ParamControl("IsVagDebugItem1Enabled",
                                            tr("Debug Item 1"),
                                            tr("Debug Item 1"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugItem1Enabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugItem1Enabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugItem1Enabled);

  // ----- Debug Item 2 -----
  mParamControlIsVagDebugItem2Enabled = new ParamControl("IsVagDebugItem2Enabled",
                                            tr("Debug Item 2"),
                                            tr("Debug Item 2"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugItem2Enabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugItem2Enabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugItem2Enabled);

  // ----- Debug Item 3 -----
  mParamControlIsVagDebugItem3Enabled = new ParamControl("IsVagDebugItem3Enabled",
                                            tr("Debug Item 3"),
                                            tr("Debug Item 3"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugItem3Enabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugItem3Enabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugItem3Enabled);

  // ----- Debug Item 4 -----
  mParamControlIsVagDebugItem4Enabled = new ParamControl("IsVagDebugItem4Enabled",
                                            tr("Debug Item 4"),
                                            tr("Debug Item 4"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugItem4Enabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugItem4Enabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugItem4Enabled);

  // ----- Debug Item 5 -----
  mParamControlIsVagDebugItem5Enabled = new ParamControl("IsVagDebugItem5Enabled",
                                            tr("Debug Item 5"),
                                            tr("Debug Item 5"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDebugItem5Enabled, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagDebugItem5Enabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDebugItem5Enabled);

  // ------ Dynamic DCC -----
  mParamControlIsVagDynamicDccEnabled = new ParamControl("IsVagDynamicDccEnabled",
                                            tr("Enable Dynamic DCC"),
                                            tr("Enable Dynamic DCC"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDynamicDccEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagDynamicDccEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDynamicDccEnabled);

  // ----- Driving Mode -----
  mParamControlIsVagDrivingModeEnabled = new ParamControl("IsVagDrivingModeEnabled",
                                            tr("Enable driving mode"),
                                            tr("Enable driving mode"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagDrivingModeEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagDrivingModeEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagDrivingModeEnabled);

  std::vector<QString> driving_mode_button{tr("Eco"), tr("Normal"), tr("Sport"), tr("Race"), tr("Snow")};
  mVagDrivingMode = new ButtonParamControl("VagDrivingMode", tr("VAG driving mode"),
                                          tr("driving mode clone from Kodiaq with DCC"),
                                          "",
                                          driving_mode_button);
  QObject::connect(mVagDrivingMode, SIGNAL(valueChanged(int)), mVagParam,  SLOT(setVagDrivingMode(int)));
  addItem(mVagDrivingMode);
}

void VagTestPanel::showEvent(QShowEvent *event) {
  UIState *s = uiState();
  const bool bus0Charisma01 = (*s->sm)["carParams"].getCarParams().getVagCarParams().getVagCanModule().getBus0Charisma01();
  const bool bus0Charisma07 = (*s->sm)["carParams"].getCarParams().getVagCarParams().getVagCanModule().getBus0Charisma07();
  if(bus0Charisma01 && bus0Charisma07) {
    mParamControlIsVagDynamicDccEnabled->setEnabled(true);
    mParamControlIsVagDrivingModeEnabled->setEnabled(true);
    mVagDrivingMode->setEnabled(true);
  } else {
    mParamControlIsVagDynamicDccEnabled->setEnabled(false);
    mParamControlIsVagDrivingModeEnabled->setEnabled(false);
    mVagDrivingMode->setEnabled(false);
  }
}

VagSettingPanel::VagSettingPanel(QWidget* parent) : ListWidget(parent) {
  mVagParam = VagParam::getInstance();
  mVBoxList = new QVBoxLayout();
  mVBoxList->setSpacing(30);
  addItem(mVBoxList);

  // ----- Maunal sound volume -----
  QString VagSoundVolume = "10";
  try {
    VagSoundVolume = QString::fromStdString(Params().get("VagSoundVolume"));
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][VagSoundVolume] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    VagSoundVolume = "10";
  }
  mParamControlIsVagManualSoundVolumeEnable = new ParamControl("IsVagManualSoundVolumeEnable",
                                            tr("Enable manual sound volume"),
                                            tr("Enable manual sound volume"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagManualSoundVolumeEnable, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagManualSoundVolumeEnable(bool)));
  mVBoxList->addWidget(mParamControlIsVagManualSoundVolumeEnable);

  mHBoxVagSoundVolume = new QHBoxLayout();
  mLabelVagSoundVolumeTitle = new QLabel();
  mSliderVagSoundVolume = new QSlider();
  mLabelVagSoundVolumeValue = new QLabel();
  QObject::connect(mSliderVagSoundVolume, &QSlider::valueChanged, this, &VagSettingPanel::setVolume);
  QObject::connect(mSliderVagSoundVolume, SIGNAL(valueChanged(int)), mLabelVagSoundVolumeValue, SLOT(setNum(int)));
  QObject::connect(mSliderVagSoundVolume, SIGNAL(valueChanged(int)), mVagParam,  SLOT(setVagSoundVolume(int)));
  mLabelVagSoundVolumeTitle->setText(tr("  Volume   "));
  mLabelVagSoundVolumeTitle->setStyleSheet( "QLabel {height: 100px; padding-top: 15px; padding-bottom: 15px;}");
  mSliderVagSoundVolume->setStyleSheet( "QSlider::horizontal {background: #FFFFFF; height: 90px;}"
                               "QSlider::groove:horizontal {background: #FFFFFF;}"
                               "QSlider::handle:horizontal {width: 100px; background: #444444;}" );
  mSliderVagSoundVolume->setOrientation(Qt::Horizontal);
  mSliderVagSoundVolume->setRange(0, 10);
  mSliderVagSoundVolume->sliderMoved(1);
  mSliderVagSoundVolume->setTickInterval(1);
  mSliderVagSoundVolume->setSingleStep(1);
  mSliderVagSoundVolume->setPageStep(1);
  mSliderVagSoundVolume->setValue(VagSoundVolume.toInt());
  mHBoxVagSoundVolume->addWidget(mLabelVagSoundVolumeTitle);
  mHBoxVagSoundVolume->addWidget(mSliderVagSoundVolume);
  mHBoxVagSoundVolume->addWidget(mLabelVagSoundVolumeValue);
  mVBoxList->addLayout(mHBoxVagSoundVolume);
  //mVBoxList->addWidget(horizontal_line());

  // ----- Maunal osd backlight -----
  QString VagOsdBacklight = QString::fromStdString(Params().get("VagOsdBacklight"));
  mParamControlIsVagManualOsdBacklightEnable = new ParamControl("IsVagManualOsdBacklightEnable",
                                            tr("Enable manual OSD Backlight"),
                                            tr("Enable manual OSD Backlight"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagManualOsdBacklightEnable, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagManualOsdBacklightEnable(bool)));
  mVBoxList->addWidget(mParamControlIsVagManualOsdBacklightEnable);

  mHBoxVagOsdBacklight = new QHBoxLayout();
  mLabelVagOsdBacklightTitle = new QLabel();
  mSliderVagOsdBacklight = new QSlider();
  mLabelVagOsdBacklightValue = new QLabel();
  QObject::connect(mSliderVagOsdBacklight, &QSlider::valueChanged, this, &VagSettingPanel::setBacklight);
  QObject::connect(mSliderVagOsdBacklight, SIGNAL(valueChanged(int)), mLabelVagOsdBacklightValue, SLOT(setNum(int)));
  QObject::connect(mSliderVagOsdBacklight, SIGNAL(valueChanged(int)), mVagParam,  SLOT(setVagOsdBacklight(int)));
  mLabelVagOsdBacklightTitle->setText(tr("  Backlight"));
  mLabelVagOsdBacklightTitle->setStyleSheet( "QLabel {height: 100px; padding-top: 15px; padding-bottom: 15px;}");
  mSliderVagOsdBacklight->setStyleSheet( "QSlider::horizontal {background: #FFFFFF;  height: 90px;}"
                               "QSlider::groove:horizontal {background: #FFFFFF;}"
                               "QSlider::handle:horizontal {width: 100px; background: #444444;}" );
  mSliderVagOsdBacklight->setOrientation(Qt::Horizontal);
  mSliderVagOsdBacklight->setRange(1, 10);
  mSliderVagOsdBacklight->sliderMoved(1);
  mSliderVagOsdBacklight->setTickInterval(1);
  mSliderVagOsdBacklight->setSingleStep(1);
  mSliderVagOsdBacklight->setPageStep(1);
  mSliderVagOsdBacklight->setValue(VagOsdBacklight.toInt());
  mHBoxVagOsdBacklight->addWidget(mLabelVagOsdBacklightTitle);
  mHBoxVagOsdBacklight->addWidget(mSliderVagOsdBacklight);
  mHBoxVagOsdBacklight->addWidget(mLabelVagOsdBacklightValue);
  mVBoxList->addLayout(mHBoxVagOsdBacklight);

  // ----- Info Box -----
  //mVBoxList->addWidget(horizontal_line());
  mParamControlIsVagInfoBoxEnabled = new ParamControl("IsVagInfoBoxEnabled",
                                            tr("Enable Info Box"),
                                            tr("Show info box on the screen"),
                                            "");
  QObject::connect(mParamControlIsVagInfoBoxEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagInfoBoxEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagInfoBoxEnabled);

  // ----- Blink -----
  //mVBoxList->addWidget(horizontal_line());
  mParamControlIsVagBlinkerEnabled = new ParamControl("IsVagBlinkerEnabled",
                                            tr("Enable Blinker"),
                                            tr("Show blinker on the screen"),
                                            "");
  QObject::connect(mParamControlIsVagBlinkerEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBlinkerEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagBlinkerEnabled);

  // ----- Brake light -----
  //mVBoxList->addWidget(horizontal_line());
  mParamControlIsVagBrakeLightEnabled = new ParamControl("IsVagBrakeLightEnabled",
                                            tr("Enable Brake Light"),
                                            tr("Show brake light on the view screen"),
                                            "");
  QObject::connect(mParamControlIsVagBrakeLightEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBrakeLightEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagBrakeLightEnabled);

  // ----- Lead car -----
  //mVBoxList->addWidget(horizontal_line());
  mParamControlIsVagLeadCarEnabled = new ParamControl("IsVagLeadCarEnabled",
                                            tr("Enable Lead Car"),
                                            tr("Show lead car on the view screen"),
                                            "");
  QObject::connect(mParamControlIsVagLeadCarEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagLeadCarEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagLeadCarEnabled);

}

void VagSettingPanel::setVolume(int volume) {
  QString stringVolume = QString::number(volume);
  try {
    Params().put("VagSoundVolume", stringVolume.toStdString());
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][VagSoundVolume] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
  }
}

void VagSettingPanel::setBacklight(int backlight) {
  bool IsVagManualOsdBacklightEnable = false;
  try {
    IsVagManualOsdBacklightEnable = Params().getBool("IsVagManualOsdBacklightEnable");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagManualOsdBacklightEnable] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    IsVagManualOsdBacklightEnable = false;
  }
  QString stringBacklight = QString::number(backlight);
  try {
    Params().put("VagOsdBacklight", stringBacklight.toStdString());
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][VagOsdBacklight] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
  }
  if(IsVagManualOsdBacklightEnable) {
    Hardware::set_brightness(backlight*backlight);
  }
}

void VagSettingPanel::setAutoShowdownMinutes(int minutes) {
  QString stringMinutes = QString::number(minutes);
  Params().put("VagAutoshutdownMinutes", stringMinutes.toStdString());
}

void VagSettingPanel::showEvent(QShowEvent *event) {
}

VagFeaturePanel::VagFeaturePanel(QWidget* parent) : ListWidget(parent) {
  UIState *s = uiState();
  mVagParam = VagParam::getInstance();
  mVBoxList = new QVBoxLayout();
  mVBoxList->setSpacing(30);
  addItem(mVBoxList);

  // ----- Blindspot -----
  mParamControlIsVagBlindspotEnabled = new ParamControl("IsVagBlindspotEnabled",
                                            tr("Enable Blindspot"),
                                            tr("Show blindspot on the view screen"),
                                            "");
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBlindspotEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagBlindspotEnabled);

  mParamControlIsVagBlindspotInfoSoundEnabled = new ParamControl("IsVagBlindspotInfoSoundEnabled",
                                            tr("  Enable Blindspot info sound"),
                                            tr("Play blindspot info sound"),
                                            "");
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagBlindspotInfoSoundEnabled, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagBlindspotInfoSoundEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBlindspotInfoSoundEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagBlindspotInfoSoundEnabled);

  mParamControlIsVagBlindspotInfoVibratorEnabled = new ParamControl("IsVagBlindspotInfoVibratorEnabled",
                                            tr("  Enable Blindspot info vibrator"),
                                            tr("Play blindspot info vibrator"),
                                            "");
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagBlindspotInfoVibratorEnabled, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagBlindspotInfoVibratorEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBlindspotInfoVibratorEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagBlindspotInfoVibratorEnabled);

  mParamControlIsVagBlindspotWarningSoundEnabled = new ParamControl("IsVagBlindspotWarningSoundEnabled",
                                            tr("  Enable Blindspot warning sound"),
                                            tr("Play blindspot warning sound"),
                                            "");
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagBlindspotWarningSoundEnabled, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagBlindspotWarningSoundEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBlindspotWarningSoundEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagBlindspotWarningSoundEnabled);

  mParamControlIsVagBlindspotWarningVibratorEnabled = new ParamControl("IsVagBlindspotWarningVibratorEnabled",
                                            tr("  Enable Blindspot warning vibrator"),
                                            tr("Play blindspot warning vibrator"),
                                            "");
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagBlindspotWarningVibratorEnabled, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagBlindspotWarningVibratorEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBlindspotWarningVibratorEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagBlindspotWarningVibratorEnabled);

  mParamControlIsVagBlindspotVibratorWithFlka = new ParamControl("IsVagBlindspotVibratorWithFlka",
                                            tr("  Enable vibrator with FLKA"),
                                            tr("Vibration may cause lower FLKA limits"),
                                            "");
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagBlindspotVibratorWithFlka, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagBlindspotVibratorWithFlka, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagBlindspotVibratorWithFlka(bool)));
  mVBoxList->addWidget(mParamControlIsVagBlindspotVibratorWithFlka);

  // ----- FLKA -----
  const bool isVagDevelopModeEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagDevelopModeEnabled();
  if(getDongleId().has_value() || isVagDevelopModeEnabled) {
    //mVBoxList->addWidget(horizontal_line());
  }

  mParamControlIsVagFulltimeLkaEnabled = new ParamControl("IsVagFulltimeLkaEnabled",
                                            tr("Enable Fulltime LKA"),
                                            tr("Fulltime enable LKA without ACC engaged"),
                                            "");
  QObject::connect(mParamControlIsVagFulltimeLkaEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagFulltimeLkaEnabled(bool)));
  if(getDongleId().has_value() || isVagDevelopModeEnabled) {
    mVBoxList->addWidget(mParamControlIsVagFulltimeLkaEnabled);
  }

  mParamControlIsVagFulltimeLkaEnableWithBlinker = new ParamControl("IsVagFulltimeLkaEnableWithBlinker",
                                            tr("  Enable Fulltime LKA with blinker"),
                                            tr("Enable Fulltime LKA with blinker"),
                                            "");
  QObject::connect(mParamControlIsVagFulltimeLkaEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagFulltimeLkaEnableWithBlinker, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagFulltimeLkaEnableWithBlinker, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagFulltimeLkaEnableWithBlinker(bool)));
  if(getDongleId().has_value() || isVagDevelopModeEnabled) {
    mVBoxList->addWidget(mParamControlIsVagFulltimeLkaEnableWithBlinker);
  }

  mParamControlIsVagFulltimeLkaEnableWithBrake = new ParamControl("IsVagFulltimeLkaEnableWithBrake",
                                            tr("  Enable Fulltime LKA with brake"),
                                            tr("Enable Fulltime LKA with brake"),
                                            "");
  QObject::connect(mParamControlIsVagFulltimeLkaEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagFulltimeLkaEnableWithBrake, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagFulltimeLkaEnableWithBrake, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagFulltimeLkaEnableWithBrake(bool)));
  if(getDongleId().has_value() || isVagDevelopModeEnabled) {
    mVBoxList->addWidget(mParamControlIsVagFulltimeLkaEnableWithBrake);
  }

  mParamControlIsVagFulltimeLkaEnableWithAssistant = new ParamControl("IsVagFulltimeLkaEnableWithAssistant",
                                            tr("  Enable Fulltime LKA with Assistant"),
                                            tr("Enable Fulltime LKA with Assistant"),
                                            "");
  QObject::connect(mParamControlIsVagFulltimeLkaEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagFulltimeLkaEnableWithAssistant, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagFulltimeLkaEnableWithAssistant, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagFulltimeLkaEnableWithAssistant(bool)));
  if(getDongleId().has_value() || isVagDevelopModeEnabled) {
    mVBoxList->addWidget(mParamControlIsVagFulltimeLkaEnableWithAssistant);
  }

  //----- Lead car going remind -----
  //mVBoxList->addWidget(horizontal_line());
  mParamControlIsVagLeadCarGoingRemindEnabled = new ParamControl("IsVagLeadCarGoingRemindEnabled",
                                            tr("Enable Lead car going remind"),
                                            tr("Enable Lead car going remind (base on acc ready state)"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagLeadCarGoingRemindEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagLeadCarGoingRemindEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagLeadCarGoingRemindEnabled);

  mParamControlIsVagLeadCarGoingRemindSoundEnabled = new ParamControl("IsVagLeadCarGoingRemindSoundEnabled",
                                            tr("  Enable Lead car going remind sound"),
                                            tr("Enable Lead car going remind sound (base on acc ready state)"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagLeadCarGoingRemindEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagLeadCarGoingRemindSoundEnabled, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagLeadCarGoingRemindSoundEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagLeadCarGoingRemindSoundEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagLeadCarGoingRemindSoundEnabled);

  //----- No lead car -----
  //mVBoxList->addWidget(horizontal_line());
  mParamControlIsVagNoLeadCarEnabled = new ParamControl("IsVagNoLeadCarEnabled",
                                            tr("Enable No lead car warning"),
                                            tr("Enable No lead car warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagNoLeadCarEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagNoLeadCarEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagNoLeadCarEnabled);

  mParamControlIsVagNoLeadCarWarningSoundEnabled = new ParamControl("IsVagNoLeadCarWarningSoundEnabled",
                                            tr("  Enable No lead car warning sound"),
                                            tr("Enable No lead car warning sound"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagNoLeadCarEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagNoLeadCarWarningSoundEnabled, SLOT(setToggleVisible(bool)));
  QObject::connect(mParamControlIsVagNoLeadCarWarningSoundEnabled, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagNoLeadCarWarningSoundEnabled(bool)));
  mVBoxList->addWidget(mParamControlIsVagNoLeadCarWarningSoundEnabled);

  // ----- Force disable startstop -----
  mParamControlIsVagForceDisableStartstop = new ParamControl("IsVagForceDisableStartstop",
                                            tr("Force disable startstop"),
                                            tr("Force disable startstop"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagForceDisableStartstop, SIGNAL(toggleFlipped(bool)), mVagParam,  SLOT(setIsVagForceDisableStartstop(bool)));
  mVBoxList->addWidget(mParamControlIsVagForceDisableStartstop);
}

void VagFeaturePanel::showEvent(QShowEvent *event) {
  // ----- Blindspot -----
  bool IsVagBlindspotEnabled = false;
  try {
    IsVagBlindspotEnabled = Params().getBool("IsVagBlindspotEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagBlindspotEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    IsVagBlindspotEnabled = false;
  }
  mParamControlIsVagBlindspotInfoSoundEnabled->setToggleVisible(IsVagBlindspotEnabled);
  mParamControlIsVagBlindspotInfoVibratorEnabled->setToggleVisible(IsVagBlindspotEnabled);
  mParamControlIsVagBlindspotWarningSoundEnabled->setToggleVisible(IsVagBlindspotEnabled);
  mParamControlIsVagBlindspotWarningVibratorEnabled->setToggleVisible(IsVagBlindspotEnabled);
  mParamControlIsVagBlindspotVibratorWithFlka->setToggleVisible(IsVagBlindspotEnabled);

  // ----- FLKA -----
  bool IsVagFulltimeLkaEnabled = false;
  try {
    IsVagFulltimeLkaEnabled = Params().getBool("IsVagFulltimeLkaEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagFulltimeLkaEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    IsVagFulltimeLkaEnabled = false;
  }
  mParamControlIsVagFulltimeLkaEnableWithBlinker->setToggleVisible(IsVagFulltimeLkaEnabled);
  mParamControlIsVagFulltimeLkaEnableWithBrake->setToggleVisible(IsVagFulltimeLkaEnabled);
  mParamControlIsVagFulltimeLkaEnableWithAssistant->setToggleVisible(IsVagFulltimeLkaEnabled);

  // ----- Lead car going -----
  bool IsVagLeadCarGoingRemindEnabled = false;
  try {
    IsVagLeadCarGoingRemindEnabled = Params().getBool("IsVagLeadCarGoingRemindEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagLeadCarGoingRemindEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    IsVagLeadCarGoingRemindEnabled = false;
  }
  mParamControlIsVagLeadCarGoingRemindSoundEnabled->setToggleVisible(IsVagLeadCarGoingRemindEnabled);

  // ----- No lead car -----
  bool IsVagNoLeadCarEnabled = false;
  try {
    IsVagNoLeadCarEnabled = Params().getBool("IsVagNoLeadCarEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%s][%d][IsVagNoLeadCarEnabled] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
    IsVagNoLeadCarEnabled = false;
  }
  mParamControlIsVagNoLeadCarWarningSoundEnabled->setToggleVisible(IsVagNoLeadCarEnabled);

  UIState *s = uiState();
  const bool bus0Bcm01 = (*s->sm)["carParams"].getCarParams().getVagCarParams().getVagCanModule().getBus0Bcm01();
  const bool bus0Motor18 = (*s->sm)["carParams"].getCarParams().getVagCarParams().getVagCanModule().getBus0Motor18();
  if(bus0Bcm01 && bus0Motor18) {
    mParamControlIsVagForceDisableStartstop->setEnabled(true);
  } else {
    mParamControlIsVagForceDisableStartstop->setEnabled(false);
  }
}

VagWarningPanel::VagWarningPanel(QWidget* parent) : ListWidget(parent) {
  mVagParam = VagParam::getInstance();
  mVBoxList = new QVBoxLayout();
  mVBoxList->setSpacing(30);
  addItem(mVBoxList);

  mParamControlIsVagWarningEngineTurboPressure = new ParamControl("IsVagWarningEngineTurboPressure",
                                            tr("Turbo pressure warning"),
                                            tr("Turbo pressure warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningEngineTurboPressure, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningEngineTurboPressure(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningEngineTurboPressure);

  mParamControlIsVagWarningEngineOilPressure = new ParamControl("IsVagWarningEngineOilPressure",
                                            tr("Engine pressure warning"),
                                            tr("Engine pressure warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningEngineOilPressure, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningEngineOilPressure(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningEngineOilPressure);

  mParamControlIsVagWarningEngineInAirTemperature = new ParamControl("IsVagWarningEngineInAirTemperature",
                                            tr("Engine in air warning"),
                                            tr("Engine in air warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningEngineInAirTemperature, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningEngineInAirTemperature(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningEngineInAirTemperature);

  mParamControlIsVagWarningEngineCoolantTemperature = new ParamControl("IsVagWarningEngineCoolantTemperature",
                                            tr("Engine coolant temperature warning"),
                                            tr("Engine coolant temperature warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningEngineCoolantTemperature, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningEngineCoolantTemperature(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningEngineCoolantTemperature);

  mParamControlIsVagWarningEngineOilTemperature = new ParamControl("IsVagWarningEngineOilTemperature",
                                            tr("Engine oil temperature warning"),
                                            tr("Engine oil temperature warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningEngineOilTemperature, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningEngineOilTemperature(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningEngineOilTemperature);

  mParamControlIsVagWarningCoolantTemperature = new ParamControl("IsVagWarningCoolantTemperature",
                                            tr("Coolant temperature warning"),
                                            tr("Coolant temperature warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningCoolantTemperature, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningCoolantTemperature(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningCoolantTemperature);

  mParamControlIsVagWarningInAirPressure = new ParamControl("IsVagWarningInAirPressure",
                                            tr("Engine air pressure warning"),
                                            tr("Engine air pressure warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningInAirPressure, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningInAirPressure(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningInAirPressure);

  mParamControlIsVagWarningGearOilTemperature = new ParamControl("IsVagWarningGearOilTemperature",
                                            tr("Gear oil temperature warning"),
                                            tr("Gear oil temperature warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningGearOilTemperature, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningGearOilTemperature(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningGearOilTemperature);

  mParamControlIsVagWarningBrakePressure = new ParamControl("IsVagWarningBrakePressure",
                                            tr("Brake pressure warning"),
                                            tr("Brake pressure warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningBrakePressure, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningBrakePressure(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningBrakePressure);

  mParamControlIsVagWarningIndoorTemperature = new ParamControl("IsVagWarningIndoorTemperature",
                                            tr("Indoor temperature warning"),
                                            tr("Indoor temperature warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningIndoorTemperature, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningIndoorTemperature(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningIndoorTemperature);

  mParamControlIsVagWarningOutdoorTemperature = new ParamControl("IsVagWarningOutdoorTemperature",
                                            tr("Outdoor temperature warning"),
                                            tr("Outdoor temperature warning"),
                                            ""
                                            );
  QObject::connect(mParamControlIsVagWarningOutdoorTemperature, SIGNAL(toggleFlipped(bool)), mVagParam, SLOT(setIsVagWarningOutdoorTemperature(bool)));
  mVBoxList->addWidget(mParamControlIsVagWarningOutdoorTemperature);

}

VagDebugWindow::VagDebugWindow(QWidget *parent) : QFrame(parent) {

  // setup two main layouts
  sidebar_widget = new QWidget;
  QVBoxLayout *sidebar_layout = new QVBoxLayout(sidebar_widget);
  sidebar_layout->setMargin(0);
  panel_widget = new QStackedWidget();
  panel_widget->setStyleSheet(R"(
    border-radius: 30px;
    background-color: #292929;
  )");

  // close button
  QPushButton *close_btn = new QPushButton("X");
  close_btn->setStyleSheet(R"(
    QPushButton {
      font-size: 140px;
      padding-bottom: 20px;
      font-weight: bold;
      border 1px grey solid;
      border-radius: 100px;
      background-color: #292929;
      font-weight: 400;
    }
    QPushButton:pressed {
      background-color: #3B3B3B;
    }
  )");
  close_btn->setFixedSize(200, 200);
  sidebar_layout->addSpacing(45);
  sidebar_layout->addWidget(close_btn, 0, Qt::AlignCenter);
  QObject::connect(close_btn, &QPushButton::clicked, this, &VagDebugWindow::closeVagDebug);

  mVagSettingPanel = new VagSettingPanel(this);
  mVagFeaturePanel = new VagFeaturePanel(this);
  mVagTestPanel = new VagTestPanel(this);
  mVagGeneralPanel = new VagGeneralPanel(this);
  mVagOsdPanel = new VagOsdPanel(this);
  mVagWarningPanel = new VagWarningPanel(this);

  QList<QPair<QString, QWidget *>> panels = {
    {tr("General"), mVagGeneralPanel},
    {tr("OSD"), mVagOsdPanel},
    {tr("Test"), mVagTestPanel},
    {tr("Setting"), mVagSettingPanel},
    {tr("Feature"), mVagFeaturePanel},
    {tr("Warning"), mVagWarningPanel},
  };

  const int padding = 15;

  nav_btns = new QButtonGroup(this);
  for (auto &[name, panel] : panels) {
    QPushButton *btn = new QPushButton(name);
    btn->setCheckable(true);
    btn->setChecked(nav_btns->buttons().size() == 0);
    btn->setStyleSheet(QString(R"(
      QPushButton {
        color: grey;
        border: none;
        background: none;
        font-size: 65px;
        font-weight: 500;
        font-height: 100;
        font-height: 70;
        padding-top: %1px;
        padding-bottom: 0px;
      }
      QPushButton:checked {
        color: white;
      }
      QPushButton:pressed {
        color: #ADADAD;
      }
    )").arg(padding));

    nav_btns->addButton(btn);
    sidebar_layout->addWidget(btn, 0, Qt::AlignLeft);

    const int lr_margin = name != "Network" ? 50 : 0;  // Network panel handles its own margins
    panel->setContentsMargins(lr_margin, 25, lr_margin, 25);

    ScrollView *panel_frame = new ScrollView(panel, this);
    panel_widget->addWidget(panel_frame);

    QObject::connect(btn, &QPushButton::clicked, [=, w = panel_frame]() {
      btn->setChecked(true);
      panel_widget->setCurrentWidget(w);
    });
  }
  sidebar_layout->setContentsMargins(50, 50, 100, 50);

  // main settings layout, sidebar + main panel
  QHBoxLayout *main_layout = new QHBoxLayout(this);

  sidebar_widget->setFixedWidth(500);
  main_layout->addWidget(sidebar_widget);
  main_layout->addWidget(panel_widget);

  setStyleSheet(R"(
    * {
      color: white;
      font-size: 50px;
    }
    VagDebugWindow {
      background-color: black;
    }
  )");
}

void VagDebugWindow::showEvent(QShowEvent *event) {
  panel_widget->setCurrentIndex(0);
  nav_btns->buttons()[0]->setChecked(true);
}
