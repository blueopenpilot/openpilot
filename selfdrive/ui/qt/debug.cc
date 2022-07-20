/*
 * Copyright (c) 2020-2022 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "selfdrive/ui/qt/debug.h"

#include <cassert>
#include <cmath>
#include <string>

#include <QDebug>

#ifndef QCOM
#include "selfdrive/ui/qt/offroad/networking.h"
#endif

#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_settings.h"
#endif

#include "selfdrive/common/params.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/input.h"
#include "selfdrive/ui/qt/widgets/scrollview.h"
#include "selfdrive/ui/qt/widgets/ssh_keys.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"


//===== VAG settings =====
void DebugSettingPanel::setVolume(int volume) {
  QString stringVolume = QString::number(volume);
  try {
  Params().put("VagSoundVolume", stringVolume.toStdString());
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][VagSoundVolume] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
  }
}

void DebugSettingPanel::setBacklight(int backlight) {
  bool IsVagManualOsdBacklightEnable = false;
  try {
    IsVagManualOsdBacklightEnable = Params().getBool("IsVagManualOsdBacklightEnable");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagManualOsdBacklightEnable] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagManualOsdBacklightEnable = false;
  }
  QString stringBacklight = QString::number(backlight);
  try {
    Params().put("VagOsdBacklight", stringBacklight.toStdString());
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][VagOsdBacklight] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
  }
  if(IsVagManualOsdBacklightEnable) {
    Hardware::set_brightness(backlight*backlight);
  }
}

void DebugSettingPanel::setAutoShowdownMinutes(int minutes) {
  QString stringMinutes = QString::number(minutes);
  Params().put("VagAutoshutdownMinutes", stringMinutes.toStdString());
}

void DebugSettingPanel::showEvent(QShowEvent *event) {
#if 0
  bool IsVagManualSoundVolumeEnable = false;
  try {
    IsVagManualSoundVolumeEnable = Params().getBool("IsVagManualSoundVolumeEnable");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagManualSoundVolumeEnable] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagManualSoundVolumeEnable = false;
  }
  mSliderVagSoundVolume->setVisible(IsVagManualSoundVolumeEnable);
  mLabelVagSoundVolumeValue->setVisible(IsVagManualSoundVolumeEnable);

  bool IsVagManualOsdBacklightEnable = false;
  try {
    IsVagManualOsdBacklightEnable = Params().getBool("IsVagManualOsdBacklightEnable");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagManualOsdBacklightEnable] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagManualOsdBacklightEnable = false;
  }
  mSliderVagOsdBacklight->setVisible(IsVagManualOsdBacklightEnable);
  mLabelVagOsdBacklightValue->setVisible(IsVagManualOsdBacklightEnable);
#endif
}

DebugSettingPanel::DebugSettingPanel(QWidget* parent) : ListWidget(parent) {
  mVBoxVagSettingList = new QVBoxLayout();
  mVBoxVagSettingList->setSpacing(30);
  addItem(mVBoxVagSettingList);

  //===== Maunal sound volume =====
  QString VagSoundVolume = "10";
  try {
    VagSoundVolume = QString::fromStdString(Params().get("VagSoundVolume"));
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][VagSoundVolume] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    VagSoundVolume = "10";
  }
  mParamControlIsVagManualSoundVolumeEnable = new ParamControl("IsVagManualSoundVolumeEnable",
                                            "Enable manual sound volume",
                                            "Enable manual sound volume",
                                            ""
                                            );
  mVBoxVagSettingList->addWidget(mParamControlIsVagManualSoundVolumeEnable);

  mHBoxVagSoundVolume = new QHBoxLayout();
  mLabelVagSoundVolumeTitle = new QLabel();
  mSliderVagSoundVolume = new QSlider();
  mLabelVagSoundVolumeValue = new QLabel();
#if 0
  QObject::connect(mParamControlIsVagManualSoundVolumeEnable, SIGNAL(toggleFlipped(bool)), mSliderVagSoundVolume, SLOT(setVisible(bool)));
  QObject::connect(mParamControlIsVagManualSoundVolumeEnable, SIGNAL(toggleFlipped(bool)), mLabelVagSoundVolumeValue, SLOT(setVisible(bool)));
#endif
  QObject::connect(mSliderVagSoundVolume, &QSlider::valueChanged, this, &DebugSettingPanel::setVolume);
  QObject::connect(mSliderVagSoundVolume, SIGNAL(valueChanged(int)), mLabelVagSoundVolumeValue, SLOT(setNum(int)));
  mLabelVagSoundVolumeTitle->setText("  Volume   ");
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
  mVBoxVagSettingList->addLayout(mHBoxVagSoundVolume);
  mVBoxVagSettingList->addWidget(horizontal_line());

  //===== Maunal osd backlight =====
  QString VagOsdBacklight = QString::fromStdString(Params().get("VagOsdBacklight"));
  mParamControlIsVagManualOsdBacklightEnable = new ParamControl("IsVagManualOsdBacklightEnable",
                                            "Enable manual OSD Backlight",
                                            "Enable manual OSD Backlight",
                                            ""
                                            );
  mVBoxVagSettingList->addWidget(mParamControlIsVagManualOsdBacklightEnable);

  mHBoxVagOsdBacklight = new QHBoxLayout();
  mLabelVagOsdBacklightTitle = new QLabel();
  mSliderVagOsdBacklight = new QSlider();
  mLabelVagOsdBacklightValue = new QLabel();
#if 0
  QObject::connect(mParamControlIsVagManualOsdBacklightEnable, SIGNAL(toggleFlipped(bool)), mSliderVagOsdBacklight, SLOT(setVisible(bool)));
  QObject::connect(mParamControlIsVagManualOsdBacklightEnable, SIGNAL(toggleFlipped(bool)), mLabelVagOsdBacklightValue, SLOT(setVisible(bool)));
#endif
  QObject::connect(mSliderVagOsdBacklight, &QSlider::valueChanged, this, &DebugSettingPanel::setBacklight);
  QObject::connect(mSliderVagOsdBacklight, SIGNAL(valueChanged(int)), mLabelVagOsdBacklightValue, SLOT(setNum(int)));
  mLabelVagOsdBacklightTitle->setText("  Backlight");
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
  mVBoxVagSettingList->addLayout(mHBoxVagOsdBacklight);

  //===== Info Box =====
  mVBoxVagSettingList->addWidget(horizontal_line());
  mParamControlIsVagInfoBoxEnabled = new ParamControl("IsVagInfoBoxEnabled",
                                            "Enable Info Box",
                                            "Show info box on the screen",
                                            "");
  mVBoxVagSettingList->addWidget(mParamControlIsVagInfoBoxEnabled);

  //===== Blink =====
  mVBoxVagSettingList->addWidget(horizontal_line());
  mParamControlIsVagBlinkerEnabled = new ParamControl("IsVagBlinkerEnabled",
                                            "Enable Blinker",
                                            "Show blinker on the screen",
                                            "");
  mVBoxVagSettingList->addWidget(mParamControlIsVagBlinkerEnabled);

  //===== Brake light =====
  mVBoxVagSettingList->addWidget(horizontal_line());
  mParamControlIsVagBrakeLightEnabled = new ParamControl("IsVagBrakeLightEnabled",
                                            "Enable Brake Light",
                                            "Show brake light on the view screen",
                                            "");
  mVBoxVagSettingList->addWidget(mParamControlIsVagBrakeLightEnabled);

  //===== Lead car =====
  mVBoxVagSettingList->addWidget(horizontal_line());
  mParamControlIsVagLeadCarEnabled = new ParamControl("IsVagLeadCarEnabled",
                                            "Enable Lead Car",
                                            "Show lead car on the view screen",
                                            "");
  mVBoxVagSettingList->addWidget(mParamControlIsVagLeadCarEnabled);

#if 0
  //===== Auto showdown =====
  mVBoxVagSettingList->addWidget(horizontal_line());
  QString VagAutoShutdownMinutes = QString::fromStdString(Params().get("VagAutoShutdownMinutes"));
  mHBoxVagAutoShutdown = new QHBoxLayout();
  mLabelVagAutoShutdownTitle = new QLabel();
  mSliderVagAutoShutdown = new QSlider();
  mLabelVagAutoShutdownValue = new QLabel();
  mLabelVagAutoShutdownUnit = new QLabel();
  QObject::connect(mSliderVagAutoShutdown, &QSlider::valueChanged, this, &DebugSettingPanel::setAutoShowdownMinutes);
  QObject::connect(mSliderVagAutoShutdown, SIGNAL(valueChanged(int)), mLabelVagAutoShutdownValue, SLOT(setNum(int)));
  mLabelVagAutoShutdownTitle->setText("Auto shutdown");
  mLabelVagAutoShutdownTitle->setStyleSheet( "QLabel {height: 100px; padding-top: 15px; padding-bottom: 15px;}");
  mSliderVagAutoShutdown->setStyleSheet( "QSlider::horizontal {background: #FFFFFF; height: 90px;}"
                               "QSlider::groove:horizontal {background: #FFFFFF;}"
                               "QSlider::handle:horizontal {width: 100px; background: #444444;}" );
  mSliderVagAutoShutdown->setOrientation(Qt::Horizontal);
  mSliderVagAutoShutdown->setRange(5, 30);
  mSliderVagAutoShutdown->sliderMoved(5);
  mSliderVagAutoShutdown->setTickInterval(5);
  mSliderVagAutoShutdown->setSingleStep(5);
  mSliderVagAutoShutdown->setPageStep(5);
  mSliderVagAutoShutdown->setValue(VagAutoShutdownMinutes.toInt());
  mLabelVagAutoShutdownUnit->setText("Minutes");
  mLabelVagAutoShutdownUnit->setStyleSheet( "QLabel {height: 100px; padding-top: 15px; padding-bottom: 15px;}");
  mHBoxVagAutoShutdown->addWidget(mLabelVagAutoShutdownTitle);
  mHBoxVagAutoShutdown->addWidget(mSliderVagAutoShutdown);
  mHBoxVagAutoShutdown->addWidget(mLabelVagAutoShutdownValue);
  mHBoxVagAutoShutdown->addWidget(mLabelVagAutoShutdownUnit);
  mVBoxVagSettingList->addLayout(mHBoxVagAutoShutdown);
#endif

#ifdef QCOM
  mVBoxVagSettingList->addWidget(horizontal_line());
  auto androidSettingsBtn = new ButtonControl("Android Settings", "OPEN");
  QObject::connect(androidSettingsBtn, &ButtonControl::clicked, [=]() { HardwareEon::launch_settings(); });
  mVBoxVagSettingList->addWidget(androidSettingsBtn);
#endif
}

void DebugFeaturePanel::showEvent(QShowEvent *event) {
  //----- Blindspot -----
  bool IsVagBlindspotEnabled = false;
  try {
    IsVagBlindspotEnabled = Params().getBool("IsVagBlindspotEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagBlindspotEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagBlindspotEnabled = false;
  }
#if 0
  mQComboBoxVagBlindspotOsdSize->setVisible(IsVagBlindspotEnabled);
#endif
  mParamControlIsVagBlindspotWarningSoundEnabled->setToggleVisible(IsVagBlindspotEnabled);

  //----- FLKA -----
  bool IsVagFulltimeLkaEnabled = false;
  try {
    IsVagFulltimeLkaEnabled = Params().getBool("IsVagFulltimeLkaEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagFulltimeLkaEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagFulltimeLkaEnabled = false;
  }
  mParamControlIsVagFulltimeLkaEnableWithBlinker->setToggleVisible(IsVagFulltimeLkaEnabled);
  mParamControlIsVagFulltimeLkaEnableWithBrake->setToggleVisible(IsVagFulltimeLkaEnabled);

  //----- Lead car going -----
  bool IsVagLeadCarGoingRemindEnabled = false;
  try {
    IsVagLeadCarGoingRemindEnabled = Params().getBool("IsVagLeadCarGoingRemindEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagLeadCarGoingRemindEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagLeadCarGoingRemindEnabled = false;
  }
  mParamControlIsVagLeadCarGoingRemindSoundEnabled->setToggleVisible(IsVagLeadCarGoingRemindEnabled);

  //----- No lead car -----
  bool IsVagNoLeadCarEnabled = false;
  try {
    IsVagNoLeadCarEnabled = Params().getBool("IsVagNoLeadCarEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagNoLeadCarEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagNoLeadCarEnabled = false;
  }
  mParamControlIsVagNoLeadCarWarningSoundEnabled->setToggleVisible(IsVagNoLeadCarEnabled);

  //----- Speed cam -----
  bool IsVagSpeedCameraEnabled = false;
  try {
    IsVagSpeedCameraEnabled = Params().getBool("IsVagSpeedCameraEnabled");
  } catch (std::exception &e) {
    printf("[BOP][%s][%d][%s()][IsVagSpeedCameraEnabled] Get param exception: %s \n", __FILE__, __LINE__, __FUNCTION__, e.what());
    IsVagSpeedCameraEnabled = false;
  }
  mParamControlIsVagSpeedLimitSoundEnabled->setToggleVisible(IsVagSpeedCameraEnabled);
#if 0
  mParamControlIsVagSaccEnabled->setToggleVisible(IsVagSpeedCameraEnabled);
#endif
}

DebugFeaturePanel::DebugFeaturePanel(QWidget* parent) : ListWidget(parent) {
  mVBoxVagFeatureList = new QVBoxLayout();
  mVBoxVagFeatureList->setSpacing(30);
  addItem(mVBoxVagFeatureList);

  //===== Blindspot =====
  mParamControlIsVagBlindspotEnabled = new ParamControl("IsVagBlindspotEnabled",
                                            "Enable Blindspot",
                                            "Show blindspot on the view screen",
                                            "");
  mVBoxVagFeatureList->addWidget(mParamControlIsVagBlindspotEnabled);

#if 0
  mHBoxVagBlindspotOsdSize = new QHBoxLayout();
  mLabelVagBlindspotOsdSize = new QLabel();
  mQComboBoxVagBlindspotOsdSize = new QComboBox();
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mQComboBoxVagBlindspotOsdSize, SLOT(setVisible(bool)));
  mLabelVagBlindspotOsdSize->setText("  Blindspot size");
  mQComboBoxVagBlindspotOsdSize->addItem("Small", 200);
  mQComboBoxVagBlindspotOsdSize->addItem("Medium", 300);
  mQComboBoxVagBlindspotOsdSize->addItem("Large", 400);
  mLabelVagBlindspotOsdSize->setStyleSheet( "QLabel {height: 100px; padding-top: 15px; padding-bottom: 15px;}");
  mQComboBoxVagBlindspotOsdSize->setStyleSheet( "QComboBox {background: #444444; border-radius: 0px; width: 1000; height: 90px;}"
                                                "QComboBox::drop-down {border: 0px;}"
                                                );
  mHBoxVagBlindspotOsdSize->addWidget(mLabelVagBlindspotOsdSize);
  mHBoxVagBlindspotOsdSize->addWidget(mQComboBoxVagBlindspotOsdSize);
  mVBoxVagFeatureList->addLayout(mHBoxVagBlindspotOsdSize);
#endif
  mParamControlIsVagBlindspotWarningSoundEnabled = new ParamControl("IsVagBlindspotWarningSoundEnabled",
                                            "  Enable Blindspot warning sound",
                                            "Play blindspot warning sound",
                                            "");
  QObject::connect(mParamControlIsVagBlindspotEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagBlindspotWarningSoundEnabled, SLOT(setToggleVisible(bool)));
  mVBoxVagFeatureList->addWidget(mParamControlIsVagBlindspotWarningSoundEnabled);

  //===== FLKA =====
  bool IsVagDevelopModeEnabled = false;
  try {
    IsVagDevelopModeEnabled = Params().getBool("IsVagDevelopModeEnabled");
  } catch (std::exception &e) {
    printf("[BOP][IsVagDevelopModeEnabled][settings.cc] Get param exception: %s \n", e.what());
    IsVagDevelopModeEnabled = false;
  }

  if(getDongleId().has_value() || IsVagDevelopModeEnabled) {
    mVBoxVagFeatureList->addWidget(horizontal_line());
  }

  mParamControlIsVagFulltimeLkaEnabled = new ParamControl("IsVagFulltimeLkaEnabled",
                                            "Enable Fulltime LKA",
                                            "Fulltime enable LKA without ACC engaged",
                                            "");
  if(getDongleId().has_value() || IsVagDevelopModeEnabled) {
    mVBoxVagFeatureList->addWidget(mParamControlIsVagFulltimeLkaEnabled);
  }

  mParamControlIsVagFulltimeLkaEnableWithBlinker = new ParamControl("IsVagFulltimeLkaEnableWithBlinker",
                                            "  Enable Fulltime LKA with blinker",
                                            "Enable Fulltime LKA with blinker",
                                            "");
  QObject::connect(mParamControlIsVagFulltimeLkaEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagFulltimeLkaEnableWithBlinker, SLOT(setToggleVisible(bool)));
  if(getDongleId().has_value() || IsVagDevelopModeEnabled) {
    mVBoxVagFeatureList->addWidget(mParamControlIsVagFulltimeLkaEnableWithBlinker);
  }

  mParamControlIsVagFulltimeLkaEnableWithBrake = new ParamControl("IsVagFulltimeLkaEnableWithBrake",
                                            "  Enable Fulltime LKA with brake",
                                            "Enable Fulltime LKA with brake",
                                            "");
  QObject::connect(mParamControlIsVagFulltimeLkaEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagFulltimeLkaEnableWithBrake, SLOT(setToggleVisible(bool)));
  if(getDongleId().has_value() || IsVagDevelopModeEnabled) {
    mVBoxVagFeatureList->addWidget(mParamControlIsVagFulltimeLkaEnableWithBrake);
  }

  //----- Lead car going remind -----
  mVBoxVagFeatureList->addWidget(horizontal_line());
  mParamControlIsVagLeadCarGoingRemindEnabled = new ParamControl("IsVagLeadCarGoingRemindEnabled",
                                            "Enable Lead car going remind (Alpha)",
                                            "Enable Lead car going remind (base on acc ready state)",
                                            ""
                                            );
  mVBoxVagFeatureList->addWidget(mParamControlIsVagLeadCarGoingRemindEnabled);

  mParamControlIsVagLeadCarGoingRemindSoundEnabled = new ParamControl("IsVagLeadCarGoingRemindSoundEnabled",
                                            "  Enable Lead car going remind sound",
                                            "Enable Lead car going remind sound",
                                            ""
                                            );
  QObject::connect(mParamControlIsVagLeadCarGoingRemindEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagLeadCarGoingRemindSoundEnabled, SLOT(setToggleVisible(bool)));
  mVBoxVagFeatureList->addWidget(mParamControlIsVagLeadCarGoingRemindSoundEnabled);

  //----- No lead car -----
  mVBoxVagFeatureList->addWidget(horizontal_line());
  mParamControlIsVagNoLeadCarEnabled = new ParamControl("IsVagNoLeadCarEnabled",
                                            "Enable No lead car warning (Alpha)",
                                            "Enable No lead car warning",
                                            ""
                                            );
  mVBoxVagFeatureList->addWidget(mParamControlIsVagNoLeadCarEnabled);

  mParamControlIsVagNoLeadCarWarningSoundEnabled = new ParamControl("IsVagNoLeadCarWarningSoundEnabled",
                                            "  Enable No lead car warning sound",
                                            "Enable No lead car warning sound",
                                            ""
                                            );
  QObject::connect(mParamControlIsVagNoLeadCarEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagNoLeadCarWarningSoundEnabled, SLOT(setToggleVisible(bool)));
  mVBoxVagFeatureList->addWidget(mParamControlIsVagNoLeadCarWarningSoundEnabled);

  //----- Speed Camera -----
  mVBoxVagFeatureList->addWidget(horizontal_line());
  mParamControlIsVagSpeedCameraEnabled = new ParamControl("IsVagSpeedCameraEnabled",
                                            "Enable Speed Camera (Alpha)",
                                            "Speed camera limitation warnning",
                                            "");
  mVBoxVagFeatureList->addWidget(mParamControlIsVagSpeedCameraEnabled);

  mParamControlIsVagSpeedLimitSoundEnabled = new ParamControl("IsVagSpeedLimitSoundEnabled",
                                            " Enable Speed Limit Sound",
                                            "Enable speed litmit sound warning",
                                            "");
  QObject::connect(mParamControlIsVagSpeedCameraEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagSpeedLimitSoundEnabled, SLOT(setToggleVisible(bool)));
  mVBoxVagFeatureList->addWidget(mParamControlIsVagSpeedLimitSoundEnabled);
#if 0
  mParamControlIsVagSaccEnabled = new ParamControl("IsVagSaccEnabled",
                                            " Enable SACC (Alpha)",
                                            "Enable Speedcamera Adaptive Cruise Control",
                                            "");
  QObject::connect(mParamControlIsVagSpeedCameraEnabled, SIGNAL(toggleFlipped(bool)), mParamControlIsVagSaccEnabled, SLOT(setToggleVisible(bool)));
  mVBoxVagFeatureList->addWidget(mParamControlIsVagSaccEnabled);
#endif
}

DebugTestPanel::DebugTestPanel(QWidget* parent) : ListWidget(parent) {
  mVBoxDebugList = new QVBoxLayout();
  mVBoxDebugList->setSpacing(30);
  addItem(mVBoxDebugList);

  auto resetCalibBtn = new ButtonControl("Reset Calibration", "RESET", " ");
  connect(resetCalibBtn, &ButtonControl::clicked, [&]() {
    if (ConfirmationDialog::confirm("Are you sure you want to reset calibration?", this)) {
      Params().remove("CalibrationParams");
    }
  });
  addItem(resetCalibBtn);

  //----- OSD Test Text -----
  mParamControlOsdTestText = new ParamControl("IsVagDebugOsdTestTextEnabled",
                                            "Enable OSD test text",
                                            "Enable OSD test text",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlOsdTestText);

  //----- Debug Item 1 -----
  mParamControlDebugItem1 = new ParamControl("IsVagDebugItem1Enabled",
                                            "Debug Item 1",
                                            "Debug Item 1",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlDebugItem1);

  //----- Debug Item 2 -----
  mParamControlDebugItem2 = new ParamControl("IsVagDebugItem2Enabled",
                                            "Debug Item 2",
                                            "Debug Item 2",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlDebugItem2);

  //----- Debug Item 3 -----
  mParamControlDebugItem3 = new ParamControl("IsVagDebugItem3Enabled",
                                            "Debug Item 3",
                                            "Debug Item 3",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlDebugItem3);

  //----- Debug Item 4 -----
  mParamControlDebugItem4 = new ParamControl("IsVagDebugItem4Enabled",
                                            "Debug Item 4",
                                            "Debug Item 4",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlDebugItem4);

  //----- Debug Item 5 -----
  mParamControlDebugItem5 = new ParamControl("IsVagDebugItem5Enabled",
                                            "Debug Item 5",
                                            "Debug Item 5",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlDebugItem5);
}

DebugGeneralPanel::DebugGeneralPanel(QWidget* parent) : ListWidget(parent) {
  mVBoxDebugList = new QVBoxLayout();
  mVBoxDebugList->setSpacing(30);
  addItem(mVBoxDebugList);

  mParamControlDevelopMode = new ParamControl("IsVagDevelopModeEnabled",
                                            "Enable develop mode",
                                            "Enable develop mode",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlDevelopMode);

  mParamControlDevelopOnRoadUi = new ParamControl("IsVagDevelopOnRoadUi",
                                            "Enable on road screen for develop",
                                            "Enable on road screen for develop",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlDevelopOnRoadUi);

  mParamControlRunningProcessLog = new ParamControl("IsVagRunningProcessLogEnabled",
                                            "Enable running process log",
                                            "Enable running process log",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlRunningProcessLog);
}

DebugOsdPanel::DebugOsdPanel(QWidget* parent) : ListWidget(parent) {
  mVBoxDebugList = new QVBoxLayout();
  mVBoxDebugList->setSpacing(30);
  addItem(mVBoxDebugList);

#if 0
  //----- InfoBox Test -----
  mParamControlInfoBox = new ParamControl("IsVagDebugInfoBoxTest",
                                            "Info Box OSD test",
                                            "Info Box OSD test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlInfoBox);

  //----- Infobar Test -----
  mParamControlInfobar = new ParamControl("IsVagDebugInfobarTest",
                                            "Infobar OSD test",
                                            "Infobar OSD test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlInfobar);
#endif

  //----- Blinker Test -----
  mParamControlBlinker = new ParamControl("IsVagDebugBlinkerTest",
                                            "Blinker OSD test",
                                            "Blinker OSD test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlBlinker);

  //----- Blindspot Info Test -----
  mParamControlBlindspotInfo = new ParamControl("IsVagDebugBlindspotInfoTest",
                                            "Blindspot Info OSD test",
                                            "Blindspot Info OSD test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlBlindspotInfo);

  //----- Blindspot Warning Test -----
  mParamControlBlindspotWarning = new ParamControl("IsVagDebugBlindspotWarningTest",
                                            "Blindspot Warning OSD test",
                                            "Blindspot Warning OSD test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlBlindspotWarning);

  //----- Brake Light Test -----
  mParamControlBrakeLight = new ParamControl("IsVagDebugBrakeLightTest",
                                            "Brake Light test",
                                            "Brake Light test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlBrakeLight);

  //----- Lead Car Going Remind Test -----
  mParamControlLeadCarGoingRemind = new ParamControl("IsVagDebugLeadCarGoingRemindTest",
                                            "Lear Car Going Remind test",
                                            "Lear Car Going Remind test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlLeadCarGoingRemind);

  //----- No Lead Car Warning Test -----
  mParamControlNoLeadCarWarning = new ParamControl("IsVagDebugNoLeadCarWarningTest",
                                            "No Lead Car Warning test",
                                            "No Lead Car Warning test",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlNoLeadCarWarning);

}

DebugLogPanel::DebugLogPanel(QWidget* parent) : ListWidget(parent) {
  mVBoxDebugList = new QVBoxLayout();
  mVBoxDebugList->setSpacing(30);
  addItem(mVBoxDebugList);

  //----- FLKA Log -----
  mParamControlFlkaLog = new ParamControl("IsVagFlkaLogEnabled",
                                            "Enable FLKA log",
                                            "Enable Fulltime LKA debug log",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlFlkaLog);

  //----- SACC Log -----
  mParamControlSaccLog = new ParamControl("IsVagSaccLogEnabled",
                                            "Enable SACC log",
                                            "Enable SACC debug log",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlSaccLog);

  //----- SACC SpeedCamera Track -----
  mParamControlSaccSpeedCamTrack = new ParamControl("IsVagSaccSpeedCamTrackEnabled",
                                            "Enable recording speed camera track",
                                            "Enable recording speed camera track",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlSaccSpeedCamTrack);

  //----- SACC Vehicle Track -----
  mParamControlSaccVehicleTrack = new ParamControl("IsVagSaccVehicleTrackEnabled",
                                            "Enable recording vehicle track",
                                            "Enable recording vehicle track",
                                            ""
                                            );
  mVBoxDebugList->addWidget(mParamControlSaccVehicleTrack);
}

void DebugWindow::showEvent(QShowEvent *event) {
  panel_widget->setCurrentIndex(0);
  nav_btns->buttons()[0]->setChecked(true);
}

DebugWindow::DebugWindow(QWidget *parent) : QFrame(parent) {

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
  QObject::connect(close_btn, &QPushButton::clicked, this, &DebugWindow::closeDebug);

  mDebugSettingPanel = new DebugSettingPanel(this);
  mDebugFeaturePanel = new DebugFeaturePanel(this);
  mDebugTestPanel = new DebugTestPanel(this);
  mDebugGeneralPanel = new DebugGeneralPanel(this);
  mDebugOsdPanel = new DebugOsdPanel(this);
  mDebugLogPanel = new DebugLogPanel(this);

  QList<QPair<QString, QWidget *>> panels = {
    {"Setting", mDebugSettingPanel},
    {"Feature", mDebugFeaturePanel},
    {"Test", mDebugTestPanel},
    {"General", mDebugGeneralPanel},
    {"OSD", mDebugOsdPanel},
    {"Log", mDebugLogPanel},
  };

  const int padding = panels.size() > 3 ? 25 : 35;

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
        padding-top: 10px;
        padding-bottom: 10px;
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
    DebugWindow {
      background-color: black;
    }
  )");
}
