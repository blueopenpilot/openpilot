/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#pragma once

#include <QComboBox>
#include <QSlider>
#include <QButtonGroup>
#include <QFileSystemWatcher>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>

#include "selfdrive/ui/qt/vag_param.h"
#include "selfdrive/ui/qt/widgets/controls.h"

class VagGeneralPanel : public ListWidget {
  Q_OBJECT

public:
  explicit VagGeneralPanel(QWidget* parent = nullptr);

private:
  VagParam* mVagParam;
  QVBoxLayout* mVBoxList;

  ParamControl* mParamControlIsVagDisableDriverMonitorAlert;
  ParamControl* mParamControlIsVagLeftBlinkerSoundEnabled;
  ParamControl* mParamControlIsVagRightBlinkerSoundEnabled;
  ParamControl* mParamControlIsVagDevelopOnRoadUi;
  ParamControl* mParamControlIsVagPandaJungleEnabled;
  ParamControl* mParamControlIsVagDevelopModeEnabled;
  ParamControl* mParamControlIsVagRunningProcessLogEnabled;
  //ParamControl* mParamControlIsVagParamFromCerealEnabled;

public slots:

signals:

};

class VagOsdPanel : public ListWidget {
  Q_OBJECT

public:
  explicit VagOsdPanel(QWidget* parent = nullptr);

private:
  VagParam* mVagParam;
  QVBoxLayout* mVBoxList;

#if 0
  ParamControl* mParamControlIsVagDebugInfoBoxTest;
  ParamControl* mParamControlIsVagDebugInfobarTest;
#endif
  ParamControl* mParamControlIsVagDebugBlinkerTest;
  ParamControl* mParamControlIsVagDebugBlindspotInfoTest;
  ParamControl* mParamControlIsVagDebugBlindspotWarningTest;
  ParamControl* mParamControlIsVagDebugBrakeLightTest;
  ParamControl* mParamControlIsVagDebugLeadCarGoingRemindTest;
  ParamControl* mParamControlIsVagDebugNoLeadCarWarningTest;

public slots:

signals:

};

class VagTestPanel : public ListWidget {
  Q_OBJECT

public:
  explicit VagTestPanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  VagParam* mVagParam;
  QVBoxLayout* mVBoxList;

  ParamControl* mParamControlIsVagDebugOsdTestTextEnabled;
  ParamControl* mParamControlIsVagDebugItem1Enabled;
  ParamControl* mParamControlIsVagDebugItem2Enabled;
  ParamControl* mParamControlIsVagDebugItem3Enabled;
  ParamControl* mParamControlIsVagDebugItem4Enabled;
  ParamControl* mParamControlIsVagDebugItem5Enabled;

  // ------ Dynamic DCC -----
  ParamControl* mParamControlIsVagDynamicDccEnabled;

  // ----- Driving Mode -----
  ParamControl* mParamControlIsVagDrivingModeEnabled;
  ButtonParamControl *mVagDrivingMode;

public slots:

signals:

};

class VagSettingPanel : public ListWidget {
  Q_OBJECT

public:
  explicit VagSettingPanel(QWidget* parent = nullptr);
  //void refreshVisible();

private:
  VagParam* mVagParam;
  void showEvent(QShowEvent *event) override;
  QVBoxLayout* mVBoxList;
#if 0
  // ----- Auto showdown -----
  QHBoxLayout* mHBoxVagAutoShutdown;
  QLabel* mLabelVagAutoShutdownTitle;
  QSlider* mSliderVagAutoShutdown;
  QLabel* mLabelVagAutoShutdownValue;
  QLabel* mLabelVagAutoShutdownUnit;
#endif
  // ----- maunal sounrd volume - start -----
  ParamControl* mParamControlIsVagManualSoundVolumeEnable;
  QHBoxLayout* mHBoxVagSoundVolume;
  QLabel* mLabelVagSoundVolumeTitle;
  QSlider* mSliderVagSoundVolume;
  QLabel* mLabelVagSoundVolumeValue;
  // ----- maunal osd backlight - start -----
  ParamControl* mParamControlIsVagManualOsdBacklightEnable;
  QHBoxLayout* mHBoxVagOsdBacklight;
  QLabel* mLabelVagOsdBacklightTitle;
  QSlider* mSliderVagOsdBacklight;
  QLabel* mLabelVagOsdBacklightValue;

  ParamControl* mParamControlIsVagInfoBoxEnabled;
  ParamControl* mParamControlIsVagBlinkerEnabled;
  ParamControl* mParamControlIsVagBrakeLightEnabled;
  ParamControl* mParamControlIsVagLeadCarEnabled;

public slots:
  void setVolume(int volume);
  void setBacklight(int backlight);
  void setAutoShowdownMinutes(int minutes);
};

class VagFeaturePanel : public ListWidget {
  Q_OBJECT

public:
  explicit VagFeaturePanel(QWidget* parent = nullptr);

private:
  VagParam* mVagParam;
  void showEvent(QShowEvent *event) override;
  QVBoxLayout* mVBoxList;

  // ----- Blindspot -----
  ParamControl* mParamControlIsVagBlindspotEnabled;
  ParamControl* mParamControlIsVagBlindspotInfoSoundEnabled;
  ParamControl* mParamControlIsVagBlindspotInfoVibratorEnabled;
  ParamControl* mParamControlIsVagBlindspotWarningSoundEnabled;
  ParamControl* mParamControlIsVagBlindspotWarningVibratorEnabled;
  ParamControl* mParamControlIsVagBlindspotVibratorWithFlka;

  // ----- FLKA -----
  ParamControl* mParamControlIsVagFulltimeLkaEnabled;
  ParamControl* mParamControlIsVagFulltimeLkaEnableWithBlinker;
  ParamControl* mParamControlIsVagFulltimeLkaEnableWithBrake;
  ParamControl* mParamControlIsVagFulltimeLkaEnableWithAssistant;

  // ----- Leac car going remind -----
  ParamControl* mParamControlIsVagLeadCarGoingRemindEnabled;
  ParamControl* mParamControlIsVagLeadCarGoingRemindSoundEnabled;

  // ----- No lead car warning -----
  ParamControl* mParamControlIsVagNoLeadCarEnabled;
  ParamControl* mParamControlIsVagNoLeadCarWarningSoundEnabled;

  // ----- Force disable startstop -----
  ParamControl* mParamControlIsVagForceDisableStartstop;

public slots:
};

class VagWarningPanel : public ListWidget {
  Q_OBJECT

public:
  explicit VagWarningPanel(QWidget* parent = nullptr);

private:
  VagParam* mVagParam;
  QVBoxLayout* mVBoxList;

  ParamControl* mParamControlIsVagWarningEngineTurboPressure;
  ParamControl* mParamControlIsVagWarningEngineOilPressure;
  ParamControl* mParamControlIsVagWarningEngineInAirTemperature;
  ParamControl* mParamControlIsVagWarningEngineCoolantTemperature;
  ParamControl* mParamControlIsVagWarningEngineOilTemperature;
  ParamControl* mParamControlIsVagWarningCoolantTemperature;
  ParamControl* mParamControlIsVagWarningInAirPressure;
  ParamControl* mParamControlIsVagWarningGearOilTemperature;
  ParamControl* mParamControlIsVagWarningBrakePressure;
  ParamControl* mParamControlIsVagWarningIndoorTemperature;
  ParamControl* mParamControlIsVagWarningOutdoorTemperature;

public slots:

signals:

};

class VagDebugWindow : public QFrame {
  Q_OBJECT

public:
  explicit VagDebugWindow(QWidget *parent = 0);

protected:
  void showEvent(QShowEvent *event) override;

signals:
  void closeVagDebug();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;

  VagGeneralPanel *mVagGeneralPanel;
  VagOsdPanel *mVagOsdPanel;
  VagTestPanel *mVagTestPanel;
  VagSettingPanel *mVagSettingPanel;
  VagFeaturePanel *mVagFeaturePanel;
  VagWarningPanel *mVagWarningPanel;
};

