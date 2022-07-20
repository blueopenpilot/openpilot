/*
 * Copyright (c) 2020-2022 bluetulippon@gmail.com Chad_Peng.
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


#include "selfdrive/ui/qt/widgets/controls.h"
//VAG settings
class DebugSettingPanel : public ListWidget {
  Q_OBJECT

public:
  explicit DebugSettingPanel(QWidget* parent = nullptr);
  //void refreshVisible();

private:
  void showEvent(QShowEvent *event) override;
  QVBoxLayout* mVBoxVagSettingList;
#if 0
  //===== Auto showdown =====
  QHBoxLayout* mHBoxVagAutoShutdown;
  QLabel* mLabelVagAutoShutdownTitle;
  QSlider* mSliderVagAutoShutdown;
  QLabel* mLabelVagAutoShutdownValue;
  QLabel* mLabelVagAutoShutdownUnit;
#endif
  //===== maunal sounrd volume - start =====
  ParamControl* mParamControlIsVagManualSoundVolumeEnable;
  QHBoxLayout* mHBoxVagSoundVolume;
  QLabel* mLabelVagSoundVolumeTitle;
  QSlider* mSliderVagSoundVolume;
  QLabel* mLabelVagSoundVolumeValue;
  //===== maunal osd backlight - start =====
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

class DebugFeaturePanel : public ListWidget {
  Q_OBJECT

public:
  explicit DebugFeaturePanel(QWidget* parent = nullptr);

private:
  void showEvent(QShowEvent *event) override;
  QVBoxLayout* mVBoxVagFeatureList;

  //----- Blindspot -----
  ParamControl* mParamControlIsVagBlindspotEnabled;
#if 0
  QHBoxLayout* mHBoxVagBlindspotOsdSize;
  QLabel* mLabelVagBlindspotOsdSize;
  QComboBox* mQComboBoxVagBlindspotOsdSize;
#endif
  ParamControl* mParamControlIsVagBlindspotWarningSoundEnabled;

  //----- FLKA -----
  ParamControl* mParamControlIsVagFulltimeLkaEnabled;
  ParamControl* mParamControlIsVagFulltimeLkaEnableWithBlinker;
  ParamControl* mParamControlIsVagFulltimeLkaEnableWithBrake;

  //----- Leac car going remind -----
  ParamControl* mParamControlIsVagLeadCarGoingRemindEnabled;
  ParamControl* mParamControlIsVagLeadCarGoingRemindSoundEnabled;

  //----- No lead car warning -----
  ParamControl* mParamControlIsVagNoLeadCarEnabled;
  ParamControl* mParamControlIsVagNoLeadCarWarningSoundEnabled;


  //----- SACC -----
  ParamControl* mParamControlIsVagSpeedCameraEnabled;
  ParamControl* mParamControlIsVagSpeedLimitSoundEnabled;
  //ParamControl* mParamControlIsVagSaccEnabled;

public slots:

};

class DebugTestPanel : public ListWidget {
  Q_OBJECT

public:
  explicit DebugTestPanel(QWidget* parent = nullptr);

private:
  QVBoxLayout* mVBoxDebugList;

  ParamControl* mParamControlOsdTestText;
  ParamControl* mParamControlDebugItem1;
  ParamControl* mParamControlDebugItem2;
  ParamControl* mParamControlDebugItem3;
  ParamControl* mParamControlDebugItem4;
  ParamControl* mParamControlDebugItem5;

public slots:

signals:

};

class DebugGeneralPanel : public ListWidget {
  Q_OBJECT

public:
  explicit DebugGeneralPanel(QWidget* parent = nullptr);

private:
  QVBoxLayout* mVBoxDebugList;

  ParamControl* mParamControlDevelopMode;
  ParamControl* mParamControlDevelopOnRoadUi;
  ParamControl* mParamControlRunningProcessLog;

public slots:

signals:

};

class DebugOsdPanel : public ListWidget {
  Q_OBJECT

public:
  explicit DebugOsdPanel(QWidget* parent = nullptr);

private:
  QVBoxLayout* mVBoxDebugList;

#if 0
  ParamControl* mParamControlInfoBox;
  ParamControl* mParamControlInfobar;
#endif
  ParamControl* mParamControlBlinker;
  ParamControl* mParamControlBlindspotInfo;
  ParamControl* mParamControlBlindspotWarning;
  ParamControl* mParamControlBrakeLight;
  ParamControl* mParamControlLeadCarGoingRemind;
  ParamControl* mParamControlNoLeadCarWarning;

public slots:

signals:

};

class DebugLogPanel : public ListWidget {
  Q_OBJECT

public:
  explicit DebugLogPanel(QWidget* parent = nullptr);

private:
  QVBoxLayout* mVBoxDebugList;

  ParamControl* mParamControlFlkaLog;
  ParamControl* mParamControlSaccLog;
  ParamControl* mParamControlSaccSpeedCamTrack;
  ParamControl* mParamControlSaccVehicleTrack;

public slots:

signals:

};

class DebugWindow : public QFrame {
  Q_OBJECT

public:
  explicit DebugWindow(QWidget *parent = 0);

protected:
  void showEvent(QShowEvent *event) override;

signals:
  void closeDebug();

private:
  QPushButton *sidebar_alert_widget;
  QWidget *sidebar_widget;
  QButtonGroup *nav_btns;
  QStackedWidget *panel_widget;

  DebugSettingPanel *mDebugSettingPanel;
  DebugFeaturePanel *mDebugFeaturePanel;
  DebugTestPanel *mDebugTestPanel;
  DebugGeneralPanel *mDebugGeneralPanel;
  DebugOsdPanel *mDebugOsdPanel;
  DebugLogPanel *mDebugLogPanel;
};

