/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#pragma once

#include <QStackedLayout>
#include <QWidget>

#include "selfdrive/ui/qt/home.h"
#include "selfdrive/ui/qt/offroad/onboarding.h"
#include "selfdrive/ui/qt/offroad/settings.h"
#include "selfdrive/ui/qt/vag_debug.h"
#include "selfdrive/ui/qt/vag_hud.h"
#include "selfdrive/ui/qt/vag_settings.h"

class MainWindow : public QWidget {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);

private:
  bool eventFilter(QObject *obj, QEvent *event) override;
  void openSettings(int index = 0, const QString &param = "");
  void closeSettings();
  void openVagDebug();
  void closeVagDebug();
  void openVagHud();
  void closeVagHud();
  void openVagSettings();
  void closeVagSettings();
  void closeVagScreen();

  QStackedLayout *main_layout;
  HomeWindow *homeWindow;
  SettingsWindow *settingsWindow;
  OnboardingWindow *onboardingWindow;
  VagDebugWindow *vagDebugWindow;
  VagHudWindow *vagHudWindow;
  VagSettingsWindow *vagSettingsWindow;
};
