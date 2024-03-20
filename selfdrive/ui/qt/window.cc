/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "selfdrive/ui/qt/window.h"

#include <QFontDatabase>

#include "system/hardware/hw.h"

MainWindow::MainWindow(QWidget *parent) : QWidget(parent) {
  main_layout = new QStackedLayout(this);
  main_layout->setMargin(0);

  homeWindow = new HomeWindow(this);
  main_layout->addWidget(homeWindow);
  QObject::connect(homeWindow, &HomeWindow::openSettings, this, &MainWindow::openSettings);
  QObject::connect(homeWindow, &HomeWindow::closeSettings, this, &MainWindow::closeSettings);
  QObject::connect(homeWindow, &HomeWindow::openVagDebug, this, &MainWindow::openVagDebug);
  QObject::connect(homeWindow, &HomeWindow::closeVagDebug, this, &MainWindow::closeVagDebug);
  QObject::connect(homeWindow, &HomeWindow::openVagHud, this, &MainWindow::openVagHud);
  QObject::connect(homeWindow, &HomeWindow::openVagSettings, this, &MainWindow::openVagSettings);

  settingsWindow = new SettingsWindow(this);
  main_layout->addWidget(settingsWindow);
  QObject::connect(settingsWindow, &SettingsWindow::closeSettings, this, &MainWindow::closeSettings);
  QObject::connect(settingsWindow, &SettingsWindow::reviewTrainingGuide, [=]() {
    onboardingWindow->showTrainingGuide();
    main_layout->setCurrentWidget(onboardingWindow);
  });
  QObject::connect(settingsWindow, &SettingsWindow::showDriverView, [=] {
    homeWindow->showDriverView(true);
  });

  vagDebugWindow = new VagDebugWindow(this);
  main_layout->addWidget(vagDebugWindow);
  QObject::connect(vagDebugWindow, &VagDebugWindow::closeVagDebug, this, &MainWindow::closeVagDebug);

  vagHudWindow = new VagHudWindow(this);
  main_layout->addWidget(vagHudWindow);
  QObject::connect(vagHudWindow, &VagHudWindow::closeVagHud, this, &MainWindow::closeVagHud);

  vagSettingsWindow = new VagSettingsWindow(this);
  main_layout->addWidget(vagSettingsWindow);
  QObject::connect(vagSettingsWindow, &VagSettingsWindow::closeVagSettings, this, &MainWindow::closeVagSettings);

  onboardingWindow = new OnboardingWindow(this);
  main_layout->addWidget(onboardingWindow);
  QObject::connect(onboardingWindow, &OnboardingWindow::onboardingDone, [=]() {
    main_layout->setCurrentWidget(homeWindow);
  });
  if (!onboardingWindow->completed()) {
    main_layout->setCurrentWidget(onboardingWindow);
  }

  QObject::connect(uiState(), &UIState::offroadTransition, [=](bool offroad) {
    if (!offroad) {
      closeSettings();
    }
  });
  QObject::connect(device(), &Device::interactiveTimeout, [=]() {
    if (main_layout->currentWidget() == settingsWindow) {
      closeSettings();
    }
  });

  // load fonts
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Black.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Bold.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-ExtraBold.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-ExtraLight.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Medium.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Regular.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-SemiBold.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/Inter-Thin.ttf");
  QFontDatabase::addApplicationFont("../assets/fonts/JetBrainsMono-Medium.ttf");

  // no outline to prevent the focus rectangle
  setStyleSheet(R"(
    * {
      font-family: Inter;
      outline: none;
    }
  )");
  setAttribute(Qt::WA_NoSystemBackground);
}

void MainWindow::openSettings(int index, const QString &param) {
  main_layout->setCurrentWidget(settingsWindow);
  settingsWindow->setCurrentPanel(index, param);
}

void MainWindow::closeSettings() {
  main_layout->setCurrentWidget(homeWindow);

  if (uiState()->scene.started) {
    homeWindow->showSidebar(false);
  }
}

void MainWindow::openVagDebug() {
  main_layout->setCurrentWidget(vagDebugWindow);
}

void MainWindow::closeVagDebug() {
  closeVagScreen();
}

void MainWindow::openVagHud() {
  main_layout->setCurrentWidget(vagHudWindow);
}

void MainWindow::closeVagHud() {
  closeVagScreen();
}

void MainWindow::openVagSettings() {
  main_layout->setCurrentWidget(vagSettingsWindow);
}

void MainWindow::closeVagSettings() {
  closeVagScreen();
}

void MainWindow::closeVagScreen() {
  UIState *s = uiState();
  const bool ignitionLine = (*s->sm)["pandaStates"].getPandaStates()[0].getIgnitionLine();

  main_layout->setCurrentWidget(homeWindow);

  if (s->scene.started) {
    homeWindow->showSidebar(false);
  }

  const bool isVagParamFromCerealEnabled = (*(uiState())->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagParamFromCerealEnabled();
  bool isVagDevelopOnRoadUi = false;
  if(isVagParamFromCerealEnabled) {
    isVagDevelopOnRoadUi = (*(uiState())->sm)["vagParam"].getVagParam().getVagParamGeneral().getIsVagDevelopOnRoadUi();
  } else {
    try {
      isVagDevelopOnRoadUi = Params().getBool("IsVagDevelopOnRoadUi");
    } catch (std::exception &e) {
      printf("[BOP][%s][%s][%d] Get param exception: %s \n", __FILE__, __FUNCTION__, __LINE__, e.what());
      isVagDevelopOnRoadUi = false;
    }
  }
  if(isVagDevelopOnRoadUi) {
    homeWindow->offroadTransition(false);
  } else {
    homeWindow->offroadTransition(!ignitionLine);
  }
}


bool MainWindow::eventFilter(QObject *obj, QEvent *event) {
  bool ignore = false;
  switch (event->type()) {
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
    case QEvent::MouseButtonPress:
    case QEvent::MouseMove: {
      // ignore events when device is awakened by resetInteractiveTimeout
      ignore = !device()->isAwake();
      device()->resetInteractiveTimeout();
      break;
    }
    default:
      break;
  }
  return ignore;
}
