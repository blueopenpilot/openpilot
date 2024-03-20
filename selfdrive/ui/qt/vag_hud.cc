/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "selfdrive/ui/qt/vag_hud.h"

#include <cassert>
#include <cmath>
#include <string>
#include <stdio.h>

#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "selfdrive/ui/qt/widgets/controls.h"
#include "selfdrive/ui/qt/widgets/toggle.h"
#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/util.h"
#include "selfdrive/ui/qt/qt_window.h"

// CloseVagHudButton
CloseVagHudButton::CloseVagHudButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(200, 200);
  close_vag_hud_img = loadPixmap("../assets/images/img_cancel.png", {200, 200});
  QObject::connect(this, &QPushButton::clicked, this, &CloseVagHudButton::ButtonClicked);
}

void CloseVagHudButton::ButtonClicked() {
}

void CloseVagHudButton::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setOpacity(1.0);  // bg dictates opacity of ellipse
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(0, 0, 0, 166));
  painter.drawEllipse(QPoint(200/2, 200/2), 200/2, 200/2);
  painter.setOpacity(1.0);
  painter.drawPixmap(QPoint(200/2, 200/2) - QPoint(close_vag_hud_img.width() / 2, close_vag_hud_img.height() / 2), close_vag_hud_img);
  painter.setOpacity(1.0);
}


void VagHudWindow::showEvent(QShowEvent *event) {
  mUpdateTimer->start(50);
  QWidget::show();
}

void VagHudWindow::hideEvent(QHideEvent *event) {
  mUpdateTimer->stop();
  QWidget::hide();
}

VagHudWindow::VagHudWindow(QWidget *parent) : QWidget(parent) {
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({"carState", "controlsState", "peripheralState"});

  QVBoxLayout* main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(40, 40, 40, 40);

  // top header
  QHBoxLayout* header_layout = new QHBoxLayout();
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(16);

  close_btn = new CloseVagHudButton(this);
  close_btn->setVisible(true);
  close_btn->setStyleSheet("background-color: #364DEF;");
  close_btn->setFixedWidth(200);
  close_btn->setFixedHeight(200);
  QObject::connect(close_btn, &QPushButton::clicked, this, &VagHudWindow::closeVagHud);
  header_layout->addWidget(close_btn, 0, Qt::AlignBottom | Qt::AlignRight);
  main_layout->addLayout(header_layout);

  mUpdateTimer = new QTimer(this);
  QObject::connect(mUpdateTimer, &QTimer::timeout, this, &VagHudWindow::updateUI);
}

void VagHudWindow::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), QColor(0, 0, 0));

  drawVagHud(painter);
  drawOpStatus(painter);

#if 0
  //TODO: Add a rectagle
  painter.setViewport(30, 30, 2160, 1080);
  //painter.setWindow(30, 30, 2100, 1020);
  VagOsd* vagOsd = new VagOsd(this);
  //vagOsd->setGeometry(30, 30, 2100, 1020);
  vagOsd->drawOsd(painter);
#endif
}

void VagHudWindow::updateUI() {
  if(this->isVisible()) {
    this->repaint();
  }
}

void VagHudWindow::drawOpStatus(QPainter &painter) {
  UIStatus status;
  UIState *s = uiState();
  auto controls_state = (*s->sm)["controlsState"].getControlsState();
  auto state = controls_state.getState();
  if (state == cereal::ControlsState::OpenpilotState::PRE_ENABLED || state == cereal::ControlsState::OpenpilotState::OVERRIDING) {
    status = STATUS_OVERRIDE;
  } else {
    status = controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
  }


  QPen pen;
  pen.setColor(bg_colors[status]);
  //pen.setColor(QColor(128, 255, 255));
  pen.setWidth(60);
  painter.setPen(pen);
  QRectF rect(0, 0, 2160, 1080);
  painter.drawRect(rect);
}

void VagHudWindow::drawVagHud(QPainter &painter) {
  UIState *s = uiState();

  //Draw HUD
  int imgSize = 150;
  char stringBuffer[100];
  QFont font;
  font.setPointSize(10*4);
  painter.setFont(font);
  painter.setPen(QColor(128, 255, 255));
  painter.setPen(QColor(128, 255, 255));

  // ===== Engine =====
  QRectF rectangleEngine(30.0, 30.0, 750.0, 510.0);
  painter.drawRoundedRect(rectangleEngine, 15.0, 15.0);
  // ----- Engine oil -----
  QPixmap imgEngineOil = QPixmap("../assets/images/img_engine_oil.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(80, 100, imgSize, imgSize, imgEngineOil);
  font.setPointSize(70);
  painter.setFont(font);
  const float moOelTemp = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoOelTemp();
  const float moOeldruck = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoOeldruck();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.2f ℃", moOelTemp);
  painter.drawText(300, 20, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.2f Bar", moOeldruck);
  painter.drawText(300, 120, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  // ----- Engine in air -----
  QPixmap imgEngineInAir = QPixmap("../assets/images/img_engine_filter.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(80, 320, imgSize, imgSize, imgEngineInAir);
  font.setPointSize(70);
  painter.setFont(font);
  const float moAnsaugluftTemp = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoAnsaugluftTemp();
  const float moRelSaugrohrdruck = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoRelSaugrohrdruck()/1000;
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.2f ℃", moAnsaugluftTemp);
  painter.drawText(300, 250, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  snprintf(stringBuffer, sizeof(stringBuffer), "%.2f Bar", moRelSaugrohrdruck);
  painter.drawText(300, 350, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));

  // ===== Gearbox =====
  QRectF rectangleGearbox(30.0, 540.0, 525.0, 510.0);
  painter.drawRoundedRect(rectangleGearbox, 15.0, 15.0);
  QPixmap imgGearbox = QPixmap("../assets/images/img_gearbox.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(80, 600, imgSize, imgSize, imgGearbox);
  // ----- Gearbox level -----
  font.setPointSize(80);
  painter.setFont(font);
  const auto gearShift = (*s->sm)["carState"].getCarState().getGearShifter();
  const int geZielgang = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getGeZielgang();
  if(gearShift==cereal::CarState::GearShifter::PARK) {
    snprintf(stringBuffer, sizeof(stringBuffer), "P");
  } else if(gearShift==cereal::CarState::GearShifter::REVERSE) {
    snprintf(stringBuffer, sizeof(stringBuffer), "R");
  } else if(gearShift==cereal::CarState::GearShifter::NEUTRAL) {
    snprintf(stringBuffer, sizeof(stringBuffer), "N");
  } else if(gearShift==cereal::CarState::GearShifter::DRIVE) {
    snprintf(stringBuffer, sizeof(stringBuffer), "D%1d", geZielgang);
  } else if(gearShift==cereal::CarState::GearShifter::SPORT) {
    snprintf(stringBuffer, sizeof(stringBuffer), "S%1d", geZielgang);
  } else if(gearShift==cereal::CarState::GearShifter::MANUMATIC) {
    snprintf(stringBuffer, sizeof(stringBuffer), "M%1d", geZielgang);
  } else if(gearShift==cereal::CarState::GearShifter::ECO) {
    snprintf(stringBuffer, sizeof(stringBuffer), "E%1d", geZielgang);
  } else {
    snprintf(stringBuffer, sizeof(stringBuffer), "UN");
  }
  painter.drawText(300, 590, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  // ----- Gearbox Temperature
  const float geSumpftemperatur = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getGeSumpftemperatur();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.0f℃", geSumpftemperatur);
  painter.drawText(180, 820, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));

  // ===== Coolant =====
  QRectF rectangleCoolant(555.0, 540.0, 525.0, 510.0);
  painter.drawRoundedRect(rectangleCoolant, 15.0, 15.0);
  // ----- Engine coolant -----
  QPixmap imgEngineCoolant = QPixmap("../assets/images/img_engine_coolant.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(600, 590, imgSize, imgSize, imgEngineCoolant);
  font.setPointSize(50);
  painter.setFont(font);
  const float moKuehlmittelTemp = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoKuehlmittelTemp();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.2f℃", moKuehlmittelTemp);
  painter.drawText(780, 550, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  // ----- Coolant -----
  QPixmap imgCoolant = QPixmap("../assets/images/img_coolant.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(600, 820, imgSize, imgSize, imgCoolant);
  font.setPointSize(50);
  painter.setFont(font);
  const float moItmKuehlmittelTemp = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoItmKuehlmittelTemp();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.2f℃", moItmKuehlmittelTemp);
  painter.drawText(780, 820, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));

  // ===== speed =====
  QRectF rectangleSpeed(780.0, 30.0, 825.0, 510.0);
  painter.drawRoundedRect(rectangleSpeed, 15.0, 15.0);
  // ----- speed -----
  QPixmap imgSpeedometer = QPixmap("../assets/images/img_speedometer.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(830, 80, imgSize, imgSize, imgSpeedometer);
  font.setPointSize(120);
  painter.setFont(font);
  const float speed = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getSpeed();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.0f", speed);
  painter.drawText(1000, 50, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  font.setPointSize(40);
  painter.setFont(font);
  snprintf(stringBuffer, sizeof(stringBuffer), "KM/H");
  painter.drawText(1450, 100, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  // ----- rpm -----
  QPixmap imgRpm = QPixmap("../assets/images/img_engine_rpm.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(830, 335, imgSize, imgSize, imgRpm);
  font.setPointSize(120);
  painter.setFont(font);
  const float rpm = (*s->sm)["carState"].getCarState().getEngineRpm();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.0f", rpm);
  painter.drawText(1000, 305, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  font.setPointSize(40);
  painter.setFont(font);
  snprintf(stringBuffer, sizeof(stringBuffer), "RPM");
  painter.drawText(1450, 355, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));

  // ===== Temperature =====
  QRectF rectangleTemperatue(1080.0, 540.0, 525.0, 510.0);
  painter.drawRoundedRect(rectangleTemperatue, 15.0, 15.0);
  // ----- In door -----
  QPixmap imgIndoorTemparature = QPixmap("../assets/images/img_indoor_temparature.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(1130, 590, imgSize, imgSize, imgIndoorTemparature);
  font.setPointSize(60);
  painter.setFont(font);
  const float kbiAussenTempGef = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getKbiAussenTempGef();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.1f℃", kbiAussenTempGef);
  painter.drawText(1300, 560, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
  // ----- Out door -----
  QPixmap imgOutdoorTemparature = QPixmap("../assets/images/img_outdoor_temparature.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(1130, 830, imgSize, imgSize, imgOutdoorTemparature);
  font.setPointSize(60);
  painter.setFont(font);
  const float bcm1AussenTempUngef = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getBcm1AussenTempUngef();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.1f℃", bcm1AussenTempUngef);
  painter.drawText(1300, 820, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));

  // ===== Battery =====
  QRectF rectangleBatteryVoltage(1605.0, 30.0, 525.0, 255.0);
  painter.drawRoundedRect(rectangleBatteryVoltage, 15.0, 15.0);
  QPixmap imgBattery = QPixmap("../assets/images/img_battery.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(1650, 80, imgSize, imgSize, imgBattery);
  font.setPointSize(60);
  painter.setFont(font);
  float voltage = 0;
  const int voltageInt = (int)(*s->sm)["peripheralState"].getPeripheralState().getVoltage();
  voltage = (float)voltageInt/1000;
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.2f V", voltage);
  painter.drawText(1820, 50, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));

  // ===== Turbo =====
  QRectF rectangleTurbo(1605.0, 285.0, 525.0, 255.0);
  painter.drawRoundedRect(rectangleTurbo, 15.0, 15.0);
  QPixmap imgTurbo = QPixmap("../assets/images/img_turbo.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(1650, 335, imgSize, imgSize, imgTurbo);
  font.setPointSize(60);
  painter.setFont(font);  const float moLadedruck = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getMoLadedruck();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.1f Bar", moLadedruck);
  painter.drawText(1820, 305, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));

  // ===== Brake =====
  QRectF rectangleBrake(1605.0, 540.0, 525.0, 255.0);
  painter.drawRoundedRect(rectangleBrake, 15.0, 15.0);
  QPixmap imgBrake = QPixmap("../assets/images/img_brakes.png").scaled(imgSize, imgSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  painter.drawPixmap(1650, 590, imgSize, imgSize, imgBrake);
  font.setPointSize(55);
  painter.setFont(font);  const float espBremsdruck = (*s->sm)["carState"].getCarState().getVagCarState().getVagUiField().getEspBremsdruck();
  snprintf(stringBuffer, sizeof(stringBuffer), "%2.1f Bar", espBremsdruck);
  painter.drawText(1820, 560, 1800, 200, Qt::AlignVCenter+Qt::AlignLeft, QString(stringBuffer));
}
