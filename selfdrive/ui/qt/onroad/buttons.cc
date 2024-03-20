#include "selfdrive/ui/qt/onroad/buttons.h"

#include <QPainter>

#include "selfdrive/ui/qt/util.h"

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
}

// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/comma.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && params.getBool("ExperimentalModeConfirmed");
  if (can_change) {
    params.putBool("ExperimentalMode", !experimental_mode);
  }
}

void ExperimentalButton::updateState(const UIState &s) {
  const auto cs = (*s.sm)["controlsState"].getControlsState();
  bool eng = cs.getEngageable() || cs.getEnabled();
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  QPixmap img = experimental_mode ? experimental_img : engage_img;
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), img, QColor(0, 0, 0, 166), (isDown() || !engageable) ? 0.6 : 1.0);
}

// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size, img_size});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), settings_img, QColor(0, 0, 0, 166), isDown() ? 0.6 : 1.0);
}


// MonitorOffButton
MonitorOffButton::MonitorOffButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);
  monitor_off_img = loadPixmap("../assets/images/img_monitor_off.png", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &MonitorOffButton::ButtonClicked);
}

void MonitorOffButton::ButtonClicked() {
  Hardware::set_display_power(false);
}

void MonitorOffButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.0);
  p.drawPixmap(QPoint(btn_size/2, btn_size/2) - QPoint(monitor_off_img.width()/2, monitor_off_img.height()/2), monitor_off_img);
  p.setOpacity(1.0);
}

// VagHudButton
VagHudButton::VagHudButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);
  vag_hud_img = loadPixmap("../assets/images/img_hud.png", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &VagHudButton::ButtonClicked);
}

void VagHudButton::ButtonClicked() {
  emit openVagHud();
}

void VagHudButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.0);
  p.drawPixmap(QPoint(btn_size/2, btn_size/2) - QPoint(vag_hud_img.width()/2, vag_hud_img.height()/2), vag_hud_img);
  p.setOpacity(1.0);
}

// VagSettingsButton
VagSettingsButton::VagSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);
  vag_settings_img = loadPixmap("../assets/images/img_settings.png", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &VagSettingsButton::ButtonClicked);
}

void VagSettingsButton::ButtonClicked() {
  emit openVagSettings();
}

void VagSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.0);
  p.drawPixmap(QPoint(btn_size/2, btn_size/2) - QPoint(vag_settings_img.width()/2, vag_settings_img.height()/2), vag_settings_img);
  p.setOpacity(1.0);
}