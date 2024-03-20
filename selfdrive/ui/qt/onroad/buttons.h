#pragma once

#include <QPushButton>

#include "selfdrive/ui/ui.h"

const int btn_size = 192;
const int img_size = (btn_size / 4) * 3;

class ExperimentalButton : public QPushButton {
  Q_OBJECT

public:
  explicit ExperimentalButton(QWidget *parent = 0);
  void updateState(const UIState &s);

private:
  void paintEvent(QPaintEvent *event) override;
  void changeMode();

  Params params;
  QPixmap engage_img;
  QPixmap experimental_img;
  bool experimental_mode;
  bool engageable;
};


class MapSettingsButton : public QPushButton {
  Q_OBJECT

public:
  explicit MapSettingsButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;

  QPixmap settings_img;
};

class MonitorOffButton : public QPushButton {
  Q_OBJECT

public:
  explicit MonitorOffButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;
  void ButtonClicked();

  QPixmap monitor_off_img;
};

class VagHudButton : public QPushButton {
  Q_OBJECT

public:
  explicit VagHudButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;
  void ButtonClicked();

  QPixmap vag_hud_img;
signals:
  void openVagHud();
};

class VagSettingsButton : public QPushButton {
  Q_OBJECT

public:
  explicit VagSettingsButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;
  void ButtonClicked();

  QPixmap vag_settings_img;
signals:
  void openVagSettings();
};

void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity);
