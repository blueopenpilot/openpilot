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
#include <QTimer>

#include "selfdrive/ui/ui.h"
#include "selfdrive/ui/qt/vag_osd.h"
#include "selfdrive/ui/qt/vag_param.h"
#include "selfdrive/ui/qt/widgets/controls.h"

class CloseVagHudButton : public QPushButton {
  Q_OBJECT

public:
  explicit CloseVagHudButton(QWidget *parent = 0);

private:
  void paintEvent(QPaintEvent *event) override;
  void ButtonClicked();

  QPixmap close_vag_hud_img;
};

class VagHudWindow : public QWidget {
  Q_OBJECT

public:
  explicit VagHudWindow(QWidget* parent = 0);
  std::unique_ptr<SubMaster> sm;

protected:

private:
  void paintEvent(QPaintEvent *event) override;
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;
  void drawOpStatus(QPainter &painter);
  void drawVagHud(QPainter &painter);

  CloseVagHudButton* close_btn;
  QTimer* mUpdateTimer;

signals:
  void closeVagHud();

public slots:
  void updateUI();

};
