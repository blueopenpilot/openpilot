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
#include "selfdrive/ui/qt/vag_param.h"
#include "selfdrive/ui/qt/widgets/controls.h"

#define COLOR_BLACK QColor(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) QColor(0, 0, 0, x)
#define COLOR_WHITE QColor(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) QColor(255, 255, 255, x)
#define COLOR_RED QColor(201, 34, 49, 255)
#define COLOR_RED_ALPHA(x) QColor(201, 34, 49, x)
#define COLOR_ORANGE QColor(255, 127, 0, 255)
#define COLOR_ORANGE_ALPHA(x) QColor(255, 127, 0, x)
#define COLOR_YELLOW QColor(218, 202, 37, 255)
#define COLOR_YELLOW_ALPHA(x) QColor(218, 202, 37, x)
#define COLOR_GREEN QColor(0, 255, 0, 255)
#define COLOR_GREEN_ALPHA(x) QColor(0, 255, 0, x)
#define COLOR_BLUE QColor(0, 0, 255, 255)
#define COLOR_BLUE_ALPHA(x) QColor(0, 0, 255, x)
#define COLOR_PURPLE QColor(255, 0, 255, 255)
#define COLOR_PURPLE_ALPHA(x) QColor(255, 0, 255, x)

#define ALIGN_LEFT Qt::AlignLeft
#define ALIGN_RIGHT Qt::AlignRight
#define ALIGN_HCENTER Qt::AlignHCenter
#define ALIGN_TOP Qt::AlignTop
#define ALIGN_BOTTOM Qt::AlignBottom
#define ALIGN_VCENTER Qt::AlignVCenter
#define ALIGN_CENTER Qt::AlignCenter
#define ALIGN_BASELINE Qt::AlignBaseline

const int btn_size = 180;
const int img_size = 170;

class VagOsd : public QWidget {
  Q_OBJECT

public:
  explicit VagOsd(QWidget* parent = 0);
  std::unique_ptr<SubMaster> sm;

  void drawOsdText(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align);
  void drawOsdElementBox(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h);
  void drawOsdElementTitle(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align);
  void drawOsdElementValue(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align);
  void drawOsdElementUnit(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align);
  void drawOsdBrakeLight(QPainter &p);
  void drawOsdBlinker(QPainter &p);
  void drawOsdBlindspot(QPainter &p);
  void drawOsdNoLeadCar(QPainter &p);
  void drawOsdLeadCarGoing(QPainter &p);
#if 0
  void drawOsdSpeedCamera(QPainter &p);
  void drawOsdOnlineMapInfo(QPainter &p);
#endif
  void drawOsdInfobar(QPainter &p);
  void drawOsdTest(QPainter &p);
  void drawOsd(QPainter &p);

  inline QColor redColor(int alpha = 255) { return QColor(201, 34, 49, alpha); }
  inline QColor whiteColor(int alpha = 255) { return QColor(255, 255, 255, alpha); }
  inline QColor blackColor(int alpha = 255) { return QColor(0, 0, 0, alpha); }

protected:

private:
  QTimer* mUpdateTimer;

};
