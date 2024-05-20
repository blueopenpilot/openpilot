/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include "selfdrive/ui/qt/vag_osd.h"

VagOsd::VagOsd(QWidget *parent) : QWidget(parent) {
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({"carState"});
}

#define COLOR_BLOCK_SIZE 100
void VagOsd::drawOsdText(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align) {
  QFont font;
#ifdef QCOM
  font.setPointSize(font_size);
#elif QCOM2
  font.setPointSize(font_size*4);
#endif
  p.setFont(font);
  p.setPen(color);
  p.drawText(x, y, w, h, align, font_string);
}

void VagOsd::drawOsdElementBox(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h) {
  QRect rc(x, y, w, h);
  p.setPen(QPen(QColor(255, 255, 255, 255), 10));
  p.setBrush(QColor(0, 0, 0, 255));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);
}

void VagOsd::drawOsdElementTitle(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align) {
  QFont font;
#ifdef QCOM
  font.setPointSize(font_size);
#elif QCOM2
  font.setPointSize(font_size*4);
#endif
  p.setFont(font);
  p.setPen(color);
  p.drawText(x, y - 60, w, h, align, font_string);
}

void VagOsd::drawOsdElementValue(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align) {
  QFont font;
#ifdef QCOM
  font.setPointSize(font_size);
#elif QCOM2
  font.setPointSize(font_size*4);
#endif
  p.setFont(font);
  p.setPen(color);
  p.drawText(x, y, w, h, align, font_string);
}

void VagOsd::drawOsdElementUnit(QPainter &p,
                        const int x,
                        const int y,
                        const int w,
                        const int h,
                        const QString &font_string,
                        const unsigned int font_size,
                        const QColor color,
                        const Qt::AlignmentFlag align) {
  QFont font;
#ifdef QCOM
  font.setPointSize(font_size);
#elif QCOM2
  font.setPointSize(font_size*4);
#endif
  p.setFont(font);
  p.setPen(color);
  p.drawText(x, y + 60, w, h, align, font_string);
}

void VagOsd::drawOsdBrakeLight(QPainter &p) {
  UIState *s = uiState();

  const bool isVagDebugBrakeLightTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBrakeLightTest();
  const bool brakeLights = (*s->sm)["carState"].getCarState().getBrakeLights();

  if(brakeLights || isVagDebugBrakeLightTest) {
    QPainterPath infobarPath;
    infobarPath.moveTo(350, s->fb_h);
    infobarPath.lineTo(350 + COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    infobarPath.lineTo(s->fb_w - (350 + COLOR_BLOCK_SIZE), s->fb_h - COLOR_BLOCK_SIZE);
    infobarPath.lineTo(s->fb_w - 350, s->fb_h);
    infobarPath.closeSubpath();
    p.fillPath(infobarPath, COLOR_RED_ALPHA(200));

    char brakeLightBuffer[100];
    snprintf(brakeLightBuffer, sizeof(brakeLightBuffer), "Brake Light");
    drawOsdText(p, 450, s->fb_h - 100, s->fb_w - 900, 100, brakeLightBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_CENTER));
  }
}

#define BLINKER_RATE 12
static int BlinkerRate = BLINKER_RATE;
void VagOsd::drawOsdBlinker(QPainter &p) {
  UIState *s = uiState();


  const bool isVagDebugBlinkerTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBlinkerTest();
  const bool leftBlinker = (*s->sm)["carState"].getCarState().getLeftBlinker();
  const bool rightBlinker = (*s->sm)["carState"].getCarState().getRightBlinker();
  if(leftBlinker || isVagDebugBlinkerTest) {
    QPainterPath leftBlinkerPath;
    leftBlinkerPath.moveTo(0, 0);
    leftBlinkerPath.lineTo(COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    leftBlinkerPath.lineTo(250 + COLOR_BLOCK_SIZE + COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    leftBlinkerPath.lineTo(250 + COLOR_BLOCK_SIZE, 0);
    leftBlinkerPath.closeSubpath();
    p.fillPath(leftBlinkerPath, (BlinkerRate >= (BLINKER_RATE / 2) ? COLOR_YELLOW_ALPHA(200) : COLOR_YELLOW_ALPHA(30)));

    leftBlinkerPath.moveTo(0, s->fb_h);
    leftBlinkerPath.lineTo(COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    leftBlinkerPath.lineTo(250 + COLOR_BLOCK_SIZE + COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    leftBlinkerPath.lineTo(250 + COLOR_BLOCK_SIZE, s->fb_h);
    leftBlinkerPath.closeSubpath();
    p.fillPath(leftBlinkerPath, (BlinkerRate >= (BLINKER_RATE / 2) ? COLOR_YELLOW_ALPHA(200) : COLOR_YELLOW_ALPHA(30)));
  }
  if(rightBlinker || isVagDebugBlinkerTest) {
    QPainterPath rightBlinkerPath;
    rightBlinkerPath.moveTo(s->fb_w, 0);
    rightBlinkerPath.lineTo(s->fb_w - COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    rightBlinkerPath.lineTo(s->fb_w - (250 + COLOR_BLOCK_SIZE + COLOR_BLOCK_SIZE), COLOR_BLOCK_SIZE);
    rightBlinkerPath.lineTo(s->fb_w - (250 + COLOR_BLOCK_SIZE), 0);
    rightBlinkerPath.closeSubpath();
    p.fillPath(rightBlinkerPath, (BlinkerRate >= (BLINKER_RATE / 2) ? COLOR_YELLOW_ALPHA(200) : COLOR_YELLOW_ALPHA(30)));

    rightBlinkerPath.moveTo(s->fb_w, s->fb_h);
    rightBlinkerPath.lineTo(s->fb_w - COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    rightBlinkerPath.lineTo(s->fb_w - (250 + COLOR_BLOCK_SIZE + COLOR_BLOCK_SIZE), s->fb_h - COLOR_BLOCK_SIZE);
    rightBlinkerPath.lineTo(s->fb_w - (250 + COLOR_BLOCK_SIZE), s->fb_h);
    rightBlinkerPath.closeSubpath();
    p.fillPath(rightBlinkerPath, (BlinkerRate >= (BLINKER_RATE / 2) ? COLOR_YELLOW_ALPHA(200) : COLOR_YELLOW_ALPHA(30)));
  }
  if(leftBlinker || rightBlinker || isVagDebugBlinkerTest) {
    BlinkerRate -= 1;
    if(BlinkerRate < 0) {
      BlinkerRate = BLINKER_RATE;
    }
  }
}

#define BLINDSPOT_INFO_RATE 8
#define BLINDSPOT_WARNING_RATE 4
static int BlinkingInfoRate = BLINDSPOT_INFO_RATE;
static int BlinkingWarningRate = BLINDSPOT_WARNING_RATE;
void VagOsd::drawOsdBlindspot(QPainter &p) {
  UIState *s = uiState();

  const bool isVagDebugBlindspotInfoTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBlindspotInfoTest();
  const bool isVagDebugBlindspotWarningTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugBlindspotWarningTest();
  const bool leftBlindspot = (*s->sm)["carState"].getCarState().getLeftBlindspot();
  const bool rightBlindspot = (*s->sm)["carState"].getCarState().getRightBlindspot();
  const bool leftBlindspotWarning = (*s->sm)["carState"].getCarState().getLeftBlindspotWarning();
  const bool rightBlindspotWarning = (*s->sm)["carState"].getCarState().getRightBlindspotWarning();

  if (leftBlindspot || isVagDebugBlindspotInfoTest) {
    QPainterPath leftBlindspotInfoPath;
    leftBlindspotInfoPath.moveTo(0, 0);
    leftBlindspotInfoPath.lineTo(COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    leftBlindspotInfoPath.lineTo(COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    leftBlindspotInfoPath.lineTo(0, s->fb_h);
    leftBlindspotInfoPath.closeSubpath();
    p.fillPath(leftBlindspotInfoPath, (BlinkingInfoRate >= (BLINDSPOT_INFO_RATE / 2) ? COLOR_ORANGE_ALPHA(200) : COLOR_ORANGE_ALPHA(50)));
  }

  if (rightBlindspot || isVagDebugBlindspotInfoTest) {
    QPainterPath rightBlindspotInfoPath;
    rightBlindspotInfoPath.moveTo(s->fb_w, 0);
    rightBlindspotInfoPath.lineTo(s->fb_w - COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    rightBlindspotInfoPath.lineTo(s->fb_w - COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    rightBlindspotInfoPath.lineTo(s->fb_w, s->fb_h);
    rightBlindspotInfoPath.closeSubpath();
    p.fillPath(rightBlindspotInfoPath, (BlinkingInfoRate >= (BLINDSPOT_INFO_RATE / 2) ? COLOR_ORANGE_ALPHA(200) : COLOR_ORANGE_ALPHA(50)));
  }

  if (leftBlindspotWarning || isVagDebugBlindspotWarningTest) {
    QPainterPath leftBlindspotWarningPath;
    leftBlindspotWarningPath.moveTo(0, 0);
    leftBlindspotWarningPath.lineTo(COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    leftBlindspotWarningPath.lineTo(COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    leftBlindspotWarningPath.lineTo(0, s->fb_h);
    leftBlindspotWarningPath.closeSubpath();

    leftBlindspotWarningPath.moveTo(150, 150);
    leftBlindspotWarningPath.lineTo(150 + COLOR_BLOCK_SIZE, 150 + COLOR_BLOCK_SIZE);
    leftBlindspotWarningPath.lineTo(150 + COLOR_BLOCK_SIZE, s->fb_h - (150 + COLOR_BLOCK_SIZE));
    leftBlindspotWarningPath.lineTo(150, s->fb_h - 150);
    leftBlindspotWarningPath.closeSubpath();
    p.fillPath(leftBlindspotWarningPath, (BlinkingWarningRate >= (BLINDSPOT_WARNING_RATE / 2) ? COLOR_RED_ALPHA(200) : COLOR_RED_ALPHA(50)));
  }

  if (rightBlindspotWarning || isVagDebugBlindspotWarningTest) {
    QPainterPath rightBlindspotWarningPath;
    rightBlindspotWarningPath.moveTo(s->fb_w, 0);
    rightBlindspotWarningPath.lineTo(s->fb_w - COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    rightBlindspotWarningPath.lineTo(s->fb_w - COLOR_BLOCK_SIZE, s->fb_h - COLOR_BLOCK_SIZE);
    rightBlindspotWarningPath.lineTo(s->fb_w, s->fb_h);
    rightBlindspotWarningPath.closeSubpath();

    rightBlindspotWarningPath.moveTo(s->fb_w - 150, 150);
    rightBlindspotWarningPath.lineTo(s->fb_w - (150 + COLOR_BLOCK_SIZE), 150 + COLOR_BLOCK_SIZE);
    rightBlindspotWarningPath.lineTo(s->fb_w - (150 + COLOR_BLOCK_SIZE), s->fb_h - (150 + COLOR_BLOCK_SIZE));
    rightBlindspotWarningPath.lineTo(s->fb_w - 150, s->fb_h - 150);
    rightBlindspotWarningPath.closeSubpath();
    p.fillPath(rightBlindspotWarningPath, (BlinkingWarningRate >= (BLINDSPOT_WARNING_RATE / 2) ? COLOR_RED_ALPHA(200) : COLOR_RED_ALPHA(50)));
  }
  if(leftBlindspot || rightBlindspot || leftBlindspotWarning || rightBlindspotWarning || isVagDebugBlindspotInfoTest || isVagDebugBlindspotWarningTest) {
    BlinkingInfoRate -= 1;
    BlinkingWarningRate -= 1;
    if(BlinkingInfoRate < 0) {
      BlinkingInfoRate = BLINDSPOT_INFO_RATE;
    }
    if(BlinkingWarningRate < 0) {
      BlinkingWarningRate = BLINDSPOT_WARNING_RATE;
    }
  }
}

#define NO_LEAD_CAR_RATE 200
static int NoLeadCarRate = NO_LEAD_CAR_RATE;
void VagOsd::drawOsdNoLeadCar(QPainter &p) {
  UIState *s = uiState();

  const bool isVagDebugNoLeadCarWarningTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugNoLeadCarWarningTest();

#if 0
  //----- Vision -----
  const bool accEnable = (bool)(*s->sm)["carState"].getCarState().getCruiseState().getEnabled();
  auto leads = (*s->sm)["modelV2"].getModelV2().getLeadsV3();
  if((!(leads[0].getProb() > .5) || !(leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)))) {
#else
  //----- Radar -----
  const bool accEnable = (bool)(*s->sm)["carState"].getCarState().getCruiseState().getEnabled();
  const int accAbstandsindex = (int)(*s->sm)["carState"].getCarState().getVagUiField().getAccAbstandsindex();
  if((accEnable && accAbstandsindex == 0) || isVagDebugNoLeadCarWarningTest) {
#endif
    QPainterPath noLeadCarPath;
    noLeadCarPath.moveTo(350, 0);
    noLeadCarPath.lineTo(350 + COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    noLeadCarPath.lineTo(s->fb_w - (350 + COLOR_BLOCK_SIZE), COLOR_BLOCK_SIZE);
    noLeadCarPath.lineTo(s->fb_w - 350 , 0);
    noLeadCarPath.closeSubpath();
    p.fillPath(noLeadCarPath, COLOR_ORANGE_ALPHA(NoLeadCarRate));

    char noLeadCarBuffer[100];
    snprintf(noLeadCarBuffer, sizeof(noLeadCarBuffer), "No lead car");
    drawOsdText(p, 450, 0, s->fb_w - 900, 100, noLeadCarBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_CENTER));

    NoLeadCarRate = NoLeadCarRate - 40;
    if(NoLeadCarRate < 0) {
      NoLeadCarRate = NO_LEAD_CAR_RATE;
    }
  }
}

#define LEAD_CAR_GOING_RATE 200
static int LeadCarGoingRate = LEAD_CAR_GOING_RATE;
static int PreviousAccAbstandsindex = -1;
static int LeadCarGoingCount = 0;
void VagOsd::drawOsdLeadCarGoing(QPainter &p) {
  UIState *s = uiState();

  const bool isVagDebugLeadCarGoingRemindTest = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugLeadCarGoingRemindTest();
  const bool accEnable = (bool) (*s->sm)["carState"].getCarState().getCruiseState().getEnabled();
  const bool accAvailable = (bool) (*s->sm)["carState"].getCarState().getCruiseState().getAvailable();
  const int accAbstandsindex = (int) (*s->sm)["carState"].getCarState().getVagUiField().getAccAbstandsindex();
  const int vEgo = (int) (*s->sm)["carState"].getCarState().getVEgo();
  const bool gasPressed = (bool) (*s->sm)["carState"].getCarState().getGasPressed();
  const int gearShifter = (int) (*s->sm)["carState"].getCarState().getGearShifter();

  if(vEgo != 0) {
    LeadCarGoingCount = 0;
  }

  if(accAbstandsindex == PreviousAccAbstandsindex + 1) {
    LeadCarGoingCount++;
  } else if (accAbstandsindex == PreviousAccAbstandsindex) {
  } else {
    LeadCarGoingCount = 0;
  }

  if((accAvailable && \
     !accEnable && \
     !gasPressed && \
     vEgo == 0 && \
     LeadCarGoingCount > 1  && \
     (gearShifter == 2 || gearShifter == 5 || gearShifter == 8 || gearShifter == 9) && \
     accAbstandsindex > 0) || isVagDebugLeadCarGoingRemindTest) {
    QPainterPath leadCarGoingPath;
    leadCarGoingPath.moveTo(350, 0);
    leadCarGoingPath.lineTo(350 + COLOR_BLOCK_SIZE, COLOR_BLOCK_SIZE);
    leadCarGoingPath.lineTo(s->fb_w - (350 + COLOR_BLOCK_SIZE), COLOR_BLOCK_SIZE);
    leadCarGoingPath.lineTo(s->fb_w - 350 , 0);
    leadCarGoingPath.closeSubpath();
    p.fillPath(leadCarGoingPath, COLOR_YELLOW_ALPHA(LeadCarGoingRate));

    char leadCarGoingBuffer[100];
    snprintf(leadCarGoingBuffer, sizeof(leadCarGoingBuffer), "Lead car going");
    drawOsdText(p, 450, 0, s->fb_w - 900, 100, leadCarGoingBuffer, 10, COLOR_WHITE, (Qt::AlignmentFlag)(ALIGN_CENTER));

    LeadCarGoingRate = LeadCarGoingRate - 40;
    if(LeadCarGoingRate < 0) {
      LeadCarGoingRate = LEAD_CAR_GOING_RATE;
    }
  }
  PreviousAccAbstandsindex = accAbstandsindex;
}

void VagOsd::drawOsdInfobar(QPainter &p) {
}

void VagOsd::drawOsdTest(QPainter &p) {
  UIState *s = uiState();

  drawOsdText(p, 300, 100, 1500, 100, ("Screen: " +
                QString::number(s->fb_w) + " " +
                QString::number(s->fb_h)), 10, COLOR_YELLOW, ALIGN_LEFT);

  const float moAnsaugluftTemp = (*s->sm)["carState"].getCarState().getVagUiField().getMoAnsaugluftTemp();
  const int moOelTemp = (*s->sm)["carState"].getCarState().getVagUiField().getMoOelTemp();
  const float moItmKuehlmittelTemp = (*s->sm)["carState"].getCarState().getVagUiField().getObdEngCoolTemp();
  const int geSumpftemperatur = (*s->sm)["carState"].getCarState().getVagUiField().getGeSumpftemperatur();
  drawOsdText(p, 300, 200, 1500, 100, ("GearT: " +
                QString::number(geSumpftemperatur)), 10, COLOR_YELLOW, ALIGN_LEFT);
  drawOsdText(p, 1200, 200, 1500, 100, ("EngT: " +
                QString::number(moAnsaugluftTemp) + " " +
                QString::number(moOelTemp) + " " +
                QString::number(moItmKuehlmittelTemp)     ), 10, COLOR_YELLOW, ALIGN_LEFT);

  const int accAbstandsindex = (*s->sm)["carState"].getCarState().getVagUiField().getAccAbstandsindex();
  const float bcm1AussenTempUngef = (*s->sm)["carState"].getCarState().getVagUiField().getBcm1AussenTempUngef();
  const int geZielgang = (*s->sm)["carState"].getCarState().getVagUiField().getGeZielgang();
  //const float geSumpftemperatur = (*s->sm)["carState"].getCarState().getVagUiField().getGeSumpftemperatur();
  const float kbiAussenTempGef = (*s->sm)["carState"].getCarState().getVagUiField().getKbiAussenTempGef();
  //const float moAnsaugluftTemp = (*s->sm)["carState"].getCarState().getVagUiField().getMoAnsaugluftTemp();
  const float moKuehlmittelTemp = (*s->sm)["carState"].getCarState().getVagUiField().getMoKuehlmittelTemp();
  //const float moOelTemp = (*s->sm)["carState"].getCarState().getVagUiField().getMoOelTemp();
  //const float moItmKuehlmittelTemp = (*s->sm)["carState"].getCarState().getVagUiField().getMoItmKuehlmittelTemp();
  const float obdEngCoolTemp = (*s->sm)["carState"].getCarState().getVagUiField().getObdEngCoolTemp();
  drawOsdText(p, 300, 300, 1500, 100, ("VagUiField: " +
                  QString::number(accAbstandsindex) + " " +
                  QString::number(bcm1AussenTempUngef) + " " +
                  QString::number(geZielgang) + " " +
                  QString::number(geSumpftemperatur) + " " +
                  QString::number(kbiAussenTempGef) + " " +
                  QString::number(moAnsaugluftTemp) + " " +
                  QString::number(moKuehlmittelTemp) + " " +
                  QString::number(moOelTemp) + " " +
                  QString::number(moItmKuehlmittelTemp) + " " +
                  QString::number(obdEngCoolTemp)           ), 10, COLOR_YELLOW, ALIGN_LEFT);

  const bool ignitionLine = (*s->sm)["pandaStates"].getPandaStates()[0].getIgnitionLine();
  const bool gasInterceptorDetected = (*s->sm)["pandaStates"].getPandaStates()[0].getGasInterceptorDetected();
  const bool ignitionCan = (*s->sm)["pandaStates"].getPandaStates()[0].getIgnitionCan();
  const bool powerSaveEnabled = (*s->sm)["pandaStates"].getPandaStates()[0].getPowerSaveEnabled();
  const bool heartbeatLost = (*s->sm)["pandaStates"].getPandaStates()[0].getHeartbeatLost();
  const auto pandaType = (*s->sm)["pandaStates"].getPandaStates()[0].getPandaType();
  const auto faultStatus = (*s->sm)["pandaStates"].getPandaStates()[0].getFaultStatus();

  drawOsdText(p, 300, 400, 1500, 100, ("ignitionLine: " +
                QString::number(ignitionLine)), 10, COLOR_YELLOW, ALIGN_LEFT);
  drawOsdText(p, 300, 500, 1500, 100, ("gasInterceptorDetected: " +
                QString::number(gasInterceptorDetected)), 10, COLOR_YELLOW, ALIGN_LEFT);
  drawOsdText(p, 300, 600, 1500, 100, ("ignitionCan: " +
                QString::number(ignitionCan)), 10, COLOR_YELLOW, ALIGN_LEFT);
  drawOsdText(p, 300, 700, 1500, 100, ("powerSaveEnabled: " +
                QString::number(powerSaveEnabled)), 10, COLOR_YELLOW, ALIGN_LEFT);
  drawOsdText(p, 300, 800, 1500, 100, ("heartbeatLost: " +
                QString::number(heartbeatLost)), 10, COLOR_YELLOW, ALIGN_LEFT);
  if(pandaType==cereal::PandaState::PandaType::BLACK_PANDA) {
    drawOsdText(p, 300, 900, 1500, 100, ("pandaType: blackPanda"), 10, COLOR_YELLOW, ALIGN_LEFT);
  } else {
    drawOsdText(p, 300, 900, 1500, 100, ("pandaType: others"), 10, COLOR_YELLOW, ALIGN_LEFT);
  }
  if(faultStatus==cereal::PandaState::FaultStatus::NONE) {
    drawOsdText(p, 300, 1000, 1500, 100, ("FaultStatus: none"), 10, COLOR_YELLOW, ALIGN_LEFT);
  } else if(faultStatus==cereal::PandaState::FaultStatus::FAULT_TEMP) {
    drawOsdText(p, 300, 1000, 1500, 100, ("FaultStatus: faultTemp"), 10, COLOR_YELLOW, ALIGN_LEFT);
  } else if(faultStatus==cereal::PandaState::FaultStatus::FAULT_PERM) {
    drawOsdText(p, 300, 1000, 1500, 100, ("FaultStatus: faultPerm"), 10, COLOR_YELLOW, ALIGN_LEFT);
  } else {
    drawOsdText(p, 300, 1000, 1500, 100, ("FaultStatus: unknown"), 10, COLOR_YELLOW, ALIGN_LEFT);
  }
}

void VagOsd::drawOsd(QPainter &p) {
  UIState *s = uiState();
#if 0
  const bool isVagInfoBoxEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagInfoBoxEnabled();
  if(isVagInfoBoxEnabled) {
    drawOsdInfobox(p);
  }
#endif
  const bool isVagBrakeLightEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagBrakeLightEnabled();
  if(isVagBrakeLightEnabled) {
    drawOsdBrakeLight(p);
  }

  const bool isVagBlinkerEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamSetting().getIsVagBlinkerEnabled();
  if(isVagBlinkerEnabled) {
    drawOsdBlinker(p);
  }

  const bool isVagBlindspotEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagBlindspotEnabled();
  if(isVagBlindspotEnabled) {
    drawOsdBlindspot(p);
  }

  const bool isVagNoLeadCarEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagNoLeadCarEnabled();
  const bool experimentalLongitudinalEnabled = (bool)(*s->sm)["vagParam"].getVagParam().getVagParamOp().getExperimentalLongitudinalEnabled();
  if(isVagNoLeadCarEnabled && experimentalLongitudinalEnabled) {
    drawOsdNoLeadCar(p);
  }

  const bool isVagLeadCarGoingRemindEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamFeature().getIsVagLeadCarGoingRemindEnabled();
  if(isVagLeadCarGoingRemindEnabled) {
    drawOsdLeadCarGoing(p);
  }

  const bool isVagDebugOsdTestTextEnabled = (*s->sm)["vagParam"].getVagParam().getVagParamTest().getIsVagDebugOsdTestTextEnabled();
  if(isVagDebugOsdTestTextEnabled) {
    drawOsdTest(p);
  }
}

