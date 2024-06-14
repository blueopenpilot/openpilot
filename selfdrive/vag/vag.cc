/*
 * Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng.
 * All Rights Reserved.
 * Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng.
 */

#include <cassert>
#include <stdio.h>

#include "cereal/gen/cpp/car.capnp.h"
#include "cereal/gen/cpp/log.capnp.h"
#include "cereal/gen/cpp/custom.capnp.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "selfdrive/vag/vag.h"



static int PreviousAccAbstandsindex = -1;
static int LeadCarGoingCount = 0;

void vag_main_thread(std::vector<std::string> serials) {
  LOGW("launching vagd");

  SubMaster sm({"carState", "vagParam"});
  PubMaster pm({"vagControl"});

  while(true) {
    // build msg
    MessageBuilder msg;
    auto vagControl = msg.initEvent().initVagControl();

    //lead car going
    sm.update(0);
    const bool isVagDebugLeadCarGoingRemindTest = (bool)sm["vagParam"].getVagParam().getVagParamOsd().getIsVagDebugLeadCarGoingRemindTest();
    const bool accEnable = (bool) sm["carState"].getCarState().getCruiseState().getEnabled();
    const bool accAvailable = (bool) sm["carState"].getCarState().getCruiseState().getAvailable();
    const int accAbstandsindex = (int) sm["carState"].getCarState().getVagCarState().getVagUiField().getAccAbstandsindex();
    const int vEgo = (int) sm["carState"].getCarState().getVEgo();
    const bool gasPressed = (bool) sm["carState"].getCarState().getGasPressed();
    const int gearShifter = (int) sm["carState"].getCarState().getGearShifter();

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
       printf("[PONTEST][%s][%s][%d] \n", __FILE__, __FUNCTION__, __LINE__);
       vagControl.setLeadCarGoingTrigged(true);
    }
    PreviousAccAbstandsindex = accAbstandsindex;

    pm.send("vagControl", msg);

    //Pon: TODO: realtime process frequency keep (100Hz)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
