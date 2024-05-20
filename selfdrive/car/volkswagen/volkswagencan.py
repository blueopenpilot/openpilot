#
# Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

# CAN controls for MQB platform Volkswagen, Audi, Skoda and SEAT.
# PQ35/PQ46/NMS, and any future MLB, to come later.

from cereal import car

def create_mqb_steering_control(packer, bus, apply_steer, idx, lkas_enabled, vibrator_enable):
  if vibrator_enable == 2:
    vibratorFreq = 0xF
    vibratorAmp = 0x3
  elif vibrator_enable == 1:
    vibratorFreq = 0x7
    vibratorAmp = 0x1
  else:
    vibratorFreq = 0x3
    vibratorAmp = 0x0

  values = {
    #"SET_ME_0X3": 0x3,
    "SET_ME_0X3": vibratorFreq,
    "Assist_Torque": abs(apply_steer),
    "Assist_Requested": lkas_enabled,
    "Assist_VZ": 1 if apply_steer < 0 else 0,
    "HCA_Available": 1,
    "HCA_Standby": 1 if (not (lkas_enabled or vibrator_enable)) else 0,
    "HCA_Active": 1 if (lkas_enabled or vibrator_enable) else 0,
    "HCA_01_Vib_Amp": vibratorAmp,
    "SET_ME_0XFE": 0xFE,
    "SET_ME_0X07": 0x07,
  }
  return packer.make_can_msg("HCA_01", bus, values, idx)

def create_mqb_hud_control(packer, bus, enabled, steering_pressed, hud_alert, left_lane_visible, right_lane_visible,
                           ldw_stock_values, left_lane_depart, right_lane_depart):
  # Lane color reference:
  # 0 (LKAS disabled) - off
  # 1 (LKAS enabled, no lane detected) - dark gray
  # 2 (LKAS enabled, lane detected) - light gray on VW, green or white on Audi depending on year or virtual cockpit.  On a color MFD on a 2015 A3 TDI it is white, virtual cockpit on a 2018 A3 e-Tron its green.
  # 3 (LKAS enabled, lane departure detected) - white on VW, red on Audi
  values = ldw_stock_values.copy()
  values.update({
    "LDW_Status_LED_gelb": 1 if enabled and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if enabled and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if enabled and left_lane_visible else 1 + left_lane_visible,
    "LDW_Lernmodus_rechts": 3 if enabled and right_lane_visible else 1 + right_lane_visible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)

def create_mqb_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Hauptschalter": CS.graHauptschalter,
    "GRA_Abbrechen": buttonStatesToSend["cancel"],
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"],
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
    "GRA_Codierung": 2,
    "GRA_Tip_Stufe_2": CS.graTipStufe2,
    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }
  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)

def create_bcm_01_control(packer, bus, bcm_01_value, disableVagStartStop):
  values = bcm_01_value

  values.update({
    "BCM_Hybrid_StartStopp_Taste": disableVagStartStop,
  })

  return packer.make_can_msg("BCM_01", bus, values)


def create_charisma_01_control(packer, bus, charisma_01_value, charisma_07_value, enableVagDrivingMode, vagDrivingMode, enableVagDynamicDcc, speed, steeringAngleDeg):
  values = charisma_01_value

  if enableVagDrivingMode:
    if vagDrivingMode == 5: #eco
      values.update({
        "CHA_Ziel_FahrPr_ESP": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_FL": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 5,
        "CHA_Ziel_FahrPr_MO": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 5,
        "CHA_Ziel_FahrPr_GE": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 5,
        "CHA_Ziel_FahrPr_DR": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_AFS": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_EPS": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_ACC": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 5,
        "CHA_Ziel_FahrPr_MStSt": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 5,
      })
    elif vagDrivingMode == 1: #comfort
      values.update({
        "CHA_Ziel_FahrPr_ESP": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_FL": 0 if charisma_01_value["CHA_Ziel_FahrPr_FL"] == 0 else 2,
        "CHA_Ziel_FahrPr_MO": 0 if charisma_01_value["CHA_Ziel_FahrPr_MO"] == 0 else 2,
        "CHA_Ziel_FahrPr_GE": 0 if charisma_01_value["CHA_Ziel_FahrPr_GE"] == 0 else 1,
        "CHA_Ziel_FahrPr_DR": 0 if charisma_01_value["CHA_Ziel_FahrPr_DR"] == 0 else 2,
        "CHA_Ziel_FahrPr_AFS": 0 if charisma_01_value["CHA_Ziel_FahrPr_AFS"] == 0 else 2,
        "CHA_Ziel_FahrPr_EPS": 0 if charisma_01_value["CHA_Ziel_FahrPr_EPS"] == 0 else 2,
        "CHA_Ziel_FahrPr_ACC": 0 if charisma_01_value["CHA_Ziel_FahrPr_ACC"] == 0 else 1,
        "CHA_Ziel_FahrPr_MStSt": 0 if charisma_01_value["CHA_Ziel_FahrPr_MStSt"] == 0 else 0,
      })
    elif vagDrivingMode == 2: #normal
      values.update({
        "CHA_Ziel_FahrPr_ESP": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_FL": 0 if charisma_01_value["CHA_Ziel_FahrPr_FL"] == 0 else 2,
        "CHA_Ziel_FahrPr_MO": 0 if charisma_01_value["CHA_Ziel_FahrPr_MO"] == 0 else 2,
        "CHA_Ziel_FahrPr_GE": 0 if charisma_01_value["CHA_Ziel_FahrPr_GE"] == 0 else 2,
        "CHA_Ziel_FahrPr_DR": 0 if charisma_01_value["CHA_Ziel_FahrPr_DR"] == 0 else 2,
        "CHA_Ziel_FahrPr_AFS": 0 if charisma_01_value["CHA_Ziel_FahrPr_AFS"] == 0 else 2,
        "CHA_Ziel_FahrPr_EPS": 0 if charisma_01_value["CHA_Ziel_FahrPr_EPS"] == 0 else 2,
        "CHA_Ziel_FahrPr_ACC": 0 if charisma_01_value["CHA_Ziel_FahrPr_ACC"] == 0 else 2,
        "CHA_Ziel_FahrPr_MStSt": 0 if charisma_01_value["CHA_Ziel_FahrPr_MStSt"] == 0 else 0,
      })
    elif vagDrivingMode == 3: #sport
      values.update({
        "CHA_Ziel_FahrPr_ESP": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_FL": 0 if charisma_01_value["CHA_Ziel_FahrPr_FL"] == 0 else 2,
        "CHA_Ziel_FahrPr_MO": 0 if charisma_01_value["CHA_Ziel_FahrPr_MO"] == 0 else 3,
        "CHA_Ziel_FahrPr_GE": 0 if charisma_01_value["CHA_Ziel_FahrPr_GE"] == 0 else 3,
        "CHA_Ziel_FahrPr_DR": 0 if charisma_01_value["CHA_Ziel_FahrPr_DR"] == 0 else 3,
        "CHA_Ziel_FahrPr_AFS": 0 if charisma_01_value["CHA_Ziel_FahrPr_AFS"] == 0 else 3,
        "CHA_Ziel_FahrPr_EPS": 0 if charisma_01_value["CHA_Ziel_FahrPr_EPS"] == 0 else 3,
        "CHA_Ziel_FahrPr_ACC": 0 if charisma_01_value["CHA_Ziel_FahrPr_ACC"] == 0 else 3,
        "CHA_Ziel_FahrPr_MStSt": 0 if charisma_01_value["CHA_Ziel_FahrPr_MStSt"] == 0 else 0,
      })
    elif vagDrivingMode == 6: #race
      values.update({
        "CHA_Ziel_FahrPr_ESP": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 2,
        "CHA_Ziel_FahrPr_FL": 0 if charisma_01_value["CHA_Ziel_FahrPr_FL"] == 0 else 2,
        "CHA_Ziel_FahrPr_MO": 0 if charisma_01_value["CHA_Ziel_FahrPr_MO"] == 0 else 6,
        "CHA_Ziel_FahrPr_GE": 0 if charisma_01_value["CHA_Ziel_FahrPr_GE"] == 0 else 6,
        "CHA_Ziel_FahrPr_DR": 0 if charisma_01_value["CHA_Ziel_FahrPr_DR"] == 0 else 6,
        "CHA_Ziel_FahrPr_AFS": 0 if charisma_01_value["CHA_Ziel_FahrPr_AFS"] == 0 else 6,
        "CHA_Ziel_FahrPr_EPS": 0 if charisma_01_value["CHA_Ziel_FahrPr_EPS"] == 0 else 6,
        "CHA_Ziel_FahrPr_ACC": 0 if charisma_01_value["CHA_Ziel_FahrPr_ACC"] == 0 else 6,
        "CHA_Ziel_FahrPr_MStSt": 0 if charisma_01_value["CHA_Ziel_FahrPr_MStSt"] == 0 else 0,
      })
    elif vagDrivingMode == 10: #snow
      values.update({
        "CHA_Ziel_FahrPr_ESP": 0 if charisma_01_value["CHA_Ziel_FahrPr_ESP"] == 0 else 10,
        "CHA_Ziel_FahrPr_FL": 0 if charisma_01_value["CHA_Ziel_FahrPr_FL"] == 0 else 2,
        "CHA_Ziel_FahrPr_MO": 0 if charisma_01_value["CHA_Ziel_FahrPr_MO"] == 0 else 4,
        "CHA_Ziel_FahrPr_GE": 0 if charisma_01_value["CHA_Ziel_FahrPr_GE"] == 0 else 10,
        "CHA_Ziel_FahrPr_DR": 0 if charisma_01_value["CHA_Ziel_FahrPr_DR"] == 0 else 2,
        "CHA_Ziel_FahrPr_AFS": 0 if charisma_01_value["CHA_Ziel_FahrPr_AFS"] == 0 else 3,
        "CHA_Ziel_FahrPr_EPS": 0 if charisma_01_value["CHA_Ziel_FahrPr_EPS"] == 0 else 3,
        "CHA_Ziel_FahrPr_ACC": 0 if charisma_01_value["CHA_Ziel_FahrPr_ACC"] == 0 else 5,
        "CHA_Ziel_FahrPr_MStSt": 0 if charisma_01_value["CHA_Ziel_FahrPr_MStSt"] == 0 else 0,
      })

  if enableVagDynamicDcc:
    if charisma_07_value["CHA_Current_Mode"] == 1 or charisma_07_value["CHA_Current_Mode"] == 2:
      #speed
      if speed > 80:
        values.update({
            "CHA_Ziel_FahrPr_DR": 3,
          })
      elif speed > 40 and speed <= 80:
        values.update({
            "CHA_Ziel_FahrPr_DR": 2,
          })
      elif speed <= 40:
        values.update({
            "CHA_Ziel_FahrPr_DR": charisma_07_value["CHA_Current_Mode"],
          })
      #steeringAngleDeg
      if steeringAngleDeg > 30:
        values.update({
            "CHA_Ziel_FahrPr_DR": 3,
          })
      elif steeringAngleDeg > 15 and steeringAngleDeg <= 30:
        values.update({
            "CHA_Ziel_FahrPr_DR": 2,
          })
      elif steeringAngleDeg <= 15:
        values.update({
            "CHA_Ziel_FahrPr_DR": charisma_07_value["CHA_Current_Mode"],
          })
  return packer.make_can_msg("Charisma_01", bus, values)
