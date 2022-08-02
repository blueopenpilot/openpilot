#
# Copyright (c) 2020-2022 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

# CAN controls for MQB platform Volkswagen, Audi, Skoda and SEAT.
# PQ35/PQ46/NMS, and any future MLB, to come later.

from common.params import Params, put_nonblocking

def create_mqb_steering_control(packer, bus, apply_steer, idx, lkas_enabled, vibrator_enable):
  #print("[BOP][volkswagencan.py][create_mqb_steering_control()][Blindspot] vibrator_enable=", vibrator_enable)
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
  #isVagParamFromCerealEnabled = self.sm['vagParam'].isVagParamFromCerealEnabled
  #if isVagParamFromCerealEnabled:
  #  isVagFlkaLogEnabled = self.sm['vagParam'].isVagFlkaLogEnabled
  #else :
  #  params = Params()
  #  try:
  #    isVagFlkaLogEnabled = params.get_bool("IsVagFlkaLogEnabled")
  #  except:
  #    print("[BOP][volkswagencan.py][create_mqb_hud_control()][IsVagFlkaLogEnabled] Get param exception")
  #    isVagFlkaLogEnabled = False

  #if isVagFlkaLogEnabled:
  #  print("[BOP][volkswagencan.py][create_mqb_hud_control()][FLKA] enabled=", enabled)

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
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],#//acc -1
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],#//acc +10
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],#//acc -10
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"], #//acc +1
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
    "GRA_Codierung": 2,
    "GRA_Tip_Stufe_2": CS.graTipStufe2,
    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }
  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)
