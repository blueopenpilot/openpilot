#
# Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

from cereal import car

def create_steering_control(packer, bus, apply_steer, lkas_enabled, vibrator_enable):
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
    "HCA_01_Status_HCA": 5 if (lkas_enabled or vibrator_enable) else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": vibratorFreq if vibrator_enable else 18,
    "HCA_01_Sendestatus": 1 if (lkas_enabled or vibrator_enable) else 0,
    "HCA_01_Vib_Amp": vibratorAmp if vibrator_enable else 0,
    "EA_ACC_Wunschgeschwindigkeit": 327.36,
  }
  return packer.make_can_msg("HCA_01", bus, values)


def create_eps_update(packer, bus, eps_stock_values, ea_simulated_torque):
  values = {s: eps_stock_values[s] for s in [
    "COUNTER",                     # Sync counter value to EPS output
    "EPS_Lenkungstyp",             # EPS rack type
    "EPS_Berechneter_LW",          # Absolute raw steering angle
    "EPS_VZ_BLW",                  # Raw steering angle sign
    "EPS_HCA_Status",              # EPS HCA control status
  ]}

  values.update({
    # Absolute driver torque input and sign, with EA inactivity mitigation
    "EPS_Lenkmoment": abs(ea_simulated_torque),
    "EPS_VZ_Lenkmoment": 1 if ea_simulated_torque < 0 else 0,
  })

  return packer.make_can_msg("LH_EPS_03", bus, values)


def create_lka_hud_control(packer, bus, ldw_stock_values, lat_active, steering_pressed, hud_alert, hud_control):
  values = {}
  if len(ldw_stock_values):
    values = {s: ldw_stock_values[s] for s in [
      "LDW_SW_Warnung_links",   # Blind spot in warning mode on left side due to lane departure
      "LDW_SW_Warnung_rechts",  # Blind spot in warning mode on right side due to lane departure
      "LDW_Seite_DLCTLC",       # Direction of most likely lane departure (left or right)
      "LDW_DLC",                # Lane departure, distance to line crossing
      "LDW_TLC",                # Lane departure, time to line crossing
    ]}

  values.update({
    "LDW_Status_LED_gelb": 1 if lat_active and steering_pressed else 0,
    "LDW_Status_LED_gruen": 1 if lat_active and not steering_pressed else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
    "LDW_Texte": hud_alert,
  })
  return packer.make_can_msg("LDW_02", bus, values)


def create_acc_buttons_control(packer, bus, gra_stock_values, cancel=False, resume=False):
  values = {s: gra_stock_values[s] for s in [
    "GRA_Hauptschalter",           # ACC button, on/off
    "GRA_Typ_Hauptschalter",       # ACC main button type
    "GRA_Codierung",               # ACC button configuration/coding
    "GRA_Tip_Stufe_2",             # unknown related to stalk type
    "GRA_ButtonTypeInfo",          # unknown related to stalk type
  ]}

  values.update({
    "COUNTER": (gra_stock_values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Tip_Wiederaufnahme": resume,
  })

  return packer.make_can_msg("GRA_ACC_01", bus, values)


def acc_control_value(main_switch_on, acc_faulted, long_active):
  if acc_faulted:
    acc_control = 6
  elif long_active:
    acc_control = 3
  elif main_switch_on:
    acc_control = 2
  else:
    acc_control = 0

  return acc_control


def acc_hud_status_value(main_switch_on, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init/gas override later
  return acc_control_value(main_switch_on, acc_faulted, long_active)


def create_acc_accel_control(packer, bus, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  commands = []

  acc_06_values = {
    "ACC_Typ": acc_type,
    "ACC_Status_ACC": acc_control,
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_zul_Regelabw_unten": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_zul_Regelabw_oben": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  }
  commands.append(packer.make_can_msg("ACC_06", bus, acc_06_values))

  if starting:
    acc_hold_type = 4  # hold release / startup
  elif esp_hold:
    acc_hold_type = 3  # hold standby
  elif stopping:
    acc_hold_type = 1  # hold request
  else:
    acc_hold_type = 0

  acc_07_values = {
    "ACC_Anhalteweg": 0.3 if stopping else 20.46,  # Distance to stop (stopping coordinator handles terminal roll-out)
    "ACC_Freilauf_Info": 2 if acc_enabled else 0,
    "ACC_Folgebeschl": 3.02,  # Not using secondary controller accel unless and until we understand its impact
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_Anforderung_HMS": acc_hold_type,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  }
  commands.append(packer.make_can_msg("ACC_07", bus, acc_07_values))

  return commands


def create_acc_hud_control(packer, bus, acc_hud_status, set_speed, lead_distance, distance):
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Gesetzte_Zeitluecke": distance + 2,
    "ACC_Display_Prio": 3,
    "ACC_Abstandsindex": lead_distance,
  }

  return packer.make_can_msg("ACC_02", bus, values)


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

def create_blinkmodi_02_control(packer, bus, blinkmodi_02_value, left_blinker, right_blinker):
  values = blinkmodi_02_value

  if left_blinker:
    values.update({
      #"Hazard_Switch": 1,
      "Comfort_Signal_Left": 1,
      #"Left_Turn_Exterior_Bulb_1": 1,
      #"Left_Turn_Exterior_Bulb_2": 1,
      "Fast_Send_Rate_Active": 1,
    })

  if right_blinker:
    values.update({
      #"Hazard_Switch": 1,
      "Comfort_Signal_Right": 1,
      #"Right_Turn_Exterior_Bulb_1": 1,
      #"Right_Turn_Exterior_Bulb_2": 1,
      "Fast_Send_Rate_Active": 1,
    })
  return packer.make_can_msg("Blinkmodi_02", bus, values)

def create_licht_anf_01_control(packer, bus, licht_anf_01_value, left_blinker, right_blinker):
  values = licht_anf_01_value

  if left_blinker:
    values.update({
      "BCM1_Kurvenlicht_links_Anf": 1,
    })

  if right_blinker:
    values.update({
      "BCM1_Kurvenlicht_rechts_Anf": 1,
    })

  return packer.make_can_msg("Licht_Anf_01", bus, values)

def create_gateway_72_control(packer, bus, gateway_72_value, left_blinker, right_blinker):
  values = gateway_72_value

  if left_blinker:
    values.update({
      "BH_Blinker_li": 1,
    })

  if right_blinker:
    values.update({
      "BH_Blinker_re": 1,
    })

  return packer.make_can_msg("Gateway_72", bus, values)

def create_parkhilfe_01_control(packer, bus, parkhilfe_01_value):
  values = parkhilfe_01_value

  values.update({
    "PH_Abschaltursache": 0, #VAL_ 1175 PH_Abschaltursache 0 "keine, bzw. System aktiv" 1 "Abschaltung durch Herausnahme R-Gang" 2 "Abschaltung ueber Geschwindigkeit" 3 "Abschaltung ueber PDC-Taster" 4 "Abschaltung wegen PLA" 5 "Abschaltung durch KL 15 Bit = 0" 6 "tbd." 7 "Fehler-/gest�rt Zustand, System noch aktiv";
    "PH_Tongeber_mute": 1,
    "PH_Taster": 1, #VAL_ 1175 PH_Taster 0 "Taster_nicht_gedrueckt" 1 "Taster_gedrueckt" 2 "reserviert" 3 "Fehler";
  })
  return packer.make_can_msg("Parkhilfe_01", bus, values)
