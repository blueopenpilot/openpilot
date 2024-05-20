#
# Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, NetworkLocation, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams, CAR

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC_FILES.mqb)
    if CP.transmissionType == TransmissionType.automatic:
      self.shifter_values = can_define.dv["Getriebe_11"]["GE_Fahrstufe"]
    elif CP.transmissionType == TransmissionType.direct:
      self.shifter_values = can_define.dv["EV_Gearshift"]["GearPosition"]
    self.hca_status_values = can_define.dv["LH_EPS_03"]["EPS_HCA_Status"]
    self.buttonStates = BUTTON_STATES.copy()

  def update(self, pt_cp, cam_cp, ext_cp, body_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["ESP_19"]["ESP_VL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_VR_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HL_Radgeschw_02"],
      pt_cp.vl["ESP_19"]["ESP_HR_Radgeschw_02"],
    )

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    #Pon Fix stop and go acc resume +1
    ret.standstill = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"]) and ret.vEgo < 0.01

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["ESP_02"]["ESP_Gierrate"] * (1, -1)[int(pt_cp.vl["ESP_02"]["ESP_VZ_Gierrate"])] * CV.DEG_TO_RAD

    # Verify EPS readiness to accept steering commands
    hca_status = self.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerError = hca_status in ("DISABLED", "FAULT")
    ret.steerWarning = hca_status in ("INITIALIZING", "REJECTED")

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    self.esp_hold_confirmation = pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"]
    ret.brakeLights = bool(pt_cp.vl["ESP_05"]["ESP_Status_Bremsdruck"])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
    elif trans_type == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["EV_Gearshift"]["GearPosition"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_14"]["MO_Kuppl_schalter"]
      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gateway_72"]["ZV_FT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_BT_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HBFS_offen"],
                        pt_cp.vl["Gateway_72"]["ZV_HD_offen"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Einheiten_01"]["KBI_MFA_v_Einheit_02"]

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"])
      ret.leftBlindspotWarning = bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspotWarning = bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    ret.brakeLights = bool(pt_cp.vl["ESP_05"]["ESP_Status_Bremsdruck"])

    #VAG
    # ----- Motor_04 -----
    try:
      ret.vagUiField.moIstgang                  = body_cp.vl["Motor_04"]["MO_Istgang"]
      ret.vagUiField.moLadedruck                = body_cp.vl["Motor_04"]["MO_Ladedruck"] #0~5.10 Bar
      ret.vagUiField.moOeldruck                 = body_cp.vl["Motor_04"]["MO_Oeldruck"] #0~10.00 Bar
      ret.vagCanModule.bus1Motor04 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_04 fail")
      ret.vagCanModule.bus1Motor04 = False;


    # ----- Motor_07 -----
    try:
      ret.vagUiField.moAnsaugluftTemp           = pt_cp.vl["Motor_07"]["MO_Ansaugluft_Temp"] #-48~141.75 DegreCelsi
      ret.vagUiField.moKuehlmittelTemp          = pt_cp.vl["Motor_07"]["MO_Kuehlmittel_Temp"] #-48~141.75 DegreCelsi
      ret.vagUiField.moOelTemp                  = pt_cp.vl["Motor_07"]["MO_Oel_Temp"] #-60~192 DegreCelsi
      ret.vagCanModule.bus0Motor07 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_07 fail")
      ret.vagCanModule.bus0Motor07 = False;

    # ----- Motor_09 -----
    #if self.CP.vagCanModule.bus1Motor09:
    try:
      ret.vagUiField.moItmKuehlmittelTemp       = body_cp.vl["Motor_09"]["MO_ITM_Kuehlmittel_Temp"] #-45.75~143.25 DegreCelsi
      ret.vagCanModule.bus1Motor09 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_09 fail")
      ret.vagCanModule.bus1Motor09 = False;

    # ----- Motor_18 -----
    #if self.CP.vagCanModule.bus0Motor18:
    try:
      ret.vagUiField.moMaxLadedruck             = pt_cp.vl["Motor_18"]["MO_max_Ladedruck"] #0~6.3 Bar
      ret.vagCanModule.bus0Motor18 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_18 fail")
      ret.vagCanModule.bus0Motor18 = False;

    # ----- Motor_20 -----
    ret.vagUiField.moRelSaugrohrdruck           = pt_cp.vl["Motor_20"]["MO_rel_Saugrohrdruck"] #0~1.116 Bar
    ret.vagUiField.moRelSaugrohrdruckGemErr     = pt_cp.vl["Motor_20"]["MO_rel_Saugrohrdruck_gem_err"]

    # ----- Getriebe_11 -----
    ret.vagUiField.geZielgang                   = pt_cp.vl["Getriebe_11"]["GE_Zielgang"]

    # ----- Getriebe_14 -----
    #if self.CP.vagCanModule.bus1Getriebe14:
    try:
      ret.vagUiField.geSumpftemperatur          = body_cp.vl["Getriebe_14"]["GE_Sumpftemperatur"] #-58~196 DegreCelsi
      ret.vagCanModule.bus1Getriebe14 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Getriebe_14 fail")
      ret.vagCanModule.bus1Getriebe14 = False;

    # ----- ESP_05 -----
    ret.vagUiField.espBremsdruck                = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] #-30~276.6 Bar
    ret.vagUiField.espBvkUnterdruck             = pt_cp.vl["ESP_05"]["ESP_BKV_Unterdruck"] #0~1.012 Bar

    # ----- Gateway_72 -----
    ret.vagUiField.bcm1AussenTempUngef          = pt_cp.vl["Gateway_72"]["BCM1_Aussen_Temp_ungef"] #-50~76.0 DegreCelsi

    # ----- OBD_01 -----
    #if self.CP.vagCanModule.bus1Obd01:
    try:
      ret.vagUiField.obdEngCoolTemp             = body_cp.vl["OBD_01"]["OBD_Eng_Cool_Temp"] #-40~215 DegreCelsi
      ret.vagCanModule.bus1Obd01 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] OBD_01 fail")
      ret.vagCanModule.bus1Obd01 = False;

    # ----- Kombi_02 -----
    #if self.CP.vagCanModule.bus0Kombi02:
    try:
      ret.vagUiField.kbiAussenTempGef           = pt_cp.vl["Kombi_02"]["KBI_Aussen_Temp_gef"] #-50~75.0 DegreCelsi
      ret.vagCanModule.bus0Kombi02 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Kombi_02 fail")
      ret.vagCanModule.bus0Kombi02 = False;

    # ----- ACC_02 -----
    ret.vagUiField.accAbstandsindex             = ext_cp.vl["ACC_02"]["ACC_Abstandsindex"]

    # ----- VehicleSpeed -----
    #if self.CP.vagCanModule.bus1Motor04:
    try:
      ret.vagUiField.speed                      = pt_cp.vl["VehicleSpeed"]["Speed"] #Km/H
      ret.vagCanModule.bus0VehicleSpeed = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] VehicleSpeed fail")
      ret.vagCanModule.bus0VehicleSpeed = False;

    # ----- Motor_12 -----
    #if self.CP.vagCanModule.bus1Motor12:
    try:
      ret.engineRpm                             = body_cp.vl["Motor_12"]["MO_Drehzahl_01"]
      ret.vagCanModule.bus1Motor12 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_12 fail")
      ret.vagCanModule.bus1Motor12 = False;


    ##### VAG Force disable startstop #####
    try:
      self.bcm_01 = pt_cp.vl["BCM_01"]
      ret.vagCanModule.bus0Bcm01 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] BCM_01 fail")
      ret.vagCanModule.bus0Bcm01 = False;

    try:
      self.motor_18 = pt_cp.vl["Motor_18"]
      ret.vagCanModule.bus0Motor18 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_18 fail")
      ret.vagCanModule.bus0Motor18 = False;

    ##### VAG Driving mode #####
    try:
      self.charisma_01 = pt_cp.vl["Charisma_01"]
      ret.vagCanModule.bus0Charisma01 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Charisma_01 fail")
      ret.vagCanModule.bus0Charisma01 = False;

    try:
      self.charisma_07 = pt_cp.vl["Charisma_07"]
      ret.vagCanModule.bus0Charisma07 = True;
    except:
      print("[BOP][carstate.py][get_can_parser()] Charisma_07 fail")
      ret.vagCanModule.bus0Charisma07 = False;

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapter on Front Assist with Braking: Golf Family for all MQB
    ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
    ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

    # Update ACC radar status.
    self.tsk_status = pt_cp.vl["TSK_06"]["TSK_Status"]
    if self.tsk_status == 2:
      # ACC okay and enabled, but not currently engaged
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
    elif self.tsk_status in (3, 4, 5):
      # ACC okay and enabled, currently regulating speed (3) or driver accel override (4) or overrun coast-down (5)
      ret.cruiseState.available = True
      ret.cruiseState.enabled = True
    else:
      # ACC okay but disabled (1), or a radar visibility or other fault/disruption (6 or 7)
      ret.cruiseState.available = False
      ret.cruiseState.enabled = False

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    if self.CP.pcmCruise:
      ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw"] * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:
        ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Hoch"])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Runter"])
    self.buttonStates["cancel"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Abbrechen"])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Setzen"])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Wiederaufnahme"])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Verstellung_Zeitluecke"])
    ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
    ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    self.graHauptschalter = pt_cp.vl["GRA_ACC_01"]["GRA_Hauptschalter"]
    self.graTypHauptschalter = pt_cp.vl["GRA_ACC_01"]["GRA_Typ_Hauptschalter"]
    self.graButtonTypeInfo = pt_cp.vl["GRA_ACC_01"]["GRA_ButtonTypeInfo"]
    self.graTipStufe2 = pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Stufe_2"]
    # Pick up the GRA_ACC_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    self.graMsgBusCounter = pt_cp.vl["GRA_ACC_01"]["COUNTER"]

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("LWI_Lenkradwinkel", "LWI_01"),           # Absolute steering angle
      ("LWI_VZ_Lenkradwinkel", "LWI_01"),        # Steering angle sign
      ("LWI_Lenkradw_Geschw", "LWI_01"),         # Absolute steering rate
      ("LWI_VZ_Lenkradw_Geschw", "LWI_01"),      # Steering rate sign
      ("ESP_VL_Radgeschw_02", "ESP_19"),         # ABS wheel speed, front left
      ("ESP_VR_Radgeschw_02", "ESP_19"),         # ABS wheel speed, front right
      ("ESP_HL_Radgeschw_02", "ESP_19"),         # ABS wheel speed, rear left
      ("ESP_HR_Radgeschw_02", "ESP_19"),         # ABS wheel speed, rear right
      ("ESP_Gierrate", "ESP_02"),                # Absolute yaw rate
      ("ESP_VZ_Gierrate", "ESP_02"),             # Yaw rate sign
      ("ZV_FT_offen", "Gateway_72"),             # Door open, driver
      ("ZV_BT_offen", "Gateway_72"),             # Door open, passenger
      ("ZV_HFS_offen", "Gateway_72"),            # Door open, rear left
      ("ZV_HBFS_offen", "Gateway_72"),           # Door open, rear right
      ("ZV_HD_offen", "Gateway_72"),             # Trunk or hatch open
      ("BCM1_Aussen_Temp_ungef", "Gateway_72"),
      ("Comfort_Signal_Left", "Blinkmodi_02"),   # Left turn signal including comfort blink interval
      ("Comfort_Signal_Right", "Blinkmodi_02"),  # Right turn signal including comfort blink interval
      ("AB_Gurtschloss_FA", "Airbag_02"),        # Seatbelt status, driver
      ("AB_Gurtschloss_BF", "Airbag_02"),        # Seatbelt status, passenger
      ("ESP_Fahrer_bremst", "ESP_05"),           # Brake pedal pressed
      ("ESP_Bremsdruck", "ESP_05"),              # Brake pressure applied
      ("ESP_Status_Bremsdruck", "ESP_05"),       # Brakes applied
      ("ESP_BKV_Unterdruck", "ESP_05"),
      ("MO_Fahrpedalrohwert_01", "Motor_20"),    # Accelerator pedal value
      ("MO_rel_Saugrohrdruck", "Motor_20"),
      ("MO_rel_Saugrohrdruck_gem_err", "Motor_20"),
      ("EPS_Lenkmoment", "LH_EPS_03"),           # Absolute driver torque input
      ("EPS_VZ_Lenkmoment", "LH_EPS_03"),        # Driver torque input sign
      ("EPS_HCA_Status", "LH_EPS_03"),           # EPS HCA control status
      ("ESP_Tastung_passiv", "ESP_21"),          # Stability control disabled
      ("ESP_Haltebestaetigung", "ESP_21"),       # ESP hold confirmation
      ("KBI_MFA_v_Einheit_02", "Einheiten_01"),  # MPH vs KMH speed display
      ("KBI_Handbremse", "Kombi_01"),            # Manual handbrake applied
      ("TSK_Status", "TSK_06"),                  # ACC engagement status from drivetrain coordinator
      ("GRA_Hauptschalter", "GRA_ACC_01"),       # ACC button, on/off
      ("GRA_Abbrechen", "GRA_ACC_01"),           # ACC button, cancel
      ("GRA_Tip_Setzen", "GRA_ACC_01"),          # ACC button, set
      ("GRA_Tip_Hoch", "GRA_ACC_01"),            # ACC button, increase or accel
      ("GRA_Tip_Runter", "GRA_ACC_01"),          # ACC button, decrease or decel
      ("GRA_Tip_Wiederaufnahme", "GRA_ACC_01"),  # ACC button, resume
      ("GRA_Verstellung_Zeitluecke", "GRA_ACC_01"),  # ACC button, time gap adj
      ("GRA_Typ_Hauptschalter", "GRA_ACC_01"),   # ACC main button type
      ("GRA_Tip_Stufe_2", "GRA_ACC_01"),         # unknown related to stalk type
      ("GRA_ButtonTypeInfo", "GRA_ACC_01"),      # unknown related to stalk type
      ("COUNTER", "GRA_ACC_01"),                 # GRA_ACC_01 CAN message counter
    ]

    checks = [
      # sig_address, frequency
      ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
      ("LH_EPS_03", 100),   # From J500 Steering Assist with integrated sensors
      ("ESP_19", 100),      # From J104 ABS/ESP controller
      ("ESP_05", 50),       # From J104 ABS/ESP controller
      ("ESP_21", 50),       # From J104 ABS/ESP controller
      ("Motor_20", 50),     # From J623 Engine control module
      ("TSK_06", 50),       # From J623 Engine control module
      ("ESP_02", 50),       # From J104 ABS/ESP controller
      ("GRA_ACC_01", 33),   # From J533 CAN gateway (via LIN from steering wheel controls)
      ("Gateway_72", 10),   # From J533 CAN gateway (aggregated data)
      ("Airbag_02", 5),     # From J234 Airbag control module
      ("Kombi_01", 2),      # From J285 Instrument cluster
      ("Blinkmodi_02", 1),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("Einheiten_01", 1),  # From J??? not known if gateway, cluster, or BCM
    ]

    if CP.transmissionType == TransmissionType.automatic:
      signals.append(("GE_Fahrstufe", "Getriebe_11"))  # Auto trans gear selector position
      signals.append(("GE_Zielgang", "Getriebe_11"))
      checks.append(("Getriebe_11", 20))  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.direct:
      signals.append(("GearPosition", "EV_Gearshift"))  # EV gear selector position
      checks.append(("EV_Gearshift", 10))  # From J??? unknown EV control module
    elif CP.transmissionType == TransmissionType.manual:
      signals += [("MO_Kuppl_schalter", "Motor_14"),  # Clutch switch
                  ("BCM1_Rueckfahrlicht_Schalter", "Gateway_72")]  # Reverse light from BCM
      checks.append(("Motor_14", 10))  # From J623 Engine control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Radars are here on CANBUS.pt
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    #VAG
    #if CP.carFingerprint in (CAR.SKODA_KODIAQ_MK1):
    # ----- Motor_07 -----
    #if CP.vagCanModule.bus0Motor07:
    try:
      signals += MqbExtraSignals.motor_07_signals
      checks += MqbExtraSignals.motor_07_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_07 fail")

    # ----- VehicleSpeed -----
    #if CP.vagCanModule.bus0VehicleSpeed:
    try:
      signals += MqbExtraSignals.vehicle_speed_signals
      checks += MqbExtraSignals.vehicle_speed_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] VehicleSpeed fail")

    # ----- BCM_01 -----
    #if CP.vagCanModule.bus0Bcm01:
    try:
      signals += MqbExtraSignals.bcm_01_signals
      checks += MqbExtraSignals.bcm_01_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] BCM_01 fail")

    # ----- Kombi_02 -----
    #if CP.vagCanModule.bus0Kombi02:
    try:
      signals += MqbExtraSignals.kombi_02_signals
      checks += MqbExtraSignals.kombi_02_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Kombi_02 fail")

    # ----- Motor_18 -----
    #if CP.vagCanModule.bus0Motor18:
    try:
      signals += MqbExtraSignals.motor_18_signals
      checks += MqbExtraSignals.motor_18_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_18 fail")

    # ----- Charisma_01 -----
    #if CP.vagCanModule.bus0Charisma01:
    try:
      signals += MqbExtraSignals.charisma_01_signals
      checks += MqbExtraSignals.charisma_01_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Charisma_01 fail")

    # ---- Charisma_07 ----
    #if CP.vagCanModule.bus0Charisma07:
    try:
      signals += MqbExtraSignals.charisma_07_signals
      checks += MqbExtraSignals.charisma_07_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Charisma_07 fail")


    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        # sig_name, sig_address
        ("LDW_SW_Warnung_links", "LDW_02"),      # Blind spot in warning mode on left side due to lane departure
        ("LDW_SW_Warnung_rechts", "LDW_02"),     # Blind spot in warning mode on right side due to lane departure
        ("LDW_Seite_DLCTLC", "LDW_02"),          # Direction of most likely lane departure (left or right)
        ("LDW_DLC", "LDW_02"),                   # Lane departure, distance to line crossing
        ("LDW_TLC", "LDW_02"),                   # Lane departure, time to line crossing
      ]
      checks += [
        # sig_address, frequency
        ("LDW_02", 10)      # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.cam)

  @staticmethod
  def get_body_can_parser(CP):
    signals = []
    checks = []

    #VAG
    #if CP.carFingerprint in (CAR.SKODA_KODIAQ_MK1):
    # ----- Getriebe_14 -----
    #if CP.vagCanModule.bus1Getriebe14:
    try:
      signals += MqbExtraSignals.getriebe_14_signals
      checks += MqbExtraSignals.getriebe_14_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Getriebe_14 fail")

    # ----- Moto_12 -----
    #if CP.vagCanModule.bus1Motor12:
    try:
      signals += MqbExtraSignals.motor_12_signals
      checks += MqbExtraSignals.motor_12_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Moto_12 fail")

    # ----- Motor_09 -----
    #if CP.vagCanModule.bus1Motor09:
    try:
      signals += MqbExtraSignals.motor_09_signals
      checks += MqbExtraSignals.motor_09_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_09 fail")

    # ----- OBD_01 -----
    #if CP.vagCanModule.bus1Obd01:
    try:
      signals += MqbExtraSignals.obd_01_signals
      checks += MqbExtraSignals.obd_01_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] OBD_01 fail")

    # ----- Motor_04 -----
    #if CP.vagCanModule.bus1Motor04:
    try:
      signals += MqbExtraSignals.motor_04_signals
      checks += MqbExtraSignals.motor_04_checks
    except:
      print("[BOP][carstate.py][get_can_parser()] Motor_04 fail")

    return CANParser(DBC_FILES.mqb, signals, checks, CANBUS.body)

class MqbExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACC_Wunschgeschw", "ACC_02"),              # ACC set speed
    ("ACC_Status_Prim_Anz", "ACC_02"),
    ("ACC_Abstandsindex", "ACC_02"),
    ("ACC_Akustik", "ACC_02"),
    ("ACC_Gesetzte_Zeitluecke", "ACC_02"),
    ("ACC_Optischer_Fahrerhinweis", "ACC_02"),
    ("ACC_Typ_Tachokranz", "ACC_02"),
    ("ACC_Anzeige_Zeitluecke", "ACC_02"),
    ("ACC_Tachokranz", "ACC_02"),
    ("ACC_Display_Prio", "ACC_02"),
    ("ACC_Relevantes_Objekt", "ACC_02"),
    ("ACC_Texte_Primaeranz", "ACC_02"),
    ("ACC_Wunschgeschw_erreicht", "ACC_02"),
    ("ACC_Status_Anzeige", "ACC_02"),
    ("ACC_Texte_Zusatzanz", "ACC_04"),
    ("ACC_Status_Zusatzanz", "ACC_04"),
    ("ACC_Texte", "ACC_04"),
    ("ACC_Texte_braking_guard", "ACC_04"),
    ("ACC_Warnhinweis", "ACC_04"),
    ("ACC_Geschw_Zielfahrzeug", "ACC_04"),
    ("ACC_Charisma_FahrPr", "ACC_04"),
    ("ACC_Charisma_Status", "ACC_04"),
    ("ACC_Charisma_Umschaltung", "ACC_04"),
    ("ACC_limitierte_Anfahrdyn", "ACC_06"),
    ("ACC_zul_Regelabw_unten", "ACC_06"),
    ("ACC_StartStopp_Info", "ACC_06"),
    ("ACC_Sollbeschleunigung_02", "ACC_06"),
    ("ACC_zul_Regelabw_oben", "ACC_06"),
    ("ACC_neg_Sollbeschl_Grad_02", "ACC_06"),
    ("ACC_pos_Sollbeschl_Grad_02", "ACC_06"),
    ("ACC_Anfahren", "ACC_06"),
    ("ACC_Anhalten", "ACC_06"),
    ("ACC_Typ", "ACC_06"),
    ("ACC_Status_ACC", "ACC_06"),
    ("ACC_Minimale_Bremsung", "ACC_06"),
    ("ACC_Distance_to_Stop", "ACC_07"),
    ("ACC_Hold_Request", "ACC_07"),
    ("ACC_Boost_Request", "ACC_07"),
    ("ACC_Freewheel_Request", "ACC_07"),
    ("ACC_Freewheel_Type", "ACC_07"),
    ("ACC_Hold_Type", "ACC_07"),
    ("ACC_Hold_Release", "ACC_07"),
    ("ACC_Accel_Secondary", "ACC_07"),
    ("ACC_Accel_TSK", "ACC_07"),
    ("AWV1_Anf_Prefill", "ACC_10"),
    ("ANB_CM_Info", "ACC_10"),
    ("AWV2_Freigabe", "ACC_10"),                 # FCW brake jerk release
    ("AWV1_HBA_Param", "ACC_10"),
    ("AWV2_Ruckprofil", "ACC_10"),
    ("AWV2_Priowarnung", "ACC_10"),
    ("ANB_CM_Anforderung", "ACC_10"),
    ("ANB_Info_Teilbremsung", "ACC_10"),
    ("ANB_Notfallblinken", "ACC_10"),
    ("ANB_Teilbremsung_Freigabe", "ACC_10"),     # AEB partial braking release
    ("ANB_Zielbrems_Teilbrems_Verz_Anf", "ACC_10"),
    ("ANB_Zielbremsung_Freigabe", "ACC_10"),     # AEB target braking release
    ("AWV_Vorstufe", "ACC_10"),
    ("AWV_Halten", "ACC_10"),
  ]
  fwd_radar_checks = [
    ("ACC_10", 50),                                 # From J428 ACC radar control module
    ("ACC_02", 17),
    ("ACC_04", 17),
    ("ACC_06", 50),
    ("ACC_07", 50),

    #("VZE_01", 10)
  ]
  bsm_radar_signals = [
    ("SWA_Infostufe_SWA_li", "SWA_01"),          # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_01"),            # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_01"),          # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_01"),            # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_01", 20),                                 # From J1086 Lane Change Assist
  ]

  # ----- Motor_07 -----
  motor_07_signals = [
    ("MO_QBit_Ansaugluft_Temp", "Motor_07"),
    ("MO_QBit_Oel_Temp", "Motor_07"),
    ("MO_QBit_Kuehlmittel_Temp", "Motor_07"),
    ("MO_Stellgliedtest_Soundaktuator", "Motor_07"),
    ("MO_HYB_Fehler_HV_Netz", "Motor_07"),
    ("MO_aktives_Getriebeheizen", "Motor_07"),
    ("MO_Absperrventil_oeffnen", "Motor_07"),
    ("MO_Ansaugluft_Temp", "Motor_07"),
    ("MO_Oel_Temp", "Motor_07"),
    ("MO_Kuehlmittel_Temp", "Motor_07"),
    ("MO_Hoeheninfo", "Motor_07"),
    ("MO_Kennfeldk", "Motor_07"),
    ("MO_Versionsinfo", "Motor_07"),
    ("MO_Getriebe_kuehlen", "Motor_07"),
    ("MO_Mom_Traegheit_02", "Motor_07"),
    ("MO_Heizungspumpenansteuerung", "Motor_07"),
    ("MO_SpannungsAnf", "Motor_07"),
    ("MO_Nachlaufzeit_Heizungspumpe", "Motor_07"),
  ]
  motor_07_checks = [
    ("Motor_07", 1),
  ]

  # ----- VehicleSpeed -----
  vehicle_speed_signals = [
    ("Speed", "VehicleSpeed"),
  ]
  vehicle_speed_checks = [
    ("VehicleSpeed", 50),
  ]

  # ----- BCM_01 -----
  bcm_01_signals = [
    ("BCM_Bremsbelag_Sensor", "BCM_01"),
    ("BCM_Bremsfluessigkeit_Sensor", "BCM_01"),
    ("BCM1_Licht_Warn", "BCM_01"),
    ("BCM_Waschwasser_Sensor", "BCM_01"),
    ("BCM_Kuehlmittel_Sensor", "BCM_01"),
    ("BCM1_Kl_15_HW_erkannt", "BCM_01"),
    ("BCM_Eis_Offroad_Taste", "BCM_01"),
    ("ZZH_Endlage_oben", "BCM_01"),
    ("ZZH_Endlage_unten", "BCM_01"),
    ("ZZH_Endlage_unplausibel", "BCM_01"),
    ("BCM2_EZS_gedrueckt", "BCM_01"),
    ("BCM2_SST_gedrueckt", "BCM_01"),
    ("BCM_Hybrid_StartStopp_Taste", "BCM_01"),
    ("BCM1_Warnblink_Taster", "BCM_01"),
    ("BCM1_Valet_Parking_Taster", "BCM_01"),
    ("BCM_Remotestart_Betrieb", "BCM_01"),
    ("BCM1_HSK_Taster", "BCM_01"),
    ("BCM1_Heckrollo_Taster", "BCM_01"),
    ("BCM1_Rueckfahrlicht_Schalter", "BCM_01"),
    ("BCM1_MH_Schalter", "BCM_01"),
    ("BCM1_MH_WIV_Schalter", "BCM_01"),
    ("BCM_Eco_Charisma_Taste", "BCM_01"),
    ("BCM_Thermomanagement", "BCM_01"),
    ("BCM_Thermomanagement_Fehler", "BCM_01"),
    ("BCM_Thermomanagement_gueltig", "BCM_01"),
    ("BCM1_Lichtwarn_Texte", "BCM_01"),
  ]
  bcm_01_checks = [
    ("BCM_01", 1),
  ]

  # ----- Kombi_02 -----
  kombi_02_signals = [
    ("KBI_Kilometerstand", "Kombi_02"),
    ("KBI_Standzeit_02", "Kombi_02"),
    ("KBI_Inhalt_Tank", "Kombi_02"),
    ("KBI_FStatus_Tank", "Kombi_02"),
    ("KBI_QBit_Aussen_Temp_gef", "Kombi_02"),
    ("KBI_Aussen_Temp_gef", "Kombi_02"),
  ]
  kombi_02_checks = [
    ("Kombi_02", 1),
  ]

  # ----- Motor_18 -----
  motor_18_signals = [
    ("MO_max_Ladedruck", "Motor_18"),
    ("MO_Hybrid_StartStopp_LED", "Motor_18"),
    ("MO_Eis_Offroad_LED", "Motor_18"),
    ("MO_Anzahl_Abgesch_Zyl", "Motor_18"),
    ("MO_Zylabsch_Texte", "Motor_18"),
    ("MO_E85_BS_Texte", "Motor_18"),
    ("MO_Drehzahl_Warnung", "Motor_18"),
    ("MO_obere_Drehzahlgrenze", "Motor_18"),
  ]
  motor_18_checks = [
    ("Motor_18", 1),
  ]

  # ----- Charisma_01 -----
  charisma_01_signals = [
    ("CHA_Ziel_FahrPr_ALR", "Charisma_01"),
    ("CHA_Ziel_FahrPr_ESP", "Charisma_01"),
    ("CHA_Ziel_FahrPr_FL", "Charisma_01"),
    ("CHA_Fahrer_Umschaltung", "Charisma_01"),
    ("CHA_Ziel_FahrPr_MO", "Charisma_01"),
    ("CHA_Ziel_FahrPr_GE", "Charisma_01"),
    ("CHA_Ziel_FahrPr_ST", "Charisma_01"),
    ("CHA_Ziel_FahrPr_SCU", "Charisma_01"),
    ("CHA_Ziel_FahrPr_DR", "Charisma_01"),
    ("CHA_Ziel_FahrPr_QS", "Charisma_01"),
    ("CHA_Ziel_FahrPr_AFS", "Charisma_01"),
    ("CHA_Ziel_FahrPr_RGS", "Charisma_01"),
    ("CHA_Ziel_FahrPr_EPS", "Charisma_01"),
    ("CHA_Ziel_FahrPr_ACC", "Charisma_01"),
    ("CHA_Ziel_FahrPr_SAK", "Charisma_01"),
    ("CHA_Ziel_FahrPr_MStSt", "Charisma_01"),
  ]
  charisma_01_checks = [
    ("Charisma_01", 1),
  ]

  # ----- Charisma_07 -----
  charisma_07_signals = [
    ("CHA_Ziel_FahrPr_EBKV", "Charisma_07"),
    ("CHA_Ziel_FahrPr_IL", "Charisma_07"),
    ("CHA_Current_Mode", "Charisma_07"),
    ("CHA_Fahrer_Umschaltung", "Charisma_07"),
    ("CHA_Ziel_FahrPr_QS", "Charisma_07"),
    ("CHA_Ziel_FahrPr_SCU", "Charisma_07"),
    ("CHA_Ziel_FahrPr_HAL", "Charisma_07"),
    ("CHA_Ziel_FahrPr_NR", "Charisma_07"),
    ("CHA_Ziel_FahrPr_AV", "Charisma_07"),
    ("CHA_Ziel_FahrPr_MXB", "Charisma_07"),
    ("CHA_Ziel_FahrPr_AFR", "Charisma_07"),
    ("CHA_Ziel_FahrPr_HDC", "Charisma_07"),
    ("CHA_Ziel_FahrPr_EAR", "Charisma_07"),
    ("CHA_Ziel_FahrPr_AEB", "Charisma_07"),
    ("CHA_Ziel_FahrPr_FCWO", "Charisma_07"),
    ("CHA_Ziel_FahrPr_FCWP", "Charisma_07"),
  ]
  charisma_07_checks = [
    ("Charisma_07", 1),
  ]

  # ----- Getriebe_14 -----
  getriebe_14_signals = [
    ("GE_OBD_AbsperrVent", "Getriebe_14"),
    ("GE_amax_moeglich", "Getriebe_14"),
    ("GE_Charisma_FahrPr", "Getriebe_14"),
    ("GE_Charisma_Status", "Getriebe_14"),
    ("GE_Verlustmoment", "Getriebe_14"),
    ("GE_Freigabe_Verfallsinfo_WFS", "Getriebe_14"),
    ("GE_Codierung_MSG", "Getriebe_14"),
    ("GE_LaunchControl", "Getriebe_14"),
    ("GE_Heizwunsch", "Getriebe_14"),
    ("GE_OBD_Status", "Getriebe_14"),
    ("GE_LFR_Adaption", "Getriebe_14"),
    ("GE_Sumpftemperatur", "Getriebe_14"),
  ]
  getriebe_14_checks = [
    ("Getriebe_14", 10),
  ]

  # ----- Motor_12 -----
  motor_12_signals = [
    ("MO_Mom_neg_verfuegbar", "Motor_12"),
    ("MO_Mom_Begr_stat", "Motor_12"),
    ("MO_Mom_Begr_dyn", "Motor_12"),
    ("MO_Momentenintegral_02", "Motor_12"),
    ("MO_QBit_Drehzahl_01", "Motor_12"),
    ("MO_Drehzahl_01", "Motor_12"),
  ]
  motor_12_checks = [
    ("Motor_12", 100),
  ]

  # ----- Motor_09 -----
  motor_09_signals = [
    ("MO_ITM_Kuehlmittel_Temp", "Motor_09"),
    ("MO_E85_Sensor", "Motor_09"),
    ("SCR_Anz_Motorstarts", "Motor_09"),
    ("SCR_Reichweite", "Motor_09"),
    ("SCR_Warnstufe_1", "Motor_09"),
    ("SCR_Warnstufe_2", "Motor_09"),
    ("SCR_Text", "Motor_09"),
    ("SCR_Akustik", "Motor_09"),
    ("MO_Kraftstofffilter_Wasser", "Motor_09"),
    ("SCR_Systemfehler", "Motor_09"),
    ("SCR_Inducement_Strategie", "Motor_09"),
    ("MO_CO2_Faktor", "Motor_09"),
  ]
  motor_09_checks = [
    ("Motor_09", 1),
  ]

  # ----- OBD_01 -----
  obd_01_signals = [
    ("OBD_Calc_Load_Val", "OBD_01"),
    ("OBD_Eng_Cool_Temp", "OBD_01"),
    ("OBD_Abs_Throttle_Pos", "OBD_01"),
    ("OBD_Abs_Load_Val", "OBD_01"),
    ("OBD_Abs_Pedal_Pos", "OBD_01"),
    ("OBD_Kaltstart_Denominator", "OBD_01"),
    ("OBD_Minimum_Trip", "OBD_01"),
    ("OBD_Driving_Cycle", "OBD_01"),
    ("OBD_Warm_Up_Cycle", "OBD_01"),
    ("OBD_Normed_Trip", "OBD_01"),
  ]
  obd_01_checks = [
    ("OBD_01", 1),
  ]

  # ----- Motor_04 -----
  motor_04_signals = [
    ("MO_Istgang", "Motor_04"),
    ("MO_Sollgang", "Motor_04"),
    ("MO_Oeldruck", "Motor_04"),
    ("MO_Anzeigedrehz", "Motor_04"),
    ("MO_Schaltempf_verfbar", "Motor_04"),
    ("MO_Ladedruck", "Motor_04"),
    ("MO_KVS", "Motor_04"),
    ("MO_KVS_Ueberlauf", "Motor_04"),
  ]
  motor_04_checks = [
    ("Motor_04", 1),
  ]