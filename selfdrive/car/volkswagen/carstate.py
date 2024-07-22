#
# Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

import numpy as np
import cereal.messaging as messaging

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.volkswagen.values import DBC, CANBUS, NetworkLocation, TransmissionType, GearShifter, \
                                            CarControllerParams, VolkswagenFlags

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.frame = 0
    self.eps_init_complete = False
    self.CCP = CarControllerParams(CP)
    self.button_states = {button.event_type: False for button in self.CCP.BUTTONS}
    self.esp_hold_confirmation = False
    self.upscale_lead_car_signal = False
    self.eps_stock_values = False

    self.sm = messaging.SubMaster(['vagParam'])

    #print("[BOP][carstate.py][__init__] CP.carName=", CP.carName)
    #print("[BOP][carstate.py][__init__] CP.carFingerprint=", CP.carFingerprint)
    #print("[BOP][carstate.py][__init__] CP.fuzzyFingerprint=", CP.fuzzyFingerprint)
    #print("[BOP][carstate.py][__init__] CP.notCar=", CP.notCar)
    #print("[BOP][carstate.py][__init__] CP.enableGasInterceptor=", CP.enableGasInterceptor)
    #print("[BOP][carstate.py][__init__] CP.pcmCruise=", CP.pcmCruise)
    #print("[BOP][carstate.py][__init__] CP.enableDsu=", CP.enableDsu)
    #print("[BOP][carstate.py][__init__] CP.enableBsm=", CP.enableBsm)
    #print("[BOP][carstate.py][__init__] CP.flags=", CP.flags)
    #print("[BOP][carstate.py][__init__] CP.experimentalLongitudinalAvailable=", CP.experimentalLongitudinalAvailable)
    #print("[BOP][carstate.py][__init__] CP.minEnableSpeed=", CP.minEnableSpeed)
    #print("[BOP][carstate.py][__init__] CP.minSteerSpeed=", CP.minSteerSpeed)
    #print("[BOP][carstate.py][__init__] CP.safetyConfigs=", CP.safetyConfigs)
    #print("[BOP][carstate.py][__init__] CP.alternativeExperience=", CP.alternativeExperience)
    #print("[BOP][carstate.py][__init__] CP.maxLateralAccel=", CP.maxLateralAccel)
    #print("[BOP][carstate.py][__init__] CP.autoResumeSng=", CP.autoResumeSng)
    #print("[BOP][carstate.py][__init__] CP.mass=", CP.mass)
    #print("[BOP][carstate.py][__init__] CP.wheelbase=", CP.wheelbase)
    #print("[BOP][carstate.py][__init__] CP.centerToFront=", CP.centerToFront)
    #print("[BOP][carstate.py][__init__] CP.steerRatio=", CP.steerRatio)
    #print("[BOP][carstate.py][__init__] CP.longitudinalTuning=", CP.longitudinalTuning)
    #print("[BOP][carstate.py][__init__] CP.lateralParams=", CP.lateralParams)
    #print("[BOP][carstate.py][__init__] CP.lateralTuning=", CP.lateralTuning)
    #print("[BOP][carstate.py][__init__] CP.steerLimitAlert=", CP.steerLimitAlert)
    #print("[BOP][carstate.py][__init__] CP.steerLimitTimer=", CP.steerLimitTimer)
    #print("[BOP][carstate.py][__init__] CP.vEgoStopping=", CP.vEgoStopping)
    #print("[BOP][carstate.py][__init__] CP.vEgoStarting=", CP.vEgoStarting)
    #print("[BOP][carstate.py][__init__] CP.stoppingControl=", CP.stoppingControl)
    #print("[BOP][carstate.py][__init__] CP.steerControlType=", CP.steerControlType)
    #print("[BOP][carstate.py][__init__] CP.radarUnavailable=", CP.radarUnavailable)
    #print("[BOP][carstate.py][__init__] CP.stopAccel=", CP.stopAccel)
    #print("[BOP][carstate.py][__init__] CP.stoppingDecelRate=", CP.stoppingDecelRate)
    #print("[BOP][carstate.py][__init__] CP.startAccel=", CP.startAccel)
    #print("[BOP][carstate.py][__init__] CP.startingState=", CP.startingState)
    #print("[BOP][carstate.py][__init__] CP.steerActuatorDelay=", CP.steerActuatorDelay)
    #print("[BOP][carstate.py][__init__] CP.longitudinalActuatorDelayLowerBound=", CP.longitudinalActuatorDelayLowerBound)
    #print("[BOP][carstate.py][__init__] CP.longitudinalActuatorDelayUpperBound=", CP.longitudinalActuatorDelayUpperBound)
    #print("[BOP][carstate.py][__init__] CP.openpilotLongitudinalControl=", CP.openpilotLongitudinalControl)
    #print("[BOP][carstate.py][__init__] CP.carVin=", CP.carVin)
    #print("[BOP][carstate.py][__init__] CP.dashcamOnly=", CP.dashcamOnly)
    #print("[BOP][carstate.py][__init__] CP.transmissionType=", CP.transmissionType)
    #print("[BOP][carstate.py][__init__] CP.carFw=", CP.carFw)
    #print("[BOP][carstate.py][__init__] CP.radarTimeStep=", CP.radarTimeStep)
    #print("[BOP][carstate.py][__init__] CP.fingerprintSource=", CP.fingerprintSource)
    #print("[BOP][carstate.py][__init__] CP.networkLocation=", CP.networkLocation)
    #print("[BOP][carstate.py][__init__] CP.wheelSpeedFactor=", CP.wheelSpeedFactor)

  def create_button_events(self, pt_cp, buttons):
    button_events = []

    for button in buttons:
      state = pt_cp.vl[button.can_addr][button.can_msg] in button.values
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state

    return button_events

  def update(self, pt_cp, cam_cp, ext_cp, body_cp, vag_info_cp, vag_gb_cp, vag_pt_cp, trans_type):
    if self.CP.flags & VolkswagenFlags.PQ:
      return self.update_pq(pt_cp, cam_cp, ext_cp, trans_type)

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
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    ret.steeringAngleDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradwinkel"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradwinkel"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["ESP_02"]["ESP_Gierrate"] * (1, -1)[int(pt_cp.vl["ESP_02"]["ESP_VZ_Gierrate"])] * CV.DEG_TO_RAD
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    # VW Emergency Assist status tracking and mitigation
    self.eps_stock_values = pt_cp.vl["LH_EPS_03"]
    if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      ret.carFaultedNonCritical = bool(cam_cp.vl["HCA_01"]["EA_Ruckfreigabe"]) or cam_cp.vl["HCA_01"]["EA_ACC_Sollstatus"] > 0

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_20"]["MO_Fahrpedalrohwert_01"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    brake_pedal_pressed = bool(pt_cp.vl["Motor_14"]["MO_Fahrer_bremst"])
    brake_pressure_detected = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])
    ret.brakePressed = brake_pedal_pressed or brake_pressure_detected
    ret.parkingBrake = bool(pt_cp.vl["Kombi_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
    elif trans_type == TransmissionType.direct:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["EV_Gearshift"]["GearPosition"], None))
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

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"])
      ret.vagCarState.leftBlindspotWarning = bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
      ret.vagCarState.rightBlindspotWarning = bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    ret.vagCarState.brakeLights = bool(pt_cp.vl["ESP_05"]["ESP_Status_Bremsdruck"])

    #VAG
    # ----- Motor_04 -----
    if self.CP.vagCarParams.vagCanModule.bus1.motor04:
      ret.vagCarState.vagUiField.moIstgang                  = body_cp.vl["Motor_04"]["MO_Istgang"]
      ret.vagCarState.vagUiField.moLadedruck                = body_cp.vl["Motor_04"]["MO_Ladedruck"] #0~5.10 Bar
      ret.vagCarState.vagUiField.moOeldruck                 = body_cp.vl["Motor_04"]["MO_Oeldruck"] #0~10.00 Bar
    # ----- Motor_07 -----
    if self.CP.vagCarParams.vagCanModule.bus0.motor07:
      ret.vagCarState.vagUiField.moAnsaugluftTemp           = pt_cp.vl["Motor_07"]["MO_Ansaugluft_Temp"] #-48~141.75 DegreCelsi
      ret.vagCarState.vagUiField.moKuehlmittelTemp          = pt_cp.vl["Motor_07"]["MO_Kuehlmittel_Temp"] #-48~141.75 DegreCelsi
      ret.vagCarState.vagUiField.moOelTemp                  = pt_cp.vl["Motor_07"]["MO_Oel_Temp"] #-60~192 DegreCelsi
    # ----- Motor_09 -----
    if self.CP.vagCarParams.vagCanModule.bus1.motor09:
      ret.vagCarState.vagUiField.moItmKuehlmittelTemp       = body_cp.vl["Motor_09"]["MO_ITM_Kuehlmittel_Temp"] #-45.75~143.25 DegreCelsi
    # ----- Motor_18 -----
    if self.CP.vagCarParams.vagCanModule.bus0.motor18:
      ret.vagCarState.vagUiField.moMaxLadedruck             = pt_cp.vl["Motor_18"]["MO_max_Ladedruck"] #0~6.3 Bar
    # ----- Motor_20 -----
    ret.vagCarState.vagUiField.moRelSaugrohrdruck           = pt_cp.vl["Motor_20"]["MO_rel_Saugrohrdruck"] #0~1.116 Bar
    ret.vagCarState.vagUiField.moRelSaugrohrdruckGemErr     = pt_cp.vl["Motor_20"]["MO_rel_Saugrohrdruck_gem_err"]
    # ----- Getriebe_11 -----
    #Pon: For panda jungle develop
    if not self.sm['vagParam'].vagParamGeneral.isVagPandaJungleEnabled:
      try:
        ret.vagCarState.vagUiField.geZielgang                 = pt_cp.vl["Getriebe_11"]["GE_Zielgang"]
      except Exception:
        print("[BOP][carstate.py] exception to set vagCarState.vagUiField.geZielgang")
    # ----- Getriebe_14 -----
    if self.CP.vagCarParams.vagCanModule.bus1.getriebe14:
      ret.vagCarState.vagUiField.geSumpftemperatur          = body_cp.vl["Getriebe_14"]["GE_Sumpftemperatur"] #-58~196 DegreCelsi
    # ----- ESP_05 -----
    ret.vagCarState.vagUiField.espBremsdruck                = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] #-30~276.6 Bar
    ret.vagCarState.vagUiField.espBvkUnterdruck             = pt_cp.vl["ESP_05"]["ESP_BKV_Unterdruck"] #0~1.012 Bar
    # ----- Gateway_72 -----
    ret.vagCarState.vagUiField.bcm1AussenTempUngef          = pt_cp.vl["Gateway_72"]["BCM1_Aussen_Temp_ungef"] #-50~76.0 DegreCelsi
    # ----- OBD_01 -----
    if self.CP.vagCarParams.vagCanModule.bus1.obd01:
      ret.vagCarState.vagUiField.obdEngCoolTemp             = body_cp.vl["OBD_01"]["OBD_Eng_Cool_Temp"] #-40~215 DegreCelsi
    # ----- Kombi_02 -----
    if self.CP.vagCarParams.vagCanModule.bus0.kombi02:
      ret.vagCarState.vagUiField.kbiAussenTempGef           = pt_cp.vl["Kombi_02"]["KBI_Aussen_Temp_gef"] #-50~75.0 DegreCelsi
    # ----- ACC_02 -----
    ret.vagCarState.vagUiField.accAbstandsindex             = ext_cp.vl["ACC_02"]["ACC_Abstandsindex"]
    # ----- VehicleSpeed -----
    #Pon: For panda jungle develop
    if not self.sm['vagParam'].vagParamGeneral.isVagPandaJungleEnabled:
      if self.CP.vagCarParams.vagCanModule.bus1.motor04:
        try:
          ret.vagCarState.vagUiField.speed                    = pt_cp.vl["VehicleSpeed"]["Speed"] #Km/H
        except Exception:
          print("[BOP][carstate.py] exception to set vagCarState.vagUiField.speed")

    # ----- Motor_12 -----
    if self.CP.vagCarParams.vagCanModule.bus1.motor12:
      ret.engineRpm                             = body_cp.vl["Motor_12"]["MO_Drehzahl_01"]

    ##### VAG Force disable startstop #####
    if self.CP.vagCarParams.vagCanModule.bus0.bcm01:
      self.bcm_01 = pt_cp.vl["BCM_01"]
    if self.CP.vagCarParams.vagCanModule.bus0.motor18:
      self.motor_18 = pt_cp.vl["Motor_18"]

    ##### VAG Driving mode #####
    if self.CP.vagCarParams.vagCanModule.bus0.charisma01:
      self.charisma_01 = pt_cp.vl["Charisma_01"]
    if self.CP.vagCarParams.vagCanModule.bus0.charisma07:
      self.charisma_07 = pt_cp.vl["Charisma_07"]

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
    self.acc_type = ext_cp.vl["ACC_06"]["ACC_Typ"]

    # ACC okay but disabled (1), ACC ready (2), a radar visibility or other fault/disruption (6 or 7)
    # currently regulating speed (3), driver accel override (4), brake only (5)
    ret.cruiseState.available = pt_cp.vl["TSK_06"]["TSK_Status"] in (2, 3, 4, 5)
    ret.cruiseState.enabled = pt_cp.vl["TSK_06"]["TSK_Status"] in (3, 4, 5)

    if self.CP.pcmCruise:
      # Cruise Control mode; check for distance UI setting from the radar.
      # ECM does not manage this, so do not need to check for openpilot longitudinal
      ret.cruiseState.nonAdaptive = ext_cp.vl["ACC_02"]["ACC_Gesetzte_Zeitluecke"] == 0
    else:
      # Speed limiter mode; ECM faults if we command ACC while not pcmCruise
      ret.cruiseState.nonAdaptive = bool(pt_cp.vl["TSK_06"]["TSK_Limiter_ausgewaehlt"])

    ret.accFaulted = pt_cp.vl["TSK_06"]["TSK_Status"] in (6, 7)

    self.esp_hold_confirmation = bool(pt_cp.vl["ESP_21"]["ESP_Haltebestaetigung"])
    ret.cruiseState.standstill = self.CP.pcmCruise and self.esp_hold_confirmation

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    if self.CP.pcmCruise:
      ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_02"] * CV.KPH_TO_MS
      if ret.cruiseState.speed > 90:
        ret.cruiseState.speed = 0

    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Left"])
    ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_02"]["Comfort_Signal_Right"])
    ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    self.gra_stock_values = pt_cp.vl["GRA_ACC_01"]

    # Additional safety checks performed in CarInterface.
    ret.espDisabled = pt_cp.vl["ESP_21"]["ESP_Tastung_passiv"] != 0

    # Digital instrument clusters expect the ACC HUD lead car distance to be scaled differently
    self.upscale_lead_car_signal = bool(pt_cp.vl["Kombi_03"]["KBI_Variante"])

    self.frame += 1
    return ret

  def update_pq(self, pt_cp, cam_cp, ext_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["Bremse_3"]["Radgeschw__VL_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__VR_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__HL_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__HR_4_1"],
    )

    # vEgo obtained from Bremse_1 vehicle speed rather than Bremse_3 wheel speeds because Bremse_3 isn't present on NSF
    ret.vEgoRaw = pt_cp.vl["Bremse_1"]["Geschwindigkeit_neu__Bremse_1_"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw == 0

    # Update EPS position and state info. For signed values, VW sends the sign in a separate signal.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit_S"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > self.CCP.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]["Giergeschwindigkeit"] * (1, -1)[int(pt_cp.vl["Bremse_5"]["Vorzeichen_der_Giergeschwindigk"])] * CV.DEG_TO_RAD
    hca_status = self.CCP.hca_status_values.get(pt_cp.vl["Lenkhilfe_2"]["LH2_Sta_HCA"])
    ret.steerFaultTemporary, ret.steerFaultPermanent = self.update_hca_state(hca_status)

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_3"]["Fahrpedal_Rohsignal"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["Bremse_5"]["Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["Bremslichtschalter"])
    ret.parkingBrake = bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])

    # Update gear and/or clutch position data.
    if trans_type == TransmissionType.automatic:
      ret.gearShifter = self.parse_gear_shifter(self.CCP.shifter_values.get(pt_cp.vl["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"], None))
    elif trans_type == TransmissionType.manual:
      ret.clutchPressed = not pt_cp.vl["Motor_1"]["Kupplungsschalter"]
      reverse_light = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Rueckfahr"])
      if reverse_light:
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # Update door and trunk/hatch lid open status.
    ret.doorOpen = any([pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_BT_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HL_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HR_geoeffnet"],
                        pt_cp.vl["Gate_Komf_1"]["BSK_HD_Hauptraste"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
    if self.CP.enableBsm:
      ret.leftBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_li"])
      ret.rightBlindspot = bool(ext_cp.vl["SWA_1"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_1"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
    self.ldw_stock_values = cam_cp.vl["LDW_Status"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapters on Front Assist with Braking and City Emergency
    # Braking for the 2016 Passat NMS
    # TODO: deferred until we can collect data on pre-MY2016 behavior, AWV message may be shorter with fewer signals
    ret.stockFcw = False
    ret.stockAeb = False

    # Update ACC radar status.
    self.acc_type = ext_cp.vl["ACC_System"]["ACS_Typ_ACC"]
    ret.cruiseState.available = bool(pt_cp.vl["Motor_5"]["GRA_Hauptschalter"])
    ret.cruiseState.enabled = pt_cp.vl["Motor_2"]["GRA_Status"] in (1, 2)
    if self.CP.pcmCruise:
      ret.accFaulted = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_StaACC"] in (6, 7)
    else:
      ret.accFaulted = pt_cp.vl["Motor_2"]["GRA_Status"] == 3

    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    # yet established an ACC setpoint, so treat it as zero.
    ret.cruiseState.speed = ext_cp.vl["ACC_GRA_Anzeige"]["ACA_V_Wunsch"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 70:  # 255 kph in m/s == no current setpoint
      ret.cruiseState.speed = 0

    # Update button states for turn signals and ACC controls, capture all ACC button state/config for passthrough
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(300, pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"],
                                                                            pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])
    ret.buttonEvents = self.create_button_events(pt_cp, self.CCP.BUTTONS)
    self.gra_stock_values = pt_cp.vl["GRA_Neu"]

    # Additional safety checks performed in CarInterface.
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["ESP_Passiv_getastet"])

    self.frame += 1
    return ret

  def update_hca_state(self, hca_status):
    # Treat INITIALIZING and FAULT as temporary for worst likely EPS recovery time, for cars without factory Lane Assist
    # DISABLED means the EPS hasn't been configured to support Lane Assist
    self.eps_init_complete = self.eps_init_complete or (hca_status in ("DISABLED", "READY", "ACTIVE") or self.frame > 600)
    perm_fault = hca_status == "DISABLED" or (self.eps_init_complete and hca_status in ("INITIALIZING", "FAULT"))
    temp_fault = hca_status in ("REJECTED", "PREEMPTED") or not self.eps_init_complete
    return temp_fault, perm_fault

  @staticmethod
  def get_can_parser(CP): #can bus 0
    if CP.flags & VolkswagenFlags.PQ:
      return CarState.get_can_parser_pq(CP)

    messages = [
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
      ("Motor_14", 10),     # From J623 Engine control module
      ("Airbag_02", 5),     # From J234 Airbag control module
      ("Kombi_01", 2),      # From J285 Instrument cluster
      ("Blinkmodi_02", 1),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
      ("Kombi_03", 0),      # From J285 instrument cluster (not present on older cars, 1Hz when present)
    ]

    if CP.transmissionType == TransmissionType.automatic:
      messages.append(("Getriebe_11", 20))  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.direct:
      messages.append(("EV_Gearshift", 10))  # From J??? unknown EV control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Radars are here on CANBUS.pt
      messages += MqbExtraSignals.fwd_radar_messages
      if CP.enableBsm:
        messages += MqbExtraSignals.bsm_radar_messages

    #VAG
    #if CP.carFingerprint in (CAR.SKODA_KODIAQ_MK1):
    if CP.vagCarParams.vagCanModule.bus0.motor07:
      messages += MqbExtraSignals.motor_07_message
    if CP.vagCarParams.vagCanModule.bus0.vehicleSpeed:
      messages += MqbExtraSignals.vehicle_speed_message
    if CP.vagCarParams.vagCanModule.bus0.bcm01:
      messages += MqbExtraSignals.bcm_01_message
    if CP.vagCarParams.vagCanModule.bus0.kombi02:
      messages += MqbExtraSignals.kombi_02_message
    if CP.vagCarParams.vagCanModule.bus0.motor18:
      messages += MqbExtraSignals.motor_18_message
    if CP.vagCarParams.vagCanModule.bus0.charisma01:
      messages += MqbExtraSignals.charisma_01_message
    if CP.vagCarParams.vagCanModule.bus0.charisma07:
      messages += MqbExtraSignals.charisma_07_message
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser(CP): #can bus 2
    if CP.flags & VolkswagenFlags.PQ:
      return CarState.get_cam_can_parser_pq(CP)

    messages = []

    if CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
      messages += [
        ("HCA_01", 1),  # From R242 Driver assistance camera, 50Hz if steering/1Hz if not
      ]

    if CP.networkLocation == NetworkLocation.fwdCamera:
      messages += [
        # sig_address, frequency
        ("LDW_02", 10),     # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      messages += MqbExtraSignals.fwd_radar_messages
      if CP.enableBsm:
        messages += MqbExtraSignals.bsm_radar_messages

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.cam)

  @staticmethod
  def get_body_can_parser(CP): #can bus 1
    messages = [
      # sig_address, frequency
    ]
    #VAG
    #if CP.carFingerprint in (CAR.SKODA_KODIAQ_MK1):
    if CP.vagCarParams.vagCanModule.bus1.getriebe14:
      messages += MqbExtraSignals.getriebe_14_message
    if CP.vagCarParams.vagCanModule.bus1.motor12:
      messages += MqbExtraSignals.motor_12_message
    if CP.vagCarParams.vagCanModule.bus1.motor09:
      messages += MqbExtraSignals.motor_09_message
    if CP.vagCarParams.vagCanModule.bus1.obd01:
      messages += MqbExtraSignals.obd_01_message
    if CP.vagCarParams.vagCanModule.bus1.motor04:
      messages += MqbExtraSignals.motor_04_message

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.body)

  @staticmethod
  def get_info_can_parser(CP): #can bus 4
    messages = [
      # sig_address, frequency
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.vag_info)

  @staticmethod
  def get_gb_can_parser(CP): #can bus 5
    messages = [
      # sig_address, frequency
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.vag_gb)

  @staticmethod
  def get_pt_can_parser(CP): #can bus 6
    messages = [
      # sig_address, frequency
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.vag_pt)

  @staticmethod
  def get_can_parser_pq(CP):
    messages = [
      # sig_address, frequency
      ("Bremse_1", 100),    # From J104 ABS/ESP controller
      ("Bremse_3", 100),    # From J104 ABS/ESP controller
      ("Lenkhilfe_3", 100),  # From J500 Steering Assist with integrated sensors
      ("Lenkwinkel_1", 100),  # From J500 Steering Assist with integrated sensors
      ("Motor_3", 100),     # From J623 Engine control module
      ("Airbag_1", 50),     # From J234 Airbag control module
      ("Bremse_5", 50),     # From J104 ABS/ESP controller
      ("GRA_Neu", 50),      # From J??? steering wheel control buttons
      ("Kombi_1", 50),      # From J285 Instrument cluster
      ("Motor_2", 50),      # From J623 Engine control module
      ("Motor_5", 50),      # From J623 Engine control module
      ("Lenkhilfe_2", 20),  # From J500 Steering Assist with integrated sensors
      ("Gate_Komf_1", 10),  # From J533 CAN gateway
    ]

    if CP.transmissionType == TransmissionType.automatic:
      messages += [("Getriebe_1", 100)]  # From J743 Auto transmission control module
    elif CP.transmissionType == TransmissionType.manual:
      messages += [("Motor_1", 100)]  # From J623 Engine control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Extended CAN devices other than the camera are here on CANBUS.pt
      messages += PqExtraSignals.fwd_radar_messages
      if CP.enableBsm:
        messages += PqExtraSignals.bsm_radar_messages

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser_pq(CP):

    messages = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      messages += [
        # sig_address, frequency
        ("LDW_Status", 10)      # From R242 Driver assistance camera
      ]

    if CP.networkLocation == NetworkLocation.gateway:
      # Radars are here on CANBUS.cam
      messages += PqExtraSignals.fwd_radar_messages
      if CP.enableBsm:
        messages += PqExtraSignals.bsm_radar_messages

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CANBUS.cam)


class MqbExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_messages = [
    ("ACC_06", 50),                              # From J428 ACC radar control module
    ("ACC_10", 50),                              # From J428 ACC radar control module
    ("ACC_02", 17),                              # From J428 ACC radar control module
  ]
  bsm_radar_messages = [
    ("SWA_01", 20),                              # From J1086 Lane Change Assist
  ]
  #VAG
  motor_07_message = [
    ("Motor_07", 2),
  ]
  vehicle_speed_message = [
    ("VehicleSpeed", 50),
  ]
  bcm_01_message = [
    ("BCM_01", 1),
  ]
  kombi_02_message = [
    ("Kombi_02", 1),
  ]
  motor_18_message = [
    ("Motor_18", 1),
  ]
  charisma_01_message = [
    ("Charisma_01", 1),
  ]
  charisma_07_message = [
    ("Charisma_07", 1),
  ]
  getriebe_14_message = [
    ("Getriebe_14", 10),
  ]
  motor_12_message = [
    ("Motor_12", 100),
  ]
  motor_09_message = [
    ("Motor_09", 1),
  ]
  obd_01_message = [
    ("OBD_01", 1),
  ]
  motor_04_message = [
    ("Motor_04", 1),
  ]
  blinkmodi_02_message = [
    ("Blinkmodi_02", 5),
  ]
  parkhilfe_01_message = [
    ("Parkhilfe_01", 10),
  ]
  licht_anf_01_message = [
    ("Licht_Anf_01", 10),
  ]
  gateway_72_message = [
    ("Gateway_72", 10),
  ]

class PqExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_messages = [
    ("ACC_System", 50),                          # From J428 ACC radar control module
    ("ACC_GRA_Anzeige", 25),                     # From J428 ACC radar control module
  ]
  bsm_radar_messages = [
    ("SWA_1", 20),                               # From J1086 Lane Change Assist
  ]
