#
# Copyright (c) 2020-2022 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, NetworkLocation, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams

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

    #VAG ACC
    ret.vagAcc.vagAcc02.accWunschgeschw = float(ext_cp.vl["ACC_02"]["ACC_Wunschgeschw"])
    ret.vagAcc.vagAcc02.accStatusPrimAnz = int(ext_cp.vl["ACC_02"]["ACC_Status_Prim_Anz"])
    ret.vagAcc.vagAcc02.accAbstandsindex = int(ext_cp.vl["ACC_02"]["ACC_Abstandsindex"])
    ret.vagAcc.vagAcc02.accAkustik = int(ext_cp.vl["ACC_02"]["ACC_Akustik"])
    ret.vagAcc.vagAcc02.accGesetzteZeitluecke = int(ext_cp.vl["ACC_02"]["ACC_Gesetzte_Zeitluecke"])
    ret.vagAcc.vagAcc02.accOptischerFahrerhinweis = bool(ext_cp.vl["ACC_02"]["ACC_Optischer_Fahrerhinweis"])
    ret.vagAcc.vagAcc02.accTypTachokranz = bool(ext_cp.vl["ACC_02"]["ACC_Typ_Tachokranz"])
    ret.vagAcc.vagAcc02.accAnzeigeZeitluecke = bool(ext_cp.vl["ACC_02"]["ACC_Anzeige_Zeitluecke"])
    ret.vagAcc.vagAcc02.accTachokranz = bool(ext_cp.vl["ACC_02"]["ACC_Tachokranz"])
    ret.vagAcc.vagAcc02.accDisplayPrio = bool(ext_cp.vl["ACC_02"]["ACC_Display_Prio"])
    ret.vagAcc.vagAcc02.accRelevantesObjekt = int(ext_cp.vl["ACC_02"]["ACC_Relevantes_Objekt"])
    ret.vagAcc.vagAcc02.accTextePrimaeranz = int(ext_cp.vl["ACC_02"]["ACC_Texte_Primaeranz"])
    ret.vagAcc.vagAcc02.accWunschgeschwErreicht = bool(ext_cp.vl["ACC_02"]["ACC_Wunschgeschw_erreicht"])
    ret.vagAcc.vagAcc02.accStatusAnzeige = int(ext_cp.vl["ACC_02"]["ACC_Status_Anzeige"])

    ret.vagAcc.vagAcc04.accTexteZusatzanz = int(ext_cp.vl["ACC_04"]["ACC_Texte_Zusatzanz"])
    ret.vagAcc.vagAcc04.accStatusZusatzanz = int(ext_cp.vl["ACC_04"]["ACC_Status_Zusatzanz"])
    ret.vagAcc.vagAcc04.accTexte = int(ext_cp.vl["ACC_04"]["ACC_Texte"])
    ret.vagAcc.vagAcc04.accTexteBrakingGuard = int(ext_cp.vl["ACC_04"]["ACC_Texte_braking_guard"])
    ret.vagAcc.vagAcc04.accWarnhinweis = bool(ext_cp.vl["ACC_04"]["ACC_Warnhinweis"])
    ret.vagAcc.vagAcc04.accGeschwZielfahrzeug = float(ext_cp.vl["ACC_04"]["ACC_Geschw_Zielfahrzeug"])
    ret.vagAcc.vagAcc04.accCharismaFahrPr = int(ext_cp.vl["ACC_04"]["ACC_Charisma_FahrPr"])
    ret.vagAcc.vagAcc04.accCharismaStatus = int(ext_cp.vl["ACC_04"]["ACC_Charisma_Status"])
    ret.vagAcc.vagAcc04.accCharismaUmschaltung = int(ext_cp.vl["ACC_04"]["ACC_Charisma_Umschaltung"])

    ret.vagAcc.vagAcc06.accLimitierteAnfahrdyn = bool(ext_cp.vl["ACC_06"]["ACC_limitierte_Anfahrdyn"])
    ret.vagAcc.vagAcc06.accZulRegelabwUnten = float(ext_cp.vl["ACC_06"]["ACC_zul_Regelabw_unten"])
    ret.vagAcc.vagAcc06.accStartStoppInfo = int(ext_cp.vl["ACC_06"]["ACC_StartStopp_Info"])
    ret.vagAcc.vagAcc06.accSollbeschleunigung02 = float(ext_cp.vl["ACC_06"]["ACC_Sollbeschleunigung_02"])
    ret.vagAcc.vagAcc06.accZulRegelabwOben = float(ext_cp.vl["ACC_06"]["ACC_zul_Regelabw_oben"])
    ret.vagAcc.vagAcc06.accNegSollbeschlGrad02 = float(ext_cp.vl["ACC_06"]["ACC_neg_Sollbeschl_Grad_02"])
    ret.vagAcc.vagAcc06.accPosSollbeschlGrad02 = float(ext_cp.vl["ACC_06"]["ACC_pos_Sollbeschl_Grad_02"])
    ret.vagAcc.vagAcc06.accAnfahren = bool(ext_cp.vl["ACC_06"]["ACC_Anfahren"])
    ret.vagAcc.vagAcc06.accAnhalten = bool(ext_cp.vl["ACC_06"]["ACC_Anhalten"])
    ret.vagAcc.vagAcc06.accTyp = int(ext_cp.vl["ACC_06"]["ACC_Typ"])
    ret.vagAcc.vagAcc06.accStatusAcc = int(ext_cp.vl["ACC_06"]["ACC_Status_ACC"])
    ret.vagAcc.vagAcc06.accMinimaleBremsung = bool(ext_cp.vl["ACC_06"]["ACC_Minimale_Bremsung"])

    ret.vagAcc.vagAcc07.accDistanceToStop = float(ext_cp.vl["ACC_07"]["ACC_Distance_to_Stop"])
    ret.vagAcc.vagAcc07.accHoldRequest = bool(ext_cp.vl["ACC_07"]["ACC_Hold_Request"])
    ret.vagAcc.vagAcc07.accBoostRequest = bool(ext_cp.vl["ACC_07"]["ACC_Boost_Request"])
    ret.vagAcc.vagAcc07.accFreewheelRequest = bool(ext_cp.vl["ACC_07"]["ACC_Freewheel_Request"])
    ret.vagAcc.vagAcc07.accFreewheelType = int(ext_cp.vl["ACC_07"]["ACC_Freewheel_Type"])
    ret.vagAcc.vagAcc07.accHoldType = int(ext_cp.vl["ACC_07"]["ACC_Hold_Type"])
    ret.vagAcc.vagAcc07.accHoldRelease = bool(ext_cp.vl["ACC_07"]["ACC_Hold_Release"])
    ret.vagAcc.vagAcc07.accAccelSecondary = float(ext_cp.vl["ACC_07"]["ACC_Accel_Secondary"])
    ret.vagAcc.vagAcc07.accAccelTsk = float(ext_cp.vl["ACC_07"]["ACC_Accel_TSK"])

    ret.vagAcc.vagAcc10.awv1AnfPrefill = bool(ext_cp.vl["ACC_10"]["AWV1_Anf_Prefill"])
    ret.vagAcc.vagAcc10.anbCmInfo = bool(ext_cp.vl["ACC_10"]["ANB_CM_Info"])
    ret.vagAcc.vagAcc10.awv2Freigabe = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
    ret.vagAcc.vagAcc10.awv1HbaParam = int(ext_cp.vl["ACC_10"]["AWV1_HBA_Param"])
    ret.vagAcc.vagAcc10.awv2Ruckprofil = int(ext_cp.vl["ACC_10"]["AWV2_Ruckprofil"])
    ret.vagAcc.vagAcc10.awv2Priowarnung = bool(ext_cp.vl["ACC_10"]["AWV2_Priowarnung"])
    ret.vagAcc.vagAcc10.anbCmAnforderung = bool(ext_cp.vl["ACC_10"]["ANB_CM_Anforderung"])
    ret.vagAcc.vagAcc10.anbInfoTeilbremsung = bool(ext_cp.vl["ACC_10"]["ANB_Info_Teilbremsung"])
    ret.vagAcc.vagAcc10.anbNotfallblinken = bool(ext_cp.vl["ACC_10"]["ANB_Notfallblinken"])
    ret.vagAcc.vagAcc10.anbTeilbremsungFreigabe = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"])
    ret.vagAcc.vagAcc10.anbZielbremsTeilbremsVerzAnf = float(ext_cp.vl["ACC_10"]["ANB_Zielbrems_Teilbrems_Verz_Anf"])
    ret.vagAcc.vagAcc10.anbZielbremsungFreigabe = bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])
    ret.vagAcc.vagAcc10.awvVorstufe = bool(ext_cp.vl["ACC_10"]["AWV_Vorstufe"])
    ret.vagAcc.vagAcc10.awvHalten = bool(ext_cp.vl["ACC_10"]["AWV_Halten"])

    ret.vagAcc.vagGraAcc01.graHauptschalter = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Hauptschalter"])
    ret.vagAcc.vagGraAcc01.graAbbrechen = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Abbrechen"])
    ret.vagAcc.vagGraAcc01.graTypHauptschalter = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Typ_Hauptschalter"])
    ret.vagAcc.vagGraAcc01.graLimiter = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Limiter"])
    ret.vagAcc.vagGraAcc01.graTipSetzen = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Setzen"])
    ret.vagAcc.vagGraAcc01.graTipHoch = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Hoch"])
    ret.vagAcc.vagGraAcc01.graTipRunter = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Runter"])
    ret.vagAcc.vagGraAcc01.graTipWiederaufnahme = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Wiederaufnahme"])
    ret.vagAcc.vagGraAcc01.graVerstellungZeitluecke = int(pt_cp.vl["GRA_ACC_01"]["GRA_Verstellung_Zeitluecke"])
    ret.vagAcc.vagGraAcc01.graCodierung = int(pt_cp.vl["GRA_ACC_01"]["GRA_Codierung"])
    ret.vagAcc.vagGraAcc01.graFehler = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Fehler"])
    ret.vagAcc.vagGraAcc01.graTyp468 = int(pt_cp.vl["GRA_ACC_01"]["GRA_Typ468"])
    ret.vagAcc.vagGraAcc01.graTipStufe2 = bool(pt_cp.vl["GRA_ACC_01"]["GRA_Tip_Stufe_2"])
    ret.vagAcc.vagGraAcc01.graButtonTypeInfo = int(pt_cp.vl["GRA_ACC_01"]["GRA_ButtonTypeInfo"])

    #VAG Traffic sign recognition
    #ret.vagTsr.vzeAnzeigemodus = int(ext_cp.vl["VZE_01"]["VZE_Anzeigemodus"])
    #ret.vagTsr.vzeHinweistext = int(ext_cp.vl["VZE_01"]["VZE_Hinweistext"])
    #ret.vagTsr.vzeStatuszaehler1 = int(ext_cp.vl["VZE_01"]["VZE_Statuszaehler_1"])
    #ret.vagTsr.vzeStatuszaehler2 = int(ext_cp.vl["VZE_01"]["VZE_Statuszaehler_2"])
    #ret.vagTsr.vzeStatuszaehler3 = int(ext_cp.vl["VZE_01"]["VZE_Statuszaehler_3"])
    #ret.vagTsr.vzeVerkehrszeichen1 = int(ext_cp.vl["VZE_01"]["VZE_Verkehrszeichen_1"])
    #ret.vagTsr.vzeVerkehrszeichen2 = int(ext_cp.vl["VZE_01"]["VZE_Verkehrszeichen_2"])
    #ret.vagTsr.vzeVerkehrszeichen3 = int(ext_cp.vl["VZE_01"]["VZE_Verkehrszeichen_3"])
    #ret.vagTsr.vzeWarnungVerkehrszeichen1 = bool(ext_cp.vl["VZE_01"]["VZE_Warnung_Verkehrszeichen_1"])
    #ret.vagTsr.vzeWarnungVerkehrszeichen2 = bool(ext_cp.vl["VZE_01"]["VZE_Warnung_Verkehrszeichen_2"])
    #ret.vagTsr.vzeWarnungVerkehrszeichen3 = bool(ext_cp.vl["VZE_01"]["VZE_Warnung_Verkehrszeichen_3"])
    #ret.vagTsr.vzeZusatzschild1 = int(ext_cp.vl["VZE_01"]["VZE_Zusatzschild_1"])
    #ret.vagTsr.vzeZusatzschild2 = int(ext_cp.vl["VZE_01"]["VZE_Zusatzschild_2"])
    #ret.vagTsr.vzeZusatzschild3 = int(ext_cp.vl["VZE_01"]["VZE_Zusatzschild_3"])

    #try:
      #print("[PONTEST][carstate.py][update()] MO_QBit_Ansaugluft_Temp=", pt_cp.vl["Motor_07"]["MO_QBit_Ansaugluft_Temp"])
      #print("[PONTEST][carstate.py][update()] MO_QBit_Oel_Temp=", pt_cp.vl["Motor_07"]["MO_QBit_Oel_Temp"])
      #print("[PONTEST][carstate.py][update()] MO_QBit_Kuehlmittel_Temp=", pt_cp.vl["Motor_07"]["MO_QBit_Kuehlmittel_Temp"])
      #print("[PONTEST][carstate.py][update()] vagEngineInAirTemperature=", pt_cp.vl["Motor_07"]["MO_Ansaugluft_Temp"])
      #print("[PONTEST][carstate.py][update()] vagEngineOilTemperature=", pt_cp.vl["Motor_07"]["MO_Oel_Temp"])
      #print("[PONTEST][carstate.py][update()] vagEngineCoolantTemperature=", pt_cp.vl["Motor_07"]["MO_Kuehlmittel_Temp"])
      #print("[PONTEST][carstate.py][update()] vagGearboxSumpfTemperatur=", pt_cp.vl["Getriebe_14"]["GE_Sumpftemperatur"])
    #except:
      #print("[PONTEST][carstate.py][update()] Get Temperature Fail!", pt_cp.vl["Motor_07"]["MO_Kuehlmittel_Temp"])

    ret.vagTemperatureInfo.vagEngineInAirTemperature = float(pt_cp.vl["Motor_07"]["MO_Ansaugluft_Temp"])
    ret.vagTemperatureInfo.vagEngineOilTemperature = int(pt_cp.vl["Motor_07"]["MO_Oel_Temp"])
    ret.vagTemperatureInfo.vagEngineCoolantTemperature = float(pt_cp.vl["Motor_07"]["MO_Kuehlmittel_Temp"])
    ret.vagTemperatureInfo.vagGearboxSumpfTemperature = int(body_cp.vl["Getriebe_14"]["GE_Sumpftemperatur"])

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
      ("Comfort_Signal_Left", "Blinkmodi_02"),   # Left turn signal including comfort blink interval
      ("Comfort_Signal_Right", "Blinkmodi_02"),  # Right turn signal including comfort blink interval
      ("AB_Gurtschloss_FA", "Airbag_02"),        # Seatbelt status, driver
      ("AB_Gurtschloss_BF", "Airbag_02"),        # Seatbelt status, passenger
      ("ESP_Fahrer_bremst", "ESP_05"),           # Brake pedal pressed
      ("ESP_Bremsdruck", "ESP_05"),              # Brake pressure applied
      ("ESP_Status_Bremsdruck", "ESP_05"),       # Brakes applied
      ("MO_Fahrpedalrohwert_01", "Motor_20"),    # Accelerator pedal value
      ("MO_QBit_Ansaugluft_Temp", "Motor_07"),
      ("MO_QBit_Oel_Temp", "Motor_07"),
      ("MO_QBit_Kuehlmittel_Temp", "Motor_07"),
      ("MO_Ansaugluft_Temp", "Motor_07"),
      ("MO_Oel_Temp", "Motor_07"),
      ("MO_Kuehlmittel_Temp", "Motor_07"),
      #("OBD_Eng_Cool_Temp", "OBD_01"),
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
      ("GRA_Limiter", "GRA_ACC_01"),
      ("GRA_Limiter", "GRA_ACC_01"),
      ("GRA_Codierung", "GRA_ACC_01"),
      ("GRA_Fehler", "GRA_ACC_01"),
      ("GRA_Typ468", "GRA_ACC_01"),
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
      ("Motor_07", 2),
      #("OBD_01", 10),
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
    signals = [
      # sig_name, sig_address
      ("GE_Sumpftemperatur", "Getriebe_14"),
    ]

    checks = [
      # sig_address, frequency
      ("Getriebe_14", 10),
    ]

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
    ("AWV2_Freigabe", "ACC_10"),                    # FCW brake jerk release
    ("AWV1_HBA_Param", "ACC_10"),
    ("AWV2_Ruckprofil", "ACC_10"),
    ("AWV2_Priowarnung", "ACC_10"),
    ("ANB_CM_Anforderung", "ACC_10"),
    ("ANB_Info_Teilbremsung", "ACC_10"),
    ("ANB_Notfallblinken", "ACC_10"),
    ("ANB_Teilbremsung_Freigabe", "ACC_10"),        # AEB partial braking release
    ("ANB_Zielbrems_Teilbrems_Verz_Anf", "ACC_10"),
    ("ANB_Zielbremsung_Freigabe", "ACC_10"),        # AEB target braking release
    ("AWV_Vorstufe", "ACC_10"),
    ("AWV_Halten", "ACC_10"),
    #("VZE_Anzeigemodus", "VZE_01"),
    #("VZE_Hinweistext", "VZE_01"),
    #("VZE_Statuszaehler_1", "VZE_01"),
    #("VZE_Statuszaehler_2", "VZE_01"),
    #("VZE_Statuszaehler_3", "VZE_01"),
    #("VZE_Verkehrszeichen_1", "VZE_01"),
    #("VZE_Verkehrszeichen_2", "VZE_01"),
    #("VZE_Verkehrszeichen_3", "VZE_01"),
    #("VZE_Warnung_Verkehrszeichen_1", "VZE_01"),
    #("VZE_Warnung_Verkehrszeichen_2", "VZE_01"),
    #("VZE_Warnung_Verkehrszeichen_3", "VZE_01"),
    #("VZE_Zusatzschild_1", "VZE_01"),
    #("VZE_Zusatzschild_2", "VZE_01"),
    #("VZE_Zusatzschild_3", "VZE_01"),
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
    ("SWA_Infostufe_SWA_li", "SWA_01"),             # Blind spot object info, left
    ("SWA_Warnung_SWA_li", "SWA_01"),               # Blind spot object warning, left
    ("SWA_Infostufe_SWA_re", "SWA_01"),             # Blind spot object info, right
    ("SWA_Warnung_SWA_re", "SWA_01"),               # Blind spot object warning, right
  ]
  bsm_radar_checks = [
    ("SWA_01", 20),                                 # From J1086 Lane Change Assist
  ]
