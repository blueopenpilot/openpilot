#
# Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# you can rename the struct, but don't change the identifier

struct VagCarState @0x81c2f05a394cf4af {
  brakeLights @0 :Bool;
  leftBlindspotWarning @1 :Bool; # Is there something blocking the right lane change
  rightBlindspotWarning @2 :Bool; # Is there something blocking the right lane change
  vagUiField @3 :VagUiField;

  struct VagUiField {
    moIstgang @0 :Int32;                #gear level
    moLadedruck @1 :Float32;            #engine turbo pressure          (HUD)
    moOeldruck @2 :Float32;             #engine oil pressure            (HUD)
    moAnsaugluftTemp @3 :Float32;       #engine in air temperature      (HUD)
    moKuehlmittelTemp @4 :Float32;      #engine coolant temperature     (HUD)
    moOelTemp @5 :Float32;              #engine oil temperature         (HUD)
    moItmKuehlmittelTemp @6 :Float32;   #coolant temperature            (HUD)
    moMaxLadedruck @7 :Float32;         #engine turbo max pressure
    moRelSaugrohrdruck @8 :Float32;     #engine in air pressure         (HUD)
    moRelSaugrohrdruckGemErr @9 :Int32; #engine in air pressure error
    geZielgang @10 :Int32;              #gear level                     (HUD)
    geSumpftemperatur @11 :Float32;     #gear oil temperature           (HUD)
    espBremsdruck @12 :Float32;         #brake pressure                 (HUD)
    espBvkUnterdruck @13 :Float32;      #brake BKV negative pressure
    bcm1AussenTempUngef @14 :Float32;   #outdoor temperature             (HUD)
    obdEngCoolTemp @15 :Float32;        #OBD engine oil temperature
    kbiAussenTempGef @16 :Float32;      #indoor temperature            (HUD)
    accAbstandsindex @17 :Int32;        #acc abstand index
    speed @18 :Float32;
  }
}

struct VagCarControl @0xaedffd8f31e7b55d {
  availableVagFlka @0 :Bool;
  availableVagBlindspotInfoVibrator @1 :Bool;
  availableVagBlindspotWarningVibrator @2 :Bool;
  disableVagStartStop @3 :Bool;
  enableVagDrivingMode @4 :Bool;
  vagDrivingMode @5 :VagDrivingMode;
  enableVagDynamicDcc @6 :Bool;
  vagAudibleAlert @7 :VagAudibleAlert;
  enableLeftBlinker @8 :Bool;
  enableRightBlinker @9 :Bool;
  enableAreaView @10: Bool;

  enum VagDrivingMode {
    notSet @0;
    comfort @1;
    normal @2;
    sport @3;
    offroad @4;
    eco @5;
    race @6;
    individual @7;
    on @8;
    off @9;
    snow @10;
    config11 @11;
    config12 @12;
    sand @13;
    config14 @14;
    config15 @15;
  }

  enum VagAudibleAlert {
    leftBlinker @0;
    rightBlinker @1;
    leftBlindspot @2;
    rightBlindspot @3;
    leadCarGoing @4;
    noLeadCarWarning @5;
  }
}

struct VagCarParams @0xf35cc4560bbf6ec2 {
  vagCanModule @0 :VagCanModule;

  struct VagCanModule {
    bus0 @0 :Bus0;
    bus1 @1 :Bus1;
    bus2 @2 :Bus2;
    bus3 @3 :Bus3;
    bus4 @4 :Bus4;
    bus5 @5 :Bus5;
    bus6 @6 :Bus6;

    #----- bus 0 -----
    struct Bus0 {
      vehicleSpeed @0 :Bool;
      charisma01 @1: Bool;
      charisma07 @2: Bool;
      motor07 @3 :Bool;
      bcm01 @4: Bool;
      motor18 @5: Bool;
      kombi02 @6: Bool;
    }
    #----- bus 1 -----
    struct Bus1 {
      motor12 @0: Bool;
      motor04 @1: Bool;
      obd01 @2: Bool;
      getriebe14 @3: Bool;
      motor09 @4: Bool;
    }
    #----- bus 2 -----
    struct Bus2 {
    }
    #----- bus 3 -----
    struct Bus3 {
    }
    #----- bus 4 -----
    struct Bus4 {
    }
    #----- bus 5 -----
    struct Bus5 {
    }
    #----- bus 6 -----
    struct Bus6 {
    }
  }
}

struct VagParam @0xda96579883444c35 {
  vagParamOp @0: VagParamOp;
  vagParamGeneral @1: VagParamGeneral;
  vagParamOsd @2: VagParamOsd;
  vagParamTest @3: VagParamTest;
  vagParamSetting @4: VagParamSetting;
  vagParamFeature @5: VagParamFeature;
  vagParamWarning @6: VagParamWarning;

  # ===== OP toggle =====
  struct VagParamOp {
    experimentalLongitudinalEnabled @0 :Bool;
    experimentalMode @1 :Bool;
  }

  # ===== General =====
  struct VagParamGeneral {
    isVagDisableDriverMonitorAlert @0 :Bool;
    isVagLeftBlinkerSoundEnabled @1 :Bool;
    isVagRightBlinkerSoundEnabled @2 :Bool;
    isVagDevelopOnRoadUi @3 :Bool;
    isVagPandaJungleEnabled @4 :Bool;
    isVagDevelopModeEnabled @5 :Bool;
    isVagRunningProcessLogEnabled @6 :Bool;
    isVagParamFromCerealEnabled @7 :Bool;
  }

  # ===== OSD =====
  struct VagParamOsd {
    isVagDebugBlinkerTest @0 :Bool;
    isVagDebugBlindspotInfoTest @1 :Bool;
    isVagDebugBlindspotWarningTest @2 :Bool;
    isVagDebugBrakeLightTest @3 :Bool;
    isVagDebugLeadCarGoingRemindTest @4 :Bool;
    isVagDebugNoLeadCarWarningTest @5 :Bool;
  }

  # ===== Test =====
  struct VagParamTest {
    isVagDebugOsdTestTextEnabled @0 :Bool;
    isVagRadarAccTestTextEnabled @1 :Bool;
    isVagVisionAccTestTextEnabled @2 :Bool;
    isVagDebugItem1Enabled @3 :Bool;
    isVagDebugItem2Enabled @4 :Bool;
    isVagDebugItem3Enabled @5 :Bool;
    isVagDebugItem4Enabled @6 :Bool;
    isVagDebugItem5Enabled @7 :Bool;
  }

  # ===== Setting =====
  struct VagParamSetting {
    isVagManualSoundVolumeEnable @0 :Bool;
    vagSoundVolume @1 :Int32;
    isVagManualOsdBacklightEnable @2 :Bool;
    vagOsdBacklight @3 :Int32;
    isVagInfoBoxEnabled @4 :Bool;
    isVagBlinkerEnabled @5 :Bool;
    isVagBrakeLightEnabled @6 :Bool;
    isVagLeadCarEnabled @7 :Bool;
  }

  # ===== Feature =====
  struct VagParamFeature {
    # ----- Blindspot -----
    isVagBlindspotEnabled @0 :Bool;
    isVagBlindspotInfoSoundEnabled @1 :Bool;
    isVagBlindspotInfoVibratorEnabled @2 :Bool;
    isVagBlindspotWarningSoundEnabled @3 :Bool;
    isVagBlindspotWarningVibratorEnabled @4 :Bool;
    isVagBlindspotVibratorWithFlka @5 :Bool;
    # ----- FLKA -----
    isVagFulltimeLkaEnabled @6 :Bool;
    isVagFulltimeLkaEnableWithBlinker @7 :Bool;
    isVagFulltimeLkaEnableWithBrake @8 :Bool;
    isVagFulltimeLkaEnableWithAssistant @9 :Bool;
    # ----- Lead car going -----
    isVagLeadCarGoingRemindEnabled @10 :Bool;
    isVagLeadCarGoingRemindSoundEnabled @11 :Bool;
    # ----- No lead car -----
    isVagNoLeadCarEnabled @12 :Bool;
    isVagNoLeadCarWarningSoundEnabled @13 :Bool;
    # ----- Force disable startstop -----
    isVagForceDisableStartstop @14 :Bool;
    # ----- Driving Mode -----
    isVagDrivingModeEnabled @15 :Bool;
    vagDrivingMode @16 :VagCarControl.VagDrivingMode;
    isVagDynamicDccEnabled @17 :Bool;
  }

  # ===== Warning =====
  struct VagParamWarning {
    isVagWarningEngineTurboPressure @0 :Bool;
    isVagWarningEngineOilPressure @1 :Bool;
    isVagWarningEngineInAirTemperature @2 :Bool;
    isVagWarningEngineCoolantTemperature @3 :Bool;
    isVagWarningEngineOilTemperature @4 :Bool;
    isVagWarningCoolantTemperature @5 :Bool;
    isVagWarningInAirPressure @6 :Bool;
    isVagWarningGearOilTemperature @7 :Bool;
    isVagWarningBrakePressure @8 :Bool;
    isVagWarningIndoorTemperature @9 :Bool;
    isVagWarningOutdoorTemperature @10 :Bool;
  }
}

struct VagControl @0x80ae746ee2596b11 {
  leadCarGoingTrigged @0 :Bool;
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}
