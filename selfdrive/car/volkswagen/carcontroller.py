#
# Copyright (c) 2020-2024 bluetulippon@gmail.com Chad_Peng(Pon).
# All Rights Reserved.
# Confidential and Proprietary - bluetulippon@gmail.com Chad_Peng(Pon).
#

import cereal.messaging as messaging
from cereal import car
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, MQB_LDW_MESSAGES, BUTTON_STATES, CarControllerParams as P
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0

    self.packer_pt = CANPacker(DBC_FILES.mqb)

    self.hcaSameTorqueCount = 0
    self.hcaEnabledFrameCount = 0
    self.graButtonStatesToSend = None
    self.graMsgSentCount = 0
    self.graMsgStartFramePrev = 0
    self.graMsgBusCounterPrev = 0

    self.steer_rate_limited = False

  def update(self, c, enabled, CS, frame, ext_bus, actuators, visual_alert, left_lane_visible, right_lane_visible, left_lane_depart, right_lane_depart):
    """ Controls thread """

    can_sends = []

    ###### BCM_01 #####
    if CS.out.vagCanModule.bus0Bcm01 and CS.out.vagCanModule.bus0Motor18:
      if c.disableVagStartStop:
        if frame % P.BCM_01_STEP == 0:
          if CS.motor_18["MO_Hybrid_StartStopp_LED"] == 0:
            can_sends.append(volkswagencan.create_bcm_01_control(self.packer_pt, CANBUS.body, CS.bcm_01, True))

    ###### CHARISMA_01 #####
    if CS.out.vagCanModule.bus0Charisma01 and CS.out.vagCanModule.bus0Charisma07:
      if c.enableVagDrivingMode or c.enableVagDynamicDcc:
        if frame % P.CHARISMA_01_STEP == 0:
          can_sends.append(volkswagencan.create_charisma_01_control(self.packer_pt, CANBUS.body, CS.charisma_01, CS.charisma_07, c.enableVagDrivingMode, c.vagDrivingMode, c.enableVagDynamicDcc, CS.out.vagUiField.speed, CS.out.steeringAngleDeg))

    # **** Steering Controls ************************************************ #
    if frame % P.HCA_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # One frame of HCA disabled is enough to reset the timer, without zeroing the
      # torque value. Do that anytime we happen to have 0 torque, or failing that,
      # when exceeding ~1/3 the 360 second timer.

      #Pon FLKA
      #print("[BOP][carcontroller.py][update()][FLKA] c.availableVagFlka=", c.availableVagFlka)
      if (c.active or c.availableVagFlka) and CS.out.vEgo > CS.CP.minSteerSpeed and not (CS.out.standstill or CS.out.steerError or CS.out.steerWarning):
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
        if apply_steer == 0:
          hcaEnabled = False
          self.hcaEnabledFrameCount = 0
        else:
          self.hcaEnabledFrameCount += 1
          if self.hcaEnabledFrameCount >= 118 * (100 / P.HCA_STEP):  # 118s
            hcaEnabled = False
            self.hcaEnabledFrameCount = 0
          else:
            hcaEnabled = True
            if self.apply_steer_last == apply_steer:
              self.hcaSameTorqueCount += 1
              if self.hcaSameTorqueCount > 1.9 * (100 / P.HCA_STEP):  # 1.9s
                apply_steer -= (1, -1)[apply_steer < 0]
                self.hcaSameTorqueCount = 0
            else:
              self.hcaSameTorqueCount = 0
      else:
        hcaEnabled = False
        apply_steer = 0

      self.apply_steer_last = apply_steer
      idx = (frame / P.HCA_STEP) % 16

      #Pon Blindspot info/warning vibrator
      #vibrator_threshold = 0.0
      vibratorEnabled = 0
      #accAvailable = CS.out.cruiseState.available
      #accEnabled = CS.out.cruiseState.enabled
      leftBlindspot = CS.out.leftBlindspot
      rightBlindspot = CS.out.rightBlindspot
      leftBlindspotWarning = CS.out.leftBlindspotWarning
      rightBlindspotWarning = CS.out.rightBlindspotWarning

      #print("[BOP][carcontroller.py][update()][Blindspot]", c.availableVagBlindspotInfoVibrator, c.availableVagBlindspotWarningVibrator)
      if (c.availableVagBlindspotInfoVibrator and (leftBlindspot or rightBlindspot)):
        #if (((not accAvailable and not accEnabled) or (accAvailable and isVagBlindspotVibratorWithFlka)) and (leftBlindspot or rightBlindspot)):
        vibratorEnabled = 1

      if (c.availableVagBlindspotWarningVibrator and (leftBlindspotWarning or rightBlindspotWarning)):
        #if (((not accAvailable and not accEnabled) or (accAvailable and isVagBlindspotVibratorWithFlka)) and (leftBlindspotWarning or rightBlindspotWarning)):
        vibratorEnabled = 2

      can_sends.append(volkswagencan.create_mqb_steering_control(self.packer_pt, CANBUS.pt, apply_steer,
                                                                 idx, hcaEnabled, vibratorEnabled))

    # **** HUD Controls ***************************************************** #
    if frame % P.LDW_STEP == 0:
      #Pon FLKA
      hudEnabled = True if (enabled or c.availableVagFlka) and not CS.out.standstill else False
      #print("[BOP][carcontroller.py][update()][FLKA] hudEnabled=", hudEnabled)

      if visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = MQB_LDW_MESSAGES["laneAssistTakeOverSilent"]
      else:
        hud_alert = MQB_LDW_MESSAGES["none"]

      #Pon FLKA
      can_sends.append(volkswagencan.create_mqb_hud_control(self.packer_pt, CANBUS.pt, hudEnabled,
                                                            CS.out.steeringPressed, hud_alert, left_lane_visible,
                                                            right_lane_visible, CS.ldw_stock_values,
                                                            left_lane_depart, right_lane_depart))

    # **** ACC Button Controls ********************************************** #

    # FIXME: this entire section is in desperate need of refactoring

    if CS.CP.pcmCruise:
      if frame > self.graMsgStartFramePrev + P.GRA_VBP_STEP:
        if not enabled and CS.out.cruiseState.enabled:
          # Cancel ACC if it's engaged with OP disengaged.
          self.graButtonStatesToSend = BUTTON_STATES.copy()
          self.graButtonStatesToSend["cancel"] = True
        elif enabled and CS.esp_hold_confirmation:
          # Blip the Resume button if we're engaged at standstill.
          # FIXME: This is a naive implementation, improve with visiond or radar input.
          self.graButtonStatesToSend = BUTTON_STATES.copy()
          self.graButtonStatesToSend["resumeCruise"] = True

      if CS.graMsgBusCounter != self.graMsgBusCounterPrev:
        self.graMsgBusCounterPrev = CS.graMsgBusCounter
        if self.graButtonStatesToSend is not None:
          if self.graMsgSentCount == 0:
            self.graMsgStartFramePrev = frame
          idx = (CS.graMsgBusCounter + 1) % 16
          can_sends.append(volkswagencan.create_mqb_acc_buttons_control(self.packer_pt, ext_bus, self.graButtonStatesToSend, CS, idx))
          self.graMsgSentCount += 1
          if self.graMsgSentCount >= P.GRA_VBP_COUNT:
            self.graButtonStatesToSend = None
            self.graMsgSentCount = 0

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX

    return new_actuators, can_sends
