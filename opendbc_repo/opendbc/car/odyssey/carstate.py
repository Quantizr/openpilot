import numpy as np
from collections import defaultdict

from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from opendbc.car import Bus, create_button_events, structs, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.odyssey.values import CAR, DBC, STEER_THRESHOLD , CruiseButtons
from opendbc.car.interfaces import CarStateBase
from opendbc.car.common.filter_simple import FirstOrderFilter

TransmissionType = structs.CarParams.TransmissionType
ButtonType = structs.CarState.ButtonEvent.Type

BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise, CruiseButtons.CANCEL: ButtonType.cancel}
# SETTINGS_BUTTONS_DICT = {CruiseSettings.DISTANCE: ButtonType.gapAdjustCruise, CruiseSettings.LKAS: ButtonType.lkas}


# def get_can_messages(CP, gearbox_msg):
#   messages = [
#     ("ENGINE_DATA", 100),
#     ("WHEEL_SPEEDS", 50),
#     ("STEERING_SENSORS", 100),
#     ("SEATBELT_STATUS", 10),
#     ("CRUISE", 10),
#     ("POWERTRAIN_DATA", 100),
#     ("CAR_SPEED", 10),
#     ("VSA_STATUS", 50),
#     ("STEER_STATUS", 100),
#     ("STEER_MOTOR_TORQUE", 0),  # TODO: not on every car
#   ]

#   if CP.carFingerprint == CAR.HONDA_ODYSSEY_CHN:
#     messages += [
#       ("SCM_FEEDBACK", 25),
#       ("SCM_BUTTONS", 50),
#     ]
#   else:
#     messages += [
#       ("SCM_FEEDBACK", 10),
#       ("SCM_BUTTONS", 25),
#     ]

#   if CP.carFingerprint in (CAR.HONDA_CRV_HYBRID, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.ACURA_RDX_3G, CAR.HONDA_E):
#     messages.append((gearbox_msg, 50))
#   else:
#     messages.append((gearbox_msg, 100))

#   if CP.flags & HondaFlags.BOSCH_ALT_BRAKE:
#     messages.append(("BRAKE_MODULE", 50))

#   if CP.carFingerprint in (HONDA_BOSCH | {CAR.HONDA_CIVIC, CAR.HONDA_ODYSSEY, CAR.HONDA_ODYSSEY_CHN}):
#     messages.append(("EPB_STATUS", 50))

#   if CP.carFingerprint in HONDA_BOSCH:
#     # these messages are on camera bus on radarless cars
#     if not CP.openpilotLongitudinalControl and CP.carFingerprint not in HONDA_BOSCH_RADARLESS:
#       messages += [
#         ("ACC_HUD", 10),
#         ("ACC_CONTROL", 50),
#       ]
#   else:  # Nidec signals
#     if CP.carFingerprint == CAR.HONDA_ODYSSEY_CHN:
#       messages.append(("CRUISE_PARAMS", 10))
#     else:
#       messages.append(("CRUISE_PARAMS", 50))

#   if CP.carFingerprint not in (CAR.HONDA_ACCORD, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.HONDA_CRV_HYBRID, CAR.HONDA_INSIGHT,
#                                CAR.ACURA_RDX_3G, CAR.HONDA_E, CAR.HONDA_ODYSSEY_CHN, CAR.HONDA_FREED, CAR.HONDA_HRV, *HONDA_BOSCH_RADARLESS,
#                                *HONDA_BOSCH_CANFD):
#     messages.append(("DOORS_STATUS", 3))

#   if CP.carFingerprint in HONDA_BOSCH_RADARLESS:
#     messages.append(("CRUISE_FAULT_STATUS", 50))
#   elif CP.openpilotLongitudinalControl:
#     messages.append(("STANDSTILL", 50))

#   return messages


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    # self.gearbox_msg = "GEARBOX"
    # if CP.carFingerprint == CAR.HONDA_ACCORD and CP.transmissionType == TransmissionType.cvt:
    #   self.gearbox_msg = "GEARBOX_15T"
    # elif CP.carFingerprint == CAR.HONDA_CIVIC_2022 and CP.transmissionType == TransmissionType.cvt:
    #   self.gearbox_msg = "GEARBOX_ALT"
    # elif CP.transmissionType == TransmissionType.manual:
    #   self.gearbox_msg = "GEARBOX_ALT_2"

    # self.main_on_sig_msg = "SCM_FEEDBACK"
    # if CP.carFingerprint in HONDA_NIDEC_ALT_SCM_MESSAGES:
    #   self.main_on_sig_msg = "SCM_BUTTONS"

    # if CP.transmissionType != TransmissionType.manual:
    self.shifter_values = can_define.dv["GEARBOX"]["GEAR_SHIFTER"]
    # self.steer_status_values = defaultdict(lambda: "UNKNOWN", can_define.dv["STEER_STATUS"]["STEER_STATUS"])

    self.brake_switch_prev = False
    self.brake_switch_active = False
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0

    # When available we use cp.vl["CAR_SPEED"]["ROUGH_CAR_SPEED_2"] to populate vEgoCluster
    # However, on cars without a digital speedometer this is not always present (HRV, FIT, CRV 2016, ILX and RDX)
    self.dash_speed_seen = False



    self.accurate_steer_angle_seen = False
    self.angle_offset = FirstOrderFilter(None, 60.0, DT_CTRL*100, initialized=False)

    self.wheel_speed_ratio = FirstOrderFilter(None, 0.5, DT_CTRL, initialized=False)
    self.steering_angle = FirstOrderFilter(None, 0.5, DT_CTRL, initialized=False)
    self.offset_counter = 0

    self.main_button = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_actuator = can_parsers[Bus.cam]
    # if self.CP.enableBsm:
    #   cp_body = can_parsers[Bus.body]

    ret = structs.CarState()

    # car params
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    prev_cruise_buttons = self.cruise_buttons
    prev_cruise_main = self.main_button
    # prev_cruise_setting = self.cruise_setting
    # self.cruise_setting = cp.vl["SCM_BUTTONS"]["CRUISE_SETTING"]
    self.cruise_buttons = cp.vl["CRUISE_CONTROL"]["CRUISE_BUTTONS"]
    self.main_button = cp.vl["CRUISE_CONTROL"]["CRUISE_MAIN"]

    # used for car hud message
    self.is_metric = True

    # ******************* parse out can *******************
    # STANDSTILL->WHEELS_MOVING bit can be noisy around zero, so use XMISSION_SPEED
    # panda checks if the signal is non-zero
    ret.standstill = cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] < 1e-5
    # # TODO: find a common signal across all cars
    # if self.CP.carFingerprint in (CAR.HONDA_ACCORD, CAR.HONDA_CIVIC_BOSCH, CAR.HONDA_CIVIC_BOSCH_DIESEL, CAR.HONDA_CRV_HYBRID, CAR.HONDA_INSIGHT,
    #                               CAR.ACURA_RDX_3G, CAR.HONDA_E, *HONDA_BOSCH_RADARLESS, *HONDA_BOSCH_CANFD):
    #   ret.doorOpen = bool(cp.vl["SCM_FEEDBACK"]["DRIVERS_DOOR_OPEN"])
    # elif self.CP.carFingerprint in (CAR.HONDA_ODYSSEY_CHN, CAR.HONDA_FREED, CAR.HONDA_HRV):
    #   ret.doorOpen = bool(cp.vl["SCM_BUTTONS"]["DRIVERS_DOOR_OPEN"])
    # else:
    #   ret.doorOpen = any([cp.vl["DOORS_STATUS"]["DOOR_OPEN_FL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_FR"],
    #                       cp.vl["DOORS_STATUS"]["DOOR_OPEN_RL"], cp.vl["DOORS_STATUS"]["DOOR_OPEN_RR"]])
    ret.doorOpen = any([cp.vl["BODY"]["DRIVER_DOOR_OPEN"], cp.vl["BODY"]["PASSENGER_DOOR_OPEN"]])
    # ret.seatbeltUnlatched = bool(cp.vl["SEATBELT_STATUS"]["SEATBELT_DRIVER_LAMP"] or not cp.vl["SEATBELT_STATUS"]["SEATBELT_DRIVER_LATCHED"])
    ret.seatbeltUnlatched = bool(cp.vl["BODY"]["SEATBELT_WARNING"])

    # steer_status = self.steer_status_values[cp.vl["STEER_STATUS"]["STEER_STATUS"]]
    # ret.steerFaultPermanent = steer_status not in ("NORMAL", "NO_TORQUE_ALERT_1", "NO_TORQUE_ALERT_2", "LOW_SPEED_LOCKOUT", "TMP_FAULT")
    # # LOW_SPEED_LOCKOUT is not worth a warning
    # # NO_TORQUE_ALERT_2 can be caused by bump or steering nudge from driver
    # ret.steerFaultTemporary = steer_status not in ("NORMAL", "LOW_SPEED_LOCKOUT", "NO_TORQUE_ALERT_2")

    # if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS:
    #   ret.accFaulted = bool(cp.vl["CRUISE_FAULT_STATUS"]["CRUISE_FAULT"])
    # else:
    #   # On some cars, these two signals are always 1, this flag is masking a bug in release
    #   # FIXME: find and set the ACC faulted signals on more platforms
    #   if self.CP.openpilotLongitudinalControl:
    #     ret.accFaulted = bool(cp.vl["STANDSTILL"]["BRAKE_ERROR_1"] or cp.vl["STANDSTILL"]["BRAKE_ERROR_2"])

    #   # Log non-critical stock ACC/LKAS faults if Nidec (camera)
    #   if self.CP.carFingerprint not in HONDA_BOSCH:
    #     ret.carFaultedNonCritical = bool(cp_cam.vl["ACC_HUD"]["ACC_PROBLEM"] or cp_cam.vl["LKAS_HUD"]["LKAS_PROBLEM"])

    # ret.espDisabled = cp.vl["VSA_STATUS"]["ESP_DISABLED"] != 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"],
    )
    v_wheel = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.0

    # blend in transmission speed at low speed, since it has more low speed accuracy
    v_weight = float(np.interp(v_wheel, v_weight_bp, v_weight_v))
    ret.vEgoRaw = (1. - v_weight) * cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] * CV.KPH_TO_MS * self.CP.wheelSpeedFactor + v_weight * v_wheel
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    self.dash_speed_seen = self.dash_speed_seen or cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] > 1e-3
    if self.dash_speed_seen:
      conversion = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
      ret.vEgoCluster = cp.vl["ENGINE_DATA"]["XMISSION_SPEED"] * conversion

    # ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEER_ANGLE"]
    # ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEER_ANGLE_RATE"]

    # ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
    #   250, cp.vl["SCM_FEEDBACK"]["LEFT_BLINKER"], cp.vl["SCM_FEEDBACK"]["RIGHT_BLINKER"])

    ret.leftBlinker = cp.vl["LIGHTS"]["LEFT_TURN_SIGNAL"] == 1
    ret.rightBlinker = cp.vl["LIGHTS"]["RIGHT_TURN_SIGNAL"] == 1

    ret.steeringPressed = False
    ret.steeringTorque = 0

    # ret.steeringPressed = ret.gasPressed # no torque sensor, so lightly pressing the gas indicates driver intention
    # if ret.steeringPressed and ret.leftBlinker:
    #   ret.steeringTorque = 1
    # elif ret.steeringPressed and  ret.rightBlinker:
    #   ret.steeringTorque = -1
    # else:
    #   ret.steeringTorque = 0

    # ret.brakeHoldActive = cp.vl["VSA_STATUS"]["BRAKE_HOLD_ACTIVE"] == 1

    # TODO: set for all cars
    # if self.CP.carFingerprint in (HONDA_BOSCH | {CAR.HONDA_CIVIC, CAR.HONDA_ODYSSEY, CAR.HONDA_ODYSSEY_CHN}):
    #   ret.parkingBrake = cp.vl["EPB_STATUS"]["EPB_STATE"] != 0

    # if self.CP.transmissionType == TransmissionType.manual:
    #   ret.clutchPressed = cp.vl["GEARBOX_ALT_2"]["GEAR_MT"] == 0
    #   if cp.vl["GEARBOX_ALT_2"]["GEAR_MT"] == 14:
    #     ret.gearShifter = GearShifter.reverse
    #   else:
    #     ret.gearShifter = GearShifter.drive
    # else:
    #   gear = int(cp.vl[self.gearbox_msg]["GEAR_SHIFTER"])
    #   ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear, None))

    gear = int(cp.vl["GEARBOX"]["GEAR_SHIFTER"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear, None))


    ret.gas = cp.vl["DRIVER_THROTTLE_POSITION"]["DRIVER_THROTTLE_POSITION"]
    ret.gasPressed = ret.gas > 1e-5

    # ret.steeringTorque = cp.vl["STEER_STATUS"]["STEER_TORQUE_SENSOR"]
    # ret.steeringTorqueEps = cp.vl["STEER_MOTOR_TORQUE"]["MOTOR_TORQUE"]
    # ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD.get(self.CP.carFingerprint, 1200)

    # if self.CP.carFingerprint in HONDA_BOSCH:
    #   # The PCM always manages its own cruise control state, but doesn't publish it
    #   if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS:
    #     ret.cruiseState.nonAdaptive = cp_cam.vl["ACC_HUD"]["CRUISE_CONTROL_LABEL"] != 0

    #   if not self.CP.openpilotLongitudinalControl:
    #     # ACC_HUD is on camera bus on radarless cars
    #     acc_hud = cp_cam.vl["ACC_HUD"] if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS else cp.vl["ACC_HUD"]
    #     ret.cruiseState.nonAdaptive = acc_hud["CRUISE_CONTROL_LABEL"] != 0
    #     ret.cruiseState.standstill = acc_hud["CRUISE_SPEED"] == 252.

    #     conversion = get_cruise_speed_conversion(self.CP.carFingerprint, self.is_metric)
    #     # On set, cruise set speed pulses between 254~255 and the set speed prev is set to avoid this.
    #     ret.cruiseState.speed = self.v_cruise_pcm_prev if acc_hud["CRUISE_SPEED"] > 160.0 else acc_hud["CRUISE_SPEED"] * conversion
    #     self.v_cruise_pcm_prev = ret.cruiseState.speed
    # else:
    #   ret.cruiseState.speed = cp.vl["CRUISE"]["CRUISE_SPEED_PCM"] * CV.KPH_TO_MS

    ret.cruiseState.speed = cp.vl["CRUISE_CONTROL"]["CRUISE_SPEED"] * CV.KPH_TO_MS

    # if self.CP.flags & HondaFlags.BOSCH_ALT_BRAKE:
    #   ret.brakePressed = cp.vl["BRAKE_MODULE"]["BRAKE_PRESSED"] != 0
    # else:
    #   # brake switch has shown some single time step noise, so only considered when
    #   # switch is on for at least 2 consecutive CAN samples
    #   # brake switch rises earlier than brake pressed but is never 1 when in park
    #   brake_switch_vals = cp.vl_all["POWERTRAIN_DATA"]["BRAKE_SWITCH"]
    #   if len(brake_switch_vals):
    #     brake_switch = cp.vl["POWERTRAIN_DATA"]["BRAKE_SWITCH"] != 0
    #     if len(brake_switch_vals) > 1:
    #       self.brake_switch_prev = brake_switch_vals[-2] != 0
    #     self.brake_switch_active = brake_switch and self.brake_switch_prev
    #     self.brake_switch_prev = brake_switch
    #   ret.brakePressed = (cp.vl["POWERTRAIN_DATA"]["BRAKE_PRESSED"] != 0) or self.brake_switch_active

    ret.brake = cp.vl["BRAKE_PRESSURE"]["BRAKE_PRESSURE"]
    ret.brakePressed = (cp.vl["POWERTRAIN_DATA"]["BRAKE_PRESSED"] != 0)
    ret.cruiseState.enabled = cp.vl["POWERTRAIN_DATA"]["CRUISE_ENGAGED"] != 0
    ret.cruiseState.available = bool(cp.vl["CRUISE_CONTROL"]["CRUISE_MAIN"])

    # Gets rid of Pedal Grinding noise when brake is pressed at slow speeds for some models
    # if self.CP.carFingerprint in (CAR.HONDA_PILOT, CAR.HONDA_RIDGELINE):
    #   if ret.brake > 0.1:
    #     ret.brakePressed = True

    # if self.CP.carFingerprint in HONDA_BOSCH:
    #   # TODO: find the radarless AEB_STATUS bit and make sure ACCEL_COMMAND is correct to enable AEB alerts
    #   if self.CP.carFingerprint not in HONDA_BOSCH_RADARLESS:
    #     ret.stockAeb = (not self.CP.openpilotLongitudinalControl) and bool(cp.vl["ACC_CONTROL"]["AEB_STATUS"] and cp.vl["ACC_CONTROL"]["ACCEL_COMMAND"] < -1e-5)
    # else:
    #   ret.stockAeb = bool(cp_cam.vl["BRAKE_COMMAND"]["AEB_REQ_1"] and cp_cam.vl["BRAKE_COMMAND"]["COMPUTER_BRAKE"] > 1e-5)
    ret.stockAeb = False

    self.acc_hud = False
    self.lkas_hud = False
    # if self.CP.carFingerprint not in HONDA_BOSCH:
    #   ret.stockFcw = cp_cam.vl["BRAKE_COMMAND"]["FCW"] != 0
    #   self.acc_hud = cp_cam.vl["ACC_HUD"]
    #   self.stock_brake = cp_cam.vl["BRAKE_COMMAND"]
    # if self.CP.carFingerprint in HONDA_BOSCH_RADARLESS:
    #   self.lkas_hud = cp_cam.vl["LKAS_HUD"]


    ret.steeringTorqueEps =  cp_actuator.vl['STEERING_STATUS']['STEERING_TORQUE']
    ssc_angle = cp_actuator.vl['STEERING_STATUS']['STEERING_ANGLE']
    ret.steerFaultTemporary = int(cp_actuator.vl['STEERING_STATUS']['CONTROL_STATUS']) & 0x4 != 0

    if v_wheel > 3: # m/s ~= 6.7mph
      self.wheel_speed_ratio.update(
        (cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FL"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RL"]) /
        (cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_FR"] + cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_RR"])
      )
      self.steering_angle.update(ssc_angle)
      if self.offset_counter < 100:
        self.offset_counter += 1
    else:
      self.wheel_speed_ratio.initialized = False
      self.steering_angle.initialized = False
      self.offset_counter = 0

    if self.offset_counter >= 100:
      self.accurate_steer_angle_seen = True

    if self.accurate_steer_angle_seen:
      if self.wheel_speed_ratio.x > 0.9995 and self.wheel_speed_ratio.x < 1.0005 and self.offset_counter >= 100 and cp.can_valid:
        self.angle_offset.update(self.steering_angle.x)
        self.offset_counter = 0

      if self.angle_offset.initialized:
        ret.steeringAngleOffsetDeg = self.angle_offset.x
        ret.steeringAngleDeg = ssc_angle - self.angle_offset.x

    ret.vehicleSensorsInvalid = not self.accurate_steer_angle_seen

    # if self.CP.enableBsm:
    #   # BSM messages are on B-CAN, requires a panda forwarding B-CAN messages to CAN 0
    #   # more info here: https://github.com/commaai/openpilot/pull/1867
    #   ret.leftBlindspot = cp_body.vl["BSM_STATUS_LEFT"]["BSM_ALERT"] == 1
    #   ret.rightBlindspot = cp_body.vl["BSM_STATUS_RIGHT"]["BSM_ALERT"] == 1

    ret.buttonEvents = [
      *create_button_events(self.cruise_buttons, prev_cruise_buttons, BUTTONS_DICT),
      *create_button_events(self.main_button, prev_cruise_main, {1: ButtonType.mainCruise}),
      # *create_button_events(self.cruise_setting, prev_cruise_setting, SETTINGS_BUTTONS_DICT),
    ]


    return ret

  # def get_can_parsers(self, CP, CP_SP):
  #   pt_messages = get_can_messages(CP, self.gearbox_msg)

  #   cam_messages = [
  #     ("STEERING_CONTROL", 100),
  #   ]

  #   if CP.carFingerprint in HONDA_BOSCH_RADARLESS:
  #     cam_messages += [
  #       ("ACC_HUD", 10),
  #       ("LKAS_HUD", 10),
  #     ]

  #   elif CP.carFingerprint not in HONDA_BOSCH:
  #     cam_messages += [
  #       ("ACC_HUD", 10),
  #       ("LKAS_HUD", 10),
  #       ("BRAKE_COMMAND", 50),
  #     ]

  #   body_messages = [
  #     ("BSM_STATUS_LEFT", 3),
  #     ("BSM_STATUS_RIGHT", 3),
  #   ]

  #   parsers = {
  #     Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).pt),
  #     Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).camera),
  #   }
  #   if CP.enableBsm:
  #     parsers[Bus.body] = CANParser(DBC[CP.carFingerprint][Bus.body], body_messages, CanBus(CP).radar)

  #   return parsers

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
    ("BODY", 3),
    ("BRAKE_PRESSURE", 143),
    ("CRUISE_CONTROL", 100),
    ("DRIVER_THROTTLE_POSITION", 100),
    ("ENGINE_DATA", 100),
    ("GEARBOX", 100),
    ("LIGHTS", 25),
    ("POWERTRAIN_DATA", 100),
    ("WHEEL_SPEEDS", 48),

  ]

    actuator_messages = [
      ("STEERING_STATUS", 100),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser('ocelot_controls', actuator_messages, 1),
    }

  # @staticmethod
  # def get_actuator_can_parser(CP):
  #   signals = [  # signal name, message name, default value
  #     ("STEERING_ANGLE", "STEERING_STATUS", 0),
  #     ("STEERING_TORQUE", "STEERING_STATUS", 0),
  #     ("STEERING_SPEED", "STEERING_STATUS", 0),
  #     ("CONTROL_STATUS", "STEERING_STATUS", 0),
  #     ("TEMPERATURE", "STEERING_STATUS", 0),
  #   ]
  #   checks = [ # refresh frequency Hz
  #   ("STEERING_STATUS", 100),
  #   ]
  #   return CANParser('ocelot_controls', signals, checks, 1)  # 1: Actuator-CAN,
