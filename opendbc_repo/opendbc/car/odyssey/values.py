from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, DbcDict, CarSpecs, PlatformConfig, Platforms, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column, Device

Ecu = structs.CarParams.Ecu
VisualAlert = structs.CarControl.HUDControl.VisualAlert
GearShifter = structs.CarState.GearShifter

class CarControllerParams:
  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  # NIDEC_ACCEL_MIN = -4.0  # m/s^2
  # NIDEC_ACCEL_MAX = 1.6  # m/s^2, lower than 2.0 m/s^2 for tuning reasons

  # NIDEC_ACCEL_LOOKUP_BP = [-1., 0., .6]
  # NIDEC_ACCEL_LOOKUP_V = [-4.8, 0., 2.0]

  # NIDEC_MAX_ACCEL_V = [0.5, 2.4, 1.4, 0.6]
  # NIDEC_MAX_ACCEL_BP = [0.0, 4.0, 10., 20.]

  # NIDEC_GAS_MAX = 198  # 0xc6
  # NIDEC_BRAKE_MAX = 1024 // 4

  # BOSCH_ACCEL_MIN = -3.5  # m/s^2
  # BOSCH_ACCEL_MAX = 2.0  # m/s^2

  # BOSCH_GAS_LOOKUP_BP = [-0.2, 2.0]  # 2m/s^2
  # BOSCH_GAS_LOOKUP_V = [0, 1600]

  # STEER_STEP = 1  # 100 Hz
  # STEER_DELTA_UP = 3  # min/max in 0.33s for all Honda
  # STEER_DELTA_DOWN = 3

  STEER_STEP = 1 # 100Hz
  STEER_MAX = 3  # Nm
  STEER_DELTA_UP = 3 / 100       # 3Nm/s
  STEER_DELTA_DOWN = 12 / 100     # 12 Nm/s
  STEER_ERROR_MAX = 999     # max delta between torque cmd and torque motor

  def __init__(self, CP):
    pass
    # self.STEER_MAX = CP.lateralParams.torqueBP[-1]
    # # mirror of list (assuming first item is zero) for interp of signed request values
    # assert(CP.lateralParams.torqueBP[0] == 0)
    # assert(CP.lateralParams.torqueBP[0] == 0)
    # self.STEER_LOOKUP_BP = [v * -1 for v in CP.lateralParams.torqueBP][1:][::-1] + list(CP.lateralParams.torqueBP)
    # self.STEER_LOOKUP_V = [v * -1 for v in CP.lateralParams.torqueV][1:][::-1] + list(CP.lateralParams.torqueV)


# class HondaSafetyFlags(IntFlag):
#   ALT_BRAKE = 1
#   BOSCH_LONG = 2
#   NIDEC_ALT = 4
#   RADARLESS = 8


# class HondaFlags(IntFlag):
#   # Detected flags
#   # Bosch models with alternate set of LKAS_HUD messages
#   BOSCH_EXT_HUD = 1
#   BOSCH_ALT_BRAKE = 2

#   # Static flags
#   BOSCH = 4
#   BOSCH_RADARLESS = 8

#   NIDEC = 16
#   NIDEC_ALT_PCM_ACCEL = 32
#   NIDEC_ALT_SCM_MESSAGES = 64

#   BOSCH_CANFD = 128

# Car button codes
class CruiseButtons:
  CANCEL = 3
  RES_ACCEL = 2
  DECEL_SET = 1


# class CruiseSettings:
#   DISTANCE = 3
#   LKAS = 1


# # See dbc files for info on values
# VISUAL_HUD = {
#   VisualAlert.none: 0,
#   VisualAlert.fcw: 1,
#   VisualAlert.steerRequired: 1,
#   VisualAlert.ldw: 1,
#   VisualAlert.brakePressed: 10,
#   VisualAlert.wrongGear: 6,
#   VisualAlert.seatbeltUnbuckled: 5,
#   VisualAlert.speedTooHigh: 8
# }


@dataclass
class HondaCarDocs(CarDocs):
  package: str = "StepperServoCAN"

  def init_make(self, CP: structs.CarParams):
    harness = CarHarness.custom
    self.car_parts = CarParts.common([Device.threex, harness])


# class Footnote(Enum):
#   CIVIC_DIESEL = CarFootnote(
#     "2019 Honda Civic 1.6L Diesel Sedan does not have ALC below 12mph.",
#     Column.FSR_STEERING)


# class HondaBoschPlatformConfig(PlatformConfig):
#   def init(self):
#     self.flags |= HondaFlags.BOSCH


# class HondaNidecPlatformConfig(PlatformConfig):
#   def init(self):
#     self.flags |= HondaFlags.NIDEC

@dataclass
class HondaOdysseyStepperServoConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'honda_odyssey_2005',
    # Bus.radar: RADAR.DELPHI_MRR,
  })
  def init(self):
    self.car_docs = []


class CAR(Platforms):
  HONDA_ODYSSEY_2005 = HondaOdysseyStepperServoConfig(
    [HondaCarDocs("Honda Odyssey 2005")],
    CarSpecs(mass=1700, wheelbase=3.0, steerRatio=16.2, centerToFrontRatio=0.45, tireStiffnessFactor=0.85), # TODO: centerToFrontRatio and tireStiffnessFactor are guesses
  )


STEER_THRESHOLD = {
  # default is 1200, overrides go here
  # CAR.ACURA_RDX: 400,
  # CAR.HONDA_CRV_EU: 400,
}

# HONDA_NIDEC_ALT_PCM_ACCEL = CAR.with_flags(HondaFlags.NIDEC_ALT_PCM_ACCEL)
# HONDA_NIDEC_ALT_SCM_MESSAGES = CAR.with_flags(HondaFlags.NIDEC_ALT_SCM_MESSAGES)
# HONDA_BOSCH = CAR.with_flags(HondaFlags.BOSCH)
# HONDA_BOSCH_RADARLESS = CAR.with_flags(HondaFlags.BOSCH_RADARLESS)
# HONDA_BOSCH_CANFD = CAR.with_flags(HondaFlags.BOSCH_CANFD)


DBC = CAR.create_dbc_map()
