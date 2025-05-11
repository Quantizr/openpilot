#pragma once

#include "safety_declarations.h"

// // All common address checks except SCM_BUTTONS which isn't on one Nidec safety configuration
// #define HONDA_COMMON_NO_SCM_FEEDBACK_RX_CHECKS(pt_bus)                                                         \
//   {.msg = {{0x1A6, (pt_bus), 8, .max_counter = 3U, .frequency = 25U},                  /* SCM_BUTTONS */       \
//            {0x296, (pt_bus), 4, .max_counter = 3U, .frequency = 25U}, { 0 }}},                                 \
//   {.msg = {{0x158, (pt_bus), 8, .max_counter = 3U, .frequency = 100U}, { 0 }, { 0 }}},  /* ENGINE_DATA */      \
//   {.msg = {{0x17C, (pt_bus), 8, .max_counter = 3U, .frequency = 100U}, { 0 }, { 0 }}},  /* POWERTRAIN_DATA */  \

// #define HONDA_COMMON_RX_CHECKS(pt_bus)                                                                     \
//   HONDA_COMMON_NO_SCM_FEEDBACK_RX_CHECKS(pt_bus)                                                           \
//   {.msg = {{0x326, (pt_bus), 8, .max_counter = 3U, .frequency = 10U}, { 0 }, { 0 }}},  /* SCM_FEEDBACK */  \

// // Alternate brake message is used on some Honda Bosch, and Honda Bosch radarless (where PT bus is 0)
// #define HONDA_ALT_BRAKE_ADDR_CHECK(pt_bus)                                                                 \
//   {.msg = {{0x1BE, (pt_bus), 3, .max_counter = 3U, .frequency = 50U}, { 0 }, { 0 }}},  /* BRAKE_MODULE */  \

#define CAN_ACTUATOR_POS_FAC 0.125
#define CAN_ACTUATOR_TQ_FAC 0.125
#define CAN_ACTUATOR_CONTROL_STATUS_SOFTOFF_BIT 2

bool honda_fmax_limit_check(float val, const float MAX_VAL, const float MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

enum {
  HONDA_BTN_NONE = 0,
  // HONDA_BTN_MAIN = 1,
  HONDA_BTN_SET = 1,
  HONDA_BTN_RESUME = 2,
  HONDA_BTN_CANCEL = 3,
};

static int honda_brake = 0;
static bool honda_brake_switch_prev = false;
static bool honda_alt_brake_msg = false;
static bool honda_fwd_brake = false;
static bool honda_bosch_long = false;
static bool honda_bosch_radarless = false;
typedef enum {HONDA_NIDEC, HONDA_BOSCH} HondaHw;
static HondaHw honda_hw = HONDA_NIDEC;




#define KPH_TO_MS 0.277778

// rounding error margin
float BMW_MARGIN = 0.1;

const struct lookup_t BMW_LOOKUP_MAX_ANGLE = {
    {5., 15., 30.},     // m/s
    {200., 20., 10.}};  // deg

const struct lookup_t BMW_ANGLE_RATE_WINDUP = { // deg/s windup rate limit
    {0., 5., 15.},      // m/s
    {500., 80., 15.}};  // deg/s

const struct lookup_t BMW_ANGLE_RATE_UNWIND = { // deg/s unwind rate limit
    {0., 5., 15.},      // m/s
    {500., 350., 40.}}; // deg/s

const struct lookup_t BMW_MAX_TQ_RATE = {
    {0., 5., 15.},      // m/s
    {16., 8., 1.}};   // Nm/10ms


// state of angle limits
float honda_desired_angle_last = 0; // last desired steer angle
float honda_rt_angle_last = 0.; // last actual angle

float angle_rate_up = 0;
float angle_rate_down = 0;
float honda_max_angle = 0;
float max_tq_rate = 0;

float honda_speed = 0;
float actuator_torque;


static int honda_get_pt_bus(void) {
  return 0;
}

static uint32_t honda_get_checksum(const CANPacket_t *to_push) {
  int checksum_byte = GET_LEN(to_push) - 1U;
  return (uint8_t)(GET_BYTE(to_push, checksum_byte)) & 0xFU;
}

static uint32_t honda_compute_checksum(const CANPacket_t *to_push) {
  int len = GET_LEN(to_push);
  uint8_t checksum = 0U;
  unsigned int addr = GET_ADDR(to_push);
  while (addr > 0U) {
    checksum += (uint8_t)(addr & 0xFU); addr >>= 4;
  }
  for (int j = 0; j < len; j++) {
    uint8_t byte = GET_BYTE(to_push, j);
    checksum += (uint8_t)(byte & 0xFU) + (byte >> 4U);
    if (j == (len - 1)) {
      checksum -= (byte & 0xFU);  // remove checksum in message
    }
  }
  return (uint8_t)((8U - checksum) & 0xFU);
}

static uint8_t honda_get_counter(const CANPacket_t *to_push) {
  int counter_byte = GET_LEN(to_push) - 1U;
  return (GET_BYTE(to_push, counter_byte) >> 4U) & 0x3U;
}

static void honda_rx_hook(const CANPacket_t *to_push) {
  const bool pcm_cruise = true; // ((honda_hw == HONDA_BOSCH) && !honda_bosch_long) || (honda_hw == HONDA_NIDEC);
  int pt_bus = honda_get_pt_bus();

  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  // sample speed
  if (addr == 0x0C8) { //0x0C8 = ENGINE_DATA
    // first 2 bytes
    vehicle_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
    float speed = ((GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1)) * 0.01 * KPH_TO_MS;
    angle_rate_up = interpolate(BMW_ANGLE_RATE_WINDUP, honda_speed) + BMW_MARGIN;   // deg/1s
    angle_rate_down = interpolate(BMW_ANGLE_RATE_UNWIND, honda_speed) + BMW_MARGIN; // deg/1s
    honda_max_angle = interpolate(BMW_LOOKUP_MAX_ANGLE, honda_speed) + BMW_MARGIN;
    max_tq_rate = interpolate(BMW_MAX_TQ_RATE, honda_speed) + BMW_MARGIN;
  }

  // check ACC main state
  // 0x326 for all Bosch and some Nidec, 0x1A6 for some Nidec
  if (addr == 0xD4) { //0xD4 = CRUISE_CONTROL
    acc_main_on = GET_BIT(to_push, 5U);
    if (!acc_main_on) {
      controls_allowed = false;
    }
  }

  // enter controls when PCM enters cruise state
  if (pcm_cruise && (addr == 0x12C)) { //POWERTRAIN_DATA
    const bool cruise_engaged = GET_BIT(to_push, 51U);
    // engage on rising edge
    if (cruise_engaged && !cruise_engaged_prev) {
      controls_allowed = true;
    }

    if (!cruise_engaged) {
      controls_allowed = false;
    }
    cruise_engaged_prev = cruise_engaged;
  }

  // state machine to enter and exit controls for button enabling
  // 0x1A6 for the ILX, 0x296 for the Civic Touring
  if ((addr == 0xD4) && (bus == pt_bus)) {
    // int button = (GET_BYTE(to_push, 0) & 0xE0U) >> 5;
    int button = (GET_BYTE(to_push, 0) & 0xC0U) >> 6; // top two bits

    // int cruise_setting = (GET_BYTE(to_push, (addr == 0x296) ? 0U : 5U) & 0x0CU) >> 2U;
    // if (cruise_setting == 1) {
    //   mads_button_press = MADS_BUTTON_PRESSED;
    // } else if (cruise_setting == 0) {
    //   mads_button_press = MADS_BUTTON_NOT_PRESSED;
    // } else {
    // }

    // enter controls on the falling edge of set or resume
    bool set = (button != HONDA_BTN_SET) && (cruise_button_prev == HONDA_BTN_SET);
    bool res = (button != HONDA_BTN_RESUME) && (cruise_button_prev == HONDA_BTN_RESUME);
    if (acc_main_on && !pcm_cruise && (set || res)) {
      controls_allowed = true;
    }

    // exit controls once main or cancel are pressed
    if ((button == HONDA_BTN_CANCEL)) {
      controls_allowed = false;
    }
    cruise_button_prev = button;
  }

  // user brake signal on 0x17C reports applied brake from computer brake on accord
  // and crv, which prevents the usual brake safety from working correctly. these
  // cars have a signal on 0x1BE which only detects user's brake being applied so
  // in these cases, this is used instead.
  // most hondas: 0x17C
  // accord, crv: 0x1BE
  // if (honda_alt_brake_msg) {
  //   if (addr == 0x1BE) {
  //     brake_pressed = GET_BIT(to_push, 4U);
  //   }
  // } else {
  if (addr == 0x12C) {
    // // also if brake switch is 1 for two CAN frames, as brake pressed is delayed
    // const bool brake_switch = GET_BIT(to_push, 32U);
    // brake_pressed = (GET_BIT(to_push, 53U)) || (brake_switch && honda_brake_switch_prev);
    // honda_brake_switch_prev = brake_switch;
    brake_pressed = GET_BIT(to_push, 48U);
  }
  // }

  if (addr == 0xAA) { // DRIVER_THROTTLE_POSITION
    gas_pressed = GET_BYTE(to_push, 0) != 0U;
  }

  // // disable stock Honda AEB in alternative experience
  // if (!(alternative_experience & ALT_EXP_DISABLE_STOCK_AEB)) {
  //   if ((bus == 2) && (addr == 0x1FA)) {
  //     bool honda_stock_aeb = GET_BIT(to_push, 29U);
  //     int honda_stock_brake = (GET_BYTE(to_push, 0) << 2) | (GET_BYTE(to_push, 1) >> 6);

  //     // Forward AEB when stock braking is higher than openpilot braking
  //     // only stop forwarding when AEB event is over
  //     if (!honda_stock_aeb) {
  //       honda_fwd_brake = false;
  //     } else if (honda_stock_brake >= honda_brake) {
  //       honda_fwd_brake = true;
  //     } else {
  //       // Leave Honda forward brake as is
  //     }
  //   }
  // }

  if ((addr == 559)  && (bus == 1)) {
    actuator_torque = ((float)(int8_t)(GET_BYTE(to_push, 2))) * CAN_ACTUATOR_TQ_FAC; //Nm

    if((((GET_BYTE(to_push, 1)>>4)>>CAN_ACTUATOR_CONTROL_STATUS_SOFTOFF_BIT) & 0x1) != 0x0){ //Soft off status means motor is shutting down due to error
      controls_allowed = false;
    }
  }
}

static bool honda_tx_hook(const CANPacket_t *to_send) {

  // const LongitudinalLimits HONDA_BOSCH_LONG_LIMITS = {
  //   .max_accel = 200,   // accel is used for brakes
  //   .min_accel = -350,

  //   .max_gas = 2000,
  //   .inactive_gas = -30000,
  // };

  // const LongitudinalLimits HONDA_NIDEC_LONG_LIMITS = {
  //   .max_gas = 198,  // 0xc6
  //   .max_brake = 255,

  //   .inactive_speed = 0,
  // };

  bool tx = true;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  int bus_pt = honda_get_pt_bus();

  // if (!msg_allowed(addr, bus, BMW_TX_MSGS, sizeof(BMW_TX_MSGS) / sizeof(BMW_TX_MSGS[0]))) {
  //   tx = 0;
  // }

  // do not transmit CAN message if steering angle too high
  if (addr == 558) {
    if (((GET_BYTE(to_send, 1) >> 4) & 0b11u) != 0x0){ //control enabled
      float steer_torque = ((float)(int8_t)(GET_BYTE(to_send, 4))) * CAN_ACTUATOR_TQ_FAC; //Nm
      if (honda_fmax_limit_check(steer_torque - actuator_torque, max_tq_rate, -max_tq_rate)){
        // puts("Violation torque rate");
        // printf("Tq: %f, ActTq: %f, Max: %f\n", steer_torque, actuator_torque, max_tq_rate);
        tx = 0;
      }
    }
    float desired_angle = 0;
    if (((GET_BYTE(to_send, 1) >> 4) & 0b11u) == 0x2){ //position control enabled
      float angle_delta_req = ((float)(int16_t)((GET_BYTE(to_send, 2)) | (GET_BYTE(to_send, 3) << 8))) * CAN_ACTUATOR_POS_FAC; //deg/10ms
      desired_angle = honda_rt_angle_last + angle_delta_req; //measured + requested delta

      if (controls_allowed == true) {
        bool violation = false;
        //check for max angles
        violation |= honda_fmax_limit_check(desired_angle, honda_max_angle, -honda_max_angle);
        // puts("Violation desired angle");
        //angle is rate limited in carcontrols so it shouldn't exceed max delta
        float angle_delta_req_side = (honda_desired_angle_last >= 0.) ? angle_delta_req : -angle_delta_req;
        violation |= honda_fmax_limit_check(angle_delta_req_side, angle_rate_up, -angle_rate_down);
        // puts("Violation  delta");

        if (violation) {
          tx = 0;
          desired_angle = honda_desired_angle_last; //nothing was sent - hold to previous
        }
      }
    }
    honda_desired_angle_last = desired_angle;
  }
  if(controls_allowed == false){
    tx = 0;
  }

  return tx;


//   int bus_buttons = (honda_bosch_radarless) ? 2 : bus_pt;  // the camera controls ACC on radarless Bosch cars

//   // ACC_HUD: safety check (nidec w/o pedal)
//   if ((addr == 0x30C) && (bus == bus_pt)) {
//     int pcm_speed = (GET_BYTE(to_send, 0) << 8) | GET_BYTE(to_send, 1);
//     int pcm_gas = GET_BYTE(to_send, 2);

//     bool violation = false;
//     violation |= longitudinal_speed_checks(pcm_speed, HONDA_NIDEC_LONG_LIMITS);
//     violation |= longitudinal_gas_checks(pcm_gas, HONDA_NIDEC_LONG_LIMITS);
//     if (violation) {
//       tx = false;
//     }
//   }

//   // BRAKE: safety check (nidec)
//   if ((addr == 0x1FA) && (bus == bus_pt)) {
//     honda_brake = (GET_BYTE(to_send, 0) << 2) + ((GET_BYTE(to_send, 1) >> 6) & 0x3U);
//     if (longitudinal_brake_checks(honda_brake, HONDA_NIDEC_LONG_LIMITS)) {
//       tx = false;
//     }
//     if (honda_fwd_brake) {
//       tx = false;
//     }
//   }

//   // BRAKE/GAS: safety check (bosch)
//   if ((addr == 0x1DF) && (bus == bus_pt)) {
//     int accel = (GET_BYTE(to_send, 3) << 3) | ((GET_BYTE(to_send, 4) >> 5) & 0x7U);
//     accel = to_signed(accel, 11);

//     int gas = (GET_BYTE(to_send, 0) << 8) | GET_BYTE(to_send, 1);
//     gas = to_signed(gas, 16);

//     bool violation = false;
//     violation |= longitudinal_accel_checks(accel, HONDA_BOSCH_LONG_LIMITS);
//     violation |= longitudinal_gas_checks(gas, HONDA_BOSCH_LONG_LIMITS);
//     if (violation) {
//       tx = false;
//     }
//   }

//   // ACCEL: safety check (radarless)
//   if ((addr == 0x1C8) && (bus == bus_pt)) {
//     int accel = (GET_BYTE(to_send, 0) << 4) | (GET_BYTE(to_send, 1) >> 4);
//     accel = to_signed(accel, 12);

//     bool violation = false;
//     violation |= longitudinal_accel_checks(accel, HONDA_BOSCH_LONG_LIMITS);
//     if (violation) {
//       tx = false;
//     }
//   }

//   // STEER: safety check
//   if ((addr == 0xE4) || (addr == 0x194)) {
//     if (!(controls_allowed || mads_is_lateral_control_allowed_by_mads())) {
//       bool steer_applied = GET_BYTE(to_send, 0) | GET_BYTE(to_send, 1);
//       if (steer_applied) {
//         tx = false;
//       }
//     }
//   }

//   // Bosch supplemental control check
//   if (addr == 0xE5) {
//     if ((GET_BYTES(to_send, 0, 4) != 0x10800004U) || ((GET_BYTES(to_send, 4, 4) & 0x00FFFFFFU) != 0x0U)) {
//       tx = false;
//     }
//   }

//   // FORCE CANCEL: safety check only relevant when spamming the cancel button in Bosch HW
//   // ensuring that only the cancel button press is sent (VAL 2) when controls are off.
//   // This avoids unintended engagements while still allowing resume spam
//   if ((addr == 0x296) && !controls_allowed && (bus == bus_buttons)) {
//     if (((GET_BYTE(to_send, 0) >> 5) & 0x7U) != 2U) {
//       tx = false;
//     }
//   }

//   // Only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
//   if (addr == 0x18DAB0F1) {
//     if ((GET_BYTES(to_send, 0, 4) != 0x00803E02U) || (GET_BYTES(to_send, 4, 4) != 0x0U)) {
//       tx = false;
//     }
//   }

//   return tx;
}


static safety_config honda_odyssey_init(uint16_t param) {
  // UNUSED(param);
  // controls_allowed = false;
  // honda_speed = 0;

  // safety_config ret;
  // return ret;

  static const CanMsg ODYSSEY_TX_MSGS[] = {{0x22E, 1, 8, .check_relay = false}}; //STEERING_COMMAND

  static RxCheck odyssey_rx_checks[] = {
    {.msg = {{0x405, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 3U}, { 0 }, { 0 }}}, //BODY
    {.msg = {{0x6A, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 143U}, { 0 }, { 0 }}}, //BRAKE_PRESSURE
    {.msg = {{0xD4, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}}, //CRUISE_CONTROL
    {.msg = {{0xAA, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}}, //DRIVER_THROTTLE_POSITION
    {.msg = {{0xC8, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}}, //ENGINE_DATA
    {.msg = {{0x188, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}}, //GEARBOX
    {.msg = {{0x1F4, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 25U}, { 0 }, { 0 }}}, //LIGHTS
    {.msg = {{0x12C, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}}, //POWERTRAIN_DATA
    {.msg = {{0x1C0, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 48U}, { 0 }, { 0 }}}, //WHEEL_SPEEDS
    {.msg = {{0x22F, 1, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 100U}, { 0 }, { 0 }}}, //STEERING_STATUS
  };

  UNUSED(param);
  return BUILD_SAFETY_CFG(odyssey_rx_checks, ODYSSEY_TX_MSGS);
}


const safety_hooks honda_odyssey_hooks = {
  .init = honda_odyssey_init,
  .rx = honda_rx_hook,
  .tx = honda_tx_hook,
  // .get_counter = honda_get_counter,
  // .get_checksum = honda_get_checksum,
  // .compute_checksum = honda_compute_checksum,
};
