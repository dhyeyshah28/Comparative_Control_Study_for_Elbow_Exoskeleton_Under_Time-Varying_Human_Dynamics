// CubeMars AK10-9 MIT Mode Control - Portenta H7


#include <mbed.h>
#include "Arduino_PowerManagement.h"

/*********************************************************************************************************
  DEFINITION SECTION
*********************************************************************************************************/

// CAN Bus Setup
mbed::CAN can1(PB_8, PH_13, 1000000);

// LED Pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Motor Configuration
#define MOTOR_ID 2
const unsigned long CAN_ID = MOTOR_ID;

// VALUE LIMITS (for AK10-9)
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -65.0f
#define T_MAX 65.0f

// SET VALUES
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 5.0f;  // HIGHER gain - motor will respond more aggressively
float kd_in = 1.0f;    // MAXIMUM damping for stability
float t_in = 0.0f;

// MEASURED VALUES
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

// Control State
bool motor_enabled = false;
bool going_to_target = true;
uint32_t lastSwitchTime = 0;
uint32_t DWELL_TIME = 2000;  // Switch every 2 seconds

// Timing
uint32_t lastSendTime = 0;
uint32_t lastReceiveTime = 0;

/*********************************************************************************************************
  CONVERSION SECTION
*********************************************************************************************************/

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  
  if (bits == 12) {
    pgg = (unsigned int)((x - offset) * 4095.0 / span);
  }
  if (bits == 16) {
    pgg = (unsigned int)((x - offset) * 65535.0 / span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  
  if (bits == 12) {
    pgg = ((float)x_int) * span / 4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float)x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

/*********************************************************************************************************
  CAN COMMUNICATION SECTION
*********************************************************************************************************/

void unpack_reply(uint8_t* dat, uint8_t len) {
  if (len != 6) return;
  
  /// Unpack ints from CAN buffer ///
  unsigned int id = dat[0];
  unsigned int p_int = (dat[1] << 8) | dat[2];
  unsigned int v_int = (dat[3] << 4) | (dat[4] >> 4);
  unsigned int i_int = ((dat[4] & 0xF) << 8) | dat[5];
  
  /// Convert uints to floats ///
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);
  
  lastReceiveTime = millis();
  
  // Blink blue LED on receive
  digitalWrite(LED_BLUE, LOW);
  delayMicroseconds(100);
  digitalWrite(LED_BLUE, HIGH);
}

void pack_cmd() {
  byte buf[8];

  /// Limit data to be within bounds ///
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);

  /// Convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// Pack ints into the CAN buffer ///
  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;

  // Send data
  mbed::CANMessage msg;
  msg.id = CAN_ID;
  msg.len = 8;
  memcpy(msg.data, buf, 8);
  
  can1.write(msg);
}

/*********************************************************************************************************
  MOTOR MODE SECTION
*********************************************************************************************************/

// ENABLE MOTOR
bool EnterMotorMode() {
  mbed::CANMessage msg;
  msg.id = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF;
  msg.data[1] = 0xFF;
  msg.data[2] = 0xFF;
  msg.data[3] = 0xFF;
  msg.data[4] = 0xFF;
  msg.data[5] = 0xFF;
  msg.data[6] = 0xFF;
  msg.data[7] = 0xFC;
  
  if (!can1.write(msg)) {
    return false;
  }
  
  // sending position commands IMMEDIATELY to prevent timeout
  for (int i = 0; i < 5; i++) {
    pack_cmd();
    delay(10);
    
    // Check for reply
    mbed::CANMessage msgIn;
    if (can1.read(msgIn, 0)) {
      if (msgIn.id == CAN_ID && msgIn.len == 6) {
        unpack_reply(msgIn.data, msgIn.len);
        motor_enabled = true;
        digitalWrite(LED_GREEN, LOW);  // Green = enabled
        delay(2000);
        return true;
      }
    }
  }
  
  return false;
}

// ZEROING MOTOR
void Zero() {
  mbed::CANMessage msg;
  msg.id = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF;
  msg.data[1] = 0xFF;
  msg.data[2] = 0xFF;
  msg.data[3] = 0xFF;
  msg.data[4] = 0xFF;
  msg.data[5] = 0xFF;
  msg.data[6] = 0xFF;
  msg.data[7] = 0xFE;
  
  can1.write(msg);
  
  // CRITICAL: Send position commands immediately after zeroing
  for (int i = 0; i < 5; i++) {
    delay(10);
    pack_cmd();
  }
}

/*********************************************************************************************************
  MAIN FUNCTION SECTION
*********************************************************************************************************/

Board board;

void setup() {

  board.begin();
  board.setExternalVoltage(3.3);

  // Initialize LEDs
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  // Initialize CAN
  can1.mode(mbed::CAN::Mode::Normal);
  delay(3000);

  // Set zero position
  Zero();

  // Set initial position to 0
  p_in = 0.0f;
  // Enter motor mode
  EnterMotorMode();
  delay(1000);
  p_in = 0.1f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.2f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.3f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.4f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.5f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.6f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.7f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.8f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.9f;
  EnterMotorMode();
  delay(1000);
  p_in = 1.0f;
  EnterMotorMode();
  delay(1000);
  

  p_in = 0.9f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.8f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.7f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.6f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.5f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.4f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.3f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.2f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.1f;
  EnterMotorMode();
  delay(1000);
  p_in = 0.0f;
  EnterMotorMode();
  delay(1000);
  

  //p_in = -1.0f;
  
  // Failed - blink all LEDs rapidly
  
  // digitalWrite(LED_RED, LOW);
  // digitalWrite(LED_BLUE, LOW);
  // digitalWrite(LED_GREEN, LOW);
  // delay(2000);
  // digitalWrite(LED_RED, HIGH);
  // digitalWrite(LED_BLUE, HIGH);
  // digitalWrite(LED_GREEN, HIGH);
  //delay(1000);

  lastSwitchTime = millis();
  lastReceiveTime = millis();
  lastSendTime = millis();
  
  // Green LED steady = motor ready
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  delay(5000);
}

void loop() {
  
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
