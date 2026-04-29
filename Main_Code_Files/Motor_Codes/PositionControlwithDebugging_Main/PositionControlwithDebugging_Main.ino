// CubeMars AK45-36 MIT Mode POSITION CONTROL with Portenta H7 on CAN1

// Libraries Required
#include <mbed.h>
#include "Arduino_PowerManagement.h"

// CAN Bus Setup (pins and rate)
mbed::CAN can1(PB_8, PH_13, 1000000);

// UART0 Debug Setup on Breakout UART0 pins
// UART0_TX = PA0, UART0_RX = PI9 from Portenta Breakout schematic
mbed::UnbufferedSerial debug_uart(PA_0, PI_9, 115200);

// LED Pins
#define LED_RED   LEDR
#define LED_GREEN LEDG
#define LED_BLUE  LEDB

// Motor Configuration (set from cubemars app)
#define MOTOR_ID 2
const unsigned long CAN_ID = MOTOR_ID;

// Value Limits (from datasheet for AK series)
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

// Set Value Params
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 20.0f;  // HIGH stiffness for position control
float kd_in = 3.0f;    // HIGH damping to prevent oscillation
float t_in = 0.0f;

// Measured Value Params
float p_out = 0.0f;
float v_out = 0.0f;
float t_out = 0.0f;

// Timing
uint32_t lastReceiveTime = 0;

// UART0 Debug Timing
uint32_t lastDebugTime = 0;

/*********************************************************************************************************
  UART0 DEBUG SECTION
*********************************************************************************************************/

void debugPrint(const char* msg) {
  debug_uart.write(msg, strlen(msg));
}

void debugPrintln(const char* msg) {
  debug_uart.write(msg, strlen(msg));
  debug_uart.write("\r\n", 2);
}

void debugMotorFeedback(uint32_t rx_id, uint8_t* dat, uint8_t len) {
  char line[256];

  snprintf(line, sizeof(line),
           "[MOTOR FEEDBACK] CAN_ID=%lu RX_ID=%lu LEN=%u | "
           "Motor_ID_byte=%u | "
           "p_out=%.4f rad | v_out=%.4f rad/s | t_out=%.4f Nm | "
           "p_cmd=%.4f rad | v_cmd=%.4f rad/s | kp=%.2f | kd=%.2f | "
           "raw=[%02X %02X %02X %02X %02X %02X]",
           CAN_ID,
           rx_id,
           len,
           dat[0],
           p_out,
           v_out,
           t_out,
           p_in,
           v_in,
           kp_in,
           kd_in,
           dat[0], dat[1], dat[2], dat[3], dat[4], dat[5]);

  debugPrintln(line);
}

void debugStatusNoFeedback() {
  char line[180];

  snprintf(line, sizeof(line),
           "[NO RECENT FEEDBACK] last_rx_age=%lu ms | "
           "p_cmd=%.4f rad | v_cmd=%.4f rad/s | p_out=%.4f rad | v_out=%.4f rad/s | t_out=%.4f Nm",
           millis() - lastReceiveTime,
           p_in,
           v_in,
           p_out,
           v_out,
           t_out);

  debugPrintln(line);
}

/*********************************************************************************************************
  CONVERSION FUNCTIONS
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
  if (len < 6) return;

  unsigned int p_int = (dat[1] << 8) | dat[2];
  unsigned int v_int = (dat[3] << 4) | (dat[4] >> 4);
  unsigned int i_int = ((dat[4] & 0xF) << 8) | dat[5];

  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);

  lastReceiveTime = millis();

  // Optional: Visual feedback on CAN RX
  digitalWrite(LED_BLUE, LOW);
  delayMicroseconds(100);
  digitalWrite(LED_BLUE, HIGH);
}

void pack_cmd() {
  byte buf[8];

  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp    = constrain(kp_in, KP_MIN, KP_MAX);
  float kd    = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff  = constrain(t_in, T_MIN, T_MAX);

  unsigned int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  buf[0] = p_int >> 8;
  buf[1] = p_int & 0xFF;
  buf[2] = v_int >> 4;
  buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  buf[4] = kp_int & 0xFF;
  buf[5] = kd_int >> 4;
  buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  buf[7] = t_int & 0xFF;

  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  memcpy(msg.data, buf, 8);
  can1.write(msg);
}

/*********************************************************************************************************
  MOTOR MODE SECTION 
*********************************************************************************************************/

bool EnterMotorMode() {
  mbed::CANMessage msg;
  msg.id  = CAN_ID;
  msg.len = 8;
  msg.data[0] = 0xFF;
  msg.data[1] = 0xFF;
  msg.data[2] = 0xFF;
  msg.data[3] = 0xFF;
  msg.data[4] = 0xFF;
  msg.data[5] = 0xFF;
  msg.data[6] = 0xFF;
  msg.data[7] = 0xFC;

  if (!can1.write(msg)) return false;

  delay(50);
  return true;
}

void Zero() {
  mbed::CANMessage msg;
  msg.id  = CAN_ID;
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
  delay(100);
}

/*********************************************************************************************************
  MAIN FUNCTION SECTION
*********************************************************************************************************/

Board board;

void setup() {

  
  // UART0 Debug Start
  delay(500);
  debug_uart.format(8, mbed::SerialBase::None, 1);

  debugPrintln("========================================");
  debugPrintln("Portenta H7 UART0 Debug Started");
  debugPrintln("UART0 TX = PA0, UART0 RX = PI9");
  debugPrintln("Baud = 115200");
  debugPrintln("========================================");

  // For 3.3V pin outputs
  board.begin();
  board.setExternalVoltage(3.3);

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE,  OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE,  HIGH);

  can1.mode(mbed::CAN::Mode::Normal);
  delay(3000);

  debugPrintln("[SETUP] CAN1 normal mode set at 1 Mbps");

  // Zero, then enable ONCE
  p_in = 0.0f;
  debugPrintln("[SETUP] Sending zero command...");
  Zero();
  delay(1000);

  debugPrintln("[SETUP] Entering motor mode...");
  if (EnterMotorMode()) {
    debugPrintln("[SETUP] EnterMotorMode CAN command sent successfully");
  } else {
    debugPrintln("[SETUP ERROR] EnterMotorMode CAN write failed");
  }
  delay(1000);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE,  LOW);
  digitalWrite(LED_RED,   LOW);

  debugPrintln("[SETUP] Setup complete. Starting 1kHz position control.");
}

// ############################
// MAIN LOOP - 1kHz Position Control
// ############################

void loop() {

  // 1. Unpack incoming CAN messages without blocking (Continuous)
  mbed::CANMessage msgIn;
  if (can1.read(msgIn, 0)) {

    char rawLine[220];

    snprintf(rawLine, sizeof(rawLine),
            "[RAW CAN RX] id=%lu len=%u data=[%02X %02X %02X %02X %02X %02X %02X %02X]",
            msgIn.id,
            msgIn.len,
            msgIn.data[0], msgIn.data[1], msgIn.data[2], msgIn.data[3],
            msgIn.data[4], msgIn.data[5], msgIn.data[6], msgIn.data[7]);

    //debugPrintln(rawLine);

    if (msgIn.id == CAN_ID && (msgIn.len == 6 || msgIn.len == 8)) {
      unpack_reply(msgIn.data, msgIn.len);

      if (millis() - lastDebugTime >= 100) {
        lastDebugTime = millis();
        debugMotorFeedback(msgIn.id, msgIn.data, msgIn.len);
      }
    }
  }

  // Print warning if no feedback received recently
  if ((millis() - lastDebugTime >= 500) && (millis() - lastReceiveTime > 500)) {
    lastDebugTime = millis();
    debugStatusNoFeedback();
  }
  
  // 2. 1 kHz (1000 microsecond) Timer Gate
  static uint32_t lastTime = micros();
  const uint32_t period_us = 1000;   

  uint32_t now = micros();
  if (now - lastTime < period_us) return;
  lastTime += period_us;

  // 3. Feedback-driven trajectory generation
  static float current_pos = 0.0f;
  static float sweep_speed = 0.25f;   // rad/s
  static int sweep_dir = 1;

  const float P_LOW  = 0.0f;
  const float P_HIGH = 1.570f;

  // max allowed tracking error before pausing trajectory advance
  const float TRACK_ERR_LIMIT = 0.035f;   // rad, about 2 deg

  float tracking_err = fabsf(current_pos - p_out);


  // Only advance command if motor is actually keeping up
  if (lastReceiveTime > 0 && tracking_err < TRACK_ERR_LIMIT) {
    current_pos += sweep_dir * sweep_speed * 0.001f;
  }

  // Reverse based on ACTUAL feedback, not just command
  if (sweep_dir == 1 && p_out >= P_HIGH - TRACK_ERR_LIMIT) {
    current_pos = P_HIGH;
    sweep_dir = -1;
  }
  else if (sweep_dir == -1 && p_out <= P_LOW + TRACK_ERR_LIMIT) {
    current_pos = P_LOW;
    sweep_dir = 1;
  }

  // Safety clamp
  current_pos = constrain(current_pos, P_LOW, P_HIGH);

  // 4. Update MIT parameters for POSITION CONTROL
  p_in = current_pos;
  
  // CRITICAL: Velocity feedforward for smooth tracking
  v_in = sweep_dir * sweep_speed;
  
  // High gains for stiff position control
  kp_in = 14.0f;  // 20x stiffer than impedance mode
  kd_in = 1.5f;    // Damping scaled proportionally
  t_in = 0.1f;     // No feedforward torque needed

  // 5. Send command to motor
  pack_cmd();
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/