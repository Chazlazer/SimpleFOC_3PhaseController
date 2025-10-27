#ifndef USER_COMMANDS_H
#define USER_COMMANDS_H

#include "Arduino.h"
#include "SimpleFOC.h"

// instantiate the commander
Commander command = Commander(Serial);
// class ProcessCommand:
// {
//     public:
//         void doTarget(char* cmd) { command.motion(&motor, cmd); }
//         void writeGR(char* cmd){Serial.println(cmd)};


// };

struct MotorConfig {
  float GR;          // Gear Ratio
  float KT;          // Torque Constant
  float I_BW;        // Current Bandwidth
  float I_MAX;       // Current Limit
  float P_MAX;       // Max Position Setpoint
  float V_MAX;       // Max Velocity Setpoint
  float KP_MAX;      // Max Position Gain
  float KD_MAX;      // Max Velocity Gain
  float I_FW_MAX;    // FW Current Limit
  float I_MAX_CONT;  // Continuous Current
  float I_CAL;       // Calibration Current
  float R_PHASE;     // Phase to Phase Resistance
  float TEMP_MAX;    // Temperature Safety Limit
  float PPAIRS;      // Number of motor Pole - Pairs
  float THETA_MIN;   // Minimum position Setpoint
  float THETA_MAX;   // Maxiumum position Setpoint
  int CAN_ID;        // CAN ID
  int CAN_MASTER;    // CAN TX ID
  int CAN_TIMEOUT;   // CAN Timeout
  int MZERO;         // Zero Offset
  MotionControlType MOTOR_MODE; // 
  Direction sensor_direction; // Optional: motor sensor direction (0=UNKNOWN,1=CCW,2=CW)
};

#endif // COMMANDER_H