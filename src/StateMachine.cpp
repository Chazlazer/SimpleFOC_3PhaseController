#include "Arduino.h"
#include "StateMachine.h"

StateMachine::StateMachine(SystemState initial) {
  currentState = initial;
  nextState = initial;
  stateChange = true;
}

void StateMachine::setState(SystemState next) {
  if (next != currentState) {
    currentState = next;
  }
}

void StateMachine::onEnter(SystemState state) {
  switch (state) {
    case MENU_MODE:
      Serial.println("Entering MENU_MODE");
      print_menu();  // print once on entry
      break;

    case CALIBRATION_MODE:
      break;

    case MOTOR_MODE:
      break;

    case SETUP_MODE:
      print_setup(); // optionally print setup info here once
      break;

    case ENCODER_MODE:
      break;

    case INIT_TEMP_MODE:
      break;

    default:
      break;
  }
}

void StateMachine::print_menu(){
    Serial.printf("\n\r\n\r");
    Serial.printf(" Commands:\n\r");
    Serial.printf(" m - Motor Mode\n\r");
    Serial.printf(" c - Calibrate Encoder\n\r");
    Serial.printf(" s - Setup\n\r");
    Serial.printf(" e - Display Encoder\n\r");
    Serial.printf(" z - Set Zero Position\n\r");
    Serial.println(" esc - Exit to Menu");
}

void StateMachine::print_setup(){
    if (!motor_cfg_ptr) {
        Serial.println("Error: motor config not set!");
        return;
    }

    MotorConfig& cfg = *motor_cfg_ptr;  // alias
    Serial.println();
    Serial.println(" Configuration Options");
    Serial.printf(" %-4s %-29s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");
    Serial.println();
    Serial.println(" Motor:");
    Serial.printf(" %-4s %-31s %-5s %-6s ", "g", "Gear Ratio", "0", "-" );                        Serial.println(cfg.GR);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "k", "Torque Constant (N-m/A)", "0", "-");            Serial.println(cfg.KT);
    Serial.printf("\r\n Control:\r\n");
    Serial.printf(" %-4s %-31s %-5s %-6s ", "b", "Current Bandwidth (Hz)", "100", "2000");        Serial.println(cfg.I_BW);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "l", "Current Limit (A)", "0.0", "12.0");             Serial.println(cfg.I_MAX);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "p", "Max Position Setpoint (rad)", "-", "-");        Serial.println(cfg.P_MAX);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "v", "Max Velocity Setpoint (rad)/s", "-", "-");      Serial.println(cfg.V_MAX);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "x", "Max Position Gain (N-m/rad)", "0.0", "1000.0"); Serial.println(cfg.KP_MAX);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "d", "Max Velocity Gain (N-m/rad/s)", "0.0", "5.0");  Serial.println(cfg.KD_MAX);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "f", "FW Current Limit (A)", "0.0", "33.0");          Serial.println(cfg.I_FW_MAX);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "h", "Temp Cutoff (C) (0 = none)", "0", "150");       Serial.println(cfg.TEMP_MAX);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "c", "Continuous Current (A)", "0.0", "40.0");        Serial.println(cfg.I_MAX_CONT);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "a", "Calibration Current (A)", "0.0", "2.0");        Serial.println(cfg.I_CAL);    
    Serial.printf("\r\n CAN:\r\n");
    Serial.printf(" %-4s %-31s %-5s %-6s ", "i", "CAN ID", "0", "127");                         Serial.println(cfg.CAN_ID);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "m", "CAN TX ID", "0", "127");                      Serial.println(cfg.CAN_MASTER);
    Serial.printf(" %-4s %-31s %-5s %-6s ", "t", "CAN Timeout (cycles)(0 = none)", "0", "1000");Serial.println( cfg.CAN_TIMEOUT);
    Serial.printf("\r\n FOC:\r\n");
    Serial.printf(" %-4s %-31s %-5s %-6s ", "r", "Loop Type ", "-", "-");
    if(cfg.MOTOR_MODE == MotionControlType::angle){
        Serial.println("Angle");
    }
    else if (cfg.MOTOR_MODE == MotionControlType::velocity)
    {
        Serial.println("Velocity");
    }
    else{
        Serial.println("UNKNOWN");
    }
    Serial.println(" \n\r To change a value, type 'prefix''value''ENTER'"); 
    Serial.println("e.g. 'b1000''ENTER'");
    // Serial.printf("VALUES NOT ACTIVE UNTIL POWER CYCLE! \n\r\n\r");    
}

void StateMachine::processCommand(String cmd) {
  char prefix = cmd[0];
  float value = cmd.substring(1).toFloat();
//   if(prefix == 'i' || prefix == 'm' || prefix == 't'){
//     int value = (int)value;
//   }
  MotorConfig& cfg = *motor_cfg_ptr;  // alias
  switch (prefix) {
    case 'g': cfg.GR = value; break;
    case 'k': cfg.KT = value; break;
    case 'b': cfg.I_BW = value; break;
    case 'l': cfg.I_MAX = value; break;
    case 'p': cfg.P_MAX = value; break;
    case 'v': cfg.V_MAX = value; break;
    case 'x': cfg.KP_MAX = value; break;
    case 'd': cfg.KD_MAX = value; break;
    case 'f': cfg.I_FW_MAX = value; break;
    case 'h': cfg.TEMP_MAX = value; break;
    case 'c': cfg.I_MAX_CONT = value; break;
    case 'a': cfg.I_CAL = value; break;
    case 'i': cfg.CAN_ID = (int)value; break;
    case 'm': cfg.CAN_MASTER = (int)value; break;
    case 't': cfg.CAN_TIMEOUT = (int)value; break;
    case 'r': // switch loop type
      if (value == 0) cfg.MOTOR_MODE = MotionControlType::angle;
      else if (value == 1) cfg.MOTOR_MODE = MotionControlType::velocity;
      break;
    default:
      Serial.println("Invalid prefix or command");
      return;
  }
}