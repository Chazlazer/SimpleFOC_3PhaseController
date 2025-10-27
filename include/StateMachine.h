#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "user_configs.h"
#include "SimpleFOC.h"

#define MENU_CMD			27
#define MOTOR_CMD			'm'
#define CAL_CMD				'c'
#define ENCODER_CMD			'e'
#define SETUP_CMD			's'
#define ZERO_CMD			'z'
#define ENTER_CMD			13

enum SystemState {
  MENU_MODE,  
  CALIBRATION_MODE,
  MOTOR_MODE,
  SETUP_MODE,
  ENCODER_MODE,
  INIT_TEMP_MODE
};  

// Mode Selection Class
class StateMachine 
{
public:
    StateMachine(SystemState initial);

    void setState(SystemState next);
    void onEnter(SystemState state);
    void onExit(SystemState state);

    SystemState getState() const { return currentState; }

    SystemState currentState;
    void print_menu();
    void print_setup();
    bool m_print_menu = true;
    bool m_in_setup = true;
    void processCommand(String cmd);
    void setMotorConfig(MotorConfig* cfgPtr) { motor_cfg_ptr = cfgPtr; }
private:
    SystemState nextState;
    bool stateChange;

    MotorConfig* motor_cfg_ptr = nullptr;  // pointer to external config

};
#endif //STATE_MACHINE_H