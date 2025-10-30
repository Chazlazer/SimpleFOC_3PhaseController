#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "EEPROM.h"
#include "StateMachine.h"
#include "hw_configs.h"
#include "user_configs.h"
#include "SimpleCAN.h"

#define VERSION_NUM 1.0f

// Motor instance
BLDCMotor motor = BLDCMotor(11, 0.4, 380);

// Driver
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// Current Sensor
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT);

// Encoder
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// User Interface State Machine
StateMachine state_machine(SystemState::MENU_MODE);
MotorConfig cfg;


// CANBUS
// pass in optional shutdown and terminator pins that disable transceiver and add 120ohm resistor respectively
SimpleCan can1(A_CAN_SHDN,A_CAN_TERM);
static void init_CAN();
static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);
float commands[5];
volatile bool can_special_msg1 = false;
volatile bool can_special_msg2 = false;
volatile bool can_new_msg = false;
uint8_t last_can_data[8];


bool align_motor_flag = false;

int flash_marker = 25;
bool store_inital_value = true;
bool led_state = false;   // tracks LED state
unsigned long lastPrintTime = 0;
float target_angle = 0.00;

float aref = 3.3; // Analog reference voltage of your STM32
float scalingFactor = 10.389; // Calculated from R1=169k and R2=18k
int bitResolution = 4095; // 12-bit resolution (2^12 - 1)

void setup_simplefoc();
void encoder_mode();
void calibration_mode();
void setup_mode();
void motor_mode();
void read_serial();
bool isConfigEmpty(const MotorConfig &c);
void setDefaultMotorConfig(MotorConfig &c);
void read_vbus();

// Commands
Commander command = Commander(Serial);
void writeGR(char* cmd){Serial.println(atoi(cmd));};

void setup() {
    // Setup Pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(A_VBUS, INPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(7000);

    // Initalize CORDIC
    // #ifdef DHAL_CORDIC_MODULE_ENABLED
    SimpleFOC_CORDIC_Config();
    // #endif

    // Setup CAN

    // use monitoring with serial 
    Serial.begin(115200);
    // enable more verbose output for debugging
    // comment out if not needed
    SimpleFOCDebug::enable(&Serial);

    // Read Config Data from Flash
    cfg = EEPROM.get(EEPROM_ADDR_DIRECTION, cfg);  // Reads entire struct

    // Check validity and apply defaults if necessary
    if(isConfigEmpty(cfg)) {
      Serial.println("Flash empty or invalid, loading defaults...");
      setDefaultMotorConfig(cfg);
      EEPROM.put(EEPROM_ADDR_DIRECTION, cfg);
    } else {
      Serial.println();
      Serial.println("Loaded motor config from flash");
    }

    // Initalize State Machine Config with Flash Config
    state_machine.setMotorConfig(&cfg);

    // Setup SimpleFOC
    setup_simplefoc();
    // ***************************

    // ****************************

    // Load in motor Direction to Skip Calibration
    if(cfg.sensor_direction != 4){
      motor.sensor_direction = cfg.sensor_direction;
      Serial.print("Loaded motor direction: ");

      if (motor.sensor_direction == Direction::CW) {
        Serial.println("CW");
        motor.init();
        motor.initFOC();
      }
      else if (motor.sensor_direction ==Direction::CCW){
        motor.init();
        motor.initFOC();
        Serial.println("CCW");
      }
      else if (motor.sensor_direction == Direction::UNKNOWN){
        Serial.println("UNKNOWN");
        Serial.println("Motor Not Calibrated");
        Serial.println("Run Calibration Before Use");
      }
      else{
        Serial.println(motor.sensor_direction);
      }
      store_inital_value = false;
    }
    
    init_CAN();
    Serial.print("Version: ");
    Serial.println(VERSION_NUM);
    state_machine.print_menu();
}

void loop() {
    if (Serial.available()) {
        read_serial();
    }

    if (can_special_msg1 && (state_machine.getState() != SystemState::MOTOR_MODE)) {
        can_special_msg1 = false;
        state_machine.setState(SystemState::MOTOR_MODE);  
        // Serial.println("Special message 1 received!");
        // e.g. stop motor
    }

    if (can_special_msg2 && (state_machine.getState() != SystemState::MENU_MODE)) {
        can_special_msg2 = false;
        state_machine.setState(SystemState::MENU_MODE);  
        // Serial.println("Special message 2 received!");
        // e.g. recalibrate or change mode
    }
    if (can_new_msg) {
        noInterrupts();
        uint8_t data_copy[8];
        memcpy(data_copy, last_can_data, 8);
        can_new_msg = false;
        interrupts();

        // Now unpack safely outside ISR
        int p_int  = (data_copy[0] << 8) | data_copy[1];
        int v_int  = (data_copy[2] << 4) | (data_copy[3] >> 4);
        int kp_int = ((data_copy[3] & 0xF) << 8) | data_copy[4];
        int kd_int = (data_copy[5] << 4) | (data_copy[6] >> 4);
        int t_int  = ((data_copy[6] & 0xF) << 8) | data_copy[7];

        commands[0] = uint_to_float(p_int,  -cfg.P_MAX,  cfg.P_MAX, 16);
        commands[1] = uint_to_float(v_int,  -cfg.V_MAX,  cfg.V_MAX, 12);
        commands[2] = uint_to_float(kp_int, -cfg.KP_MAX, cfg.KP_MAX, 12);
        commands[3] = uint_to_float(kd_int, -cfg.KD_MAX, cfg.KD_MAX, 12);
        commands[4] = uint_to_float(t_int,  -cfg.I_MAX*cfg.KT*cfg.GR, cfg.I_MAX*cfg.KT*cfg.GR, 12);

        // Serial.print("Target Set to: ");
        // Serial.println(commands[0]);
    }
    // --- State-specific behavior ---
    switch (state_machine.currentState) {
      case SystemState::MENU_MODE:
        if (led_state) {
          digitalWrite(LED_BUILTIN,LOW);
          led_state = false;
        }
        break;

      case SystemState::MOTOR_MODE:
        if(motor.sensor_direction == Direction::UNKNOWN){
          Serial.println();
          Serial.println("Motor Not Calibrated");
          Serial.println("Run Calibration First");
          Serial.println();
          break;
        }
        if (!led_state) {
          digitalWrite(LED_BUILTIN,HIGH);
          led_state = true;
        }

        motor_mode();
        break;
      
      case SystemState::SETUP_MODE: {
        setup_mode();
        state_machine.setState(SystemState::MENU_MODE);
        break;
      }
      case SystemState::CALIBRATION_MODE:
        calibration_mode();
        state_machine.setState(SystemState::MENU_MODE);
        state_machine.print_menu();
        break;
      case SystemState::ENCODER_MODE:
        encoder_mode();
        break;
      default:
        led_state = false;
        break;
    }
}

void setup_simplefoc(){

    // configure i2C
    Wire.setClock(1000000);
    // initialise magnetic sensor hardware
    sensor.init();

    // link the motor to the sensor
    motor.linkSensor(&sensor);  
    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    // float vbus = analogRead(A_VBUS);
    driver.pwm_frequency = 45000;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);

    // link current sense and the driver
    currentSense.linkDriver(&driver);
    // current sensing
    currentSense.init();
    // no need for aligning
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    read_vbus();

    // Calculate Calibration Voltage
    float v_cal = cfg.I_CAL*motor.phase_resistance;
    Serial.println();Serial.print("VCAL: ");Serial.println(v_cal);

    // aligning voltage [V]
    motor.voltage_sensor_align = 3;
    // index search velocity [rad/s]
    motor.velocity_index_search = 3;

    // set motion control loop to be used
    motor.controller = cfg.MOTOR_MODE;

    // contoller configuration 
    // default parameters in defaults.h

    // velocity PI controller parameters
    motor.PID_velocity.P = 0.15;
    motor.PID_velocity.I = 15;
    // default voltage_power_supply
    motor.voltage_limit = 6;
    // jerk control using voltage voltage ramp
    // default value is 300 volts per sec  ~ 0.3V per millisecond
    motor.PID_velocity.output_ramp = 1000;

    // velocity low pass filtering time constant
    motor.LPF_velocity.Tf = 0.01;

    // angle P controller
    motor.P_angle.P = 20;
    //  maximal velocity of the position control
    motor.velocity_limit = 20;

   // comment out if not needed
    motor.useMonitoring(Serial);
    motor.init();   
}

void read_serial(){
    char key = Serial.read();
    switch (key) {
      case MENU_CMD:
          state_machine.setState(SystemState::MENU_MODE);    
          state_machine.print_menu();
          break;
      case MOTOR_CMD:
          state_machine.setState(SystemState::MOTOR_MODE);
          break;
      case SETUP_CMD:
          state_machine.setState(SystemState::SETUP_MODE);
          state_machine.print_setup();
          break;
      case CAL_CMD:
          state_machine.setState(SystemState::CALIBRATION_MODE);
          break;
      case ENCODER_CMD:
          state_machine.setState(SystemState::ENCODER_MODE);
          break;
      case ZERO_CMD:
          motor.sensor_offset = sensor.getMechanicalAngle();
          Serial.print("\n\r  Saved new Mechanical Angle zero position:");
          Serial.print(motor.sensor_offset);
          Serial.println();
          cfg.MZERO = motor.sensor_offset;
          state_machine.print_menu();
          break;
      default: 
          break;
    }
}

void encoder_mode(){
    sensor.update();
    unsigned long now = micros();
    if (now - lastPrintTime >= 10000) {
        lastPrintTime += 10000;
        Serial.print("Output Angle: ");
        Serial.print(sensor.getAngle()/cfg.GR);
        Serial.print(" Mechanical Angle: ");
        Serial.print(sensor.getMechanicalAngle());
        Serial.print(" Electrical Angle: ");
        Serial.print(sensor.getAngle());
        Serial.print(" Velocity: ");
        Serial.println(sensor.getVelocity());
    }
}

void calibration_mode()
{
  // Indicate Motor Is On
  digitalWrite(MOTOR_LED, HIGH);

  // Reset Values Before Init
  motor.sensor_direction = Direction::UNKNOWN;
  motor.zero_electric_angle = NOT_SET;
  motor.init();
  motor.initFOC();

  // Save new direction to EEPROM
  Serial.print("Motor Direction is: ");
  Serial.println(motor.sensor_direction);
  cfg.sensor_direction = motor.sensor_direction;
  EEPROM.put(EEPROM_ADDR_DIRECTION, cfg);
  Serial.print("Motor direction determined — saved ");
  Serial.print(cfg.sensor_direction);
  Serial.println(" to EEPROM");
  Serial.println();

  // Indicate Motor Is Off
  digitalWrite(MOTOR_LED, LOW);
}

void setup_mode(){
    MotorConfig inital_motor_cfg = cfg;
    state_machine.m_in_setup = true;
    String input = "";
    Serial.println("Entering SETUP MODE. Type something and press Enter:");
    Serial.println("(Press ESC to exit)");

    while (state_machine.m_in_setup) {
      if (Serial.available()) {
        if(Serial.peek() == MENU_CMD){
          state_machine.m_in_setup = false;
        }
        else{
        char c = Serial.read();

        // Handle newline (Enter key)
        if (c == '\n' || c == '\r') {
          if (input.length() > 0) {
            state_machine.processCommand(input);                
            input = "";  // reset for next line
            EEPROM.put(EEPROM_ADDR_DIRECTION, cfg);
            state_machine.print_setup();
          }
        } else {
          input += c; // append character
        }
      }
    }
  }
}

void motor_mode(){
  if(cfg.MOTOR_MODE == MotionControlType::velocity){
  }
  else if(cfg.MOTOR_MODE == MotionControlType::angle){
    target_angle = commands[0];
    motor.loopFOC();
    motor.move(target_angle);
    // Serial.print("Target Set to: ");
    // Serial.println(target_position);

  }
}

void setDefaultMotorConfig(MotorConfig &c) {
  c.GR = 1.0f;
  c.KT = 0.08f;
  c.I_BW = 1000.0f;
  c.I_MAX = 10.0f;
  c.P_MAX = 12.5f;
  c.V_MAX = 30.0f;
  c.KP_MAX = 30.0f;
  c.KD_MAX = 0.5f;
  c.I_FW_MAX = 5.0f;
  c.I_MAX_CONT = 5.0f;
  c.I_CAL = 1.0f;
  c.R_PHASE = 0.5f;
  c.TEMP_MAX = 80.0f;
  c.PPAIRS = 7;
  c.THETA_MIN = -12.5;
  c.THETA_MAX = 12.5;
  c.CAN_ID = 0;
  c.CAN_MASTER = 0;
  c.CAN_TIMEOUT = 100;
  c.MZERO = 0;
  c.MOTOR_MODE = MotionControlType::angle;
  c.sensor_direction = Direction::UNKNOWN;
}

bool isConfigEmpty(const MotorConfig &c) {
  // Check for invalid or erased flash (all 0xFF or all 0.0)
  return (
    isnan(c.GR) || c.GR == 0.0f ||
    c.KT == 0.0f ||
    c.PPAIRS <= 0 ||
    c.sensor_direction > 4
  );
}

void read_vbus(){
    analogReadResolution(12);
    float vbus = analogRead(A_VBUS);
    // float vbus = _readADCVoltageLowSide(A_VBUS,currentSense.params);
    float measuredVoltage = vbus * (aref / bitResolution);
    // Scale the voltage to get the original source voltage
    float sourceVoltage = measuredVoltage * scalingFactor;
    // Print the results to the Serial Monitor
    Serial.print("ADC Raw Value: ");
    Serial.println(vbus);

    Serial.print("Measured Voltage at Pin: ");
    Serial.print(measuredVoltage);
    Serial.println(" V");

    Serial.print("12V Source Voltage: ");
    Serial.print(sourceVoltage);
    Serial.println(" V");

    Serial.println("-------------------------");
}


static void init_CAN()
{
	Serial.println(can1.init(CanSpeed::Mbit1) == HAL_OK
					   ? "CAN: initialized."
					   : "CAN: error when initializing.");

	FDCAN_FilterTypeDef sFilterConfig;

	// Configure Rx filter
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = cfg.CAN_ID;
	sFilterConfig.FilterID2 = 0x7FF;

	can1.configFilter(&sFilterConfig);
	can1.configGlobalFilter(FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	can1.activateNotification(&can1RxHandler);

	Serial.println(can1.start() == HAL_OK
					   ? "CAN: started."
					   : "CAN: error when starting.");
}

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
    // Copy message quickly
    for (int i = 0; i < 8; i++) {
        last_can_data[i] = rxData[i];
    }

    // Convert to 64-bit word for fast compare
    uint64_t msg64;
    memcpy(&msg64, rxData, 8);

    // Compare both patterns
    if (msg64 == 0xFCFFFFFFFFFFFFFFULL) {
        can_special_msg1 = true;
    }
    else if (msg64 == 0xFDFFFFFFFFFFFFFFULL) {
        can_special_msg2 = true;
    }
    else {
        can_new_msg = true;
    }
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}