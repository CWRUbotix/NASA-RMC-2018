#include <Sabertooth.h>
#include <RoboClaw.h>
#include <math.h>

#define COMMAND_READ_SENSORS (0x01)
#define COMMAND_SET_OUTPUTS (0x02)
#define COMMAND_HCI_TEST    (0x5A)

#define RESPONSE_HCI_TEST   (0xA5)

#define ADDRESS_RC_0 (0x80)
#define ADDRESS_RC_1 (0x81)
#define ADDRESS_RC_2 (0x82)
#define ADDRESS_RC_3 (0x83)

#define CMD_HEADER_SIZE (2)
#define RPY_HEADER_SIZE (2)

#define FAULT_T uint16_t
#define NO_FAULT (1)
#define FAULT_FAILED_WRITE (2)
#define FAULT_INCOMPLETE_HEADER (3)
#define FAULT_CORRUPTED_HEADER (4)
#define FAULT_INCOMPLETE_BODY (5)
#define FAULT_CORRUPTED_BODY (6)
#define FAULT_LOST_ROBOCLAW (6)

enum SensorHardware {
  SH_NONE,
  SH_RC_POT,
  SH_RC_ENC,
  SH_RC_CUR,
  SH_PIN_LIMIT,
  SH_PIN_POT
};

typedef struct SensorInfo {
  SensorHardware hardware;
  uint8_t addr; // When hardware = SH_RC_*
  uint8_t whichMotor; // When hardware = SH_RC_*
  uint8_t whichPin; // When hardware = SH_PIN_*
  float responsiveness;
  uint16_t scale; // 1 unless needed
} SensorInfo;

enum MotorHardware {
  MH_NONE,
  MH_RC_PWM,
  MH_RC_VEL,
  MH_RC_POS,
  MH_RC_POS_BOTH,
  MH_ST_PWM,
  MH_ST_POS,
  MH_ST_PWM_BOTH,
  MH_PIN_PWM,
  MH_ALL
};

typedef struct MotorInfo {
  MotorHardware hardware;
  uint8_t addr;
  uint8_t whichMotor;
  uint16_t scale; // 1 unless needed
  float kp; // When hardware = MH_RC_POS or MC_RC_VEL
  float ki; // When hardware = MH_RC_POS or MC_RC_VEL
  float kd; // When hardware = MH_RC_POS or MC_RC_VEL
  uint32_t qpps; // When hardware = MH_RC_POS or MC_RC_VEL
  uint32_t deadband; // When hardware = MH_RC_POS
  uint32_t minpos; // When hardware = MH_RC_POS
  uint32_t maxpos; // When hardware = MH_RC_POS
  uint32_t accel;
  uint16_t feedbackSensorID;
  float saturation;
  
} MotorInfo;

SensorInfo sensor_infos[256] = {}; // All initialized to SH_NONE
MotorInfo motor_infos[256] = {}; // All initialized to MH_NONE
int16_t motor_setpoints[256] = {0,0,0,0,1000,1000,1000,1000}; // All others initialized to 0
uint8_t sensor_lastLimitVals[256] = {}; // All initialized to 0
int16_t sensor_storedVals[256] = {}; // All initialized to 0
float motor_integrals[256] = {}; //All initialized to 0
int16_t motor_lastUpdateTime[256] = {}; //All initialized to 0
bool stopped = true;

RoboClaw roboclaw(&Serial1,10000);
Sabertooth sabretooth[4] = {
  Sabertooth(0x80, Serial2),
  Sabertooth(0x81, Serial2),
  Sabertooth(0x82, Serial2),
  Sabertooth(0x83, Serial2),
};

void setup() {
  // Front left wheel encoder
  sensor_infos[1].hardware = SH_RC_ENC;
  sensor_infos[1].addr = ADDRESS_RC_0;
  sensor_infos[1].whichMotor = 1;
  sensor_infos[1].scale = 100;
  
  // Front right wheel encoder
  sensor_infos[0].hardware = SH_RC_ENC;
  sensor_infos[0].addr = ADDRESS_RC_0;
  sensor_infos[0].whichMotor = 2;
  sensor_infos[0].scale = 100;
  
  // Back left wheel encoder
  sensor_infos[3].hardware = SH_RC_ENC;
  sensor_infos[3].addr = ADDRESS_RC_1;
  sensor_infos[3].whichMotor = 1;
  sensor_infos[3].scale = 100;
  
  // Back right wheel encoder
  sensor_infos[2].hardware = SH_RC_ENC;
  sensor_infos[2].addr = ADDRESS_RC_1;
  sensor_infos[2].whichMotor = 2;
  sensor_infos[2].scale = 100;

  // 3 is front left
  sensor_infos[4].hardware = SH_PIN_POT;
  sensor_infos[4].whichPin = 3;
  sensor_infos[4].scale = 1;
  sensor_infos[4].responsiveness = 0.1;
  // 2 is front right
  sensor_infos[5].hardware = SH_PIN_POT;
  sensor_infos[5].whichPin = 2;
  sensor_infos[5].scale = 1;
  sensor_infos[5].responsiveness = 0.1;
  // Back left wheel pod potentiometer
  sensor_infos[6].hardware = SH_PIN_POT;
  sensor_infos[6].whichPin = 1;
  sensor_infos[6].scale = 1;
  sensor_infos[6].responsiveness = 0.1;
  // 0 is back right
  sensor_infos[7].hardware = SH_PIN_POT;
  sensor_infos[7].whichPin = 0;
  sensor_infos[7].scale = 1;
  sensor_infos[7].responsiveness = 0.1;

  // DUMMY SENSORS for setting pot mode
  sensor_infos[8].hardware = SH_RC_POT;
  sensor_infos[8].addr = ADDRESS_RC_3;
  sensor_infos[8].whichMotor = 2;
  sensor_infos[8].scale = 1;
  
  sensor_infos[9].hardware = SH_RC_POT;
  sensor_infos[9].addr = ADDRESS_RC_2;
  sensor_infos[9].whichMotor = 1;
  sensor_infos[9].scale = 1;
  // END DUMMY SENSORS

  //BC Arm position pin pot A
  sensor_infos[16].hardware = SH_PIN_POT;
  sensor_infos[16].whichPin = 4;
  sensor_infos[16].scale = 1;
  sensor_infos[16].responsiveness = 0.1;

   //BC Arm position pin pot B
  sensor_infos[19].hardware = SH_PIN_POT;
  sensor_infos[19].whichPin = 5;
  sensor_infos[19].scale = 1;
  sensor_infos[19].responsiveness = 0.1;

/*
  // BC Arm Pot A
  sensor_infos[16].hardware = SH_RC_POT;
  sensor_infos[16].addr = ADDRESS_RC_2;
  sensor_infos[16].whichMotor = 1;
  sensor_infos[16].scale = 1;
  
  // BC Arm Pot B
  sensor_infos[19].hardware = SH_RC_POT;
  sensor_infos[19].addr = ADDRESS_RC_2;
  sensor_infos[19].whichMotor = 2;
  sensor_infos[19].scale = 1;
*/
  
  // BC Translation Pot
  sensor_infos[22].hardware = SH_PIN_POT;
  sensor_infos[22].whichPin = 6;
  sensor_infos[22].scale = 1;
  sensor_infos[22].responsiveness = 1;

  //BC Limit Switch A Retracted
  sensor_infos[23].hardware = SH_PIN_LIMIT;
  sensor_infos[23].whichPin = 36;
  sensor_infos[23].scale = 1;

  //BC Limit Switch A Extended
  sensor_infos[24].hardware = SH_PIN_LIMIT;
  sensor_infos[24].whichPin = 37;
  sensor_infos[24].scale = 1;

  //BC Limit Switch B Retracted
  sensor_infos[25].hardware = SH_PIN_LIMIT;
  sensor_infos[25].whichPin = 38;
  sensor_infos[25].scale = 1;

  //BC Limit Switch B Extended
  sensor_infos[26].hardware = SH_PIN_LIMIT;
  sensor_infos[26].whichPin = 39;
  sensor_infos[26].scale = 1;

  sensor_infos[38].hardware = SH_RC_CUR;
  sensor_infos[38].addr = ADDRESS_RC_3;
  sensor_infos[38].whichMotor = 1;
  sensor_infos[38].scale = 1;
  
  // Front left wheel motor
  motor_infos[1].hardware = MH_RC_VEL;
  motor_infos[1].addr = ADDRESS_RC_0;
  motor_infos[1].whichMotor = 1;
  motor_infos[1].kp = 16;
  motor_infos[1].ki = 0;
  motor_infos[1].kd = 0;
  motor_infos[1].qpps = 32768;
  motor_infos[1].scale = 3;
  
  // Front right wheel motor
  motor_infos[0].hardware = MH_RC_VEL;
  motor_infos[0].addr = ADDRESS_RC_0;
  motor_infos[0].whichMotor = 2;
  motor_infos[0].kp = 16;
  motor_infos[0].ki = 0;
  motor_infos[0].kd = 0;
  motor_infos[0].qpps = 32768;
  motor_infos[0].scale = 3;
  
  // Back left wheel motor
  motor_infos[3].hardware = MH_RC_VEL;
  motor_infos[3].addr = ADDRESS_RC_1;
  motor_infos[3].whichMotor = 1;
  motor_infos[3].kp = 16;
  motor_infos[3].ki = 0;
  motor_infos[3].kd = 0;
  motor_infos[3].qpps = 32768;
  motor_infos[3].scale = 3;
  
  // Back right wheel motor
  motor_infos[2].hardware = MH_RC_VEL;
  motor_infos[2].addr = ADDRESS_RC_1;
  motor_infos[2].whichMotor = 2;
  motor_infos[2].kp = 16;
  motor_infos[2].ki = 0;
  motor_infos[2].kd = 0;
  motor_infos[2].qpps = 32768;
  motor_infos[2].scale = 3;

  // Actuator FL 
  motor_infos[4].hardware = MH_ST_POS;
  motor_infos[4].addr = 0;
  motor_infos[4].whichMotor = 1;
  motor_infos[4].feedbackSensorID = 4;
  motor_infos[4].kp = 2;
  motor_infos[4].deadband = 15;
  motor_infos[4].scale = 1;
  
  // Actuator FR 
  motor_infos[5].hardware = MH_ST_POS;
  motor_infos[5].addr = 1;
  motor_infos[5].whichMotor = 2;
  motor_infos[5].feedbackSensorID = 5;
  motor_infos[5].kp = -2;
  motor_infos[5].deadband = 15;
  motor_infos[5].scale = 1;
  
  // Actuator BL
  motor_infos[6].hardware = MH_ST_POS;
  motor_infos[6].addr = 1;
  motor_infos[6].whichMotor = 1;
  motor_infos[6].feedbackSensorID = 6;
  motor_infos[6].kp = 2;
  motor_infos[6].deadband = 15;
  motor_infos[6].scale = 1;
  
  // Actuator BR
  motor_infos[7].hardware = MH_ST_POS;
  motor_infos[7].addr = 0;
  motor_infos[7].whichMotor = 2;
  motor_infos[7].feedbackSensorID = 7;
  motor_infos[7].kp = -2;
  motor_infos[7].deadband = 15;
  motor_infos[7].scale = 1;
  // Implied use same-id sensor

  // Bucket Conveyor Drive motor TODO
  motor_infos[8].hardware = MH_RC_PWM;
  motor_infos[8].addr = ADDRESS_RC_3;
  motor_infos[8].whichMotor = 1;
  motor_infos[8].scale = 1;

  // Bucket Conveyor Linear motor TODO
  motor_infos[9].hardware = MH_ST_POS;
  motor_infos[9].addr = 3;
  motor_infos[9].whichMotor = 1;
  motor_infos[9].scale = 1;
  motor_infos[9].feedbackSensorID = 22;
  motor_infos[9].deadband = 10;
  motor_infos[9].kp = 0.6;
  motor_infos[9].ki = 0.005;
  motor_setpoints[9] = analogRead(sensor_infos[motor_infos[9].feedbackSensorID].whichPin);
  sensor_storedVals[motor_infos[9].feedbackSensorID] = motor_setpoints[9];
  motor_lastUpdateTime[9] = millis();
  motor_infos[9].saturation = 64/ motor_infos[9].ki;

  // Bucket Conveyor Actuators
  motor_infos[10].hardware = MH_RC_POS_BOTH;
  motor_infos[10].addr = ADDRESS_RC_2;
  motor_infos[10].kp = 10;
  motor_infos[10].ki = 0;
  motor_infos[10].kd = 0;
  motor_infos[10].qpps = 200;
  motor_infos[10].deadband = 10;
  motor_infos[10].minpos = 0;
  motor_infos[10].maxpos = 2047;
  motor_infos[10].accel = 1000;
  motor_infos[10].scale = 1;
  motor_infos[10].feedbackSensorID = 16;
  motor_setpoints[10] = 300;

  // Deposition Conveyor Motor TODO
  motor_infos[11].hardware = MH_ST_PWM;
  motor_infos[11].addr = 3;
  motor_infos[11].whichMotor = 2;
  motor_infos[11].scale = 1;

  // Deposition Actuators
  motor_infos[12].hardware = MH_ST_PWM_BOTH;
  motor_infos[12].addr = 2;
  motor_infos[12].scale = 1;

  motor_infos[13].hardware = MH_PIN_PWM;
  motor_infos[13].addr = 12;
  pinMode(motor_infos[13].addr, OUTPUT);
  digitalWrite(motor_infos[13].addr, LOW);
  motor_infos[13].scale = 1;

  // Send a command to all motors
  motor_infos[50].hardware = MH_ALL;
  motor_infos[50].scale = 1;

  /*
  motor_infos[2].hardware = MH_RC_POS;
  motor_infos[2].addr = ADDRESS_RC_3;
  motor_infos[2].whichMotor = 0;
  motor_infos[2].kp = 31512.70535;
  motor_infos[2].ki = 23.57707;
  motor_infos[2].kd = 7019890.76290;
  motor_infos[2].qpps = 330;
  motor_infos[2].deadband = 10;
  motor_infos[2].minpos = 84;
  motor_infos[2].maxpos = 1676;
  motor_infos[2].accel = 9999999;
  */

  /*
  motor_infos[2].hardware = MH_ST_PWM;
  motor_infos[2].addr = 0;
  motor_infos[2].whichMotor = 1;
  */

  /*
  motor_infos[2].hardware = MH_RC_VEL;
  motor_infos[2].addr = ADDRESS_RC_1;
  motor_infos[2].whichMotor = 0;
  motor_infos[2].kp = 50;
  motor_infos[2].ki = 327;
  motor_infos[2].kd = 0;
  motor_infos[2].qpps = 737992;
  motor_infos[2].accel = 500000;
  */
  
  setup_comms();
  setup_sabretooth();
  setup_roboclaw();
  configure_sensors();
  configure_motors();
  save_roboclaw();
}

void setup_comms() {
  SerialUSB.begin(9600);
  Serial.begin(9600);
}

void setup_sabretooth() {
  Serial2.begin(9600);
}

FAULT_T setup_roboclaw() {
  roboclaw.begin(38400);
  bool success;
  success = roboclaw.SetConfig(ADDRESS_RC_0, 0x8063);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  success = roboclaw.SetConfig(ADDRESS_RC_1, 0x8163);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  success = roboclaw.SetConfig(ADDRESS_RC_2, 0x8263);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  success = roboclaw.SetConfig(ADDRESS_RC_3, 0x8363);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  return NO_FAULT;
}

FAULT_T save_roboclaw() {
  bool success;
  success = roboclaw.WriteNVM(ADDRESS_RC_0);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  success = roboclaw.WriteNVM(ADDRESS_RC_1);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  success = roboclaw.WriteNVM(ADDRESS_RC_2);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  success = roboclaw.WriteNVM(ADDRESS_RC_3);
  if (!success) {
    return FAULT_LOST_ROBOCLAW;
  }
  return NO_FAULT;
}

FAULT_T configure_sensors() {
  bool success;
  for (int i = 0; i < 256; i++) {
    SensorInfo sensor_info = sensor_infos[i];
    switch (sensor_info.hardware) {
    case SH_RC_POT:
      if (sensor_info.whichMotor == 2) {
        success = roboclaw.SetM2EncoderMode(sensor_info.addr, 0x81);
      } else {
        success = roboclaw.SetM1EncoderMode(sensor_info.addr, 0x81);
      }
      if (!success) {
        // Commented out to debug
        //return FAULT_LOST_ROBOCLAW;
        Serial.println("Lost roboclaw");
      }
      break;
    case SH_RC_ENC:
      if (sensor_info.whichMotor == 2) {
        success = roboclaw.SetM2EncoderMode(sensor_info.addr, 0x80);
      } else {
        success = roboclaw.SetM1EncoderMode(sensor_info.addr, 0x80);
      }
      if (!success) {
        // Commented out to debug
        //return FAULT_LOST_ROBOCLAW;
        Serial.println("Lost roboclaw");
      }
      break;
    case SH_PIN_LIMIT:
      // Pull-up
      pinMode(sensor_info.whichPin,INPUT);
      digitalWrite(sensor_info.whichPin,HIGH);
      break;
    case SH_PIN_POT:
      // Nothing to do here
      break;
    default:
      break;
    }
  }
  return NO_FAULT;
}

FAULT_T configure_motors() {
  bool success;
  for (int i = 0; i < 256; i++) {
    MotorInfo motor_info = motor_infos[i];
    switch (motor_info.hardware) {
    case MH_RC_PWM:
      // Nothing to do, default config
      break;
    case MH_RC_VEL:
      if (motor_info.whichMotor == 2) {
        success = roboclaw.SetM2VelocityPID(
          motor_info.addr,
          motor_info.kp,
          motor_info.ki,
          motor_info.kd,
          motor_info.qpps);
      } else {
        success = roboclaw.SetM1VelocityPID(
          motor_info.addr,
          motor_info.kp,
          motor_info.ki,
          motor_info.kd,
          motor_info.qpps);
      }
      if (!success) {
        return FAULT_LOST_ROBOCLAW;
      }
      break;
    case MH_RC_POS:
      if (motor_info.whichMotor == 2) {
        success = roboclaw.SetM2PositionPID(
          motor_info.addr,
          motor_info.kp,
          motor_info.ki,
          motor_info.kd,
          motor_info.qpps,
          motor_info.deadband,
          motor_info.minpos,
          motor_info.maxpos);
      } else {
        success = roboclaw.SetM1PositionPID(
          motor_info.addr,
          motor_info.kp,
          motor_info.ki,
          motor_info.kd,
          motor_info.qpps,
          motor_info.deadband,
          motor_info.minpos,
          motor_info.maxpos);
      }
      if (!success) {
        return FAULT_LOST_ROBOCLAW;
      }
      break;
    case MH_ST_PWM:
      // Nothing to do, default config
      break;
    case MH_ST_POS:
      // Nothing to do, default config
      break;
    case MH_ST_PWM_BOTH:
      // Nothing to do, default config
      break;  
    case MH_RC_POS_BOTH:
      success = roboclaw.SetM1PositionPID(
          motor_info.addr,
          motor_info.kp,
          motor_info.ki,
          motor_info.kd,
          motor_info.qpps,
          motor_info.deadband,
          motor_info.minpos,
          motor_info.maxpos);
      if (!success) {
        return FAULT_LOST_ROBOCLAW;
      }
      success = roboclaw.SetM2PositionPID(
          motor_info.addr,
          motor_info.kp,
          motor_info.ki,
          motor_info.kd,
          motor_info.qpps,
          motor_info.deadband,
          motor_info.minpos,
          motor_info.maxpos);
      if (!success) {
        return FAULT_LOST_ROBOCLAW;
      }
      break;
    case MH_ALL:
      //nothing to do
      break;  
    default:
      break;
    }
  }
  return NO_FAULT;
}

void loop() {
  while (true) {
    FAULT_T retfault;
    byte cmd[256];
    hciWait();
    retfault = hciRead(cmd);
    if (retfault != NO_FAULT) {
      // Enter sync mode
      // When done:
      continue;
    } else {
      // cmd is valid
      execute(cmd);
    }
  }
  
}

void execute(byte cmd[]) {
  byte rpy[256]; // max-sized response buffer
  uint8_t type = cmd_type(cmd);
  uint8_t num_sensors_requested;
  uint8_t num_motors_requested;
  FAULT_T retfault;
  switch(type) {
    case COMMAND_HCI_TEST:
      SerialUSB.write(RESPONSE_HCI_TEST);
      break;
    case COMMAND_READ_SENSORS:
      rpy_init(rpy, type);
      num_sensors_requested = cmd_sense_num_sensors(cmd);
      for(int i = 0; i < num_sensors_requested; i++) {
        uint16_t id =  cmd_sense_sensor_id(cmd, i);
        int16_t val;
        FAULT_T retfault = getSensor(id, &val);
        if (retfault != NO_FAULT) {
          val = -500; // Error code
        }
        bool overflow = rpy_sense_add_sensor(rpy, id, val);
        if (overflow) {
          // do nothing'  
        }
      }
      rpy_finalize(rpy);
      retfault = hciWrite(rpy);
      if (retfault != NO_FAULT) {
        // TODO: enter sync mode
      }
      break;
    case COMMAND_SET_OUTPUTS:
      rpy_init(rpy, type);
      num_motors_requested = cmd_ctl_num_motors(cmd);
      for(int i = 0; i < num_motors_requested; i++) {
        uint16_t id = cmd_ctl_motor_id(cmd, i);
        int16_t val = cmd_ctl_motor_val(cmd, i);
        setActuator(id, val);
      }
      rpy_finalize(rpy);
      retfault = hciWrite(rpy);
      if (retfault != NO_FAULT) {
        // TOCO: enter sync mode
      }
      break;
  }
}

FAULT_T getSensor(uint16_t ID, int16_t *val) {
  SensorInfo sensor_info = sensor_infos[ID];
  uint8_t status;
  bool valid;
  int32_t val32;
  int16_t dummy;
  int16_t readVal;
  switch (sensor_info.hardware) {
  case SH_RC_POT:
    if (sensor_info.whichMotor == 2) {
      val32 = roboclaw.ReadEncM2(sensor_info.addr, &status, &valid);
    } else {
      val32 = roboclaw.ReadEncM1(sensor_info.addr, &status, &valid);
    }
    if (!valid){
      return FAULT_LOST_ROBOCLAW;
    }
    *val = (int16_t)(val32 / sensor_info.scale);
    break;
  case SH_RC_ENC:
    if (sensor_info.whichMotor == 2) {
      val32 = roboclaw.ReadSpeedM2(sensor_info.addr, &status, &valid);
      //roboclaw.ReadCurrents(sensor_info.addr, dummy, *val);
    } else {
      val32 = roboclaw.ReadSpeedM1(sensor_info.addr, &status, &valid);
      //roboclaw.ReadCurrents(sensor_info.addr, *val, dummy);
    }
    if (!valid){
      return FAULT_LOST_ROBOCLAW;
    }
    *val = (int16_t)(val32 / sensor_info.scale);
    break;
    //Not sure if this works yet!
  case SH_RC_CUR:
    if (sensor_info.whichMotor == 2){
       valid = roboclaw.ReadCurrents(sensor_info.addr, dummy, *val);
    } else {
       valid = roboclaw.ReadCurrents(sensor_info.addr, *val, dummy);
    }
    if (!valid){
      return FAULT_LOST_ROBOCLAW;
    }
    *val = *val / sensor_info.scale;
    break;
  case SH_PIN_LIMIT:
    //if the pin limit switch is for BC translation:
    if(ID >= 23 && ID <= 26){
      *val = !digitalRead(sensor_info.whichPin);
      if(*val && !sensor_lastLimitVals[ID]) {
        //something just changed from low to high, stop actuation.
        sabretooth[motor_infos[9].addr].motor(motor_infos[9].whichMotor, 0);
      }
      //else we should be ok
    }
    sensor_lastLimitVals[ID] = *val;
    break;
  case SH_PIN_POT:
    readVal = (int16_t)analogRead(sensor_info.whichPin) / sensor_info.scale;
    sensor_storedVals[ID] = (sensor_storedVals[ID] * (1 - sensor_info.responsiveness)) + readVal * sensor_info.responsiveness;
    *val = sensor_storedVals[ID];
    break;
  default:
    break;
  }
  return NO_FAULT;
}

FAULT_T setActuator(uint16_t ID, int16_t val) {

  bool success;
  MotorInfo motor_info = motor_infos[ID];
  int val_scaled = val * motor_infos[ID].scale;
  switch (motor_info.hardware) {
  case MH_RC_PWM:
    if(stopped){
      break;
    }
    if (motor_info.whichMotor == 2) {
      success = roboclaw.DutyM2(motor_info.addr, val_scaled);
    } else {
      success = roboclaw.DutyM1(motor_info.addr, val_scaled);
    }
    break;
  case MH_RC_VEL:
    if(stopped){
      break;
    }
    if (motor_info.whichMotor == 2) {
      success = roboclaw.SpeedAccelM2(motor_info.addr, motor_info.qpps, val_scaled);
    } else {
      success = roboclaw.SpeedAccelM1(motor_info.addr, motor_info.qpps, val_scaled);
    }
    if (!success) {
      return FAULT_LOST_ROBOCLAW;
    }
    break;
  case MH_RC_POS:
    if(stopped){
      break;
    }
    if (motor_info.whichMotor == 2) {
      success = roboclaw.SpeedAccelDeccelPositionM2(
        motor_info.addr,
        motor_info.accel,
        motor_info.qpps,
        motor_info.accel,
        val_scaled,
        0);
    } else {
      success = roboclaw.SpeedAccelDeccelPositionM1(
        motor_info.addr,
        motor_info.accel,
        motor_info.qpps,
        motor_info.accel,
        val_scaled,
        0);
    }
    if (!success) {
      return FAULT_LOST_ROBOCLAW;
    }
    break;
  case MH_ST_PWM:
    if(stopped){
      break;
    }
    //whenever we try to move the BC translation motor, we check if limits are pressed
    //jank solution with hardcoded values yay
    /*if(ID == 9) {
      if(val > 0 && (digitalRead(37) == LOW || digitalRead(39) == LOW)) {
        //We hit a switch and are trying to move in the same direction, stop!
        sabretooth[motor_info.addr].motor(motor_info.whichMotor, 0);
        break;
      }
      else if(val < 0 && (digitalRead(36) == LOW || digitalRead(38) == LOW)) {
        //We hit a switch and are trying to move in the same direction, stop!
        sabretooth[motor_info.addr].motor(motor_info.whichMotor, 0);
        break;
      }
    }*/
    sabretooth[motor_info.addr].motor(motor_info.whichMotor, val_scaled);
      
    Serial.println((val_scaled != 0) * 80);   
    break;
  case MH_ST_POS:
    motor_setpoints[ID] = val_scaled;
    break;
  case MH_RC_POS_BOTH:
    motor_setpoints[ID] = val_scaled;
    break;
  case MH_ST_PWM_BOTH:
    if(stopped){
      break;
    }
    sabretooth[motor_info.addr].motor(1, -val_scaled);
    sabretooth[motor_info.addr].motor(2, val_scaled); 
    break;
  case MH_PIN_PWM:
    if(stopped){
      break;
    }
    analogWrite(motor_info.addr, val_scaled); 
    break;  
  case MH_ALL:
    if(val_scaled == 0){ // we want to stop all of the motors
      for(int i = 0; i < 13; i++){
        if(motor_infos[i].hardware != MH_ST_POS && motor_infos[i].hardware != MH_RC_POS_BOTH){
           setActuator(i, 0);
        }
        else if(motor_infos[i].hardware == MH_ST_POS){
          sabretooth[motor_infos[i].addr].motor(motor_infos[i].whichMotor, 0);
        }
        else if(motor_infos[i].hardware == MH_RC_POS_BOTH){
          roboclaw.ForwardM1(motor_infos[i].addr, 0);
          roboclaw.ForwardM2(motor_infos[i].addr, 0);
        }
      }
      stopped = true; //prevents access to hciwait();
    }
    else if (val_scaled == 1){ //we can now start all the motors
      stopped = false; //allows access to hciwait();
      motor_setpoints[4] = analogRead(sensor_infos[motor_infos[4].feedbackSensorID].whichPin); //FL wheel actuator
      motor_setpoints[5] = analogRead(sensor_infos[motor_infos[5].feedbackSensorID].whichPin); //FR wheel actuator
      motor_setpoints[6] = analogRead(sensor_infos[motor_infos[6].feedbackSensorID].whichPin); //BL wheel actuator
      motor_setpoints[7] = analogRead(sensor_infos[motor_infos[7].feedbackSensorID].whichPin); //BR wheel actuator
      motor_setpoints[9] = analogRead(sensor_infos[motor_infos[9].feedbackSensorID].whichPin); //BC translation
      motor_setpoints[10] = analogRead(sensor_infos[motor_infos[10].feedbackSensorID].whichPin); //BC rotation
      motor_lastUpdateTime[9] = millis();
      motor_integrals[9] = 0;
    }
  default:
    break;
  }
  return NO_FAULT;
}

void hciWait() {
  do {
    
    if(stopped){
      continue; 
    }
    for (int id = 0; id < 256; id++) {
      MotorInfo motor_info = motor_infos[id];
      if (motor_info.hardware == MH_ST_POS || motor_info.hardware == MH_RC_POS_BOTH) {
        int sensorID = motor_info.feedbackSensorID;
        int16_t pos;
        getSensor(sensorID, &pos); // TODO: detect fault
        int err = motor_setpoints[id] - pos;    
        if (err <= (signed)motor_info.deadband) {
          if (err >= -(signed)motor_info.deadband) {
            // In deadband
            
            err = 0;
          } else {
            // Below deadband
            err += motor_info.deadband;
          }
        } else {
          // Above deadband
          err -= motor_info.deadband;
        }
        int16_t updateTime = millis();
        motor_integrals[id] += err * (updateTime - motor_lastUpdateTime[id]);
        if (motor_integrals[id] > motor_info.saturation) {
          motor_integrals[id] = motor_info.saturation;
        }
        else if (motor_integrals[id] < -motor_info.saturation) {
          motor_integrals[id] = -motor_info.saturation;
        }
        motor_lastUpdateTime[id] = updateTime;
        if(err * motor_integrals[id] < 0){
          motor_integrals[id] = 0;
        }
        int val = motor_info.kp * err + motor_info.ki * motor_integrals[id];
        if (val > 127) {
          val = 127;
        }
        else if (val < -127) {
          val = -127;
        }
        
        bool success;
        if(motor_info.hardware == MH_ST_POS) {
          if(id == 9) {
            Serial.print(id);
            Serial.print(" ");
            Serial.print(pos);
            Serial.print(" ");
            Serial.print(motor_setpoints[id]);
            Serial.print(" ");
            Serial.print(motor_info.kp * err);
            Serial.print(" ");
            Serial.print(motor_info.ki * motor_integrals[id]);
            Serial.print(" ");
            Serial.print(val);
            Serial.println(" ");
            if(val > 0 && (digitalRead(37) == LOW || digitalRead(39) == LOW)) {
              //We hit a switch and are trying to move in the same direction, stop!
              sabretooth[motor_info.addr].motor(motor_info.whichMotor, 0);
              Serial.println("Limits preventing positive movement");
              continue;
            }
            else if(val < 0 && (digitalRead(36) == LOW || digitalRead(38) == LOW)) {
              //We hit a switch and are trying to move in the same direction, stop!
              sabretooth[motor_info.addr].motor(motor_info.whichMotor, 0);
              Serial.println("Limits preventing negative movement");
              continue;
            }
          }
          sabretooth[motor_info.addr].motor(motor_info.whichMotor, val);
        }
        else if(motor_info.hardware == MH_RC_POS_BOTH){
          if(val >= 0){
            success = roboclaw.BackwardM1(motor_info.addr, val);
            success = roboclaw.BackwardM2(motor_info.addr, val);
          }
          else{
            success = roboclaw.ForwardM1(motor_info.addr, -val);
            success = roboclaw.ForwardM2(motor_info.addr, -val);
          }      
        }  
      }
    }
  } while (!SerialUSB.available());
}

FAULT_T hciWrite(byte rpy[]) {
  uint8_t retval;
  uint8_t len = rpy_len(rpy);
  retval = SerialUSB.write(rpy,len+2);
  if (retval != len+2) {
    return FAULT_FAILED_WRITE;
  }
  return NO_FAULT;
}

FAULT_T hciRead(byte cmd[]) {
  uint8_t retval;
  retval = SerialUSB.readBytes(cmd, CMD_HEADER_SIZE);
  if (retval != CMD_HEADER_SIZE) {
    return FAULT_INCOMPLETE_HEADER;
  }
  bool valid = cmd_check_head(cmd);
  if (!valid) {
    return FAULT_CORRUPTED_HEADER;
  }
  uint8_t len = cmd_len(cmd);
  retval = SerialUSB.readBytes(cmd + CMD_HEADER_SIZE, len);
  if (retval != len) {
    return FAULT_INCOMPLETE_BODY;
  }
  valid = cmd_check_body(cmd);
  if (!valid) {
    return FAULT_CORRUPTED_BODY;
  }
  return NO_FAULT;
}

// Precondition: every message array has length at least 256

bool cmd_check_head(byte cmd[]) {
  return true;
}

uint8_t cmd_type(byte cmd[]) {
  return cmd[0];
}

uint8_t cmd_len(byte cmd[]) {
  return cmd[1];
}

bool cmd_check_body(byte cmd[]) {
  return true;
}

uint8_t cmd_sense_num_sensors(byte cmd[]) {
  return cmd_len(cmd)/2;
}

uint16_t cmd_sense_sensor_id(byte cmd[], uint8_t i) {
  // i unchecked
  return ((uint16_t)cmd[CMD_HEADER_SIZE + 2*i + 0] << 8) | cmd[CMD_HEADER_SIZE + 2*i + 1];
}

uint8_t cmd_ctl_num_motors(byte cmd[]) {
  return cmd_len(cmd)/4;
}

uint16_t cmd_ctl_motor_id(byte cmd[], uint8_t i) {
  // i unchecked
  return ((uint16_t)cmd[CMD_HEADER_SIZE + 4*i + 0] << 8) | cmd[CMD_HEADER_SIZE + 4*i + 1];
}

int16_t cmd_ctl_motor_val(byte cmd[], uint8_t i) {
  // i unchecked
  return ((int16_t)cmd[CMD_HEADER_SIZE + 4*i + 2] << 8) | cmd[CMD_HEADER_SIZE + 4*i + 3];
}

void rpy_init(byte rpy[], uint8_t type) {
  rpy[0] = type;
  rpy[1] = 0; // len
}

uint8_t rpy_len(byte rpy[]) {
  return rpy[1];
}

void rpy_set_len(byte rpy[], uint8_t new_len) {
  rpy[1] = new_len;
}

bool rpy_sense_add_sensor(byte rpy[], uint16_t id, int16_t val) {
  // size unchecked
  uint8_t len = rpy_len(rpy);
  if (len >= 252) {
    return true;
  }
  byte *rpy_id_high_ptr = rpy + RPY_HEADER_SIZE + len + 0;
  byte *rpy_id_low_ptr = rpy + RPY_HEADER_SIZE + len + 1;
  byte *rpy_val_high_ptr = rpy + RPY_HEADER_SIZE + len + 2;
  byte *rpy_val_low_ptr = rpy + RPY_HEADER_SIZE + len + 3;
  *rpy_id_high_ptr = (byte)(id >> 8);
  *rpy_id_low_ptr = (byte)id;
  *rpy_val_high_ptr = (byte)((unsigned)val >> 8);
  *rpy_val_low_ptr = (byte)val;
  len += 4;
  rpy_set_len(rpy, len);
  return false;
}

bool rpy_ctl_add_motor(byte rpy[], uint16_t id, int16_t val) {
  // size unchecked
  uint8_t len = rpy_len(rpy);
  if (len >= 252) {
    return true;
  }
  byte *rpy_id_high_ptr = rpy + RPY_HEADER_SIZE + len + 0;
  byte *rpy_id_low_ptr = rpy + RPY_HEADER_SIZE + len + 1;
  byte *rpy_val_high_ptr = rpy + RPY_HEADER_SIZE + len + 2;
  byte *rpy_val_low_ptr = rpy + RPY_HEADER_SIZE + len + 3;
  *rpy_id_high_ptr = (byte)(id >> 8);
  *rpy_id_low_ptr = (byte)id;
  *rpy_val_high_ptr = (byte)((unsigned)val >> 8);
  *rpy_val_low_ptr = (byte)val;
  len += 4;
  rpy_set_len(rpy, len);
  return false;
}

void rpy_finalize(byte rpy[]) {}


