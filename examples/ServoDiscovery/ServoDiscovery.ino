#include <HalfDuplexHardwareSerial.h>

#include <DynamixelBus.h>

DynamixelBus dynamixel(Serial1);

#define CONNECTED_SERVO_COUNT 4
#define MAX_SERVO_ID 4

const char* baudrate[] = {  "unknown", "9600", "19200", "57600", "115200", "200000", "250000", "400000", "500000", "1000000" };

void report(uint8_t* ids, size_t index) {

  uint8_t servoId = ids[index];

  // The discovery operation will have saved the servo's baud rate
  DynamixelBus::BaudRate baud;
  dynamixel.getBaudRate(servoId,baud);
 
  Serial.print(F("Servo #"));
  Serial.println(servoId);

  // Switch to the servo's baud rate
  dynamixel.begin(baud);

  // Uncomment the following two statements to change the servo baud rate
  // dynamixel.setBaudRate(servoId,DynamixelBus::k115200);
  // dynamixel.begin(DynamixelBus::k115200);

  // Instruct the servo to respond to all instructions for testing purposes
  dynamixel.setStatusReturnLevel(servoId,DynamixelBus::kRespondToAll);


  // Uncomment the following to reset triggers to desired values
  // dynamixel.setAlarmLedTriggers(servoId,DynamixelBus::kOverloadError | DynamixelBus::kTemperatureLimitError);
  // dynamixel.setAlarmShutdownTriggers(servoId,DynamixelBus::kOverloadError | DynamixelBus::kTemperatureLimitError);

  // Now capture the current state of the servo..
  // (No guarantee every one of these calls will succeed!)

  uint16_t servoModelNumber = 12;
  uint8_t servoFirmwareVersion = 0;
  uint16_t servoReturnDelayTime = 0;
  uint16_t servoMinAngle = 0;
  uint16_t servoMaxAngle = 0;
  uint8_t servoTemperatureLimit = 0;
  uint16_t servoLowVoltageLimit = 0;
  uint16_t servoHighVoltageLimit = 0;
  uint16_t servoInitialTorqueLimit = 0;
  uint8_t servoAlarmLedTriggers = 0;
  uint8_t servoAlarmShutdownTriggers = 0;
  bool servoTorqueEnable = false;
  uint8_t servoCWComplianceMargin = 0;
  uint8_t servoCCWComplianceMargin = 0;
  uint8_t servoCWComplianceSlope = 0;
  uint8_t servoCCWComplianceSlope = 0;
  uint16_t servoGoalPosition = 0;
  uint16_t servoGoalSpeed = 0;
  uint16_t servoTorqueLimit = 0;
  uint16_t servoPresentPosition = 0;
  uint16_t servoPresentSpeed = 0;
  uint16_t servoPresentLoad = 0;
  uint8_t servoPresentLoadDirection = 0;
  uint16_t servoPresentVoltage = 0;
  uint8_t servoPresentTemperature = 0;
  bool servoLock = false;
  uint16_t servoPunch = 0;

  dynamixel.getModelNumber(servoId,servoModelNumber);
  dynamixel.getFirmwareVersion(servoId,servoFirmwareVersion);
  dynamixel.getReturnDelayTime(servoId,servoReturnDelayTime);
  dynamixel.getAngleLimits(servoId,servoMinAngle,servoMaxAngle);
  dynamixel.getTemperatureLimit(servoId,servoTemperatureLimit);
  dynamixel.getVoltageLimits(servoId,servoLowVoltageLimit,servoHighVoltageLimit);
  dynamixel.getInitialTorqueLimit(servoId,servoInitialTorqueLimit);
  dynamixel.getAlarmLedTriggers(servoId,servoAlarmLedTriggers);
  dynamixel.getAlarmShutdownTriggers(servoId,servoAlarmShutdownTriggers);
  dynamixel.getTorqueEnable(servoId,servoTorqueEnable);
  dynamixel.getComplianceMargins(servoId,servoCWComplianceMargin,servoCCWComplianceMargin);
  dynamixel.getComplianceSlopes(servoId,servoCWComplianceSlope,servoCCWComplianceSlope);
  dynamixel.getGoalPosition(servoId,servoGoalPosition);
  dynamixel.getGoalSpeed(servoId,servoGoalSpeed);
  dynamixel.getTorqueLimit(servoId,servoTorqueLimit);
  dynamixel.getLock(servoId,servoLock);
  dynamixel.getPunch(servoId,servoPunch);
  dynamixel.getPresentPosition(servoId,servoPresentPosition);
  dynamixel.getPresentSpeed(servoId,servoPresentSpeed);
  dynamixel.getPresentLoad(servoId,servoPresentLoadDirection,servoPresentLoad);
  dynamixel.getPresentVoltage(servoId,servoPresentVoltage);
  dynamixel.getPresentTemperature(servoId,servoPresentTemperature);
  
  Serial.println(F("------------------------------"));
  Serial.print(F(": ID = "));
  Serial.println(servoId);
  Serial.print(F(": Baud Rate = "));
  Serial.print(baudrate[baud]);
  Serial.print(F(" ("));
  Serial.print(baud);
  Serial.println(F(")"));
  Serial.print(F(": Model Number = "));
  Serial.println(servoModelNumber);
  Serial.print(F(": Firmware Version = "));
  Serial.println(servoFirmwareVersion);
  Serial.print(F(": Return Delay Time (usec) = "));
  Serial.println(servoReturnDelayTime);
  Serial.print(F(": Angle Limits: cw = "));
  Serial.print(servoMinAngle);
  Serial.print(F("/1023 ("));
  Serial.print((servoMinAngle * 300L / 1023));
  Serial.print(F("°)  ccw = "));
  Serial.print(servoMaxAngle);
  Serial.print(F("/1023 ("));
  Serial.print((servoMaxAngle * 300L / 1023));
  Serial.println("°)");
  Serial.print(F(": Temperature Limit (C) = "));
  Serial.println(servoTemperatureLimit);
  Serial.print(F(": Voltage Limits (V): low = "));
  Serial.print(servoLowVoltageLimit/1000.0);
  Serial.print(F(" high = "));
  Serial.println(servoHighVoltageLimit/1000.0);
  Serial.print(F(": Initial Torque Limit (EEPROM) = "));
  Serial.print(servoInitialTorqueLimit);
  Serial.println(F("/1023"));
  Serial.print(F(": Alarm LED Errors: "));
  if (servoAlarmLedTriggers & DynamixelBus::kInstructionError) Serial.print(F("instruction, "));
  if (servoAlarmLedTriggers & DynamixelBus::kOverloadError) Serial.print(F("overload, "));
  if (servoAlarmLedTriggers & DynamixelBus::kChecksumError) Serial.print(F("checksum, "));
  if (servoAlarmLedTriggers & DynamixelBus::kRangeError) Serial.print(F("range, "));
  if (servoAlarmLedTriggers & DynamixelBus::kTemperatureLimitError) Serial.print(F("temperature, "));
  if (servoAlarmLedTriggers & DynamixelBus::kAngleLimitError) Serial.print(F("angle, "));
  if (servoAlarmLedTriggers & DynamixelBus::kVoltageLimitError) Serial.print(F("voltage"));
  Serial.println();
  Serial.print(F(": Alarm Shutdown Errors: "));
  if (servoAlarmShutdownTriggers & DynamixelBus::kInstructionError) Serial.print(F("instruction, "));
  if (servoAlarmShutdownTriggers & DynamixelBus::kOverloadError) Serial.print(F("overload, "));
  if (servoAlarmShutdownTriggers & DynamixelBus::kChecksumError) Serial.print(F("checksum, "));
  if (servoAlarmShutdownTriggers & DynamixelBus::kRangeError) Serial.print(F("range, "));
  if (servoAlarmShutdownTriggers & DynamixelBus::kTemperatureLimitError) Serial.print(F("temperature, "));
  if (servoAlarmShutdownTriggers & DynamixelBus::kAngleLimitError) Serial.print(F("angle, "));
  if (servoAlarmShutdownTriggers & DynamixelBus::kVoltageLimitError) Serial.print(F("voltage"));

  Serial.println();
  Serial.print(F(": Torque Enable = "));
  Serial.println(servoTorqueEnable ? F("true") : F("false"));
  Serial.print(F(": Compliance Margins: cw = "));
  Serial.print(servoCWComplianceMargin);
  Serial.print(F(" ccw = "));
  Serial.println(servoCCWComplianceMargin);
  Serial.print(F(": Compliance Slopes: cw = "));
  Serial.print(servoCWComplianceSlope);
  Serial.print(F(" ccw = "));
  Serial.println(servoCCWComplianceSlope);
  Serial.print(F(": Goal Position = "));
  Serial.print(servoGoalPosition);
  Serial.println(F("/1023"));
  Serial.print(F(": Goal Speed = "));
  Serial.print(servoGoalSpeed);
  Serial.println(F("/1023"));
  Serial.print(F(": Torque Limit (RAM) = "));
  Serial.print(servoTorqueLimit);
  Serial.println(F("/1023"));
  Serial.print(F(": Present Position = "));
  Serial.println(servoPresentPosition);
  Serial.print(F(": Present Speed = "));
  Serial.println(servoPresentSpeed);
  Serial.print(F(": Present Load = "));
  Serial.print(servoPresentLoad);
  Serial.print(F(" direction = "));
  Serial.println(servoPresentLoadDirection);
  Serial.print(F(": Present Voltage (Mv) = "));
  Serial.println(servoPresentVoltage);
  Serial.print(F(": Present Temperature (C) = "));
  Serial.println(servoPresentTemperature);
  Serial.print(F(": Lock = "));
  Serial.println(servoLock ? F("true") : F("false"));
  Serial.print(F(": Punch = "));
  Serial.print(servoPunch);
  Serial.println(F("/1023"));
  Serial.println();

}

void setup() {

  delay(1000);
  
  Serial.begin(115200);
  while(Serial.read() == -1);

  Serial.println("DynamixelBus Servo Discovery\n");

  dynamixel.begin(DynamixelBus::k1000000);
  
  // Must call this method to update internal state even
  // if the initial baud rate is incorrect as the DynamixelBus
  // initially assumes a state of kRespondToNone.
  dynamixel.setStatusReturnLevel(0xFE,DynamixelBus::kRespondToAll);
  dynamixel.setMaxRetriesOnError(4);

  // Flush input so we don't immediately repeat the discovery process
  while(Serial.read() != -1);
}

bool doAgain = true;

void loop() {

  size_t servoCount = 0;
  uint8_t servoIds[CONNECTED_SERVO_COUNT];

  Serial.println("Starting servo discovery...");
  servoCount = dynamixel.discover(servoIds,CONNECTED_SERVO_COUNT,MAX_SERVO_ID);
  if (servoCount == 0) {
    Serial.println("NO SERVOS WERE FOUND!");
  }
  else {
    Serial.println();
    for (size_t i=0; i < servoCount; i++) {
      report(servoIds,i);
    }
    Serial.println("Servo discovery complete.");
  }
  Serial.println("Press a key to try again...");
  while(Serial.read() == -1);

}
