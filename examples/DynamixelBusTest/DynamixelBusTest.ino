#include <HalfDuplexHardwareSerial.h>

/**
 * Unit tests for DynamixelBus class.
 * 
 */

#include <DynamixelBus.h>

#include <ArduinoUnit.h>

DynamixelBus dynamixel(Serial1);

/*
 * CAUTION!
 * These tests will run using the first servo found using the discovery method.
 * The servo limits, position, speed and torque limit values are modified
 * in many of the tests. The tests may be safely run using a freestanding
 * servo, but care must be taken when running the tests if the servo is attached
 * brackets or assemblies that restrict the movement of the servo.
 * The following macros may be used to establish limits for the test suite.
 * However, test failures may leave the servo configured in a state different from that
 * proir to executing the tests. You can try re-running the test suite to restore
 * the defaults given below if the test failures occurred as a result of
 * communication errors or alarm conditions.
 */
#define DEFAULT_MIN_ANGLE (512 - 64)
#define DEFAULT_MAX_ANGLE (512 + 64)
#define DEFAULT_TORQUE_LIMIT 256
#define DEFAULT_GOAL_POSITION 512
#define DEFAULT_GOAL_SPEED 256
#define DEFAULT_PUNCH 32

#define DEFAULT_MIN_VOLTAGE 6000
#define DEFAULT_MAX_VOLTAGE 14000

/*
 * Set these macros to TRUE or FALSE to control whether a test is included.
 * Each test reduces the amount of memory for program execution,
 * so you will generally need to disable most tests to successfully 
 * compile the sketch while also leaving enough RAM to run the tests.
*/
#define TRUE 1
#define FALSE 0

#define TEST_PING                   FALSE
#define TEST_MODELNUMBER_AX12       FALSE
#define TEST_FIRMWAREVERSION_AX12   FALSE
#define TEST_DISCOVER               FALSE
#define TEST_BAUD_RATE              FALSE
#define TEST_RETURN_DELAY_TIME      FALSE
#define TEST_ANGLE_LIMITS           FAKSE
#define TEST_TEMPERATURE_LIMIT      FALSE
#define TEST_VOLTAGE_LIMITS         FALSE
#define TEST_INITIAL_TORQUE_LIMIT   FALSE
#define TEST_STATUS_RETURN_LEVEL    FALSE
#define TEST_ALARM_LED              FALSE
#define TEST_ALARM_SHUTDOWN         FALSE
#define TEST_TORQUE_ENABLE          FALSE
#define TEST_LED                    FALSE
#define TEST_COMPLIANCE_MARGINS     FALSE
#define TEST_COMPLIANCE_SLOPES      FALSE
#define TEST_GOAL_POSITION          FALSE
#define TEST_GOAL_SPEED             FALSE
#define TEST_TORQUE_LIMIT           FALSE
#define TEST_PRESENT_STATE          FALSE
#define TEST_REGISTERED_INSTRUCTION FALSE
#define TEST_MOVING                 FALSE
#define TEST_LOCK                   FALSE
#define TEST_PUNCH                  FALSE

// Global state variable required in multiple tests
uint8_t servoId = 0;
uint16_t servoMinAngle = DEFAULT_MIN_ANGLE; // +90
uint16_t servoMaxAngle = DEFAULT_MAX_ANGLE; // -90


void setup() {
  delay(1000);
  
  Serial.begin(115200);
  while(Serial.read() == -1);

  Serial.println("DynamixelBus Unit Tests\n");

  dynamixel.begin(DynamixelBus::k1000000);
  
  // Must call this method to update internal state even
  // if the initial baud rate is incorrect as the DynamixelBus
  // initially assumes a state of kRespondToNone.
  dynamixel.setStatusReturnLevel(0xFE,DynamixelBus::kRespondToAll);

  Serial.println("Starting servo discovery...\n");
  dynamixel.setMaxRetriesOnError(4);
  size_t count = dynamixel.discover(&servoId,1,3);
  if (count == 0 || servoId == 0) {
    Serial.println("### No tests can be run because no servos were found!");
    return;
  }
  Serial.println("Servo discovery complete.\n");

  // Bump up the retries to account for occasional bouts
  // communication errors that arise for some, as yet, unkown reason.
  dynamixel.setMaxRetriesOnError(8);

  // The discovery operation will have saved the servo's baud rate
  DynamixelBus::BaudRate baud;
  dynamixel.getBaudRate(servoId,baud);
 
  Serial.print(F("Servo #"));
  Serial.print(servoId);
  Serial.print(F(" baud="));
  Serial.println(baud);

  // Switch to the servo's baud rate
  dynamixel.begin(baud);

  // Uncomment the following two statements to change the servo baud rate
  // dynamixel.setBaudRate(servoId,DynamixelBus::k115200);
  // dynamixel.begin(DynamixelBus::k115200);

  // Instruct the servo to respond to all instructions for testing purposes
  dynamixel.setStatusReturnLevel(servoId,DynamixelBus::kRespondToAll);
  dynamixel.setLed(servoId,false);

  // Enable torque so the servo can move as required in some tests
  dynamixel.setTorqueLimit(servoId,DEFAULT_TORQUE_LIMIT);
  dynamixel.setAngleLimits(servoId,DEFAULT_MIN_ANGLE,DEFAULT_MAX_ANGLE);
  dynamixel.setGoalPosition(servoId,DEFAULT_GOAL_POSITION);
  dynamixel.setGoalSpeed(servoId,DEFAULT_GOAL_SPEED);
  dynamixel.setTorqueEnable(servoId,true);

  // Uncomment the following to reset triggers to desired values
  // dynamixel.setAlarmLedTriggers(servoId,DynamixelBus::kOverloadError | DynamixelBus::kTemperatureLimitError);
  // dynamixel.setAlarmShutdownTriggers(servoId,DynamixelBus::kOverloadError | DynamixelBus::kTemperatureLimitError);


  // Now capture the current state of the servo..
  // (No guarantee every one of these calls will succeed!)

  uint16_t servoModelNumber = 12;
  uint8_t servoFirmwareVersion = 0;
  uint16_t servoReturnDelayTime = 0;
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
  
  Serial.println(F("Executing tests using following servo:\n"));
  Serial.print(F("Servo ID = "));
  Serial.println(servoId);
  Serial.print(F(": Baud Rate index = "));
  Serial.println(baud);
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

  // Select tests (that are included by the preprocessor)
  Test::exclude("ModelNumber_*");
  switch (servoModelNumber) {
#if TEST_MODELNUMBER_AX12 == TRUE
    case 12: Test::include("ModelNumber_AX12"); break;
#endif
  }

}

#if TEST_PING == TRUE
test(Ping) {
  assertTrue(dynamixel.ping(servoId));
  assertFalse(dynamixel.ping(253));
}
#endif

#if TEST_MODELNUMBER_AX12 == TRUE
test(ModelNumberAX12) {
  uint16_t getModelNumber = 0;
  assertEqual(dynamixel.getModelNumber(servoId,getModelNumber),DynamixelBus::kOk);
  assertEqual(getModelNumber,12);
}
#endif

#if TEST_FIRMWAREVERSION_AX12 == TRUE
test(FirmwareVersion_AX12) {
  uint8_t getVersion = 0;
  assertEqual(dynamixel.getFirmwareVersion(servoId,getVersion),DynamixelBus::kOk);
  assertMore(getVersion,0);
}
#endif

#if TEST_DISCOVER == TRUE
test(Discover) {
  uint8_t id;
  size_t count = dynamixel.discover(&id,1,servoId+1);
  assertTrue(count > 0);
  assertEqual(servoId,id);
}
#endif

#if TEST_BAUD_RATE == TRUE
test(BaudRate) {
  DynamixelBus::BaudRate currentBaud = DynamixelBus::kUnknown;
  dynamixel.getBaudRate(servoId,currentBaud);
  DynamixelBus::BaudRate putBaud = DynamixelBus::k115200;
  if (currentBaud == putBaud) putBaud = DynamixelBus::k57600;
  assertEqual(dynamixel.setBaudRate(servoId,putBaud),DynamixelBus::kOk);
  DynamixelBus::BaudRate getBaud = DynamixelBus::kUnknown;
  uint8_t error = dynamixel.getBaudRate(servoId,getBaud);
  if (error) {
    dynamixel.setBaudRate(servoId,currentBaud);
    fail();
  }
  assertTrue(putBaud == getBaud);
  dynamixel.setBaudRate(servoId,currentBaud);
}
#endif

#if TEST_RETURN_DELAY_TIME == TRUE
test(ReturnDelayTime) {
  uint16_t currentReturnDelayTime = 0;
  assertEqual(dynamixel.getReturnDelayTime(servoId,currentReturnDelayTime),DynamixelBus::kOk);
  //Serial.print("current return delay time: ");
  //Serial.println(currentReturnDelayTime);
  uint16_t putReturnDelayTime = (currentReturnDelayTime + 256) / 2;
  assertEqual(dynamixel.setReturnDelayTime(servoId,putReturnDelayTime),DynamixelBus::kOk);
  //Serial.print("put return delay time: ");
  //Serial.println(putReturnDelayTime);
  uint16_t getReturnDelayTime = -1;
  assertEqual(dynamixel.getReturnDelayTime(servoId,getReturnDelayTime),DynamixelBus::kOk);
  //Serial.print("get return delay time: ");
  //Serial.println(getReturnDelayTime);
  assertTrue(putReturnDelayTime == getReturnDelayTime);
  assertEqual(dynamixel.setReturnDelayTime(servoId,currentReturnDelayTime),DynamixelBus::kOk);
}
#endif

#if TEST_ANGLE_LIMITS == TRUE
test(AngleLimits) 
{

  uint16_t currentMinAngleLimit = 0;
  uint16_t currentMaxAngleLimit = 0;

  assertEqual(dynamixel.getAngleLimits(servoId,currentMinAngleLimit,currentMaxAngleLimit),DynamixelBus::kOk);

  uint16_t offset = (currentMaxAngleLimit - currentMinAngleLimit) / 3;
  uint16_t putMinAngleLimit = currentMinAngleLimit + offset;
  uint16_t putMaxAngleLimit = currentMaxAngleLimit - offset;

  assertEqual(dynamixel.setAngleLimits(servoId,putMinAngleLimit,putMaxAngleLimit),DynamixelBus::kOk);

  uint16_t getMinAngleLimit = 0;
  uint16_t getMaxAngleLimit = 0;

  delay(18);

  assertEqual(dynamixel.getAngleLimits(servoId,getMinAngleLimit,getMaxAngleLimit),DynamixelBus::kOk);
  assertTrue(getMinAngleLimit == putMinAngleLimit);
  assertTrue(getMaxAngleLimit == putMaxAngleLimit);

  assertEqual(dynamixel.setAngleLimits(servoId,currentMinAngleLimit,currentMaxAngleLimit),DynamixelBus::kOk);
  
}
#endif

#if TEST_TEMPERATURE_LIMIT == TRUE
test(TemperatureLimit) {
  uint8_t currentTemperatureLimit = 0;
  assertEqual(dynamixel.getTemperatureLimit(servoId,currentTemperatureLimit),DynamixelBus::kOk);

  // This value will probaby trigger the temperature limit alarms if enabled
  uint8_t putTemperatureLimit = currentTemperatureLimit / 2;
  assertEqual(dynamixel.setTemperatureLimit(servoId,putTemperatureLimit),DynamixelBus::kOk);

  uint8_t getTemperatureLimit = 0;
  assertEqual(dynamixel.getTemperatureLimit(servoId,getTemperatureLimit),DynamixelBus::kOk);
  assertEqual(putTemperatureLimit,getTemperatureLimit);

  assertEqual(dynamixel.setTemperatureLimit(servoId,currentTemperatureLimit),DynamixelBus::kOk);
  
}
#endif

#if TEST_VOLTAGE_LIMITS == TRUE
test(VoltageLimits) {
  uint16_t currentMinVoltageLimit = 0;
  uint16_t currentMaxVoltageLimit = 0;

  assertEqual(dynamixel.getVoltageLimits(servoId,currentMinVoltageLimit,currentMaxVoltageLimit),DynamixelBus::kOk);

  uint16_t offset = (currentMaxVoltageLimit - currentMinVoltageLimit) / 3;
  uint16_t putMinVoltageLimit = ((currentMinVoltageLimit + offset) / 100) * 100;
  uint16_t putMaxVoltageLimit = ((currentMaxVoltageLimit - offset) / 100) * 100;

  assertEqual(dynamixel.setVoltageLimits(servoId,putMinVoltageLimit,putMaxVoltageLimit),DynamixelBus::kOk);

  uint16_t getMinVoltageLimit = 0;
  uint16_t getMaxVoltageLimit = 0;

  assertEqual(dynamixel.getVoltageLimits(servoId,getMinVoltageLimit,getMaxVoltageLimit),DynamixelBus::kOk);
  assertEqual(putMinVoltageLimit,getMinVoltageLimit);
  assertEqual(putMaxVoltageLimit,getMaxVoltageLimit);

  assertEqual(dynamixel.setVoltageLimits(servoId,DEFAULT_MIN_VOLTAGE,DEFAULT_MAX_VOLTAGE),DynamixelBus::kOk);
}
#endif

#if TEST_INITIAL_TORQUE_LIMIT == TRUE
test(InitialTorqueLimit) {
  uint16_t currentInitialTorqueLimit = 0;
  assertEqual(dynamixel.getInitialTorqueLimit(servoId,currentInitialTorqueLimit),DynamixelBus::kOk);

  uint16_t putInitialTorquLimit = currentInitialTorqueLimit / 2;
  assertEqual(dynamixel.setInitialTorqueLimit(servoId,putInitialTorquLimit),DynamixelBus::kOk);

  uint16_t getInitialTorquLimit = 0;
  assertEqual(dynamixel.getInitialTorqueLimit(servoId,getInitialTorquLimit),DynamixelBus::kOk);
  assertEqual(putInitialTorquLimit,getInitialTorquLimit);

  assertEqual(dynamixel.setInitialTorqueLimit(servoId,currentInitialTorqueLimit),DynamixelBus::kOk);
}
#endif

#if TEST_STATUS_RETURN_LEVEL == TRUE
test(StatusReturnLevel) 
{
  uint8_t error = 0;
  uint8_t currentStatusReturnLevel = DynamixelBus::kRespondToNone;
  uint8_t putStatusReturnLevel = DynamixelBus::kRespondToRead;
  uint8_t getStatusReturnLevel = DynamixelBus::kRespondToNone;

  assertEqual(dynamixel.getStatusReturnLevel(servoId,currentStatusReturnLevel),DynamixelBus::kOk);
  assertEqual(dynamixel.setStatusReturnLevel(servoId,putStatusReturnLevel),DynamixelBus::kOk);
  assertEqual(dynamixel.getStatusReturnLevel(servoId,getStatusReturnLevel),DynamixelBus::kOk);
  assertEqual(putStatusReturnLevel,getStatusReturnLevel);

  putStatusReturnLevel = DynamixelBus::kRespondToAll;
  getStatusReturnLevel = DynamixelBus::kRespondToNone;

  assertEqual(dynamixel.setStatusReturnLevel(servoId,currentStatusReturnLevel),DynamixelBus::kOk);
  assertEqual(dynamixel.getStatusReturnLevel(servoId,getStatusReturnLevel),DynamixelBus::kOk);
  assertEqual(getStatusReturnLevel,currentStatusReturnLevel);
}
#endif

#if TEST_ALARM_LED == TRUE
test(AlarmLed) {
  uint8_t currentAlarmLed = 0;
  uint8_t putAlarmLed = 0x7F;
  assertEqual(dynamixel.getAlarmLedTriggers(servoId,currentAlarmLed),DynamixelBus::kOk);
  assertEqual(dynamixel.setAlarmLedTriggers(servoId,putAlarmLed),DynamixelBus::kOk);
  uint8_t getAlarmLed = 0;
  assertEqual(dynamixel.getAlarmLedTriggers(servoId,getAlarmLed),DynamixelBus::kOk);
  assertEqual(putAlarmLed,getAlarmLed);
  assertEqual(dynamixel.setAlarmLedTriggers(servoId,currentAlarmLed),DynamixelBus::kOk);
}
#endif

#if TEST_ALARM_SHUTDOWN == TRUE
test(AlarmShutdown) {
  uint8_t currentAlarmShutdown = 0;
  uint8_t putAlarmShutdown = 0x7F;
  assertEqual(dynamixel.getAlarmShutdownTriggers(servoId,currentAlarmShutdown),DynamixelBus::kOk);
  assertEqual(dynamixel.setAlarmShutdownTriggers(servoId,putAlarmAlarmShutdown),DynamixelBus::kOk);
  uint8_t getAlarmShutdown = 0;
  assertEqual(dynamixel.getAlarmShutdownTriggers(servoId,getAlarmShutdown),DynamixelBus::kOk);
  assertEqual(putAlarmShutdown,getAlarmShutdown);
  assertEqual(dynamixel.setAlarmShutdownTriggers(servoId,currentAlarmShutdown),DynamixelBus::kOk);
}
#endif

#if TEST_TORQUE_ENABLE == TRUE
test(TorqueEnable) {
  assertEqual(dynamixel.setTorqueEnable(servoId,false),DynamixelBus::kOk);
  bool getTorqueEnable = true;
  assertEqual(dynamixel.getTorqueEnable(servoId,getTorqueEnable),DynamixelBus::kOk);
  assertTrue(getTorqueEnable);
  assertEqual(dynamixel.setTorqueEnable(servoId,true),DynamixelBus::kOk);
}
#endif

#if TEST_LED == TRUE
test(Led) {
  bool currentLed = false;
  assertEqual(dynamixel.getLed(servoId,currentLed), DynamixelBus::kOk);
  for (size_t i=0; i < 10; i++) {
    assertEqual(dynamixel.setLed(servoId,i & 0x1), DynamixelBus::kOk);
    delay(100);
  }
  assertEqual(dynamixel.setLed(servoId,!currentLed), DynamixelBus::kOk);
  bool getLed = false;
  assertEqual(dynamixel.getLed(servoId,getLed), DynamixelBus::kOk);
  assertTrue(currentLed != getLed);
  assertEqual(dynamixel.setLed(servoId,currentLed), DynamixelBus::kOk);
}
#endif

#if TEST_COMPLIANCE_MARGINS == TRUE
test(ComplianceMargins) {

  uint8_t currentCWComplianceMargin = 0;
  uint8_t currentCCWComplianceMargin = 0;
  assertEqual(dynamixel.getComplianceMargins(servoId,currentCWComplianceMargin,currentCCWComplianceMargin),DynamixelBus::kOk);

  uint8_t putCWComplianceMargin = 24;
  uint8_t putCCWComplianceMargin = 12;
  assertEqual(dynamixel.setComplianceMargins(servoId,putCWComplianceMargin,putCCWComplianceMargin),DynamixelBus::kOk);

  assertEqual(dynamixel.setTorqueLimit(servoId,0x3FF),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalSpeed(servoId,0x3FF),DynamixelBus::kOk);
  assertEqual(dynamixel.setPunch(servoId,0xFF),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalPosition(servoId,DEFAULT_GOAL_POSITION),DynamixelBus::kOk);
  delay(100);
  bool isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  uint16_t targetPosition = servoMinAngle; // + putCWComplianceMargin;
  
  assertEqual(dynamixel.setGoalPosition(servoId,targetPosition),DynamixelBus::kOk);
  delay(100);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  uint16_t actualPosition = 0;

  assertEqual(dynamixel.getPresentPosition(servoId,actualPosition),DynamixelBus::kOk);
  int offset = actualPosition - targetPosition;
  offset = abs(offset);
  assertLessOrEqual(offset,putCWComplianceMargin);

  targetPosition = servoMaxAngle;// - putCCWComplianceMargin;

  assertEqual(dynamixel.setGoalPosition(servoId,targetPosition),DynamixelBus::kOk);
  delay(100);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  assertEqual(dynamixel.getPresentPosition(servoId,actualPosition),DynamixelBus::kOk);
  offset = actualPosition - targetPosition;
  offset = abs(offset);
  assertLessOrEqual(offset,putCCWComplianceMargin);

  assertEqual(dynamixel.setTorqueLimit(servoId,DEFAULT_TORQUE_LIMIT),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalSpeed(servoId,DEFAULT_GOAL_SPEED),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalPosition(servoId,DEFAULT_GOAL_POSITION),DynamixelBus::kOk);
  assertEqual(dynamixel.setPunch(servoId,DEFAULT_PUNCH),DynamixelBus::kOk);
  
  delay(100);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  assertEqual(dynamixel.setComplianceMargins(servoId,currentCWComplianceMargin,currentCCWComplianceMargin),DynamixelBus::kOk);
}
#endif

#if TEST_COMPLIANCE_SLOPES == TRUE
test(ComplianceSlopes) {

  uint8_t currentCWComplianceSlope = 0;
  uint8_t currentCCWComplianceSlope = 0;
  assertEqual(dynamixel.getComplianceSlopes(servoId,currentCWComplianceSlope,currentCCWComplianceSlope),DynamixelBus::kOk);

  uint8_t putCWComplianceSlope = 0xFE;
  uint8_t putCCWComplianceSlope = 0xFE;
  assertEqual(dynamixel.setComplianceSlopes(servoId,putCWComplianceSlope,putCCWComplianceSlope),DynamixelBus::kOk);

  uint8_t getCWComplianceSlope = 0x0;
  uint8_t getCCWComplianceSlope = 0x0;
  assertEqual(dynamixel.getComplianceSlopes(servoId,getCWComplianceSlope,getCCWComplianceSlope),DynamixelBus::kOk);
  assertEqual(putCWComplianceSlope,getCWComplianceSlope);
  assertEqual(putCCWComplianceSlope,getCCWComplianceSlope);

  assertEqual(dynamixel.setTorqueLimit(servoId,0x3FF),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalSpeed(servoId,0x3FF),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalPosition(servoId,DEFAULT_GOAL_POSITION),DynamixelBus::kOk);
  delay(100);
  bool isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  uint16_t targetPosition = servoMinAngle;
  assertEqual(dynamixel.setGoalPosition(servoId,targetPosition),DynamixelBus::kOk);
  delay(100);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  targetPosition = servoMaxAngle;
  assertEqual(dynamixel.setGoalPosition(servoId,servoMaxAngle),DynamixelBus::kOk);
  delay(100);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  assertEqual(dynamixel.setTorqueLimit(servoId,DEFAULT_TORQUE_LIMIT),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalSpeed(servoId,DEFAULT_GOAL_SPEED),DynamixelBus::kOk);
  assertEqual(dynamixel.setGoalPosition(servoId,DEFAULT_GOAL_POSITION),DynamixelBus::kOk);
  delay(100);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  assertEqual(dynamixel.setComplianceSlopes(servoId,currentCWComplianceSlope,currentCCWComplianceSlope),DynamixelBus::kOk);
}
#endif

#if TEST_GOAL_POSITION == TRUE
test(GoalPosition) 
{
  uint16_t putPosition = servoMinAngle;
  uint16_t getPosition = 0;

  if (putPosition < servoMinAngle) putPosition = servoMinAngle;
  else if (putPosition > servoMaxAngle) putPosition = servoMaxAngle;

  assertEqual(dynamixel.setGoalPosition(servoId,putPosition),DynamixelBus::kOk);
  delay(100);
  bool isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  assertEqual(dynamixel.getGoalPosition(servoId,getPosition),DynamixelBus::kOk);
  assertEqual(putPosition,getPosition);

  assertEqual(dynamixel.setGoalPosition(servoId,512),DynamixelBus::kOk);
  delay(100);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }
}
#endif

#if TEST_GOAL_SPEED == TRUE
test(GoalSpeed) 
{
  uint16_t putSpeed = millis() & 0x3FF;
  uint16_t getSpeed = 0;

  assertEqual(dynamixel.setGoalSpeed(servoId,putSpeed),DynamixelBus::kOk);
  assertEqual(dynamixel.getGoalSpeed(servoId,getSpeed),DynamixelBus::kOk);
  assertEqual(putSpeed,getSpeed);
}
#endif

#if TEST_TORQUE_LIMIT == TRUE
test(TorqueLimit) {
  uint16_t currentTorqueLimit = 0;
  assertEqual(dynamixel.getTorqueLimit(servoId,currentTorqueLimit),DynamixelBus::kOk);

  uint16_t putTorquLimit = currentTorqueLimit / 2;
  assertEqual(dynamixel.setTorqueLimit(servoId,putTorquLimit),DynamixelBus::kOk);

  uint16_t getTorquLimit = 0;
  assertEqual(dynamixel.getTorqueLimit(servoId,getTorquLimit),DynamixelBus::kOk);
  assertEqual(putTorquLimit,getTorquLimit);

  assertEqual(dynamixel.setTorqueLimit(servoId,currentTorqueLimit),DynamixelBus::kOk);
}
#endif

#if TEST_PRESENT_STATE == TRUE
test(PresentState) {
  uint16_t servoPresentPosition = 0;
  uint16_t servoPresentSpeed = 0;
  uint16_t servoPresentLoad = 0;
  uint8_t servoPresentLoadDirection = 0;
  uint16_t servoPresentVoltage = 0;
  uint8_t servoPresentTemperature = 0;
  assertEqual(dynamixel.getPresentPosition(servoId,servoPresentPosition),DynamixelBus::kOk);
  assertEqual(dynamixel.getPresentSpeed(servoId,servoPresentSpeed),DynamixelBus::kOk);
  assertEqual(dynamixel.getPresentLoad(servoId,servoPresentLoadDirection,servoPresentLoad),DynamixelBus::kOk);
  assertEqual(dynamixel.getPresentVoltage(servoId,servoPresentVoltage),DynamixelBus::kOk);
  assertEqual(dynamixel.getPresentTemperature(servoId,servoPresentTemperature),DynamixelBus::kOk);
}
#endif

#if TEST_REGISTERED_INSTRUCTION == TRUE
test(RegisteredInstruction) {
  fail();
}
#endif

#if TEST_MOVING == TRUE
test(Moving) {

  bool isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }

  assertEqual(dynamixel.setGoalPosition(servoId,DEFAULT_MIN_ANGLE),DynamixelBus::kOk);
  delay(10);
  assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
  assertTrue(isMoving);
  delay(100);
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }
  
  assertEqual(dynamixel.setGoalPosition(servoId,DEFAULT_GOAL_POSITION),DynamixelBus::kOk);
  isMoving = true;
  while(isMoving) {
    assertEqual(dynamixel.getMoving(servoId,isMoving),DynamixelBus::kOk);
    delay(100);
  }
}
#endif

#if TEST_LOCK == TRUE
test(Lock) {
  
}
#endif

#if TEST_PUNCH == TRUE
test(Punch) {

  uint16_t currentPunch = 0;
  assertEqual(dynamixel.getPunch(servoId,currentPunch),DynamixelBus::kOk);

  uint16_t putPunch = DEFAULT_PUNCH / 2;
  assertEqual(dynamixel.setPunch(servoId,putPunch),DynamixelBus::kOk);
  
  uint16_t getPunch = 0;
  assertEqual(dynamixel.getPunch(servoId,getPunch),DynamixelBus::kOk);
  assertEqual(putPunch,getPunch);

  assertEqual(dynamixel.setPunch(servoId,DEFAULT_PUNCH),DynamixelBus::kOk);
}
#endif


void loop() {
  if (servoId != 0) Test::run();
}
