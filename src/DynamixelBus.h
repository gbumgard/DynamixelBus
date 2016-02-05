#ifndef _DYNAMIXELBUS_H_
#define _DYNAMIXELBUS_H_

#include "Arduino.h"
#include "HalfDuplexHardwareSerial.h"


/**
 * 
 */
class DynamixelBus {

public:

  static const uint8_t kRespondToNone;
  static const uint8_t kRespondToRead;
  static const uint8_t kRespondToAll;
  static const uint8_t kClockwise;
  static const uint8_t kCounterClockwise;

  static const uint8_t kOk;
  static const uint8_t kReadError;
  static const uint8_t kReadNotAllowedError;
  static const uint8_t kReadTimeoutError;
  static const uint8_t kReadFramingError;
  static const uint8_t kReadChecksumError;

  static const uint8_t kInstructionError;
  static const uint8_t kOverloadError;
  static const uint8_t kChecksumError;
  static const uint8_t kRangeError;
  static const uint8_t kTemperatureLimitError;
  static const uint8_t kAngleLimitError;
  static const uint8_t kVoltageLimitError;

  enum BaudRate { kUnknown, k9600, k19200, k57600, k115200, k200000, k250000, k400000, k500000, k1000000 };

  DynamixelBus(HardwareSerial& serial);

  virtual ~DynamixelBus();

  void begin(BaudRate baudRate);
  void begin(BaudRate baudRate, uint8_t config);

  void setReadTimeout(unsigned long msec) {
    mReadTimeoutMs = msec;
  }

  void setMaxRetriesOnError(size_t maxRetries) {
    mMaxRetriesOnError = maxRetries;
  }

  bool ping(uint8_t servoId);

  uint8_t reset(uint8_t servoId);

  size_t discover(uint8_t* pIds, size_t servoCount, uint8_t maxId = 253);
  
  uint8_t getModelNumber(uint8_t servoId, uint16_t& modelNumber);

  uint8_t getFirmwareVersion(uint8_t servoId, uint8_t& version);
 
  uint8_t setId(uint8_t servoId, uint8_t newId);
  uint8_t getId(uint8_t servoId, uint8_t& id);

  uint8_t setBaudRate(BaudRate baudRate = k1000000, size_t servoCount = 253, size_t maxServoId = 253);
  uint8_t setBaudRate(uint8_t servoId, BaudRate baudRate);
  uint8_t getBaudRate(uint8_t servoId, BaudRate& baudrate);

  uint8_t setReturnDelayTime(uint8_t servoId, uint16_t delay_us = 500);
  uint8_t getReturnDelayTime(uint8_t servoId, uint16_t& usec);

  uint8_t setAngleLimits(uint8_t servoId, uint16_t cwLimit = 0x000, uint16_t ccwLimit = 0x3FF);
  uint8_t getAngleLimits(uint8_t servoId, uint16_t& cwLimit, uint16_t& ccwLimit);

  uint8_t setTemperatureLimit(uint8_t servoId, uint8_t celsius = 85);
  uint8_t getTemperatureLimit(uint8_t servoId, uint8_t& celsius);

  uint8_t setVoltageLimits(uint8_t servoId, uint16_t lowMv = 6000, uint16_t highMv = 19000);
  uint8_t getVoltageLimits(uint8_t servoId, uint16_t& lowMv, uint16_t& highMv);

  uint8_t setInitialTorqueLimit(uint8_t servoId, uint16_t torque = 0x3FF);
  uint8_t getInitialTorqueLimit(uint8_t servoId, uint16_t& torque);

  uint8_t setStatusReturnLevel(uint8_t servoId, uint8_t level = kRespondToAll);
  uint8_t getStatusReturnLevel(uint8_t servoId, uint8_t& level);

  uint8_t setAlarmLedTriggers(uint8_t servoId, uint8_t bitmask = kTemperatureLimitError);
  uint8_t getAlarmLedTriggers(uint8_t servoId, uint8_t& bitmask);

  uint8_t setAlarmShutdownTriggers(uint8_t servoId, uint8_t bitmask = kTemperatureLimitError);
  uint8_t getAlarmShutdownTriggers(uint8_t servoId, uint8_t& bitmask);

  uint8_t setTorqueEnable(uint8_t servoId, bool enable = false);
  uint8_t getTorqueEnable(uint8_t servoId, bool& enable);

  uint8_t setLed(uint8_t servoId, bool on = false);
  uint8_t getLed(uint8_t servoId, bool& on);

  uint8_t setComplianceMargin(uint8_t servoId, uint8_t margin = 0x00);
  uint8_t setComplianceMargins(uint8_t servoId, uint8_t cwMargin = 0x00, uint8_t ccwMargin = 0x00);
  uint8_t getComplianceMargins(uint8_t servoId, uint8_t& cwMargin , uint8_t& ccwMargin);

  uint8_t setComplianceSlope(uint8_t servoId, uint8_t slope = 0x20);
  uint8_t setComplianceSlopes(uint8_t servoId, uint8_t cwSlope = 0x20, uint8_t ccwSlope = 0x20);
  uint8_t getComplianceSlopes(uint8_t servoId, uint8_t& cwSlope, uint8_t& ccwSlope);

  uint8_t setGoalPosition(uint8_t servoId, uint16_t position);
  uint8_t getGoalPosition(uint8_t servoId, uint16_t& position);

  uint8_t setGoalSpeed(uint8_t servoId, uint16_t speed = 0x00);
  uint8_t getGoalSpeed(uint8_t servoId, uint16_t& speed);

  uint8_t setEndlessTurn(uint8_t servoId, uint8_t direction, uint16_t speed);
  uint8_t getEndlessTurn(uint8_t servoId, uint8_t& direction, uint16_t& speed);

  uint8_t setTorqueLimit(uint8_t servoId, uint16_t torque);
  uint8_t getTorqueLimit(uint8_t servoId, uint16_t& torque);

  uint8_t getPresentPosition(uint8_t servoId, uint16_t& position);
  uint8_t getPresentSpeed(uint8_t servoId, uint16_t& speed);
  uint8_t getPresentLoad(uint8_t servoId, uint8_t& direction, uint16_t& load);
  uint8_t getPresentVoltage(uint8_t servoId, uint16_t& voltage);
  uint8_t getPresentTemperature(uint8_t servoId, uint8_t& temperature);
  uint8_t getMoving(uint8_t servoId, bool& isMoving);

  uint8_t setLock(uint8_t servoId, bool lock = false);
  uint8_t getLock(uint8_t servoId, bool& lock);

  uint8_t setPunch(uint8_t servoId, uint16_t currentLimit = 0x20);
  uint8_t getPunch(uint8_t servoId, uint16_t& currentLimit);
 
  static bool containsInstructionError(uint8_t error) {
    return error & kInstructionError;
  }

  static bool containsOverloadError(uint8_t error) {
    return error & kOverloadError;
  }

  static bool containsRangeError(uint8_t error) {
    return error & kRangeError;
  }

  static bool containsTemperatureLimitError(uint8_t error) {
    return error & kTemperatureLimitError;
  }

  static bool containsAngleLimitError(uint8_t error) {
    return error & kAngleLimitError;
  }

  static bool containsVoltageLimitError(uint8_t error) {
    return error & kVoltageLimitError;
  }

protected:
  
  static const uint8_t kPingInstruction            = 0x01;
  static const uint8_t kReadDataInstruction        = 0x02;
  static const uint8_t kWriteDataInstruction       = 0x03;
  static const uint8_t kRegWriteInstruction        = 0x04;
  static const uint8_t kActionInstruction          = 0x05;
  static const uint8_t kResetInstruction           = 0x06;
  static const uint8_t kSyncWriteInstruction       = 0x83;

  static const uint8_t kServoModelNumberL          = 0x00;
  static const uint8_t kServoModelNumberH          = 0x01;
  static const uint8_t kServoFirmwareVersion       = 0x02;
  static const uint8_t kServoId                    = 0x03;
  static const uint8_t kServoBaudRate              = 0x04;
  static const uint8_t kServoReturnDelayTime       = 0x05;
  static const uint8_t kServoCWAngleLimitL         = 0x06;
  static const uint8_t kServoCWAngleLimitH         = 0x07;
  static const uint8_t kServoCCWAngleLimitL        = 0x08;
  static const uint8_t kServoCCWAngleLimitH        = 0x09;
  static const uint8_t kServoTemperatureLimit      = 0x0B;
  static const uint8_t kServoLowVoltageLimit       = 0x0C;
  static const uint8_t kServoHighVoltageLimit      = 0x0D;
  static const uint8_t kServoInitialTorqueLimitL   = 0x0E;
  static const uint8_t kServoInitialTorqueLimitH   = 0x0F;
  static const uint8_t kServoStatusReturnLevel     = 0x10;
  static const uint8_t kServoAlarmLed              = 0x11;
  static const uint8_t kServoAlarmShutdown         = 0x12;
  static const uint8_t kServoTorqueEnable          = 0x18;
  static const uint8_t kServoLed                   = 0x19;
  static const uint8_t kServoCWComplianceMargin    = 0x1A;
  static const uint8_t kServoCCWComplianceMargin   = 0x1B;
  static const uint8_t kServoCWComplianceSlope     = 0x1C;
  static const uint8_t kServoCCWComplianceSlope    = 0x1D;
  static const uint8_t kServoGoalPositionL         = 0x1E;
  static const uint8_t kServoGoalPositionH         = 0x1F;
  static const uint8_t kServoMovingSpeedL          = 0x20;
  static const uint8_t kServoMovingSpeedH          = 0x21;
  static const uint8_t kServoTorqueLimitL          = 0x22;
  static const uint8_t kServoTorqueLimitH          = 0x23;
  static const uint8_t kServoPresentPositionL      = 0x24;
  static const uint8_t kServoPresentPositionH      = 0x25;
  static const uint8_t kServoPresentSpeedL         = 0x26;
  static const uint8_t kServoPresentSpeedH         = 0x27;
  static const uint8_t kServoPresentLoadL          = 0x28;
  static const uint8_t kServoPresentLoadH          = 0x29;
  static const uint8_t kServoPresentVoltage        = 0x2A;
  static const uint8_t kServoPresentTemperature    = 0x2B;
  static const uint8_t kServoRegisteredInstruction = 0x2C;
  static const uint8_t kServoMoving                = 0x2E;
  static const uint8_t kServoLock                  = 0x2F;
  static const uint8_t kServoPunchL                = 0x30;
  static const uint8_t kServoPunchH                = 0x31;

  bool checkWriteStatus(uint8_t servoId);
  bool checkReadStatus(uint8_t servoId);

  void putInstructionPacket(uint8_t* pPayload, size_t payloadLength);
  uint8_t getStatusPacket(uint8_t* pPayload, size_t payloadLength);
  uint8_t getStatusPacket();

  uint8_t writeData(uint8_t* pPayload, size_t payloadLength);
  uint8_t writeDataB(uint8_t servoId, uint8_t address, bool value);
  uint8_t writeData8(uint8_t servoId, uint8_t address, uint8_t value);
  uint8_t writeData16(uint8_t servoId, uint8_t address, uint16_t value);
  uint8_t writeData8(uint8_t servoId, uint8_t address, uint8_t value1, uint8_t value2);
  uint8_t writeData16(uint8_t servoId, uint8_t address, uint16_t value1, uint16_t value2);

  uint8_t readData(uint8_t* pPayloadOut, size_t outLength, uint8_t* pPayloadIn, size_t inLength);
  uint8_t readDataB(uint8_t servoId, uint8_t address, bool& value);
  uint8_t readData8(uint8_t servoId, uint8_t address, uint8_t& value);
  uint8_t readData16(uint8_t servoId, uint8_t address, uint16_t& value);
  uint8_t readData8(uint8_t servoId, uint8_t address, uint8_t& value1, uint8_t& value2);
  uint8_t readData16(uint8_t servoId, uint8_t address, uint16_t& value1, uint16_t& value2);

  HalfDuplexHardwareSerial mSerial;

  struct ServoState {
    unsigned mStatusReturnLevel : 2;
    BaudRate mBaudRate : 4;
  };

  ServoState* mpServoState;

  unsigned long mReadTimeoutMs;
  size_t mMaxRetriesOnError;

  static const uint32_t sSerialBaudRateMap[10];
  static const uint8_t sServoBaudRateMap[10];
};


#endif

