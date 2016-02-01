
#include "DynamixelBus.h"

const DynamixelBus::BaudRate DynamixelBus::sBaudRates[10] = {  kUnknown, k9600,k19200,k57600,k115200,k200000,k250000,k400000,k500000,k1000000 };

DynamixelBus::DynamixelBus(HardwareSerial& serial)
: mSerial(serial)
, mpServoState(new ServoState[255])
, mReadTimeoutMs(18)
, mMaxRetriesOnError(1)
{
  
}

DynamixelBus::~DynamixelBus() {
  delete[] mpServoState;
}

void DynamixelBus::putInstructionPacket(uint8_t* pPayload, size_t payloadLength) {
  //Serial.println("-> putInstruction");
  uint16_t checksum = 0;
  while(mSerial.read() != -1);
  mSerial.setDirection(kTransmitOnly);
  mSerial.write(0xFF);
  mSerial.write(0xFF);
  for (size_t i = 0; i < payloadLength; i++) {
    uint8_t byte = pPayload[i];
    //Serial.print(byte);
    //Serial.print(" ");
    mSerial.write(byte);
    checksum += byte;
    //mSerial.flush();
  }
  checksum = (~checksum) & 0xFF;
  mSerial.writeLast(checksum);
  //Serial.println(" <- putInstruction");
}

uint8_t DynamixelBus::getStatusPacket(uint8_t* pPayload, size_t payloadLength) {
  //Serial.println("-> getStatusPacket");
  uint8_t checksum = 0;
  unsigned long timeout;
  bool isTimeout = false;
  size_t count = 0;
  size_t length = payloadLength + 3;
  uint8_t* ptr = pPayload;
  while (count < length && !isTimeout) {
    timeout = millis() + mReadTimeoutMs;
    while (!mSerial.available()) {
      isTimeout = (millis() > timeout); 
      if (isTimeout) {
        return kReadTimeOutError;
        break;
      }
    }
    uint8_t byte = mSerial.read();
    if (count == 0 || count == 1) {
      if (byte != 0xFF) {
        return kReadFramingError;
      }
      count++;
      continue;
    }
    count++;
    if (count == length) {
      if (((~checksum) & 0xFF) != byte) {
        return kReadChecksumError;
      }
    }
    else {
      *ptr++ = byte;
      checksum += byte;
    }
  }
  //Serial.println("<- getStatusPacket");
  return pPayload[2];
}

uint8_t DynamixelBus::getStatusPacket() {
  //Serial.println("-> getStatusPacket");
  uint8_t checksum = 0;
  unsigned long timeout;
  bool isTimeout = false;
  size_t count = 0;
  uint8_t error = 0;
  while (count < 6 && !isTimeout) {
    timeout = millis() + mReadTimeoutMs;
    while (!mSerial.available()) {
      isTimeout = (millis() > timeout); 
      if (isTimeout) {
        return kReadTimeOutError;
        break;
      }
    }
    uint8_t byte = mSerial.read();
    if (count == 0 || count == 1) {
      if (byte != 0xFF) {
        return kReadFramingError;
      }
      count++;
      continue;
    }
    if (count == 4) {
      error = byte;
    }
    count++;
    if (count == 6) {
      if (((~checksum) & 0xFF) != byte) {
        return kReadChecksumError;
      }
    }
    else {
      checksum += byte;
    }
  }
  //Serial.println("<- getStatusPacket");
  return error;
}

uint8_t DynamixelBus::writeDataB(uint8_t servoId, uint8_t address, bool value) {
  return writeData8(servoId, address, value ? 1 : 0);
}

uint8_t DynamixelBus::writeData8(uint8_t servoId, uint8_t address, uint8_t value) {
  static uint8_t out[] = { 0, 4, kWriteDataInstruction, 0, 0 };
  out[0] = servoId;
  out[3] = address;
  out[4] = value;
  putInstructionPacket(out,sizeof(out));
  return checkWriteStatus(servoId) ? getStatusPacket() : 0;
}

uint8_t DynamixelBus::writeData16(uint8_t servoId, uint8_t address, uint16_t value) {
  static uint8_t out[] = { 0, 5, kWriteDataInstruction, 0, 0, 0 };
  out[0] = servoId;
  out[3] = address;
  out[4] = value & 0xFF;
  out[5] = (value >> 8) & 0x3;
  putInstructionPacket(out,sizeof(out));
  return checkWriteStatus(servoId) ? getStatusPacket() : 0;
}

uint8_t DynamixelBus::writeData8(uint8_t servoId, uint8_t address, uint8_t value1, uint8_t value2) {
  static uint8_t out[] = { 0, 5, kWriteDataInstruction, 0, 0, 0 };
  out[0] = servoId;
  out[3] = address;
  out[4] = value1;
  out[5] = value2;
  putInstructionPacket(out,sizeof(out));
  return checkWriteStatus(servoId) ? getStatusPacket() : 0;
}

uint8_t DynamixelBus::writeData16(uint8_t servoId, uint8_t address, uint16_t value1, uint16_t value2) {
  static uint8_t out[] = { 0, 7, kWriteDataInstruction, 0, 0, 0, 0, 0 };
  out[0] = servoId;
  out[3] = address;
  out[4] = value1 & 0xFF;
  out[5] = (value1 >> 8) & 0x3;
  out[6] = value2 & 0xFF;
  out[7] = (value2 >> 8) & 0x3;
  putInstructionPacket(out,sizeof(out));
  return checkWriteStatus(servoId) ? getStatusPacket() : 0;
}

uint8_t DynamixelBus::readDataB(uint8_t servoId, uint8_t address, bool& value) {
  uint8_t v;
  uint8_t error = readData8(servoId,address,v);
  value = v != 0;
  return error;  
}

uint8_t DynamixelBus::readData8(uint8_t servoId, uint8_t address, uint8_t& value) {
  static uint8_t out[] = { 0, 4, kReadDataInstruction, 0, 1 };
  static uint8_t in[4];
  if (servoId == 0xFE) return 0;
  out[0] = servoId;
  out[3] = address;
  putInstructionPacket(out,sizeof(out));
  uint8_t error = 0;
  if (checkReadStatus(servoId) && (error = getStatusPacket(in,sizeof(in))) == 0) {
    value = in[3];
  }
  return error;
}

uint8_t DynamixelBus::readData16(uint8_t servoId, uint8_t address, uint16_t& value) {
  static uint8_t out[] = { 0, 4, kReadDataInstruction, 0, 2 };
  static uint8_t in[5];
  if (servoId == 0xFE) return 0;
  out[0] = servoId;
  out[3] = address;
  putInstructionPacket(out,sizeof(out));
  uint8_t error = 0;
  if (checkReadStatus(servoId) && (error = getStatusPacket(in,sizeof(in))) == 0) {
    value = in[3] | ((in[4] & 0x3) << 8);
  }
  return error;
}

uint8_t DynamixelBus::readData8(uint8_t servoId, uint8_t address, uint8_t& value1, uint8_t& value2) {
  static uint8_t out[] = { 0, 4, kReadDataInstruction, 0, 2 };
  static uint8_t in[5];
  if (servoId == 0xFE) return 0;
  out[0] = servoId;
  out[3] = address;
  putInstructionPacket(out,sizeof(out));
  uint8_t error = 0;
  if (checkReadStatus(servoId) && (error = getStatusPacket(in,sizeof(in))) == 0) {
    value1 = in[3];
    value2 = in[4];
  }
  return error;
}

uint8_t DynamixelBus::readData16(uint8_t servoId, uint8_t address, uint16_t& value1, uint16_t& value2) {
  static uint8_t out[] = { 0, 4, kReadDataInstruction, 0, 4 };
  static uint8_t in[7];
  if (servoId == 0xFE) return 0;
  out[0] = servoId;
  out[3] = address;
  putInstructionPacket(out,sizeof(out));
  uint8_t error = 0;
  if (checkReadStatus(servoId) && (error = getStatusPacket(in,sizeof(in))) == 0) {
    value1 = in[3] | ((in[4] & 0x3) << 8);
    value2 = in[5] | ((in[6] & 0x3) << 8);
  }
  return error;
}

uint8_t DynamixelBus::ping(uint8_t servoId, size_t retries) {
  static uint8_t out[] = { 0, 2, kPingInstruction };
  out[0] = servoId;
  uint8_t error = 0;
  if (checkReadStatus(servoId)) {
    for (size_t i = 0; i <= retries; i++) {
      putInstructionPacket(out,sizeof(out));
      error = getStatusPacket();
      if (!error || (error & kReadError == 0)) return error;
    }
  }
  else {
    putInstructionPacket(out,sizeof(out));
  }
  return error;
}

uint8_t DynamixelBus::reset(uint8_t servoId) {
  static uint8_t out[] = { 0, 2, kResetInstruction };
  out[0] = servoId;
  putInstructionPacket(out,sizeof(out));
  return checkWriteStatus(servoId) ? getStatusPacket() : 0;
}

size_t DynamixelBus::discover(uint8_t* pIds, size_t servoCount, uint8_t maxId) {
  size_t count = 0;
  for (size_t servoId = 0; servoId < 254; servoId++) {
    mpServoState[servoId].mBaudRateIndex = 0;
  }
  for (size_t baudRateIndex=1; baudRateIndex < 10 && count < servoCount; baudRateIndex++) {
    BaudRate baud = sBaudRates[baudRateIndex];
    mSerial.begin(baud);
    for (size_t servoId=0; servoId<maxId && count < servoCount; servoId++) {
      if (mpServoState[servoId].mBaudRateIndex == 0) {
        if (ping(servoId) == 0) {
          mpServoState[servoId].mBaudRateIndex = baudRateIndex;
          if (pIds) pIds[count++] = servoId;
        }
      }
    }
  }
  return count;
}

uint8_t DynamixelBus::getModelNumber(uint8_t servoId, uint16_t& modelNumber) {
  return readData16(servoId, kServoModelNumberL, modelNumber);
}

uint8_t DynamixelBus::getFirmwareVersion(uint8_t servoId, uint8_t& version) {
  return readData8(servoId, kServoFirmwareVersion, version);
}

uint8_t DynamixelBus::setId(uint8_t servoId, uint8_t newId) {
  return writeData8(servoId, kServoId, newId);
}

uint8_t DynamixelBus::getId(uint8_t servoId, uint8_t& id) {
  return readData8(servoId, kServoId, id);
}

uint8_t DynamixelBus::setBaudRate(BaudRate baudRate) {
  BaudRate last = kUnknown;
  for (size_t servoId = 0; servoId < 253; servoId++) {
    BaudRate baud = sBaudRates[mpServoState[servoId].mBaudRateIndex];
    if (baud == kUnknown) continue;
    if (baud != last) {
      mSerial.begin(baud);
      last = baud;
    }
    setBaudRate(servoId,baudRate);
  }
  mSerial.begin(baudRate);
}

uint8_t DynamixelBus::setBaudRate(uint8_t servoId, BaudRate baudRate) {
  uint8_t baud = ((uint32_t)2000000 - (uint32_t)baudRate) / (uint32_t)baudRate;
  uint8_t error = writeData8(servoId, kServoBaudRate, baud);
  if (!error) {
    size_t index;
    for (index = 1; index < 10; index++) {
      if (sBaudRates[index] == baudRate) {
        mpServoState[servoId].mBaudRateIndex = index;
        return 0;
        break;
      }
    }
  }
}

uint8_t DynamixelBus::getBaudRate(uint8_t servoId, BaudRate& baudRate) {
  baudRate = sBaudRates[mpServoState[servoId].mBaudRateIndex];
  return 0;
}

uint8_t DynamixelBus::setReturnDelayTime(uint8_t servoId, uint16_t usec) {
  return writeData8(servoId, kServoReturnDelayTime, usec / 2);
}

uint8_t DynamixelBus::getReturnDelayTime(uint8_t servoId, uint16_t& usec) {
  uint8_t value;
  uint8_t error = readData8(servoId, kServoReturnDelayTime, value);
  if (!error) usec = value * 2;
  return error;
}

uint8_t DynamixelBus::setAngleLimits(uint8_t servoId, uint16_t cwLimit, uint16_t ccwLimit) {
  return writeData16(servoId, kServoCWAngleLimitL, cwLimit, ccwLimit);
}

uint8_t DynamixelBus::getAngleLimits(uint8_t servoId, uint16_t& cwLimit, uint16_t& ccwLimit) {
  return readData16(servoId, kServoCWAngleLimitL, cwLimit, ccwLimit);
}

uint8_t DynamixelBus::setTemperatureLimit(uint8_t servoId, uint8_t celsius) {
  return writeData8(servoId, kServoTemperatureLimit, celsius);
}

uint8_t DynamixelBus::getTemperatureLimit(uint8_t servoId, uint8_t& celsius) {
  return readData8(servoId, kServoTemperatureLimit, celsius);
}

uint8_t DynamixelBus::setVoltageLimits(uint8_t servoId, uint8_t lowMv, uint8_t highMv) {
  return writeData8(servoId, kServoLowVoltageLimit, lowMv, highMv);
}

uint8_t DynamixelBus::getVoltageLimits(uint8_t servoId, uint8_t& lowMv, uint8_t& highMv) {
  return readData8(servoId, kServoLowVoltageLimit, lowMv, highMv);
}

uint8_t DynamixelBus::setInitialTorqueLimit(uint8_t servoId, uint16_t limit) {
  return writeData16(servoId, kServoInitialTorqueLimitL, limit);
}

uint8_t DynamixelBus::getInitialTorqueLimit(uint8_t servoId, uint16_t& limit) {
  return readData16(servoId, kServoInitialTorqueLimitL, limit);
}

uint8_t DynamixelBus::setStatusReturnLevel(uint8_t servoId, uint8_t level) {
  if (servoId == 0xFE) {
    for(size_t i=0; i < 0xFF; i++) mpServoState[i].mStatusReturnLevel = level;
  }
  else {
    mpServoState[servoId].mStatusReturnLevel = level;
  }
  return writeData8(servoId, kServoStatusReturnLevel, level);
}

uint8_t DynamixelBus::getStatusReturnLevel(uint8_t servoId, uint8_t& level) {
  uint8_t error = readData8(servoId, kServoStatusReturnLevel, level);
  if (error == 0 && servoId != 0xFE) {
    mpServoState[servoId].mStatusReturnLevel = level;
  }
  return error;
}

uint8_t DynamixelBus::setAlarmLedTriggers(uint8_t servoId, uint8_t bitmask) {
  return writeData8(servoId, kServoAlarmLed, bitmask);
}

uint8_t DynamixelBus::getAlarmLedTriggers(uint8_t servoId, uint8_t& bitmask) {
  return readData8(servoId, kServoAlarmLed, bitmask);
}

uint8_t DynamixelBus::setAlarmShutdownTriggers(uint8_t servoId, uint8_t bitmask) {
  return writeData8(servoId, kServoAlarmShutdown, bitmask);
}

uint8_t DynamixelBus::getAlarmShutdownTriggers(uint8_t servoId, uint8_t& bitmask) {
  return readData8(servoId, kServoAlarmShutdown, bitmask);
}

uint8_t DynamixelBus::setTorqueEnable(uint8_t servoId, bool enable) {
  return writeDataB(servoId, kServoTorqueEnable, enable);
}

uint8_t DynamixelBus::getTorqueEnable(uint8_t servoId, bool& enable) {
  return readDataB(servoId, kServoTorqueEnable, enable);
}

uint8_t DynamixelBus::setLedState(uint8_t servoId, bool on) {
  return writeDataB(servoId, kServoLed, on);
}

uint8_t DynamixelBus::getLedState(uint8_t servoId, bool& isOn) {
  return readDataB(servoId, kServoLed, isOn);
}

uint8_t DynamixelBus::setComplianceMargin(uint8_t servoId, uint8_t margin) {
  return setComplianceMargins(servoId,margin,margin);
}

uint8_t DynamixelBus::setComplianceMargins(uint8_t servoId, uint8_t cwMargin, uint8_t ccwMargin) {
  return writeData8(servoId, kServoCWComplianceMargin, cwMargin, ccwMargin);
}

uint8_t DynamixelBus::getComplianceMargins(uint8_t servoId, uint8_t& cwMargin, uint8_t& ccwMargin) {
  return readData8(servoId, kServoCWComplianceMargin, cwMargin, ccwMargin);
}

uint8_t DynamixelBus::setComplianceSlope(uint8_t servoId, uint8_t slope) {
  return setComplianceSlopes(servoId,slope,slope);
}

uint8_t DynamixelBus::setComplianceSlopes(uint8_t servoId, uint8_t cwSlope, uint8_t ccwSlope) {
  return writeData8(servoId, kServoCWComplianceSlope, cwSlope, ccwSlope);
}

uint8_t DynamixelBus::getComplianceSlopes(uint8_t servoId, uint8_t& cwSlope, uint8_t& ccwSlope) {
  return readData8(servoId, kServoCWComplianceSlope, cwSlope, ccwSlope);
}

uint8_t DynamixelBus::setGoalPosition(uint8_t servoId, uint16_t position) {
  return writeData16(servoId, kServoGoalPositionL, position);
}

uint8_t DynamixelBus::getGoalPosition(uint8_t servoId, uint16_t& position) {
  return readData16(servoId, kServoGoalPositionL, position);
}

uint8_t DynamixelBus::setGoalSpeed(uint8_t servoId, uint16_t speed) {
  return writeData16(servoId, kServoMovingSpeedL, speed);
}

uint8_t DynamixelBus::getGoalSpeed(uint8_t servoId, uint16_t& speed) {
  return readData16(servoId, kServoMovingSpeedL, speed);
}

uint8_t DynamixelBus::setEndlessTurn(uint8_t servoId, uint8_t direction, uint16_t speed) {
  uint8_t error = setAngleLimits(servoId, 0, 0);
  if (!error) {
    uint16_t value = (direction == kClockwise ? 0x400 : 0x000) | (speed & 0x3FF);
    error = setGoalSpeed(servoId,value);   
  }
  return error;
}

uint8_t DynamixelBus::getEndlessTurn(uint8_t servoId, uint8_t& direction, uint16_t& speed) {
  uint16_t cwLimit = 0;
  uint16_t ccwLimit = 0;
  uint8_t error = getAngleLimits(servoId, cwLimit, ccwLimit);
  if (!error) {
    if (cwLimit == 0 && ccwLimit == 0) {
      uint16_t value;
      error = getGoalSpeed(servoId,value);   
      if (!error) {
        direction = value & 0x400 ? kClockwise : kCounterClockwise;
        speed = value & 0x3FF;
      }
    }
    else {
      direction = 0;
      speed = 0;
    }
  }
  return error;
}

uint8_t DynamixelBus::setTorqueLimit(uint8_t servoId, uint16_t torque) {
  return writeData16(servoId, kServoTorqueLimitL, torque);  
}

uint8_t DynamixelBus::getTorqueLimit(uint8_t servoId, uint16_t& torque) {
  return readData16(servoId, kServoTorqueLimitL, torque);  
}


uint8_t DynamixelBus::getPresentPosition(uint8_t servoId, uint16_t& position) {
  return readData16(servoId, kServoPresentPositionL, position);
}

uint8_t DynamixelBus::getPresentSpeed(uint8_t servoId, uint16_t& speed) {
  return readData16(servoId, kServoPresentSpeedL, speed);
}


uint8_t DynamixelBus::getPresentLoad(uint8_t servoId, uint8_t& direction, uint16_t& load) {
  char buf[20];
  uint16_t value;
  uint8_t error = readData16(servoId, kServoPresentLoadL, value);
  if (!error) {
    direction = value & 0x400 ? kClockwise : kCounterClockwise;
    load = value & 0x3FF;
  }
  return error;
}

uint8_t DynamixelBus::getPresentVoltage(uint8_t servoId, uint16_t& voltageMv) {
  uint8_t value = 0;
  uint8_t error = readData8(servoId, kServoPresentVoltage, value);
  if (!error) {
    voltageMv = value * 100;
  }
  return error;
}

uint8_t DynamixelBus::getPresentTemperature(uint8_t servoId, uint8_t& celsius) {
  return readData8(servoId, kServoPresentTemperature, celsius);
}

uint8_t DynamixelBus::getMoving(uint8_t servoId, bool& isMoving) {
  return readDataB(servoId, kServoMoving, isMoving);
}

uint8_t DynamixelBus::setLock(uint8_t servoId, bool lock) {
  return writeDataB(servoId, kServoLock, lock);
}

uint8_t DynamixelBus::getLock(uint8_t servoId, bool& isLocked) {
  return readDataB(servoId, kServoLock, isLocked);
}

uint8_t DynamixelBus::setPunch(uint8_t servoId, uint16_t currentLevel) {
  return writeData16(servoId, kServoPunchL, currentLevel);
}

uint8_t DynamixelBus::getPunch(uint8_t servoId, uint16_t& currentLevel) {
  return readData16(servoId, kServoPunchL, currentLevel);
}


