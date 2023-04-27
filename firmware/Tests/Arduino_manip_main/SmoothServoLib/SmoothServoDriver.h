#pragma once
#include <Arduino.h>
//#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

class PCA9685SmoothServo {
  public:
  Adafruit_PWMServoDriver sservo = Adafruit_PWMServoDriver();
  int pinServo, minPulseWidth, maxPulseWidth, zero, servoPosition = 0, servoDelta = 0, myDelta = 0, vmax = 150, accelmax = 300, maxAngle, FREQUENCY;
  uint8_t addr;
  uint32_t myTimer = 0;
  bool attached = false, inPosition = true;
  float k, myDelta0, myDelta1, _vmax, timeToPosition, _accelmax;
  void attach(int _pinServo, int _minPulseWidth, int _maxPulseWidth, int _zero, float _k, int _servoPosition = 0, int _maxAngle = 180, int _FREQUENCY = 50, uint8_t addr = 0x40);
  bool toPosition(int newPosition);
  bool tick();
  int getPosition();
  void setV(int _v);
  void setAccel(int _accel);
  int sgn(float a);
  private:
};