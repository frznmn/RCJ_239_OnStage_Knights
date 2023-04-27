#include <SmoothServoDriver.h>

void PCA9685SmoothServo::attach(int _pinServo, int _minPulseWidth, int _maxPulseWidth, int _zero, float _k, int _servoPosition = 0, int _maxAngle = 180, int _FREQUENCY = 50, uint8_t addr = 0x40) {
  pinServo = _pinServo;
  minPulseWidth = _minPulseWidth;
  maxPulseWidth = _maxPulseWidth;
  servoPosition = _servoPosition;
  zero = _zero;
  k = _k;
  FREQUENCY = _FREQUENCY;
  maxAngle = _maxAngle;
  sservo._i2caddr = addr;
  sservo.begin();
  sservo.setPWMFreq(FREQUENCY);
  sservo.setPWM(pinServo, 0, float(map(servoPosition * k + zero, 0, maxAngle, minPulseWidth, maxPulseWidth)) * FREQUENCY * 64.0 / 15625.0);
  //Serial.println(servoPosition);
  attached = true;
}

bool PCA9685SmoothServo::toPosition(int newPosition) {
  if(!attached) {
	  Serial.println("haven't attached");
	  return false;
  }
  servoPosition += myDelta * sgn(servoDelta);
  servoDelta = newPosition - servoPosition;
  myDelta = 0;
  myDelta0 = float(vmax * vmax) / float(accelmax * 2);
  if (abs(servoDelta) < 2.0 * myDelta0) {
    _vmax = sqrt(float(abs(servoDelta)) * float(accelmax));
  } else {
    _vmax = vmax;
  }
  _accelmax = accelmax;
  timeToPosition = float(abs(servoDelta)) / _vmax + _vmax / float(_accelmax);
  myDelta0 = _vmax * _vmax / float(_accelmax * 2);
  myDelta1 = (float(timeToPosition) - 2.0 * _vmax / float(_accelmax)) * _vmax;
  inPosition = false;
  myTimer = micros();
  return true;
}

bool PCA9685SmoothServo::tick() {
  if(!attached) {
	Serial.println("haven't attached");
	return false;
  }
  float nowTime = float(micros() - myTimer) / 1000000.0;
  if (nowTime > timeToPosition) {
    servoPosition += servoDelta;
    servoDelta = 0;
    myDelta = 0;
	if(inPosition == false) {
		sservo.setPWM(pinServo, 0, float(map(float(servoPosition) * k + zero, 0, maxAngle, minPulseWidth, maxPulseWidth)) * FREQUENCY * 64.0 / 15625.0);
		//Serial.println(servoPosition);
		inPosition = true;
	}
    return true;
  } else {
    float vnow = 0;
    if (nowTime < _vmax / float(_accelmax)) {
      vnow = float(_accelmax) * nowTime;
      myDelta = float(vnow * vnow) / float(_accelmax * 2.0);
    } else {
      if (nowTime < timeToPosition - _vmax / float(_accelmax)) {
        vnow = _vmax;
        myDelta = myDelta0 + (nowTime - _vmax / float(_accelmax)) * _vmax;
      } else {
        vnow = _vmax - float(_accelmax * (nowTime + _vmax / float(_accelmax) - timeToPosition));
        myDelta = 2.0 * myDelta0 + myDelta1 - float(vnow * vnow) / float(_accelmax * 2);
      }
    }
    sservo.setPWM(pinServo, 0, float(map(float(servoPosition + myDelta * sgn(servoDelta)) * k + zero, 0, maxAngle, minPulseWidth, maxPulseWidth)) * FREQUENCY * 64.0 / 15625.0);
    //Serial.println(servoPosition + myDelta * sgn(servoDelta));
    return false;
  }
}

int PCA9685SmoothServo::sgn(float a) {
  if(a > 0) return 1;
  if(a < 0) return -1;
  return 0;
}

void PCA9685SmoothServo::setV(int _v) {
  vmax = _v;
}

void PCA9685SmoothServo::setAccel(int _accel) {
  accelmax = _accel;
}

int PCA9685SmoothServo::getPosition() {
  return servoPosition + myDelta * sgn(servoDelta);
}