#include <Wire.h>                     //Library which allows I2C communication.
#include <Adafruit_PWMServoDriver.h>  //This library must be downloaded to run the code.

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  //Instantiating objects with the Adafruit_PWMServoDriver class.

int MIN_PULSE[5] = { 508, 464, 464, 411, 411};  //These are the minimum and maximum wavelength values which serve MG 995.
int MAX_PULSE[5] = { 2524, 2597, 2597, 2636, 2636};
int zero[5] = {0, 0, 0, 70, 0};
int FREQUENCY = 50;

int pulseWidth(int angle) {  //This function calculates servo's motion angle.
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE[3], MAX_PULSE[3]);  //This function get angle from 0 to 180 degrees and map from length minimum value to maximum.
  analog_value = int(float(pulse_wide) * FREQUENCY * 4096 / 1000000);
  Serial.println(analog_value);
  return analog_value;  //The value this function returns.
}

class SmoothServo {
  public:
  int pinServo, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, zero, servoPosition = 180, servoDelta = 0, myDelta = 0, vmax = 120, accelmax = 60;
  uint32_t myTimer = 0, tPos;
  float k;
  SmoothServo(int _pinServo, int _MIN_PULSE_WIDTH, int _MAX_PULSE_WIDTH, int _zero, float _k) {
    pinServo = _pinServo;
    MIN_PULSE_WIDTH = _MIN_PULSE_WIDTH;
    MAX_PULSE_WIDTH = _MAX_PULSE_WIDTH;
    zero = _zero;
    k = _k;
    pwm.setPWM(pinServo, 0, int(float(MAX_PULSE_WIDTH) * FREQUENCY * 4096.0 / 100000.0));
  }
  int toPosition(int newPosition) {
    servoPosition += myDelta;
    newPosition = float(newPosition) * k + zero;
    servoDelta = newPosition - servoPosition;
    myDelta = 0;
    tPos = (float(servoDelta) - 2.0 * float(vmax * vmax) / float(accelmax)) / float(vmax) * float(pow(10, 6));
    myTimer = micros();
  }
  bool tick() {
    if(micros() - myTimer > tPos) {
      servoPosition += servoDelta;
      servoDelta = 0;
      myDelta = 0;
      return true;
    }
    else {
      float vnow;
      if(micros() - myTimer < float(vmax) / float(accelmax) * float(pow(10, 6))) {
        vnow = float(accelmax * (micros() - myTimer)) / float(pow(10, 6));
        myDelta = 2.0 * float(vnow * vnow) / float(accelmax);
      }
      else {
        if(micros() - myTimer < tPos - float(vmax) / float(accelmax) * float(pow(10, 6))) {
          myDelta = 2.0 * float(vmax * vmax) / float(accelmax) + (float(micros() - myTimer) / float(pow(10, 6)) - float(vmax) / float(accelmax) * float(vmax));
        }
        else {
          vnow = float(vmax) - float(accelmax * (micros() - myTimer - tPos + float(vmax) / float(accelmax) * float(pow(10, 6)))) / float(pow(10, 6));
          myDelta = 2.0 * float(vmax * vmax) / float(accelmax) + (float(tPos - 2.0 * float(vmax) / float(accelmax)) * float(vmax)) + 2.0 * float(vnow * vnow) / float(accelmax);
        }
      }
      int angle = map(int(servoPosition + myDelta), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      angle = int(float(angle) * FREQUENCY * 4096.0 / 1000000.0);
      pwm.setPWM(pinServo, 0, angle);
    }
  }
  private:
};

void setup() {
  Serial.begin(9600);
  //Serial.println("16 channel Servo test!");
  pwm.begin();                //Initialize the library and send PWM signals.
  pwm.setPWMFreq(FREQUENCY);  //Servo's update frequency at 60 Hertz.
}

void loop() {
  pwm.setPWM(3, 0, pulseWidth(0));
}