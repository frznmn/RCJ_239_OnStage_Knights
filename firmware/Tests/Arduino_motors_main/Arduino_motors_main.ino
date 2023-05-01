//pins for motors
#define M1EN 12
#define M1INA 8
#define M1INB 13
#define M1PWM 11
#define M2EN 6
#define M2INA 7
#define M2INB 10
#define M2PWM 9
#define ENCODERA 0
#define ENCODERB 1
#define INTERRUPTA 0
#define INTERRUPTB 1
#define PINENCODERA 28
#define PINENCODERB 29


//variables
float kpmglobal = 4, kdmglobal = 1, kimglobal = 0.001, kpsglobal = 5, kdsglobal = 1, kisglobal = 0.007, kvglobal = 0.1;
long encoders[2] = { 0, 0 }, Astop = 0, Bstop = 0;
int rnum = 1;

//functions for external interrupts
void countEncoderA() {
  encoders[ENCODERA] -= digitalRead(PINENCODERA) * 2 - 1;
}

void countEncoderB() {
  encoders[ENCODERB] -= digitalRead(PINENCODERB) * 2 - 1;
}

//class for motors
class MOTOR {
public:
  //variables
  int pinMotEN, pinMotINA, pinMotINB, pinMotPWM, reverse;
  //constructor
  MOTOR(int _pinMotEN, int _pinMotINA, int _pinMotINB, int _pinMotPWM, int _reverse) {
    pinMotEN = _pinMotEN;
    pinMotINA = _pinMotINA;
    pinMotINB = _pinMotINB;
    pinMotPWM = _pinMotPWM;
    reverse = _reverse;
    pinMode(pinMotEN, 1);
    pinMode(pinMotINA, 1);
    pinMode(pinMotINB, 1);
    pinMode(pinMotPWM, 1);
    digitalWrite(pinMotEN, 1);
  }
  //function that sets motor speed
  void rotate(int motorSpeed) {
    motorSpeed *= -reverse * 2 + 1;
    int v = map(abs(motorSpeed), 0, 100, 0, 255);
    digitalWrite(pinMotINA, (sgn(motorSpeed) + 1) / 2);
    digitalWrite(pinMotINB, (-sgn(motorSpeed) + 1) / 2);
    analogWrite(pinMotPWM, v);
  }
  //function that stops motor
  void stay() {
    digitalWrite(pinMotINA, 1);
    digitalWrite(pinMotINB, 1);
    digitalWrite(pinMotPWM, 1);
  }
private:
};

//initialisation motors
MOTOR motorA(M1EN, M1INA, M1INB, M1PWM, 0);
MOTOR motorB(M2EN, M2INA, M2INB, M2PWM, 1);

//number's sign function
int sgn(float a) {
  if (a > 0) return 1;
  if (a < 0) return -1;
  return 0;
}

long fromCamera(int space = 95) {
  int a = -1;
  long x = 0;
  int znak = 1;
  while (true) {
    while (!Serial2.available())
      ;
    a = Serial2.read();
    if (a == space) return x * znak;
    else {
      if (a == 45) znak = -1;
      else {
        x *= 10;
        x += a - 48;
      }
    }
  }
}

//synchronization of motors to go forward/backward
void forward(int distance, int vmax = 100, float _kp = kpmglobal, float _kd = kdmglobal, float _ki = kimglobal, float kv = kvglobal) {
  long zeros[2] = { encoders[ENCODERA], encoders[ENCODERB] }, ei = 0;
  int va = 0;
  int vb = 0;
  encoders[ENCODERA] = encoders[ENCODERB] = 0;
  vmax = abs(vmax);
  int encA = encoders[ENCODERA], encB = encoders[ENCODERB], epold = encB - encA, ep = 0, ed = 0;
  float u;
  int v;
  float kp, kd, ki;
  while (abs(encA + encB) < abs(distance) * 2) {
    encA = encoders[ENCODERA];
    encB = encoders[ENCODERB];
    if (abs(encA + encB) < abs(distance)) {
      v = abs(encA + encB) / 2;
    } else {
      v = abs(distance) - abs(encA + encB) / 2;
    }
    v = float(v) * kv;
    v += 15;
    v = max(15, v);
    v = min(100, v);
    kp = float(_kp * v) / 100.0;
    ep = encB - encA;
    //ed = epold - ep;
    ei += ep;
    u = float(ep) * kp + ed * kd + float(ei) * ki;
    epold = ep;
    va = v * sgn(distance) + u;
    va = max(abs(va), 15) * sgn(va);
    vb = v * sgn(distance) - u;
    vb = max(abs(vb), 15) * sgn(vb);
    motorA.rotate(va);
    motorB.rotate(vb);
    Serial.print(encoders[ENCODERA]);
    Serial.print(" ");
    Serial.print(encoders[ENCODERB]);
    Serial.print(" ");
    Serial.print(v);
    Serial.print(" ");
    Serial.println(u);
  }
  motorA.stay();
  motorB.stay();
  encoders[ENCODERA] += zeros[ENCODERA];
  encoders[ENCODERB] += zeros[ENCODERB];
  delay(250);
}

//synchronization of motors to turn
void turn(int distance, int vmax = 100, float _kp = kpmglobal, float _kd = kdmglobal, float _ki = kimglobal, float kv = kvglobal) {
  distance = float(distance) * 450.0 / 90.0;
  long zeros[2] = { encoders[ENCODERA], encoders[ENCODERB] }, ei = 0;
  int va = 0;
  int vb = 0;
  encoders[ENCODERA] = encoders[ENCODERB] = 0;
  vmax = abs(vmax);
  int encA = encoders[ENCODERA], encB = encoders[ENCODERB], epold = -encB - encA, ep = 0, ed = 0;
  float u;
  int v;
  float kp, kd, ki = _ki;
  while (abs(encA - encB) < abs(distance) * 2) {
    encA = encoders[ENCODERA];
    encB = encoders[ENCODERB];
    if (abs(encA - encB) < abs(distance)) {
      v = abs(encA - encB) / 2;
    } else {
      v = abs(distance) - abs(encA - encB) / 2;
    }
    v = float(v) * kv;
    v += 15;
    v = max(15, v);
    v = min(100, v);
    kp = float(_kp * v) / 100.0;
    ep = -encB - encA;
    //ed = epold - ep;
    ei += ep;
    u = float(ep) * kp + ed * kd + float(ei) * _ki;
    epold = ep;
    va = v * sgn(distance) + u;
    va = max(abs(va), 15) * sgn(va);
    vb = v * sgn(distance) - u;
    vb = max(abs(vb), 15) * sgn(vb);
    motorA.rotate(va);
    motorB.rotate(-vb);
    Serial.print(encoders[ENCODERA]);
    Serial.print(" ");
    Serial.print(encoders[ENCODERB]);
    Serial.print(" ");
    Serial.print(v);
    Serial.print(" ");
    Serial.println(u);
  }
  motorA.stay();
  motorB.stay();
  encoders[ENCODERA] += zeros[ENCODERA];
  encoders[ENCODERB] += zeros[ENCODERB];
  delay(250);
}

bool toTags(int tormoz = 10, int vmin = 15, int space = 95) {
  forward(100);
  while (Serial2.available()) Serial.println(Serial2.read());
  //while (fromCamera() % 2000 / 4 == 0 and fromCamera() / 2000 == 0)
  //  ;
  bool tag1 = false;
  bool noTags = false;
  long errors = fromCamera();
  tag1 = max(tag1, errors % 2);
  noTags = 1 - errors % 4 / 2;
  int a = -1;
  long x = 0;
  int znak = 1;
  int errorTag = errors % 2000 / 4;
  errorTag -= 250 * sgn(errorTag);
  int toTag = errors / 2000;
  float ke = 0.2;
  float kv = 1.2;
  int v = vmin;
  float u = 0;
  uint32_t myTimer = millis();
  while (true) {
    if (!noTags) {
      v = toTag * kv;
      v -= tormoz;
      v = max(vmin, v);
      v = min(vmin, v);
    }
    u = float(errorTag) * ke;
    u = min(abs(u), v / 2) * sgn(u);
    motorA.rotate(v + u);
    motorB.rotate(v - u);
    if (toTag == 0 and !noTags and millis() - myTimer > 500) {
      motorA.stay();
      motorB.stay();
      delay(250);
      forward(100);
      Serial.print(errors);
      Serial.println("ok");
      break;
    }
    Serial.print(errors);
    Serial.print(" ");
    Serial.print(errorTag);
    Serial.print(" ");
    Serial.println(toTag);
    if (Serial2.available()) {
      a = Serial2.read();
      if (a == space) {
        a = -1;
        znak = 1;
        errors = x;
        tag1 = max(tag1, errors % 2);
        noTags = 1 - errors % 4 / 2;
        errorTag = errors % 2000 / 4;
        errorTag -= 250 * sgn(errorTag);
        toTag = errors / 2000;
        x = 0;
      } else {
        if (a == 45) znak = -1;
        else {
          x *= 10;
          x += a - 48;
        }
      }
    }
  }
  return tag1;
}

int fromArduino(int space = 95) {
  int a = -1;
  int x = 0;
  int znak = 1;
  int len = 1;
  while (true) {
    while (!Serial3.available())
      ;
    a = Serial3.read();
    if (a == space) return x * znak;
    else {
      if (a == 45) znak = -1;
      else {
        x += (a - 48) * len;
        len *= 10;
      }
    }
  }
}

void toArduino(int a) {
  while (a != 0) {
    if (a < 0) {
      Serial3.write(45);
      a *= -1;
    }
    Serial3.write(a % 10 + 48);
    a /= 10;
  }
  Serial3.write(95);
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  attachInterrupt(INTERRUPTA, countEncoderA, RISING);
  pinMode(PINENCODERA, 0);
  attachInterrupt(INTERRUPTB, countEncoderB, RISING);
  pinMode(PINENCODERB, 0);
  delay(2500);
  forward(750);
  forward(-750);
  turn(360);
  turn(-360);
  for(int i = 0; i < 5; i++) {
    toArduino(1);
  }
}

void loop() {
}
