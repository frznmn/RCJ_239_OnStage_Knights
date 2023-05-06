//pins for motors
#define M1EN 12
#define M1INA 8
#define M1INB 13
#define M1PWM 11
#define M2EN 6
#define M2INA 10
#define M2INB 7
#define M2PWM 9
#define ENCODERA 0
#define ENCODERB 1
#define INTERRUPTA 0
#define INTERRUPTB 1
#define PINENCODERA 30
#define PINENCODERB 31


//variables
float kpmglobal = 10, kdmglobal = 1, kimglobal = 0.001, kvglobal = 0.1;
volatile long encoders[2] = { 0, 0 }, Astop = 0, Bstop = 0;
int rnum = 1;
int distanceg = 6;
int nUdarov = 6;
int check = 0;

//functions for external interrupts
void countEncoderA() {
  encoders[ENCODERA] += digitalRead(PINENCODERA) * 2 - 1;
}

void countEncoderB() {
  encoders[ENCODERB] += digitalRead(PINENCODERB) * 2 - 1;
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
    analogWrite(pinMotPWM, constrain(v, -255, 255));
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
  while (encA + encB < abs(distance) * 2) {
    encA = abs(encoders[ENCODERA]);
    encB = abs(encoders[ENCODERB]);
    if (encA + encB < abs(distance)) {
      v = (encA + encB) / 2;
    } else {
      v = abs(distance) - (encA + encB) / 2;
    }
    v = float(v) * kv;
    v += 15;
    v = min(vmax, v);
    kp = float(_kp * v) / 100.0;
    ep = (encB - encA) * sgn(distance);
    //ed = epold - ep;
    ei += ep;
    if (sgn(epold) != sgn(ep)) ei = 0;
    u = float(ep) * kp + ed * kd + float(ei) * ki;
    epold = ep;
    va = v * sgn(distance) + u;
    vb = v * sgn(distance) - u;
    motorA.rotate(va);
    motorB.rotate(vb);
  }
  motorA.stay();
  motorB.stay();
  encoders[ENCODERA] += zeros[ENCODERA];
  encoders[ENCODERB] += zeros[ENCODERB];
  delay(250);
}

//synchronization of motors to turn
void turn(int distance, int vmax = 100, float _kp = kpmglobal, float _kd = kdmglobal, float _ki = kimglobal, float kv = kvglobal) {
  distance = float(distance) * 415.0 / 90.0;
  long zeros[2] = { encoders[ENCODERA], encoders[ENCODERB] }, ei = 0;
  int va = 0;
  int vb = 0;
  encoders[ENCODERA] = encoders[ENCODERB] = 0;
  vmax = abs(vmax);
  int encA = encoders[ENCODERA], encB = encoders[ENCODERB], epold = -encB - encA, ep = 0, ed = 0;
  float u, uold;
  int v;
  float kp, kd, ki = _ki;
  while (encA + encB < abs(distance) * 2) {
    encA = abs(encoders[ENCODERA]);
    encB = abs(encoders[ENCODERB]);
    if (encA + encB < abs(distance)) {
      v = (encA + encB) / 2;
    } else {
      v = abs(distance) - (encA + encB) / 2;
    }
    v = float(v) * kv;
    v += 15;
    v = min(vmax, v);
    kp = float(_kp * v) / 100.0;
    ep = (encB - encA) * sgn(distance);
    //ed = epold - ep;
    ei += ep;
    if (sgn(epold) != sgn(ep)) ei = 0;
    u = float(ep) * kp + ed * kd + float(ei) * _ki;
    epold = ep;
    va = constrain(v * sgn(distance) + u, -100, 100);
    vb = constrain(v * sgn(distance) - u, -100, 100);
    motorA.rotate(va);
    motorB.rotate(-vb);
    Serial.print(encA);
    Serial.print(" ");
    Serial.print(encB);
    Serial.print(" ");
    Serial.print(va);
    Serial.print(" ");
    Serial.print(vb);
    Serial.print(" ");
    Serial.println(ep);
  }
  motorA.stay();
  motorB.stay();
  encoders[ENCODERA] += zeros[ENCODERA];
  encoders[ENCODERB] += zeros[ENCODERB];
  delay(250);
}

int isLeaved(int dist = distanceg - 1, int space = 85, uint16_t time = 1500) {
  int leaved = 0;
  while (Serial2.available()) Serial2.read();
  fromCamera();
  long errors = 0;
  uint32_t myTimer = millis();
  int a = -1;
  long x = 0;
  int znak = 1;
  int galsx = 0;
  int galsw = 0;
  int udar = 0;
  while (millis() - myTimer < time) {
    if (Serial2.available()) {
      a = Serial2.read();
      if (a == space) {
        errors = x * znak;
        a = -1;
        znak = 1;
        x = 0;
        galsx = errors % 400 / 2 - 100;
        galsw = errors % 40000 / 400;
        udar = errors % 40000;
      } else {
        if (a == 45) znak *= -1;
        else {
          x *= 10;
          x += a - 48;
        }
      }
    }
    if (galsw < dist and galsw != 0) {
      leaved = 1;
      break;
    }
  }
  return leaved;
}

void waitUntilLeaves(int dist = distanceg - 1, int space = 85, uint16_t time = 5000) {
  while (Serial2.available()) Serial2.read();
  fromCamera();
  long errors = 0;
  uint32_t myTimer = millis();
  int a = -1;
  long x = 0;
  int znak = 1;
  int galsx = 0;
  int galsw = 0;
  int udar = 0;
  while (millis() - myTimer < time) {
    if (Serial2.available()) {
      a = Serial2.read();
      if (a == space) {
        errors = x * znak;
        a = -1;
        znak = 1;
        x = 0;
        galsx = errors % 400 / 2 - 100;
        galsw = errors % 40000 / 400;
        udar = errors % 40000;
      } else {
        if (a == 45) znak *= -1;
        else {
          x *= 10;
          x += a - 48;
        }
      }
    }
    if (galsw < dist and galsw != 0) {
      break;
    }
  }
}

void waitUntilArrives(int dist = distanceg - 1, int space = 85, uint16_t time = 5000) {
  dist -= 1;
  while (Serial2.available()) Serial2.read();
  fromCamera();
  long errors = 0;
  uint32_t myTimer = millis();
  int a = -1;
  long x = 0;
  int znak = 1;
  int galsx = 0;
  int galsw = 0;
  int udar = 0;
  while (millis() - myTimer < time) {
    if (Serial2.available()) {
      a = Serial2.read();
      if (a == space) {
        errors = x * znak;
        a = -1;
        znak = 1;
        x = 0;
        galsx = errors % 400 / 2 - 100;
        galsw = errors % 40000 / 400;
        udar = errors % 40000;
      } else {
        if (a == 45) znak *= -1;
        else {
          x *= 10;
          x += a - 48;
        }
      }
    }
    if (galsw >= dist and galsw != 0) {
      break;
    }
  }
}

void toOpp(int dist = distanceg, int v = 15, int space = 95) {
  while (Serial2.available()) Serial.println(Serial2.read());
  fromCamera();
  long errors = fromCamera();
  int a = -1;
  long x = 0;
  int znak = 1;
  int galsx = 0;
  int galsw = 0;
  int udar = 0;
  float ke = 0.6;
  float u = 0;
  int flag = 0;
  uint32_t myTimer = millis();
  while (true) {
    Serial.println(errors);
    u = float(galsx) * ke;
    motorA.rotate(v + u);
    motorB.rotate(v - u);
    if (galsw >= dist and galsw != 0 and millis() - myTimer > 1500 and flag == 0) {
      v = 0;
      flag = 1;
    }
    if (flag == 1 and abs(galsx) < 5) {
      motorA.stay();
      motorB.stay();
      delay(250);
      break;
    }
    if (Serial2.available()) {
      a = Serial2.read();
      if (a == space) {
        errors = x * znak;
        a = -1;
        znak = 1;
        x = 0;
        galsx = errors % 400 / 2 - 100;
        galsw = errors % 40000 / 400;
        udar = errors % 40000;
      } else {
        if (a == 45) znak *= -1;
        else {
          x *= 10;
          x += a - 48;
        }
      }
    }
  }
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

int defense(int nUdar = 5) {
  int operation = 0;
  int n1 = 0, n2 = 0, n3 = 0;
  int a = 0;
  int blok = 0;
  int defensed = 0;
  forward(-1000);
  waitUntilArrives();
  while(Serial2.available()) Serial2.read();
  fromCamera();
  for(int i = 0; i < nUdar; i++) {
    a = fromCamera() % 40000;
    switch(a) {
      case 1: {
        n1 += 1;
        break;
      }
      case 2: {
        n2 += 1;
        break;
      }
      case 3: {
        n3 += 1;
        break;
      }
      default: {
        break;
      }
    }
  }
  if(n1 > n2 and n1 > n3) {
    blok = 1;
  }
  else {
    if(n2 > n1 and n2 > n3) {
      blok = 2;
    }
    else {
      if(n3 > n1 and n2 > n3) {
        blok = 3;
      }
      else {
        blok = random(1, 4);
      }
    }
  }
  blok += 3;
  for(int i = 0; i < 5; i++) {
    toArduino(blok * 2 + check);
  }
  check = (check + 1) % 2;
  while(Serial3.available()) Serial3.read();
  fromArduino();
  defensed = fromArduino();
  if(defensed == 2) {
    operation = 2;
  }
  else {
    operation = 1;
  }
  return operation;
}

void setup() {
  int naction = 0;
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  attachInterrupt(INTERRUPTA, countEncoderA, RISING);
  pinMode(PINENCODERA, 0);
  attachInterrupt(INTERRUPTB, countEncoderB, RISING);
  pinMode(PINENCODERB, 0);
  turn(360);
  //delay(2500);
  // motorA.stay();
  // motorB.stay();
}

void loop() {
}
