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
int rnum = 0;
int distanceg = 10;
int distancet = 26;
int nUdarov = 6;

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
MOTOR motorA(M1EN, M1INA, M1INB, M1PWM, 1);
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
    if (Serial2.available()) {
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

void toArduino(int a, int space = 95) {
  while (a != 0) {
    if (a < 0) {
      Serial3.write(45);
      a *= -1;
    }
    Serial3.write(a % 10 + 48);
    a /= 10;
  }
  Serial3.write(space);
}

int cleanReadBuff3(int space = 95) {
  while (Serial3.available()) Serial3.read();
  fromArduino(space);
  int a = fromArduino(space);
  return a;
}

int writeBuff3(int a, int n = 5, int space = 95) {
  for (int i = 0; i < n; i++) {
    toArduino(a, space);
    delay(10);
  }
}

int cleanReadBuff2(int space = 95) {
  while (Serial2.available()) Serial2.read();
  fromCamera(space);
  int a = fromCamera(space);
  return a;
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

int isLeaved(int dist = distanceg - 1, int space = 95, uint16_t time = 3000) {
  int leaved = 0;
  long errors = cleanReadBuff2(space);
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

void waitUntilLeaves(int dist = distanceg - 1, int space = 95, uint16_t time = 5500) {
  long errors = cleanReadBuff2(space);
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

void waitUntilArrives(int dist = distanceg - 1, int space = 95, uint16_t time = 5500) {
  dist -= 1;
  long errors = cleanReadBuff2(space);
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
      //Serial.println(a);
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
      Serial.println("ok");
      break;
    }
  }
}

void toOpp(int dist = distanceg, int v = 15, int space = 95, int wUst = 15, float kvOpp = 1.2) {
  long errors = cleanReadBuff2(space);
  int a = -1;
  long x = 0;
  int znak = 1;
  int galsx = 0;
  int galsw = 0;
  int udar = 0;
  float ke = 0.6;
  float u = 0;
  int flag = 0;
  int uOpp = 0;
  uint32_t myTimer = millis();
  while (true) {
    u = float(galsx) * ke;
    if (dist == distancet) {
      uOpp = float(wUst - galsw) * kvOpp;
      uOpp = min(abs(uOpp), v / 3) * sgn(uOpp);
      motorA.rotate(v + uOpp + u);
      motorB.rotate(v + uOpp - u);
    } else {
      motorA.rotate(v + u);
      motorB.rotate(v - u);
    }
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

int defense(int nUdar = 10, int dist = distanceg, int nWrite = 5, int space = 95) {
  int operation = 0;
  int n1 = 0, n2 = 0, n3 = 0;
  int blok = 0;
  int defensed = 0;
  int toUart = 0;
  forward(-500);
  waitUntilArrives(dist - 2);
  delay(1000);
  int a = cleanReadBuff2(space);
  for (int i = 0; i < nUdar; i++) {
    a = fromCamera() / 40000;
    Serial.println(a);
    switch (a) {
      case 1:
        {
          n1 += 1;
          break;
        }
      case 2:
        {
          n2 += 1;
          break;
        }
      case 3:
        {
          n3 += 1;
          break;
        }
      default:
        {
          break;
        }
    }
  }
  if (n1 > n2 and n1 > n3) {
    blok = 1;
  } else {
    if (n2 > n1 and n2 > n3) {
      blok = 2;
    } else {
      if (n3 > n1 and n3 > n2) {
        blok = 3;
      } else {
        blok = random(1, 4);
      }
    }
  }
  toUart = blok + 3;
  Serial.println(blok);
  writeBuff3(toUart, nWrite, space);
  defensed = cleanReadBuff3(space);
  if (defensed == 2) {
    operation = 2;
  } else {
    operation = 1;
  }
  Serial.println(operation);
  return operation;
}

int attack(int dist = distanceg, int degr1 = 14, int degr2 = 28, int degr3 = 10, int nWrite = 5, int space = 95) {
  int operation = 0;
  int toUart = 0;
  int leaved = 0;
  waitUntilLeaves(dist - 2);
  toUart = random(1, 4);
  writeBuff3(toUart, nWrite, space);
  toOpp(dist);
  delay(2000);
  switch (toUart) {
    case 1:
      {
        turn(-degr1);
        break;
      }
    case 2:
      {
        turn(-degr2);
        break;
      }
    case 3:
      {
        turn(-degr3);
        break;
      }
    default:
      {
        break;
      }
  }
  writeBuff3(1, nWrite, space);
  cleanReadBuff3(space);
  switch (toUart) {
    case 1:
      {
        turn(degr1);
        break;
      }
    case 2:
      {
        turn(degr2);
        break;
      }
    case 3:
      {
        turn(degr3);
        break;
      }
    default:
      {
        break;
      }
  }
  leaved = isLeaved(dist - 2);
  if (leaved == 1) {
    operation = 2;
  } else {
    operation = 1;
  }
  return operation;
}

void setup() {
  randomSeed(analogRead(1));
  int naction = 0;
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  attachInterrupt(INTERRUPTA, countEncoderA, RISING);
  pinMode(PINENCODERA, 0);
  attachInterrupt(INTERRUPTB, countEncoderB, RISING);
  pinMode(PINENCODERB, 0);
  int summOper = 0;
  //for(;;);
  // delay(2500);
  // writeBuff3(1);
  // delay(2000);
  // writeBuff3(1);
  int operation;
  if (rnum == 0) {
    while (cleanReadBuff2() % 2 == 0)
      ;
    forward(2500, 20);
    writeBuff3(7);
    delay(3000);
    turn(180);
    delay(2000);
    operation = attack();
  } else {
    uint32_t myTimer = millis();
    int gw = 0;
    while (millis() - myTimer < 250) {
      gw = cleanReadBuff2() % 40000 / 400;
      Serial.println(gw);
      if (gw >= distancet * 10 / 13) {
        myTimer = millis();
      }
    }
    Serial.println("aaa");
    toOpp(distancet, 20);
    forward(-500);
    writeBuff3(7);
    delay(5000);
    operation = defense();
  }
  for (int i = 1; i < nUdarov; i++) {
    delay(500);
    if ((operation == 2 and summOper < 2) or summOper <= -2) {
      operation = attack();
      summOper ++;
    } else {
      operation = defense();
      summOper --;
    }
  }
  if (rnum == 0) {  
    turn(-90);
    writeBuff3(8);
    turn(-270, 15);
    delay(5000);
    writeBuff3(7);
      while (cleanReadBuff2() % 2 == 0)
    ;
    forward(2500);
  } else {
    forward(-100);
    delay(2000);
    writeBuff3(9);
    delay(5000);
    turn(180);
      while (cleanReadBuff2() % 2 == 0)
    ;
    writeBuff3(10);
    forward(2500);
  }
  motorA.stay();
  motorB.stay();
}

void loop() {
  Serial.print(encoders[ENCODERA]);
  Serial.print(" ");
  Serial.println(encoders[ENCODERB]);
}
