#include "SmoothServoDriver.h"
#include "Wire.h"
#include "LSM6.h"

LSM6 imu;
long gyromax = 17000;
PCA9685SmoothServo servo0;
PCA9685SmoothServo servo1;
PCA9685SmoothServo servo2;
PCA9685SmoothServo servo3;
PCA9685SmoothServo servo4;

#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

int MIN_PULSES[5] = { 508, 464, 464, 411, 411 };  //These are the minimum and maximum wavelength values which serve MG 995.
int MAX_PULSES[5] = { 2524, 2597, 2597, 2636, 2636 };
int zeros[5] = { 80, 88, 80, 70, 60 };
float ks[5] = { -8.0 / 9.0, -8.0 / 9.0, 85.0 / 90.0, -54.0 / 90.0, -54.0 / 90.0 };
int rnum = 1;

int ticks(bool a0 = true, bool a1 = true, bool a2 = true, bool a3 = true, bool a4 = true) {
  int a = 0;
  if (a0) a += servo0.tick();
  else a += 1;
  if (a1) a += servo1.tick() * 2;
  else a += 2;
  if (a2) a += servo2.tick() * 4;
  else a += 4;
  if (a3) a += servo3.tick() * 8;
  else a += 8;
  if (a4) a += servo4.tick() * 16;
  else a += 16;
  return a;
}

void toPositions(int a0 = 1000, int a1 = 1000, int a2 = 1000, int a3 = 1000, int a4 = 1000, bool wait = true) {
  int a[5] = { true, true, true, true, true };
  if (a0 != 1000 and servo0.getPosition() != a0) servo0.toPosition(a0);
  else a[0] = false;
  if (a1 != 1000 and servo1.getPosition() != a1) servo1.toPosition(a1);
  else a[1] = false;
  if (a2 != 1000 and servo2.getPosition() != a2) servo2.toPosition(a2);
  else a[2] = false;
  if (a3 != 1000 and servo3.getPosition() != a3) servo3.toPosition(a3);
  else a[3] = false;
  if (a4 != 1000 and servo4.getPosition() != a4) servo4.toPosition(a4);
  else a[4] = false;
  int t = 0;
  if (wait) {
    while (t != 31) {
      t = ticks(a[0], a[1], a[2], a[3], a[4]);
    }
  }
}

void udarVlevo() {
  long gyrox, gyroy, gyroz, gyroold;
  toPositions(0, -90, 0, 90, 0, true);
  toPositions(-45, -90, 0, 90, 0, true);
  toPositions(-45, -90, -45, 90, 0, true);
  imu.read();
  gyrox = imu.g.x;
  gyroy = imu.g.y;
  gyroz = imu.g.z;
  gyrox = abs(gyrox);
  gyroy = abs(gyroy);
  gyroz = abs(gyroz);
  gyroold = gyrox + gyroy + gyroz;
  toPositions(-45, -90, -45, 0, 90, false);
  uint32_t myTimer = millis();
  myDFPlayer.play(random(1, 4));
  while (true) {
    imu.read();
    gyrox = imu.g.x;
    gyroy = imu.g.y;
    gyroz = imu.g.z;
    gyrox = abs(gyrox);
    gyroy = abs(gyroy);
    gyroz = abs(gyroz);
    Serial.print(abs(gyrox + gyroy + gyroz));
    Serial.print(" ");
    Serial.println(abs(gyroold));
    if (abs(gyrox + gyroy + gyroz - gyroold) > gyromax and millis() - myTimer > 500) {
      Serial.println(abs(gyrox + gyroy + gyroz - gyroold));
      toPositions(-45, -90, -45, servo3.getPosition() + 10, servo4.getPosition() - 10, true);
      break;
    }
    gyroold = gyrox + gyroy + gyroz;
    int nServs = ticks();
    if (nServs == 31) break;
  }
  toPositions(-45, -90, -45, 90, 0, true);
  toPositions(-45, -90, 0, 90, 0, true);
  toPositions(0, -90, 0, 90, 0, true);
}

void udarVpravo() {
  long gyrox, gyroy, gyroz, gyroold;
  toPositions(0, -90, 0, 90, 0, true);
  toPositions(0, -90, 45, 90, 0, true);
  imu.read();
  gyrox = imu.g.x;
  gyroy = imu.g.y;
  gyroz = imu.g.z;
  gyrox = abs(gyrox);
  gyroy = abs(gyroy);
  gyroz = abs(gyroz);
  gyroold = gyrox + gyroy + gyroz;
  toPositions(0, -90, 45, 0, 90, false);
  uint32_t myTimer = millis();
  myDFPlayer.play(random(1, 4));
  while (true) {
    imu.read();
    gyrox = imu.g.x;
    gyroy = imu.g.y;
    gyroz = imu.g.z;
    gyrox = abs(gyrox);
    gyroy = abs(gyroy);
    gyroz = abs(gyroz);
    Serial.print(gyrox + gyroy + gyroz);
    Serial.print(" ");
    Serial.println(abs(gyroold));
    if (abs(gyrox + gyroy + gyroz - gyroold) > gyromax and millis() - myTimer > 500) {
      Serial.println(abs(gyrox + gyroy + gyroz - gyroold));
      toPositions(-45, -90, 45, servo3.getPosition() + 10, servo4.getPosition() - 10, true);
      break;
    }
    gyroold = gyrox + gyroy + gyroz;
    int nServs = ticks();
    if (nServs == 31) break;
  }
  toPositions(0, -90, 45, 90, 0, true);
  toPositions(0, -90, 0, 90, 0, true);
}

void udarVpered() {
  long gyrox, gyroy, gyroz, gyroold;
  toPositions(0, -90, 0, 90, 0, true);
  imu.read();
  gyrox = imu.g.x;
  gyroy = imu.g.y;
  gyroz = imu.g.z;
  gyrox = abs(gyrox);
  gyroy = abs(gyroy);
  gyroz = abs(gyroz);
  gyroold = gyrox + gyroy + gyroz;
  toPositions(-90, -90, 0, 0, 0, false);
  uint32_t myTimer = millis();
  myDFPlayer.play(random(1, 4));
  while (true) {
    imu.read();
    gyrox = imu.g.x;
    gyroy = imu.g.y;
    gyroz = imu.g.z;
    gyrox = abs(gyrox);
    gyroy = abs(gyroy);
    gyroz = abs(gyroz);
    Serial.print(abs(gyrox + gyroy + gyroz));
    Serial.print(" ");
    Serial.println(abs(gyroold));
    if (abs(gyrox + gyroy + gyroz - gyroold) > gyromax and millis() - myTimer > 500) {
      Serial.println(abs(gyrox + gyroy + gyroz - gyroold));
      toPositions(servo0.getPosition() + 10, -90, 0, servo3.getPosition() + 10, 0, true);
      break;
    }
    gyroold = gyrox + gyroy + gyroz;
    int nServs = ticks();
    if (nServs == 31) break;
  }
  toPositions(0, -90, 0, 90, 0, true);
}

void blokVpered() {
  long gyrox, gyroy, gyroz, gyroold;
  toPositions(0, -90, 0, 90, 0, true);
  toPositions(0, -90, 0, 45, 0, true);
  imu.read();
  gyrox = imu.g.x;
  gyroy = imu.g.y;
  gyroz = imu.g.z;
  gyrox = abs(gyrox);
  gyroy = abs(gyroy);
  gyroz = abs(gyroz);
  gyroold = gyrox + gyroy + gyroz;
  toPositions(20, -90, -90, 45, 0, true);
  uint32_t myTimer = millis();
  while (true) {
    imu.read();
    gyrox = imu.g.x;
    gyroy = imu.g.y;
    gyroz = imu.g.z;
    gyrox = abs(gyrox);
    gyroy = abs(gyroy);
    gyroz = abs(gyroz);
    if ((abs(gyrox + gyroy + gyroz - gyroold) > gyromax or analogRead(0) < 650) and millis() - myTimer > 500) {
      break;
    }
    gyroold = gyrox + gyroy + gyroz;
  }
  delay(1000);
  if (abs(gyrox + gyroy + gyroz - gyroold) > gyromax) {
    myDFPlayer.play(random(4, 7));
  } else {
    myDFPlayer.play(random(7, 10));
  }
  toPositions(0, -90, 0, 45, 0, true);
  toPositions(0, -90, 0, 90, 0, true);
}

void blokVpravo() {
  long gyrox, gyroy, gyroz, gyroold;
  toPositions(  0, -90, 0, 90, 0, true);
  imu.read();
  gyrox = imu.g.x;
  gyroy = imu.g.y;
  gyroz = imu.g.z;
  gyrox = abs(gyrox);
  gyroy = abs(gyroy);
  gyroz = abs(gyroz);
  gyroold = gyrox + gyroy + gyroz;
  toPositions(-70, -90, 0, 0, -90, true);
  uint32_t myTimer = millis();
  while (true) {
    imu.read();
    gyrox = imu.g.x;
    gyroy = imu.g.y;
    gyroz = imu.g.z;
    gyrox = abs(gyrox);
    gyroy = abs(gyroy);
    gyroz = abs(gyroz);
    if ((abs(gyrox + gyroy + gyroz - gyroold) > gyromax or analogRead(0) < 650) and millis() - myTimer > 500) {
      break;
    }
    gyroold = gyrox + gyroy + gyroz;
  }
  delay(1000);
  if (abs(gyrox + gyroy + gyroz - gyroold) > gyromax) {
    myDFPlayer.play(random(4, 7));
  } else {
    myDFPlayer.play(random(7, 10));
  }
  toPositions(0, -90, 0, 90, 0, true);
}

void blokVlevo() {
  long gyrox, gyroy, gyroz, gyroold;
  toPositions(0, -90, 0, 90, 0, true);
  toPositions(0, -90, 0, 45, 0, true);
  imu.read();
  gyrox = imu.g.x;
  gyroy = imu.g.y;
  gyroz = imu.g.z;
  gyrox = abs(gyrox);
  gyroy = abs(gyroy);
  gyroz = abs(gyroz);
  gyroold = gyrox + gyroy + gyroz;
  toPositions(-45, -90, -75, 110, 40, true);
  uint32_t myTimer = millis();
  while (true) {
    imu.read();
    gyrox = imu.g.x;
    gyroy = imu.g.y;
    gyroz = imu.g.z;
    gyrox = abs(gyrox);
    gyroy = abs(gyroy);
    gyroz = abs(gyroz);
    if ((abs(gyrox + gyroy + gyroz - gyroold) > gyromax or analogRead(0) < 650) and millis() - myTimer > 500) {
      break;
    }
    gyroold = gyrox + gyroy + gyroz;
  }
  delay(1000);
  if (abs(gyrox + gyroy + gyroz - gyroold) > gyromax) {
    myDFPlayer.play(random(4, 7));
  } else {
    myDFPlayer.play(random(7, 10));
  }
  toPositions(0, -90, 0, 45, 0, true);
  toPositions(0, -90, 0, 90, 0, true);
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
  Wire.begin();
  Serial2.begin(9600);
  Serial3.begin(9600);
  if (!imu.init()) {
    Serial.println("Failed to detect and initialize imu!");
    while (1)
      ;
  }
  imu.enableDefault();
  if (!myDFPlayer.begin(Serial2)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    // while (true) {
    //   delay(0);  // Code to compatible with ESP8266 watch dog.
    // }
  }
  myDFPlayer.volume(0);  //Set volume value. From 0 to 30
  servo0.attach(0, MIN_PULSES[0], MAX_PULSES[0], zeros[0], ks[0], 0);
  servo1.attach(1, MIN_PULSES[1], MAX_PULSES[1], zeros[1], ks[1], -90);
  servo2.attach(2, MIN_PULSES[2], MAX_PULSES[2], zeros[2], ks[2], 0);
  servo3.attach(3, MIN_PULSES[3], MAX_PULSES[3], zeros[3], ks[3], 90);
  servo4.attach(4, MIN_PULSES[4], MAX_PULSES[4], zeros[4], ks[4], 0);
  delay(2500);
  //udarVpravo();
}

void loop() {
}
