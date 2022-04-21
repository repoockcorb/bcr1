#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

#include <bcr1_control/armCmd.h>
#include <bcr1_control/bcr1Telemetry.h>

const float pi = 3.14159265359;

float effort[6] = {0, 0, 0, 0, 0, 0};

void cmd_cb(const bcr1_control::armCmd& cmd_arm)
{
  effort[0] = cmd_arm.effort[0];
  effort[1] = cmd_arm.effort[1];
  effort[2] = cmd_arm.effort[2];
  effort[3] = cmd_arm.effort[3];
  effort[4] = cmd_arm.effort[4];
  effort[5] = cmd_arm.effort[5];
}


ros::NodeHandle nh;
ros::Subscriber<bcr1_control::armCmd> sub("/arduino/armCmd", cmd_cb);

bcr1_control::bcr1Telemetry msg;
ros::Publisher pub("/arduino/bcr1Telemetry", &msg);

// How many motors
#define NMOTORS 6

// Pins
const int enca[] = {3, 2, 18, 19, 20, 21};
const int encb[] = {34, 35, 36, 37, 38, 39};
const int pwm[] = {13, 12, 11, 10, 9, 8};
const int in1[] = {22, 25, 26, 28, 30, 32};
const int in2[] = {23, 24, 27, 29, 31, 33};
const int hall[] = {40, 41, 42, 43, 44, 45};

// Globals
long prevT = 0;
volatile int posi[] = {0, 0, 0, 0, 0, 0};
volatile int hall_s[] = {0, 0, 0, 0, 0, 0};
volatile int prevPwmVal[] = {0, 0, 0, 0, 0, 0};
volatile int pwmVal[] = {0, 0, 0, 0, 0, 0};


void setup() {
  Serial.begin(115200);

  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);
    pinMode(hall[k], INPUT);
  }

  // homing routine
  hall_s[0] = !digitalRead(hall[0]);
  hall_s[1] = !digitalRead(hall[1]);
  hall_s[2] = !digitalRead(hall[2]);
  hall_s[3] = !digitalRead(hall[3]);
  hall_s[4] = digitalRead(hall[4]);
  hall_s[5] = digitalRead(hall[5]);

  Serial.print("The hall status is:");
  Serial.println(hall_s[0]);
  Serial.print("The hall status is:");
  Serial.println(hall_s[1]);
  Serial.print("The hall status is:");
  Serial.println(hall_s[2]);
  Serial.print("The hall status is:");
  Serial.println(hall_s[3]);
  Serial.print("The hall status is:");
  Serial.println(hall_s[4]);
  Serial.print("The hall status is:");
  Serial.println(hall_s[5]);

  delay(1000);
  int k = 1;
  while (k < 5) {
    if (hall_s[k] == 0 || hall_s[k] == 1) {
      //      delay(1);
      hall_s[k] = !digitalRead(hall[k]);
      Serial.println("Homing");
      Serial.println(hall_s[k]);
      if (hall_s[k] == 1) {
        setMotor(-1, 0, pwm[k], in1[k], in2[k]);
        Serial.println("entered");
        k++;
        Serial.println(k);
        hall_s[k] = digitalRead(hall[k]);
        delay(2);
      }
      if (k == 0 || k == 1) {
        setMotor(0, -180, pwm[k], in1[k], in2[k]);
      } else {
        setMotor(0, 180, pwm[k], in1[k], in2[k]);
      }
    }
  }

  delay(1000);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(pub);

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder<4>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder<5>, RISING);

  Serial.println("target pos");
}


void loop() {

  nh.spinOnce();
  delay(1);

  // read effort

  for (int k = 0; k < NMOTORS; k++) {
    pwmVal[k] = effort[k];
  }

  // set target position
  int target[NMOTORS];

  target[0] = ((6600 / 360) * 0) + ((6600 / 360) * 0);
  target[1] = ((6600 / 360) * 0) + ((6600 / 360) * 77);
  target[2] = ((6600 / 360) * 0) - ((6600 / 360) * 53);
  target[3] = ((8910 / 360) * 0) + ((8910 / 360) * 0);
  target[4] = ((6600 / 360) * 0) + ((6600 / 360) * 0);
  target[5] = ((6600 / 360) * 0) + ((6600 / 360) * 0);



  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;


  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for (int k = 0; k < NMOTORS; k++) {
    pos[k] = posi[k];
  }
  interrupts(); // turn interrupts back on


  //  Serial.println();
  msg.angle[0] = float(pos[0]) / (6600 / 360) + 0;
  msg.angle[1] = float(pos[1]) / (6600 / 360) - 77;
  msg.angle[2] = float(pos[2]) / (6600 / 360) + 53;
  msg.angle[3] = float(pos[3]) / (6600 / 360) - 0;
  msg.angle[4] = float(pos[4]) / (6600 / 360);
  msg.angle[5] = float(pos[5]) / (6600 / 360);
  pub.publish(&msg);
  delay(10);

  for (int k = 0; k < NMOTORS; k++) {
    setMotor(prevPwmVal[k], pwmVal[k], pwm[k], in1[k], in2[k]);
  }


  for (int k = 0; k < NMOTORS; k++) {
    int prevPwmVal = effort[k];
  }

}

//void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
//  analogWrite(pwm, pwmVal);
//  if (dir == 1) {
//    digitalWrite(in1, HIGH);
//    digitalWrite(in2, LOW);
//  }
//  else if (dir == -1) {
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//  }
//  else {
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, LOW);
//  }
//}

void setMotor(int prevPwmVal, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, fabs(pwmVal));
  if (pwmVal >= prevPwmVal) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwmVal <= prevPwmVal) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) {
    posi[j]++;
  }
  else {
    posi[j]--;
  }
}
