

// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts()
// are used. Please use this code if your
// platform does not support ATOMIC_BLOCK.
//#include <util/atomic.h> // For the ATOMIC_BLOCK macro
//
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <ros.h>
//#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

#include <std_msgs/Float32MultiArray.h>

#include <armCmd.h>
#include <bcr1Telemetry.h>

const float pi = 3.14159265359;

float angle[6] = {0, 0, 0, 0, 0, 0};

void cmd_cb(const bcr1_control::armCmd& cmd_arm)
{
  angle[0] = cmd_arm.angle[0];
  angle[1] = cmd_arm.angle[1];
  angle[2] = cmd_arm.angle[2];
  angle[3] = cmd_arm.angle[3];
  angle[4] = cmd_arm.angle[4];
  angle[5] = cmd_arm.angle[5];
}


ros::NodeHandle nh;
//std_msgs::Float64 mydata;
//ros::Subscriber<sensor_msgs::JointState> sub("/move_group/fake_controller_joint_states", cmd_cb);
//ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", cmd_cb);

//ros::Subscriber<sensor_msgs::JointState> sub("/arduino/armCmd", cmd_cb);
ros::Subscriber<bcr1_control::armCmd> sub("/arduino/armCmd", cmd_cb);


bcr1_control::bcr1Telemetry msg;

ros::Publisher pub("/arduino/bcr1Telemetry", &msg);



// A class to compute the control signal
class SimplePID {
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
    // Constructor
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}

    // A function to set the parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // A function to compute the control signal
    void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
      // error
      int e = target - value;

      // derivative
      float dedt = (e - eprev) / (deltaT);

      // integral
      eintegral = eintegral + e * deltaT;

      // control signal
      float u = kp * e + kd * dedt + ki * eintegral;

      // motor power
      pwr = (int) fabs(u);
      if ( pwr > umax ) {
        pwr = umax;
      }

      // motor direction
      dir = 1;
      if (u < 0) {
        dir = -1;
      }

      // store previous error
      eprev = e;
      //      nh.spinOnce();
      //
      //      delay(2);
    }

};

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

// PID class instances
SimplePID pid[NMOTORS];

void setup() {
  Serial.begin(115200);

  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT);
    pinMode(encb[k], INPUT);
    pinMode(pwm[k], OUTPUT);
    pinMode(in1[k], OUTPUT);
    pinMode(in2[k], OUTPUT);
    pinMode(hall[k], INPUT);
            pid[k].setParams(5, 0.105, 0, 255);
    //    pid[k].setParams(1, 0.102, 0, 255);
    //        pid[k].setParams(1, 0, 0, 255);
    //pid[k].setParams(1, 0, 0, 255);
//    pid[k].setParams(1, 0.065, 0, 255);
    //    pid[k].setParams(5, 0, 0, 255);
    //       pid[k].setParams(3, 0.065, 0, 255);
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
      if(k == 0 || k == 1){
        setMotor(-1, 180, pwm[k], in1[k], in2[k]);
      } else {
        setMotor(1, 180, pwm[k], in1[k], in2[k]);
      }
    }
  }

  delay(1000);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(pub);
//  msg.base = (float*)malloc(sizeof(float) * 6);
//  msg.base = 6;

  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[4]), readEncoder<4>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[5]), readEncoder<5>, RISING);

  Serial.println("target pos");
}

void loop() {
//  int count = 0;
//  while (count < 1000) {
//    nh.spinOnce();
//    delay(0.005);
//    count++;
//  }
  nh.spinOnce();
  delay(1);

  // set target position
  int target[NMOTORS];
//  target[0] = ((6600 / 360) * angle[0] * 180 / pi) + ((6600 / 360) * 0);
//  target[1] = ((6600 / 360) * angle[1] * 180 / pi) + ((6600 / 360) * 78);
//  target[2] = ((6600 / 360) * angle[2] * 180 / pi) - ((6600 / 360) * 58);
//  target[3] = ((8910 / 360) * angle[3] * 180 / pi) + ((8910 / 360) * 90);
//  target[4] = ((6600 / 360) * angle[4] * 180 / pi) + ((6600 / 360) * 0);
//  target[5] = ((6600 / 360) * angle[5] * 180 / pi) + ((6600 / 360) * 0);
  target[0] = ((6600 / 360) * angle[0]) + ((6600 / 360) * 0);
  target[1] = ((6600 / 360) * angle[1]) + ((6600 / 360) * 78);
  target[2] = ((6600 / 360) * angle[2]) - ((6600 / 360) * 58);
  target[3] = ((8910 / 360) * angle[3]) + ((8910 / 360) * 0);
  target[4] = ((6600 / 360) * angle[4]) + ((6600 / 360) * 0);
  target[5] = ((6600 / 360) * angle[5]) + ((6600 / 360) * 0);

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

  // loop through the motors
  for (int k = 0; k < NMOTORS; k++) {
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k], target[k], deltaT, pwr, dir);

    // signal the motor
    setMotor(dir, pwr, pwm[k], in1[k], in2[k]);
  }



  
//  for (int k = 0; k < NMOTORS; k++) {
//    Serial.print(target[k]);
//    Serial.print(" ");
//    Serial.print(pos[k]);
//    Serial.print(" ");
////    msg.angle[k] = float(pos[k])/(6600/360);
//  }
//  
//  Serial.println();
  msg.angle[0] = float(pos[0])/(6600/360)+0;
  msg.angle[1] = float(pos[1])/(6600/360)-78;
  msg.angle[2] = float(pos[2])/(6600/360)+58;
  msg.angle[3] = float(pos[3])/(6600/360)-0;
  msg.angle[4] = float(pos[4])/(6600/360);
  msg.angle[5] = float(pos[5])/(6600/360);
  pub.publish(&msg);
  delay(10);
//  Serial.println((float(pos[1])/(6600/360)));
 
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
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


//template <int j>
//void readEncoder() {
//  int b = digitalRead(encb[j]);
//  int a = digitalRead(enca[j]);
//  if (digitalRead(encb[j]) != digitalRead(enca[j])) {
//    posi[j]++;
//  }
//  else {
//    posi[j]--;
//  }
//}
