#include <mbed.h>
#include <cmath>
#include"Motor.h"
#include "Movement.h"
using namespace mbed;

extern Motor leftMotor;
extern Motor rightMotor;

// Set up an InterruptIn for the encoder B signal which is the left hand motor
extern InterruptIn leftEncoder;
long int leftEncoderCount;
// This is an absolute value for the left encoder, ie it doesn't decrement with each rising edge, and it should also be reset every time the Timer is reset so it can be used for measuring speed
long int leftEncoderAbsoluteCount;

// And for the right hand motor
extern InterruptIn rightEncoder;
long int rightEncoderCount;
long int rightEncoderAbsoluteCount;

// 

// This array is for storing temporary distances of each wheel that can be reset whenever comparison of the wheel distances over a set period of time is needed e.g. with calibration in the turning functions.
float DistanceComparisonLR[2];

// This timer is for measuring average speed of each wheel.
extern Timer speedTimer;

// Functions for calculating the distance travelled by each wheel in cm. This can go negative because it includes going backwards, so is more like displacement than distance.
// the 44 comes from 660/15 to find the encoder counts per cm, since there are 660 encoder counts per revolution, and the wheel has a circumference of 15cm
// Returns an array using pointers.
float * distanceArray(){
  static float distancesLR[2];
  distancesLR[0] = (float)leftEncoderCount/44; 
  distancesLR[1] = (float)rightEncoderCount/44;
  return distancesLR;
}

// A function for calculating the average speed since this function was previously called: does not account for direction as it uses an absolute encoder count.
// Returns a vector with the left and right wheel speeds in cm per second, in the array in that order: the 22727.27 is a factor to both scale to seconds from microseconds, and to cms per second from encoder counts per second based on wheel odometry.
//Speed = distance/time : circumference = 15cm : cm per encoder count = 15/660 : so distance = 15/660*encoder counts : scale to seconds from microseconds by multiplying by 1000000 : so finally speed = (encoder counts*(15*1000000/660))/time
float * speedArray(){
  int timeElapsed = speedTimer.elapsed_time().count();
  static float speedsLR[2];
  speedsLR[0] = (leftEncoderAbsoluteCount*22727.27f)/timeElapsed;
  speedsLR[1] = (rightEncoderAbsoluteCount*22727.27f)/timeElapsed;
  rightEncoderAbsoluteCount = 0;
  leftEncoderAbsoluteCount = 0;
  speedTimer.reset();
  return speedsLR;
}

//-------------------------calibrated movement functions-------------------------//

// These calibrated speed functions take a speed in cm per second and calculate a pwm value for each motor using a second order polynomial based on plotted motor calibration data.
void forwardsCalib(float speed){
  float leftpwm = -0.0002*pow(speed, 2) + 0.0337*speed + 0.0543;
  float rightpwm = -0.0002*pow(speed, 2) + 0.0344*speed + 0.0581;
  leftMotor.move(0, leftpwm);
  rightMotor.move(1, rightpwm);
}
void backwardsCalib(float speed){
  float leftpwm = -0.0002*pow(speed, 2) + 0.0343*speed + 0.0539;
  float rightpwm = -0.0003*pow(speed, 2) + 0.0365*speed + 0.0558;
  leftMotor.move(1, leftpwm);  
  rightMotor.move(0, rightpwm);
}

// These two functions are for turning clockwise or anti clockwise taking an angle, and a time to turn that angle, as their arguments.
void clockwiseCalib(float angle_rad, int time_us){
  // The 7350000 factor is to convert to cm per second, and factor in d of the robot, which is 7.35cm, since output speed of one wheel is 7.35cm*(delta_theta/delta_t)
  float speed = (angle_rad/time_us)*7350000;
  float leftpwm = -0.0002*pow(speed, 2) + 0.0337*speed + 0.0543;
  float rightpwm = -0.0003*pow(speed, 2) + 0.0365*speed + 0.0558;
  leftMotor.move(0, leftpwm);
  rightMotor.move(0, rightpwm);
  wait_us(time_us);
  stop();
}
void antiClockwiseCalib(float angle_rad, int time_us){
  float speed = (angle_rad/time_us)*7350000;
  float leftpwm = -0.0002*pow(speed, 2) + 0.0343*speed + 0.0539;
  float rightpwm = -0.0002*pow(speed, 2) + 0.0344*speed + 0.0581;
  leftMotor.move(1, leftpwm);
  rightMotor.move(1, rightpwm);
  wait_us(time_us);
  stop();
}

// This function turns to the left with a turning radius of about 17cm, which equates to turning centered approximately around a corner when the left IR sensor is 8cm away from the wall, with a linear velocity of about 17cm per second.
// Note that I haven't written this version of the calculation on paper yet
// The angle is turned through by changing the amount of time the function should run given the set wheel speeds: delta_t = delta_theta*(2d/(Vr-Vl))= delta_theta*(14.7cm/(20cm/s-8cm/s))*1000000 to convert to us from seconds
void leftTurnR_is_17cm(float angle_rad){
  int time_us = 1225000 * angle_rad;
  float leftpwm = -0.0002*pow(8.0, 2) + 0.0337*8.0 + 0.0543;
  float rightpwm = -0.0002*pow(20.0, 2) + 0.0344*20.0 + 0.0581;
  leftMotor.move(0, leftpwm);
  rightMotor.move(1, rightpwm);
  wait_us(time_us);
  stop();
}

// This function does the same as the above, except turning to the right.
void rightTurnR_is_17cm(float angle_rad){
  int time_us = 1225000 * angle_rad;
  float leftpwm = -0.0002*pow(20.0, 2) + 0.0337*20.0 + 0.0543;
  float rightpwm = -0.0002*pow(8.0, 2) + 0.0344*8.0 + 0.0581;
  leftMotor.move(0, leftpwm);
  rightMotor.move(1, rightpwm);
  wait_us(time_us);
  stop();
}

// These large turning circle functions are for nudging the robot towards, or away from, the left wall. They equate to a linear velocity of about 16cm per second. The turning circle is a value of R of about 117.6cm.
void leftTurnLargeR(){
  float leftpwm = -0.0002*pow(15.0, 2) + 0.0337*15.0 + 0.0543;
  float rightpwm = -0.0002*pow(17.0, 2) + 0.0344*17.0 + 0.0581;
  leftMotor.move(0, leftpwm);
  rightMotor.move(1, rightpwm);
}

void rightTurnLargeR(){
  float leftpwm = -0.0002*pow(17.0, 2) + 0.0337*17.0 + 0.0543;
  float rightpwm = -0.0002*pow(15.0, 2) + 0.0344*15.0 + 0.0581;
  leftMotor.move(0, leftpwm);
  rightMotor.move(1, rightpwm);
}

void stop(){
  leftMotor.brake();
  rightMotor.brake();
}

// The callback function to call on a rising edge of the encoder B signal for incrementing encoder counts only: shaft revolutions not required for my use case.
void leftEncoderCounter() {
  leftEncoderAbsoluteCount++;
  if (leftMotor.getDirection() == 0) {
    leftEncoderCount++;
  }
  else {
    leftEncoderCount--;
  }
}
// The callback function for counts of encoder A which is the right motor on my robot.
void rightEncoderCounter() {
  rightEncoderAbsoluteCount++;
  if (rightMotor.getDirection() == 1) {
    rightEncoderCount++;
  }
  else {
    rightEncoderCount--;
  }
}