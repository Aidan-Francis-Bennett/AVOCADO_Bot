#ifndef Movement_h
#define Movement_h
using namespace mbed;

//-------------------------speed and distance calculation functions-------------------------//

float * distanceArray();
float * speedArray();

//-------------------------calibrated movement functions-------------------------//

void forwardsCalib(float speed);
void backwardsCalib(float speed);
void clockwiseCalib(float angle_rad, int time_us);
void antiClockwiseCalib(float angle_rad, int time_us);
void leftTurnR_is_17cm(float angle_rad);
void rightTurnR_is_17cm(float angle_rad);
void leftTurnLargeR();
void rightTurnLargeR();
void stop();

// The callback function to call on a rising edge of the encoder B signal for incrementing encoder counts only: shaft revolutions not required for my use case.
void leftEncoderCounter();
// The callback function for counts of encoder A which is the right motor on my robot.
void rightEncoderCounter();

#endif