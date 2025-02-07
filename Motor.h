#ifndef Motor_h
#define Motor_h
using namespace mbed;

// Define macros for the DigitalOut and PwmOut object pins for each motor on my robot
#define LEFT_DIRECTION P0_5
#define LEFT_PWM P1_2
#define RIGHT_DIRECTION P0_4
#define RIGHT_PWM P0_27

class Motor {
  public:
    Motor(PinName directionPin, PinName pwmPin);
    void setup();
    void move(int direction, float pwm);
    void brake();
    int getDirection();
  private:
    DigitalOut _directionPin;
    PwmOut _pwmPin;
    int _direction;
};

#endif