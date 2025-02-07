#include <mbed.h>
#include "Motor.h"
using namespace mbed;

Motor::Motor(PinName directionPin, PinName pwmPin)
 : _directionPin(DigitalOut(directionPin)), _pwmPin(PwmOut(pwmPin)) {}

void Motor::setup(){
    _pwmPin.period_ms(2);
}

// Turns the given motor in a specified direction with a specified pulse width modulation value
void Motor::move(int direction, float pwm) {
  _directionPin.write(direction);
  _pwmPin.write(pwm);
}


void Motor::brake() {
  _pwmPin.write(0);
}

int Motor::getDirection() {
  _direction = _directionPin.read();
  return _direction;
}