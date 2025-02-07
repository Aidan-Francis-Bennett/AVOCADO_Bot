#ifndef UltraSonic_h
#define UltraSonic_h
using namespace mbed;

// Define macros for the DigitalInOut objects for the trigger AND pulse of each ultrasonic being used. Pins on nano BLE taken from lab 5
#define FRONT_ULTRASONIC P1_15
#define LEFT_ULTRASONIC P1_14
#define RIGHT_ULTRASONIC P1_13

class UltraSonic {
public:
  UltraSonic(PinName signalPin);
  // The distance is an integer because there were lots of errors when I tested calculating it as a float; for the purpose it is used for I only need an integer
  int Distance();
  int getCorrection();

private:
  DigitalInOut _signalPin;
  int _Correction;
  Timer _Echo;
};

#endif



