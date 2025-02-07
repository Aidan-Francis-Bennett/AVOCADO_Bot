#include "mbed.h"
#include"Motor.h"
#include "UltraSonic.h"
#include "IR.h"
#include "Movement.h"
using namespace mbed;
using namespace rtos;
// https://www.circuitbasics.com/programming-with-classes-and-objects-on-the-arduino/ was used to help write Motor and Ultrasonic classes

//-------------------------instantiating global variables, constants, and macros-------------------------//
// This boolean value controls if the state machine runs or not.
volatile bool running = true;

// This boolean value detects if it is the first IR reading of the program iteration, to avoid a spurious rate of change value which would cause the robot to instantly turn left.
volatile bool firstIRReading = true;

// These two pointers to memory are used to return arrays from the distanceArray and speedArray functions into that memory, so they will essentially be cast into arrays.
float *distancesLR;
float *speedsLR;

// These arrays are for calculating a rolling average of the speed used later in wall crashing escape logic.
// The initial values are placeholders so the robot doesn't start in an endless loop of wall escaping.
float leftSpeedRollingAverageArray[10] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20}; 
float rightSpeedRollingAverageArray[10] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
float leftSpeedRollingAverage = 20;
float rightSpeedRollingAverage = 20; 

const float desiredWallDistance = 8.0;
const float wallDistanceTolerance = 0.5;
const float leftTurnDistanceChangeTolerance = 40.0;
const int LRUltrasonicThreashold = 5.0;
const int frontUltrasonicThreashold = LRUltrasonicThreashold + 5.0;
const int USONIC_SATURATION = 1000;
const float IRSATURATION = 2.45;

// Variables for the occupancy grid to represent a maze area of 200cm by 160cm, with each square on the map being 1cm^2
char occupancy_grid[200][160];


// These macros define a more readable form of the sensor value array indexes
#define IRLEFT 0
#define IRRIGHT 1
#define USLEFT 0
#define USFRONT 1
#define USRIGHT 2


//-------------------------Motors and encoders-------------------------//

// The left and right motor objects are created using the motor class:
Motor leftMotor(LEFT_DIRECTION, LEFT_PWM);
// Set up an InterruptIn for the encoder B signal which is the left hand motor
InterruptIn leftEncoder(P1_12);

// And the same for the right motor and encoder
Motor rightMotor(RIGHT_DIRECTION, RIGHT_PWM);
InterruptIn rightEncoder(P1_11);

// The timer for measuring speed
Timer speedTimer;

//-------------------------Sensors variables-------------------------//

UltraSonic leftSonic(LEFT_ULTRASONIC);
UltraSonic frontSonic(FRONT_ULTRASONIC);
UltraSonic rightSonic(RIGHT_ULTRASONIC);

// Index 0 is used for the left hand US sensor, index 1 for the front one, and index 2 for the right hand one.
int sonicDistances[3];

// These arrays are used for calculating rolling averages of the left and right US so they can avoid obstacles more accurately. The values initialised are just placeholders before the first readings come in so a spurious turn doesn't occur right at the start of the program.
int leftSonicRollingAverageArray[2] = {100, 100};
int rightSonicRollingAverageArray[2] = {100, 100};
int leftSonicRollingAverage = 100;
int rightSonicRollingAverage = 100;

// Declares where the SCL and SDA are on the chip so read and write operations can be done
I2C i2c(P0_31, P0_2);
// The multiplexer address.
const char mux_addr = 0xEE;
// In this array each I2C bus is defined by bit shifting one bit along the output of the multiplexer to the correct bit representing which sensor it is connected to: the amount the bit is shifted by matches with the number printed next to the physical multiplexer output
const char mux_commands[2] = {1 << 2, 1 << 3};

// Index 0 is used for the right hand IR sensor, and index 1 is the left hand IR sensor
float IRDistances_cm[2];
float previousLeftIRDistance;
float leftIRDistanceRateOfChange;

// These values select which sensor value to print for debugging, as they happen too quickly to print simultaneously.
enum SensorPrint{
  LEFT_US,
  FRONT_US,
  RIGHT_US,
  LEFT_IR,
  RIGHT_IR,
  SPEED_AVERAGE,
  NO
} printSensor;

//-------------------------Thread set up-------------------------//

Thread IRthreadLeft;
Thread IRthreadRight; // Not used in this algorithm, but may be needed for a future version of the code with localisation and mapping.

void IRMeasurementsLeft(){
  while (running) {
    IRDistances_cm[IRLEFT] = IRDistance(mux_addr, mux_commands[1], &i2c);
    if (firstIRReading == true){
      firstIRReading = false;
      previousLeftIRDistance = IRDistances_cm[IRLEFT];
    }
    else{
      leftIRDistanceRateOfChange = abs(IRDistances_cm[IRLEFT]-previousLeftIRDistance)/0.1;
      previousLeftIRDistance = IRDistances_cm[IRLEFT];
      // Serial.println(leftIRDistanceRateOfChange);
    }
    if (printSensor == LEFT_IR){
      Serial.println("Left IR: " + String(IRDistances_cm[0]));
    }
  }
}
void IRMeasurementsRight(){
  while (running) {
    IRDistances_cm[IRRIGHT] = IRDistance(mux_addr, mux_commands[0], &i2c);
    if (printSensor == RIGHT_IR){
      Serial.println("Right IR: " + String(IRDistances_cm[1])); 
    }
  }
}

Thread leftUltrasonicThread;
Thread frontUltrasonicThread;
Thread rightUltrasonicThread;

void leftUltrasonicMeasurements(){
  while (running) {
    sonicDistances[USLEFT] = leftSonic.Distance();
    leftSonicRollingAverageArray[0] = leftSonicRollingAverageArray[1];
    leftSonicRollingAverageArray[1] = sonicDistances[USLEFT];
    leftSonicRollingAverage = (leftSonicRollingAverageArray[0]+leftSonicRollingAverageArray[1])/2;
    if (printSensor == LEFT_US){
      Serial.println("Left US: " + String(sonicDistances[0]));
    }
    ThisThread::sleep_for(150);
  }

}
void frontUltrasonicMeasurements(){
  while (running) {
    sonicDistances[USFRONT] = frontSonic.Distance();
    if (printSensor == FRONT_US){
      Serial.println("Front US: " + String(sonicDistances[1]));
    }
    ThisThread::sleep_for(150);
  }

}
void rightUltrasonicMeasurements(){
  while (running) {
    sonicDistances[USRIGHT] = rightSonic.Distance();
    rightSonicRollingAverageArray[0] = rightSonicRollingAverageArray[1];
    rightSonicRollingAverageArray[1] = sonicDistances[USRIGHT];
    rightSonicRollingAverage = (rightSonicRollingAverageArray[0]+rightSonicRollingAverageArray[1])/2;
    if (printSensor == RIGHT_US){
      Serial.println("Right US: " + String(sonicDistances[2]));
    }
    ThisThread::sleep_for(150);
  }

}

Thread averageSpeedThread;

void averageSpeed(){
  while(running) {
    speedsLR = speedArray();

    // First shift up all the values in the leftSpeedRollingAverageArray, then add them all up for a sum, then calculate that average using this sum.
    for (int i = 0; i < 10; ++i){
      leftSpeedRollingAverageArray[i] = leftSpeedRollingAverageArray[i + 1];
    }
    leftSpeedRollingAverageArray[9] = speedsLR[0];
    float leftSpeedSum = 0.0;
    for (int i = 0; i < 10; ++i) {
      leftSpeedSum += leftSpeedRollingAverageArray[i];
    }
    leftSpeedRollingAverage = leftSpeedSum/10;

    // First shift up all the values in the rightSpeedRollingAverageArray, then add them all up for a sum, then calculate that average using this sum.
    for (int i = 0; i < 10; ++i){
      rightSpeedRollingAverageArray[i] = rightSpeedRollingAverageArray[i + 1];
    }
    rightSpeedRollingAverageArray[9] = speedsLR[1];
    float rightSpeedSum = 0.0;
    for (int i = 0; i < 10; ++i) {
        rightSpeedSum += rightSpeedRollingAverageArray[i];
    }
    rightSpeedRollingAverage = rightSpeedSum/10;

    if (printSensor == SPEED_AVERAGE) {
      Serial.println(String(leftSpeedRollingAverage) + " : " + String(rightSpeedRollingAverage));
    }

    ThisThread::sleep_for(100);
  }
}

//-------------------------Definition of state machine enum variables-------------------------//

enum RobotState {
  STOP,
  FOLLOW_WALL,
  ANTICLOCKWISE_90,
  CLOCKWISE_90, // Not used in this algorithm, but may be needed for a future version of the code with localisation and mapping.
  ESCAPE_ANTICLOCKWISE_22_5,
  ESCAPE_CLOCKWISE_22_5,
  LEFT_CORNER_R_IS_17,
  RIGHT_CORNER_R_IS_17 // Not used in this algorithm, but may be needed for a future version of the code with localisation and mapping.

} currentState;

//-------------------------Setup code body-------------------------//

void setup() {
  Serial.begin(9600);
  // while (!Serial);

  leftMotor.setup();
  leftEncoder.rise(&leftEncoderCounter);
  leftEncoder.fall(&leftEncoderCounter);

  rightMotor.setup();
  rightEncoder.rise(&rightEncoderCounter);
  rightEncoder.fall(&rightEncoderCounter);

  speedTimer.start();

// Start the Left and right IR threads with half a cycle pause so that they run in an even time interval sequence.
  IRthreadLeft.start(callback(IRMeasurementsLeft));
  ThisThread::sleep_for(50);
  IRthreadRight.start(callback(IRMeasurementsRight)); // Not used in this algorithm, but may be needed for a future version of the code with localisation and mapping.

// Start the left, front, and right ultrasonic sensors a third of a cycle apart so they don't interfere with each other.
  leftUltrasonicThread.start(callback(leftUltrasonicMeasurements));
  ThisThread::sleep_for(50);
  frontUltrasonicThread.start(callback(frontUltrasonicMeasurements));
  ThisThread::sleep_for(50);
  rightUltrasonicThread.start(callback(rightUltrasonicMeasurements));

  averageSpeedThread.start(callback(averageSpeed));

  currentState = FOLLOW_WALL;
  printSensor = NO;

}

//-------------------------Loop code body-------------------------//

void loop() {
  while (running) {
    switch (currentState){
    case STOP:
      stop();
      if (sonicDistances[USFRONT] > frontUltrasonicThreashold){
        currentState = FOLLOW_WALL;
      }
      else {
        currentState = CLOCKWISE_90;
      }
      break;

    case FOLLOW_WALL:
      if (sonicDistances[USFRONT] > frontUltrasonicThreashold) {
        if ((leftIRDistanceRateOfChange > leftTurnDistanceChangeTolerance) && ( (IRDistances_cm[IRLEFT] > 1.5*desiredWallDistance) || (IRDistances_cm[IRLEFT] == IRSATURATION) ) || (IRDistances_cm[IRLEFT] > 60.0) ){
          currentState = LEFT_CORNER_R_IS_17;
        }
        // This logic checks if the left and right US sensors are too close to a wall, or have saturated as they are right up against a wall, and turns 45 degrees away from the wall if so.
        // It will also change to the escaping state if either of the rotary encoders detect a speed of less than 1cm per second, as this suggests that the robot is driving into a wall and needs to reverse and turn.
        if ((leftSonicRollingAverage < LRUltrasonicThreashold) || (leftSonicRollingAverage > USONIC_SATURATION)  ) {
          currentState = ESCAPE_CLOCKWISE_22_5;
        }
        else if ((rightSonicRollingAverage < LRUltrasonicThreashold) || (rightSonicRollingAverage > USONIC_SATURATION) || (leftSpeedRollingAverage < 1) || (rightSpeedRollingAverage < 1)) {
          currentState = ESCAPE_ANTICLOCKWISE_22_5;
        }
        else {
          if (IRDistances_cm[IRLEFT] > desiredWallDistance + wallDistanceTolerance){
            // Turns left with a large turning circle, to nudge towards the wall, while the IR sensor distance is greater than the upper desired wall threashold, and the front ultrasonic sensor distance is below the threashold
            leftTurnLargeR();
            // Serial.println("Towards wall");
          }
          // Turns away from the wall if closer than the wall distance tolerance, which includes if the sensor saturates as then the value will be 2.45
          else if (IRDistances_cm[IRLEFT] < desiredWallDistance - wallDistanceTolerance){
            rightTurnLargeR();
            // Serial.println("Away from wall");
          }
          else {
            // Goes forwards while IRdistances are less than the upper bound of desired wall distance, and greater than the lower bound of wall distance tolerance, and the front ultrasonic sensor is below it's threashold
            forwardsCalib(16);
            // Serial.println("straight ahead");
          }
        }  
      } 
      else{
        // Serial.println("The robot should have stopped");
        currentState = STOP;
      }
      break;

    case ANTICLOCKWISE_90:
      // Serial.println("anticlockwise 90");
      antiClockwiseCalib(HALF_PI, 1000000);
      currentState= FOLLOW_WALL;
      break;

    case CLOCKWISE_90:
      // Serial.println("clockwise 90");
      clockwiseCalib(HALF_PI, 1000000);
      currentState = FOLLOW_WALL;
      break;
    
    case ESCAPE_ANTICLOCKWISE_22_5:
      // Serial.println("anticlockwise 22.5");  
      backwardsCalib(8);
      wait_us(200000);    
      antiClockwiseCalib(HALF_PI/4.0f, 500000);
      currentState = FOLLOW_WALL;
      break;

    case ESCAPE_CLOCKWISE_22_5:
      // Serial.println("clockwise 22.5");
      backwardsCalib(8);
      wait_us(200000);  
      clockwiseCalib(HALF_PI/4.0f, 500000);
      currentState = FOLLOW_WALL;
      break;

    case LEFT_CORNER_R_IS_17:
      leftTurnR_is_17cm(HALF_PI);
      forwardsCalib(16);
      wait_us(500000);
      // Serial.println("Left turn turning circle is 17cm");
      currentState = FOLLOW_WALL;
      break;

    case RIGHT_CORNER_R_IS_17:
      rightTurnR_is_17cm(HALF_PI);
      break;
    } 
  }
  leftUltrasonicThread.join();
  frontUltrasonicThread.join();
  rightUltrasonicThread.join();
  IRthreadLeft.join();
  IRthreadRight.join();
  averageSpeedThread.join();
}
