#include <Arduino.h>
#include "digitalWriteFast.h"
/**
 * Hardware pin defines
 */
#define BOARD UKMARSBOT_V1
const int ENCODER_LEFT_CLK = 2;
const int ENCODER_RIGHT_CLK = 3;
const int ENCODER_LEFT_B = 4;
const int ENCODER_RIGHT_B = 5;
const int MOTOR_LEFT_DIR = 7;
const int MOTOR_RIGHT_DIR = 8;
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;
const int LED_RIGHT = 6;
const int LED_LEFT = 11;
const int EMITTER = 12;
const int SENSOR_0 = A0;
const int SENSOR_1 = A1;
const int SENSOR_2 = A2;
const int SENSOR_3 = A3;
const int SENSOR_4 = A4;
const int SENSOR_5 = A5;
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;
const float MAX_MOTOR_VOLTS = 6.0f;
/****/

// Global variables for sensor values and walls
// the default values for the front sensor when the robot is backed up to a wall
const int FRONT_REFERENCE = 44;
// the default values for the side sensors when the robot is centred in a cell
const int LEFT_REFERENCE = 38;
const int RIGHT_REFERENCE = 49;

// the values above which, a wall is seen
const int FRONT_WALL_THRESHOLD = FRONT_REFERENCE / 20;  // minimum value to register a wall
const int LEFT_WALL_THRESHOLD = LEFT_REFERENCE / 2;     // minimum value to register a wall
const int RIGHT_WALL_THRESHOLD = RIGHT_REFERENCE / 2;   // minimum value to register a wall

// working copies of the reference values
int gFrontReference = FRONT_REFERENCE;
int gLeftReference = LEFT_REFERENCE;
int gRightReference = RIGHT_REFERENCE;
// the current value of the sensors
volatile int gSensorFront;
volatile int gSensorLeft;
volatile int gSensorRight;
// true f a wall is present
volatile bool gFrontWall;
volatile bool gLeftWall;
volatile bool gRightWall;
// steering and turn position errors
volatile int gSensorFrontError;   // zero when robot in cell centre
volatile float gSensorCTE;  // zero when robot in cell centre
volatile int32_t encoderLeftCount;
volatile int32_t encoderRightCount;

// Timing variables
uint32_t updateTime;
uint32_t updateInterval = 40;  // (ms)


const float batteryDividerRatio = 2.0f;
float gBatteryVolts;
float getBatteryVolts() {
  int adcValue = analogRead(BATTERY_VOLTS);
  gBatteryVolts = adcValue * (5.0f * batteryDividerRatio / 1023.0f);
  return gBatteryVolts;
}

void setupEncoder() {
    // left
  pinMode(ENCODER_LEFT_CLK, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);
  // configure the pin change
  bitClear(EICRA, ISC01);
  bitSet(EICRA, ISC00);
  // enable the interrupt
  bitSet(EIMSK, INT0);
  encoderLeftCount = 0;
  // right
  pinMode(ENCODER_RIGHT_CLK, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  // configure the pin change
  bitClear(EICRA, ISC11);
  bitSet(EICRA, ISC10);
  // enable the interrupt
  bitSet(EIMSK, INT1);
  encoderRightCount = 0;
}

void analogueSetup() {
  // increase speed of ADC conversions to 28us each
  // by changing the clock prescaler from 128 to 16
  bitClear(ADCSRA, ADPS0);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);
}

ISR(INT0_vect) {
  static bool oldB = 0;
  bool newB = bool(digitalReadFast(ENCODER_LEFT_B));
  bool newA = bool(digitalReadFast(ENCODER_LEFT_CLK)) ^ newB;
  if (newA == oldB) {
    encoderLeftCount--;
  } else {
    encoderLeftCount++;
  }
  oldB = newB;
}

ISR(INT1_vect) {
  static bool oldB = 0;
  bool newB = bool(digitalReadFast(ENCODER_RIGHT_B));
  bool newA = bool(digitalReadFast(ENCODER_RIGHT_CLK)) ^ newB;
  if (newA == oldB) {
    encoderRightCount++;
  } else {
    encoderRightCount--;
  }
  oldB = newB;
}

// Timer interrupt for sensor updates
ISR(TIMER2_COMPA_vect) {
  updateWallSensor();
}


void updateWallSensor() {
  // first read them dark
  int right = analogRead(A0);
  int front = analogRead(A1);
  int left = analogRead(A2);
  // light them up
  digitalWrite(EMITTER, 1);
  // wait until all the detectors are stable
  delayMicroseconds(50);
  // now find the differences
  right = analogRead(A0) - right;
  front = analogRead(A1) - front;
  left = analogRead(A2) - left;
  // and go dark again.
  digitalWrite(EMITTER, 0);

  gFrontWall = front > gFrontReference / 4;
  gLeftWall = left > gLeftReference / 2;
  gRightWall = right > gRightReference / 2;
  digitalWrite(LED_LEFT, gLeftWall);
  digitalWrite(LED_RIGHT, gRightWall);
  digitalWrite(LED_BUILTIN, gFrontWall);
 // digitalWrite(LED_LEFT,  gFrontWall);
 // digitalWrite(LED_RIGHT,  gFrontWall);
  // calculate the alignment error - too far right is negative
  if ((left + right) > (gLeftReference + gRightReference) / 4) {
    if (left > right) {
      gSensorCTE = (left - LEFT_REFERENCE);
      gSensorCTE /= left;
    } else {
      gSensorCTE = (RIGHT_REFERENCE - right);
      gSensorCTE /= right;
    }
  } else {
    gSensorCTE = 0;
  }
  // make the results available to the rest of the program
  gSensorLeft= left;
  gSensorRight = right;
  gSensorFront = front;
}



void setupSystick() {
  bitClear(TCCR2A, WGM20);
  bitSet(TCCR2A, WGM21);
  bitClear(TCCR2B, WGM22);
  bitSet(TCCR2B, CS22);
  bitClear(TCCR2B, CS21);
  bitSet(TCCR2B, CS20);
  OCR2A = 249;
  bitSet(TIMSK2, OCIE2A);
}

void motorSetup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  digitalWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR, 0);
  digitalWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_DIR, 0);
}

void setLeftMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_LEFT_DIR, 0);
    analogWrite(MOTOR_LEFT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, 1);
    analogWrite(MOTOR_LEFT_PWM, pwm);
  }
}

void setRightMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_RIGHT_DIR, 0);
    analogWrite(MOTOR_RIGHT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, 1);
    analogWrite(MOTOR_RIGHT_PWM, pwm);
  }
}

void setMotorPWM(int left, int right) {
  setLeftMotorPWM(left);
  setRightMotorPWM(right);
}

void setLeftMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setLeftMotorPWM(motorPWM);
}

void setRightMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setRightMotorPWM(motorPWM);
}

void setMotorVolts(float left, float right) {
  setLeftMotorVolts(left);
  setRightMotorVolts(right);
}

void setup() {
  Serial.begin(9600);
  digitalWrite(EMITTER, 0);  // Turn off emitter initially
  pinMode(EMITTER, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  setupEncoder();
  motorSetup();
  analogueSetup();           // increase the ADC conversion speed
  setupSystick();
  updateTime = millis() + updateInterval;
}

void loop() {

  
  if (millis() > updateTime) {
    updateTime += updateInterval;
   // int adcValue = analogRead(BATTERY_VOLTS);
    getBatteryVolts();
  
    
 // Convert the three boolean values into a single integer value (binary representation)
  int inputValue = (gFrontWall << 2) | (gLeftWall << 1) | gRightWall;

  switch (inputValue) {
    case 0: // input1 = 0, input2 = 0, input3 = 0
      setMotorVolts(2, 2); // go forward
      break;
    case 1: // input1 = 0, input2 = 0, input3 = 1
      setMotorVolts(-2, 2);  // turn left
      break;
    case 2: // input1 = 0, input2 = 1, input3 = 0
      setMotorVolts(2, -2);  //turn right
      break;
    case 3: // input1 = 0, input2 = 1, input3 = 1
      setMotorVolts(2, 2); // go forward
      break;
    case 4: // input1 = 1, input2 = 0, input3 = 0
      setMotorVolts(-2, 2);  // turn left
      break;
    case 5: // input1 = 1, input2 = 0, input3 = 1
      setMotorVolts(-2, 2);  // turn left
      break;
    case 6: // input1 = 1, input2 = 1, input3 = 0
      setMotorVolts(2, -2);  //turn right
      break;
    case 7: // input1 = 1, input2 = 1, input3 = 1
      setMotorVolts(1, -4);  //turn around
      break;
    default:
     // Serial.println("Invalid input combination.");
      setMotorVolts(0, 0); // stop
      break;
  }



    // Debugging information
    Serial.print("Right: ");
    Serial.print(gSensorRight);
    Serial.print("  Front: ");
    Serial.print(gSensorFront);
    Serial.print("  Left: ");
    Serial.print(gSensorLeft);
    Serial.print("  Error: ");
    Serial.print(gSensorCTE);
    Serial.println();
  }
}
