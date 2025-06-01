#include "ECE3.h"

// this version is slow, but works!

const int LED_RF = 41;

#define NUM_SENSORS 8
#define KP 0.015
#define KD 0.15

#define TURN_COUNTS 3
#define TURN_TIMEOUT_MILLIS 500
#define BLACK_LINE_TRESHOLD 1600
#define SPLIT_BLACK_LINE_COUNT 3
#define IS_END 2300

#define LEFT_NSLP_PIN 31 // nslp ==> awake & ready for PWM
#define LEFT_DIR_PIN 29
#define LEFT_PWN_PIN 40
#define RIGHT_NSLP_PIN 11
#define RIGHT_DIR_PIN 30
#define RIGHT_PWM_PIN 39

#define MAX_ERROR 15000
#define BASE_SPEED 10
#define REVERSE_SPEED 20
#define REVERSE_COUNTER_TRIGGER 1
#define START_TIME_STRAIGHT 1000

uint16_t sensorValues[8];
uint16_t minTerms[] = {805, 728, 711, 688, 640, 758, 734, 805};
uint16_t maxDiff[] = {1695, 1772, 1789, 1254, 1515, 1742, 1766, 1695};
float floatSensorValues[NUM_SENSORS];
float normalizedValues[NUM_SENSORS];
float previousErrorTerm = 0;
int currentDirection = 0; // 0 for up, 1 for down

uint8_t rightReverseCounter = 0;
uint8_t leftReverseCounter = 0;

unsigned long start_time = 0;

struct WheelSpeed {
  int rightSpeed;
  int leftSpeed;
} wheel_speed = {
  30, 30
};

///////////////////////////////////
void setup() {
  pinMode(LED_RF, OUTPUT);

  pinMode(LEFT_NSLP_PIN,OUTPUT);
  pinMode(LEFT_DIR_PIN,OUTPUT);
  pinMode(LEFT_PWN_PIN,OUTPUT);

  digitalWrite(LEFT_DIR_PIN,LOW);

  // #ifndef IS_DEBUG
  digitalWrite(LEFT_NSLP_PIN,HIGH);
  // #endif

  pinMode(RIGHT_NSLP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  
  digitalWrite(RIGHT_DIR_PIN, LOW);

  // #ifndef IS_DEBUG
  digitalWrite(RIGHT_NSLP_PIN, HIGH);
  // #endif

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();

  // set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 

  analogWrite(LEFT_PWN_PIN, 40);
  analogWrite(RIGHT_PWM_PIN, 40);

  start_time = millis();
}

void loop() {
  #ifdef IS_DEBUG
  delay(1000);
  #endif

  ECE3_read_IR(sensorValues);

  // get sensorvalues as a float
  for (int i = 0; i < NUM_SENSORS; ++i) {
    floatSensorValues[i] = (float)sensorValues[i];
  }

  if (isEnd()) {
    turnAroundAndMoveForward();
    currentDirection = 1;
    return;
  }

  digitalWrite(RIGHT_NSLP_PIN,HIGH);
  digitalWrite(LEFT_NSLP_PIN,HIGH);

  float sensorTotal = getNormalizedValues(normalizedValues);

  float errorTerm = 0;
  // if (millis() - start_time < START_TIME_STRAIGHT) {
  //   errorTerm = calculateRegularError(normalizedValues);
  // }
  if (true) {
    errorTerm = calculate1514128Error(normalizedValues);
  }

  #ifdef IS_DEBUG
  Serial.println(errorTerm);
  #endif

  updateRightWheelSpeed(errorTerm);
  updateLeftWheelSpeed(errorTerm);

  previousErrorTerm = errorTerm;
}

float getNormalizedValues(float results[]) {
  float sum = 0;
  for (int i =0; i < NUM_SENSORS; ++i) {
    results[i] = (sensorValues[i] - minTerms[i]) * 1000 / maxDiff[i];
    sum += sensorValues[i];

    #ifdef IS_DEBUG
    Serial.println(sum);
    #endif
  }

  return sum;
}

// when it veers off to the right, the error is negative
// when it veers off to the left, the error is positive

// altering weighting scheme works for split back down the track
// altering weighting scheme works for arch back down the track too
// car works fine for the esses as well
float calculate1514128Error(float normalized_values[]) {  
    if (currentDirection == 0) {
        return (
            normalized_values[0] * -15 +
            normalized_values[1] * -14 +
            normalized_values[2] * -12 +
            normalized_values[3] * -8 +
            normalized_values[4] * 8 * 1.9 +
            normalized_values[5] * 12 * 1.9 +
            normalized_values[6] * 14 * 1.9 +
            normalized_values[7] * 15 * 1.9
        ) / 8;
    }
  
    return (
    normalized_values[0] * -15 * 1.9 +
    normalized_values[1] * -14 * 1.9 +
    normalized_values[2] * -12 * 1.9+
    normalized_values[3] * -8 * 1.9 +
    normalized_values[4] * 8 +
    normalized_values[5] * 12 +
    normalized_values[6] * 14 +
    normalized_values[7] * 15 
  ) / 8;
}

float calculateRegularError(float normalized_values[]) {
    return (normalized_values[0] * -15 +
            normalized_values[1] * -14 +
            normalized_values[2] * -12 +
            normalized_values[3] * -8 +
            normalized_values[4] * 8 +
            normalized_values[5] * 12 +
            normalized_values[6] * 14 +
            normalized_values[7] * 15
        / 8);
}

// when veering off to the left, this becomes positive
// when veering off to the right, this becomes negative
void updateLeftWheelSpeed(float errorTerm) {
  wheel_speed.leftSpeed = BASE_SPEED - KP * errorTerm - KD * (errorTerm - previousErrorTerm);

  if (wheel_speed.leftSpeed > 0) { // if on left side
    digitalWrite(LEFT_DIR_PIN, LOW); // turn left wheel backwards
  } else {
              digitalWrite(LEFT_DIR_PIN, HIGH);
  }

  analogWrite(LEFT_PWN_PIN, max(30, min(abs(wheel_speed.leftSpeed), 130)));
}

void updateRightWheelSpeed(float errorTerm) {

  wheel_speed.rightSpeed = BASE_SPEED + KP * errorTerm + KD * (errorTerm - previousErrorTerm);

  if (wheel_speed.rightSpeed > 0) { // if on right side
    rightReverseCounter = 0;
    digitalWrite(RIGHT_DIR_PIN, LOW);
  } 
  else {
  
      digitalWrite(RIGHT_DIR_PIN, HIGH);
  }

  analogWrite(RIGHT_PWM_PIN, max(30, min(abs(wheel_speed.rightSpeed), 130)));
}

bool isEnd() {
  float total = 0;
  for (int i = 0; i < NUM_SENSORS; ++i) {
      total += floatSensorValues[i];
  }

  // return total >= 16800;

  if (total < 16800) {
    return false;
  }

  digitalWrite(RIGHT_NSLP_PIN,LOW);
  digitalWrite(LEFT_NSLP_PIN,LOW);
  delay(500);

  ECE3_read_IR(sensorValues);

  // get sensorvalues as a float
  for (int i = 0; i < NUM_SENSORS; ++i) {
    floatSensorValues[i] = (float)sensorValues[i];
  }

  total = 0;
  for (int i = 0; i < NUM_SENSORS; ++i) {
      total += floatSensorValues[i];
  }

  return total >= 16800;
}

void turnAroundAndMoveForward() {
    digitalWrite(RIGHT_NSLP_PIN,HIGH);
    digitalWrite(LEFT_NSLP_PIN,HIGH);

    digitalWrite(RIGHT_DIR_PIN, HIGH);
    digitalWrite(LEFT_DIR_PIN,LOW);

    analogWrite(LEFT_PWN_PIN, 60);
    analogWrite(RIGHT_PWM_PIN, 60);
    delay(800);

    analogWrite(LEFT_PWN_PIN, 30);
    analogWrite(RIGHT_PWM_PIN, 30);
    digitalWrite(RIGHT_DIR_PIN,LOW);
    digitalWrite(LEFT_DIR_PIN,LOW);
}

