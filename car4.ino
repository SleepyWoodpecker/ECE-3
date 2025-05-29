#include "ECE3.h"

const int LED_RF = 41;

#define NUM_SENSORS 8
#define KP 0.001
#define KD 0.01
#define ERROR_TERM_TOLERANCE 200

#define TURN_COUNTS 3
#define TURN_TIMEOUT_MILLIS 500
#define BLACK_LINE_TRESHOLD 1600
#define SPLIT_BLACK_LINE_COUNT 3
#define IS_END 2500

#define LEFT_NSLP_PIN 31 // nslp ==> awake & ready for PWM
#define LEFT_DIR_PIN 29
#define LEFT_PWN_PIN 40
#define RIGHT_NSLP_PIN 11
#define RIGHT_DIR_PIN 30
#define RIGHT_PWM_PIN 39

#define MAX_ERROR 15000
#define BASE_SPEED 50
#define REVERSE_SPEED 20
#define REVERSE_COUNTER_TRIGGER 3

uint16_t sensorValues[8];
uint16_t minTerms[] = {805, 728, 711, 688, 640, 758, 734, 805};
uint16_t maxDiff[] = {1695, 1772, 1789, 1254, 1515, 1742, 1766, 1695};
float floatSensorValues[NUM_SENSORS];
float normalizedValues[NUM_SENSORS];
float previousErrorTerm = 0;
int currentDirection = 0; // 0 for up, 1 for down

uint8_t rightReverseCounter = 0;
uint8_t leftReverseCounter = 0;

struct WheelSpeed {
  int rightSpeed;
  int leftSpeed;
} wheel_speed = {
  30, 30
};

///////////////////////////////////
void setup() {
  pinMode(LED_RF, OUTPUT);

// put your setup code here, to run once:
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
  delay(200); //Wait 2 seconds before starting 
  // Serial.println("BEGIN");

  analogWrite(LEFT_PWN_PIN, 40);
  analogWrite(RIGHT_PWM_PIN, 40);
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

  float sensorTotal = getNormalizedValues(normalizedValues);
  float errorTerm = calculate1514128Error(normalizedValues);

  #ifdef IS_DEBUG
  Serial.println(errorTerm);
  #endif

  updateRightWheelSpeed(errorTerm);
  updateLeftWheelSpeed(errorTerm);

  previousErrorTerm = errorTerm;

  // I should check how straight its path is and debug from there
  // delay(1);
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
        );
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

// when veering off to the left, this becomes positive
// when veering off to the right, this becomes negative
void updateLeftWheelSpeed(float errorTerm) {
  
  // based on experimentation, the tolerable error term is about 200
  if (abs(errorTerm) <= ERROR_TERM_TOLERANCE) {
    digitalWrite(RIGHT_DIR_PIN, LOW);

    #ifdef IS_DEBUG
    Serial.println("Exiting because margin not high enough");
    #endif

    return;
  }

  wheel_speed.leftSpeed = BASE_SPEED + KP * errorTerm + KD * (errorTerm - previousErrorTerm) / 6;

  if (errorTerm < 0) { // if on left side
    digitalWrite(LEFT_DIR_PIN, LOW); // turn left wheel backwards
  } else {
    leftReverseCounter += 1;

    if (leftReverseCounter >= REVERSE_COUNTER_TRIGGER) {
      digitalWrite(LEFT_DIR_PIN, HIGH);
      wheel_speed.leftSpeed = REVERSE_SPEED;
    }
  }

  analogWrite(LEFT_PWN_PIN, min(30, max(150, wheel_speed.leftSpeed)));
}

void updateRightWheelSpeed(float errorTerm) {
  if (abs(errorTerm) <= ERROR_TERM_TOLERANCE) {
    digitalWrite(RIGHT_DIR_PIN, LOW);

    #ifdef IS_DEBUG
    Serial.println("Exiting because margin not high enough");
    #endif

    return;
  }

  wheel_speed.rightSpeed = BASE_SPEED + KP * errorTerm + KD * (errorTerm - previousErrorTerm) / 6;

  if (errorTerm > 0) { // if on right side
    rightReverseCounter = 0;
    digitalWrite(RIGHT_DIR_PIN, LOW);
  } 
  else {
    rightReverseCounter += 1;

    if (rightReverseCounter >= REVERSE_COUNTER_TRIGGER) {
      digitalWrite(RIGHT_DIR_PIN, HIGH);
      wheel_speed.rightSpeed = REVERSE_SPEED;
    }
  }

  analogWrite(RIGHT_PWM_PIN, min(30, max(150, wheel_speed.rightSpeed)));
}

bool isEnd() {
    for (int i = 0; i < NUM_SENSORS; ++i) {
        if (floatSensorValues[i] != IS_END) {
            return false;
        }
    }

    return true;
}

void turnAroundAndMoveForward() {
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