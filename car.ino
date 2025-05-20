#include "ECE3.h"

const int LED_RF = 41;

#define NUM_SENSORS 8
#define KP 0.005
#define ERROR_TERM_TOLERANCE 200

#define TURN_COUNTS 3
#define TURN_TIMEOUT_MILLIS 300
#define BLACK_LINE_TRESHOLD 1600
#define SPLIT_BLACK_LINE_COUNT 3

#define LEFT_NSLP_PIN 31 // nslp ==> awake & ready for PWM
#define LEFT_DIR_PIN 29
#define LEFT_PWN_PIN 40
#define RIGHT_NSLP_PIN 11
#define RIGHT_DIR_PIN 30
#define RIGHT_PWM_PIN 39

#define MAX_ERROR 15000
#define BASE_SPEED 30
#define REVERSE_SPEED 20
#define REVERSE_COUNTER_TRIGGER 10

// #define IS_DEBUG 

uint16_t sensorValues[8];
uint16_t minTerms[] = {805, 728, 711, 688, 640, 758, 734, 805};
uint16_t maxDiff[] = {1695, 1772, 1789, 1254, 1515, 1742, 1766, 1695};
float normalizedValues[NUM_SENSORS];
float resultValues[NUM_SENSORS];

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
  delay(20); //Wait 2 seconds before starting 
  // Serial.println("BEGIN");
  
  updateLeftWheelSpeed(30);
  updateRightWheelSpeed(30);
}

void loop() {
  // while (!Serial.available()) {}

  ECE3_read_IR(sensorValues);

  float sensorTotal = getNormalizedValues(normalizedValues);
  #ifdef IS_DEBUG
  Serial.println(sensorTotal);
  #endif

  // go left when heading out on the split, go right when heading back on the split
  if (isSplit()) {
    #ifdef IS_DEBUG
    Serial.println("THIS IS THE SPLIT");
    #endif

    turnLeft();
  }

  float errorTerm = calculate1514128Error(normalizedValues);

  #ifdef IS_DEBUG
  Serial.println(errorTerm);
  #endif

  updateRightWheelSpeed(errorTerm);
  updateLeftWheelSpeed(errorTerm);

  #ifdef IS_DEBUG
  // delay(1000);
  #endif
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
float calculate1514128Error(float normalized_values[]) {  
  return (
    normalized_values[0] * -15 +
    normalized_values[1] * -14 +
    normalized_values[2] * -12 +
    normalized_values[3] * -8 +
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

  wheel_speed.leftSpeed = BASE_SPEED + KP * errorTerm;

  if (errorTerm < 0) { // if on left side
    digitalWrite(LEFT_DIR_PIN, LOW); // turn left wheel backwards
  } else {
    leftReverseCounter += 1;

    if (leftReverseCounter >= REVERSE_COUNTER_TRIGGER) {
      digitalWrite(LEFT_DIR_PIN, HIGH);
      wheel_speed.leftSpeed = REVERSE_SPEED;
    }
  }

  analogWrite(LEFT_PWN_PIN, wheel_speed.leftSpeed);
}

void updateRightWheelSpeed(float errorTerm) {
  if (abs(errorTerm) <= ERROR_TERM_TOLERANCE) {
    digitalWrite(RIGHT_DIR_PIN, LOW);

    #ifdef IS_DEBUG
    Serial.println("Exiting because margin not high enough");
    #endif

    return;
  }

  wheel_speed.rightSpeed = BASE_SPEED + KP * errorTerm;

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

  analogWrite(RIGHT_PWM_PIN, wheel_speed.rightSpeed);
}

bool isSplit() {
  // detect split by checking how many sensors indicate that there is a black line
  #ifdef IS_DEBUG
  Serial.println("SENSOR VALUES");
  for (int i = 0; i < NUM_SENSORS; ++i) {
    Serial.println((float)sensorValues[i]);
  }
  Serial.println("SENSOR VALUES END");
  #endif

  uint8_t blackLineCount = 0;
  for (int i = 0; i < NUM_SENSORS; ++i) {
    if ((float)sensorValues[i] > BLACK_LINE_TRESHOLD) {
      ++blackLineCount;
    }
  }

  return blackLineCount >= SPLIT_BLACK_LINE_COUNT;
}

void turnLeft() {
  #ifdef IS_DEBUG
  Serial.println("Making manual left turn");
  #endif

  delay(1000);

  // get the intial state of the pins
  uint8_t originalLeftPinDir = digitalRead(LEFT_DIR_PIN);
  uint8_t originalRightPinDir = digitalRead(RIGHT_DIR_PIN);
  
  // turn left
  digitalWrite(LEFT_DIR_PIN, HIGH);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  unsigned long start_time = millis();

  // turn left for 0.5 seconds
  while (millis() - start_time < TURN_TIMEOUT_MILLIS) {
    analogWrite(RIGHT_PWM_PIN, wheel_speed.rightSpeed);
    analogWrite(LEFT_PWN_PIN, wheel_speed.leftSpeed);
  }

  // restore the pins to their original state
  digitalWrite(LEFT_DIR_PIN, originalLeftPinDir);
  digitalWrite(RIGHT_DIR_PIN, originalRightPinDir);
}
