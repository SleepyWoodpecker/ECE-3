#include "ECE3.h"

const int LED_RF = 41;

#define NUM_SENSORS 8
#define KP 0.003
#define KD 0.06
#define ERROR_TERM_TOLERANCE 200

#define TURN_COUNTS 3
#define TURN_TIMEOUT_MILLIS 500
#define BLACK_LINE_TRESHOLD 1600
#define SPLIT_BLACK_LINE_COUNT 3

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

  updateLeftWheelSpeed(30);
  updateRightWheelSpeed(30);
}

void loop() {
  #ifdef IS_DEBUG
  delay(1000);
  #endif

  ECE3_read_IR(sensorValues);

  // go left when heading out on the split, go right when heading back on the split
  if (isDoubleLine() || isSplit()) {
    digitalWrite(LED_RF, HIGH);
    removeExtraSide();
  } else {
    digitalWrite(LED_RF, LOW);
  }

  // get sensorvalues as a float
  for (int i = 0; i < NUM_SENSORS; ++i) {
    floatSensorValues[i] = (float)sensorValues[i];
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

  analogWrite(RIGHT_PWM_PIN, wheel_speed.rightSpeed);
}

/*
Based on experimentation, the determined unique pattern for what a split looks like is the followig:
Pattern for split:
0.     |  1.    | 2.      | 3       | 4       |     5        | 6          | 7
827.00 | 746.00 | 1351.00 | 2071.00 | 1605.00 | 2213.00 | 827.00 | 827.00 | SENSOR VALUES END
823.00 | 732.00 | 1192.00 | 2289.00 | 1659.00 | 1916.00 | 754.00 | 754.00 | SENSOR VALUES END
840.00 | 748.00 | 1171.00 | 2359.00 | 1611.00 | 1797.00 | 771.00 | 771.00 | SENSOR VALUES END
860.00 | 795.00 | 1159.00 | 2374.00 | 1623.00 | 1953.00 | 818.00 | 841.00 | SENSOR VALUES END


This was to avoid the turning behavior at the following other components:
Pattern for ESSES:
719.00 | 627.00 | 627.00 | 787.00 | 1606.00 | 2500.00 | 1606.00 | 1064.00 | SENSOR VALUES END -> at the end of the ESS
769.00 | 746.00 | 769.00 | 1067.00 | 2120.00 | 2500.00 | 1618.00 | 1160.00 | SENSOR VALUES END -> at the end of the ESS
875.00 | 806.00 | 806.00 | 1127.00 | 2326.00 | 2466.00 | 1604.00 | 1358.00 | SENSOR VALUES END -> at the end of the ESS
687.00 | 595.00 | 573.00 | 687.00 | 1639.00 | 2500.00 | 1603.00 | 1418.00 | SENSOR VALUES END


Pattern for arch:
2500.00 | 907.00   | 792.00  | 815.00   | 1666.00 | 1620.00 | 838.00 | 838.00 | SENSOR VALUES END
2500.00 | 898.00   | 806.00 | 829.00  | 1694.00 | 1624.00 | 806.00 | 783.00 | SENSOR VALUES END
2270.00 | 875.00    | 760.00 | 806.00 | 1614.00 | 1638.00 | 829.00 | 852.00 | SENSOR VALUES END
1890.00 | 1657.00 | 706.00 | 683.00 | 683.00 | 2078.00 | 1379.00 | 775.00 | SENSOR VALUES END
2074.00 | 1632.00 | 737.00 | 692.00 | 737.00 | 2424.00 | 1198.00 | 783.00 | SENSOR VALUES END
2027.00 | 1607.00 | 680.00 | 702.00 | 771.00 | 2500.00 | 863.00 | 748.00 | SENSOR VALUES END
*/
bool isSplit() {
  return floatSensorValues[0] < BLACK_LINE_TRESHOLD && 
  floatSensorValues[1] < BLACK_LINE_TRESHOLD && 
  floatSensorValues[2] < BLACK_LINE_TRESHOLD &&
  floatSensorValues[3] >= BLACK_LINE_TRESHOLD && 
  floatSensorValues[4] >= BLACK_LINE_TRESHOLD && 
  floatSensorValues[5] >= BLACK_LINE_TRESHOLD && 
  floatSensorValues[6] < BLACK_LINE_TRESHOLD && 
  floatSensorValues[7] < BLACK_LINE_TRESHOLD;
}

// to speed it up, just get the unsigned representation for the treshold values
bool isDoubleLine() {
  return (
    floatSensorValues[0] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[1] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[2] < BLACK_LINE_TRESHOLD &&
    floatSensorValues[3] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[4] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[5] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[6] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[7] < BLACK_LINE_TRESHOLD
  ) ^ 
  (
    floatSensorValues[0] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[1] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[2] < BLACK_LINE_TRESHOLD &&
    floatSensorValues[3] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[4] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[5] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[6] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[7] < BLACK_LINE_TRESHOLD
  ) ^ 
  (
    floatSensorValues[0] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[1] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[2] < BLACK_LINE_TRESHOLD &&
    floatSensorValues[3] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[4] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[5] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[6] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[7] < BLACK_LINE_TRESHOLD
  ) ^ 
  (
    floatSensorValues[0] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[1] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[2] < BLACK_LINE_TRESHOLD &&
    floatSensorValues[3] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[4] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[5] < BLACK_LINE_TRESHOLD && 
    floatSensorValues[6] >= BLACK_LINE_TRESHOLD && 
    floatSensorValues[7] < BLACK_LINE_TRESHOLD
  );
}

void removeExtraSide() {
  for (int i = 0; i < 4; ++i) {
    sensorValues[i] = 650;
  }
}

void turnLeft() {
  #ifdef IS_DEBUG
  Serial.println("Making manual left turn");
  #endif

  digitalWrite(LEFT_NSLP_PIN,LOW);
  digitalWrite(RIGHT_NSLP_PIN,LOW);

  delay(3000);

  digitalWrite(LEFT_NSLP_PIN,HIGH);
  digitalWrite(RIGHT_NSLP_PIN,HIGH);

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

  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  updateLeftWheelSpeed(30);
  updateRightWheelSpeed(30);
  delay(TURN_TIMEOUT_MILLIS - 200);
}

void blinkYellowLED() {
  digitalWrite(LED_RF, HIGH);
}

/*
Issues with car:
1. Cannot start properly on points that aren’t 1
2. Cannot detect the split / arch, though there may be a solution for that
3. Donut at the end
*/

/*
I think that the split and arch detection should be the key here
Sometimes the car also stalls, but idk why either
*/

/*
Priorities for tmr:
  1. detect that the split/arch has occurred
  2. Go left on the way up (should return early and add some delay into the function)
  3. make a donut at the end (probably have some readings)
  4. the car sometimes stalls -> should try to identify the portion of the track at which it stalls and debug print what is happening there
*/