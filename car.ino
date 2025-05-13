#include "ECE3.h"

const int LED_RF = 41;

#define NUM_SENSORS 8
#define KP 0.1
#define DEVIATION_FROM_CENTER_MARGIN 200

#define LEFT_NSLP_PIN 31 // nslp ==> awake & ready for PWM
#define LEFT_DIR_PIN 29
#define LEFT_PWN_PIN 40
#define RIGHT_NSLP_PIN 11
#define RIGHT_DIR_PIN 30
#define RIGHT_PWM_PIN 39

uint16_t sensorValues[8];
uint16_t minTerms[] = {805, 728, 711, 688, 640, 758, 734, 805};
uint16_t maxDiff[] = {1695, 1772, 1789, 1254, 1515, 1742, 1766, 1695};
float normalizedValues[NUM_SENSORS];
float resultValues[NUM_SENSORS];

struct WheelSpeed {
  int rightSpeed;
  int leftSpeed;
} wheel_speed = {
  20, 20
};

///////////////////////////////////
void setup() {
// put your setup code here, to run once:
  pinMode(LEFT_NSLP_PIN,OUTPUT);
  pinMode(LEFT_DIR_PIN,OUTPUT);
  pinMode(LEFT_PWN_PIN,OUTPUT);

  digitalWrite(LEFT_DIR_PIN,LOW);
  digitalWrite(LEFT_NSLP_PIN,HIGH);

  pinMode(RIGHT_NSLP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  
  digitalWrite(RIGHT_DIR_PIN, LOW);
  digitalWrite(RIGHT_NSLP_PIN, HIGH);

  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();

  // set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  Serial.println("BEGIN");
  
  updateLeftWheelSpeed(0);
  updateRightWheelSpeed(0);
}

void loop() {
  // while (!Serial.available()) {}

  Serial.println("HI");

  ECE3_read_IR(sensorValues);

  getNormalizedValues(normalizedValues);
  float errorTerm = calculate1514128Error(normalizedValues);

  Serial.println(errorTerm);

  updateRightWheelSpeed(errorTerm);
  updateLeftWheelSpeed(errorTerm);

  delay(5000);
}

void getNormalizedValues(float results[]) {
  for (int i =0; i < NUM_SENSORS; ++i) {
    results[i] = (sensorValues[i] - minTerms[i]) * 1000 / maxDiff[i];
  }
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
  wheel_speed.leftSpeed -= KP * errorTerm;

  Serial.print("The left wheel speed is ");
  Serial.println(wheel_speed.leftSpeed);

  if (wheel_speed.leftSpeed < 0) {
    digitalWrite(LEFT_DIR_PIN, HIGH);
  } else {
    digitalWrite(LEFT_DIR_PIN, LOW);
  }

  // analogWrite(LEFT_PWN_PIN, abs(wheel_speed.leftSpeed));
  analogWrite(LEFT_PWN_PIN, 20);
}

void updateRightWheelSpeed(float errorTerm) {
  wheel_speed.rightSpeed += KP * errorTerm;

  Serial.print("The right wheel speed is ");
  Serial.println(wheel_speed.rightSpeed);

  if (wheel_speed.rightSpeed < 0) {
    digitalWrite(RIGHT_DIR_PIN, HIGH);
  } else {
    digitalWrite(RIGHT_DIR_PIN, LOW);
  }

  // analogWrite(RIGHT_PWM_PIN, (abs(wheel_speed.rightSpeed)));
  analogWrite(RIGHT_PWM_PIN, 20);
}
