#include "ECE3.h"

const int LED_RF = 41;

#define NUM_SENSORS 8
#define KP 0.003
#define KD 0.06
#define ERROR_TERM_TOLERANCE 200
#define OPEN_LOOP_TIME 2000

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
unsigned long start_time;

uint8_t rightReverseCounter = 0;
uint8_t leftReverseCounter = 0;

struct WheelSpeed {
  int rightSpeed;
  int leftSpeed;
} wheel_speed = {
  40, 40
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
//   digitalWrite(LEFT_NSLP_PIN,HIGH);
  // #endif

  pinMode(RIGHT_NSLP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  
  digitalWrite(RIGHT_DIR_PIN, LOW);

  // #ifndef IS_DEBUG
//   digitalWrite(RIGHT_NSLP_PIN, HIGH);
  // #endif
  
  ECE3_Init();

  // set the data rate in bits/second for serial data transmission
  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
  // Serial.println("BEGIN");

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
    Serial.print(floatSensorValues[i]);
    Serial.print(" | ");
  }

  Serial.println();
}

