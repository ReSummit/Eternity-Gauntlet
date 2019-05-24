#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Servo thumbServo;  // Create servo object to control a servo
Servo indexServo;  // Create servo object to control a servo
Servo middleServo;  // Create servo object to control a servo
Servo ringServo;  // Create servo object to control a servo
Servo pinkyServo;  // Create servo object to control a servo

/* Debugging only
int longPos = 0;    // Store servo position
int shortPos = 0;    // Store servo position
*/

/******************************************************************************
Flex_Sensor_Example.ino
Example sketch for SparkFun's flex sensors
  (https://www.sparkfun.com/products/10264)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining a flex sensor with a 47k resistor.
- The resistor should connect from A0 to GND.
- Thx_1_z_yese flex sensor should connect from A0 to 3.3V
As the resistance of the flex sensor increases (meaning it's being bent), the
voltage at A0 should decrease.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/
const int T_FLEX_PIN = PA0; // Pin connected to voltage divider output
const int I_FLEX_PIN = PA1; // Pin connected to voltage divider output
const int M_FLEX_PIN = PA2; // Pin connected to voltage divider output
const int R_FLEX_PIN = PA3; // Pin connected to voltage divider output
const int P_FLEX_PIN = PA4; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
//const float VCC = 3.3; // Measured voltage of Ardunio 5V line
const float VCC = 5; // Measured voltage of Ardunio 5V line
const float LONG_R_DIV = 25000.0; // Measured resistance of 3.3k resistor
const float SHORT_R_DIV = 60500.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
//const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
//const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
// GOOD
#define SERVOT_MIN  90 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOT_MAX  500 // this is the 'maximum' pulse length count (out of 4096)

// GOOD
#define SERVOI_MIN  95 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOI_MAX  530 // this is the 'maximum' pulse length count (out of 4096)

// GOOD
#define SERVOM_MIN  102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOM_MAX  507 // this is the 'maximum' pulse length count (out of 4096)

// GOOD
#define SERVOR_MIN  85 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOR_MAX  486 // this is the 'maximum' pulse length count (out of 4096)

// GOOD
#define SERVOP_MIN  105 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOP_MAX  440 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 5;

// Counter used for averaging
#define SAMPLES 3 // Number of samples used for avergaing
uint16_t samples1[SAMPLES];
uint16_t samples2[SAMPLES];
uint16_t samples3[SAMPLES];
uint16_t samples4[SAMPLES];
uint16_t samples5[SAMPLES];

/**
 * Short:
 *  Straight: 26.25k
 *  Bend: 100k
 *  R2 = 65k
 *  
 * Long:
 *  Straight: 12.3k
 *  Bend: 36k
 *  R2 = 24k
 */
const float SHORT_STRAIGHT_RESISTANCE = 26250.0;
const float SHORT_BEND_RESISTANCE = 100000.0;
const float LONG_STRAIGHT_RESISTANCE = 10500.0;
const float LONG_BEND_RESISTANCE = 27000.0;

/**
 * Wiring
 * 
 * Long flex:
 * - LONG_FLEX_PIN = PA0;
 * - longServo (1 fin) = PA2
 * 
 * Short flex:
 * - SHORT_FLEX_PIN = PA1;
 * - shortServo (2 fin) = PA3
 */

void setup()
{
  Serial.begin(9600);

  // Initialize samples array
  for(int i = 0; i < SAMPLES; i++){
    samples1[i] = 0;
    samples2[i] = 0;
    samples3[i] = 0;
    samples4[i] = 0;
    samples5[i] = 0;
  }

  delay(1000);

  // Set up flex sensors
  pinMode(T_FLEX_PIN, INPUT);
  pinMode(I_FLEX_PIN, INPUT);  
  pinMode(M_FLEX_PIN, INPUT);
  pinMode(R_FLEX_PIN, INPUT);
  pinMode(P_FLEX_PIN, INPUT);
  
  // PWM Code:
  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  delay(10);

}

void loop()
{
  // Read the ADC, and calculate voltage and resistance from it
  //int flexADC = analogRead(FLEX_PIN);

  /* Old code
  uint16_t longFlexADC = analogRead(LONG_FLEX_PIN);
  uint16_t shortFlexADC = analogRead(SHORT_FLEX_PIN);
  */

  uint16_t thumbFlexADC = analogRead(T_FLEX_PIN);
  uint16_t indexFlexADC = analogRead(I_FLEX_PIN);
  uint16_t middleFlexADC = analogRead(M_FLEX_PIN);
  uint16_t ringFlexADC = analogRead(R_FLEX_PIN);
  uint16_t pinkyFlexADC = analogRead(P_FLEX_PIN);

  /* Old code
  float longFlexV = longFlexADC * VCC / 4096.0;
  float shortFlexV = shortFlexADC * VCC / 4096.0;
  */
  
  float thumbFlexV = thumbFlexADC * VCC / 4096.0;
  float indexFlexV = indexFlexADC * VCC / 4096.0;
  float middleFlexV = middleFlexADC * VCC / 4096.0;
  float ringFlexV = ringFlexADC * VCC / 4096.0;
  float pinkyFlexV = pinkyFlexADC * VCC / 4096.0;

  //Serial.println("longflexADC= "+ String(longFlexADC)+" longflexV = "+String(longFlexV));
  //Serial.println("shortflexADC= "+ String(shortFlexADC)+" shortflexV = "+String(shortFlexV));

  /* Old Code
  float longFlexR = LONG_R_DIV * (VCC / longFlexV - 1.0);
  float shortFlexR = SHORT_R_DIV * (VCC / shortFlexV - 1.0);
  */
  
  float thumbFlexR = SHORT_R_DIV * (VCC / thumbFlexV - 1.0);
  float indexFlexR = LONG_R_DIV * (VCC / indexFlexV - 1.0);
  float middleFlexR = LONG_R_DIV * (VCC / middleFlexV - 1.0);
  float ringFlexR = LONG_R_DIV * (VCC / ringFlexV - 1.0);
  float pinkyFlexR = SHORT_R_DIV * (VCC / pinkyFlexV - 1.0);
  
  //Serial.println("Long Resistance: " + String(longFlexR) + " ohms");
  //Serial.println("Short Resistance: " + String(shortFlexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  //SHORT_STRAIGHT_RESISTANCE = 26250.0;
  //SHORT_BEND_RESISTANCE = 100000.0;
  //LONG_STRAIGHT_RESISTANCE = 10500.0;
  //LONG_BEND_RESISTANCE = 27000.0;
  /* Old code
  float longAngle = constrain(map(longFlexR, LONG_STRAIGHT_RESISTANCE, LONG_BEND_RESISTANCE, 0, 180.0), 0, 180);
  float shortAngle = constrain(map(shortFlexR, SHORT_STRAIGHT_RESISTANCE, SHORT_BEND_RESISTANCE, 0, 180.0), 0, 180);
  */
  
  float thumbAngle = constrain(map(thumbFlexR, SHORT_STRAIGHT_RESISTANCE, SHORT_BEND_RESISTANCE, 0, 180.0), 0, 180);
  float indexAngle = constrain(map(indexFlexR, LONG_STRAIGHT_RESISTANCE, LONG_BEND_RESISTANCE, 0, 180.0), 0, 180);
  float middleAngle = constrain(map(middleFlexR, LONG_STRAIGHT_RESISTANCE, LONG_BEND_RESISTANCE, 0, 180.0), 0, 180);
  float ringAngle = constrain(map(ringFlexR, LONG_STRAIGHT_RESISTANCE, LONG_BEND_RESISTANCE, 0, 180.0), 0, 180);
  float pinkyAngle = constrain(map(pinkyFlexR, SHORT_STRAIGHT_RESISTANCE, SHORT_BEND_RESISTANCE, 0, 180.0), 0, 180);
  
  //Serial.println("LongAngle: " + String(longAngle) + " degrees");
  //Serial.println("ShortAngle: " + String(shortAngle) + " degrees");

  //Middle and Ring finger are LONG sensors
  //Everything else is a SHORT sensor
  /* olde code
  float longPulseLen = constrain(map(longAngle, 0, 180.0, SERVOMIN, SERVOMAX), SERVOMIN, SERVOMAX);
  float shortPulseLen = constrain(map(shortAngle, 0, 180.0, SERVOMIN, SERVOMAX), SERVOMIN, SERVOMAX);
  */

  float thumbPulseLen = constrain(map(thumbAngle, 0, 180.0, SERVOT_MIN, SERVOT_MAX), SERVOT_MIN, SERVOT_MAX);
  float indexPulseLen = constrain(map(indexAngle, 0, 180.0, SERVOI_MIN, SERVOI_MAX), SERVOI_MIN, SERVOI_MAX);
  float middlePulseLen = constrain(map(middleAngle, 0, 180.0, SERVOM_MIN, SERVOM_MAX), SERVOM_MIN, SERVOM_MAX);
  float ringPulseLen = constrain(map(ringAngle, 0, 180.0, SERVOR_MIN, SERVOR_MAX), SERVOR_MIN, SERVOR_MAX);
  float pinkyPulseLen = constrain(map(pinkyAngle, 0, 180.0, SERVOP_MIN, SERVOP_MAX), SERVOP_MIN, SERVOP_MAX);

  /* old code
  Serial.println("LongPulseLen: " + String(longPulseLen));
  Serial.println("ShortPulseLen: " + String(shortPulseLen));
  */
  
  Serial.println("ThumbPulseLen: " + String(thumbPulseLen));
  Serial.println("IndexPulseLen: " + String(indexPulseLen));
  Serial.println("MiddlePulseLen: " + String(middlePulseLen));
  Serial.println("RingPulseLen: " + String(ringPulseLen));
  Serial.println("PinkyPulseLen: " + String(pinkyPulseLen));


  // Shift array left
  for(int i = 0; i < SAMPLES - 1; i++){
    samples1[i] = samples1[i+1];
    samples2[i] = samples2[i+1];
    samples3[i] = samples3[i+1];
    samples4[i] = samples4[i+1];
    samples5[i] = samples5[i+1];
  }

  // Set last element in array
  samples1[SAMPLES-1] = thumbPulseLen;
  samples2[SAMPLES-1] = indexPulseLen;
  samples3[SAMPLES-1] = middlePulseLen;
  samples4[SAMPLES-1] = ringPulseLen;
  samples5[SAMPLES-1] = pinkyPulseLen;

  
  // Find average of samples
  uint16_t avg1 = 0;
  uint16_t avg2 = 0;
  uint16_t avg3 = 0;
  uint16_t avg4 = 0;
  uint16_t avg5 = 0;
  for(int i = 0; i < SAMPLES; i++){
    avg1 += (samples1[i]/SAMPLES);
    avg2 += (samples2[i]/SAMPLES);
    avg3 += (samples3[i]/SAMPLES);
    avg4 += (samples4[i]/SAMPLES);
    avg5 += (samples5[i]/SAMPLES);
  }

  /*
  avg1 = (avg1 / 10) * 10;
  avg2 = (avg2 / 10) * 10;
  avg3 = (avg3 / 10) * 10;
  avg4 = (avg4 / 10) * 10;
  avg5 = (avg5 / 10) * 10;
  *?

  Serial.println("avg1: " + String(avg1));
  Serial.println("avg1: " + String(avg1));

  /*
  // Write same angle to servo
  float longVal = abs(longAngle);
  float shortVal = abs(shortAngle);
  
  Serial.println("longVal: " + String(longVal));
  Serial.println("shortVal: "+ String(shortVal));

  longServo.write(longVal);
  shortServo.write(shortVal);
  */

  
  //longServo.write(longAngle);
  //shortServo.write(shortAngle);

  //pwm.setPWM(0, 0, shortPulseLen);
  //pwm.setPWM(1, 0, longPulseLen);

  pwm.setPWM(0, 0, avg1);
  pwm.setPWM(1, 0, avg2);
  pwm.setPWM(2, 0, avg3);
  pwm.setPWM(3, 0, avg4);
  pwm.setPWM(4, 0, avg5);
  

  /*
  float val = longPos;
  Serial.println(val);
  longServo.write(val);
  longPos += 5;
  */
  /*
  float val = shortPos;
  Serial.println(val);
  shortServo.write(val);
  shortPos += 2;
  */
  
  Serial.println();
  
  //delay( 50 );
}
