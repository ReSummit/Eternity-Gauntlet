#include <Servo.h>

Servo longServo;  // Create servo object to control a servo
Servo shortServo;  // Create servo object to control a servo

int longPos = 0;    // Store servo position
int shortPos = 0;    // Store servo position

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
const int LONG_FLEX_PIN = PA0; // Pin connected to voltage divider output
const int SHORT_FLEX_PIN = PA1; // Pin connected to voltage divider output

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

  delay(1000);

  // Set up flex sensors
  pinMode(LONG_FLEX_PIN, INPUT);
  pinMode(SHORT_FLEX_PIN, INPUT);  

  // Set up servos
  longServo.attach(PA2);
  longServo.write(0);

  shortServo.attach(PA3);
  shortServo.write(0);

}

void loop()
{
  // Read the ADC, and calculate voltage and resistance from it
  //int flexADC = analogRead(FLEX_PIN);

  uint16_t longFlexADC = analogRead(LONG_FLEX_PIN);
  uint16_t shortFlexADC = analogRead(SHORT_FLEX_PIN);
  
  float longFlexV = longFlexADC * VCC / 4096.0;
  float shortFlexV = shortFlexADC * VCC / 4096.0;

  Serial.println("longflexADC= "+ String(longFlexADC)+" longflexV = "+String(longFlexV));
  Serial.println("shortflexADC= "+ String(shortFlexADC)+" shortflexV = "+String(shortFlexV));
  
  float longFlexR = LONG_R_DIV * (VCC / longFlexV - 1.0);
  float shortFlexR = SHORT_R_DIV * (VCC / shortFlexV - 1.0);
  
  Serial.println("Long Resistance: " + String(longFlexR) + " ohms");
  Serial.println("Short Resistance: " + String(shortFlexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  //SHORT_STRAIGHT_RESISTANCE = 26250.0;
  //SHORT_BEND_RESISTANCE = 100000.0;
  //LONG_STRAIGHT_RESISTANCE = 10500.0;
  //LONG_BEND_RESISTANCE = 27000.0;
  float longAngle = constrain(map(longFlexR, LONG_STRAIGHT_RESISTANCE, LONG_BEND_RESISTANCE, 0, 90.0), 0, 180);
  float shortAngle = constrain(map(shortFlexR, SHORT_STRAIGHT_RESISTANCE, SHORT_BEND_RESISTANCE, 0, 90.0), 0, 180);
  
  Serial.println("LongAngle: " + String(longAngle) + " degrees");
  Serial.println("ShortAngle: " + String(shortAngle) + " degrees");

  /*
  // Write same angle to servo
  float longVal = abs(longAngle);
  float shortVal = abs(shortAngle);
  
  Serial.println("longVal: " + String(longVal));
  Serial.println("shortVal: "+ String(shortVal));

  longServo.write(longVal);
  shortServo.write(shortVal);
  */

  /*
  float val = longPos;
  Serial.println(val);
  longServo.write(val);
  longPos += 1;
  */
  /*
  float val = shortPos;
  Serial.println(val);
  shortServo.write(val);
  shortPos += 1;
  */
  
  
  Serial.println();
  
  delay(1000);
}
