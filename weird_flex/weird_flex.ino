#include <Servo.h>

Servo myservo;  // Create servo object to control a servo
int pos = 0;    // Store servo position

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
const int FLEX_PIN = PA0; // Pin connected to voltage divider output

// Measure the voltage at 5V and the actual resistance of your
// 47k resistor, and enter them below:
const float VCC = 3.3; // Measured voltage of Ardunio 5V line
const float R_DIV = 50000.0; // Measured resistance of 3.3k resistor

// Upload the code, then try to adjust these values to more
// accurately calculate bend degree.
//const float STRAIGHT_RESISTANCE = 37300.0; // resistance when straight
//const float BEND_RESISTANCE = 90000.0; // resistance at 90 deg

/**
 * Short:
 *  Straight: 30.3k
 *  Bend: 100k
 *  
 * Long:
 *  Straight: 12.3k
 *  Bend: 36k
 */
const float SHORT_STRAIGHT_RESISTANCE = 30300.0;
const float SHORT_BEND_RESISTANCE = 100000.0;
const float LONG_STRAIGHT_RESISTANCE = 12300.0;
const float LONG_BEND_RESISTANCE = 36000.0;



void setup()
{
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT);

  myservo.attach(PA2);
}

void loop()
{
  // Read the ADC, and calculate voltage and resistance from it
  int flexADC = analogRead(FLEX_PIN);
  float flexV = (float)flexADC * VCC / 4096.0;
  Serial.println("flexADC= "+ String(flexADC)+" flexV = "+String(flexV));
  float flexR = R_DIV * (VCC / flexV - 1.0);
  Serial.println("Resistance: " + String(flexR) + " ohms");

  // Use the calculated resistance to estimate the sensor's
  // bend angle:
  float angle = map(flexR, LONG_STRAIGHT_RESISTANCE, LONG_BEND_RESISTANCE,
                   0, 90.0);
  Serial.println("Bend: " + String(angle) + " degrees");

  // Write same angle to servo
  float val = abs(angle)-35;  // Subtract 35 because lowest degree is 35 instead of 0 for some reason. Work in progress to fix.
  Serial.println(val);
  myservo.write(val);
  
  Serial.println();
  
  delay(50);
}
