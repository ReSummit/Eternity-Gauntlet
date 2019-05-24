/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

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

// DOUBLE CHECK
#define SERVOP_MIN  105 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOP_MAX  440 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
uint8_t servonum = 3;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
  // Drive each servo one at a time
  //Serial.println(servonum);

  // Change servo[letter]
  // Change port on pwm

  int servoMin = SERVOP_MIN;
  int servoMax = SERVOP_MAX;

  
  Serial.println("Going from half to max");
  for (uint16_t pulselen = servoMin + (((servoMax-servoMin)*4)/5); pulselen < servoMax; pulselen++) {
    pwm.setPWM(servonum, 0, pulselen);
    Serial.println(pulselen);
    delay(300);
  }
  delay(2000);
  
  
  
  Serial.println("Going from half to min");
  for (uint16_t pulselen = servoMin + (servoMax-servoMin)/5; pulselen > servoMin; pulselen--) {
    pwm.setPWM(servonum, 0, pulselen);
    Serial.println(pulselen);
    delay(300);
  }
  delay(10000);
  
  
  

  //pwm.setPWM(servonum, 0, SERVOP_MIN);

  //servonum ++;
  //if (servonum > 7) servonum = 0;
}





// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}
