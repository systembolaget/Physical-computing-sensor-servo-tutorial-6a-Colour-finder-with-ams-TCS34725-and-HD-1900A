// Tutorial 6a. Colour finder w. ams TCS34725 and HD-1900A

// Main parts: Adafruit Metro Mini, Adafruit TCS34725 sensor breakout-
// board, Pololu HD-1900A servo, ultra-matt black sensor shroud

// Libraries required to use the I2C communication protocol, drive the
// servo and interface with the sensor; use the latest version
#include <Wire.h>
#include "Servo.h"
#include "Adafruit_TCS34725.h"

// Variables that remain constant
const byte interruptPin = 2; // Signal input pin from TCS34725 interrupt

// Two-dimensional array containing averaged RGB values from "colour
// training"; matching servo rotation angles for the 10° colour wheel
// segments. No particular order or sorting of samples is necessary
const int SAMPLES[][4] = {
  {2062, 1469,  861,  665},
  {1735,  758,  502,  755},
  {1148,  440,  415,  850},
  {1226,  478,  520,  950},
  { 907,  580,  620, 1050},
  {1837, 1529, 1437, 1155},
  { 825,  821,  971, 1265},
  { 452,  404,  397, 1370},
  { 888, 1192, 1220, 1475},
  { 477,  614,  655, 1585},
  { 537,  640,  464, 1685},
  {1241, 1326,  785, 1790},
  {1390, 1204,  648, 1890},
  {1402,  973,  567, 1990},
};
// Determines the number of samples stored in the array
const byte samplesCount = sizeof(SAMPLES) / sizeof(SAMPLES[0]);
// Instances a Servo object from the library and sets the sensing duration
// (integration time) and sensitivity (gain); see library options. Longer
// integration time plus lower gain = higher accuracy
Adafruit_TCS34725 SENSOR = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
// Instances a Servo object from the library
Servo SERVOHD1900A;

// Variables that can change
unsigned long timeNow = 0; // Timestamp that updates each loop() iteration
const byte timeInterval = 5; // Time to pass until the next servo rotation
int angleCurrent; // Angle in microseconds = finer rotation resolution
int angleTarget; // Angle in microseconds = finer rotation resolution
uint16_t redSensor, greenSensor, blueSensor, clearSensor; // Raw readings
volatile boolean state = false; // Flag to toggle interrupt on and off

// For the compiler: isr() = function called when an interrupt occurs
void isr()
{
  state = true;
}

void setup()
{
  // TCS34725 interrupt output is active-LOW and open-drain
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, FALLING);

  // Initialises the sensor; the LED is on, but could be switched via pin
  // A0 programmatically, or via connection to GND (LED off)
  SENSOR.begin();
  // Set persistence filter = generates an interrupt for every cycle,
  // regardless of the integration time limits
  SENSOR.write8(TCS34725_PERS, TCS34725_PERS_NONE);
  // Activate the sensor's interrupt function
  SENSOR.setInterrupt(true);

  // Uncomment for "colour training" and checking purposes
  //Serial.begin(19200);

  // Initialises the servo; connect its signal wire to pin 3
  SERVOHD1900A.attach(3);
  // Rotates the servo to initial position (angle 0° in microseconds)
  SERVOHD1900A.writeMicroseconds(600);

  // Seed both with an initial value to start with something. All samples
  // are related to an angle expressed in microseconds, stored in the
  // array. At the start, the current angle must be lower than the target
  // angle = the servo starts from the 0° position
  angleCurrent = 0;
  angleTarget = 600;
}

void loop()
{
  // A call to this function reads from the TCS34725 sensor. Unlike reading
  // from the sensor in the basic way, listening to the sensor's external
  // interrupt, the long integration time of 700ms - necessary to obtain
  // more accurate colour readings - does no longer block loop() from
  // executing other code
  readSensor();
  // A call to this function iterates through the array to retrieve a
  // matching colour sample
  identifySample();
  // A call to this function rotates the servo based on the angle in
  // microseconds, also retrieved from the array
  rotateServo();
}

void readSensor()
{
  if (state)
  {
    // A call to this function reads the RGB values into the matching
    // variables (& points to the memory address of a variable). The clear
    // value is not needed, but must be read as per the library code
    getRawData_noDelay(&redSensor, &greenSensor, &blueSensor, &clearSensor);

    // Uncomment for initial "colour training"
    /*Serial.print("R: "); Serial.print(redSensor, DEC); Serial.print(" ");
      Serial.print("G: "); Serial.print(greenSensor, DEC); Serial.print(" ");
      Serial.print("B: "); Serial.print(blueSensor, DEC); Serial.print(" ");
      Serial.print("C: "); Serial.print(clearSensor, DEC); Serial.print(" ");
      Serial.println(" ");*/

    // Dectivate the sensor's interrupt function after taking a reading
    SENSOR.clearInterrupt();
    state = false;
  }
}

void identifySample()
{
  int colourDistance;

  // Iterate through the array to find a matching colour sample
  for (byte i = 0; i < samplesCount; i++)
  {
    colourDistance = getColourDistance(redSensor, greenSensor, blueSensor, SAMPLES[i][0], SAMPLES[i][1], SAMPLES[i][2]);
    // The more constant the illumination conditions during "colour training"
    // and when using the sensor, the lower the colour distance comparator
    // can be chosen, and even more different colours can be reliably
    // identified; hence the need for an ultra-matt black sensor shroud
    if (colourDistance < 50)
    {
      // Retrieve a matching angle value (in microseconds) from the array
      angleTarget = SAMPLES[i][3] + 150;
      // Uncomment after "colour training", also to check if the colour
      // distance is as low as possible
      /*Serial.print("Distance: "); Serial.println(colourDistance);
        Serial.print("Sample no.: "); Serial.println(++i);*/
    }
  }
}

void rotateServo()
{
  // Check if it is time to rotate the servo another step towards the
  // target angle retrieved from the array. Depending on timeInterval,
  // the servo can be slowed down from its regular speed
  if (millis() - timeNow >= timeInterval)
  {
    // Create a new timestamp for the next step = loop() execution
    timeNow = millis();
    // Don't write to the servo after the target angle was reached
    if (angleCurrent != angleTarget)
    {
      if (angleCurrent <= angleTarget)
      {
        angleCurrent += 5;
        SERVOHD1900A.writeMicroseconds(angleCurrent);
      }
      else
      {
        if (angleCurrent >= angleTarget)
        {
          angleCurrent -= 5;
          SERVOHD1900A.writeMicroseconds(angleCurrent);
        }
      }
    }
  }
}

void getRawData_noDelay(uint16_t *redSensor, uint16_t *greenSensor, uint16_t *blueSensor, uint16_t *clearSensor)
{
  // getRawData() would cause a delay of the duration of the selected
  // integration time, thus blocking loop() like a delay() in the code;
  // but using the ISR (= interrupt) method, there is no more delay,
  // because one receives an interrupt once the integration is done and
  // loop() is no longer waiting for the sensor (* creates a pointer to
  // the memory address of a variable)
  *clearSensor = SENSOR.read16(TCS34725_CDATAL);
  *redSensor = SENSOR.read16(TCS34725_RDATAL);
  *greenSensor = SENSOR.read16(TCS34725_GDATAL);
  *blueSensor = SENSOR.read16(TCS34725_BDATAL);
}

int getColourDistance(int redSensor, int greenSensor, int blueSensor, int redSample, int greenSample, int blueSample)
{
  // Calculates the Euclidean distance between two RGB colours
  // https://en.wikipedia.org/wiki/Color_difference
  return sqrt(pow(redSensor - redSample, 2) + pow(greenSensor - greenSample, 2) + pow(blueSensor - blueSample, 2));
}
