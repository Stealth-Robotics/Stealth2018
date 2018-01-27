/*
 * This sketch manages the sensors on the Stealth FIRST FRC 4089 robot.
 * 
 * The sketch is meant to run on an Adruino Leonardo and includes get functions. 
 * 
 * This sketch uses: https://www.sparkfun.com/products/8958
 *  2.8V at 15cm
 *  0.4V at 150cm
 */

#include <SoftwareSerial.h>

const unsigned int BAUD_RATE = 9600; //serial
const unsigned int BT_BAUD_RATE = 115200; //bluetooth

const int numSensors = 5; 

int readIndex = 0;            // the index of the current reading
double average = 0;
int delayValue = 5;
//pin setup
const int rxpin = 0;
const int txpin = 1;
int sensorOnePin = A0;
int sensorTwoPin = A1;
int sensorThreePin = A2;
int sensorFourPin = A3;
double linearVal = 0;
double sensorVal = 0;

// sensor setup
const int sensor8958MaxV = 2.8;
const int sensor8958MinV = 0.4;
const int sensor8958MinD = 15; //cm
const int sensor8958MaxD = 150; //cm

int distance = 0;               //distance from 9858

void setup() {
  
  Serial.begin(BAUD_RATE);
  Serial.println("Starting Serial Connection");

}

void loop() {

 
  sensorVal = analogRead(sensorOnePin);
  Serial.println(sensorVal);
  /*
   *
    Serial.println(analogRead(sensorOnePin));
    Serial.println(analogRead(sensorTwoPin));
    Serial.println(analogRead(sensorThreePin));
    Serial.println(analogRead(sensorFourPin));
    */
    delay(delayValue);
}

int calcDistance(int inputVal){
  linearVal = ((sensor8958MinD*(sensor8958MinV-inputVal)+sensor8958MaxD*(inputVal-sensor8958MaxV))/(sensor8958MinV-sensor8958MaxV));
  linearVal = abs(linearVal);
  return linearVal;
}

