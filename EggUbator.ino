/* EggUbator
 *  All inclusive PID temp controlled
 *  hands free egg rolling
 *  incubator
 */

#include <OneWire.h>
#include <DallasTemperature.h>

// One-Wire Temperature Sensor
// Data wire is plugged into port 2 on the Arduino
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 2
#define ONE_WIRE_PWR 3
#define ONE_WIRE_GND 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

#include <PID_v1.h>
#define RelayPin 13

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT); // was 2 5 1

int WindowSize = 1000;
unsigned long windowStartTime;

// constants won't change. They're used here to set pin numbers and other values:
const int ForceStartSwitch = 11;       // pin number for the ForceStartSwitch
const int BumpSwitch = 10;             // pin number for the bumpswitch
const int Motor =  9;                  // pin number for the Motor (PWM pin)
const int MotorDriveValue = 140;       // PWM value to drive motor
const long MotorMoveTime = 59000;      // 59 seconds Motor move time in milliseconds
const long MotorRestTime = 21600000;   // 21600000 = 6 hours Motor rest time in milliseconds (1000 x 60 x 60 x 6)

// variables will change:
int motorState = 0;          // variable for tracking motor state
int switchState = 0;         // variable for tracking switch state

// variables for keeping track of time
unsigned long previousMillis = 0;

void setup() {
  // Set up Ground & Power for the sensor from GPIO pins
   pinMode(ONE_WIRE_GND, OUTPUT);
   digitalWrite(ONE_WIRE_GND, LOW);
   pinMode(ONE_WIRE_PWR, OUTPUT);
   digitalWrite(ONE_WIRE_PWR, HIGH);

  // start serial port
  Serial.begin(9600);
  Serial.println("Incubator Data Out");

  // Start up the Dallas Temperature IC Control library
  sensors.begin();

  windowStartTime = millis();
  
  //initialize the variables we're linked to
  Setpoint = 102; //for some reason the thermometer reads about 2 deg F high

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetSampleTime(1000);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // initialize RelayPin as an output.
  pinMode(RelayPin, OUTPUT);
  
  // initialize the button pins as an input:
  pinMode(ForceStartSwitch, INPUT);
  pinMode(BumpSwitch, INPUT);
  
  // initialize Motor Pin as an output.
  pinMode(Motor, OUTPUT);

  //set switch state
  if (digitalRead(BumpSwitch)) {
    switchState = 1; //in home possition
  } 

}

void loop() {

  float t2move; //time to move variable for showing % of time left till next move 

  //check for force start switch and start motor if pressed
  if (digitalRead(ForceStartSwitch) && motorState == 0) {
    previousMillis = millis();
    motorState = 1; //Turn motor on
    analogWrite(Motor, MotorDriveValue);
  }
  
  //check motor state / progress
  if (motorState == 0) { //motor currently off
    //check time against wait time
    //Serial.println(MotorRestTime);
    if (millis() - previousMillis >= MotorRestTime) {
      previousMillis = millis();
      motorState = 1; //Turn motor on
      analogWrite(Motor, MotorDriveValue);
      }
  } else { //motor currently on
    if (switchState == 1) {  
     //Motor moves untill until MotorMoveTime is reached (Away from Home)
      if (millis() - previousMillis >= MotorMoveTime) {
        // switch motor state & direction 
        previousMillis = millis();
        motorState = 0;
        switchState = 0;
        analogWrite(Motor, 0);
      }
    } else {
   //Motor moves untill switch is on or until MotorMoveTime is reached (toward Home)
    if ((digitalRead(BumpSwitch)) || (millis() - previousMillis >= MotorMoveTime)) {
      // switch motor state & direction 
      previousMillis = millis();
      motorState = 0;
      switchState = 1;
      analogWrite(Motor, 0);
      }
    }
  }

  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  Input = sensors.getTempFByIndex(0);  
  myPID.Compute();
  
  // a little bit of serial output if desired
  Serial.print("Setpoint:"); Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print("Current_Temp:"); Serial.print(Input);
  Serial.print("\t");
  Serial.print("PID_Output%:"); Serial.print(Output/WindowSize*100);
  Serial.print("\t");
  
  //calculate the % of time till the next move (and make sure it stays withing bounds
  t2move = float(MotorRestTime-(millis() - previousMillis))/MotorRestTime*100;
  if(t2move>100.0){
    t2move=100.0;
    }
  Serial.print("Time_to_Move%:"); Serial.println(t2move);

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if(millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > (millis() - windowStartTime)) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);
  
}
