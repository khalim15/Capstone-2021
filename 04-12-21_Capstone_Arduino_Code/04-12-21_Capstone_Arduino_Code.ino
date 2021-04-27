/*  Updated April 12, 2021.
 *  Written by: Maaz Khalid

This Code includes libraries for Firebase, wifiNINA, and IMU sensing data.
It takes data from an IMU, encoder/potentiometer, and time.
For the purposes of validation, a simulated set of load cell data is generated from the encoder/potentiometer.
This fake data is sent to an online database and then retreived to simulate the data retreived from the load cell amp.

Board: Arduino Uno Wifi Rev 2 

*** IMPORTANT ***
Ensure MegaAVR boards is downloaded to use this code with the same arduino. 
Also requires a firmware update to the latest version for any kind of bluetooth or Wifi usage.

IMU: Adafruit BNO055
Bluetooth module: HC-05
Fake Load cell realtime database: Firebase (personal database used)

Below: Libraries and data for firebase database (pseudo data to simulate load cell wifi data.
 
#include "Firebase_Arduino_WiFiNINA.h"
#define FIREBASE_HOST "fakeloadcelldata-default-rtdb.firebaseio.com" this was my firebase used for testing.
#define FIREBASE_AUTH "WlmyQ8NgbaItKoOsSv2ibj8xkvvWZu3zncoOXHIO" the secret password for the database.
#define WIFI_SSID "WIFI SSID"
#define WIFI_PASSWORD "WIFI Password"
FirebaseData firebaseData;
String path = "/Force&Moment"; This was the path name for my database.
*/

//Declared variables and libraries
#include <SoftwareSerial.h> 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Definition that identifies the TX and RX pins that the the HC-05 is connected to (in this case digital pins 12 and 13). Also defined the IMU sensor.
SoftwareSerial BTSerial(12, 13); // RX | TX 
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//Variables generated for sending data such as forces, moments, etc.
//Used A0 pin for potentiometer.
int sensorPin = A0;
float fX, fY, fZ, mX, mY, mZ;
float eX, eY, eZ;
float forceX, forceY, forceZ, momentX, momentY, momentZ;
float halfPot = 512;
float maxPot = 1023;

//Variables for calculating time, used for generating pseudo data
unsigned long previousMillis;
int angleValue;
float sensorValue;
int millisec;
int tseconds;
int tminutes;
int seconds;
String DisplayTime;
unsigned long times;

void setup() {
  //initializes serial port for allowing user to check that code works with serial monitor. Make sure baud rate is always 9600.
  Serial.begin(9600); 
  BTSerial.begin(9600); 
  delay(1000);
/*
  //Wifi setup for firebase
  Serial.print("Connecting to WiFi…");
  int status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("...");
    delay(500);
  }
  Serial.print(" Connected, IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH, WIFI_SSID, WIFI_PASSWORD);
  Firebase.reconnectWiFi(true);
  */

  //BT and IMU initialization. Uses a simple password of 0000 or 1234. Callibration is done with the BNO algorithm, gives rating betweeen 0 and 3, where 3 means callibrated and 0 means no callibration.
  Serial.println("Ready to connect\nDefualt password is 1234 or 000"); 
  Serial.println("Orientation Sensor Raw Data"); 
  Serial.println("");
  if(!bno.begin()){
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("Calibration status: 0=uncalibrated, 3=fully calibrated");
  delay(1000);
}
  
void loop() {

  /*tick rate of data: method 1
  if ((previousMillis - millis()) >= 100) {
    previousMillis = millis(); 
    
    Method 1 incorporates this into every loop to try to synchcronize the values being generated.
    Method 2 was used instead of this loop. We used a global timer at the end of the code to tell the arduino to wait 100 ms before a new message was sent to the app.
    */
  
    // Potentiometer data (substitude for the encoder). Reads the A0 (anaolog 1) pin.
       int angleValue = analogRead(sensorPin);
  
    // IMU data 
       imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
       eX = euler.x();
       eY = euler.y();
       eZ = euler.z();
    
    // Time calculation. Millis function keeps track of time arduino is on in ms. Uses modulus to find the mins and seconds. We use time to send data timestamps to the app.
       times = millis();
       millisec  = times % 1000;
       tseconds = times / 1000;
       tminutes = tseconds / 60;
       seconds = tseconds % 60;
       DisplayTime = String(tminutes) + ":" + String(seconds,DEC) + ":" + String(millisec,DEC);
       //Serial.println(DisplayTime); print statement for time. 
    
    // Fake Load cell data (from firebase). We also used this section for generating pseudo data.
    // Reads potentiometer value and converts it to fake force and moment data. For testing, we only used fX to generate our pseudo data. Fy, Fz, Mx, My, Mz are generated based on Fx.

    // This first loop is for when the angle is between 0 and the halfpoint. A force porportional to the angle in created.
        float sensorValue = analogRead(sensorPin);
        if (sensorValue <= 512) {
          fX = (sensorValue/halfPot)*350;
          fY = (sensorValue/halfPot)*20;
          fZ = (sensorValue/halfPot);
          mX = fX*0.3;
          mY = fY*0.3;
          mZ = fZ*0.3;
          /* Section for calculating simulated load cell data less than half angle
          fY = (sensorValue/halfPot)*20 ;
          fZ = (sensorValue/halfPot) ;
          mX = fX*0.3; 
          mY = fY*0.3; 
          mZ = fZ*0.3; 
          */ 
        }
        // This second loop is for when the angle is between the halfpoint and the maximum. A force neagtively porportional to the angle in created. 
        // Together the 2 loops create a piece wise function where maximum force is at the halfpoint, then decreases to 0 at the min and max potentiometer angle.
        else if (sensorValue > 512) {
          fX = ((maxPot-sensorValue)/halfPot)*350;
          fY = ((maxPot-sensorValue)/halfPot)*20;
          fZ = ((maxPot-sensorValue)/halfPot);
          mX = fX*0.3;
          mY = fY*0.3;
          mZ = fZ*0.3;
        }
          /* Section for calculating simulated load cell data greater than half angle
          fY = ((maxPot-sensorValue)/halfPot)*20 ;
          fZ = ((maxPot-sensorValue)/halfPot) ;
          mX = fX*0.3;
          mY = fY*0.3;
          mZ = fZ*0.3;
          */
       
       /* Commented out section for demo purposes. This section will send the fX value to the firebase database and then retrieve it from the path to simulate
          the recieved signal from the load cell amp. However, the transfer can lag behind and slow the arduino message to the app.
          Therefore we settled on commenting this out and directly generating pseudo data.
          A more realistic approach would be to connect with the load cell amp WIFI and then retrieve the force/moment data every 100 ms.
         
          // Send data to Firebase with specific path
          if (Firebase.setFloat(firebaseData, path + "/fX", fX)) {
              Serial.println(firebaseData.dataPath() + "sent data = " + fX);
          }
          if (Firebase.setFloat(firebaseData, path + "/fY", fY)) {
             Serial.println(firebaseData.dataPath() + " = " + fY);
          }
          if (Firebase.setFloat(firebaseData, path + "/fZ", fZ)) {
             Serial.println(firebaseData.dataPath() + " = " + fZ);
          }
          if (Firebase.setFloat(firebaseData, path + "/mX", mX)) {
             Serial.println(firebaseData.dataPath() + " = " + mX);
          }
          if (Firebase.setFloat(firebaseData, path + "/mY", mY)) {
             Serial.println(firebaseData.dataPath() + " = " + mY);
          }
          if (Firebase.setFloat(firebaseData, path + "/mZ", mZ)) {
             Serial.println(firebaseData.dataPath() + " = " + mZ);
          }
          else {
             Serial.println("Error: " + firebaseData.errorReason());
          }
          
         //read each database path to extract the generated float
          if (Firebase.getFloat(firebaseData, "/Force&Moment/fX")) {
            if (firebaseData.dataType() == "float") {
              forceX = firebaseData.floatData();
            }
          }
          if (Firebase.getFloat(firebaseData, "/Force&Moment/fY")) {
            if (firebaseData.dataType() == "float") {
              forceY = firebaseData.floatData();
              Serial.print("forceY ");
              Serial.println(forceY);
            }
          }
          if (Firebase.getFloat(firebaseData, "/Force&Moment/fZ")) {
            if (firebaseData.dataType() == "float") {
              forceZ = firebaseData.floatData();
              Serial.print("forceZ ");
              Serial.println(forceZ);
            }
          }
          if (Firebase.getFloat(firebaseData, "/Force&Moment/mX")) {
            if (firebaseData.dataType() == "float") {
              momentX = firebaseData.floatData();
              Serial.print("momentX ");
              Serial.println(momentX);
            }
          }
          if (Firebase.getFloat(firebaseData, "/Force&Moment/mY")) {
            if (firebaseData.dataType() == "float") {
              momentY = firebaseData.floatData();
              Serial.print("momentY ");
              Serial.println(momentY);
            }
          }
          if (Firebase.getFloat(firebaseData, "/Force&Moment/mZ")) {
            if (firebaseData.dataType() == "float") {
              momentZ = firebaseData.floatData();
              Serial.print("momentZ ");
              Serial.println(momentZ);
            }
          }
          else {
            Serial.println("Error: " + firebaseData.errorReason());
          }
          */

    //Print statement to the to serial monitor for testing and checking, to ensure the values are correct/realistic.
       Serial.print("Angle value: ");
       Serial.println(angleValue);
       Serial.print("X: ");
       Serial.print(eX,2);
       Serial.print(" Y: ");
       Serial.print(eY,2);
       Serial.print(" Z: ");
       Serial.print(eZ,2);
       Serial.print("\t");
       // Callibration print statement of the IMU sensor.
       uint8_t system, gyro, accel, mag = 0;
       bno.getCalibration(&system, &gyro, &accel, &mag);
       Serial.print("CALIBRATION: Sys=");
       Serial.print(system, DEC);
       Serial.print(" Gyro=");
       Serial.print(gyro, DEC);
       Serial.print(" Accel=");
       Serial.print(accel, DEC);
       Serial.print(" Mag=");
       Serial.println(mag, DEC);
       Serial.print("forceX recieved: ");
       Serial.println(fX);
    
    //BT message to app. This is instantly sent as string converted to a byte buffer stream through bluetooth, finally handled by the app. 
    // Its all one line, separated by commas as delimeters so that the app can sort the message into a list.
       BTSerial.print(sensorValue);
       BTSerial.print(",");
       BTSerial.print(eX);
       BTSerial.print(",");
       BTSerial.print(eY);
       BTSerial.print(",");
       BTSerial.print(eZ);
       BTSerial.print(",");
       BTSerial.print(fX);
       BTSerial.print(",");
       BTSerial.print(fY);
       BTSerial.print(",");
       BTSerial.print(fZ);
       BTSerial.print(",");
       BTSerial.print(momentX);
       BTSerial.print(",");
       BTSerial.print(momentY);
       BTSerial.print(",");
       BTSerial.print(momentZ);
       BTSerial.print(",");
       BTSerial.println(DisplayTime);
       //tick rate of data method 2. We used 200 ms since it is still decently fast. We do not reccomend going below 100 ms since it can create data synchronization issues.
       delay(200);
}
