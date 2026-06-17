//PID
#include <PID_v1.h>

// PID1 - wheel velocity 0
double Pk1 = 3.5;
double Ik1 = 0.5;
double Dk1 = 0.05;
double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

// PID2 - wheel velocity 1
double Pk2 = 3.5;
double Ik2 = 0.5;
double Dk2 = 0.05;
double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

// PID3 - wheel velocity 2
double Pk3 = 3.5;
double Ik3 = 0.5;
double Dk3 = 0.05;
double Setpoint3, Input3, Output3, Output3a;    // PID variables
PID PID3(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);

// PID4 - balancing X - sideways  *** pitch of IMU ***
double Pk4 = 19;
double Ik4 = 150;
double Dk4 = 0.3;
double Setpoint4, Input4, Output4;    // PID variables
PID PID4(&Input4, &Output4, &Setpoint4, Pk4, Ik4 , Dk4, DIRECT);

// PID5 - balancing Y - front/back  *** roll of IMU ***
double Pk5 = 19;
double Ik5 = 150;
double Dk5 = 0.3;
double Setpoint5, Input5, Output5;    // PID variables
PID PID5(&Input5, &Output5, &Setpoint5, Pk5, Ik5 , Dk5, DIRECT);

#include <ESP32Servo.h>     // ESP32 Servo library from libraries manager

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

#define BNO08X_INT  19
#define BNO08X_RST  18
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int a;    // pot1
  int b;    // pot2
  int c; // button 1
  int d; // button 2

} struct_message;

int pot1;                   // remote joystick pots
int pot2;
float pot1Scaled;           // scaled values
float pot2Scaled;
float pot1ScaledFiltered;   // filtered values
float pot2ScaledFiltered;
float rotVel;               // extra for the back wheel rotation
int but1;                   // remote buttons
int but2;

int motorEnable;

int trimPot1;
int trimPot2;
float trimPot1Scaled;
float trimPot2Scaled;

float pitch;
float roll;

#define encoder0PinA 13
#define encoder0PinB 14

#define encoder1PinA 4
#define encoder1PinB 16

#define encoder2PinA 17
#define encoder2PinB 23

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
volatile long encoder2Pos = 0;

int encoder0PosPrev = 0;
int encoder1PosPrev = 0;
int encoder2PosPrev = 0;

int motor0Vel = 0;
int motor1Vel = 0;
int motor2Vel = 0;

int velY;       // front/back velocity
int velX;       // sideways velocity

int enc1;     // motor1 encoder
int enc2;     
int enc3;     // motor2 encoder
int enc4;
int enc5;     // motor3 encoder
int enc6;

unsigned long currentMillis;
unsigned long previousMillis = 0;   // set up timers

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  pot2 = myData.a - 1919;     // modify offset to get zero when stick is stationary
  pot1 = myData.b - 1950;     // modify offset to get zero when stick is stationary
  but1 = myData.c;
  but2 = myData.d;

  // un-comment lines below to get centre stick positions
/*
  Serial.print(myData.a);
  Serial.print(" , ");
  Serial.print(myData.b);
  Serial.println(" , ");
*/

  // threshold value for control sticks
  // makes a dead-band of +/- 50
  if (pot1 > 50) {
    pot1 = pot1 -50;
  }
  else if (pot1 < -50) {
    pot1 = pot1 +50;
  }
  else {
    pot1 = 0;
  }    
  
  // threshold value for control sticks
  if (pot2 > 50) {
    pot2 = pot2 -50;
  }
  else if (pot2 < -50) {
    pot2 = pot2 +50;
  }
  else {
    pot2 = 0;
  }  
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(10);

  PID3.SetMode(AUTOMATIC);              
  PID3.SetOutputLimits(-255, 255);
  PID3.SetSampleTime(10);

  PID4.SetMode(AUTOMATIC);              
  PID4.SetOutputLimits(-255, 255);
  PID4.SetSampleTime(10);

  PID5.SetMode(AUTOMATIC);              
  PID5.SetOutputLimits(-255, 255);
  PID5.SetSampleTime(10);

  pinMode(encoder0PinA, INPUT);     // motor 1 encoder
  pinMode(encoder0PinB, INPUT);     
  pinMode(encoder1PinA, INPUT);     // motor 2 encoder
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT);     // motor 3 encoder
  pinMode(encoder2PinB, INPUT);

  attachInterrupt(encoder0PinA, doEncoder0A, CHANGE);
  attachInterrupt(encoder0PinB, doEncoder0B, CHANGE);

  attachInterrupt(encoder1PinA, doEncoder1A, CHANGE);
  attachInterrupt(encoder1PinB, doEncoder1B, CHANGE);

  attachInterrupt(encoder2PinA, doEncoder2A, CHANGE);
  attachInterrupt(encoder2PinB, doEncoder2B, CHANGE);

  pinMode(26, OUTPUT);      // left motor
  pinMode(27, OUTPUT);
  pinMode(25, OUTPUT);      // right motor
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);      // back motor
  pinMode(5, OUTPUT);

  pinMode(35, INPUT);       // trim pot 1;
  pinMode(34, INPUT);       // trim pot 2;
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  Wire.begin();

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
      Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
      while (1);
  }
  Serial.println("BNO08x found!");

}


// Here is where you define the sensor outputs you want to receive
void setReports(void) {
    Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
}

 
void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {     // this loop runs every 10ms
      previousMillis = currentMillis;

      // ** READ IMU DATA **
      if (myIMU.wasReset()) {
        Serial.println("sensor was reset ");
        setReports();
      }    
      // Has a new event come in on the Sensor Hub Bus?
      if (myIMU.getSensorEvent() == true) {    
        // is it the correct sensor data we want?
        if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {     
            pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
            roll = (myIMU.getRoll()) * 180.0 / PI; // Convert pitch to degrees
        }
      }
      // ** END OF READ IMU DATA **

      /*
      // test encoder pins
      enc1 = digitalRead(encoder0PinA);
      enc2 = digitalRead(encoder0PinB);
      enc3 = digitalRead(encoder1PinA);
      enc4 = digitalRead(encoder1PinB);
      enc5 = digitalRead(encoder2PinA);
      enc6 = digitalRead(encoder2PinB);
      Serial.print(enc1);
      Serial.print(" , ");
      Serial.print(enc2);
      Serial.print('\t');
      Serial.print(enc3);
      Serial.print(" , ");
      Serial.print(enc4);
      Serial.print('\t');
      Serial.print(enc5);
      Serial.print(" , ");
      Serial.print(enc6);
      Serial.println();
      // end of test encoder pins
      */

      // read remote buttons for motor enable
      if (but2 == 0) {
        motorEnable = 0;
      }
      else if (but1 == 0) {
        motorEnable = 1;
      }
      

      // read analog trim pots mouted on robot

      trimPot1 = analogRead(35);
      trimPot1Scaled = float (trimPot1 - 2000)/250;
      trimPot2 = analogRead(34);
      trimPot2Scaled = float (trimPot2 - 2000)/250;

      // calculate motor velocities by comparing current eoncder count with the previous bookmarked value.
      
      motor0Vel = encoder0Pos - encoder0PosPrev;
      encoder0PosPrev = encoder0Pos;  

      motor1Vel = encoder1Pos - encoder1PosPrev;
      encoder1PosPrev = encoder1Pos; 

      motor2Vel = encoder2Pos - encoder2PosPrev;
      encoder2PosPrev = encoder2Pos;
      
      /*
      // test encoder counts from ISRs      
      //Serial.print(encoder0Pos);
      //Serial.print('\t');
      //Serial.print(encoder1Pos);
      //Serial.print('\t');
      //Serial.print(encoder2Pos);
      //Serial.print('\t');
      Serial.print(motor0Vel);
      Serial.print('\t');
      Serial.print(motor1Vel);
      Serial.print('\t');
      Serial.print(motor2Vel);
      Serial.print('\t');
      //Serial.println();
      
      // test remote data, IMU and onboard pots;

      Serial.print(pot1);
      Serial.print('\t');
      Serial.print(pot2);
      Serial.print('\t');
      Serial.print(but1);
      Serial.print('\t');
      Serial.print(but2);
      Serial.print('\t');
      Serial.print(motorEnable);
      
      Serial.print('\t');
      Serial.print(pitch);
      Serial.print('\t');
      Serial.print(roll);
      Serial.print('\t');
      Serial.println(); 
    
      Serial.print(trimPot1Scaled);
      Serial.print('\t');
      Serial.print(trimPot2Scaled);
      Serial.print('\t');
      Serial.println();
      */      

      pot1Scaled = (float) pot1 / 500;   // remote joystick scaling to degrees

      if (but1 == 1) {
        pot2Scaled = (float) pot2 / 500;
      }
      else {
        pot2Scaled = 0;
      }

      pot1ScaledFiltered = filter(pot1Scaled, pot1ScaledFiltered, 20);
      pot2ScaledFiltered = filter(pot2Scaled, pot2ScaledFiltered, 20);

      Input4 = pitch*-1;       // balancing sideways - pitch of IMU
      Setpoint4 = trimPot1Scaled + pot2ScaledFiltered;
      PID4.Compute();

      Input5 = roll;        // balancing front/back - roll of IMU
      Setpoint5 = trimPot2Scaled + pot1ScaledFiltered;
      PID5.Compute();
   
      velY = Output5;      // back/front velocity
      velX = Output4;      // sideways velocity
   

      
      Setpoint1 = velY - velX;     // required wheel 0 velocity
      Input1 = motor0Vel;   // actual wheel 0 velocity
      PID1.Compute();       // calc PID
      // velocity controller for wheel 1 - right
      Setpoint2 = velY + velX;     // required wheel 0 velocity
      Input2 = motor1Vel;   // actual wheel 0 velocity
      PID2.Compute();       // calc PID
      // velocity controller for wheel 2 - back
      Setpoint3 = (velX/1.5) - rotVel;     // required wheel 0 velocity
      Input3 = motor2Vel;   // actual wheel 0 velocity
      PID3.Compute();       // calc PID

      if (motorEnable == 1) {       // motor enable

          // velocity controller for wheel 0 - left
          if (Output1 > 0 && motorEnable == 1) {
            analogWrite(26, 0);
            analogWrite(27, Output1);
          }
          else if (Output1 < 0) {
            Output1a = abs(Output1);
            analogWrite(26, Output1a);
            analogWrite(27, 0);
          }
          else {
            analogWrite(26, 0);
            analogWrite(27, 0);
          }
          
          // velocity controller for wheel 1 - right
          if (Output2 > 0 && motorEnable == 1) {
            analogWrite(25, 0);
            analogWrite(33, Output2);
          }
          else if (Output2 < 0) {
            Output2a = abs(Output2);
            analogWrite(25, Output2a);
            analogWrite(33, 0);
          }
          else {
            analogWrite(25, 0);
            analogWrite(33, 0);
          }

          // rotation axis
          if (but1 == 0) {
              rotVel = (float) pot2 / 20;
          }
          else {
            rotVel = 0;
          }    
          
          // velocity controller for wheel 2 - back
          if (Output3 > 0) {
            analogWrite(32, 0);
            analogWrite(5, Output3);
          }
          else if (Output3 < 0) {
            Output3a = abs(Output3);
            analogWrite(32, Output3a);
            analogWrite(5, 0);
          }
          else {
            analogWrite(32, 0);
            analogWrite(5, 0);
          } 
      }

      else if (motorEnable == 0)      {         // stop motors
        analogWrite(26, 0);
        analogWrite(27, 0);
        analogWrite(25, 0);
        analogWrite(33, 0);
        analogWrite(32, 0);
        analogWrite(5, 0);
      }

    }  // end of 10ms timed loop
    
} // end of main loop

// motion filter to filter motions
float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}


// *** ENCODER 0 ISR ***
void doEncoder0A(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  } 
}
void doEncoder0B(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }  
}
// *** END OF ENCODER 0 ISR ***

// *** ENCODER 1 ISR ***
void doEncoder1A(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  } 
}
void doEncoder1B(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }  
}
// *** END OF ENCODER 1 ISR ***

// *** ENCODER 2 ISR ***
void doEncoder2A(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder2PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder2PinB) == LOW) {  
      encoder2Pos = encoder2Pos + 1;         // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder2PinB) == HIGH) {   
      encoder2Pos = encoder2Pos + 1;          // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;          // CCW
    }
  } 
}
void doEncoder2B(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder2PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder2PinA) == HIGH) {  
      encoder2Pos = encoder2Pos + 1;         // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder2PinA) == LOW) {   
      encoder2Pos = encoder2Pos + 1;          // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;          // CCW
    }
  }  
}
// *** END OF ENCODER 2 ISR ***

