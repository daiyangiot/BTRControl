/* Daiyang IoT Bluetooth Remote Control Robot 2015
  * By Sonia Lee <daiyangelectrical.com.au> <openmylab.com>
  * This sketch contains bluetooth control (HC-06, remote control- Dai yang Robots app)
  * Upload sketch BEFORE attaching bluetooth to robot, or else leads to error
  * Components: Arduino Uno, sensor shield, ultrasonic distance sensor, 1 servo, motor driver, 2x motors
  * Wires also called "dupont" wires. The ends: male has pin end, female has open end (no pin)
  * For instructions on hardware and wiring, please become a member and "adopt a robot" :)
    Members have access to free workshops, technical support, monthly experiments
  */
  
#include <SoftwareSerial.h>
#include <VarSpeedServo.h> 

VarSpeedServo myservo;

SoftwareSerial BT(0,1);       //TXD, RXD (plug into bluetooth pins on sensor shield; Switch TXD & RXD plugs if Bluetooth is not producing movement; +VCC, GND, RXD, TXD)
String readdata;

  int in1 = 9;
  int in2 = 11;
  int in3 = 10;
  int in4 = 6;
  int Trig = 3;
  int Echo = 2;
  const int servoPin = 5;

void setup() {
 BT.begin(9600);
 Serial.begin(9600);

     pinMode (in1, OUTPUT);   
     pinMode (in2, OUTPUT);   
     pinMode (in3, OUTPUT);   
     pinMode (in4, OUTPUT); 
     myservo.attach(servoPin); 
     myservo.write(100,100,true);
}

void loop() {
  while (BT.available()){  
  delay(10);
  char c = BT.read(); 
  readdata += c; 
  } 
  if (readdata.length() > 0) {
    Serial.println(readdata);

  if(readdata == "forward")
  {
    digitalWrite(in1, LOW);
    digitalWrite (in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(100);
  }

  else if(readdata == "reverse")
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(100);
  }

  else if (readdata == "right")
  {
    digitalWrite (in1, HIGH);  
    digitalWrite (in2, HIGH);  
    digitalWrite (in3, LOW); 
    digitalWrite (in4, LOW);   
    delay (100);
   
  }

 else if ( readdata == "left")
 {
   digitalWrite (in1, LOW); 
   digitalWrite (in2, LOW); 
   digitalWrite (in3, HIGH); 
   digitalWrite (in4, HIGH);  
   delay (100);
 }

 else if (readdata == "stop")
 {
   digitalWrite (in1, HIGH);
   digitalWrite (in2, HIGH);
   digitalWrite (in3, HIGH);
   digitalWrite (in4, HIGH);
   delay (100);
 }
 else if (readdata == "sweep")
  {
  myservo.write(180,40,true);   
  myservo.write(60,20,true);      
  }
readdata="";}}
