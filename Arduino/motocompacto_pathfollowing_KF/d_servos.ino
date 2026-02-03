// https://github.com/atomic14/esp32-s3-pinouts?tab=readme-ov-file -> explains why 45,46 are not great to use.
// the V5 board should be programmed with Arduino using the "ESP32S3 dev module" board definition.

// SERVO STUFF
// Include the ESP32 Arduino Servo Library instead of the original Arduino Servo Library
#include <ESP32Servo.h> 

Servo kickservo;  // create servo object to control a servo
Servo steerservo;

int steerPin = 13;
int kickPin = 25;//GPIO 46 is connected to steer servo
                  //GPIO 45 is servo 3 (top connector)
                  //GPIO 47 is servo 2 (kickstand)
//void setupServos(){
//  /////servo setup
//  // Allow allocation of all timers
//  ESP32PWM::allocateTimer(0);
//  ESP32PWM::allocateTimer(1);
//  ESP32PWM::allocateTimer(2);
//  ESP32PWM::allocateTimer(3);
//  kickservo.setPeriodHertz(50);// Standard 50hz servo
//  kickservo.attach(kickPin, 400, 2400);   // attaches the servo on pin 18 to the servo object
//  steerservo.setPeriodHertz(50);
//  steerservo.attach(steerPin,400,2400);
//                                         // using SG90 servo min/max of 500us and 2400us
//                                         // for MG995 large servo, use 1000us and 2000us,
//                                         // which are the defaults, so this line could be
//                                         // "myservo.attach(servoPin);"
//  steerservo.write(steerCenterAngle);
//  kickservo.write(leanAngle);
//}

void setupSteering() {
  steerservo.setPeriodHertz(50);    // standard 50 hz servo
  pinMode(steerPin, OUTPUT);
  steerservo.attach(steerPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
  pinMode(kickPin, OUTPUT);
  kickservo.attach(kickPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

}

void setSteerangle(float posrad) {
  int pos = int(posrad * (180 / PI) + 90);
  //  Serial.print("\t Steeringpos: ");
  //  Serial.print(pos);
  //  Serial.print("\t");
  //  Serial.print("\t timer: ");
  //  Serial.print(tw);
  steerservo.write(pos);
  kickservo.write(pos);
}

void writeServos(int steerAngle, int kickAngle){
  kickservo.write(kickAngle);
  steerservo.write(steerAngle);
}
