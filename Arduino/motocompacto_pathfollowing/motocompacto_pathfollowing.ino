#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>  // must come after AsyncTCP
#include <ESPmDNS.h>
#include "SPIFFS.h"
/*
 * This code is different from the "demo" in that the webpage now has ONLY a goal roll slider, and both a "logging" and a "go" button.
 *
 */
int modeBTN_Pin = 21;
int driveVal = 220;
int goalMPH = 1;

// Global variable for slider position
volatile int rollSliderPosition = 0;  // Default value
volatile int rollBiasPosition = 0; 
bool logging = false;
bool oldlogging = false;
bool goButtonState = false;
// Mutex for thread safety
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float az=0;
float gy=0;

float yaw = 0;
float roll = 0;
float roll_a=0;
float rollBias = 0; 
float rollFinal = 0;
String rollFieldValue = "Waiting...";
float rollRate = 0;
float yawRate = 0;
float yawAngle = 0;
float alpha= 0.99; 
float desFilt_tau = 0.25;//0.125;//0.25;//seconds, time constant for filter on goal roal
float goalRoll_filt = 0;
float goalRoll = 0;

float goduration = 100;
float startime = 0;
float Goelapsed = 0;
volatile long eCounts = 0;
long eCountsSaved = 0;
long oldCounts = 0;
float U = 0;
String speedFieldValue = "Waiting...";
float dt = 0;
float told = 0;
float t = 0;

int stowAngle = 170;
int upAngle = 110;
int leanAngle = 120;
float steerCenterAngle = 103;
float steer = 0;
float steerFinal = 0;
float steerFB  = 0; //steering feedback
//control gain
float Klocus = .9;
float Kp = 6;
float Ki = 2; 
float Kd = .75;//0.1 worked with complementary filter (and no kick servo)
float intE = 0;


float x, y, z;
float oldx,oldy,oldz;
float yawBias=0;
float KPrev = .05;
float KdPrev = .05;
float previewDist = 5.0;
float ePrev = 0;
float station = 0;
float ePrev_old = 0;
float dEPrev = 0;


//fsm implementation
int state; 

bool s1=true; //STOP
bool s2=false; //SWING UP
bool s3=false; //GO
bool s4=false; //SWING DOWN

bool t1; //s1 latch
bool t2; //s1 -> s2
bool t3; //s2 latch
bool t4; //s2->s3
bool t5; //s3 latch
bool t6; //s3->s4
bool t7; //s4 latch
bool t8; //s4->s1

bool sd_sig = false;
bool su_sig = false;
bool roll_sig = false;
float su_elapsed=0; //swing up timer 
float su_start=0; //swing up start timer
float sd_elapsed=0; //swing down timer
float sd_start=0;


void setup() {
    pinMode(modeBTN_Pin,INPUT);
    Serial.begin(115200);
    //handles the setup of the webpage, access point, and server
    setupWebServerAP();
    //setup CAN bus for rear drive speed measurement
    setupCAN();
    //set up throttle control
    setupThrottle();
    //set up servos
    //setupServos();
    setupSteering();
    //setup IMU
    setupIMU();
    //setup path
    setupPath();
    //setup hedgehog beacon
    setupBeacon();
    
    
    
}

void loop() {
    t = micros() / 1.0e6;
    dt = t - told;

    doFSM();

    //note that this whole void loop should really be in an FSM tab with blocks in appropriate places.
    //these globals all get updated asynchronously by the webpage, so
    //it's important to get local copies to avoid corruption.
  
//    portENTER_CRITICAL(&mux);
//    int currentRollSlider = rollSliderPosition;
//    portEXIT_CRITICAL(&mux);
//
//    portENTER_CRITICAL(&mux);
//    int currentBiasSlider = rollBiasPosition;
//    portEXIT_CRITICAL(&mux);

   


//    Serial.print(millis() / 1000.0, 4);
//    Serial.print(",");
//    Serial.print(s1);
//    Serial.print(s2);
//    Serial.print(s3);
//    Serial.print(s4);
//    Serial.print(",");
//    Serial.print("Current Button State: ");
//    Serial.print(goButtonState);
//    Serial.print(",");
//    Serial.print("Rollbias ");
//    Serial.print(rollBiasPosition);
//    Serial.print(",");
//    Serial.print(U);
//    Serial.print(",");
    Serial.print(",");
    Serial.print(goalRoll_filt);
    Serial.print(rollFinal);
    Serial.print(",");
    Serial.print(rollRate);
    Serial.print(",");
    Serial.print(yawRate);
//    Serial.print(",");
//    Serial.print(Throttleread());
    
    Serial.println();
//    Serial.print(",");
//    Serial.print(rollRate_bno);
//    Serial.print(",");
//    Serial.println(dt);
    

    delay(1); // Adjust as needed
}
