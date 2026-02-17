#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MarvelMindSuperBeacon.h>
#include <PathMap.h>


//BNO set up: sample rate & address
//FFT: Do we want to add median/bias filter?
#define BNO055_SAMPLERATE_DELAY_MS (5)
//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//
//sensors_event_t a, g, temp;
const char *rollbiasfilename = "/roll_bias.txt";

unsigned long lastGPSPoll = 0;
int GPSDelay = 1;//allow only 10Hz updates


//set up super beacon (GPS equivalent)
MarvelMindSuperBeacon beacon(Serial2);

//set up path
PathMap path;

AckermannKF kf;


void setupBeacon(){
  beacon.begin(500000);
   // Initialize filter
    kf.begin(
        0.0f,   // initial speed
        3.5f,   // initial X
        5.5f,   // initial Y
        0.0f,   // initial yaw
        2500.0f, //was 25.0f  // GPS XY variance (5m std dev)
        0.05f,  // yaw measurement variance
        0.5f    // velocity time constant tauU (seconds)
    );

    // Tune trust levels
    kf.setGPSCovariance(25.0f);      // weak GPS
    kf.setGPSGateThreshold(9.21f);   // 99% gate

    // Optional: tune process noise
    kf.setProcessNoise(
        0.005f,    // U noise
        0.01f,   // X noise
        0.01f,   // Y noise
        0.0005f   // psi noise
    );
  
}

void setupPath(){
  path.begin("/map.csv");
}

void setupIMU(){
  // initialize MPU 6050 IMU. make sure to specify SDA,SCL pins with Wire.begin(SDA,SCL)
//  Wire.begin(15,16);///////////////// MUST ADD THIS!!!! /////////////////
  if (!bno.begin())
  {
    while (!bno.begin()) {
      Serial.print("No BNO055 detected");
      delay(100);
      //    while (1);
    }
  }
  
}

void readIMU(){
  sensors_event_t orientationData, gravityData, angVelocityData;
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  roll = -orientationData.orientation.z;
  rollRate = angVelocityData.gyro.x * 180 / PI; 
  yawRate = angVelocityData.gyro.z;
  yaw = (-orientationData.orientation.x*PI/180)-yawBias;
  rollFinal = roll-rollBiasPosition;
//  rollFieldValue = String(rollFinal)+" deg, goal: "+String(goalRoll_filt)+" deg";//// THIS IS THE OLD WAY OF SENDING DATA TO WEB
}

void readGPS(){
  if((millis()-lastGPSPoll)>GPSDelay){
  beacon.update(); //reads the beacon
//  Serial.println("reading GPS");
  lastGPSPoll = millis();
  }
  bool newGPS = beacon.positionAvailable();
  if(newGPS){
    lastGPSTime = millis();
  }
  GPSAge = millis()-lastGPSTime;
  oldx=x;oldy=y;oldz=z;
  beacon.getPosition(xraw, yraw, zraw);
  if(abs(xraw-oldx)>100){xraw=oldx;yraw=oldy;}if(abs(yraw-oldy)>100){yraw=oldy;xraw=oldx;}//crappy jumps
  //now update kf
  unsigned long now = micros();
  float U_cmd = 2.2*(s2||s3);
  kf.update(
        now,
        U_cmd,
        yawRate,
        newGPS,
       xraw,
        -yraw,
        yaw
    );
    x = kf.getX();
    y = -kf.getY();
  
}

void getPathError(){
  ePrev = path.previewLateralError(x,-y,kf.getPsi(),previewDist);
  dEPrev = (ePrev-ePrev_old)/dt;
  ePrev_old = ePrev; 
  station = path.estimateStation(x,-y,yaw);
}
