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


//set up super beacon (GPS equivalent)
MarvelMindSuperBeacon beacon(Serial2);

//set up path
PathMap path;




void setupBeacon(){
  beacon.begin();
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
  beacon.update(); //reads the beacon
  oldx=x;oldy=y;oldz=z;
  beacon.getPosition(x, y, z);
  if(abs(x-oldx)>100){x=oldx;y=oldy;}if(abs(y-oldy)>100){y=oldy;x=oldx;}//crappy jumps
}

void getPathError(){
  ePrev = path.previewLateralError(x,-y,yaw,previewDist);
  dEPrev = (ePrev-ePrev_old)/dt;
  ePrev_old = ePrev; 
  station = path.estimateStation(x,-y,yaw);
}
