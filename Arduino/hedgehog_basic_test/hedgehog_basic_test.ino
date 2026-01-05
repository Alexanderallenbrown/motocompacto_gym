#include <Marvelmind.h>

MarvelmindHedge hedge;
FusionIMUValue fusionIMU;
PositionValue hedgePosition;

float x_m, y_m, z_m;
float qw, qx, qy, qz;
float vx, vy, vz, ax, ay, az;

float roll, pitch, yaw;
//float r2d = 57.2957795; // 1 Radian = 57.2957795 degrees

unsigned long t1;
const unsigned long period = 100;

#define RXD2 16 //for ESP32-WROOM-DA module
#define TXD2 17 //for ESP32-WROOM-DA module
long baudrate = 115200;

void setup() {
  Serial.begin(baudrate);
  Serial2.begin(baudrate, SERIAL_8N1, RXD2, TXD2);
//  Serial2.begin(baudrate);

  hedge.begin(&Serial2);
  t1 = millis();
}

void loop() {
  hedge.read();
  if(millis() - t1 > period){
//    hedge.getFusionIMUFromMarvelmindHedge(&fusionIMU);
//
//    x_m = ((float) fusionIMU.x) / 1000.0f;
//    y_m = ((float) fusionIMU.y) / 1000.0f;
//    z_m = ((float) fusionIMU.z) / 1000.0f;

    hedge.getPositionFromMarvelmindHedge(&hedgePosition);

    x_m = ((float) hedgePosition.x) / 1000.0f;
    y_m = ((float) hedgePosition.y) / 1000.0f;
    z_m = ((float) hedgePosition.z) / 1000.0f;

//    qw = ((float) fusionIMU.qw) / 10000.0f;
//    qx = ((float) fusionIMU.qx) / 10000.0f;
//    qy = ((float) fusionIMU.qy) / 10000.0f;
//    qz = ((float) fusionIMU.qz) / 10000.0f;
//
//    vx = ((float) fusionIMU.vx) / 1000.0f;
//    vy = ((float) fusionIMU.vy) / 1000.0f;
//    vz = ((float) fusionIMU.vz) / 1000.0f;
//
//    ax = ((float) fusionIMU.ax) / 1000.0f;
//    ay = ((float) fusionIMU.ay) / 1000.0f;
//    az = ((float) fusionIMU.az) / 1000.0f;
//
//    yaw   = ( atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz) ) * r2d;
//    pitch = ( asin(-2.0*(qx*qz - qw*qy)) ) * r2d;
//    roll  = ( atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz) ) * r2d;

    Serial.print(" X= ");
    Serial.print(x_m, 3);
    Serial.print(" Y= ");
    Serial.print(z_m, 3);
    Serial.print(" Z= ");
    Serial.print(z_m, 3);

    Serial.print(" | ");
//
//    Serial.print(" R= ");
//    Serial.print(roll, 3);
//    Serial.print(" P= ");
//    Serial.print(pitch, 3);
//    Serial.print(" Y= ");
//    Serial.print(yaw, 3);

//  Serial.print(Serial2.available());

    Serial.println();

    t1 = millis();
  }
}
