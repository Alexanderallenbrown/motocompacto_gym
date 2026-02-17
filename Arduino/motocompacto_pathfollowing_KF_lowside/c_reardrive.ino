#include <SPI.h> //for CAN chip
#include <mcp2515.h> //for CAN chip
#define cancs 5 //chip select pin for CAN chip
struct can_frame canMsg; //can message
MCP2515 mcp2515(5); //initialize mcp object

float eU = 0;
float goalU = 6;//mph, goal speed.
float intE_U = 0;
float kp_U = 1;
float ki_U = 25;
//for throttle interface
int throttle_pin = 12;
int throttle_read_pin = 35;//TRUE throttle read pin is 2, but can't use that with wifi. Using 35 (steer sensor) instead
const int ledChannel = 3;

void setupCAN() {
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
}


int Throttleread() {
  int Throttle_read = analogRead(2);
  return Throttle_read;
}
void writeDriveMotor(int counts) {
  int Throttle_out = map(counts, 0, 255, 50, 255);
  ledcWrite(throttle_pin, Throttle_out);
}

  void canSpeed(float &speedvar) {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if ((canMsg.can_id == 0x0031) ) {

      if (canMsg.can_dlc == 8) {
        unsigned int speedMsg = canMsg.data[1] << 8 | canMsg.data[2];
        speedvar =float(speedMsg / 200.0 * 1.26);
      }
    }
    
  }
}

void openLoopUControl(){
  int rawU = map(goalMPH,0,14,0,255);
  rawU = constrain(rawU,0,255);
  writeDriveMotor(rawU);
}

void doUControl(float goal) {
  if (goal == 0) {
    writeDriveMotor(0);
    intE_U = 0;
  }
  else {
    eU = goal - U;
    intE_U += eU * dt;
    float Vcomm = kp_U * eU + ki_U * intE_U;
    int Ccomm = int(Vcomm * 255 / 8.0);
    Ccomm = constrain(Ccomm, 0, 255);
    writeDriveMotor(Ccomm);
  }
}


void setupThrottle() {
  pinMode(throttle_pin, OUTPUT);
  //instead of AnalogWrite, must use ledc library so we can specify which timer to use
  ledcAttachChannel(throttle_pin,5000,8,ledChannel);
}


void doThrottleManual() {
  int Throttle_read = analogRead(throttle_read_pin);
//  Serial.print("Throttle reads: ");Serial.println(analogRead(throttle_read_pin));
  int Throttle_out = map(Throttle_read, 900, 4095, 25, 255);
  ledcWrite(throttle_pin, Throttle_out);
//writeDriveMotor(Throttle_out);
}
