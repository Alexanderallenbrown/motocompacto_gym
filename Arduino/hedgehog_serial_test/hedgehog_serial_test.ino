
#define RXD2 16 //for ESP32-WROOM-DA module
#define TXD2 17 //for ESP32-WROOM-DA module
long baudrate = 115200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(baudrate);
  Serial2.begin(baudrate, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  // put your main code here, to run repeatedly:
while (Serial2.available()){
  Serial.print(Serial2.read());
}
}
