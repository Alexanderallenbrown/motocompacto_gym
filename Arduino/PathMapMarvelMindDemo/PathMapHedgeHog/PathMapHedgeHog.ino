
#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <PathMap.h>
#include <MarvelMindSuperBeacon.h>
#include <ESP32Servo.h>

// Servos
Servo gasServo;
Servo steerServo;

int gasVal = 90;
int steerVal = 90;

MarvelMindSuperBeacon beacon(Serial2);

AsyncWebServer server(80);

PathMap path;

float x, y, z;
float roll, pitch, yaw;
float vx, vy, vz;
float previewDist = 1.5;

void setup() {
  Serial.begin(115200);
  WiFi.softAP("PathMap", "12345678");

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS failed"); while (1);
  }

  path.begin("/map.csv");
  beacon.begin();

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/preview", HTTP_GET, [](AsyncWebServerRequest * req) {
    if (req->hasParam("d")) previewDist = req->getParam("d")->value().toFloat();
    req->send(200, "text/plain", "OK");
  });

  // servo route
  server.on("/servo", HTTP_GET, [](AsyncWebServerRequest * req) {
    if (req->hasParam("gas")) {
      gasVal = req->getParam("gas")->value().toInt();
      gasServo.write(gasVal);
    }
    if (req->hasParam("steer")) {
      steerVal = req->getParam("steer")->value().toInt();
      steerServo.write(steerVal);
    }
    req->send(200, "text/plain", "OK");
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest * req) {
    float station = path.estimateStation(x, y, yaw);
    float ePrev   = path.previewLateralError(x, y, yaw, previewDist);

    String res = "{";
    res += "\"x\":" + String(x, 3) + ",";
    res += "\"y\":" + String(y, 3) + ",";
    res += "\"yaw\":" + String(yaw, 3) + ",";
    res += "\"station\":" + String(station, 3) + ",";
    res += "\"ePrev\":" + String(ePrev, 3);
    res += "}";

    req->send(200, "application/json", res);
  });

server.on("/map", HTTP_GET, [](AsyncWebServerRequest *req){
    File f = SPIFFS.open("/map.csv", "r");
    if(!f){
        req->send(404, "text/plain", "map.csv not found");
        return;
    }
    req->send(f, "text/plain", f.size());
    f.close();
});

  server.begin();
}

void loop() {
  beacon.update();
  beacon.getPosition(x, y, z);
  beacon.getEuler(roll, pitch, yaw);

  steerServo.write(steerVal);
  gasServo.write(gasVal);

  Serial.print(millis()); Serial.print(", ");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(yaw); Serial.print(", ");
  Serial.print(gasVal); Serial.print(", ");
  Serial.print(steerVal); Serial.println();
}
