
/*
   NOTE:
   The async web server library can be downloaded here:
   https://github.com/me-no-dev/ESPAsyncWebServer/archive/refs/heads/master.zip
   You also need the asynctcp library, which can be downloaded from
   the libraries manager (the web server library cannot)
*/

unsigned long lastSend = 0;

// http://192.168.4.1/
// AP Credentials

//pull MAC address so this bike has a unique Wifi network name
uint8_t mac[6];
//WiFi.macAddress(mac);

//SSID will include last byte of mac address for uniquenessw
char ssid[32]; // Adjust size as needed
//snprintf(ssid, sizeof(ssid), "MicroBike_%02", mac[0]);

//const char *ssid = "Compacto_WebServer";
const char *password = "compacto";

//create a desired hostname. Will be at "microbike.local"
const char* hostname = "compacto";
// Create AsyncWebServer instance on port 80
AsyncWebServer server(80);

//websocket for data streaming:
AsyncWebSocket ws("/ws");




// ---------------- WebSocket handler ----------------
void onWsEvent(
    AsyncWebSocket* server, 
    AsyncWebSocketClient* client, 
    AwsEventType type, 
    void* arg, 
    uint8_t* data, 
    size_t len
) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected\n", client->id());
            break;

        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;

        case WS_EVT_DATA:
            // This triggers when a message is received from a client
            if (data && len > 0) {
                Serial.print("Received data from client #");
                Serial.print(client->id());
                Serial.print(": ");
                for (size_t i = 0; i < len; i++) {
                    Serial.print((char)data[i]);
                }
                Serial.println();
            }
            break;

        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            // Optional: handle ping/pong or errors
            break;
    }
}
// ---------------- Send data to all clients ----------------
void sendData() {
    if (millis() - lastSend >= 10) {  // 100 Hz
        lastSend = millis();

        String res = "{";
        res += "\"x\":" + String(x,3) + ",";
        res += "\"y\":" + String(-y,3) + ",";
        res += "\"yaw\":" + String(yaw,3) + ",";
        res += "\"station\":" + String(station,3) + ",";
        res += "\"ePrev\":" + String(ePrev,3) + ",";
        res += "\"rollFinal\":" + String(rollFinal,3) + ",";
        res += "\"goalRoll_filt\":" + String(goalRoll_filt,3) + ",";
        res += "\"rollRate\":" + String(rollRate,3) + ",";
        res += "\"steer\":" + String(steer,3);
        res += "}";

        ws.textAll(res);   // broadcast to all connected clients
    }
}

void setupWebServerAP() {

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS failed"); while (1);
  }
  // Start AP
  WiFi.macAddress(mac);
  snprintf(ssid, sizeof(ssid), "Compacto:%12X", mac);
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  //use MDNS to handle domain name
  WiFi.setHostname(hostname);
  if (MDNS.begin(hostname)) {
    Serial.println("mDNS started");
  }
  else {
    Serial.println("mDNS failed");
  }

  // Serve the webpage
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  //websocket stuff:
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);


  /////// NEW HANDLERS. NEED TO MAKE BUTTONS AND JS FOR THESE
  server.on("/map", HTTP_GET, [](AsyncWebServerRequest * req) {
    if (!SPIFFS.exists("/map.csv")) {
      req->send(404, "text/plain", "map.csv not found");
      return;
    }

    req->send(SPIFFS, "/map.csv", "text/plain");
  });

  server.on("/zero_yaw", HTTP_GET, [](AsyncWebServerRequest * request) {
    yawBias += yaw;
    request->send(200, "text/plain", "OK");
  });
//  server.on("/data", HTTP_GET, [](AsyncWebServerRequest * req) {
//    String res = "{";
//    res += "\"x\":" + String(x, 3) + ",";
//    res += "\"y\":" + String(-y, 3) + ",";
//    res += "\"yaw\":" + String(yaw, 3) + ",";
//    res += "\"station\":" + String(station, 3) + ",";
//    res += "\"ePrev\":" + String(ePrev, 3)+ ",";
//    res += "\"rollFinal\":" + String(rollFinal,3) +",";
//    res += "\"goalRoll_Filt\":" + String(goalRoll_Filt,3) +",";
//    res += "\"rollRate\":" + String(rollRate,3) +",";
//    res += "\"rollFinal\":" + String(rollFinal,3) +",";
//    res += "\"steer\":" + String(steer,3);
//    res += "}";
//
//    req->send(200, "application/json", res);
//  });


  /////// OLD HANDLERS. CLEAN UP AND GET RID OF EXTRAS


  //handle go toggle
  server.on("/toggle_go", HTTP_GET, [](AsyncWebServerRequest * request) {
    portENTER_CRITICAL(&mux);
    goButtonState = !goButtonState;
    String buttonText = goButtonState ? "Stop Vehicle" : "Start Vehicle";
    portEXIT_CRITICAL(&mux);

    String response = "{\"buttonText\":\"" + buttonText +  "}";
    request->send(200, "application/json", response);
  });
  
  server.on("/zero_roll", HTTP_GET, [](AsyncWebServerRequest * request) {
    portENTER_CRITICAL(&mux);
    rollBiasPosition += rollFinal;
    portEXIT_CRITICAL(&mux);
  });


  //
  // Start server
  server.begin();
}
