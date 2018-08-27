#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define USE_SERIAL Serial

WebSocketsClient webSocket;
const char* ssid = "ESP_12345";
const char* password = "12345678";

// Dallas DS18B20
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress ds18b20;
// DeviceAddress InTemp = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
// DeviceAddress OutTemp   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };

float Temperature = 25.0F;

void setup(void) {
  Serial.begin(115200);  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) {
    delay(500);
    if (i == 21) {
      Serial.print("Could not connect to"); 
      Serial.println(ssid);
    }     
  }
  
  webSocket.begin("42.42.42.42", 81, "/");
  webSocket.onEvent(webSocketEvent);  
  webSocket.setReconnectInterval(5000);

  sensors.begin();
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  if (!sensors.getAddress(ds18b20, 0)) Serial.println("Unable to find address for Device 0");
  sensors.setResolution(ds18b20, 10);
    
}


void loop(void) {   
  webSocket.loop();
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

  switch(type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: 
      USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);
      webSocket.sendTXT("Connected");    
      break;
    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);
      if(payload[0] == '?'){
        sensors.requestTemperatures();          
        Temperature = sensors.getTempC(ds18b20);
        char wsBuffer[10];
        char tmp[8];
        dtostrf(Temperature, 6, 2, tmp);        
        USE_SERIAL.printf("Water temperature: %s°C\n", tmp);         
        sprintf(wsBuffer, ">%s<", tmp);
        webSocket.sendTXT(wsBuffer);    
      }      
      break;

  }

}
