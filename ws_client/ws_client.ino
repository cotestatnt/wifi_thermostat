#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define USE_SERIAL Serial

#include <WebSocketsClient.h>
WebSocketsClient webSocket;
const char* ssid = "ESP_12345";
const char* password = "12345678";

// Dallas DS18B20
#define ONE_WIRE_BUS D2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress ds18b20;

#define Relay D1
float Temperature = 25.0F;
float Setpoint = 25.0F;
uint32_t updateTempTime = millis();

void setup(void) {
  Serial.begin(115200);  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  pinMode(Relay, OUTPUT);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) {
    delay(500);
    if (i == 21) {
      Serial.print("Could not connect to"); 
      Serial.println(ssid);
    }     
  }
  
  webSocket.begin("10.10.10.10", 80, "/");
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

  EEPROM.begin(32);
  readEEprom(); 
}



void loop(void) {  
  if(millis() - updateTempTime > 1000){ 
    sensors.requestTemperatures();          
    Temperature = sensors.getTempC(ds18b20); 
  }
  if(Temperature > Setpoint + 0.5F){
    digitalWrite(Relay, HIGH);
  }

  if(Temperature < Setpoint - 0.5F){
    digitalWrite(Relay, LOW);
  }
  
  webSocket.loop();
}



void readEEprom(){
  uint16_t adr = 0;
  EEPROM.get(adr, Setpoint);    
  // First boot on fresh ESP8266
  if( Setpoint > 100 ) Setpoint = 25.0F;
}


void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: 
      USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);          
      break;
    case WStype_BIN:
      break;
    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);
      String msg = "";
      for(size_t i=0; i < length; i++) 
        msg += (char) payload[i];
      USE_SERIAL.printf("%s\n",msg.c_str());
      
      if(msg.indexOf("?TEMP") > -1){        
        USE_SERIAL.printf("Water temperature: %d°C\n", (int)Temperature);      
        char ws_buffer[10];
        sprintf(ws_buffer, ">%d.%2d<", (int)Temperature, (int)(Temperature*100)%100);               
        webSocket.sendTXT(ws_buffer);    
      }
      
      if(msg.indexOf(">SETPOINT<") > -1){                    
        Setpoint = msg.substring(msg.indexOf(">SETPOINT<")).toFloat();   
        uint16_t adr = 0;
        EEPROM.put(0, Setpoint);     
        EEPROM.commit(); 
        USE_SERIAL.printf("New setpoint: %d°C\n", (int)Setpoint);  
        webSocket.sendTXT("OK");    
      } 
      break;
   
  }

}
