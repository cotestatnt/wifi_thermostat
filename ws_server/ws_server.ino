#include <ESP8266WiFi.h>
#include <EEPROM.h>

#include <WebSocketsServer.h>
IPAddress AP_IP(42,42,42,42);
const uint8_t ws_port = 81;
WebSocketsServer webSocket = WebSocketsServer(ws_port);

// Dallas DS18B20 library
#include <DallasTemperature.h>
#define ONE_WIRE_BUS D5
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress ds18b20;

// Button event handler
#include <SeqButton.h>
SeqButton  ShortClick, LongClick;

// OLED display library   
#define SDA D6
#define SCL D7
#include <Wire.h>
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);
#define thermo_width 32
#define thermo_height 32

// Stranamente, nel font contenente il set di icone fornito con la libreria, 
// non c'è nulla che ha a che fare con la temperatura. Usiamo quindi delle bitmap XBM ( create con Gimp).
static const unsigned char thermometer1_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00,
   0x00, 0xe0, 0x07, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x60, 0x06, 0x0c,
   0x00, 0x60, 0x06, 0x0e, 0xf0, 0x67, 0x06, 0x0f, 0x00, 0x60, 0x86, 0x0f,
   0x80, 0x67, 0x86, 0x0d, 0x00, 0x60, 0x06, 0x0c, 0x80, 0x67, 0x06, 0x0c,
   0x00, 0x60, 0x06, 0x0c, 0xf0, 0x67, 0x06, 0x0c, 0x00, 0x60, 0x06, 0x0c,
   0x80, 0x67, 0x06, 0x0c, 0x00, 0x60, 0x06, 0x0c, 0x80, 0x67, 0x06, 0x0c,
   0x00, 0x60, 0x06, 0x0c, 0xf0, 0x67, 0x06, 0x3f, 0x00, 0x60, 0x06, 0x3f,
   0x00, 0x60, 0x06, 0x00, 0x00, 0x70, 0x0e, 0x00, 0x00, 0x38, 0x1c, 0x00,
   0x00, 0x98, 0x19, 0x00, 0x00, 0x98, 0x19, 0x00, 0x00, 0x18, 0x18, 0x00,
   0x00, 0x30, 0x0c, 0x00, 0x00, 0xe0, 0x07, 0x00, 0x00, 0xc0, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const unsigned char thermometer2_bits[] PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03, 0x00,
   0x00, 0xe0, 0x07, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x60, 0x06, 0x0f,
   0x00, 0x60, 0x86, 0x1f, 0xf0, 0x67, 0x86, 0x39, 0x00, 0x60, 0xc6, 0x30,
   0x80, 0x67, 0xc6, 0x30, 0x00, 0x60, 0x06, 0x30, 0x80, 0x67, 0x06, 0x38,
   0x00, 0x60, 0x06, 0x18, 0xf0, 0x67, 0x06, 0x0c, 0x00, 0x60, 0x06, 0x0e,
   0x80, 0x67, 0x06, 0x06, 0x00, 0x60, 0x06, 0x03, 0x80, 0x67, 0x86, 0x01,
   0x00, 0x60, 0xc6, 0x01, 0xf0, 0x67, 0xc6, 0x3f, 0x00, 0x60, 0xc6, 0x3f,
   0x00, 0x60, 0x06, 0x00, 0x00, 0x70, 0x0e, 0x00, 0x00, 0x38, 0x1c, 0x00,
   0x00, 0x98, 0x19, 0x00, 0x00, 0x98, 0x19, 0x00, 0x00, 0x18, 0x18, 0x00,
   0x00, 0x30, 0x0c, 0x00, 0x00, 0xe0, 0x07, 0x00, 0x00, 0xc0, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Debug serial type
#define DBG_SERIAL Serial

// Output 
#define RelayPin D1

// Input
#define SWITCH D2
#define ENC_A D3    
#define ENC_B D4  

// Gloabl variables
long lastEncoder, encoder = 0;
char oled_buffer[30];
uint8_t varSelector, pageSelector = 0;
bool ProgramMode = false , ShortClickEnabled = true;
uint32_t updateTime = millis();
uint32_t updateOledTime = millis();        

float T1 = 30.0F, T2 = 45.0F;
float Setpoint = 50.0F;
float Hysteresys = 2.0F;
boolean relayMode = 0;  //  0:  T1 - external; >0 T1 - Internal

void setup(void) {
  digitalWrite(RelayPin, HIGH);             // Make sure relay is off when start 
  pinMode(RelayPin, OUTPUT);                // Output mode to drive relay
  pinMode(SWITCH, INPUT_PULLUP);  
  pinMode(ENC_A, INPUT_PULLUP); 
  pinMode(ENC_B, INPUT_PULLUP);  
  ShortClick.init(SWITCH, NULL, &f_ShortClick, false, LOW, 60);
  LongClick.init(SWITCH, &f_LongClick, NULL, false, LOW, 2000);  
  
  DBG_SERIAL.begin(115200);
  DBG_SERIAL.println(); 
  DBG_SERIAL.println("Configuring access point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255,255,255,0));
  WiFi.softAP("ESP_12345", "12345678");   // password MUST ust be 8-32 character
  IPAddress myIP = WiFi.softAPIP();
  // Start websocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  DBG_SERIAL.printf("\nWebSocket server ready!\nAddress: ws://%d.%d.%d.%d:%d", myIP[0], myIP[1], myIP[2], myIP[3], ws_port );  
 
  sensors.begin();
  DBG_SERIAL.printf("\nFound %d devices", sensors.getDeviceCount());    
  DBG_SERIAL.printf("\nParasite power is: %d.", sensors.isParasitePowerMode());
  if (!sensors.getAddress(ds18b20, 0)) 
    DBG_SERIAL.println("Unable to find address for Device 0");
  sensors.setResolution(ds18b20, 10);

  oled.begin();  
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
  EEPROM.begin(32);
  readEEprom();    
}


void loop(void) {    
  webSocket.loop();
  
  ShortClick.handler();
  LongClick.handler();
  
  update_oled();
  update_serial_msg();

  // Aggiorniamo la situazione ogni 1 secondo
  if(millis() - updateTime > 1000){
    updateTime = millis();    
    sensors.requestTemperatures();    
    if(relayMode == 0){
      T1 = sensors.getTempC(ds18b20);
      T1 = constrain(T1, -55, 99);
    }
    else {
      T2 = sensors.getTempC(ds18b20);
      T2 = constrain(T2, -55, 99);
    }
    webSocket.broadcastTXT("?TEMP?");
    
    if((T2 + Hysteresys >=  T1)&&(T2 > Setpoint))
      digitalWrite(RelayPin, LOW);
    if((T2 + Hysteresys <=  T1)&&(T2 > Setpoint))
      digitalWrite(RelayPin, HIGH);
  }  
  
  // L'encoder rotativo è stato ruotato in fase di programmazione?
  if((encoder != lastEncoder)  && (encoder != 0)){  
    lastEncoder = encoder;
    delay(10);   
    // Variazione dei parametri. La variabile varSelector determina quale parametro viene modificato
    if(pageSelector == 1){      
      switch(varSelector){ 
        case 0:
          break;
        case 1:   
          Setpoint = Setpoint + (float)encoder;    
          Setpoint = constrain(Setpoint, -10, 99);      
          DBG_SERIAL.printf("\nSetpoint: %d°C", (int)Setpoint);   
          break; 
        case 2: 
          Hysteresys = Hysteresys + (float)encoder;               
          Hysteresys = constrain(Hysteresys, 0, 10);             
          DBG_SERIAL.printf("\nHysteresys %d°C",(int)Hysteresys);  
          break; 
        case 3: 
          relayMode = !relayMode; 
          DBG_SERIAL.println(relayMode ? "T1: remoto" : "T1: locale");    // C++ ternary operator (x == y) ? a : b   
          break;
      }        
      // Force oled update
      updateOledTime = millis();
    }    
    encoder = 0;    
  }

}

// **************************  **********************************************

// Filtriamo un po' i valori ricevuti dal websocket
float output = 0;
float smooth(float newData){
  output += (newData - output) * 0.8;  
}

void readEEprom(){
  uint16_t adr = 0;
  EEPROM.get(adr, Setpoint);  
  adr += sizeof(float);
  EEPROM.get(adr, Hysteresys);  
  adr += sizeof(uint8_t);
  EEPROM.get(adr, relayMode);
  Setpoint = constrain(Setpoint, -10, 100);  
  Hysteresys = constrain(Hysteresys, 0, 10); 
}

/*
  La variabile "stateEncoder" terrà conto contemporaneamente dello stato precedente
  e di quello attuale degli ingressi collegati all'encoder PINA e PINB secondo il seguente schema:
 
  statoEncoder bit n:
  7     6       5       4       3       2       1       0
  NULL  NULL    NULL    NULL    OLDA    OLDB    PINA    PINB
 
  Per come funziona un encoder in quadratura, solo alcune combinazioni sono valide:
 
                            _______         _______      
                PinA ______|       |_______|       |______ PinA
  negativo <---         _______         _______         __      --> positivo
                PinB __|       |_______|       |_______|   PinB
 
        OLDA    OLDB    PINA    PINB    Valore  Array
        0       0       0       1       +1      (indice 1)
        0       0       1       0       -1      (indice 2)
        0       1       0       0       -1      (indice 4)
        0       1       1       1       +1      (indice 7)
        1       0       0       0       +1      (indice 8)
        1       0       1       1       -1      (indice 11)
        1       1       0       1       -1      (indice 13)
        1       1       1       0       +1      (indice 14)        
*/
void read_encoder(void) {   
  const int validStates[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, +1, 0};  
  static volatile uint8_t stateEncoder, lastStateEncoder = 0;
  static volatile uint8_t arrayIndex = 0;  
  static bool tictoc = true;
  stateEncoder <<= 2;                                               // Eseguo uno shift di due posizioni a sinistra per non sovrascrivere la lettura precedente    
  stateEncoder |=  digitalRead(ENC_A) << 1 | digitalRead(ENC_B) ;   // Imposto i bit relativi allo stato attuale del canale A e del canale B      
  arrayIndex = 0b00001111 & stateEncoder;                           // Applico la maschera per tenere conto solo dei 4 bit di encoderState realmente usati 
   
  if(stateEncoder != lastStateEncoder && tictoc){    
    lastStateEncoder = stateEncoder;
    encoder += validStates[arrayIndex]; 
  }
  tictoc = !tictoc;
}



///// ****************** Short Switch callback function ****************** /////
void f_ShortClick(SeqButton * button){  
  if(ShortClickEnabled){
    DBG_SERIAL.println("\nShort Click"); 
    if(pageSelector == 1){
      // In fase di programmazione selezioniamo quale variabile sarà modificata
      varSelector = (varSelector+1) % 4;      
    }        
  }
  else
    ShortClickEnabled = true;  
  // Se non sono in fase di programmazione, mostro gli attuali parametri di funzionamento.
  if(!ProgramMode) {   
    oled.firstPage();
    do {
      oled.setFont(u8g2_font_courB10_tf );
      sprintf(oled_buffer, "Setpoint %2d%cC", (int)Setpoint, 0xB0);
      oled.drawStr(0, 25, oled_buffer);  
      sprintf(oled_buffer, "Isteresi %c%2d%cC", 0xB1, (int)Hysteresys, 0xB0);
      oled.drawStr(0, 50, oled_buffer);        
    } while ( oled.nextPage() ); 
    delay(1000);    
  }
 
}

///// ****************** Long Switch callback function ****************** /////
void f_LongClick(SeqButton * button){  
  DBG_SERIAL.println("\nLong Click"); 
  switch(pageSelector){
    case 0:
      ProgramMode = true;      
      ShortClickEnabled = false;    
      DBG_SERIAL.println("\nProgram mode ON");
      pageSelector = 1;    
      varSelector = 0;  
      update_oled();
      delay(500);
      break;
    case 1:
      pageSelector = (pageSelector+1) % 3;
      update_oled();
      ShortClickEnabled = false;
      break;    
    case 2:         
      uint16_t adr = 0;
      EEPROM.put(adr, Setpoint);      
      adr += sizeof(float);
      EEPROM.put(adr, Hysteresys);
      adr += sizeof(uint8_t);
      EEPROM.put(adr, relayMode);
      EEPROM.commit();
      pageSelector = 0;      
      update_oled();
      delay(2000);
      ShortClickEnabled = true;    
      break;      
  }
}


void update_serial_msg(){
  static uint32_t updateSerial;   
  if(millis() - updateSerial > 5000){     
    updateSerial = millis(); 
    switch(pageSelector){
      case 0:          
        DBG_SERIAL.printf("\n\nActual Setpoint: %d", (int)Setpoint);
        DBG_SERIAL.printf("\nInternal temperature: %d.%02d°C", (int)T1%100, (int)(T1*100)%100 );
        DBG_SERIAL.printf("\nExternal temperature: %d.%02d°C", (int)T2%100, (int)(T2*100)%100);   
        break;
      case 1:    
        break;    
      case 2:
       DBG_SERIAL.println("\nPush button for 1s to save EEPROM.");     
       break;
    }
  }
}


// **************************      OLED DISPLAY  Handler     *************************
void update_oled(){  
  if(millis() - updateOledTime > 100){  
    updateOledTime = millis();  
    oled.firstPage();
    do {       
      switch(pageSelector){
        
        // Pagina principale
        case 0:                         
          oled.drawXBMP( 0, 0, thermo_width, thermo_height, thermometer1_bits);
          // Se il relé è attivo, cambiamo icona e facciamola blinkare 
          if(digitalRead(RelayPin) == LOW){
            oled.setFont(u8g2_font_open_iconic_all_4x_t);   
            static uint32_t blinkIconTime = millis();
            static bool blinkIcon = false;            
            if(millis() - blinkIconTime > 500){
              blinkIconTime = millis();
              blinkIcon = !blinkIcon;               
            } 
            sprintf(oled_buffer, "%c", blinkIcon ? 0xCD : 0x20);  
            oled.drawStr(0, 64, oled_buffer);  
          }
          else
            oled.drawXBMP( 0, 34, thermo_width, thermo_height, thermometer2_bits);
          
          // Ora stampiamo il valore delle due temperature
          oled.setFont(u8g2_font_osb26_tf);   
          // Per avere maggior flessibilità uso sprintf per formattare l'array di char oled_buffer.
          // In questo modo passo direttamente il codice dei caratteri speciali.
          // Ad esempio il codice 0xB0 per il carattere selezionato corrisponde al simbolo '°'
          sprintf(oled_buffer, "%02d%cC", (int)T1, 0xB0);  
          oled.drawStr(35, 28, oled_buffer);  
          sprintf(oled_buffer, "%02d%cC", (int)T2, 0xB0);
          oled.drawStr(35, 64, oled_buffer);                           
          break;
          
        // Pagina di variazione dei parametri di funzionamento  
        case 1:
          oled.setFont(u8g2_font_courB10_tf);
          sprintf(oled_buffer, "Setpoint %2d%cC", (int)Setpoint, 0xB0);     // 0xB0 == '°'  0xB1 == '+-'
          oled.drawStr(2, 13, oled_buffer);  
          sprintf(oled_buffer, "Isteresi %c%2d%cC", 0xB1, (int)Hysteresys, 0xB0);   
          oled.drawStr(2, 37, oled_buffer);   
          oled.drawStr(2, 60, relayMode ? "T1: remota" : "T1: locale");            
          
          // Nella fase di programmazione dei parametri, 
          // evidenziamo quello che si sta modificando con una cornice
          if(varSelector == 1)            
            oled.drawFrame(0, 1, 128, 17);
          else if (varSelector == 2) 
            oled.drawFrame(0, 24, 128, 17);
          else if (varSelector == 3) 
            oled.drawFrame(0, 47, 128, 17);                     
          break; 
             
        // Pagina di conferma salvataggio parametri
        case 2:  
          oled.setFont(u8g2_font_courB10_tf);  
          oled.drawStr(28, 30, "Premi 1s");
          oled.drawStr(15, 50, "per salvare");        
         break;
      }
    } while ( oled.nextPage() );  
  } 
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      DBG_SERIAL.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:      
      DBG_SERIAL.printf("[%u] New client connected.\n", num);
      webSocket.sendTXT(num, "Welcome!");            
      break;
    case WStype_TEXT:
      DBG_SERIAL.printf("[%u] get Text: %s\n", num, payload);      
      webSocket.sendTXT(num, "Message received.");
      if (payload[0] == '>') {
        String tempStr;
        uint8_t i = 1;
        while(payload[i] != '<'){
          tempStr += (char)payload[i];
          i++;
        }
        if(relayMode == 0){
          T2 = smooth(tempStr.toFloat());
          DBG_SERIAL.println(T2);
        }
        else {
           T1 = smooth(tempStr.toFloat());
           DBG_SERIAL.println(T1);
        }
        
        DBG_SERIAL.printf("External water temperature: %s°C\n", tempStr.c_str());  
      }
      break;
    case WStype_BIN:
      DBG_SERIAL.printf("[%u] get binary length: %u\n", num, length);
      hexdump(payload, length);
      break;
  }
}



