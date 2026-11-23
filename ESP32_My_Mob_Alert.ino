/*********
   
   object : autonomous system to send a MOB alert on board instruments (not on VHF or AIS transmitter/recepteur)
            Nmea0183 and/or Nmea2000
  
   target : ESP32-Wroom-32 Dev Module     
            ESP32S3 N16R8 https://github.com/microrobotics/ESP32-S3-N16R8/blob/main/ESP32-S3-N16R8_User_Guide.pdf
              
            temporary push-button to reset config (optional)
            temporary push-button to send MOB alert
            active buzzer (or speaker or ...) (optional)
            voltage regulator LM2596D
            
            CAN transceiver SN65HVD230 3.3V
            or
            CAN transceiver TJA1050 5.0V + TXS0108E Logic Level Converter (OE also connected thru 1k ohm to 3.3v)
            or 
            MCP2515 Shield Can-Bus 5.0V + TXS0108E Logic Level Converter (OE also connected thru 1k ohm to 3.3v)
            
            for volts 3.3 volts and intensity 20mA:
            red led        resistance 85 ohm    // 100 ohm for the 2 led is enough, connected on GND
            green led      resistance 60 ohm
            (yellow led    resistance 60 ohm)
            (blue led      resistance  0 ohm)
            (white led     resistance 15 ohm)
            (red led HL    resistance 65 ohm)
            (green led HL  resistance  0 ohm)
            (yellow led HL resistance 65 ohm)
            (blue led HL   resistance  0 ohm)
                        
            ESP32 library 3.3.12

      Partition Scheme : Minimal SPIFFS(1.9MB APP with OTA/190KB SPIFFS)
         don't forget to mount SPIFFS
         warning if ESP32 library is upgraded, must be checked
         on ESP32S3 N16R8 FlashSize is 16MB, disable PSRAM or enable OPI PSRAM. Sometimes, Flash must be erased before download
      
   author : Patrick Dubois
   licence: public domain

   GNSS sentences
   input  TTL     :  OK, GNSS chipset on TTL
   input  RS232   :  OK, tried with a Python program UDP to Serial, ESP32 Serial to UDP (OpenCPN on other PC)
   input  RS422   :  OK, tried with a Python program UDP to Serial, ESP32 Serial to UDP (OpenCPN on other PC)
   input  UDP     :  OK
   input  N2K     :  OK, tried with another ESP32 sending GNSS sentences
 
   output USB     :  OpenCPN OK, qtVLM OK
   output RS232   :  OpenCPN OK, qtVLM OK
   output RS422   :  OpenCPN OK, qtVLM OK
   output UDP     :  OpenCPN OK, qtVLM OK
   output N2K     :  seems OK, tried by sending PGN to a Python Nmea2000 decoder. No Nmea2000 instrument available.
   
   TODO           : Add AIS SART Message Type 14 Option and Text for NMEA0183 and N2K (PGN129802)
                     check this in BuildType1 :   b.AddUInt(0, 2);                       // Repeat Indicator
   
*********/

String sVersion_number = "version 26.11", sVersion = __DATE__;

// Import required libraries
// if the library is global   use <>
// if the library is local    use ""
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_mac.h>                      // standard, included
#include <esp_system.h>                   // standard, included
#include <esp_task_wdt.h>                 // standard, included
#include <rom/ets_sys.h>                  // standard, included

#include <ArduinoJson.h>                  // https://arduinojson.org/
#include <AsyncTCP.h>                     // Async TCP by ESP32Async
#include <ElegantOTA.h>                   // ElagantOTA by Ayush Sharma warning if library is updated https://docs.elegantota.pro/getting-started/async-mode
#include <ESPAsyncWebServer.h>            // ESP Async WebServer by ESP32Async
#include <ESPmDNS.h>                      // standard, included
#include <SPIFFS.h>                       // standard, included
#include <time.h>                         // standard, included
#include <WiFi.h>                         // standard, included

#include <NMEA0183.h>                     // https://github.com/ttlappalainen/NMEA0183
#include <NMEA0183Msg.h>                  // https://github.com/ttlappalainen/NMEA0183
#include <NMEA0183Messages.h>             // https://github.com/ttlappalainen/NMEA0183

#define button0Pin            0           // GPIO 0 or PRG or Boot button

#define LAT                   0
#define LNG                   1
#define DD_position_format    1           // decimal degrees
#define DMD_position_format   2           // degrees minutes decimals
#define DMS_position_format   3           // degrees minutes seconds

#define wSettings_FILE "MyMobAlertSettings.json"    // SPIFFS files are case-sensitive
#define CONFIGRESET           0
#define CONFIGREAD            1
#define CONFIGWRITE           2

#define MOB_TRIGGER           1950        // triggers the AIS-SART and/or MOB after the button has been pressed x milliseconds

#define WDT_TIMEOUT           15          // watchdog matériel. restart the ESP32 if the main loop is locked more than 15 seconds
//#define WIFI_RECONNECT 15               // in STA mode, try to reconnect every 15s if WiFi disconnected

//uncomment this line if you want the buzzer
//#define USE_BUZZER                        // https://garrysblog.com/2022/12/16/experimenting-with-audio-tones-using-the-esp32-for-use-in-projects/
#if defined USE_BUZZER
   #define LEDCPin            13          // Define output pin for speaker. To change on ESP32S3
   #define LEDCResolution     10          // Set resolution to 10 bits
   #define TaskCore0           0
   #define TaskCore1           1
   TaskHandle_t BuzzerTaskHandle = NULL;
#endif

//uncomment this line if you want NMEA2000
#define NMEA2K_CODE
#if (defined(ARDUINO_ESP32_DEV) || defined(ARDUINO_UPESY_WROOM))
      #define rx0GPIO               3     // pin 34
      #define tx0GPIO               1     // pin 35
      #define rx1GPIO               19    // pin 31 ,as native rx1GPIO  9 is used to flash
      #define tx1GPIO               18    // pin 30 ,as native tx1GPIO 10 is used to flash
      #define rx2GPIO               16    // pin 27
      #define tx2GPIO               17    // pin 28
      #define buttonMOB             27    // (pin 11)
      #define BuiltInLed            2     // GPIO led ON on some ESP32 after WiFi on
      // these 2 leds are blinking if a MOB alert has been set
      #define P_ONLed               25    // (pin  9) red light, system is ON
      #define GNSSLed               26    // (pin 10) green led, GNNS is valid
      #if defined NMEA2K_CODE 
         // https://github.com/ttlappalainen/NMEA2000
         // https://github.com/ttlappalainen/NMEA2000_esp32
         // GPIO definitions must be placed before
         #define ESP32_CAN_TX_PIN GPIO_NUM_23   // default is GPIO_NUM_16
         #define ESP32_CAN_RX_PIN GPIO_NUM_22   // default is GPIO_NUM_4
      #endif
   #elif defined(ARDUINO_ESP32S3_DEV)
      #define rx0GPIO               44
      #define tx0GPIO               43
      #define rx1GPIO               15
      #define tx1GPIO               16
      #define rx2GPIO               19
      #define tx2GPIO               20
      #define buttonMOB             6
      #define BuiltInLed            38    // GPIO led ON on some ESP32 after WiFi on
      #define RGBLed                48    // RGB Led on this ESP32S3 N16R8
      // these 2 leds are blinking if a MOB alert has been set
      #define P_ONLed               4     // red light, system is ON
      #define GNSSLed               5     // green led, GNNS is valid
      #if defined NMEA2K_CODE
         // https://github.com/ktand/NMEA2000_esp32_twai
         // GPIO definitions must be placed before
         #define ESP32_CAN_TX_PIN GPIO_NUM_9    // default is GPIO_NUM_16
         #define ESP32_CAN_RX_PIN GPIO_NUM_10   // default is GPIO_NUM_4
      #endif
   #elif defined(ARDUINO_ESP32C3_DEV)
      #include <HardwareSerial.h>
      #define rx0GPIO               20
      #define tx0GPIO               21
      #define rx1GPIO               8
      #define tx1GPIO               9
      #define rx2GPIO               18
      #define tx2GPIO               19
      HardwareSerial Serial2(2);
      #define buttonMOB             10
      //#define BuiltInLed            2     // GPIO led ON on some ESP32 after WiFi on
      #define RGBLed                8     // RGB Led on this ESP32S3 N16R8
      // these 2 leds are blinking if a MOB alert has been set
      #define P_ONLed               6     // red light, system is ON
      #define GNSSLed               7     // green led, GNNS is valid
      #if defined NMEA2K_CODE
         // https://github.com/ktand/NMEA2000_esp32_twai
         // GPIO definitions must be placed before
         #define ESP32_CAN_RX_PIN GPIO_NUM_4    // default is GPIO_NUM_4
         #define ESP32_CAN_TX_PIN GPIO_NUM_5    // default is GPIO_NUM_16
      #endif
  #endif

#if defined NMEA2K_CODE 
   #include <NMEA2000_CAN.h>              // Automatically select the CAN driver
   #include <N2kMessages.h>
   // Optionnel mais recommandé : ne faire suivre au handler que les PGN utiles
   const unsigned long MOBTransmitPGN[]  PROGMEM = {127233L,129038L,129802L,0};           // 0 = fin de liste
   const unsigned long GNSSReceivedPGN[] PROGMEM = {129025L,129026L,129029L,129033L,0};   // 0 = fin de liste
   #define DEV_MOB   0
   #define DEV_GNSS  1
#endif

#define MOBUSBSerial          Serial      // rx0GPIO tx0GPIO or USB
#define GNSSSerial            Serial1     // rx1GPIO tx1GPIO
#define MOBRSxxxSerial        Serial2     // rx2GPIO tx2GPIO

struct GNSS_Coordinates {
	double GNSSTime;                       // RMC & GGA
	double Latitude;                       // RMC & GGA
	double Longitude;                      // RMC & GGA
   double trueCOG;                        // RMC
	double SOG;                            // RMC
	unsigned long daysSince1970;           // RMC
	double MagneticVariation;              // RMC
	int GPSQualityIndicator;               // GGA
	int satelliteCount;                    // GGA
	double HDOP;                           // GGA
	double Altitude;                       // GGA
	double geoidalSeparation;              // GGA
	double DGPSAge;                        // GGA
	int DGPSReferenceStationID;            // GGA
   uint32_t LastFix = -9999;
};
GNSS_Coordinates LastGNSS_data;

IPAddress STAlocalIP, STAsubnetMaskIP, STAgatewayIP, STAbroadcastIP,
         APlocalIP, APsubnetMaskIP, APgatewayIP, APbroadcastIP, APdhcp_startIP,
         localIP, subnetMaskIP, gatewayIP;
         
bool bAPSTAmode, bSTAmode, bAPmode, bDHCP, bStatic;
String APSTAHostName;
uint8_t nbAPclients;

// it can be good to use unassigned ports for uiG_Port & uiM_Port, check at https://www.iana.org/assignments/service-names-port-numbers
bool bG_Serial, bG_Udp, bG_N2K;
uint16_t uiG_Port, uiG_Pgn;
uint32_t baudGNSSSerial, uiG_TimeOut;

bool bAIS_SART_t1, bAIS_SART_t14, bWPL_MOB;
uint8_t uiR_Message;
uint32_t uiMMSI_number;

bool bM_SerialU, bM_Serial, bM_Udp, bM_N2K;
uint16_t uiM_Port;
uint32_t baudMOBUSB = 38400, baudMOBSerial;

bool bDisplay = false;
volatile bool bButtonResetPressed = false;
volatile uint32_t oldMOBMillis;

bool bbuttonMOBLastState = LOW;
uint32_t buttonMOBPressedTime = 0;

// we receive GNNS data on GNSS_UDP if enabled, and we send MOB alerts on STA_UDP and/or AP_UDP if enabled
WiFiUDP GNSS_UDP, STA_UDP, AP_UDP;        // WiFiUDP is a typedef of NetworkUDP

tNMEA0183 NMEA0183;

// to use json to load and save configuration file
JsonDocument jConfig;
String jsonReplyString;

// Init HTTP server
#define Web_Server_Port       80
AsyncWebServer server(Web_Server_Port);

// function prototype declarations
// Nmea0183
void AddChecksum(char *msg);              // checksum computation added to Nmea0183 sentences
void BuildMOB(char *stringAIS_SART_t1, size_t sizeofAIS_SART_t1, char *stringAIS_SART_t14, size_t sizeofAIS_SART_t14, char *stringWPL_MOB, size_t sizeofWPL_MOB);
void BuildType1Payload(char *outPayload, size_t outPayloadSize, uint32_t MMSI_number, double latitude, double longitude, bool cogSogValid, double cog, double sog);
void BuildType14Payload(char *outPayload, size_t outPayloadSize, uint32_t MMSI_number, const char *safetyMessage);
void Handle_GNSS_NMEA0183Msg(const tNMEA0183Msg &N0183Msg);
void TransmitNmea0183(char *nmea0183string);
// Nmea2000
#if defined NMEA2K_CODE 
   void NMEA2K_Init(void);
   void Handle_GNSS_NMEA2000Msg(const tN2kMsg &N2kMsg);
   void sendN2kAIS_SART_129038();         // Man Overboard Notification(AIS SART) Type  1 Position Report Class A Navigation Status 14
   void sendN2kAIS_SART_129802();         // Man Overboard Notification(AIS SART) Type 14 Safety-Related Broadcast Message
   void SendN2kWPL_MOB_127233();          // Man Overboard Notification(WPL MOB)
#endif
// All
bool isButtonMOBPressed(void);
String DD_to_DMD_v2(float fValue);                                               // decimal degrees to degrees minutes decimals
String DD_to_DMS(float fValue);                                                  // decimal degrees to degree minutes secondes
String *DecimalDegreesFormat_v2(byte position_format, float fLat, float fLng);   // conversion de format
void DisplayIMessage(String sMessage, bool bClear = false, bool bCRLN = false);  // light version, as no display
String HH_MM_SS(unsigned long elapsed);                                          // conversion from seconds to hh:mm:ss
void Storage_Init(void);                                                         // SPIFFS partition mounting
void WiFi_Init(void);                                                            // as the name :-)
void WSettingsRW(JsonDocument& jnewConfig, byte configValue);                    // reset, load and store json config file
String YYYY_MM_DD(unsigned long daysSince1970);
// end declarations

void IRAM_ATTR onButtonResetEvent() {
   bButtonResetPressed = true;
}

bool isButtonMOBPressed() {
   static bool isPressing, isLongDetected, bCurrentState;
   
   isLongDetected = false;
   bCurrentState = digitalRead(buttonMOB);
   if(bbuttonMOBLastState == HIGH && bCurrentState == LOW) {                // button is pressed
      buttonMOBPressedTime = millis();
      isPressing = true;
      isLongDetected = false;
   }
   else 
      //isPressing = (bbuttonMOBLastState == LOW && bCurrentState == HIGH);   // button must be released to detect time
      isPressing = (bbuttonMOBLastState == LOW && bCurrentState == LOW);    // time is detected even if button is not released
   if (isPressing && !isLongDetected) {
      isLongDetected = (millis() - buttonMOBPressedTime) > MOB_TRIGGER;
   }
   bbuttonMOBLastState = bCurrentState;
   return isLongDetected;
}

void convertFromJson(JsonVariantConst source, IPAddress& dest) {
  dest.fromString(source.as<const char*>());
}

void setup() {

   btStop();
   MOBUSBSerial.begin(baudMOBUSB);        // Serial USB, always started at startup, then following settings
   DisplayIMessage("\nMy MOB Alert");
   DisplayIMessage(sVersion_number + " compiled on " + sVersion);
   
   pinMode(button0Pin, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(button0Pin), onButtonResetEvent, RISING);
   pinMode(buttonMOB, INPUT_PULLUP);
   #if defined LEDCPin
      pinMode(LEDCPin, OUTPUT);
   #endif
   #if defined BuiltInLed
      pinMode(BuiltInLed , OUTPUT);
   #endif
   pinMode(P_ONLed, OUTPUT);
   digitalWrite(P_ONLed, HIGH);           // ON after Power ON
   pinMode(GNSSLed, OUTPUT);

   Storage_Init();
   WSettingsRW(jConfig, CONFIGREAD);
   WiFi_Init();
   #if defined BuiltInLed
      digitalWrite(BuiltInLed, LOW);     // ON after Wifi connection so set it OFF
   #endif
   // manage web pages and load/save configuration
   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 
      request->send(SPIFFS, "/config.html", "text/html");
   });
   // save parameters from memory to JSON
   server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (request->url() == "/saveConfig") {
         JsonDocument jnewConfig;
         DeserializationError error [[maybe_unused]] = deserializeJson(jnewConfig, data, len);
         WSettingsRW(jnewConfig, CONFIGWRITE);
         //request->send(200, "text/plain", "Config reçue");
      }
   });   
   // load parameters from JSON in memory
   server.on("/loadConfig", HTTP_GET, [](AsyncWebServerRequest *request) {
      String jsonString;
      serializeJson(jConfig, jsonString); // Convert JSON to string
      //Serial.println(jsonString);
      request->send(200, "application/json", jsonString);  // send JSON
   });    

   // Start HTTP server and OTA update
   ElegantOTA.begin(&server);
   server.begin();
   
   // Watchdog armé
   esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,                  // Convertin ms
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,   // Bitmask of all cores, https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/task_watchdog/main/task_watchdog_example_main.c
      .trigger_panic = true                              // Enable panic to restart ESP32
   };
   // WDT Init
   esp_err_t ESP32_ERROR [[maybe_unused]] = esp_task_wdt_init(&wdt_config);
   //Serial.println("Last Watchdog reset: " + String(esp_err_to_name(ESP32_ERROR)));
   esp_task_wdt_add(NULL);  //add current thread to WDT watch

   if (bG_Serial) {
      GNSSSerial.begin(baudGNSSSerial, SERIAL_8N1, rx1GPIO, tx1GPIO);
      NMEA0183.SetMessageStream(&GNSSSerial);                              // Serial input
   }
   if (bG_Udp) {
      GNSS_UDP.begin(uiG_Port);
      NMEA0183.SetMessageStream(&GNSS_UDP);                                // UDP input
   }
   if (bG_Serial || bG_Udp) {
      NMEA0183.SetMsgHandler(Handle_GNSS_NMEA0183Msg);                     // Manage Nmea0183 GNSS sentences
      NMEA0183.Open();
   }
   if (bM_SerialU) {
      MOBUSBSerial.begin(baudMOBUSB, SERIAL_8N1, rx0GPIO, tx0GPIO);        // serial output USB
   } 
   if (bM_Serial) {
      MOBRSxxxSerial.begin(baudMOBSerial, SERIAL_8N1, rx2GPIO, tx2GPIO);   // serial output rs232 or rs422
   }  
   // if (bM_Udp) {};   // nothing to do here, started in WiFi_Init()        // UDP output
   #if defined NMEA2K_CODE
   if (bG_N2K || bM_N2K) {                                                 // start GNNS by NM2K and NM2K output
      NMEA2K_Init();
      NMEA2000.SetMsgHandler(Handle_GNSS_NMEA2000Msg);                     // Manage Nmea2000 GNSS sentences
   }
   #endif
   
   #if defined USE_BUZZER
      // Buzzer in an independant task
      // https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/system/freertos.html
      // The stack size can be computed in the task function and adjusted
      // xTaskCreate(TriggerBuzzer, "TriggerBuzzer", 1768, NULL, 5, &BuzzerTaskHandle);
      xTaskCreatePinnedToCore(TriggerBuzzer, "TriggerBuzzer", 1768, NULL, 5, &BuzzerTaskHandle, tskNO_AFFINITY);     // any core
      // comment the 2 following lines if a test is not needed at startup
      ledcAttach(LEDCPin, 50, LEDCResolution);           // Attach pin before starting tone
      vTaskResume(BuzzerTaskHandle);                     // Buzzer is an independant task
   #endif
}

void loop() {
   static uint8_t nCount;
   static bool bButtonMOBPressed, bGNSS_StillValid;
   char stringAIS_SART_t1[128] = {0};
   char stringAIS_SART_t14[128] = {0};
   char stringWPL_MOB[128] = {0};
   
   esp_task_wdt_reset();
   delay(1);  // needed
   
   if (bButtonResetPressed) {
      WSettingsRW(jConfig, CONFIGRESET);
   }
   if (bG_Udp) { 
      GNSS_UDP.parsePacket();                               // analyse les trames Nmea0183 GNSS by UDP
   }
   if (bG_Serial || bG_Udp) { 
      NMEA0183.ParseMessages();                             // déclenche Handle_GNSS_NMEA0183Msg
   }
   #if defined NMEA2K_CODE
   if (bG_N2K) {
      NMEA2000.ParseMessages();                             // recoit les trames CAN GNSS et déclenche Handle_GNSS_NMEA2000Msg
   }
   #endif

   bGNSS_StillValid = millis() < (LastGNSS_data.LastFix + uiG_TimeOut);    // true si un fix RMC ou GGA valide a été recu
   digitalWrite(GNSSLed, bGNSS_StillValid);                 // ON if GNSS position is valid
   bButtonMOBPressed = isButtonMOBPressed();
   if (bGNSS_StillValid && bButtonMOBPressed) {
      #if defined USE_BUZZER
         ledcAttach(LEDCPin, 50, LEDCResolution);           // Attach pin before starting tone
         vTaskResume(BuzzerTaskHandle);                     // Buzzer is an independant task
      #endif
      if (bM_SerialU || bM_Serial || bM_Udp) {              // Serial and UDP are only Nmea0183
         BuildMOB(stringAIS_SART_t1, sizeof(stringAIS_SART_t1), 
                     stringAIS_SART_t14, sizeof(stringAIS_SART_t14), 
                     stringWPL_MOB, sizeof(stringWPL_MOB));       // build Nmea0183 AIS_SART and WPL_MOB sentences
      }
      nCount = 0;
      do {                                                  // LED blinking & Transmit sentences
         if (bM_SerialU || bM_Serial || bM_Udp) {           // Serial and UDP are only Nmea0183
            if (bAIS_SART_t1) {                             // Man Overboard Notification(AIS-SART) Type  1 Position Report Class A Navigation Status 14
               TransmitNmea0183(stringAIS_SART_t1);         
            }
            if (bAIS_SART_t14) {                            // Man Overboard Notification(AIS SART) Type 14 Safety-Related Broadcast Message
               TransmitNmea0183(stringAIS_SART_t14);
            }
            if (bWPL_MOB) {                                 // Man Overboard Notification(WPL MOB)
               TransmitNmea0183(stringWPL_MOB);    
            }   
         }
         #if defined NMEA2K_CODE
         if (bAIS_SART_t1) { 
            sendN2kAIS_SART_129038();                       // Man Overboard Notification(AIS SART) Type  1 Position Report Class A Navigation Status 14
         }
         if (bAIS_SART_t14) { 
            sendN2kAIS_SART_129802();                       // Man Overboard Notification(AIS SART) Type 14 Safety-Related Broadcast Message
         }
         if (bWPL_MOB) {                                    // Man Overboard Notification(WPL MOB)
            SendN2kWPL_MOB_127233();             
         }
         NMEA2000.ParseMessages();
         #endif
         digitalWrite(P_ONLed, LOW); digitalWrite(GNSSLed, LOW);
         delay(500);
         digitalWrite(P_ONLed, HIGH); digitalWrite(GNSSLed, HIGH);
         delay(500);
      }
      while (++nCount < uiR_Message);
   }   
   ElegantOTA.loop();
}

#if defined USE_BUZZER
void TriggerBuzzer(void *pvParameters) {              // Buzzer is an independant task
   int nCount = 0;

   while (true) {
      ledcWriteTone(LEDCPin, 435);                    // 435Hz Pompier for 0.5 second
      vTaskDelay(pdMS_TO_TICKS(500));
      ledcWriteTone(LEDCPin, 488);                    // 488Hz Pompier for 0.5 second
      vTaskDelay(pdMS_TO_TICKS(500));
      if (++nCount == 5) {
         ledcDetach(LEDCPin);                         // noSound()
         vTaskSuspend(NULL);
         nCount = 0;
      }
      //UBaseType_t marge_octets = uxTaskGetStackHighWaterMark(NULL);
      //printf("Marge minimale de pile : %u octets\n", (unsigned)marge_octets);
   }
}
#endif

String DD_to_DMD_v2(float fValue) {
   static uint16_t deg;
   static float minutesRemainder;
   static char buffer[21] = {0};

   fValue = abs(fValue);
   deg = fValue;
   minutesRemainder = abs(fValue - deg) * 60;
   snprintf(buffer,sizeof(buffer), "%03d%0.5f", deg, minutesRemainder);
   return String(buffer);
}

String DD_to_DMS(float fValue) {
   static uint16_t deg, arcMinutes;
   static float minutesRemainder, arcSeconds;
   static char buffer[31] = {0};

   fValue = abs(fValue);
   deg = fValue;
   minutesRemainder = abs(fValue - deg) * 60;
   arcMinutes = minutesRemainder;
   arcSeconds = (minutesRemainder - arcMinutes) * 60;
   snprintf(buffer,sizeof(buffer)," %03d%c%02d'%05.3f\"", deg, byte(223), arcMinutes, arcSeconds);
   return String(buffer);
}

String *DecimalDegreesFormat_v2(byte position_format, float fLat, float fLng) {
   /*    Latitude +N   0-90
         Latitude -S
         Longitude +E  0-180
         Longitude -W         */
   static String sLat, sLng;
   static String sDD_to_OF[2];                  // array of String to return
   static char buffer[17] = {0};
   
   // fLat = -fLat ; fLng = -fLng;              // to test South and West :-)
   switch (position_format) {
      case DD_position_format :  { 
         snprintf(buffer,sizeof(buffer), "%+012.7f    ", fLat);     // https://cplusplus.com/reference/cstdio/printf/
         sLat = String(buffer);
         snprintf(buffer,sizeof(buffer),"%+012.7f    ", fLng);
         sLng = String(buffer);
         break;
      }
      case DMD_position_format : { 
         sLat = DD_to_DMD_v2(fLat);
         sLat.concat(fLat < 0 ? ",S" : ",N");
         sLng = DD_to_DMD_v2(fLng);
         sLng.concat(fLng < 0 ? ",W" : ",E");
         break;
      }
      case DMS_position_format : { 
         sLat = DD_to_DMS(fLat);
         sLat.concat(fLat < 0 ? ",S" : ",N");
         sLng = DD_to_DMS(fLng);
         sLng.concat(fLng < 0 ? ",W" : ",E");
         break;
      }
   }
   sDD_to_OF[LAT] = sLat;
   sDD_to_OF[LNG] = sLng;
   return sDD_to_OF;
}

void WSettingsRW(JsonDocument& jnewConfig, byte configValue) {
   
   if (!SPIFFS.exists("/" wSettings_FILE)) { 
      // Serial.println(String(wSettings_FILE) + " doesn't exist");
      configValue = CONFIGRESET;
   }
   
   switch (configValue) {
      case CONFIGRESET : {             // 0
         File file = SPIFFS.open("/" wSettings_FILE, FILE_WRITE);
         jnewConfig["APSTAHostName"] = "My_MOB_Alert";
         jnewConfig["APmode"] = true;
         jnewConfig["APClients"] = 4;
         jnewConfig["APPassword"] = "12345678";
         jnewConfig["APlocalIP"] = "192.168.4.1";
         jnewConfig["APsubnetMaskIP"] = "255.255.255.0";
         jnewConfig["APgatewayIP"] = "192.168.4.99"; 
         jnewConfig["APdhcp_startIP"] = "192.168.4.11";
         jnewConfig["STAmode"] = false;
         jnewConfig["DHCP"] = true;
         jnewConfig["Static"] = false;
         jnewConfig["STASSID"] = "SSID_to_change";
         jnewConfig["STAPassword"] = "Password_to_change";
         jnewConfig["STAlocalIP"] = "192.168.1.64";
         jnewConfig["STAsubnetMaskIP"] = "255.255.255.0";
         jnewConfig["STAgatewayIP"] = "192.168.1.254";
         jnewConfig["G_Serial"] = true;
         jnewConfig["G_SerialB"] = 38400;
         jnewConfig["G_Udp"] = false;
         jnewConfig["G_Port"] = 2010;
         jnewConfig["G_N2K"] = false;
         jnewConfig["G_Pgn"] = 129029;
         jnewConfig["G_TimeOut"] = 15;
         jnewConfig["MMSI_number"]= 972990001;
         jnewConfig["AIS_SART_t1"] = false;
         jnewConfig["AIS_SART_t14"] = false;
         jnewConfig["WPL_MOB"] = true;
         jnewConfig["R_Message"] = 1;
         jnewConfig["M_SerialU"] = false;
         jnewConfig["M_SerialUB"] = 38400;
         jnewConfig["M_Serial"] = true;
         jnewConfig["M_SerialB"] = 38400;
         jnewConfig["M_Udp"] = false;
         jnewConfig["M_Port"] = 10110;
         jnewConfig["M_N2K"] = false;
 
         serializeJsonPretty(jnewConfig, file);       // indent 2 spaces
         file.close();
         ESP.restart();
         break;
      }
      case CONFIGREAD : {              // 1
         File file = SPIFFS.open("/" wSettings_FILE, FILE_READ);
         DeserializationError error [[maybe_unused]] = deserializeJson(jnewConfig, file);
         //Serial.println(jnewConfig.as<String>());
         APSTAHostName = jConfig["APSTAHostName"].as<String>();
         bAPmode = jConfig["APmode"];
         nbAPclients = jConfig["APClients"];
         bSTAmode = jConfig["STAmode"]; 
         bAPSTAmode = bAPmode && bSTAmode;
         bDHCP = jConfig["DHCP"];
         bStatic = jConfig["Static"];
         convertFromJson(jConfig["APlocalIP"], APlocalIP);
         convertFromJson(jConfig["APsubnetMaskIP"], APsubnetMaskIP);
         convertFromJson(jConfig["APgatewayIP"], APgatewayIP);
         convertFromJson(jConfig["APdhcp_startIP"], APdhcp_startIP);
         convertFromJson(jConfig["STAlocalIP"], STAlocalIP);
         convertFromJson(jConfig["STAsubnetMaskIP"], STAsubnetMaskIP);
         convertFromJson(jConfig["STAgatewayIP"], STAgatewayIP);

         bG_Serial = jConfig["G_Serial"];
         baudGNSSSerial = jConfig["G_SerialB"];
         bG_Udp = jConfig["G_Udp"];
         uiG_Port = jConfig["G_Port"];
         bG_N2K = jConfig["G_N2K"];
         uiG_Pgn = jConfig["G_Pgn"];
         uiG_TimeOut = jConfig["G_TimeOut"];
         uiG_TimeOut *= 1000;
         uiMMSI_number = jConfig["MMSI_number"];
         bAIS_SART_t1 = jConfig["AIS_SART_t1"];
         bAIS_SART_t14 = jConfig["AIS_SART_t14"];
         bWPL_MOB = jConfig["WPL_MOB"];
         uiR_Message = jConfig["R_Message"];

         bM_SerialU = jConfig["M_SerialU"];
         baudMOBUSB = jConfig["M_SerialUB"];
         bM_Serial = jConfig["M_Serial"];
         baudMOBSerial = jConfig["M_SerialB"];
         bM_Udp = jConfig["M_Udp"];
         uiM_Port = jConfig["M_Port"];
         bM_N2K = jConfig["M_N2K"];
         file.close();
         break;
      }
      case CONFIGWRITE : {             // 2
         File file = SPIFFS.open("/" wSettings_FILE, FILE_WRITE);
         // Serial.println(jnewConfig.as<String>());
         file.print(jnewConfig.as<String>());
         file.close();
         ESP.restart();
         break;
      }
   }
}

void Storage_Init() {

   if(!SPIFFS.begin(true))
      DisplayIMessage("Storage Mount Failed", false, true);
   else {
      DisplayIMessage("Storage Mount OK", false, true);
   }
   delay(1000);
}

void WiFi_Init() {
   uint32_t oldMillis = millis();
   String Global_SSID_NAME, Global_PASSWORD;

   if (bAPSTAmode) { WiFi.mode(WIFI_AP_STA);}
   else if (bSTAmode && !bAPmode) { WiFi.mode(WIFI_STA);}
   else if (bAPmode && !bSTAmode ) { WiFi.mode(WIFI_AP);}
   WiFi.setHostname(APSTAHostName.c_str());
   
   if (bSTAmode) {
      if (bStatic) {
         WiFi.config(STAlocalIP, STAgatewayIP, STAsubnetMaskIP); // primaryDNS, secondaryDNS))
      }
      Global_SSID_NAME = jConfig["STASSID"].as<String>(); 
      Global_PASSWORD = jConfig["STAPassword"].as<String>();
      WiFi.setAutoReconnect(true);
      WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);
      WiFi.begin(Global_SSID_NAME, Global_PASSWORD, 0, NULL, true);
      DisplayIMessage("Connect " + String(Global_SSID_NAME));
      do {
         delay(500);
         Serial.print(".");
      }
      while ((WiFi.status() != WL_CONNECTED) && (millis() < oldMillis + 15000));     // 15 seconds
      jConfig["STAlocalIP"] = STAlocalIP = WiFi.localIP();
      jConfig["STAsubnetMaskIP"] = STAsubnetMaskIP = WiFi.subnetMask();
      jConfig["STAgatewayIP"] = STAgatewayIP = WiFi.gatewayIP();
      STAbroadcastIP = WiFi.broadcastIP();
      if (WiFi.status() != WL_CONNECTED) {
         DisplayIMessage("No STA connection");
         bAPmode = true;
         bSTAmode = false;
         WiFi.mode(WIFI_AP);
      }
      else {
         if (bM_Udp) {
            STA_UDP.begin(uiM_Port);
         }
      DisplayIMessage("\nIP " + STAlocalIP.toString());
      }
   }

   if (bAPmode) {
      // bool softAPConfig(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dhcp_lease_start = (uint32_t) 0);
      // bool softAP(const char* ssid, const char* passphrase = NULL, int channel = 1, int ssid_hidden = 0, int max_connection = 4, bool ftm_responder = false);
      Global_SSID_NAME = APSTAHostName.c_str();
      Global_PASSWORD = "12345678"; //jConfig["APPassword"].as<String>();
      DisplayIMessage("Starting Autonomous");
      delay(500);
      WiFi.softAPConfig(APlocalIP, APgatewayIP, APsubnetMaskIP, APdhcp_startIP);
      WiFi.softAP(Global_SSID_NAME, Global_PASSWORD, 1, 0, nbAPclients, false);
      APlocalIP = WiFi.softAPIP();
      APsubnetMaskIP = WiFi.softAPSubnetMask();
      APbroadcastIP = WiFi.softAPBroadcastIP();
      if (bM_Udp) {
         AP_UDP.begin(uiM_Port);
      }
      DisplayIMessage("IP " + APlocalIP.toString());
   }
   MDNS.begin(APSTAHostName.c_str());
}

String HH_MM_SS(unsigned long elapsed) {  // converts millisecondes to HH:MM:SS
   char cReturn[17] = {0};
   
   int HH = elapsed / 3600;
   int MM = elapsed % 3600 / 60;
   int SS = elapsed % 60;
   snprintf(cReturn, sizeof(cReturn), "%02d:%02d:%02d", HH, MM, SS);
   return cReturn;
}

String YYYY_MM_DD(unsigned long daysSince1970) {
  char cReturn[37] = {0};
  
  time_t epochDays = (time_t) daysSince1970 * 86400;
  struct tm dateInfo;
  gmtime_r(&epochDays, &dateInfo);
  snprintf(cReturn, sizeof(cReturn), "%04d-%02d-%02d", dateInfo.tm_year + 1900, dateInfo.tm_mon + 1, dateInfo.tm_mday);
  return cReturn;
}

void DisplayIMessage(String sMessage, bool bClear, bool bCRLN) {
   //static int nRow = 0;  / not needed in this simplified version

   if (bDisplay) {
      if (bClear) {
      //   display_.clear();
      //   display_.setCursor(0, nRow=0);
      }
      if (bCRLN) { 
      //   display_.setCursor(0, nRow++);
      }
      //display_.print(sMessage);
   } 
   else
      Serial.println(sMessage);
}

// Petit assembleur de trame AIS : accumule des bits MSB-first et les convertit par paquets de 6 bits en caracteres ASCII encapsulés, // selon le codage standard AIVDM (cf. ITU-R M.1371 / documentation gpsd AIVDM).
class tAISPayloadBuilder {
   public:
      void AddUInt(uint32_t value, uint8_t numBits) {
         for (int8_t i = numBits - 1; i >= 0; i--) {
            PushBit((value >> i) & 0x1);
         }
      }
      void AddInt(int32_t value, uint8_t numBits) {
         uint32_t mask = (numBits >= 32) ? 0xFFFFFFFFUL : ((1UL << numBits) - 1UL);
         AddUInt((uint32_t)value & mask, numBits);
      }
      const char *GetPayload() {
         Flush();
         payload[payloadLen] = '\0';
         return payload;
      }
   private:
      void PushBit(uint8_t bit) {
         acc = (acc << 1) | (bit & 0x1);
         accBits++;
         if (accBits == 6) EmitSixBits();
      }
      void Flush() {
         if (accBits > 0) {
            acc = acc << (6 - accBits); // bourrage a droite avec des 0
            accBits = 6;
            EmitSixBits();
         }
      }
      void EmitSixBits() {
         uint8_t v = acc & 0x3F;
         char c = (v < 40) ? (char)(v + 48) : (char)(v + 56);
         if (payloadLen < sizeof(payload) - 1) payload[payloadLen++] = c;
         acc = 0;
         accBits = 0;
      }
      uint8_t acc = 0;
      uint8_t accBits = 0;
      char payload[64] = {0}; // // type 1 = 28 caracteres, type 14 = jusqu'a ~60
      uint8_t payloadLen = 0;
};

void TransmitNmea0183(char *nmea0183string) {

   if (bM_SerialU) { MOBUSBSerial.println(nmea0183string); }
   if (bM_Serial) { MOBRSxxxSerial.println(nmea0183string); }
   if (bM_Udp) {
      if (bSTAmode) {
         STA_UDP.beginPacket(STAbroadcastIP, uiM_Port);  // broadcast_IP
         STA_UDP.print(nmea0183string);
         STA_UDP.endPacket();
      }
      if (bAPmode) {
         AP_UDP.beginPacket(APbroadcastIP, uiM_Port);  // broadcast_IP
         AP_UDP.print(nmea0183string);
         AP_UDP.endPacket();
      }
   }
}

// Handler appele par la librairie NMEA0183 pour chaque trame recue et
// reconnue. On traite ici les trames RMC (position/cap/vitesse/date) et GGA
void Handle_GNSS_NMEA0183Msg(const tNMEA0183Msg &N0183Msg) {
   // Dernieres valeurs GNSS recues (mises a jour depuis la trame RMC)
   double   gGNSSTime;                       // seconds since UTC midnight
   double   gLatitude;                       // decimal degrees
   double   gLongitude;                      // decimal degrees
   double   gAltitude;                       // Antenna Altitude above/bealow mean sea level, in meters
   double   gCOG;                            // COG true (cap fond), in radians
   double   gSOG;                            // Speed Over Ground, in m/s
   unsigned long gDaysSince1970 = 0;         // date, in days since 1970-01-01
   double   MagneticVariation;               // magnetic variation
   // Dernieres valeurs GNSS recues (mises a jour depuis la trame GGA, en plus de celles ci-dessus)
   int      gQualityIndicator;               // Quality indicator, 0 = not available, 1 = GPS fix
   int      gSatelliteCount;                 // Number of satellites in use (0-12)
   double   gHDOP;                           // Horizontal dilution
   //double   gPDOP;                            // Position Dilution of Precision. Obsolete  
   double   gGeoidalSeparation;              
   double   gDGPSAge;
   int      gDGPSReferenceStationID;
   
   if (N0183Msg.IsMessageCode("RMC")) {
      if (NMEA0183ParseRMC(N0183Msg, gGNSSTime, gLatitude, gLongitude, gCOG, gSOG, gDaysSince1970, MagneticVariation)) {
         LastGNSS_data.GNSSTime = gGNSSTime;
         LastGNSS_data.Latitude = gLatitude;
         LastGNSS_data.Longitude = gLongitude;
         LastGNSS_data.trueCOG = gCOG;
         LastGNSS_data.SOG = gSOG ;
         LastGNSS_data.daysSince1970 = gDaysSince1970;
         LastGNSS_data.MagneticVariation = MagneticVariation ;
         if (gLatitude + gLongitude > 0)     // not working if we are exactly at North Pole
            LastGNSS_data.LastFix = millis();
      } 
   }
   else if (N0183Msg.IsMessageCode("GGA")) {
      if (NMEA0183ParseGGA(N0183Msg, gGNSSTime, gLatitude, gLongitude, gQualityIndicator, gSatelliteCount, gHDOP, gAltitude, gGeoidalSeparation, gDGPSAge, gDGPSReferenceStationID )) {
         LastGNSS_data.GNSSTime = gGNSSTime;
         LastGNSS_data.Latitude = gLatitude;
         LastGNSS_data.Longitude = gLongitude;
         LastGNSS_data.GPSQualityIndicator = gQualityIndicator;
         LastGNSS_data.satelliteCount = gSatelliteCount;
         LastGNSS_data.HDOP = gHDOP;
         LastGNSS_data.Altitude = gAltitude;
         LastGNSS_data.geoidalSeparation = gGeoidalSeparation;
         LastGNSS_data.DGPSAge = gDGPSAge;
         LastGNSS_data.DGPSReferenceStationID = gDGPSReferenceStationID;
         if (gLatitude + gLongitude > 0)     // not working if we are exactly at North Pole
            LastGNSS_data.LastFix = millis();
      } 
   }
}

// Envoie l'alerte MOB en NMEA0183 : une trame WPL "MOB" + une trame AIVDM simulant une cible AIS de type MOB,
// avec la derniere position/cap/vitesse connus (recus via UDP).
// https://www.itu.int/dms_pubrec/itu-r/rec/m/R-REC-M.1371-6-202602-I!!PDF-E.pdf
// https://gpsd.gitlab.io/gpsd/AIVDM.html
void BuildMOB(char *stringAIS_SART_t1, size_t sizeofAIS_SART_t1, char *stringAIS_SART_t14, size_t sizeofAIS_SART_t14, char *stringWPL_MOB, size_t sizeofWPL_MOB) {
   String * sDD_to_OF;                    // array of String returned from called function
   
   if (bAIS_SART_t1) {                    // Trame AIS Position Report Class A Navigation Status 14
                                          // & Trame AIS Type 14 Safety-Related Broadcast Message
      char payload_t1[32] = {0};
      BuildType1Payload(payload_t1, sizeof(payload_t1), uiMMSI_number, LastGNSS_data.Latitude, LastGNSS_data.Longitude,
                           true, LastGNSS_data.trueCOG, LastGNSS_data.SOG);
      snprintf(stringAIS_SART_t1, sizeofAIS_SART_t1, "!AIVDM,1,1,,A,%s,0", payload_t1);
      AddChecksum(stringAIS_SART_t1);
      
      char payload_t14[64] = {0};
      BuildType14Payload(payload_t14, sizeof(payload_t14), uiMMSI_number, "MOB ACTIVE");
      snprintf(stringAIS_SART_t14, sizeofAIS_SART_t14, "!AIVDM,1,1,,A,%s,0", payload_t14);
      AddChecksum(stringAIS_SART_t14);
   }   
   if (bWPL_MOB) {                        // Trame WPL "MOB" 
      sDD_to_OF = DecimalDegreesFormat_v2(DMD_position_format, LastGNSS_data.Latitude, LastGNSS_data.Longitude);
      snprintf(stringWPL_MOB, sizeofWPL_MOB, "$GPWPL,%s,%s,%s", sDD_to_OF[LAT].c_str(), sDD_to_OF[LNG].c_str(), "MOB ACTIVE");
      AddChecksum(stringWPL_MOB);
   }
}

void AddChecksum(char *msg) {
  unsigned int i = 1;        // First character not included in checksum, excluding $ or !
  uint8_t tmp, chkSum = 0;
  char ascChkSum[5];       // 5 instead of 4

  while (msg[i] != '\0') {
    chkSum ^= msg[i++];
  }
  ascChkSum[0] = '*';
  ascChkSum[3] = '\r';     // added, else problem in qtVlm
  ascChkSum[4] = '\0';
  tmp = chkSum / 16;
  ascChkSum[1] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
  tmp = chkSum % 16;
  ascChkSum[2] = tmp > 9 ? 'A' + tmp-10 : '0' + tmp;
  strcat(msg, ascChkSum);
}

// Construit le contenu (payload arme sur 6 bits) d'un message AIS de type 1 "Position Report",
// avec NavigationalStatus = 14 ("AIS-SART / MOB-AIS / EPIRB-AIS actif"), à partir de la derniere position/cap/vitesse connus.
void BuildType1Payload(char *outPayload, size_t outPayloadSize, uint32_t MMSI_number, double latitude, double longitude, bool bcogSogValid, double cog, double sog) {
   tAISPayloadBuilder b;

   b.AddUInt(1, 6);                       // Message Type = 1 (Position Report Class A)
   b.AddUInt(3, 2);                       // Repeat Indicator  N2kaisr_Final
   b.AddUInt(MMSI_number, 30);            // MMSI
   b.AddUInt(14, 4);                      // Navigational status = 14 = AIS-SART/MOB-AIS/EPIRB-AIS actif
   b.AddInt(-128, 8);                     // Rate Of Turn = non disponible (valeur reservee -128)
   uint16_t sogTenthsKn = 1023;
   if (bcogSogValid) {
      sogTenthsKn = (uint16_t)constrain(round(sog * 1.9438444924406047516198704103672 * 10.0), 0, 1022);
   }
   b.AddUInt(sogTenthsKn, 10);            // SOG en 1/10 de noeud
   b.AddUInt(0, 1);                       // Position Accuracy (0 = by default)
   b.AddInt((int32_t)round(longitude * 600000.0), 28);   // position is valid, else this function is not called
   b.AddInt((int32_t)round(latitude  * 600000.0), 27);   // position is valid, else this function is not called
   uint16_t cogTenthsDeg = 3600;
   if (bcogSogValid) {
      double cogDeg = cog * 180.0 / M_PI;  // conversion radians -> degres
      cogTenthsDeg = (uint16_t)constrain(round(cogDeg * 10.0), 0, 3599);
   } 
   b.AddUInt(cogTenthsDeg, 12);           // COG en 1/10 de degre
   b.AddUInt(511, 9);                     // Cap vrai (Heading) non disponible
   b.AddUInt(60, 6);                      // Horodatage (secondes UTC) non disponible
   b.AddUInt(0, 2);                       // Indicateur de manoeuvre : non disponible
   b.AddUInt(0, 3);                       // Spare
   b.AddUInt(0, 1);                       // Drapeau RAIM
   b.AddUInt(0, 19);                      // Etat de communication (place-holder)
   const char *p = b.GetPayload();
   strncpy(outPayload, p, outPayloadSize - 1);
   outPayload[outPayloadSize - 1] = '\0';
}

void BuildType14Payload(char *outPayload, size_t outPayloadSize, uint32_t MMSI_number, const char *safetyMessage ) {
   tAISPayloadBuilder b;
   #define AIS_TYPE14_MAX_CHARS 53

   b.AddUInt(14, 6);                      // Message Type = 14 (Safety-Related Broadcast Message)
   b.AddUInt(3, 2);                       // Repeat Indicator  N2kaisr_Final
   b.AddUInt(MMSI_number, 30);            // MMSI
   b.AddUInt(0, 2);                       // Spare 

   size_t len = strlen(safetyMessage);
   if (len > AIS_TYPE14_MAX_CHARS) len = AIS_TYPE14_MAX_CHARS;

   for (size_t i = 0; i < len; i++) {
      char c = safetyMessage[i];
      if (c >= 'a' && c <= 'z') 
         c -= 32;          // le "6-bit ASCII" AIS n'a pas de minuscules

      uint8_t v;
      if (c >= '@' && c <= '_')      
         v = c - 64;  // '@'..'_' -> 0..31
      else if (c >= ' ' && c <= '?') 
         v = c;       // ' '..'?' -> 32..63
      else 
         v = 32;      // caractere non supporte -> espace
      b.AddUInt(v, 6);
   }
   const char *p = b.GetPayload();
   strncpy(outPayload, p, outPayloadSize - 1);
   outPayload[outPayloadSize - 1] = '\0';
}

#if defined NMEA2K_CODE
void NMEA2K_Init() {
   uint32_t NMEA2K_DeviceNumber;
   uint8_t chipid[6];
   
   // Generate unique number from chip id
   esp_efuse_mac_get_default(chipid);
   NMEA2K_DeviceNumber = chipid[0] + (chipid[1]<<8) + (chipid[2]<<16) + (chipid[3]<<24);
   //for (uint8_t i = 0; i < 6; i++) NMEA2K_DeviceNumber += (chipid[i] << (7 * i));
   //MOBUSBSerial.println(); MOBUSBSerial.println(NMEA2K_DeviceNumber);
   
   NMEA2000.SetDeviceCount(1);
   
   // Set product information
   NMEA2000.SetProductInformation("P-0102-D",                        // Manufacturer's Model serial code
                                    102,                             // Manufacturer's product code (101 GNSS, 102 AIS, 103 Sensors)
                                    "My MOB Alert",                  // Manufacturer's Model ID
                                    "SW-Vers:  2026-10-13",          // Manufacturer's Software version code
                                    "Mod-Vers: 2026-10-13",          // Manufacturer's Model version
                                    0xff,                            // load equivalency - use default
                                    0xffff,                          // NMEA 2000 version - use default
                                    0xff,                            // Certification level - use default
                                    DEV_MOB
                                 );
   // Set device information
   NMEA2000.SetDeviceInformation(NMEA2K_DeviceNumber,                // Unique number. Use e.g. Serial number.
                                    135,                             // Man Overboard
                                    //130,                             // Emergency Position Indicating Radio Beacon (EPIRB)
                                    20,                              // Safety Systems
                                    2046,                            // 
                                    4,                               // Marine
                                    DEV_MOB
                                 );
   
   // Set product information
   NMEA2000.SetProductInformation("P-0101-D",                        // Manufacturer's Model serial code
                                    101,                             // Manufacturer's product code (101 GNSS, 102 AIS, 103 Sensors)
                                    "GNSS",                          // Manufacturer's Model ID
                                    "SW-Vers : 2026-03-31",          // Manufacturer's Software version code
                                    "Mod-Vers: 2026-03-31",          // Manufacturer's Model version
                                    0xff,                            // load equivalency - use default
                                    0xffff,                          // NMEA 2000 version - use default
                                    0xff,                            // Certification level - use default
                                    DEV_GNSS
                                 );
   // Set device information
   NMEA2000.SetDeviceInformation(NMEA2K_DeviceNumber+1,                // Unique number. Use e.g. Serial number.
                                    145,                             // Devices Devices that report vessel Latitude, Longitude, ground speed, formerly GNSS
                                    60,                              // Navigation
                                    2046,                            // 
                                    4,                               // Marine
                                    DEV_GNSS
                                 );

   // Uncomment 2 rows below to see, what device will send to bus
   //NMEA2000.SetForwardStream(&Serial);
   //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Show in clear text.
   //If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
   NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 135);
   //NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 135);
   //NMEA2000.EnableForward(false); // false -> pas de retransmission des trames CAN brutes sur le port serie
   NMEA2000.ExtendTransmitMessages(MOBTransmitPGN, DEV_MOB);
   NMEA2000.ExtendReceiveMessages(GNSSReceivedPGN, DEV_GNSS);

   NMEA2000.Open();
}

// Handler appele par la librairie NMEA2000 pour CHAQUE message recu sur le bus CAN (les siens et ceux des autres appareils,
// car le device est en mode N2km_ListenAndNode). On ne traite ici que les PGN utiles au MOB.
void Handle_GNSS_NMEA2000Msg(const tN2kMsg &N2kMsg) {
   unsigned char SID;
   double   gGNSSTime;                       // seconds since UTC midnight
   double   gLatitude;                       // decimal degrees
   double   gLongitude;                      // decimal degrees
   double   gAltitude;                       // Antenna Altitude above/bealow mean sea level, in metters
   double   gCOG;                            // COG true (cap fond), in radians
   double   gSOG;                            // Speed Over Ground, in m/s
   uint16_t gDaysSince1970;                  // date, in days since 1970-01-01
   //double   gMagneticVariation;               // magnetic variation
   // Dernieres valeurs GNSS recues (mises a jour depuis la trame GGA, en plus de celles ci-dessus)
   //int      gQualityIndicator;               // Quality indicator, 0 = not available, 1 = GPS fix
   unsigned char gSatelliteCount;            // Number of satellites in use (0-12)
   double   gHDOP;                           // Horizontal dilution
   double   gPDOP;                           // Position Dilution of Precision. Obsolete
   double   gGeoidalSeparation;              
   double   gDGPSAge;
   uint16_t gDGPSReferenceStationID;

   switch (N2kMsg.PGN) {
      case 129029: {                            // GNSS Position Data
         tN2kGNSStype gGNSStype;
         tN2kGNSSmethod gGNSSmethod;
         tN2kGNSStype ReferenceStationType;
         unsigned char nReferenceStations;

         if (ParseN2kPGN129029(N2kMsg, SID, gDaysSince1970, gGNSSTime, gLatitude, gLongitude, gAltitude,
                                gGNSStype, gGNSSmethod, gSatelliteCount, gHDOP, gPDOP, gGeoidalSeparation, 
                                nReferenceStations, ReferenceStationType, gDGPSReferenceStationID, gDGPSAge)) {
            LastGNSS_data.GNSSTime = gGNSSTime;
            LastGNSS_data.Latitude = gLatitude;
            LastGNSS_data.Longitude = gLongitude;
            LastGNSS_data.Altitude = gAltitude;
            // gGNSStype N2kGNSSt_integrated, not affected
            LastGNSS_data.GPSQualityIndicator = gGNSSmethod;
            LastGNSS_data.satelliteCount = gSatelliteCount;
            LastGNSS_data.HDOP = gHDOP;
            LastGNSS_data.trueCOG = gCOG;
            LastGNSS_data.SOG = gSOG ;
            LastGNSS_data.daysSince1970 = gDaysSince1970;
            LastGNSS_data.geoidalSeparation = gGeoidalSeparation;
            // nReferenceStations   0, not affected
            // ReferenceStationType N2kGNSSt_integrated, not affected
            LastGNSS_data.DGPSReferenceStationID = gDGPSReferenceStationID;
            LastGNSS_data.DGPSAge = gDGPSAge;
            if (gLatitude + gLongitude > 0)     // not working if we are exactly at North Pole
               LastGNSS_data.LastFix = millis();
         }
         break;
      }
      case 129025:                              // Position, Rapid Update
         if (ParseN2kPGN129025(N2kMsg, gLatitude, gLongitude)) {
            LastGNSS_data.Latitude = gLatitude;
            LastGNSS_data.Longitude = gLongitude;
            if (gLatitude + gLongitude > 0)     // not working if we are exactly at North Pole
               LastGNSS_data.LastFix = millis();
         }
         break;
      case 129026:                              // COG & SOG, Rapid Update
         tN2kHeadingReference N2khr;

         if (ParseN2kPGN129026(N2kMsg, SID, N2khr, gCOG, gSOG)) {
            LastGNSS_data.trueCOG = gCOG;
            LastGNSS_data.SOG = gSOG ;
            if (gCOG + gSOG > 0)
               LastGNSS_data.LastFix = millis();
         }
         break;
      case 129033:                           // Local Time Offset
         break;
      default: break;
     }
}

void SendN2kWPL_MOB_127233() {
   tN2kMsg N2kMsg;
   #define SID                0xff
   #define MobEmitterId       1 
   
   // SetN2kPGN127233 (same parameters as below)
   SetN2kMOBNotification(N2kMsg, SID, MobEmitterId, ManualOnBoardMOBButtonActivation,
                        (double)(millis() / 1000.0), // ActivationTime (secondes ecoulees depuis un top de reference)
                        PositionEstimatedByVessel, LastGNSS_data.daysSince1970, LastGNSS_data.GNSSTime,
                        LastGNSS_data.Latitude, LastGNSS_data.Longitude,
                        N2khr_true, LastGNSS_data.trueCOG, LastGNSS_data.SOG, uiMMSI_number, Good);
   NMEA2000.SendMsg(N2kMsg);
}

void sendN2kAIS_SART_129038() {           // Man Overboard Notification(AIS SART) Type  1 Position Report Class A Navigation Status 14
   tN2kMsg N2kMsg;
   #define SID                0xff
   #define MessageTypeId1     1           // ITU-R M.1371

   // SetN2kPGN129038 (same parameters as below)
   // 0x3c -> Time Stamp not available
   SetN2kAISClassAPosition(N2kMsg, MessageTypeId1, N2kaisr_Final, uiMMSI_number, LastGNSS_data.Latitude, LastGNSS_data.Longitude, 
                           false, 0, 0x3c, LastGNSS_data.trueCOG, LastGNSS_data.SOG, N2kaisown_information_not_broadcast,
                           N2kDoubleNA, N2kDoubleNA, N2kaisns_AIS_SART, SID);
   NMEA2000.SendMsg(N2kMsg);
}

void sendN2kAIS_SART_129802() {           // Man Overboard Notification(AIS SART) Type 14 Safety-Related Broadcast Message
   tN2kMsg N2kMsg;
   #define MessageTypeId14    14          // ITU-R M.1371
   
   // SetN2kPGN129802 (same parameters as below)
   SetN2kAISSafetyRelatedBroadcastMsg(N2kMsg, MessageTypeId14, N2kaisr_Final, uiMMSI_number, 
                                       N2kaisown_information_not_broadcast, "MOB ACTIVE");
   NMEA2000.SendMsg(N2kMsg);
}
#endif