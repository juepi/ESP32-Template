/*
 * ESP32 Sketch template
 * ========================
 * 
 * Includes useful functions like
 * - Watchdog
 * - DeepSleep
 * - Read VCC (-> doesn't work!)
 * - MQTT
 * - OTA Sketch Updates (ATTN: requires MQTT!)
 * 
 * ATTENTION: Keep in mind that it takes quite a while after the sketch has booted until we receive messages from all subscribed topics!
 * This is especially important if you want to maximize battery lifetime.
 * To get OTA update working on windows, you need to install python and python.exe needs to be in %PATH%
 * First program download needs to be wired of course. Afterwards Arduino IDE needs to be restarted if you cannot find
 * the ESP OTA-port in the IDE (also MQTT ota_topic needs to be set to "on" to be able to flash OTA).
 * Keep in mind that you'll need a reliable power source for OTA updates, 2x AA batteries might not work.
 * If you brick your ESP during OTA update, you can probably revive it by flashing it wired.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <driver/adc.h>
#include "esp_system.h"


// Pin used for reading VCC (ADC1 channel 0)
// ==========================================
// ATTN: i have no idea how this supposed to work, as it seems that there is no real
// reference voltage (other than VCC, which is useless of course) for the ADC - at least for my DevkitC board
// with 0dB attenuation Full Scale voltage *should be* 1.1V... use 330k/120k voltage divider (factor 3.75)
#define VCC_ADC_PIN 36


// Enable (define) debug output on serial port
// ============================================
#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif


// Enable Watchdog (sends ESP to DeepSleep if sketch hangs)
// =========================================================
#define ENABLEWATCHDOG
#ifdef ENABLEWATCHDOG
  // OS Timer for Software Watchdog
  hw_timer_t *WDTimer = NULL;
  // WDT will trigger every 10 seconds
  #define WDTIMEOUT 10
  #define RESET_WATCHDOG timerWrite(WDTimer, 0)
#else
  #define RESET_WATCHDOG
#endif
// WD may be overriden (i.e. OTA Update)
bool OverrideWD = false;


// Builtin LED on 2 (LED inverted!)
// =================================
// No onboard LED on my DevKit!
//#define USELED //do not define this to disable LED signalling
#define LED 2
#define LEDON LOW
#define LEDOFF HIGH


// Enable ESP DeepSleep
// =====================
// Enable (define) ESP deepSleep
//#define DEEPSLEEP
// Define sleep time (30 minutes)
#define DeepSleepMinutes 30
uint64_t sleep_time_us = DeepSleepMinutes * 60 * 1000000;


// WLAN Network SSID and PSK
// ==========================
#define WIFINAME ESP32Test1
WiFiClient WIFINAME;
const char* ssid = "xxx";
const char* password = "xxx";


// OTA Update settings
// ====================
// OTANAME will show up as Arduino IDE "Port" Name
#define OTANAME "Esp32DevKit1"
#define OTAPASS "xxx"


// MQTT Broker Settings
// =====================
#define mqtt_server "192.168.1.1"
#define mqtt_Client_Name "ESP32test"
// Maximum connection attempts to MQTT broker before going to sleep
const int MaxConnAttempts = 3;
// Message buffer for incoming Data from MQTT subscriptions
char message_buff[20];


// MQTT Topics and corresponding local vars
// =========================================
//OTA Update specific vars
//to start an OTA update on the ESP, you will need to set ota_topic to "on" on your broker
//(don't forget to add the "retain" flag, especially if you want a sleeping ESP to enter flash mode at next boot)
#define ota_topic "HB7/Test/OTAupdate" //local BOOL, MQTT either "on" or "off"
bool OTAupdate = false;
#define otaStatus_topic "HB7/Test/OTAstatus"
// OTAstatus strings sent by sketch
#define UPDATEREQ "update_requested"
#define UPDATECANC "update_cancelled"
#define UPDATEOK "update_success"
bool SentUpdateRequested = false;
//An additional "external flag" is required to "remind" a freshly running sketch that it was just OTA-flashed..
//during an OTA update, PubSubClient functions do not run (or cannot access the network)
//so this flag will be set to ON when actually waiting for the OTA update to start
//it will be reset if OtaInProgress and OTAupdate are true (in that case, ESP has most probably just been successfully flashed)
#define otaInProgress_topic "HB7/Test/OTAinProgress" //local BOOL, MQTT either "on" or "off"
bool OtaInProgress = false;
bool OtaIPsetBySketch = false;
bool SentOtaIPtrue = false;

// Topic where VCC will be published
#define vcc_topic "HB7/Test/Vcc"
float VCC = 3.333;


// Use RTC RAM to store Variables that should survive DeepSleep
// =============================================================
// ATTN: define KEEP_RTC_SLOWMEM or vars will be lost (PowerDomain disabled)
#define KEEP_RTC_SLOWMEM
RTC_DATA_ATTR int SaveMe = 0;



/*
 * Callback Functions
 * ========================================================================
 */

// Watchdog Timer Callback function
// =================================
#ifdef ENABLEWATCHDOG
void IRAM_ATTR WDTCallback()
{
  if (OverrideWD) {
    return;
  }
  // WD function triggered, program is probably dead, go to DeepSleep
  #ifdef USELED
  // signal SOS
  digitalWrite(LED, LEDOFF);
  delay(250);
  ToggleLed(LED,200,6);
  ToggleLed(LED,600,6);
  ToggleLed(LED,200,6);
  #endif
  DEBUG_PRINTLN("Watchdog Timer detected program not responding, going to sleep!");
  #ifdef DEEPSLEEP
  WiFi.disconnect();
  esp_sleep_enable_timer_wakeup(sleep_time_us);
  esp_deep_sleep_start();
  delay(3000);
  #endif
  // ..or you might want to reset the ESP instead..
  // esp_restart();
  delay(100);
}
#endif


// MQTT Subscription callback function
// ====================================
void MqttCallback(char* topic, byte* payload, unsigned int length)
{
  int i = 0;
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  String msgString = String(message_buff);

  DEBUG_PRINTLN("MQTT: Message arrived [" + String(topic) + "]: " + String(msgString));

  // run through topics
  if ( String(topic) == ota_topic ) {
    if (msgString == "on") { OTAupdate = true; }
    else if (msgString == "off") { OTAupdate = false; }
    else
    {
      DEBUG_PRINTLN("MQTT: ERROR: Fetched invalid OTA-Update: " + String(msgString));
      delay(200);
    }
  }
  else if ( String(topic) == otaInProgress_topic ) {
    if (msgString == "on") { OtaInProgress = true; }
    else if (msgString == "off") { OtaInProgress = false; }
    else
    {
      DEBUG_PRINTLN("MQTT: ERROR: Fetched invalid OtaInProgress: " + String(msgString));
      delay(200);
    }
  }
  else {
    DEBUG_PRINTLN("ERROR: Unknown topic: " + String(topic));
    DEBUG_PRINTLN("ERROR: Unknown topic value: " + String(msgString));
    delay(200);
  }     
}


/*
 * Setup PubSub Client instance
 * ===================================
 * must be done before setting up ConnectToBroker function and after MqttCallback Function
 * to avoid compilation errors
 */
PubSubClient mqttClt(mqtt_server,1883,MqttCallback,WIFINAME);


/*
 * Common Functions
 * =================================================
 */

bool ConnectToBroker()
{
  bool RetVal = false;
  int ConnAttempt = 0;
  // Try to connect x times, then return error
  while (ConnAttempt < MaxConnAttempts)
  {
    DEBUG_PRINT("Connecting to MQTT broker..");
    // Attempt to connect
    if (mqttClt.connect(mqtt_Client_Name))
    {
      DEBUG_PRINTLN("connected");
      RetVal = true;
      
      // Subscribe to Topics
      if (mqttClt.subscribe(ota_topic))
      {
        DEBUG_PRINTLN("Subscribed to " + String(ota_topic));
        delay(1);
      }
      else
      {
        DEBUG_PRINTLN("Failed to subscribe to " + String(ota_topic));
        delay(100);
      }
      if (mqttClt.subscribe(otaInProgress_topic))
      {
        DEBUG_PRINTLN("Subscribed to " + String(otaInProgress_topic));
        delay(1);
      }
      else
      {
        DEBUG_PRINTLN("Failed to subscribe to " + String(otaInProgress_topic));
        delay(100);
      }
      delay(200);
      break;
    } else {
      DEBUG_PRINTLN("failed, rc=" + String(mqttClt.state()));
      DEBUG_PRINTLN("Sleeping 2 seconds..");
      delay(2000);
      ConnAttempt++;
    }
  }
  return RetVal;
}


void ToggleLed (int PIN,int WaitTime,int Count)
{
  // Toggle digital output
  for (int i=0; i < Count; i++)
  {
   digitalWrite(PIN, !digitalRead(PIN));
   delay(WaitTime); 
  }
}


/*
 * Setup
 * ========================================================================
 */
void setup() {
  //ATTN: This delay might be required if ESP doesn't boot every time after DeepSleep
  //delay(500);
  
  // start serial port and digital Outputs
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  DEBUG_PRINTLN();
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("ESP32 Template");
  #ifdef USELED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LEDOFF);
  ToggleLed(LED,200,6);
  #endif

  // Setup ADC
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0 );
  adc1_config_width(ADC_WIDTH_BIT_12);
  
  // Disable all power domains on ESP while in DeepSleep (actually Hibernation)
  // wake up only by RTC
  #ifndef KEEP_RTC_SLOWMEM
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  #endif
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

  // Connect to WiFi network  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Connecting to " + String(ssid));
  WiFi.begin(ssid, password);
  // Next command avoids ESP broadcasting an unwanted ESSID..
  WiFi.mode(WIFI_STA);
   
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("WiFi connected");
  DEBUG_PRINT("Device IP Address: ");
  DEBUG_PRINTLN(WiFi.localIP());
  #ifdef USELED
  // WiFi connected - blink once
  ToggleLed(LED,200,2);
  #endif
  
  // Setup MQTT Connection to broker and subscribe to topic
  if (ConnectToBroker())
  {    
    DEBUG_PRINTLN("Connected to MQTT broker, fetching topics..");
    mqttClt.loop();
    #ifdef USELED
    // broker connected - blink twice
    ToggleLed(LED,200,4);
    #else
    delay(300);
    #endif
  }
  else
  {
    DEBUG_PRINTLN("3 connection attempts to broker failed, using default values..");
    delay(100);
  }

  // Setup OTA Updates
  //ATTENTION: calling MQTT Publish function inside ArduinoOTA functions MIGHT NOT WORK!
  ArduinoOTA.setHostname(OTANAME);
  ArduinoOTA.setPassword(OTAPASS);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
  });
  ArduinoOTA.onEnd([]() {
    #ifdef USELED
    ToggleLed(LED,200,4);
    #else
    //ATTENTION: calling MQTT Publish function here does NOT WORK!
    delay(200);
    #endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int percentComplete = (progress / (total / 100));
    if (percentComplete = 100) {
      mqttClt.publish(otaStatus_topic, String("upload_complete").c_str(), true);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    mqttClt.publish(ota_topic, String("off").c_str(), true);
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      mqttClt.publish(otaStatus_topic, String("Auth_Error").c_str(), true);
    } else if (error == OTA_BEGIN_ERROR) {
      mqttClt.publish(otaStatus_topic, String("Begin_Error").c_str(), true);
    } else if (error == OTA_CONNECT_ERROR) {
      mqttClt.publish(otaStatus_topic, String("Connect_Error").c_str(), true);
    } else if (error == OTA_RECEIVE_ERROR) {
      mqttClt.publish(otaStatus_topic, String("Receive_Error").c_str(), true);
    } else if (error == OTA_END_ERROR) {
      mqttClt.publish(otaStatus_topic, String("End_Error").c_str(), true);
    }
    delay(300);
  });
  ArduinoOTA.begin();

  #ifdef ENABLEWATCHDOG
  // Assign function and arm Watchdog Timer
  WDTimer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(WDTimer, &WDTCallback, true);  //attach callback
  timerAlarmWrite(WDTimer, WDTIMEOUT * 1000000, false); //set time in us
  timerAlarmEnable(WDTimer);                          //enable interrupt
  RESET_WATCHDOG; // Reset WD Timer
  OverrideWD = false;
  #endif

  #ifdef USELED
  // Signal setup finished
  delay(300);
  ToggleLed(LED,200,6);
  #endif
}



/*
 * Main Loop
 * ========================================================================
 */
void loop() {
  // Dont forget to reset WD timer regularily if enabled
  RESET_WATCHDOG; // Reset WD Timer
  delay(200);
  // Check connection to MQTT broker and update topics
  if (!mqttClt.connected()) {
    if (ConnectToBroker()) {
      mqttClt.loop();
    } else {
      DEBUG_PRINTLN("Unable to connect to MQTT broker.");
      delay(100);
    }
  } else {
    mqttClt.loop();
  }

  // If OTA Firmware Update is requested,
  // only loop through OTA function until finished (or reset by MQTT)
  if (OTAupdate) {
    DEBUG_PRINTLN("Millis: " + String(millis()));
    if (millis() < 27000) {
      // this delay is required to make sure that we know our correct status before doing anything..
      // shorter delay will not work reliably (fetching all MQTT topics takes a long time)
      DEBUG_PRINTLN("Sketch just booted, delaying OTA operation until all MQTT topics arrived..");
      #ifdef USELED
      ToggleLed(LED,1000,2);
      #else
      delay(2000);
      #endif
      return;
    }
    if (OtaInProgress && !OtaIPsetBySketch) {
      DEBUG_PRINTLN("OTA firmware update successful, resuming normal operation..");
      mqttClt.publish(otaStatus_topic, String(UPDATEOK).c_str(), true);
      mqttClt.publish(ota_topic, String("off").c_str(), true);
      mqttClt.publish(otaInProgress_topic, String("off").c_str(), true);
      OTAupdate = false;
      OtaInProgress = false;
      OtaIPsetBySketch = true;
      SentOtaIPtrue = false;
      SentUpdateRequested = false;
      RESET_WATCHDOG; // Reset WD Timer
      OverrideWD = false;    
      return;
    }
    if (!SentUpdateRequested) {
      mqttClt.publish(otaStatus_topic, String(UPDATEREQ).c_str(), true);
      SentUpdateRequested = true;
    }
    // Override watchdog during OTA update
    OverrideWD = true;
    DEBUG_PRINTLN("OTA firmware update requested, waiting for upload..");
    #ifdef USELED
    // Signal OTA update requested
    ToggleLed(LED,100,10);
    #endif
    //set MQTT reminder that OTA update was executed
    if (!SentOtaIPtrue) {
      DEBUG_PRINTLN("Setting MQTT OTA-update reminder flag on broker..");
      mqttClt.publish(otaInProgress_topic, String("on").c_str(), true);
      OtaInProgress = true;      
      SentOtaIPtrue = true;
      OtaIPsetBySketch = true;
      delay(100);
    }
    //call OTA function to receive upload
    ArduinoOTA.handle();
    return;
  } else {
    if (SentUpdateRequested) {
      DEBUG_PRINTLN("OTA firmware update cancelled by MQTT, resuming normal operation..");
      mqttClt.publish(otaStatus_topic, String(UPDATECANC).c_str(), true);
      mqttClt.publish(otaInProgress_topic, String("off").c_str(), true);
      OtaInProgress = false;
      OtaIPsetBySketch = true;
      SentOtaIPtrue = false;
      SentUpdateRequested = false;
      RESET_WATCHDOG; // Reset WD Timer
      OverrideWD = false;
    }
  }

  RESET_WATCHDOG; // Reset WD Timer

  // START STUFF YOU WANT TO RUN HERE!
  // ============================================
  #ifdef USELED
  // Toggle LED at each loop
  ToggleLed(LED,500,4);
  #endif
  
  // Read VCC and publish to MQTT
  // Disabled due to bullshit readings
  //  VCC = 3.75f * 1.1f * float(analogRead(VCC_ADC_PIN)) / 4096.0f;
  //  mqttClt.publish(vcc_topic, String(VCC).c_str(), true);
  //  DEBUG_PRINTLN("VCC = " + String(VCC) + " V");
  //  DEBUG_PRINT("Raw ADC Pin readout: ");
  //  DEBUG_PRINTLN(analogRead(VCC_ADC_PIN));
  //  delay(100);

  #ifdef DEEPSLEEP
  // disconnect WiFi and go to sleep
  DEBUG_PRINTLN("Good night for " + String(DeepSleepMinutes) + " minutes.");
  WiFi.disconnect();
  esp_sleep_enable_timer_wakeup(sleep_time_us);
  esp_deep_sleep_start();
  #else
  DEBUG_PRINTLN("Loop finished, DeepSleep disabled. Restarting in 5 seconds.");
  RESET_WATCHDOG; // Reset WD Timer
  #endif
  //ATTN: Sketch continues to run for a short time after initiating DeepSleep, so pause here
  delay(5000);
}
