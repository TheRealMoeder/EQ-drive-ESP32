/*
 * Arduino Stepper Driver with TMC2209 and Webinterface.
 * This is a free software with NO WARRANTY.
 */

 #if defined(ESP8266)
  /* ESP8266 Dependencies */
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <ESP8266mDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#elif defined(ESP32)
  /* ESP32 Dependencies */
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#endif
#include <ESPDash.h>
#include <LittleFS.h>
#include <TMCStepper.h> // TMCStepper library

/* Platform Settings */
#define PLATFORM_MAX_RUNTIME 50 // in minutes
#define PLATFORM_ACCEL_DECEL_TIME 196 // time needed for platform acceleration an decelaration in seconds
#define STEPINTERVAL_SIDERAL 12233 // sideral step interval for 32 microsteps : 12233 (depends on individual platform design and gear ratio)

/* Setup Stepper-driver for TMC2209 */
#define EN_PIN           D5 // Enable
#define DIR_PIN          D6 // Direction
#define STEP_PIN         D7 // Step
#define SW_RX            D4 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            D8 // TMC2208/TMC2224 SoftwareSerial transmit pin
//#define SERIAL_PORT Serial // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075
TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS); // Software Serial

/* SoftAP Settings */
const char* ssid = "EQ Plattform"; // SSID
const char* password = ""; // Password
IPAddress apIP(192, 168, 4, 1); // Static IP for local and gateway

/* UI Settings */
#define WEB_UPDATE_RATE 10000000 // update-rate for Webinterface in µs

/* Start Webserver */
AsyncWebServer server(80);

/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(&server);
Card dash_step_delay(&dashboard, GENERIC_CARD, "Intervall Schrittmotor", "µs");
Card dash_step_slider(&dashboard, SLIDER_CARD, "Intervallkorrektur", "µs", -500, 500);
Card dash_change_dir(&dashboard, BUTTON_CARD, "Richtung umkehren");
Card dash_reset_slider(&dashboard, SLIDER_CARD, "Plattform zurücksetzen", "min", 0-PLATFORM_MAX_RUNTIME, PLATFORM_MAX_RUNTIME);
Card dash_reset_last(&dashboard, BUTTON_CARD, "Letzte Plattformbewegung zurücksetzen");
Card dash_runtime(&dashboard, GENERIC_CARD, "Laufzeit", "min");
char _buffer[11];
bool shaft = false;
uint32_t stepdelay = STEPINTERVAL_SIDERAL;
unsigned long time_last_web_update = 0;
unsigned long time_last_step = 0;
long runtime = 0;


void setup(void)
{
  delay(1000);  // wait a second
  Serial.begin(115200);

  /* Setup Filesystem and read configs*/
  LittleFS.begin();
  File f = LittleFS.open(F("/config.txt"), "r");
  if(f) {
    String stored_delay = f.readString();
    stepdelay = STEPINTERVAL_SIDERAL + stored_delay.toInt();
    Serial.print("Adjusting from config.txt: stepdelay ");
    Serial.println(stepdelay);
    dash_step_slider.update(stored_delay);
  } else {
    Serial.println("Cannot read config file");
  }
  f.close();
  
  /* Start Access Point */
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid);
  
  /* Setup OTA Updates */
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();

  /* Start AsyncWebServer */
  server.begin();

  /* Setup Dash Callbacks */
  dash_step_slider.attachCallback([&](int value){
    Serial.println("Slider Callback Triggered: "+String(value));
    dash_step_slider.update(value);
    stepdelay = STEPINTERVAL_SIDERAL + value;
    // store delay and direction to config file
    File f = LittleFS.open(F("/config.txt"), "w");
    f.print(String(value));
    f.close();
    dashboard.sendUpdates();
  });
  dash_change_dir.attachCallback([&](bool value){
    Serial.println("[dash_change_dir] Button Callback Triggered: "+String((value)?"true":"false"));
    dash_change_dir.update(value);
    flipDirection();
    dashboard.sendUpdates();
  });
  dash_reset_last.attachCallback([&](bool value){
    Serial.println("[dash_reset_last] Button Callback Triggered: "+String((value)?"true":"false"));
    if (value && runtime > PLATFORM_ACCEL_DECEL_TIME) {
      int reset_factor = (int)(runtime - PLATFORM_ACCEL_DECEL_TIME);
      reset_plattform(reset_factor);
    }
  });
  dash_reset_slider.attachCallback([&](int value){
    Serial.println("Reset Slider Callback Triggered: "+String(value));
      dash_reset_slider.update(value);
    dashboard.sendUpdates();
    if(value <0) {
      flipDirection();
      reset_plattform(60*-value);
      flipDirection();
    } else {
      reset_plattform(60*value);
    }
    dash_reset_slider.update(0);
    dashboard.sendUpdates();
  });

  // Setup TMC2209 driver board
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
  //SERIAL_PORT.begin(115200);      // HW UART drivers
  driver.beginSerial(115200);     // SW UART drivers
  driver.begin();                 // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(400);        // Set motor RMS current
  driver.microsteps(32);          // Set microsteps to 1/16th
  //driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true);     // Needed for stealthChop
}

void flipDirection()
{
  Serial.print("Switching direction to ");
  shaft = !shaft;
  Serial.println(shaft);
  driver.shaft(shaft);
}

void set_speed(long new_speed) {
  Serial.print("Changing speed from ");
  Serial.print(stepdelay);
  Serial.print(" to ");
  Serial.println(new_speed);
  stepdelay = new_speed;
}
void reset_plattform(long amount) {
  Serial.print("Resetting plattform ");
  Serial.print(amount + PLATFORM_ACCEL_DECEL_TIME);
  Serial.println(" runtime seconds");

  flipDirection();
  // Accelerate plattform
  for (uint16_t i = 8000; i>0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    ESP.wdtFeed(); // clear watchdog timer
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(50+i/50);
  }
  // Reset given amount for plattform
  for (unsigned long i = amount; i>0; i--) {
        ESP.wdtFeed(); // clear watchdog timer
    for(int k = 82; k>0; k--) { // reset 1 second of runtime per loop
        digitalWrite(STEP_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(50);
    }
  }
  // Deccelerate plattform
  for (uint16_t i = 0; i<8000; i++) {
    digitalWrite(STEP_PIN, HIGH);
    ESP.wdtFeed(); // clear watchdog timer
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(50+i/50);
  }
  flipDirection();
  Serial.println("Finished resetting plattform");
  runtime = 0;
  dash_runtime.update(0);
}

void loop()
{
  unsigned long current_micros = micros();

  // Send next step if necessary
  if (current_micros - time_last_step >= stepdelay) {
    digitalWrite(STEP_PIN, HIGH);
    time_last_step = current_micros;
    digitalWrite(STEP_PIN, LOW);
  }
  
  // Update Webinterface if necessary
  if (current_micros - time_last_web_update >= WEB_UPDATE_RATE) {
    time_last_web_update = current_micros;
    runtime += WEB_UPDATE_RATE / 1000000;
    /* Update Card Values */
    dash_step_delay.update((int)stepdelay);
    dash_runtime.update((float)runtime/60); // display runtime in minutes
    /* Send Updates to our Dashboard (realtime) */
    dashboard.sendUpdates();
    ArduinoOTA.handle();
  }

  // Reset plattform if maximum runtime is exceeded
  if (runtime >= (PLATFORM_MAX_RUNTIME*60-PLATFORM_ACCEL_DECEL_TIME))
    reset_plattform(runtime);
}
