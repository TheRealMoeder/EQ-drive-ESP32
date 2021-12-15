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

/* SoftAP WiFi Credentials */
const char* ssid = "EQ Plattform"; // SSID
const char* password = ""; // Password
/* Start Webserver */
AsyncWebServer server(80);

/* Attach ESP-DASH to AsyncWebServer */
ESPDash dashboard(&server);
Card dash_step_delay(&dashboard, GENERIC_CARD, "Intervall Schrittmotor", "µs");
Card dash_step_slider(&dashboard, SLIDER_CARD, "Intervallkorrektur", "µs", -500, 500);
Card dash_change_dir(&dashboard, BUTTON_CARD, "Richtung umkehren");
Card dash_reset_plattform(&dashboard, BUTTON_CARD, "Plattform 10 Minuten zurücksetzen");
Card dash_reset_last(&dashboard, BUTTON_CARD, "Letzte Plattformbewegung zurücksetzen");
Card dash_runtime(&dashboard, GENERIC_CARD, "Laufzeit", "min");

#define WEB_UPDATE_RATE 5000000 // update-rate for Webinterface in µs

/* Setup Stepper-driver fr TMC2209 */
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

char _buffer[11];
bool shaft = false;
#define BASE_STEPS 12233 // base steps für 32 fach microstepping: 12233
uint32_t stepdelay = BASE_STEPS;
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
    stepdelay = BASE_STEPS + stored_delay.toInt();
    Serial.print("Adjusting from config.txt: stepdelay ");
    Serial.println(stepdelay);
    dash_step_slider.update(stored_delay);
  } else {
    Serial.println("Cannot read config file");
  }
  f.close();
  
  /* Start Access Point */
  WiFi.mode(WIFI_AP);
  Serial.println(WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0)) ? "Ready" : "Failed!");
  Serial.println(WiFi.softAP(ssid) ? "Ready" : "Failed!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

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

  
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  /* Start AsyncWebServer */
  server.begin();

  /* Setup Dash Callbacks */
  dash_step_slider.attachCallback([&](int value){
    Serial.println("Slider Callback Triggered: "+String(value));
    dash_step_slider.update(value);
    stepdelay = BASE_STEPS + value;
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
  dash_reset_plattform.attachCallback([&](bool value){
    Serial.println("[dash_reset_plattform] Button Callback Triggered: "+String((value)?"true":"false"));
//   dash_reset_plattform.update(value);
//    dashboard.sendUpdates();
    if (value) reset_plattform(60*10); // 10 Minuten zurücksetzen
  });
  dash_reset_last.attachCallback([&](bool value){
    Serial.println("[dash_reset_last] Button Callback Triggered: "+String((value)?"true":"false"));
//    dash_reset_last.update(value);
//    dashboard.sendUpdates();
    if (value && runtime > 196) {
      int reset_factor = (int)(runtime - 196) ; // Letzte Bewegung abzüglich 196 Sekunden Laufzeit für Be- und Entschleunigung
      reset_plattform(reset_factor);
    }
  });

  //SETUP debug Serial
  Serial.println("Setting things up...let's go!");

  // SETUP TMC2209 driver board
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
//  SERIAL_PORT.begin(115200);      // HW UART drivers
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
  Serial.print(amount + 196);
  Serial.println(" runtime seconds");

  flipDirection();
  // Accelerate plattform
  for (uint16_t i = 8000; i>0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    ESP.wdtFeed();
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(50+i/50);
  }
  // Reset given amount for plattform
  for (unsigned long i = amount; i>0; i--) {
        ESP.wdtFeed();
    for(int k = 82; k>0; k--) { // pro Loop eine Sekunde Laufzeit zurücksetzen
        digitalWrite(STEP_PIN, HIGH);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(50);
    }
  }
  // Deccelerate plattform
  for (uint16_t i = 0; i<8000; i++) {
    digitalWrite(STEP_PIN, HIGH);
    ESP.wdtFeed();
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
    dash_runtime.update((float)runtime/60);
    /* Send Updates to our Dashboard (realtime) */
    dashboard.sendUpdates();
    ArduinoOTA.handle();
  }
}
