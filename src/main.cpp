#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>   // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include "PwmControl.h"
#include <WebServer.h>     // For incoming POST, GET requests
#include <HTTPClient.h>    // For outgoing POST requests
#include <WiFi.h>          // Needed for HTTPClient and WiFi functions
#include <ArduinoJson.h>
#include <atomic>
#include <dmx.h>
//Make sure to update the library variables from the README
//#include <SparkFunDMX.h>

#define MAX_DMX_CHANNELS 512
#define LED_START_CHANNEL 220   // The channel the LEDs start on the lightkey universe
#define LED_STRIP_AMOUNT 10     // The amount of LED strips currently used
#define CONTROLLER_COUNT 3      // Adafruit 16 channel 12 bit PWM controllers
#define CHANNELS_PER_CONTROLLER 16
#define CONTROLLER_FIRST_ADDRESS 0x40
#define CONTROLLER_FREQ 600
#define PORT 800
#define LOGGING_URL "http://10.10.10.69" // URL to send the DMX log to

void taskProcessDMX(void *parameter);
void taskProcessNetwork(void *parameter);
void taskIdle(void *parameter);
void printDMXData(uint8_t *chanVal); 
void checkUnexpectedColorData(uint8_t *chanVal, size_t *length);
void pulseLogic(int *pulseValue);
void setDMXLights(uint8_t *chanVal);
void sendDMXLog(uint8_t *dmxData);

//atomic flags for thread-safe mode control
std::atomic<bool> pulseMode(false); 
std::atomic<bool> manualMode(false);

struct RGBW {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
};
volatile RGBW postColor; 

const uint8_t BUTTON_PIN = 0;
const uint8_t LED_PIN = GPIO_NUM_13;
volatile int8_t pulseDirection = 1;
volatile int pulseSpeed = 2; // Speed of the pulse effect

PwmControl *pwmControllers[CONTROLLER_COUNT];
WiFiManager wifiManager;
WebServer server(PORT);

void handlePost()
{
  if (server.hasArg("plain") == false)
  {
    server.send(400, "application/json", "{\"message\":\"Invalid request\"}");
    return;
  }
  String body = server.arg("plain");
  StaticJsonDocument<250> jsonDocument;
  DeserializationError error = deserializeJson(jsonDocument, body);

  if (error) {
    server.send(400, "application/json", "{\"message\":\"JSON parse error\"}");
    return;
  }

  bool multi = jsonDocument["multi"] | false;
  bool pulse = jsonDocument["pulse"] | false;
  bool manual = jsonDocument["manual"] | false;
  int pulseSpeed = jsonDocument["pulseSpeed"] | 2;
  
  //Default false values for pulse and manual
  pulseMode.store(false);
  manualMode.store(false);

  uint8_t red = jsonDocument["red"] | 0;
  uint8_t green = jsonDocument["green"] | 0;
  uint8_t blue = jsonDocument["blue"] | 0;
  uint8_t white = jsonDocument["white"] | 0;

  postColor.r = red;
  postColor.g = green;
  postColor.b = blue;
  postColor.w = white;

  if (pulse) {
    pulseMode.store(true);
    server.send(200, "application/json", "{\"message\":\"pulse mode enabled, Ignoring DMX\"}");
  }
  if (manual) {
    manualMode.store(true);
    server.send(200, "application/json", "{\"message\":\"manual mode enabled, ignoring DMX data\"}");
  }

  if (multi) { //Currently causes it to turn on and then immediately turn off
    for (int controllerNum = 0; controllerNum < CONTROLLER_COUNT; controllerNum++) {
      for (int channel = 0; channel < CHANNELS_PER_CONTROLLER; channel++) {
        pwmControllers[controllerNum]->SetColor(channel, red, green, blue, white);
      }
    }
    Serial.printf("All channels set to R:%d G:%d B:%d W:%d\n", red, green, blue, white);
    server.send(200, "application/json", "{\"message\":\"All LEDs updated successfully\"}");
  } 
  else {
    uint8_t channel = jsonDocument["channel"];
    uint8_t controllerNum = channel / CHANNELS_PER_CONTROLLER;
    if (controllerNum < CONTROLLER_COUNT) {
      pwmControllers[controllerNum]->SetColor(channel - (controllerNum * CHANNELS_PER_CONTROLLER), red, green, blue, white);
      Serial.printf("Channel %d set to R:%d G:%d B:%d W:%d\n", channel, red, green, blue, white);
      server.send(200, "application/json", "{\"message\":\"LED updated successfully\"}");
    } else { //This doesnt work if the channel is not in the range of the controllers
      server.send(500, "application/json", "{\"message\":\"Invalid channel\"}");
    }
  }
}

void sendDMXLog(uint8_t *dmxData) {
  StaticJsonDocument<1024> jsonDocument;
  jsonDocument["type"] = "dmx_log";
  char timestamp[25];
  time_t now = time(nullptr);
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));
  jsonDocument["timestamp"] = timestamp;

  for (uint8_t led = 0; led < LED_STRIP_AMOUNT; led++) {
    jsonDocument["LED Strip"][led + 1]["R"] = dmxData[led * 4];
    jsonDocument["LED Strip"][led + 1]["G"] = dmxData[led * 4 + 1];
    jsonDocument["LED Strip"][led + 1]["B"] = dmxData[led * 4 + 2];
    jsonDocument["LED Strip"][led + 1]["W"] = dmxData[led * 4 + 3];
  }
  String jsonString;
  serializeJson(jsonDocument, jsonString);

  HTTPClient http;
  http.begin(LOGGING_URL);
  http.addHeader("Content-Type", "application/json");
  int responseCode = http.POST(jsonString);
  Serial.println(responseCode);
  http.end();


  // Find a way to send a POST request to a server that this chip can handle
}

void setup_routing()
{
  server.on("/led", HTTP_POST, handlePost);

  // start server
  server.begin();
}

void setup() //Gets run once
{
  Serial.begin(9600);
  TaskHandle_t xIdleHandle = NULL;

  DMX::Initialize(input);

  Serial.println("DMX Light Controller");
  
  pinMode(LED_PIN, OUTPUT);

  for (int i = 0; i < CONTROLLER_COUNT; i++)
  {
    pwmControllers[i] = new PwmControl(CONTROLLER_FIRST_ADDRESS + i, CONTROLLER_FREQ);
    pwmControllers[i]->begin();
  }

  TaskHandle_t xDMXHandle = NULL;
  xTaskCreatePinnedToCore(taskProcessDMX, "taskProcessDMX", 4096, NULL, 2, &xDMXHandle, 1);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  wifiManager.setConfigPortalTimeout(30);
  wifiManager.autoConnect("DMX_Controller");

  ArduinoOTA
    .onStart([]() 
    {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); 
    })
    .onEnd([]()
      { Serial.println("\nEnd"); })
    .onProgress([](unsigned int progress, unsigned int total)
      { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
    .onError([](ota_error_t error)
      {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR)
        Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)
        Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR)
        Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR)
        Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)
        Serial.println("End Failed");
      });

  ArduinoOTA.begin();

  setup_routing();

  TaskHandle_t xNetHandle = NULL;
  xTaskCreate(taskProcessNetwork, "taskProcessNetwork", 4096, NULL, 1, &xNetHandle);
}

void loop() {
  delay(100);
}

void taskProcessNetwork(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    server.handleClient();
    ArduinoOTA.handle();
  }
}

void taskProcessDMX(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t chanVal[CONTROLLER_COUNT * CHANNELS_PER_CONTROLLER + 1];
  uint8_t lastValidData[CONTROLLER_COUNT * CHANNELS_PER_CONTROLLER + 1];
  uint8_t counter = 0;
  int pulseValue = 0;
  const uint8_t print_every = 100;
  for (;;)
  {

    if (manualMode.load()) {
      // Serial.println("Manual mode enabled, skipping DMX processing");
      continue;
    }

    //This is to test how smooth the leds are when changing the brightness
    if (pulseMode.load()) {
      pulseLogic(&pulseValue);
      continue; // skip DMX processing when in pulse mode
    }

    //Without this delay the LEDs will flash random colors
    //Will need to play with the value to see how much it effects the performance
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);

    if (!DMX::IsHealthy()) { //If no DMX data is available

      digitalWrite(LED_PIN, LOW); //turns off the LED
      Serial.println("DMX is not healthy, skipping read");

      //Sets the LEDs to the last valid data
      setDMXLights(lastValidData);
      continue; //Skips DMX
    }

    digitalWrite(LED_PIN, HIGH); //turns on the LED

    //Currently there is a issue on the lights not being smooth when changing values
    DMX::ReadAll(chanVal, LED_START_CHANNEL, sizeof(chanVal) - 1);
    setDMXLights(chanVal);

    if (counter % print_every * 20 == 0 && DMX::IsHealthy()) {
      memcpy(lastValidData, chanVal, sizeof(chanVal)); // Copy the current data to lastValidData
    }

    // printDMXData(chanVal);
    counter++;
    // yield(); // Allow other tasks to run

  }
}

void taskIdle(void *parameter)
{
  while (true)
  {
    Serial.print(".");
  }
}

void printDMXData(uint8_t *chanVal) {
  Serial.println("DMX data:");

  // Formatting so the data is easier to read
  for (uint8_t controller = 0; controller < CONTROLLER_COUNT; controller++) {
    for (uint8_t channel = 0; channel < CHANNELS_PER_CONTROLLER; channel += 4) {
        uint8_t currentChannel = channel + (controller * CHANNELS_PER_CONTROLLER);
        Serial.printf("LED: %d, Controller %d, Channel %d: R:%d G:%d B:%d W:%d\n",
            currentChannel/4, controller, currentChannel,
            chanVal[currentChannel], chanVal[currentChannel + 1],
            chanVal[currentChannel + 2], chanVal[currentChannel + 3]);
    }
  }
}

void pulseLogic(int *pulseValue) {

  uint8_t red = *pulseValue;
  uint8_t green = *pulseValue;
  uint8_t blue = *pulseValue;
  uint8_t white = *pulseValue;

  // Will check if the POST request sent any color values
  // If not, it will set them to 0 | else it will pulse the color
  if (postColor.r < 1) {
    red = 0;
  }
  if (postColor.g < 1) {
    green = 0;
  }
  if (postColor.b < 1) {
    blue = 0;
  }
  if (postColor.w < 1) {
    white = 0;
  }

  // Pulse effect logic
  // This will set the color of all LEDs to the pulse value
  for (int ledStrip = 0; ledStrip < LED_STRIP_AMOUNT; ledStrip++) {
    for (uint8_t controller = 0; controller < CONTROLLER_COUNT; controller++) {
      pwmControllers[controller]->SetColor(ledStrip*4, red, green, blue, white);
    }
  }

  // Bounce pulseValue between 0 and 255
  *pulseValue += pulseDirection * pulseSpeed;

  if (*pulseValue >= 255) {
    *pulseValue = 255;
    pulseDirection = -1;
  } else if (*pulseValue <= 0) {
    *pulseValue = 0;
    pulseDirection = 1;
  }
}

void setDMXLights(uint8_t *chanVal) {
  for (uint8_t controller = 0; controller < CONTROLLER_COUNT; controller++) {
    for (uint8_t channel = 0; channel < CHANNELS_PER_CONTROLLER; channel += 4) {
        // Calculate the actual channel index
        uint8_t currentChannel = channel + (controller * CHANNELS_PER_CONTROLLER);

        if ((controller * channel)/ 4 >= LED_STRIP_AMOUNT) {
          return; // Prevents out of bounds access writes
        }
        pwmControllers[controller]->SetColor(
          channel, chanVal[currentChannel], chanVal[currentChannel + 1],
          chanVal[currentChannel + 2], chanVal[currentChannel + 3]);
    }
  }
}