#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include "PwmControl.h"
#include <WebServer.h>
#include <ArduinoJson.h>
#include <dmx.h>
//The reason why dmx.h didn't work is because it is hard coded to use pin 4 for the direction pin
//If there is still issues look into using a different library like SparkFunDMX
//#include <SparkFunDMX.h>

#define MAX_DMX_CHANNELS 512
#define LED_START_CHANNEL 220 //The channel the LEDs start on the lightkey universe
#define LED_STRIP_AMOUNT 12 //The amount of LED strips in the lightkey universe
#define CONTROLLER_COUNT 3 //Adafruit 16 channel 12 bit PWM controllers
#define CHANNELS_PER_CONTROLLER 16
#define CONTROLLER_FIRST_ADDRESS 0x40
#define CONTROLLER_FREQ 600
#define PORT 800

void taskProcessDMX(void *parameter);
void taskProcessNetwork(void *parameter);
void taskIdle(void *parameter);

volatile bool pulseMode = false;
volatile bool manualMode = false;
const uint8_t BUTTON_PIN = 0;
const uint8_t LED_PIN = GPIO_NUM_13;
int8_t pulseDirection = 1;
int pulseSpeed = 2; // Speed of the pulse effect
int pulseValue = 0; // instead of uint8_t

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

  if (pulse) {
    pulseMode = true;
    server.send(200, "application/json", "{\"message\":\"pulse mode enabled\"}");
  }
  if (manual) {
    manualMode = true;
    server.send(200, "application/json", "{\"message\":\"manual mode enabled, ignoring DMX data\"}");
  }

  uint8_t red = jsonDocument["red"];
  uint8_t green = jsonDocument["green"];
  uint8_t blue = jsonDocument["blue"];
  uint8_t white = jsonDocument["white"];

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
  //2176 = stack size, which is the amount of memory allocated for the task
  xTaskCreatePinnedToCore(taskProcessDMX, "taskProcessDMX", 2176, NULL, 2, &xDMXHandle, 1);

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
  // initialize PWM controllers

  TaskHandle_t xNetHandle = NULL;
  xTaskCreate(taskProcessNetwork, "taskProcessNetwork", 4096, NULL, 1, &xNetHandle);
}

void loop() {
  delay(100); // Adjust as needed
}

void checkButton()
{
  static bool portalRunning = false;
  // is auto timeout portal running
  if (portalRunning)
  {
    wifiManager.process();
  }

  // is configuration portal requested?
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    delay(100);
    if (digitalRead(BUTTON_PIN) == LOW)
    {
      if (!portalRunning)
      {
        Serial.println("Button Pressed, Starting Portal");
        wifiManager.startWebPortal();
        portalRunning = true;
      }
      else
      {
        Serial.println("Button Pressed, Stopping Portal");
        wifiManager.stopWebPortal();
        portalRunning = false;
      }
    }
  }
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
  const uint8_t print_every = 100;
  for (;;)
  {
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);

    //Pauses the task until manualMode is set to false from the API
    while (manualMode) {}

    //This is to test how smooth the leds are when changing the brightness
    if (pulseMode) {
      for (int ledStrip = 0; ledStrip < LED_STRIP_AMOUNT; ledStrip++) {
        pwmControllers[0]->SetColor(ledStrip*4, 0, 0, 0, (uint8_t)pulseValue);
        pwmControllers[1]->SetColor(ledStrip*4, 0, 0, 0, (uint8_t)pulseValue);
        pwmControllers[2]->SetColor(ledStrip*4, 0, 0, 0, (uint8_t)pulseValue);
      }

      // Bounce pulseValue between 0 and 255
      pulseValue += pulseDirection * pulseSpeed;
      if (pulseValue >= 255) {
        pulseValue = 255;
        pulseDirection = -1;
      } else if (pulseValue <= 0) {
        pulseValue = 0;
        pulseDirection = 1;
      }
      continue; // skip DMX processing when in pulse mode
    }

    if (!DMX::IsHealthy()) { //If no DMX data is available

      digitalWrite(LED_PIN, LOW); //turns off the LED
      Serial.println("DMX is not healthy, skipping read");

      //Sets the LEDs to the last valid data
      for (int channel = 0; channel < sizeof(lastValidData) - 9; channel += 4)
      {
        uint8_t controllerNum = channel / CHANNELS_PER_CONTROLLER;
        pwmControllers[controllerNum]->SetColor(
            channel - (controllerNum * CHANNELS_PER_CONTROLLER),
            lastValidData[channel], lastValidData[channel + 1], lastValidData[channel + 2], lastValidData[channel + 3]);
      }
      continue; //Skips DMX
    }

    digitalWrite(LED_PIN, HIGH); //turns on the LED

    //Currently there is a issue on the lights not being smooth when changing values
    DMX::ReadAll(chanVal, LED_START_CHANNEL, sizeof(chanVal) - 9);
    for (int channel = 0; channel < sizeof(chanVal) - 9; channel += 4)
    {
      uint8_t controllerNum = channel / CHANNELS_PER_CONTROLLER;
      pwmControllers[controllerNum]->SetColor(
          channel - (controllerNum * CHANNELS_PER_CONTROLLER),
          chanVal[channel], chanVal[channel + 1], chanVal[channel + 2], chanVal[channel + 3]);
    }
 
    Serial.println("DMX data:");

    // Formatting so the data is easier to read
    // Print the channel values in groups of 5 CH(W, R, G, B)
    //Could be imporoved to be a function call
    const int groupSize = 4;
    const int groupsPerLine = 5;

    int totalGroups = (sizeof(chanVal) - 9) / groupSize;

    for (int i = 0; i < totalGroups; i++) {
      int index = i * groupSize;

      Serial.printf("%d(", i);
      
      for (int j = 0; j < groupSize; j++) {
        Serial.printf("%d", chanVal[index + j]);
        if (j < groupSize - 1) Serial.print(", ");
      }

      Serial.print(")");

      if ((i + 1) % groupsPerLine == 0 || i == totalGroups - 1) {
        Serial.println();
      } else {
        Serial.print("  ");
      }
    }

    if (counter % print_every * 20 == 0 && DMX::IsHealthy()) {
      memcpy(lastValidData, chanVal, sizeof(chanVal)); // Copy the current data to lastValidData
    }
    counter++;
  }
}

void taskIdle(void *parameter)
{
  while (true)
  {
    Serial.print(".");
  }
}