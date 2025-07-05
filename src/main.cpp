#include <Arduino.h>
#include <ArduinoOTA.h>

#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include "PwmControl.h"

#include <WebServer.h>
#include <ArduinoJson.h>

#include <dmx.h>

WiFiManager wifiManager;
void checkButton();

WebServer server(800);

char buffer[250];

#define CONTROLLER_COUNT 3
#define CHANNELS_PER_CONTROLLER 16
#define CONTROLLER_FIRST_ADDRESS 0x40
#define CONTROLLER_FREQ 600
//* pwm controllers
PwmControl *pwmControllers[CONTROLLER_COUNT];

const uint8_t BUTTON_PIN = 0;
const uint8_t LED_PIN = GPIO_NUM_13;

void taskProcessDMX(void *parameter);
void taskProcessNetwork(void *parameter);
void taskIdle(void *parameter);

void handlePost()
{
  if (server.hasArg("plain") == false)
  {
    // handle error here
  }
  String body = server.arg("plain");
  // JSON data buffer
  StaticJsonDocument<250> jsonDocument;
  deserializeJson(jsonDocument, body);

  const uint8_t channel = jsonDocument["channel"];
  // Get RGB components
  const uint8_t red = jsonDocument["red"];
  const uint8_t green = jsonDocument["green"];
  const uint8_t blue = jsonDocument["blue"];
  const uint8_t white = jsonDocument["white"];

  uint8_t controllerNum = channel / CHANNELS_PER_CONTROLLER;
  if (controllerNum < CONTROLLER_COUNT)
  {
    pwmControllers[controllerNum]->SetColor(channel - (controllerNum * CHANNELS_PER_CONTROLLER), red, green, blue, white);
    printf("%i, %i, %i, %i \n", red, green, blue, white);
    // Respond to the client
    server.send(200, "application/json", "{}");
  }
  else
  {
    server.send(500, "application/json", "{\"message\":\"Invalid channel\"}");
  }
}

void setup_routing()
{
  server.on("/led", HTTP_POST, handlePost);

  // start server
  server.begin();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  TaskHandle_t xIdleHandle = NULL;
  // xTaskCreate(taskIdle, "taskIdle", 1024, NULL, 0, &xIdleHandle);
  // delay(10);
  Serial.println("DMX Light Controller");
  DMX::Initialize(input);
  pinMode(LED_PIN, OUTPUT);

  for (int i = 0; i < CONTROLLER_COUNT; i++)
  {
    pwmControllers[i] = new PwmControl(CONTROLLER_FIRST_ADDRESS + i, CONTROLLER_FREQ);
    pwmControllers[i]->begin();
  }

  TaskHandle_t xDMXHandle = NULL;
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
                 Serial.println("Start updating " + type); })
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
                   Serial.println("End Failed"); });

  ArduinoOTA.begin();

  setup_routing();
  // initialize PWM controllers

  TaskHandle_t xNetHandle = NULL;
  xTaskCreate(taskProcessNetwork, "taskProcessNetwork", 2048, NULL, 1, &xNetHandle);
}

void loop()
{
  delay(1000);
  // checkButton();
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
  uint8_t counter;
  const uint8_t print_every = 5;
  for (;;)
  {
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    if (counter % print_every == 0)
      Serial.print(millis());
    if (DMX::IsHealthy())
    {
      if (counter % print_every == 0)
        Serial.print(": ok - ");
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      if (counter % print_every == 0)
        Serial.print(": fail - ");
      digitalWrite(LED_PIN, LOW);
    }
    DMX::ReadAll(chanVal, 220, sizeof(chanVal) - 1);
    for (int channel = 0; channel < sizeof(chanVal) - 1; channel += 4)
    {
      uint8_t controllerNum = channel / CHANNELS_PER_CONTROLLER;
      pwmControllers[controllerNum]->SetColor(channel - (controllerNum * CHANNELS_PER_CONTROLLER), chanVal[channel], chanVal[channel + 1], chanVal[channel + 2], chanVal[channel + 3]);
    }
    if (counter % print_every == 0)
      Serial.printf("%s\n", chanVal);
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