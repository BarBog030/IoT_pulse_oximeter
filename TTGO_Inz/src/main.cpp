#include <Arduino.h>
#include "Display.h"
#include "MAX30102.h"
#include "Network.h"
#include "Pangodream_18650_CL.h"

Display* display;
void Display_Init() {
  display = new Display();
  display->initTFT();
  display->centerMsg("Place finger on sensor");
}

Network *network;
void Network_Init() {
  network = new Network();
  network->taskNetwork(display);
}

#define INT_PIN  2 //Connect INT pin on MC to pin 2

// max30102 sensor
MAX30102 sensor;

// Timer variables (send new readings every 5 seconds)
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelayData = 5000;

void setup() {
  Serial.begin(115200);
  Serial.println("Sensor communication init");

  // Initialize sensor communication
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }

  // Initialize display
  Display_Init();
  
  // Initialize Network
  Network_Init();
  delay(2000);

  // Initialize sensor registers
  sensor.MAX30102_Init();

  // Initialize interrupts
  pinMode(INT_PIN, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(INT_PIN) == LOW) //Hardware way of reading interrupts
  {
    sensor.Max30102_InterruptCallback();
  }

  sensor.Max30102_Task(display);
  if (sensor.CheckIfCalculationsDone())
  {
    // Send new readings to database
    if (WiFi.status() == WL_CONNECTED && network->firebaseReady() && (millis() - sendDataPrevMillis > timerDelayData) || sendDataPrevMillis == 0) {
      sendDataPrevMillis = millis();
          
      // Send last readings to database:
      network->sendInt(network->getHrPath(), sensor.Max30102_GetHeartRate());
      network->sendInt(network->getSpo2Path(), sensor.Max30102_GetSpO2Value());
    }
  }
}