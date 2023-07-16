#pragma once

#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include "SPIFFS.h"

class Display {
private:
  TFT_eSPI* tft;
  friend bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t * bitmap);

public:
  Display();
  ~Display();

  void initTFT();
  void clearScreen(); 
  void centerMsg(String text);
  void valueUpdatesInit(String hrInitText, String spo2InitText);
  void valueUpdates(String heartRate, String spo2);
  void showWiFiIcon(bool isOn);
  void showFirebaseIcon(bool isOn);
  void drawBatteryIcon(String filePath);
  void drawBatteryText(String text);
};