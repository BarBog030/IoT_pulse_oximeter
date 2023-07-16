#include "Display.h"

#define BATTERY_ICON_WIDTH 70
#define BATTERY_ICON_HEIGHT 36
#define BATTERY_STATUS_HEIGHT_BAR BATTERY_ICON_HEIGHT
#define BATTERY_ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define BATTERY_ICON_POS_X 0

static Display *instance = NULL; // aby moc przezkazac instancje do tft_output ktora nie jest z klasy Display

Display::Display() {
  instance = this;
  tft = new TFT_eSPI();
}

Display::~Display() {
  delete tft;
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t * bitmap){
  if (y >= instance->tft->height()) return 0;
  instance->tft->pushImage(x, y, w, h, bitmap);
  return 1;
}

void Display::initTFT() {
  tft->init();
  tft->fillScreen(TFT_BLACK);
  tft->setRotation(1);
  tft->setTextColor(TFT_WHITE, TFT_BLACK);
  tft->setTextDatum(MC_DATUM);
  tft->setFreeFont(&FreeSans12pt7b);

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  if(!SPIFFS.begin()){
    while(1) yield();
  }
}

void Display::clearScreen() {
  tft->fillRect(0, 40, tft->width(), tft->height() - 40, TFT_BLACK);
}

void Display::centerMsg(String text) {
  tft->drawString(text, tft->width() / 2, 60);
}

void Display::valueUpdatesInit(String hrInitText, String spo2InitText){
  tft->drawString(hrInitText, tft->width() / 2, 60);
  tft->drawString(spo2InitText, tft->width() / 2, 90);
}

void Display::valueUpdates(String heartRate, String spo2) {
  tft->drawString(heartRate, tft->width() / 2, 60);
  tft->drawString(spo2, tft->width() / 2, 90);
}

void Display::showWiFiIcon(bool isOn){
  tft->fillRect(tft->width() -30, 0, 30, 30, TFT_BLACK);
  TJpgDec.drawFsJpg(tft->width() -30, 0, isOn ? "/icon_wifi_on.jpg" : "/icon_wifi_off.jpg");
}

void Display::showFirebaseIcon(bool isOn){
  tft->fillRect(tft->width() -60, 0, 30, 30, TFT_BLACK);
  TJpgDec.drawFsJpg(tft->width() -60, 0, isOn ? "/icon_firebase_on.jpg" : "/icon_firebase_off.jpg");
}

void Display::drawBatteryIcon(String filePath){
  tft->fillRect(0, 0, BATTERY_ICON_WIDTH, BATTERY_ICON_HEIGHT, TFT_BLACK);
  TJpgDec.drawFsJpg(BATTERY_ICON_POS_X, 0, filePath);
}

void Display::drawBatteryText(String text){
  tft->fillRect(BATTERY_ICON_WIDTH, 0, BATTERY_ICON_WIDTH + 15, BATTERY_ICON_HEIGHT, TFT_BLACK);
  tft->drawString(text, BATTERY_ICON_WIDTH + 25, BATTERY_STATUS_HEIGHT_BAR/2, 4);
}