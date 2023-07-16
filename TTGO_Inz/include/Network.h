#pragma once

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "Display.h"
#include <Pangodream_18650_CL.h>

class Network {
private:
    friend void keepWiFiAlive(void *parameters);
public:
    Network();
    String getHrPath();
    String getSpo2Path();
    void taskNetwork(Display *display);
    void taskFirebase(Display *display);
    void initFirebase();
    bool firebaseReady();
    void sendInt(String path, int32_t value);
};

