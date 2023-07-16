#include "Network.h"
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define WIFI_SSID "HUAWEI P20"
#define WIFI_PASSWORD "71542100"
#define NETWORK_TIMEOUT_MS 10000

#define API_KEY "AIzaSyCWZ-rPMcLbYvjAvQyEssgsNEWdQhEEwsA"
#define DATABASE_URL "https://esp32-firebase-b114a-default-rtdb.europe-west1.firebasedatabase.app"

#define USER_EMAIL "przykladowekonto@yahoo.com"
#define USER_PASSWORD "PracaInzMax30102"

// Define Firebase Objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Variable to save user UID
String uid;

// Variables to save database paths
String databasePath;
String hrPath;
String spo2Path;

static Network *networkInstance = NULL;

Network::Network(){
    networkInstance = this;
}

String Network::getHrPath(){
    return hrPath;
}

String Network::getSpo2Path(){
    return spo2Path;
}

Pangodream_18650_CL pngd = Pangodream_18650_CL(DEF_PIN, DEF_CONV_FACTOR, DEF_READS);

typedef enum {
    WIFI_CONNECTED,
    WIFI_DISCONNECTED
} INFOBAR_WIFI_STATUS;

typedef enum {
    FIREBASE_READY,
    FIREBASE_NOT_READY
} INFOBAR_FIREBASE_STATUS;

INFOBAR_WIFI_STATUS wifiStatus = WIFI_DISCONNECTED;
INFOBAR_WIFI_STATUS wifiStatusPrev = WIFI_DISCONNECTED;
INFOBAR_FIREBASE_STATUS firebaseStatus = FIREBASE_NOT_READY;
INFOBAR_FIREBASE_STATUS firebaseStatusPrev = FIREBASE_NOT_READY;

//To use the pointer inside your task, you only need to cast it back from void*
void keepWiFiAlive(void *parameters){
    static_cast<Display*>(parameters)->showWiFiIcon(false);
    static_cast<Display*>(parameters)->showFirebaseIcon(false);
    while(1){
        wifiStatusPrev = wifiStatus;
        firebaseStatusPrev = firebaseStatus;
        pngd.batteryInfo(static_cast<Display*>(parameters));
        if (WiFi.status() == WL_CONNECTED){
            wifiStatus = WIFI_CONNECTED;
            if (Firebase.ready()){
                firebaseStatus = FIREBASE_READY;
            } else {
                firebaseStatus = FIREBASE_NOT_READY;
            }
        } else {
            wifiStatus = WIFI_DISCONNECTED;
            firebaseStatus = FIREBASE_NOT_READY;
        }


        if (wifiStatusPrev == WIFI_DISCONNECTED && wifiStatus == WIFI_CONNECTED){
            static_cast<Display*>(parameters)->showWiFiIcon(true);
            wifiStatusPrev = wifiStatus;
        }
        if (wifiStatusPrev == WIFI_CONNECTED && wifiStatus == WIFI_DISCONNECTED){
            static_cast<Display*>(parameters)->showWiFiIcon(false);
        }
        if (wifiStatus == WIFI_DISCONNECTED){
            Serial.println("Connecting to Wi-Fi");
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }

        if (firebaseStatusPrev == FIREBASE_NOT_READY && firebaseStatus == FIREBASE_READY){
            static_cast<Display*>(parameters)->showFirebaseIcon(true);
        }
        if (firebaseStatusPrev == FIREBASE_READY && firebaseStatus == FIREBASE_NOT_READY){
            static_cast<Display*>(parameters)->showFirebaseIcon(false);
        }
        if (firebaseStatus == FIREBASE_NOT_READY){
            networkInstance->initFirebase();
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
void Network::taskNetwork(Display *display){
    xTaskCreatePinnedToCore(
        keepWiFiAlive,      // Nazwa funkcji która ma być wykonywana w tworzonym zadaniu
        "Keep WiFi alive",  // Opis tworzonego zadania
        10000,              // rozmiar stosu na przechowywanie zmiennych tworzonego zadania w bajtach
        (void *) display,   // parametry przekazywane do funkcji keepWiFiAlive
        1,                  // priorytet zadania
        NULL,               // obsługa zadań (nie wykorzystywane)
        0                   // wybór rdzenia na którym ma być wykonywane zadanie (0 lub 1 dla ESP32)
    );
}

void Network::initFirebase(){
    // Assign the api key (required)
    config.api_key = API_KEY;

    // Assign the user sign in credentials
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    // Assign the RTDB URL (required)
    config.database_url = DATABASE_URL;

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    // Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

    // Assign the maximum retry of token generation
    config.max_token_generation_retry = 5;

    // Initialize the library with the Firebase authen and config
    Firebase.begin(&config, &auth);

    // Getting the user UID might take a few seconds
    Serial.println("Getting User UID");
    while ((auth.token.uid) == "") {
        Serial.print('.');
        delay(1000);
    }
    // Print user UID
    uid = auth.token.uid.c_str();
    Serial.print("User UID: ");
    Serial.println(uid);

    // Update database path
    databasePath = "/UsersData/" + uid;

    // Update database path for sensor readings
    hrPath = databasePath + "/Heart Rate"; // --> UsersData/<user_uid>/Heart Rate
    spo2Path = databasePath + "/SPO2"; // --> UsersData/<user_uid>/SPO2
}

void Network::sendInt(String path, int32_t value){
    if (Firebase.RTDB.setInt(&fbdo, path.c_str(), value)){
        /*
        Serial.print("Writing value: ");
        Serial.print (value);
        Serial.print(" on the following path: ");
        Serial.println(path);
        Serial.println("PASSED");
        Serial.print("PATH: ");
        Serial.println(fbdo.dataPath());
        Serial.print("TYPE: ");
        Serial.println(fbdo.dataType());
        */
    }
    else {
        Serial.println("FAILED");
        Serial.print("REASON: ");
        Serial.println(fbdo.errorReason());
    }
}

bool Network::firebaseReady(){
    if(Firebase.ready()){
        return true;
    }
    return false;
}