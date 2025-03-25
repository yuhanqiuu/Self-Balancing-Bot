#include "WifiCam.hpp"
#include <WiFi.h>

static const char *WIFI_SSID = "Hotpot";
static const char *WIFI_PASS = "hahahaha";

esp32cam::Resolution initialResolution;

WebServer server(80);

void setup()
{
    Serial.begin(115200);
    Serial.println();
    esp32cam::setLogger(Serial);
    delay(1000);

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
    delay(1000);

    {
        using namespace esp32cam;

        initialResolution = Resolution::find(320, 240);

        Config cfg;
        cfg.setPins(pins::AiThinker);
        cfg.setResolution(initialResolution);
        cfg.setJpeg(80);

        bool ok = Camera.begin(cfg);
        if (!ok)
        {
            Serial.println("camera initialize failure");
            delay(5000);
            ESP.restart();
        }
        Serial.println("camera initialize success");
    }

    Serial.println("camera starting");
    Serial.print("http://");
    Serial.println(WiFi.localIP());
    
    addRequestHandlers();
    server.begin();
}

void loop()
{
    server.handleClient();
}