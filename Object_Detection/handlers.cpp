#include <WebServer.h>
#include "WifiCam.hpp"

extern WebServer server;

void serveVideo()
{
  esp32cam::Camera.streamMjpeg(server.client());
}

void serveJpeg()
{
  auto frame = esp32cam::capture();
  if (frame == nullptr)
  {
    Serial.println("capture() failure");
    server.send(500, "text/plain", "still capture error\n");
    return;
  }

  Serial.printf("JPEG captured: %dx%d %zub\n", frame->getWidth(), frame->getHeight(), frame->size());

  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  frame->writeTo(server.client());
}

void addRequestHandlers()
{
  server.on("/", HTTP_GET, []()
            {
    server.sendHeader("Content-Type", "text/html; charset=utf-8");
    server.send(200, "text/html", R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head><meta charset="utf-8"><title>ESP32 TTS</title></head>
      <body>
        <h2>ESP32 :)</h2>
      </body>
      </html>
    )rawliteral"); });
  server.on("/video", HTTP_GET, serveVideo);
  server.on("/jpeg", HTTP_GET, serveJpeg);
}