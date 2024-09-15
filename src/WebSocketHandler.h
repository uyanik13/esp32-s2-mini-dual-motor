#ifndef WEBSOCKETHANDLER_H
#define WEBSOCKETHANDLER_H

#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

class WebSocketHandler {
public:
    void setupWebSocket(AsyncWebServer* server);
    void handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
    void cleanupClients();

private:
    void serializeAndSendToMotorControl(String msg);  // Serializes and sends data to MotorControl
};

#endif
