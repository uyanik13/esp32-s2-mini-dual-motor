#include "WebSocketHandler.h"
#include "MotorControl.h"
#include <ArduinoJson.h>
#include <Arduino.h>

extern MotorControl motorControl; // Reference to the motor control instance

AsyncWebSocket ws("/ws");

void WebSocketHandler::setupWebSocket(AsyncWebServer* server) {
    ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        handleWebSocketEvent(server, client, type, arg, data, len);
    });
    server->addHandler(&ws);
}

void WebSocketHandler::handleWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        String msg = "";
        for (size_t i = 0; i < len; i++) {
            msg += (char)data[i];
        }
        Serial.printf("Received message from client #%u: %s\n", client->id(), msg.c_str());
        serializeAndSendToMotorControl(msg);  // Serialize and send data to MotorControl
    } else if (type == WS_EVT_ERROR) {
        Serial.printf("WebSocket error for client #%u\n", client->id());
    } else if (type == WS_EVT_PONG) {
        Serial.printf("WebSocket Pong from client #%u\n", client->id());
    }
}

void WebSocketHandler::serializeAndSendToMotorControl(String msg) {
    // Parse the JSON message
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, msg);

    if (error) {
        Serial.printf("JSON deserialization failed: %s\n", error.f_str());
        motorControl.stopAll();
        return;
    }

    // Extract the values from the JSON message
    const char* direction = doc["direction"];
    int speed = doc["speed"];
    float angle = doc["angle"];

    // Log the parsed data
    Serial.printf("Serialized data - Direction: %s, Speed: %d, Angle: %.1f\n", direction, speed, angle);

    // Send the data to MotorControl
    motorControl.handleWebSocketInput(String(direction), speed, angle);
}

void WebSocketHandler::cleanupClients() {
    ws.cleanupClients();
}
