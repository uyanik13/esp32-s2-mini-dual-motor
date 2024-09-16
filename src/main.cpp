#include <WiFi.h>
#include <LittleFS.h>
#include "MotorControl.h"
#include "ServoControl.h"
#include "ObstacleDetection.h"
#include "WebSocketHandler.h"

// Global Instances
MotorControl motorControl;
ServoControl servoControl;
ObstacleDetection obstacleDetection;
WebSocketHandler webSocketHandler;

bool obstacleDetected = false;

AsyncWebServer server(80);

// Modular Function to Setup LittleFS File System
void setupFileSystem() {
    if (!LittleFS.begin()) {
        Serial.println("LittleFS Mount Failed, trying to format...");
        LittleFS.format();
        if (!LittleFS.begin()) {
            Serial.println("LittleFS Mount Failed after formatting");
            return;
        }
    }
    Serial.println("LittleFS Mount Success");
}

// Modular Function to Setup WiFi as Access Point
void setupWiFi() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Ai-Autonomos-car", "588245");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
}

// Modular Function to Setup HTTP Server Routes
void setupServerRoutes() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!LittleFS.exists("/index.html")) {
            request->send(404, "text/plain", "File not found");
            return;
        }
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/style.css", "text/css");
    });

    server.on("/joystick.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/joystick.js", "application/javascript");
    });

    // Emergency Stop Route
    server.on("/emergency_stop", HTTP_GET, [](AsyncWebServerRequest *request) {
        motorControl.stopAll();
        request->send(200, "text/plain", "Emergency stop");
    });
}

// Modular Function for setting up the server and starting it
void setupServer() {
    setupFileSystem();
    setupWiFi();
    setupServerRoutes();

    // Setup WebSocket
    webSocketHandler.setupWebSocket(&server);

    // Start the server
    server.begin();
    Serial.println("Server started");
}

// Main Setup Function
void setup() {
    Serial.begin(115200);

    // Setup motor, servo, and obstacle detection
    motorControl.setup();
    servoControl.setup();
    servoControl.center();
    obstacleDetection.setup();

    // Setup the server
    setupServer();
}

// Main Loop Function
void loop() {
    // Handle WebSocket communication
    webSocketHandler.cleanupClients();
    

    // Continuously check for obstacles
    obstacleDetection.detectObstacle();
    // servoControl.moveLeft();
    // delay(1000); // Wait for a second

    // // Test moving servo right
    // servoControl.moveRight();
    // delay(1000); // Wait for a second

    // // Test moving servo back to center
    // servoControl.center();
    // delay(1000); // Wait for a second
}
