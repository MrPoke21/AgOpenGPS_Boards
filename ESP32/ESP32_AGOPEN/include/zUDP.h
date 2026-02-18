#ifndef ZUDP_H
#define ZUDP_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include "Configuration.h"

// UDP packet queue structure
struct UDPPacket {
  uint8_t data[256];
  uint16_t length;
};

// WiFi status enumeration
enum WiFiStatus {
  WIFI_INIT,
  WIFI_AP_READY,
  WIFI_STA_CONNECTING,
  WIFI_STA_CONNECTED,
  WIFI_ERROR
};

// ===== FUNCTION DECLARATIONS =====

/**
 * Initialize WiFi and UDP
 * @return true if successful
 */
bool initWiFi();

/**
 * Initialize UDP socket
 * @return true if successful
 */
bool initUDP();

/*** Send data via UDP (non-blocking queue-based)
 * @param data: pointer to data buffer
 * @param length: data length
 * @return true if queued successfully
 */
bool sendUDP(const uint8_t* data, uint16_t length);

/**
 * UDP send task - processes queued packets (run in separate task)
 */
void udpSendTask(void* params);

/**
 * Send data to Serial or UDP (or both) depending on configuration
 * @param data: pointer to data buffer
 * @param length: data length
 */
void sendData(const uint8_t* data, uint16_t length);

/**
 * Receive UDP data (non-blocking)
 * @param buffer: pointer to receive buffer
 * @param maxLen: max buffer length
 * @return number of bytes received (0 if none)
 */
uint16_t receiveUDP(uint8_t* buffer, uint16_t maxLen);

/**
 * Handle received UDP data
 */
void handleUDPData();

/**
 * Get WiFi status
 * @return WiFi status enum
 */
WiFiStatus getWiFiStatus();

/**
 * Get connected client count (for AP mode)
 * @return number of connected clients
 */
uint8_t getWiFiClientCount();

/**
 * Print WiFi status to Serial
 */
void printWiFiStatus();

extern WiFiStatus wifiStatus;
extern AsyncUDP udp;
extern IPAddress udpRemoteIP;
extern uint16_t udpRemotePort;

#endif
