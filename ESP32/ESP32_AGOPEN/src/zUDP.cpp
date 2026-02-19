#include "zUDP.h"
#include "zSerial.h"
#include "Configuration.h"
#include <esp_wifi.h>

// ===== GLOBAL VARIABLES =====
WiFiStatus wifiStatus = WIFI_INIT;
AsyncUDP udp;
IPAddress udpRemoteIP;
uint16_t udpRemotePort = 0;

// UDP receive buffer
volatile uint8_t udpRxBuffer[UDP_BUFFER_SIZE];
volatile uint16_t udpRxLen = 0;
volatile bool udpDataAvailable = false;

// UDP send queue
QueueHandle_t udpSendQueue = NULL;

// ===== WiFi INITIALIZATION =====
bool initWiFi() {
  DEBUG_PRINTLN("\n[UDP] Initializing WiFi...");
  DEBUG_PRINTF("[UDP] WiFi Buffer Config: RX=%d, TX=%d (optimized for low latency)\n",
               WIFI_RX_BUF_COUNT, WIFI_TX_BUF_COUNT);
  
  // Note: RX/TX buffer tuning requires custom esp_wifi_init_config
  // For now, rely on AsyncUDP library optimizations and disable power save
  
  // Disable power save for lower latency
#if WIFI_DISABLE_PS == 1
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power save at ESP-IDF level
  DEBUG_PRINTLN("[UDP] WiFi Power Save: DISABLED (all levels)");
#else
  WiFi.setSleep(WIFI_PS_MIN_MODEM);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  DEBUG_PRINTLN("[UDP] WiFi Power Save: MIN_MODEM");
#endif
  
#if WIFI_MODE == 1
  // AP Mode - create access point (lower latency)
  DEBUG_PRINTLN("[UDP] Starting WiFi Access Point (Lower Latency)");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  
  // Set TX power to maximum
  WiFi.setTxPower((wifi_power_t)WIFI_TX_POWER);
  
  // Set channel 6 (center of non-overlapping channels)
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  
  // Optimize WiFi bandwidth and buffer for low latency
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);  // 40MHz bandwidth
  DEBUG_PRINTLN("[UDP] WiFi Bandwidth: 40MHz (HT40)");
  
  IPAddress apIP(192, 168, 4, 1);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  
  DEBUG_PRINT("[UDP] AP IP: ");
  DEBUG_PRINTLN(WiFi.softAPIP());
  DEBUG_PRINTLN("[UDP] AP Mode: Recommended for lowest latency");
  
  wifiStatus = WIFI_AP_READY;
  return true;
  
#else
  // STA Mode - connect to existing network (higher latency)
  DEBUG_PRINTLN("[UDP] Starting WiFi Station Mode (Higher Latency ~30-50ms)");
  WiFi.mode(WIFI_STA);
  
  // Set TX power to maximum
  WiFi.setTxPower((wifi_power_t)WIFI_TX_POWER);
  
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);  // 40MHz bandwidth
  DEBUG_PRINTLN("[UDP] WiFi Bandwidth: 40MHz (HT40)");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Reapply power save settings after connection attempt
#if WIFI_DISABLE_PS == 1
  esp_wifi_set_ps(WIFI_PS_NONE);
#else
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
#endif
  
  uint8_t attempts = 0;
  wifiStatus = WIFI_STA_CONNECTING;
  
  while (WiFi.status() != WL_CONNECTED && attempts++ < 20) {
    delay(500);
    DEBUG_PRINT(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    DEBUG_PRINTLN("\n[UDP] WiFi Connected!");
    DEBUG_PRINT("[UDP] IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
    wifiStatus = WIFI_STA_CONNECTED;
    return true;
  } else {
    DEBUG_PRINTLN("\n[UDP] WiFi Connection Failed!");
    wifiStatus = WIFI_ERROR;
    return false;
  }
#endif
}

// ===== UDP INITIALIZATION =====
bool initUDP() {
  DEBUG_PRINTLN("[UDP] Initializing UDP socket...");
  
  // Create queue for async UDP sending (30 packets max for better buffering)
  udpSendQueue = xQueueCreate(30, sizeof(UDPPacket));
  if (udpSendQueue == NULL) {
    DEBUG_PRINTLN("[UDP] Failed to create UDP send queue!");
    return false;
  }
  
  // Create background task for UDP sending (Core 0, high priority)
  xTaskCreatePinnedToCore(
    udpSendTask,
    "udpSend",
    2048,
    NULL,
    10,  // Increased priority from 1 to 10 (higher = more priority)
    NULL,
    0  // Core 0 (WiFi stack uses Core 0)
  );
  
  if (udp.listen(UDP_PORT)) {
    DEBUG_PRINTF("[UDP] Listening on port %d\n", UDP_PORT);
    
    // Register UDP receive callback
    udp.onPacket([](AsyncUDPPacket packet) {
      uint16_t len = packet.length();
      
      // Only process non-empty packets
      if (len == 0 || len > UDP_BUFFER_SIZE) {
        return;
      }
      
      memcpy((void*)udpRxBuffer, packet.data(), len);
      udpRxLen = len;
      udpDataAvailable = true;
      
      // Save sender info for responses (first connection only)
      if (udpRemotePort == 0) {
        udpRemoteIP = packet.remoteIP();
        udpRemotePort = packet.remotePort();
      }
      
      DEBUG_PRINTF("[UDP] Received %d bytes from %s:%d\n", 
                    len, udpRemoteIP.toString().c_str(), udpRemotePort);
    });
    
    return true;
  } else {
    DEBUG_PRINTLN("[UDP] Failed to listen on UDP port!");
    return false;
  }
}

// ===== SEND DATA VIA UDP (NON-BLOCKING QUEUE) =====
bool sendUDP(const uint8_t* data, uint16_t length) {
  if (wifiStatus == WIFI_ERROR || udpSendQueue == NULL) {
    return false;
  }
  
  // Check buffer size
  if (length > 256) {
    DEBUG_PRINTF("[UDP] Data too large: %d bytes\n", length);
    return false;
  }
  
  // No client connected yet
  if (udpRemotePort == 0) {
    return false;
  }
  
  // Queue packet for async sending (non-blocking)
  UDPPacket packet;
  memcpy(packet.data, data, length);
  packet.length = length;
  
  return xQueueSend(udpSendQueue, &packet, 0) == pdTRUE;
}

// ===== RECEIVE UDP DATA ======
uint16_t receiveUDP(uint8_t* buffer, uint16_t maxLen) {
  if (!udpDataAvailable || udpRxLen == 0) {
    return 0;
  }
  
  uint16_t copyLen = (udpRxLen > maxLen) ? maxLen : udpRxLen;
  memcpy(buffer, (const void*)udpRxBuffer, copyLen);
  
  udpDataAvailable = false;
  udpRxLen = 0;
  
  return copyLen;
}

// ===== HANDLE UDP DATA =====
void handleUDPData() {
  if (!udpDataAvailable) {
    return;
  }
  
  DEBUG_PRINTF("[UDP] Received %d bytes\n", udpRxLen);
  // Data will be processed by autoSteerPacketPerser() via receiveUDP()
}

// ===== GET WIFI STATUS =====
WiFiStatus getWiFiStatus() {
  return wifiStatus;
}

// ===== GET CONNECTED CLIENT COUNT =====
uint8_t getWiFiClientCount() {
#if WIFI_MODE == 1
  return WiFi.softAPgetStationNum();
#else
  return (WiFi.status() == WL_CONNECTED) ? 1 : 0;
#endif
}

// ===== PRINT WIFI STATUS =====
void printWiFiStatus() {
  DEBUG_PRINTLN("\n===== WiFi Status =====");
  
  switch (wifiStatus) {
    case WIFI_INIT:
      DEBUG_PRINTLN("Status: Initializing");
      break;
    case WIFI_AP_READY:
      DEBUG_PRINTLN("Status: AP Ready");
      DEBUG_PRINT("SSID: ");
      DEBUG_PRINTLN(WIFI_SSID);
      DEBUG_PRINT("IP: ");
      DEBUG_PRINTLN(WiFi.softAPIP());
      DEBUG_PRINT("Clients: ");
      DEBUG_PRINTLN(getWiFiClientCount());
      break;
    case WIFI_STA_CONNECTING:
      DEBUG_PRINTLN("Status: Connecting to network");
      break;
    case WIFI_STA_CONNECTED:
      DEBUG_PRINTLN("Status: Connected");
      DEBUG_PRINT("IP: ");
      DEBUG_PRINTLN(WiFi.localIP());
      break;
    case WIFI_ERROR:
      DEBUG_PRINTLN("Status: ERROR");
      break;
  }
  
  DEBUG_PRINTF("UDP Port: %d\n", UDP_PORT);
  DEBUG_PRINTLN("=======================\n");
}

// ===== UDP SEND TASK (Background) =====
void udpSendTask(void* params) {
  UDPPacket packet;
  
  while (1) {
    // Wait for packet in queue (1000ms timeout)
    if (xQueueReceive(udpSendQueue, &packet, pdMS_TO_TICKS(1000))) {
      // Only send if we have a connected client
      if (udpRemotePort != 0 && wifiStatus != WIFI_ERROR) {
        bool success = udp.writeTo(packet.data, packet.length, udpRemoteIP, 9999);
        if (!success) {
          DEBUG_PRINTF("[UDP] ERROR: Failed to send %d bytes to %s:%d\n", 
                       packet.length, udpRemoteIP.toString().c_str(), udpRemotePort);
        }
      }
    }
  }
  vTaskDelete(NULL);
}
