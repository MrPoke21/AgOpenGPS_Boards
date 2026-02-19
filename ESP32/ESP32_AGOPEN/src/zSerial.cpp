#include "zSerial.h"
#include <freertos/queue.h>

// Serial send queue
QueueHandle_t serialSendQueue = NULL;

// ===== SERIAL QUEUE INITIALIZATION =====
bool initSerialQueue() {
  DEBUG_PRINTLN("[Serial] Initializing Serial queue...");
  
  // Create queue for async Serial sending (30 packets max)
  serialSendQueue = xQueueCreate(30, sizeof(SerialPacket));
  if (serialSendQueue == NULL) {
    DEBUG_PRINTLN("[Serial] Failed to create Serial send queue!");
    return false;
  }
  
  // Create background task for Serial sending (Core 1, high priority)
  xTaskCreatePinnedToCore(
    serialSendTask,
    "serialSend",
    2048,
    NULL,
    10,  // High priority for low latency
    NULL,
    1  // Core 1
  );
  
  DEBUG_PRINTLN("[Serial] Serial queue initialized successfully");
  return true;
}

// ===== SEND DATA VIA SERIAL (NON-BLOCKING QUEUE) =====
bool sendSerial(const uint8_t* data, uint16_t length) {
  if (serialSendQueue == NULL) {
    return false;
  }
  
  // Check buffer size
  if (length > 256) {
    return false;
  }
  
  // Queue packet for async sending (non-blocking)
  SerialPacket packet;
  memcpy(packet.data, data, length);
  packet.length = length;
  
  return xQueueSend(serialSendQueue, &packet, 0) == pdTRUE;
}

// ===== SERIAL SEND TASK (Background) =====
void serialSendTask(void* params) {
  SerialPacket packet;
  
  while (1) {
    // Wait for packet in queue (1000ms timeout)
    if (xQueueReceive(serialSendQueue, &packet, pdMS_TO_TICKS(1000))) {
      Serial.write(packet.data, packet.length);
      Serial.flush();
      vTaskDelay(pdMS_TO_TICKS(1));  // Yield to OS
    }
  }
  vTaskDelete(NULL);
}
