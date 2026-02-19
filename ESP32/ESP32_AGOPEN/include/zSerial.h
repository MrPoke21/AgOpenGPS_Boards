#ifndef ZSERIAL_H
#define ZSERIAL_H

#include <Arduino.h>
#include "Configuration.h"

// Serial packet queue structure
struct SerialPacket {
  uint8_t data[256];
  uint16_t length;
};

// ===== FUNCTION DECLARATIONS =====

/**
 * Initialize Serial Queue (non-blocking serial writes)
 * @return true if successful
 */
bool initSerialQueue();

/**
 * Send data via Serial (non-blocking queue-based)
 * @param data: pointer to data buffer
 * @param length: data length
 * @return true if queued successfully
 */
bool sendSerial(const uint8_t* data, uint16_t length);

/**
 * Serial send task - processes queued packets (run in separate task)
 */
void serialSendTask(void* params);

#endif
