/* Zigbee alert helper - sends a debounced On/Off Toggle broadcast
 * This header exposes a simple helper so sensor code can notify the
 * Zigbee network when an event (impact or ADC threshold) occurs.
 */
#pragma once

#include <stdint.h>

/**
 * Send a debounced Zigbee On/Off Toggle as a broadcast to RX-on-when-idle devices.
 * @param debounce_ms minimum milliseconds between sends
 */
void send_zigbee_alert_toggle_debounced(uint32_t debounce_ms);
