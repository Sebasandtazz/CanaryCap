/* Implementation of Zigbee alert helper.
 * Builds and sends a ZCL On/Off Toggle command as a broadcast (0xFFFD).
 * The function is safe to call from application tasks and internally
 * uses esp_zb_lock to protect stack calls.
 */
#include "zb_alert.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_command.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "zb_alert";

void send_zigbee_alert_toggle_debounced(uint32_t debounce_ms)
{
    static int64_t last_sent_us = 0;
    int64_t now_us = esp_timer_get_time();
    if ((now_us - last_sent_us) < (int64_t)debounce_ms * 1000LL) {
        return; // too soon
    }

    esp_zb_zcl_on_off_cmd_t cmd_req;
    memset(&cmd_req, 0, sizeof(cmd_req));

    /* Source endpoint on this device (switch endpoint created in esp_zb_task) */
    cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;

    /* Destination: broadcast to all RX-on-when-idle devices (0xFFFD) */
    cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0xFFFD;
    /* 0xFF instructs APS to send to the broadcast endpoint */
    cmd_req.zcl_basic_cmd.dst_endpoint = 0xFF;

    /* Use 16-bit address + endpoint present addressing */
    cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t rc = esp_zb_zcl_on_off_cmd_req(&cmd_req);
    esp_zb_lock_release();

    if (rc == ESP_OK) {
        ESP_LOGI(TAG, "Zigbee alert sent (on_off toggle broadcast)");
        last_sent_us = now_us;
    } else {
        ESP_LOGW(TAG, "Failed to send Zigbee alert: %s", esp_err_to_name(rc));
    }
}
