#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "../../_components/nvs_component.h"
#include "../../_components/sd_component.h"
#include "../../_components/alert_component.h"
#include "../../_components/input_component.h"
#include "../../_components/edge_monitor.h"

/*
 * Second-zone RX, passive/promiscuous mode.
 *
 * The bedroom RX (active_ap) holds a real AP-STA association with the TX --
 * a single STA can't join two APs, so a second zone can't be a second AP the
 * same TX also associates with. Instead this board never associates at all:
 * it puts the radio in promiscuous mode on the bedroom link's channel and
 * extracts CSI from data frames it overhears, exactly the way ESP32-CSI-Tool's
 * own `passive/` example does. Everything downstream (edge_csi.h's callback,
 * ring buffer, DSP, state machine) is unchanged -- CSI extraction only needs
 * esp_wifi_set_csi_rx_cb() to be registered, and that has never depended on
 * being the AP or STA side of a connection.
 *
 * Known gap, not yet fixed: WIFI_PROMIS_FILTER_MASK_DATA takes every data
 * frame on this channel, not just the bedroom TX's. If another WiFi device
 * shares the channel, its frames get treated as this zone's CSI too. A MAC
 * filter on the TX's address (set in the promiscuous callback, checking
 * info->rx_ctrl against the known TX MAC) is the fix; it needs the TX's MAC
 * read once via idf.py monitor first, which hasn't been done. Fine on an
 * isolated test channel, not fine for the final submission without the
 * filter -- track this before demo day.
 */

#ifdef CONFIG_WIFI_CHANNEL
#define WIFI_CHANNEL CONFIG_WIFI_CHANNEL
#else
#define WIFI_CHANNEL 6   /* must match the bedroom AP's channel (sdkconfig: CONFIG_WIFI_CHANNEL=6) */
#endif

static const char *TAG = "passive_zone";

static void passive_wifi_init(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    const wifi_promiscuous_filter_t filt = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA
                     | WIFI_PROMIS_FILTER_MASK_DATA_MPDU
                     | WIFI_PROMIS_FILTER_MASK_DATA_AMPDU
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filt));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "promiscuous CSI sniffing on channel %d", WIFI_CHANNEL);
}

static void vTask_input_loop(void *pvParameters) {
    input_loop();
}

extern "C" void app_main(void) {
    nvs_init();
    sd_init();
    alert_init();
    passive_wifi_init();

    xTaskCreate(&vTask_input_loop, "input_loop", 4096, NULL, 5, NULL);

    edge_monitor_start();
}
