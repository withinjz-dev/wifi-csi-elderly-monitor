#ifndef ESP32_CSI_EDGE_MQTT_H
#define ESP32_CSI_EDGE_MQTT_H

/*
 * Remote alert transport: publishes zone/alert JSON to an MQTT broker so a
 * server-side subscriber can forward to FCM -> the guardian's phone.
 *
 * This node still needs a route to the broker. The RX board's WiFi is
 * currently the SoftAP serving the TX link (closed network, no internet).
 * ESP-IDF supports AP+STA concurrent mode on one ESP32 (two interfaces, one
 * radio, time-shared) -- call esp_wifi_set_mode(WIFI_MODE_APSTA) instead of
 * WIFI_MODE_AP, keep the existing AP config for the TX link, and add
 * esp_wifi_connect() against the home router as STA. Channel conflict risk:
 * AP and STA share one channel in APSTA mode, so the SoftAP's channel gets
 * forced to match whatever channel the home router is on when STA connects.
 * If the TX is hardcoded to the AP's original channel this can silently break
 * the TX-RX link -- either fix the TX to follow the AP's advertised channel,
 * or pin the home router to the same channel the AP already uses. Verify this
 * on hardware before trusting it; it is not yet tested in this project.
 *
 * Not yet wired into edge_monitor.c -- call edge_mqtt_init() once at startup
 * (after WiFi STA is connected) and edge_mqtt_publish_state() from the
 * decision task wherever it currently only sets the local alert level.
 */

#include "mqtt_client.h"
#include "esp_log.h"
#include <stdio.h>

#define EDGE_MQTT_URI        "mqtt://BROKER_HOST:1883"   /* set at deploy time, TLS (mqtts://) recommended */
#define EDGE_MQTT_TOPIC_FMT  "elder-monitor/%s/state"     /* %s = zone or device name */

static const char *EDGE_MQTT_TAG = "edge_mqtt";
static esp_mqtt_client_handle_t edge_mqtt_client = NULL;

static void edge_mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                     int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(EDGE_MQTT_TAG, "connected to broker");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(EDGE_MQTT_TAG, "disconnected -- client auto-reconnects, "
                     "publishes made while down are dropped, not queued");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(EDGE_MQTT_TAG, "mqtt error");
            break;
        default:
            break;
    }
}

static inline void edge_mqtt_init(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = EDGE_MQTT_URI,
    };
    edge_mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(edge_mqtt_client, ESP_EVENT_ANY_ID,
                                    edge_mqtt_event_handler, NULL);
    esp_mqtt_client_start(edge_mqtt_client);
}

/*
 * Fire-and-forget publish (QoS 1, not retained). If the link is down this
 * silently drops -- there is no outbox/retry here yet. That is the P1 gap
 * called out in the roadmap ("network-drop recovery demo"): a proper version
 * needs a small ring buffer of unsent alerts flushed on MQTT_EVENT_CONNECTED,
 * otherwise an emergency that happens during a WiFi outage is lost entirely.
 */
static inline void edge_mqtt_publish_state(const char *zone_name,
                                           const char *state_name,
                                           const char *alert_level_name) {
    if (!edge_mqtt_client) return;
    char topic[64];
    snprintf(topic, sizeof(topic), EDGE_MQTT_TOPIC_FMT, zone_name);
    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"zone\":\"%s\",\"state\":\"%s\",\"alert\":\"%s\"}",
             zone_name, state_name, alert_level_name);
    esp_mqtt_client_publish(edge_mqtt_client, topic, payload, 0, 1, 0);
}

#endif /* ESP32_CSI_EDGE_MQTT_H */
