#ifndef ESP32_CSI_INPUT_COMPONENT_H
#define ESP32_CSI_INPUT_COMPONENT_H

#include <string.h>
#include "csi_component.h"
#include "alert_component.h"
/* Included directly rather than guarded on include order: main.cc pulls in
 * input_component.h before edge_monitor.h, so an #ifdef would silently
 * compile the REGISTER command out. */
#include "edge_presence.h"

char input_buffer[256];
int input_buffer_pointer = 0;

void _handle_input() {
    if (match_set_timestamp_template(input_buffer)) {
        printf("Setting local time to %s\n", input_buffer);
        time_set(input_buffer);
    } else if (strcmp(input_buffer, "ALERT_ON") == 0) {
        alert_set(true);
        printf("ALERT_ON\n");
    } else if (strcmp(input_buffer, "ALERT_OFF") == 0) {
        alert_set(false);
        printf("ALERT_OFF\n");
    } else if (strcmp(input_buffer, "TEST_LED") == 0) {
        alert_set_level(ALERT_FAULT);  // LED only, no buzzer -- bench test
        printf("TEST_LED\n");
    } else if (strcmp(input_buffer, "REGISTER") == 0) {
        // Install-time: bind the resident's phone. Reconnect it within the window.
        edge_presence_begin_registration();
        printf("REGISTER\n");
    } else if (strncmp(input_buffer, "REGISTER ", 9) == 0) {
        // REGISTER AA:BB:CC:DD:EE:FF -- bind explicitly when the window above
        // reported several devices and could not choose.
        uint8_t mac[6];
        if (edge_presence_parse_mac(input_buffer + 9, mac)) {
            edge_presence_register_mac(mac);
            printf("REGISTER OK\n");
        } else {
            printf("REGISTER: bad MAC '%s'\n", input_buffer + 9);
        }
    } else {
        printf("Unable to handle input %s\n", input_buffer);
    }
}

void input_check() {
    uint8_t ch = fgetc(stdin);

    while (ch != 0xFF) {
        if (ch == '\n') {
            _handle_input();
            input_buffer[0] = '\0';
            input_buffer_pointer = 0;
        } else {
            input_buffer[input_buffer_pointer] = ch;
            input_buffer[input_buffer_pointer + 1] = '\0';
            input_buffer_pointer++;
        }

        ch = fgetc(stdin);
    }
}

void input_loop() {
    edge_presence_button_init();
    while (true) {
        input_check();
        edge_presence_button_poll();   // BOOT button = registration trigger
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

#endif //ESP32_CSI_INPUT_COMPONENT_H
