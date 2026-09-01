#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <esp_http_server.h>

char *data = (char *) "1\n";

/*
 * Broadcast, not unicast, to 192.168.4.1 (the AP). Unicast data frames go out
 * addressed to one MAC and an unassociated listener's radio drops them before
 * CSI is ever extracted -- confirmed on hardware: a passive/promiscuous 2nd
 * zone board sitting right next to the TX/RX pair saw its own CSI callback
 * fire ~1 time total in 20s while a raw promiscuous capture on the same radio
 * was clearly receiving traffic. Broadcast frames (dest ff:ff:ff:ff:ff:ff, the
 * same class of frame a beacon is) don't have that per-station gate, which is
 * exactly why the original passive/ example -- built around overhearing
 * beacons -- worked at all. The AP-associated RX still receives this fine;
 * broadcast doesn't require a subscriber.
 */
void socket_transmitter_sta_loop(bool (*is_wifi_connected)()) {
    int socket_fd = -1;
    while (1) {
        close(socket_fd);
        char *ip = (char *) "192.168.4.255";
        struct sockaddr_in caddr;
        caddr.sin_family = AF_INET;
        caddr.sin_port = htons(2223);
        while (!is_wifi_connected()) {
            // wait until connected to AP
            printf("wifi not connected. waiting...\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        printf("initial wifi connection established.\n");
        if (inet_aton(ip, &caddr.sin_addr) == 0) {
            printf("ERROR: inet_aton\n");
            continue;
        }

        socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
        if (socket_fd == -1) {
            printf("ERROR: Socket creation error [%s]\n", strerror(errno));
            continue;
        }
        int broadcast_enable = 1;
        if (setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) == -1) {
            printf("ERROR: setsockopt SO_BROADCAST error [%s]\n", strerror(errno));
            continue;
        }
        if (connect(socket_fd, (const struct sockaddr *) &caddr, sizeof(struct sockaddr)) == -1) {
            printf("ERROR: socket connection error [%s]\n", strerror(errno));
            continue;
        }

        printf("sending frames.\n");
        double lag = 0.0;
        while (1) {
            double start_time = get_steady_clock_timestamp();
            if (!is_wifi_connected()) {
                printf("ERROR: wifi is not connected\n");
                break;
            }

            if (sendto(socket_fd, &data, strlen(data), 0, (const struct sockaddr *) &caddr, sizeof(caddr)) !=
                strlen(data)) {
                vTaskDelay(1);
                continue;
            }

#if defined CONFIG_PACKET_RATE && (CONFIG_PACKET_RATE > 0)
            double wait_duration = (1000.0 / CONFIG_PACKET_RATE) - lag;
            int w = floor(wait_duration);
            vTaskDelay(w);
#else
            vTaskDelay(10); // This limits TX to approximately 100 per second.
#endif
            double end_time = get_steady_clock_timestamp();
            lag = end_time - start_time;
        }
    }
}
