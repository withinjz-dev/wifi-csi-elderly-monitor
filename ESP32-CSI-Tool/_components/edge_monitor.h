#ifndef ESP32_CSI_EDGE_MONITOR_H
#define ESP32_CSI_EDGE_MONITOR_H

/*
 * Decision task -- the top of the on-device pipeline.
 *
 *   Raw CSI
 *     -> selected-subcarrier power        (edge_csi.h)
 *     -> 10 Hz time-binned ring buffer    (edge_csi.h)
 *     -> DC / drift removal               (here)
 *     -> 0.2-0.5 Hz IIR band-pass         (edge_dsp.h)
 *     -> Goertzel bank                    (edge_dsp.h)
 *     -> peak frequency + concentration   (edge_dsp.h)
 *     -> frequency consistency + streak   (edge_state.h)
 *     -> LED / buzzer                     (alert_component.h)
 *
 * Nothing above needs a host. The serial link carries a one-line status per
 * cycle purely so a developer can watch what the device already decided.
 */

#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "edge_dsp.h"
#include "edge_csi.h"
#include "edge_baseline.h"
#include "edge_state.h"
#include "edge_presence.h"
#include "alert_component.h"

/* Set per-board at build time, e.g. via -DEDGE_ZONE_NAME='"bedroom"'.
 * Defaults keep the existing single-board build behavior unchanged. */
#ifndef EDGE_ZONE_NAME
#define EDGE_ZONE_NAME "bedroom"
#endif

/* Off by default: no board has the AP+STA-to-internet route wired up yet
 * (see edge_mqtt.h). Flip on with -DEDGE_ENABLE_MQTT=1 once that's verified. */
#ifndef EDGE_ENABLE_MQTT
#define EDGE_ENABLE_MQTT 0
#endif
#if EDGE_ENABLE_MQTT
#include "edge_mqtt.h"
#endif

/* Frames to observe before locking in the subcarrier selection (~1 min). */
#define EDGE_SELECTION_FRAMES   600

/* Persist the baseline at most this often -- NVS has a finite erase budget. */
#define EDGE_NVS_SAVE_EVERY     60   /* decision cycles: 60 x 10 s = 10 min */

static const char *EDGE_MON_TAG = "edge_monitor";

static edge_baseline_t  edge_baseline;
static edge_state_ctx_t edge_state;
static edge_goertzel_bank_t edge_goertzel;

/* One window's worth of scratch, reused every cycle. Static rather than stack:
 * 256 floats is 1 kB and the decision task's stack should not have to carry it. */
static float edge_work[EDGE_WINDOW_N];

static inline int edge_local_hour(void) {
    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    /* Clock never set: epoch-ish year. Report -1 so the state machine treats
     * the time-of-day rule as satisfied rather than silently never alerting. */
    if (tm_now.tm_year < (2020 - 1900)) return -1;
    return tm_now.tm_hour;
}

static void edge_decision_task(void *arg) {
    uint32_t cycle = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(EDGE_DECISION_MS));
        cycle++;

        /* No-op on boards that aren't running AP mode, or without a
         * configured EDGE_PHONE_MAC -- see edge_presence.h. */
        edge_presence_poll_ap_stations();

        /* Lock in the subcarrier choice once enough frames have been scored. */
        if (!edge_pipe.selection_done && edge_pipe.sc_frames >= EDGE_SELECTION_FRAMES) {
            edge_finish_selection(&edge_pipe);
        }

        uint32_t frames = edge_pipe.frames_this_window;
        uint32_t dropped = edge_csi_dropped;
        bool have_data = edge_ring_full(&edge_pipe.ring) && frames > 20;

        float motion_feature = edge_stat_std(&edge_pipe.rssi_stat);
        edge_stat_init(&edge_pipe.rssi_stat);
        edge_pipe.frames_this_window = 0;

        float breath_score = 0.0f;
        float peak_freq    = 0.0f;
        float ac_freq      = 0.0f;
        float ac_strength  = 0.0f;

        if (have_data) {
            edge_ring_linearize(&edge_pipe.ring, edge_work);

            /* Drift removal. The host prototype used scipy's linear detrend
             * after discovering that plain mean subtraction let low-frequency
             * drift leak into the bottom of the breathing band and fabricate a
             * peak in an empty room. Least-squares line fit, same idea, no
             * matrix library. */
            float n  = (float) EDGE_WINDOW_N;
            float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
            for (int i = 0; i < EDGE_WINDOW_N; i++) {
                float xi = (float) i;
                sx  += xi;
                sy  += edge_work[i];
                sxx += xi * xi;
                sxy += xi * edge_work[i];
            }
            float denom = n * sxx - sx * sx;
            float slope = (fabsf(denom) > 1e-6f) ? (n * sxy - sx * sy) / denom : 0.0f;
            float icpt  = (sy - slope * sx) / n;
            for (int i = 0; i < EDGE_WINDOW_N; i++) {
                edge_work[i] -= (slope * (float) i + icpt);
            }

            /* Band-pass. Reset per window so the result depends only on this
             * window's samples -- required for the host/device cross-check. */
            edge_biquad_t bp;
            edge_biquad_init_bandpass(&bp, EDGE_BREATH_LO_HZ, EDGE_BREATH_HI_HZ,
                                      EDGE_FS_HZ);
            for (int i = 0; i < EDGE_WINDOW_N; i++) {
                edge_work[i] = edge_biquad_step(&bp, edge_work[i]);
            }

            edge_spectrum_t spec;
            edge_goertzel_analyze(&edge_goertzel, edge_work, EDGE_WINDOW_N, &spec);
            breath_score = edge_breathing_score(&spec);
            peak_freq    = spec.peak_freq;

            ac_freq = edge_autocorr_peak_freq(edge_work, EDGE_WINDOW_N,
                                              EDGE_FS_HZ, &ac_strength);
        }

        edge_decision_t d;
        edge_state_step(&edge_state,
                        motion_feature,
                        breath_score,
                        peak_freq,
                        edge_motion_threshold(&edge_baseline),
                        edge_breath_threshold(&edge_baseline),
                        edge_departure_threshold(&edge_baseline),
                        have_data,
                        edge_local_hour(),
                        &d);

        /* Learn only from windows the state machine vouched for. */
        if (d.baseline_updatable) {
            edge_baseline_update(&edge_baseline, motion_feature, breath_score);
        }

        /* Phone-MAC presence (edge_presence.h) is the primary HOME/AWAY
         * signal; d.away_inferred (a CSI motion-burst heuristic) is a
         * secondary, independent one. Either suppresses emergency escalation
         * -- they fail differently, so requiring both would defeat the point. */
        bool away = !edge_presence_is_home() || d.away_inferred;

        /* Emergency outranks fault: if the room is visible and respiration is
         * absent, that is what the buzzer should be saying. */
        alert_level_t want = (d.should_alert && !away) ? ALERT_EMERGENCY
                           : d.should_fault ? ALERT_FAULT
                           : ALERT_NONE;
        if (want != alert_get_level()) {
            alert_set_level(want);
            if (want == ALERT_EMERGENCY) {
                ESP_LOGW(EDGE_MON_TAG,
                         "*** EMERGENCY: %u windows with no respiration ***",
                         (unsigned) d.streak);
            } else if (want == ALERT_FAULT) {
                ESP_LOGW(EDGE_MON_TAG,
                         "SENSOR FAULT: %u windows with no CSI (check TX board power/link)",
                         (unsigned) d.data_streak);
            } else if (away) {
                ESP_LOGI(EDGE_MON_TAG,
                         "no response, but assumed AWAY (%s%s%s) -- escalation suppressed (streak=%u)",
                         !edge_presence_is_home() ? "phone off home network" : "",
                         (!edge_presence_is_home() && d.away_inferred) ? " + " : "",
                         d.away_inferred ? "CSI departure burst" : "",
                         (unsigned) d.streak);
            } else {
                ESP_LOGI(EDGE_MON_TAG, "alert cleared");
            }
#if EDGE_ENABLE_MQTT
            edge_mqtt_publish_state(EDGE_ZONE_NAME, edge_state_name(d.state),
                                    want == ALERT_EMERGENCY ? "EMERGENCY"
                                      : want == ALERT_FAULT ? "FAULT"
                                      : away ? "AWAY" : "NONE");
#endif
        }

        if (cycle % EDGE_NVS_SAVE_EVERY == 0 && edge_baseline_ready(&edge_baseline)) {
            edge_baseline_save(&edge_baseline);
        }

        /* Developer-facing only; the device does not need a listener. */
        printf("EDGE,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%.3f,%.3f,%d,%s,%d\n",
               edge_state_name(d.state),
               motion_feature,
               breath_score,
               peak_freq,
               ac_freq,
               ac_strength,
               (unsigned) d.streak,
               (unsigned) d.data_streak,
               (unsigned) frames,
               (unsigned) dropped,
               edge_motion_threshold(&edge_baseline),
               edge_breath_threshold(&edge_baseline),
               edge_baseline_ready(&edge_baseline) ? 1 : 0,
               want == ALERT_EMERGENCY ? "EMERGENCY"
                 : want == ALERT_FAULT ? "FAULT" : "-",
               d.away_inferred ? 1 : 0);

        /* Acquisition-layer diagnostics, separate from the decision line. */
        printf("EDGE_DIAG,cb=%u,nullbuf=%u,lastlen=%u,rejected=%u,ringfill=%d,"
               "presence=%s/%s\n",
               (unsigned) edge_csi_cb_count,
               (unsigned) edge_csi_null_buf,
               (unsigned) edge_csi_last_len,
               (unsigned) edge_pipe.rejected_frames,
               edge_pipe.ring.filled,
               edge_presence_mode(),
               edge_presence_is_home() ? "HOME" : "AWAY");
        fflush(stdout);
    }
}

static inline void edge_monitor_start(void) {
    edge_goertzel_init(&edge_goertzel, EDGE_FS_HZ);
    edge_state_init(&edge_state);
    edge_baseline_load(&edge_baseline);
    edge_presence_init();

    edge_csi_start();

#if EDGE_ENABLE_MQTT
    edge_mqtt_init();
#endif

    /* Priority 3: below the CSI ingest task so queue draining is never starved
     * by a decision cycle's arithmetic. */
    xTaskCreate(&edge_decision_task, "edge_decision", 8192, NULL, 3, NULL);

    printf("EDGE_HEADER,state,motion,breath_score,peak_hz,ac_hz,ac_strength,"
           "streak,data_streak,frames,dropped,motion_thr,breath_thr,calibrated,alert,away_inferred\n");
#if EDGE_DEMO
    ESP_LOGW(EDGE_MON_TAG,
             "*** DEMO BUILD *** 무응답 %d초 · 시간대 게이트 해제 — 실사용 금지",
             EDGE_NO_RESPONSE_STREAK * (EDGE_DECISION_MS / 1000));
#endif
    ESP_LOGI(EDGE_MON_TAG, "standalone edge monitor running");
}

#endif /* ESP32_CSI_EDGE_MONITOR_H */
