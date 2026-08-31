#ifndef ESP32_CSI_EDGE_CSI_H
#define ESP32_CSI_EDGE_CSI_H

/*
 * CSI acquisition and feature extraction, on-device.
 *
 * Threading follows the ESP-IDF guidance for esp_wifi_set_csi_rx_cb(): the
 * callback runs in the Wi-Fi task's context, so it does no arithmetic beyond a
 * memcpy and hands the frame to a queue. Everything else happens in a lower
 * priority task that the Wi-Fi stack can preempt freely.
 *
 *   Wi-Fi task ──(csi cb: memcpy only)──▶ xQueue ──▶ CSI task ──▶ ring buffer
 *                                                                     │
 *                                                      Decision task ─┘ (10 s)
 *
 * Data reduction: a HT20 LLTF frame carries 52 usable subcarriers as int8 I/Q
 * pairs. Storing 20 s of that raw would be 52 x 2 x 11 Hz x 20 s = 23 kB per
 * window for no benefit -- the question being asked is only "moving / still /
 * breathing", not pose reconstruction. Each frame therefore collapses to a
 * single scalar before it is ever stored, and the ring buffer holds 256 floats
 * (1 kB) total.
 *
 * Power, not amplitude: we use I^2 + Q^2 and skip the square root. For a
 * change-detection statistic the sqrt buys nothing and costs a transcendental
 * per subcarrier per frame. It is not perfectly free -- squaring a signal
 * A + B*sin(wt) produces a small 2w term -- but with B << A for breathing that
 * harmonic sits far below the fundamental, and the Goertzel bank only reads the
 * 0.2-0.5 Hz band anyway.
 */

#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "edge_dsp.h"

/* ---- HT20 LLTF subcarrier map ------------------------------------------- */
/*
 * 64 slots of int8 I/Q. The occupied slots were measured from a real capture
 * rather than assumed (data/breathing_test_03.csv, 300 frames): slots 6..31 and
 * 33..58 carry data in every frame, slot 32 is DC and always null, and 2..5 /
 * 59..63 are guard. That is 52 usable subcarriers, matching the HT20 LLTF spec,
 * but shifted from the textbook 1..26 / 38..63 layout -- reading the wrong
 * slots would have silently sampled guard bands.
 */
#define EDGE_CSI_SLOTS        64
#define EDGE_SUBCARRIER_MAX   52
#define EDGE_SELECTED_MAX      8
#define EDGE_SC_LOW_BASE       6   /* compact idx 0..25  -> slots  6..31 */
#define EDGE_SC_HIGH_BASE     33   /* compact idx 26..51 -> slots 33..58 */

/* ---- queue -------------------------------------------------------------- */
#define EDGE_CSI_QUEUE_LEN    16
#define EDGE_CSI_MAX_BYTES   128

typedef struct {
    int64_t  t_us;
    int8_t   rssi;
    uint16_t len;
    bool     first_word_invalid;
    int8_t   buf[EDGE_CSI_MAX_BYTES];
} edge_csi_frame_t;

static QueueHandle_t edge_csi_queue = NULL;
static const char *EDGE_CSI_TAG = "edge_csi";

/* Dropped-frame counter: exposed so the decision task can report link quality
 * instead of silently degrading when the queue backs up. */
static volatile uint32_t edge_csi_dropped = 0;

/* Callback-entry counter. Distinguishes "the Wi-Fi driver never calls us" from
 * "it calls us but every frame is rejected downstream" -- from the outside both
 * look identical (frames=0) and the two have completely different causes. */
static volatile uint32_t edge_csi_cb_count = 0;
static volatile uint32_t edge_csi_null_buf = 0;
static volatile uint16_t edge_csi_last_len = 0;

/* ---- shared pipeline state ---------------------------------------------- */

typedef struct {
    /* subcarrier selection */
    uint8_t  selected[EDGE_SELECTED_MAX];
    uint8_t  n_selected;
    bool     selection_done;

    /* calibration scoring (only allocated logic-wise during selection) */
    edge_biquad_t sc_bp[EDGE_SUBCARRIER_MAX];
    edge_stat_t   sc_raw[EDGE_SUBCARRIER_MAX];
    edge_stat_t   sc_band[EDGE_SUBCARRIER_MAX];
    uint32_t      sc_frames;

    /* 100 ms time-binning to a uniform 10 Hz stream */
    int64_t  bin_start_us;
    float    bin_accum;
    uint32_t bin_count;
    float    last_bin_value;
    bool     have_last_bin;

    /* analysis window */
    edge_ring_t  ring;
    edge_stat_t  rssi_stat;      /* reset each decision cycle */
    uint32_t     frames_this_window;
    uint32_t     rejected_frames;  /* wrong-length frames, for diagnosis */
} edge_pipeline_t;

static edge_pipeline_t edge_pipe;

/* Map a compact subcarrier index (0..51) onto its slot in the 64-slot frame. */
static inline int edge_sc_slot(int idx) {
    return (idx < 26) ? (idx + EDGE_SC_LOW_BASE)
                      : (idx - 26 + EDGE_SC_HIGH_BASE);
}

/* ---- CSI callback: memcpy only ------------------------------------------ */

static void edge_csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
    edge_csi_cb_count++;
    if (!info || !info->buf) { edge_csi_null_buf++; return; }
    edge_csi_last_len = info->len;
    if (!edge_csi_queue) return;

    edge_csi_frame_t f;
    f.t_us = esp_timer_get_time();
    f.rssi = info->rx_ctrl.rssi;
    f.len  = (info->len > EDGE_CSI_MAX_BYTES) ? EDGE_CSI_MAX_BYTES : info->len;
    f.first_word_invalid = info->first_word_invalid;
    memcpy(f.buf, info->buf, f.len);

    /* Never block the Wi-Fi task. A full queue means the CSI task fell behind,
     * which is worth reporting but not worth stalling the radio for. */
    if (xQueueSend(edge_csi_queue, &f, 0) != pdTRUE) {
        edge_csi_dropped++;
    }
}

/* ---- feature extraction ------------------------------------------------- */

/* power[k] = I^2 + Q^2 for one subcarrier, from the int8 I/Q pair. */
static inline float edge_sc_power(const int8_t *buf, int slot) {
    float im = (float) buf[slot * 2];
    float re = (float) buf[slot * 2 + 1];
    return im * im + re * re;
}

static inline void edge_pipeline_init(edge_pipeline_t *p) {
    memset(p, 0, sizeof(*p));
    edge_ring_init(&p->ring);
    edge_stat_init(&p->rssi_stat);
    for (int i = 0; i < EDGE_SUBCARRIER_MAX; i++) {
        edge_biquad_init_bandpass(&p->sc_bp[i],
                                  EDGE_BREATH_LO_HZ, EDGE_BREATH_HI_HZ, EDGE_FS_HZ);
        edge_stat_init(&p->sc_raw[i]);
        edge_stat_init(&p->sc_band[i]);
    }
    /* Until selection completes, fall back to a spread of mid-band subcarriers
     * (compact indices, so 4..21 and 30..47 in slot terms) rather than
     * averaging all 52 -- the band-edge subcarriers are the noisiest. */
    const uint8_t fallback[EDGE_SELECTED_MAX] = {4, 9, 14, 21, 30, 37, 43, 47};
    memcpy(p->selected, fallback, sizeof(fallback));
    p->n_selected = EDGE_SELECTED_MAX;
}

/*
 * Subcarrier selection -- a compute reduction, NOT an accuracy improvement.
 *
 * What this measures: the fraction of a subcarrier's variance that survives the
 * 0.2-0.5 Hz band-pass, gated on the subcarrier having enough raw variance to
 * be alive at all (so guard bands and dead slots drop out).
 *
 * What it is worth, measured rather than assumed. Running the full pipeline on
 * the reference recording with four different subcarrier sets -- all 52, a
 * fixed 8, the best-scoring 8, and deliberately the *worst*-scoring 8 -- gave
 * 19-20 of 23 breathing detections in every case. Even the worst-8 set matched
 * the best. On this hardware and recording the subcarriers are highly
 * correlated, so picking "good" ones buys nothing in detection rate; the only
 * measurable gain was peak-frequency stability (0.067 Hz sd for the best 8 vs
 * 0.082 Hz for all 52).
 *
 * The reason to keep it is therefore arithmetic, and that reason is real:
 * processing 8 subcarriers instead of 52 cuts per-frame work by 6.5x for
 * equivalent decisions, which matters when this runs continuously on a core the
 * Wi-Fi stack also wants. A concentration-based score discriminated about 2x
 * better (1.63x spread vs 1.23x) but needs a 256-sample history per subcarrier
 * -- 52 kB -- and, given selection quality does not move the decision, that
 * memory would buy nothing. The cheap streaming metric stays.
 */
static inline void edge_score_subcarriers(edge_pipeline_t *p, const int8_t *buf) {
    for (int i = 0; i < EDGE_SUBCARRIER_MAX; i++) {
        float pw = edge_sc_power(buf, edge_sc_slot(i));
        edge_stat_push(&p->sc_raw[i], pw);
        float y = edge_biquad_step(&p->sc_bp[i], pw);
        edge_stat_push(&p->sc_band[i], y);
    }
    p->sc_frames++;
}

static inline void edge_finish_selection(edge_pipeline_t *p) {
    float score[EDGE_SUBCARRIER_MAX];
    float raw_var_max = 0.0f;

    for (int i = 0; i < EDGE_SUBCARRIER_MAX; i++) {
        float rv = edge_stat_var(&p->sc_raw[i]);
        if (rv > raw_var_max) raw_var_max = rv;
    }

    for (int i = 0; i < EDGE_SUBCARRIER_MAX; i++) {
        float rv = edge_stat_var(&p->sc_raw[i]);
        float bv = edge_stat_var(&p->sc_band[i]);
        /* Dead or near-dead subcarrier: exclude outright. */
        if (rv < 1e-3f || rv < 0.02f * raw_var_max) { score[i] = -1.0f; continue; }
        score[i] = bv / rv;
    }

    /* Partial selection sort for the top EDGE_SELECTED_MAX. */
    bool taken[EDGE_SUBCARRIER_MAX] = {false};
    p->n_selected = 0;
    for (int s = 0; s < EDGE_SELECTED_MAX; s++) {
        int best = -1;
        for (int i = 0; i < EDGE_SUBCARRIER_MAX; i++) {
            if (taken[i] || score[i] < 0.0f) continue;
            if (best < 0 || score[i] > score[best]) best = i;
        }
        if (best < 0) break;
        taken[best] = true;
        p->selected[p->n_selected++] = (uint8_t) best;
    }

    if (p->n_selected == 0) {
        ESP_LOGW(EDGE_CSI_TAG, "subcarrier selection found nothing usable; keeping fallback");
        p->n_selected = EDGE_SELECTED_MAX;
    } else {
        p->selection_done = true;
        ESP_LOGI(EDGE_CSI_TAG, "selected %u subcarriers after %u frames:",
                 p->n_selected, (unsigned) p->sc_frames);
        for (int i = 0; i < p->n_selected; i++) {
            ESP_LOGI(EDGE_CSI_TAG, "   sc[%2u] band/raw variance ratio %.4f",
                     p->selected[i], score[p->selected[i]]);
        }
    }
}

/* Composite scalar for one frame: mean power over the selected subcarriers. */
static inline float edge_frame_feature(const edge_pipeline_t *p, const int8_t *buf) {
    float sum = 0.0f;
    for (int i = 0; i < p->n_selected; i++) {
        sum += edge_sc_power(buf, edge_sc_slot(p->selected[i]));
    }
    return sum / (float) p->n_selected;
}

/*
 * Fold one frame into the uniform 10 Hz stream.
 *
 * CSI frames arrive at roughly 11 Hz but with jitter, and both the biquad and
 * Goertzel assume a fixed sample rate. Rather than resampling with timestamps
 * the way the host version does with np.interp, the device bins by arrival time
 * into fixed 100 ms slots and emits each slot's mean. An empty slot repeats the
 * previous value, which is the cheapest defensible interpolation and keeps the
 * sample index locked to wall-clock time even when packets are lost.
 */
static inline void edge_ingest_frame(edge_pipeline_t *p, const edge_csi_frame_t *f) {
    /* first_word_invalid means the leading I/Q pair is garbage. Our lowest
     * usable slot is 6, so the affected region is well below anything we read. */
    (void) f->first_word_invalid;

    /* The expected HT20 LLTF frame is 128 bytes. Report the first frame's
     * actual length once -- a silent length mismatch here shows up downstream
     * only as "no data", which is expensive to diagnose from the outside. */
    static bool logged_len = false;
    if (!logged_len) {
        logged_len = true;
        ESP_LOGI(EDGE_CSI_TAG, "first CSI frame: len=%u rssi=%d (need >= %d)",
                 (unsigned) f->len, (int) f->rssi, EDGE_CSI_SLOTS * 2);
    }

    if (f->len < EDGE_CSI_SLOTS * 2) {
        p->rejected_frames++;
        return;
    }

    edge_stat_push(&p->rssi_stat, (float) f->rssi);
    p->frames_this_window++;

    if (!p->selection_done) edge_score_subcarriers(p, f->buf);

    float feat = edge_frame_feature(p, f->buf);

    if (p->bin_start_us == 0) p->bin_start_us = f->t_us;

    /* Close out every 100 ms slot the frame's timestamp has passed. */
    while (f->t_us - p->bin_start_us >= EDGE_BIN_US) {
        float v;
        if (p->bin_count > 0) {
            v = p->bin_accum / (float) p->bin_count;
            p->last_bin_value = v;
            p->have_last_bin  = true;
        } else if (p->have_last_bin) {
            v = p->last_bin_value;             /* zero-order hold across a gap */
        } else {
            v = feat;
        }
        edge_ring_push(&p->ring, v);
        p->bin_accum   = 0.0f;
        p->bin_count   = 0;
        p->bin_start_us += EDGE_BIN_US;
    }

    p->bin_accum += feat;
    p->bin_count++;
}

/* ---- task --------------------------------------------------------------- */

static void edge_csi_task(void *arg) {
    edge_csi_frame_t f;
    for (;;) {
        if (xQueueReceive(edge_csi_queue, &f, portMAX_DELAY) == pdTRUE) {
            edge_ingest_frame(&edge_pipe, &f);
        }
    }
}

static inline void edge_csi_start(void) {
    edge_pipeline_init(&edge_pipe);

    edge_csi_queue = xQueueCreate(EDGE_CSI_QUEUE_LEN, sizeof(edge_csi_frame_t));
    if (!edge_csi_queue) {
        ESP_LOGE(EDGE_CSI_TAG, "queue alloc failed");
        return;
    }

    /* Priority 4: below the Wi-Fi task (23) so the radio always wins, above
     * the idle/logging work so the queue drains promptly. */
    xTaskCreate(&edge_csi_task, "edge_csi", 4096, NULL, 4, NULL);

    /* Order matters: esp_wifi_set_csi(1) must come first. The upstream
     * CSI-Tool enables CSI before configuring it, and reversing that here
     * produced a callback that never fired. */
    ESP_ERROR_CHECK(esp_wifi_set_csi(1));

    wifi_csi_config_t cfg = {};
    cfg.lltf_en           = 1;
    cfg.htltf_en          = 1;
    cfg.stbc_htltf2_en    = 1;
    cfg.ltf_merge_en      = 1;
    cfg.channel_filter_en = 0;
    cfg.manu_scale        = 0;

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(&edge_csi_rx_cb, NULL));

    ESP_LOGI(EDGE_CSI_TAG, "edge CSI pipeline started (queue=%d, window=%d @ %.0f Hz)",
             EDGE_CSI_QUEUE_LEN, EDGE_WINDOW_N, EDGE_FS_HZ);
}

#endif /* ESP32_CSI_EDGE_CSI_H */
