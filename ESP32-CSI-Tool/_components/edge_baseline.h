#ifndef ESP32_CSI_EDGE_BASELINE_H
#define ESP32_CSI_EDGE_BASELINE_H

/*
 * Per-room adaptive baseline.
 *
 * Why this shape, and not a per-room threshold table:
 *   Day 5 measured per-room Otsu thresholds spanning 1.22..6.32 (5.18x) and we
 *   initially read that as "every room needs its own threshold". Re-checking it
 *   (data/validate_normalization.py) showed most of that spread came from how
 *   vigorously the person happened to move in each session, not from the room's
 *   RF character -- Otsu splits between the still cluster and the motion
 *   cluster, so the motion cluster's position drags the threshold around.
 *   Comparing only the controlled condition (person still) the rooms differ by
 *   just 1.47x.
 *
 *   So the device does not learn a threshold. It learns the *still baseline*
 *   (mean, sigma) and applies a firmware constant k on top:
 *
 *       motion  <=>  rssi_std > mu_still + K_MOTION * sigma_still
 *
 *   Validated on the four recorded rooms: threshold spread drops 5.18x -> 1.30x
 *   with 0/69 false "motion" on still windows and 1/16 missed motion at k=3.
 *   That collapses the whole calibration problem to two floats in NVS and
 *   removes Otsu from the device entirely.
 */

#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#define EDGE_NVS_NAMESPACE   "edge_cal"
#define EDGE_NVS_KEY         "baseline"

/* Sigma multipliers, chosen against the recorded dataset. A false "motion"
 * is the more dangerous error here: it resets the no-response streak and can
 * therefore mask a real emergency, so we bias toward missing gentle motion. */
#define EDGE_K_MOTION        3.0f
#define EDGE_K_BREATH        1.8f   /* on the flat-noise-normalized score */

/* EWMA rate for slow drift (furniture moved, seasonal RF change).
 * 0.002 at one update per 10 s is a time constant of roughly 80 minutes. */
#define EDGE_EWMA_ALPHA      0.002f

/* Bootstrap: how many still windows before the baseline is trusted. */
#define EDGE_BASELINE_MIN_N  30

typedef struct {
    float    motion_mu;      /* mean rssi_std while still            */
    float    motion_sigma;   /* sigma of rssi_std while still        */
    float    breath_mu;      /* mean breathing score while still     */
    float    breath_sigma;
    uint32_t n;              /* still windows folded in so far       */
    uint32_t magic;
} edge_baseline_t;

#define EDGE_BASELINE_MAGIC  0xB5E1C0DEu

static const char *EDGE_BL_TAG = "edge_baseline";

static inline void edge_baseline_reset(edge_baseline_t *b) {
    memset(b, 0, sizeof(*b));
    b->magic = EDGE_BASELINE_MAGIC;
    /* Conservative pre-calibration defaults so the device is usable on first
     * boot; they are replaced as soon as EDGE_BASELINE_MIN_N windows land. */
    b->motion_mu    = 0.90f;
    b->motion_sigma = 0.22f;
    b->breath_mu    = 1.00f;
    b->breath_sigma = 0.30f;
}

static inline bool edge_baseline_ready(const edge_baseline_t *b) {
    return b->n >= EDGE_BASELINE_MIN_N;
}

static inline float edge_motion_threshold(const edge_baseline_t *b) {
    return b->motion_mu + EDGE_K_MOTION * b->motion_sigma;
}

static inline float edge_breath_threshold(const edge_baseline_t *b) {
    return b->breath_mu + EDGE_K_BREATH * b->breath_sigma;
}

/*
 * Fold one window into the baseline.
 *
 * The caller MUST only pass windows it is confident are "person present and
 * still" or "empty and quiet" -- never a window that is currently driving an
 * alert. Updating during an anomaly would teach the device that the anomaly is
 * normal, which is the one failure mode that silently disables the alarm.
 */
static inline void edge_baseline_update(edge_baseline_t *b,
                                        float motion_feature,
                                        float breath_feature) {
    if (b->n < EDGE_BASELINE_MIN_N) {
        /* Bootstrap phase: plain running mean/variance so the estimate
         * converges fast instead of crawling at the EWMA rate. */
        float n1 = (float)(b->n + 1);
        float dm = motion_feature - b->motion_mu;
        b->motion_mu += dm / n1;
        b->motion_sigma = sqrtf(fmaxf(0.0f,
            (b->motion_sigma * b->motion_sigma * (float) b->n
             + dm * (motion_feature - b->motion_mu)) / n1));

        float db = breath_feature - b->breath_mu;
        b->breath_mu += db / n1;
        b->breath_sigma = sqrtf(fmaxf(0.0f,
            (b->breath_sigma * b->breath_sigma * (float) b->n
             + db * (breath_feature - b->breath_mu)) / n1));

        b->n++;
    } else {
        /* Steady state: exponential forgetting so the room can drift. */
        const float a = EDGE_EWMA_ALPHA;
        float dm = motion_feature - b->motion_mu;
        b->motion_mu    = (1.0f - a) * b->motion_mu + a * motion_feature;
        b->motion_sigma = sqrtf(fmaxf(1e-6f,
            (1.0f - a) * b->motion_sigma * b->motion_sigma + a * dm * dm));

        float db = breath_feature - b->breath_mu;
        b->breath_mu    = (1.0f - a) * b->breath_mu + a * breath_feature;
        b->breath_sigma = sqrtf(fmaxf(1e-6f,
            (1.0f - a) * b->breath_sigma * b->breath_sigma + a * db * db));
    }

    /* A collapsed sigma would make the threshold hair-trigger. */
    if (b->motion_sigma < 0.05f) b->motion_sigma = 0.05f;
    if (b->breath_sigma < 0.05f) b->breath_sigma = 0.05f;
}

/* ---- NVS persistence ---------------------------------------------------- */

static inline bool edge_baseline_load(edge_baseline_t *b) {
    nvs_handle_t h;
    if (nvs_open(EDGE_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        edge_baseline_reset(b);
        return false;
    }
    size_t len = sizeof(*b);
    esp_err_t err = nvs_get_blob(h, EDGE_NVS_KEY, b, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(*b) || b->magic != EDGE_BASELINE_MAGIC) {
        edge_baseline_reset(b);
        return false;
    }
    ESP_LOGI(EDGE_BL_TAG, "baseline loaded: motion mu=%.3f sigma=%.3f n=%u",
             b->motion_mu, b->motion_sigma, (unsigned) b->n);
    return true;
}

static inline void edge_baseline_save(const edge_baseline_t *b) {
    nvs_handle_t h;
    if (nvs_open(EDGE_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(EDGE_BL_TAG, "nvs_open failed; baseline not persisted");
        return;
    }
    if (nvs_set_blob(h, EDGE_NVS_KEY, b, sizeof(*b)) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

#endif /* ESP32_CSI_EDGE_BASELINE_H */
