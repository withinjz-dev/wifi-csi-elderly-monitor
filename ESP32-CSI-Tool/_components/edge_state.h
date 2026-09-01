#ifndef ESP32_CSI_EDGE_STATE_H
#define ESP32_CSI_EDGE_STATE_H

/*
 * Anomaly state machine.
 *
 *                        motion detected
 *                   ┌──────────────────────▶ ACTIVE
 *                   │
 *   window ─────────┤
 *                   │ no motion
 *                   ▼
 *          breathing periodicity?
 *             │              │
 *            YES             NO
 *             │              │
 *             ▼              ▼
 *      SLEEP_NORMAL     NO_RESPONSE
 *                            │
 *                    sustained 60 s?  AND  sleep hours?
 *                            │
 *                           YES ──▶ ALERT
 *
 * Two guards carried over from the host prototype, both of which were added in
 * response to measured failures rather than anticipated ones:
 *
 *   - frequency consistency: a single window's breathing peak is not
 *     trustworthy. Real breathing holds a roughly steady rate for a minute;
 *     noise peaks land at a random bin each window. A peak is only accepted if
 *     it matches one of the last EDGE_FREQ_HISTORY windows within tolerance.
 *
 *   - hysteresis: alert only after EDGE_NO_RESPONSE_STREAK consecutive
 *     no-response windows, because the host version measured ~18% single-window
 *     false negatives while the subject was demonstrably breathing.
 */

#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#define EDGE_FREQ_HISTORY        3
#define EDGE_FREQ_TOLERANCE_HZ   0.12f
/*
 * Demo build: idf.py -DEDGE_DEMO=1 flash
 *
 * Two things make the real thresholds impossible to film. Emergency is gated
 * to sleep hours, so a daytime recording never triggers one; and the 60 s
 * hold is a long time to sit still on camera. This switch relaxes both.
 *
 * It exists as a build flag rather than as edits to the constants below
 * because the obvious alternative -- change the numbers, film, change them
 * back -- ships demo-tuned firmware the one time someone forgets. Here the
 * default build is always the real one, and a demo build announces itself in
 * the boot log.
 */
#ifndef EDGE_DEMO
#define EDGE_DEMO 0
#endif

#if EDGE_DEMO
  #define EDGE_NO_RESPONSE_STREAK  2      /* x 10 s = 20 s */
  #define EDGE_SLEEP_HOUR_START    0       /* 시간대 게이트 해제 */
  #define EDGE_SLEEP_HOUR_END      24
#else
  #define EDGE_NO_RESPONSE_STREAK  6      /* x 10 s = 60 s sustained */
  #define EDGE_SLEEP_HOUR_START    22
  #define EDGE_SLEEP_HOUR_END      7
#endif

/*
 * Departure inference -- DISABLED. Kept compiled-out rather than deleted
 * because the measurements that killed it are worth preserving alongside it.
 *
 * The idea was that walking out past the sensor produces a motion burst far
 * larger than ordinary in-room movement, so an amplitude threshold could mean
 * "they left". A live experiment says it does not.
 *
 *   First run, which appeared to support it, was invalid. The transmitter was
 *   an Apple Watch charging OUTSIDE the room, so the exit path crossed the
 *   line of sight between it and the receiver. Departures read 8-12 while
 *   in-room movement read under 2.1 -- but that gap measured "crossed the
 *   link", not "left the room". Geometry, not occupancy.
 *
 *   Second run used a fixed TX board inside the room and compared deliberate
 *   line-of-sight crossings (staying inside) against real departures. Every
 *   window across both conditions fell in 0.55-5.09 with no separation, and
 *   the 8-12 range never reappeared. data/departure_experiment.csv.
 *
 * So rssi_std amplitude encodes how vigorously and how near someone moved,
 * not whether they left. A bathroom trip past the link looks exactly like
 * walking out the front door. No threshold separates them, which is why this
 * is disabled rather than retuned -- the problem is the feature, not the
 * number.
 *
 * Why disabled rather than merely inaccurate: away_inferred SUPPRESSES
 * emergency escalation, and it was OR'd with the phone-presence signal, so a
 * single false "departure" silently disarmed the alarm while the resident was
 * in the room. Combining a reliable signal with an unreliable one under OR
 * makes the system less safe, not more.
 *
 * Re-enabling would need a feature that carries direction or sequence (zone
 * transitions, phase/Doppler), not a larger constant. Set EDGE_ENABLE_DEPARTURE
 * to 1 to compile the original logic back in.
 */
#ifndef EDGE_ENABLE_DEPARTURE
#define EDGE_ENABLE_DEPARTURE 0
#endif

#define EDGE_AWAY_TIMEOUT_CYCLES 4320

typedef enum {
    EDGE_STATE_NO_DATA = 0,
    EDGE_STATE_ACTIVE,
    EDGE_STATE_SLEEP_NORMAL,
    EDGE_STATE_NO_RESPONSE,
} edge_state_t;

static inline const char *edge_state_name(edge_state_t s) {
    switch (s) {
        case EDGE_STATE_ACTIVE:       return "ACTIVE";
        case EDGE_STATE_SLEEP_NORMAL: return "SLEEP_NORMAL";
        case EDGE_STATE_NO_RESPONSE:  return "NO_RESPONSE";
        default:                      return "NO_DATA";
    }
}

/*
 * Two streaks, deliberately not one.
 *
 * NO_RESPONSE means the device can see the room and finds no respiration --
 * a possible medical emergency. NO_DATA means the device cannot see the room
 * at all, because the transmitter died or the link dropped -- a maintenance
 * problem. A live test collapsed these into one counter and rang the full
 * emergency buzzer for a TX board that had simply lost power. They escalate
 * separately now, and only the first is gated on sleep hours: a blind sensor
 * is worth reporting whenever it happens.
 */
typedef struct {
    float    freq_hist[EDGE_FREQ_HISTORY];
    int      freq_count;
    int      freq_head;
    uint32_t no_response_streak;
    uint32_t no_data_streak;
    bool     alert_active;
    edge_state_t last_state;
    bool     away_inferred;
    uint32_t away_cycles;
} edge_state_ctx_t;

static inline void edge_state_init(edge_state_ctx_t *c) {
    memset(c, 0, sizeof(*c));
    c->last_state = EDGE_STATE_NO_DATA;
}

static inline void edge_freq_push(edge_state_ctx_t *c, float f) {
    c->freq_hist[c->freq_head] = f;
    c->freq_head = (c->freq_head + 1) % EDGE_FREQ_HISTORY;
    if (c->freq_count < EDGE_FREQ_HISTORY) c->freq_count++;
}

static inline bool edge_freq_consistent(const edge_state_ctx_t *c, float f) {
    for (int i = 0; i < c->freq_count; i++) {
        if (fabsf(f - c->freq_hist[i]) <= EDGE_FREQ_TOLERANCE_HZ) return true;
    }
    return false;
}

static inline bool edge_in_sleep_hours(int hour) {
    if (EDGE_SLEEP_HOUR_START > EDGE_SLEEP_HOUR_END) {
        return hour >= EDGE_SLEEP_HOUR_START || hour < EDGE_SLEEP_HOUR_END;
    }
    return hour >= EDGE_SLEEP_HOUR_START && hour < EDGE_SLEEP_HOUR_END;
}

typedef struct {
    edge_state_t state;
    bool  breathing_confirmed;
    bool  should_alert;        /* emergency: room visible, no respiration    */
    bool  should_fault;        /* maintenance: room not visible at all       */
    bool  baseline_updatable;  /* safe to fold this window into the baseline? */
    float peak_freq;
    float breath_score;
    float motion_feature;
    uint32_t streak;
    uint32_t data_streak;
    bool     away_inferred;    /* departure burst seen; escalation suppressed */
} edge_decision_t;

/*
 * One decision cycle.
 *
 *   motion_feature  -- rssi_std over the window
 *   breath_score    -- flat-noise-normalized spectral concentration (1.0 = flat)
 *   peak_freq       -- strongest breathing-band bin, Hz
 *   have_data       -- did the window actually fill with CSI?
 *   hour            -- local hour, or -1 if the clock was never set
 */
static inline void edge_state_step(edge_state_ctx_t *c,
                                   float motion_feature,
                                   float breath_score,
                                   float peak_freq,
                                   float motion_threshold,
                                   float breath_threshold,
                                   float departure_threshold,
                                   bool have_data,
                                   int hour,
                                   edge_decision_t *out) {
    memset(out, 0, sizeof(*out));
    out->motion_feature = motion_feature;
    out->breath_score   = breath_score;
    out->peak_freq      = peak_freq;

    if (!have_data) {
        out->state = EDGE_STATE_NO_DATA;
        c->no_data_streak++;
        /* Deliberately does NOT touch no_response_streak: a blind sensor is
         * not evidence about the person either way. */
#if EDGE_ENABLE_DEPARTURE
    } else if (motion_feature > departure_threshold) {
        /* Disabled by default -- measured not to separate departures from
         * ordinary movement. See the note above EDGE_ENABLE_DEPARTURE. */
        c->no_data_streak = 0;
        out->state = EDGE_STATE_ACTIVE;
        c->no_response_streak = 0;
        c->freq_count = 0;
        c->freq_head  = 0;
        c->away_inferred = true;
        c->away_cycles   = 0;
#endif
    } else if (motion_feature > motion_threshold) {
        c->no_data_streak = 0;
        out->state = EDGE_STATE_ACTIVE;
        c->no_response_streak = 0;
        c->freq_count = 0;          /* motion invalidates the breathing history */
        c->freq_head  = 0;
        /* Ordinary in-room motion is presence, not a departure -- clears any
         * standing away assumption immediately (they're evidently still/back
         * home moving around). */
        c->away_inferred = false;
        c->away_cycles   = 0;
    } else if (breath_score > breath_threshold) {
        c->no_data_streak = 0;
        bool consistent = edge_freq_consistent(c, peak_freq);
        edge_freq_push(c, peak_freq);
        if (consistent) {
            out->state = EDGE_STATE_SLEEP_NORMAL;
            out->breathing_confirmed = true;
            c->no_response_streak = 0;
            out->baseline_updatable = true;   /* still + breathing == good sample */
            /* Breathing is direct evidence of presence -- clears the away
             * assumption even though there was no motion burst back. */
            c->away_inferred = false;
            c->away_cycles   = 0;
        } else {
            out->state = EDGE_STATE_NO_RESPONSE;
            c->no_response_streak++;
        }
    } else {
        c->no_data_streak = 0;
        out->state = EDGE_STATE_NO_RESPONSE;
        c->no_response_streak++;
    }

    /* While away is assumed, nothing above re-confirmed presence: count this
     * cycle against the timeout. Once it lapses, drop the assumption so
     * ordinary NO_RESPONSE escalation can resume -- the streak kept counting
     * underneath the whole time, so escalation is immediate at that point,
     * not delayed another 60 s. */
    if (c->away_inferred && out->state != EDGE_STATE_ACTIVE
                          && out->state != EDGE_STATE_SLEEP_NORMAL) {
        c->away_cycles++;
        if (c->away_cycles >= EDGE_AWAY_TIMEOUT_CYCLES) {
            c->away_inferred = false;
            c->away_cycles   = 0;
        }
    }

    out->streak        = c->no_response_streak;
    out->data_streak    = c->no_data_streak;
    out->away_inferred = c->away_inferred;

    /* Emergency is gated on sleep hours -- a person out of the room during the
     * day is not an emergency. A blind sensor is not gated: whenever the device
     * stops seeing, somebody should know. */
    bool sleep_hours = (hour < 0) ? true : edge_in_sleep_hours(hour);
    out->should_alert = (c->no_response_streak >= EDGE_NO_RESPONSE_STREAK)
                        && sleep_hours
                        && !c->away_inferred;
    out->should_fault = (c->no_data_streak >= EDGE_NO_RESPONSE_STREAK);

    /* Never learn from a window that is driving, or about to drive, an alert. */
    if (out->should_alert || c->no_response_streak > 0 || !have_data) {
        out->baseline_updatable = false;
    }

    c->alert_active = out->should_alert;
    c->last_state   = out->state;
}

#endif /* ESP32_CSI_EDGE_STATE_H */
