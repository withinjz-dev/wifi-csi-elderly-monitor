#ifndef ESP32_CSI_EDGE_PRESENCE_H
#define ESP32_CSI_EDGE_PRESENCE_H

/*
 * HOME/AWAY from the resident's phone, without a second CSI zone.
 *
 * The phone leaves the house with its owner and stays home when they do, so
 * whether it is currently associated with this board's SoftAP is a direct
 * occupancy signal that costs the phone nothing -- it only reads association
 * state the phone maintains anyway. (Using the phone as a CSI *transmitter*
 * was considered and rejected: sustaining a breathing-band frame rate would
 * block the phone's WiFi power save and drain its battery, and a dead phone
 * is itself a safety failure for the resident.)
 *
 * Three tiers, most specific first. Each falls back to the next, so the
 * module always produces an answer:
 *
 *   1. PINNED   -- EDGE_PHONE_MAC set at build time. Exact, but breaks when
 *                  the OS rotates its per-network random MAC (iOS "Private
 *                  Wi-Fi Address", Android per-SSID randomization). Those are
 *                  stable per network in practice and only re-roll if the
 *                  network is forgotten and rejoined -- but that happened
 *                  during this project's own bring-up, so it is a real risk.
 *
 *   2. REGISTERED -- set once at install by the REGISTER serial command.
 *                  Covers the days before automatic learning has enough
 *                  evidence. Never overwritten by adoption.
 *
 *   2b. ADOPTED -- learned automatically, no setup by the resident.
 *
 *   3. ANY      -- any *qualified* associated station counts as the resident.
 *                  Valid because this is a closed SoftAP, not the home
 *                  router: the installer joins one phone to it and nothing
 *                  else has the credentials. Immune to MAC rotation.
 *
 * The qualification rule, shared by tiers 2 and 3 -- "comes and goes", never
 * "always here":
 *
 *   The obvious rule is to trust the device that is present most often. That
 *   is exactly backwards. An always-on device (a TV, a tablet left on the
 *   charger) would win, and a device that is never absent carries zero
 *   occupancy information -- it would report HOME forever and silently
 *   disable every emergency alert. That is the worst failure this module
 *   could have, because nothing about it looks broken.
 *
 *   So a device only counts once it has been observed *leaving*. One
 *   departure qualifies it for tier 3; EDGE_PRESENCE_ADOPT_TRANSITIONS of
 *   them promotes it to tier 2. Only a device that travels with a person can
 *   report whether a person is here.
 *
 *   This also self-heals MAC rotation: a rotated MAC is simply a new device
 *   accumulating its own departures, while the old one goes stale and is
 *   dropped after EDGE_PRESENCE_READOPT_MS.
 *
 * Failure direction is deliberately the safe one throughout: anything
 * unknown, unqualified, or not yet learned resolves to HOME, which leaves
 * emergency escalation fully armed. Reporting AWAY -- which suppresses
 * alerts -- requires positive evidence of absence from a device that has
 * proven it moves.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "alert_component.h"   /* LED feedback during the registration window */

/*
 * Tier 1. All-zero (the default) leaves pinning off and lets tiers 2-3 run,
 * which is the intended product behavior -- no per-household MAC lookup at
 * install time. Set it only to lock onto one known handset, e.g. the bench
 * phone used during this project's verification:
 *   #define EDGE_PHONE_MAC {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}
 */
#ifndef EDGE_PHONE_MAC
#define EDGE_PHONE_MAC {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#endif

/* Unseen this long => that device is gone. Comfortably longer than the WiFi
 * power-save gaps that make a phone miss a poll or two while still in the room. */
#define EDGE_PRESENCE_TIMEOUT_MS    (5 * 60 * 1000)

/* Distinct devices tracked while learning. A closed SoftAP sees very few. */
#define EDGE_PRESENCE_MAX_CAND      4

/* Departures before a device may be adopted (tier 2). One departure already
 * qualifies it for tier 3 -- see the qualification rule above. */
#define EDGE_PRESENCE_ADOPT_TRANSITIONS  3

/* An adopted device unseen this long is presumed replaced (MAC rotated, new
 * handset). Adoption is dropped and learning restarts. Long, because this
 * covers hospital stays and holidays, and dropping it early would only mean
 * falling back to tier 3 anyway. */
#define EDGE_PRESENCE_READOPT_MS    (7LL * 24 * 60 * 60 * 1000)

/* How long the install-time REGISTER window stays open. It always runs to
 * completion (the decision needs to see every arrival), so this is also how
 * long the installer waits. */
#define EDGE_PRESENCE_REGISTER_MS   (60 * 1000)

/*
 * Registration button. GPIO0 is the BOOT button already fitted to every
 * ESP32 DevKit board, so this costs nothing and adds no wiring -- the
 * alternative was asking an installer to attach a laptop and type a serial
 * command in an elderly resident's home. Outside of reset it is an ordinary
 * input with an internal pull-up, reading LOW while pressed.
 *
 * One caveat inherited from the hardware: GPIO0 held LOW *during reset* puts
 * the chip into download mode instead of booting. That only matters if
 * someone presses it while power-cycling, and the recovery is to release it
 * and reset again.
 */
#define EDGE_REG_BUTTON_GPIO        GPIO_NUM_0
#define EDGE_REG_BUTTON_HOLD_MS     2000

/* Shares edge_baseline.h's namespace; distinct key. */
#define EDGE_PRESENCE_NVS_NAMESPACE "edge_cal"
#define EDGE_PRESENCE_NVS_KEY       "presence"
#define EDGE_PRESENCE_MAGIC         0x9E5EACE1u

static const char *EDGE_PRESENCE_TAG = "edge_presence";

typedef struct {
    uint8_t  mac[6];
    bool     used;
    uint32_t transitions;     /* departures observed -- the qualification metric */
} edge_presence_learned_t;

/* Exactly what survives a reboot: identities and their departure counts.
 * Timestamps are esp_timer values relative to boot and are meaningless
 * across one, so they live only in RAM and reset to "not seen yet". */
typedef struct {
    uint32_t magic;
    bool     adopted;
    bool     registered;      /* adopted_mac came from REGISTER, not learning */
    uint8_t  adopted_mac[6];
    edge_presence_learned_t cand[EDGE_PRESENCE_MAX_CAND];
} edge_presence_nvs_t;

typedef struct {
    uint8_t  pinned_mac[6];
    bool     pinned;
    int64_t  pinned_last_seen_ms;

    edge_presence_nvs_t persist;          /* mirrored to NVS */
    bool     dirty;

    bool     present_last[EDGE_PRESENCE_MAX_CAND];
    int64_t  cand_last_seen_ms[EDGE_PRESENCE_MAX_CAND];

    /* Install-time registration window */
    bool     reg_pending;
    int64_t  reg_until_ms;
    bool     reg_present_at_start[EDGE_PRESENCE_MAX_CAND];
    bool     reg_appeared[EDGE_PRESENCE_MAX_CAND];

    int64_t  adopted_last_seen_ms;
    int64_t  any_last_seen_ms;            /* qualified stations only */

    /* Registration button */
    int64_t  btn_down_since_ms;           /* 0 = not currently pressed */
    bool     btn_fired;                   /* latched until release */
} edge_presence_state_t;

static edge_presence_state_t edge_presence;

/* ---- NVS ---------------------------------------------------------------- */

static inline void edge_presence_save(void) {
    nvs_handle_t h;
    if (nvs_open(EDGE_PRESENCE_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(EDGE_PRESENCE_TAG, "nvs_open failed; learning not persisted");
        return;
    }
    if (nvs_set_blob(h, EDGE_PRESENCE_NVS_KEY,
                     &edge_presence.persist, sizeof(edge_presence.persist)) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

static inline bool edge_presence_load(void) {
    nvs_handle_t h;
    if (nvs_open(EDGE_PRESENCE_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    size_t len = sizeof(edge_presence.persist);
    esp_err_t err = nvs_get_blob(h, EDGE_PRESENCE_NVS_KEY, &edge_presence.persist, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(edge_presence.persist) ||
        edge_presence.persist.magic != EDGE_PRESENCE_MAGIC) {
        memset(&edge_presence.persist, 0, sizeof(edge_presence.persist));
        edge_presence.persist.magic = EDGE_PRESENCE_MAGIC;
        return false;
    }
    return true;
}

/* ---- init --------------------------------------------------------------- */

static inline void edge_presence_init(void) {
    memset(&edge_presence, 0, sizeof(edge_presence));
    edge_presence.persist.magic = EDGE_PRESENCE_MAGIC;

    const uint8_t mac[6] = EDGE_PHONE_MAC;
    memcpy(edge_presence.pinned_mac, mac, 6);
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0x00) { edge_presence.pinned = true; break; }
    }

    if (edge_presence.pinned) {
        ESP_LOGI(EDGE_PRESENCE_TAG, "pinned to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return;
    }

    if (edge_presence_load()) {
        int known = 0;
        for (int i = 0; i < EDGE_PRESENCE_MAX_CAND; i++) {
            if (edge_presence.persist.cand[i].used) known++;
        }
        if (edge_presence.persist.adopted) {
            const uint8_t *a = edge_presence.persist.adopted_mac;
            /* Printed every boot on purpose: this is how an installer confirms
             * the right handset is bound, without a display or an app. */
            ESP_LOGI(EDGE_PRESENCE_TAG,
                     "restored from NVS: %d device(s), %s device is "
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     known,
                     edge_presence.persist.registered ? "registered" : "adopted",
                     a[0], a[1], a[2], a[3], a[4], a[5]);
        } else {
            ESP_LOGI(EDGE_PRESENCE_TAG,
                     "restored from NVS: %d device(s), none bound yet", known);
        }
    } else {
        ESP_LOGI(EDGE_PRESENCE_TAG,
                 "no stored learning -- starting fresh, unknown resolves to HOME");
    }
}

/* ---- install-time registration ------------------------------------------ */

/*
 * Close the learning gap. Automatic adoption needs several observed
 * departures, so for the first days after install nothing is qualified and
 * the module can only answer HOME -- safe, but the away suppression that
 * prevents daytime false alarms is not yet working.
 *
 * REGISTER covers that window with one deliberate action by the installer
 * (not the resident): issue the command, then toggle the phone's WiFi off
 * and back on.
 *
 * Devices already associated when the command is issued are excluded, so the
 * signal is "which device just (re)appeared". The window then runs to
 * completion and binds only if EXACTLY ONE device appeared. An earlier
 * version bound the first arrival instead, and on this project's own bench
 * it promptly registered the wrong handset -- three devices were sitting on
 * the closed network and another one happened to re-associate a moment
 * before the phone did. Binding the wrong device is silent and disables
 * away suppression, so ambiguity must fail loudly rather than guess: with
 * two or more arrivals the attempt is refused and the installer is told to
 * retry with other devices powered down.
 */
/*
 * Bind a specific MAC outright. This is the escape hatch for the ambiguous
 * case: the window above refuses when several devices join at once and
 * prints what it saw, and the installer then names the right one from that
 * list. On this project's bench three devices re-associated together every
 * time, so automatic disambiguation could never have succeeded there --
 * refusing forever with no way forward would be a worse outcome than letting
 * a human decide.
 */
static inline void edge_presence_register_mac(const uint8_t *mac) {
    memcpy(edge_presence.persist.adopted_mac, mac, 6);
    edge_presence.persist.adopted    = true;
    edge_presence.persist.registered = true;
    edge_presence.adopted_last_seen_ms = 0;   /* unseen-yet resolves to HOME */
    edge_presence.reg_pending = false;
    edge_presence.dirty       = true;
    edge_presence_save();
    edge_presence.dirty = false;

    ESP_LOGI(EDGE_PRESENCE_TAG,
             "registered %02X:%02X:%02X:%02X:%02X:%02X as the resident's device",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* "AA:BB:CC:DD:EE:FF" (or with '-', or bare hex) -> 6 bytes. */
static inline bool edge_presence_parse_mac(const char *s, uint8_t *out) {
    int n = 0;
    unsigned v = 0;
    int digits = 0;
    for (const char *p = s; *p; p++) {
        char c = *p;
        int d;
        if      (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else if (c == ':' || c == '-') continue;
        else return false;

        v = (v << 4) | (unsigned) d;
        if (++digits == 2) {
            if (n >= 6) return false;
            out[n++] = (uint8_t) v;
            v = 0; digits = 0;
        }
    }
    return (n == 6 && digits == 0);
}

static inline void edge_presence_begin_registration(void) {
    int64_t now_ms = esp_timer_get_time() / 1000;

    for (int i = 0; i < EDGE_PRESENCE_MAX_CAND; i++) {
        edge_presence.reg_present_at_start[i] = edge_presence.present_last[i];
        edge_presence.reg_appeared[i]         = false;
    }
    edge_presence.reg_pending  = true;
    edge_presence.reg_until_ms = now_ms + EDGE_PRESENCE_REGISTER_MS;

    /* Any previous binding is cleared up front, so a failed attempt leaves
     * nothing bound rather than silently keeping a stale or wrong device.
     * Unbound falls back to tier 3, which resolves to HOME -- the safe way
     * to be wrong. */
    if (edge_presence.persist.adopted) {
        edge_presence.persist.adopted    = false;
        edge_presence.persist.registered = false;
        edge_presence.dirty = true;
        ESP_LOGI(EDGE_PRESENCE_TAG, "previous binding cleared");
    }

    ESP_LOGI(EDGE_PRESENCE_TAG,
             "registration open for %d s -- turn the resident's phone WiFi off and "
             "on now; keep other devices off the network",
             EDGE_PRESENCE_REGISTER_MS / 1000);
}

/* ---- polling ------------------------------------------------------------ */

/* Locate this MAC's slot, allocating one if there is room. -1 when the table
 * is full: that device is not a learning candidate, and since it can never
 * qualify it also cannot hold the system in a false HOME. */
static inline int edge_presence_slot(const uint8_t *mac) {
    int free_slot = -1;
    for (int i = 0; i < EDGE_PRESENCE_MAX_CAND; i++) {
        edge_presence_learned_t *c = &edge_presence.persist.cand[i];
        if (c->used) {
            if (memcmp(c->mac, mac, 6) == 0) return i;
        } else if (free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot >= 0) {
        edge_presence_learned_t *c = &edge_presence.persist.cand[free_slot];
        memcpy(c->mac, mac, 6);
        c->used        = true;
        c->transitions = 0;
        edge_presence.present_last[free_slot]     = false;
        edge_presence.cand_last_seen_ms[free_slot] = 0;
        edge_presence.dirty = true;
    }
    return free_slot;
}

/*
 * Poll the SoftAP's association table. Call once per decision cycle from
 * AP-mode firmware; a no-op if this board is not an AP.
 *
 * (Promiscuous/router-beacon mode has no association table to read, so a
 * second-zone board would instead need to match frame addresses in a
 * promiscuous callback. Not implemented -- registering such a callback on
 * this project once crashed the scheduler under real traffic, so it needs a
 * memcmp-only handler and a load test before it can be trusted.)
 */
static inline void edge_presence_poll_ap_stations(void) {
    wifi_sta_list_t sta_list;
    if (esp_wifi_ap_get_sta_list(&sta_list) != ESP_OK) return;

    int64_t now_ms = esp_timer_get_time() / 1000;
    bool present_now[EDGE_PRESENCE_MAX_CAND] = {false};

    for (int i = 0; i < sta_list.num; i++) {
        const uint8_t *mac = sta_list.sta[i].mac;

        if (edge_presence.pinned &&
            memcmp(mac, edge_presence.pinned_mac, 6) == 0) {
            edge_presence.pinned_last_seen_ms = now_ms;
        }
        if (edge_presence.persist.adopted &&
            memcmp(mac, edge_presence.persist.adopted_mac, 6) == 0) {
            edge_presence.adopted_last_seen_ms = now_ms;
        }

        int idx = edge_presence_slot(mac);
        if (idx >= 0) {
            present_now[idx] = true;
            edge_presence.cand_last_seen_ms[idx] = now_ms;

            /* Note arrivals during the window; the choice is made when it
             * closes, so a second arrival can still veto a wrong bind. */
            if (edge_presence.reg_pending && !edge_presence.reg_present_at_start[idx]) {
                edge_presence.reg_appeared[idx] = true;
            }

            /* Tier 3 counts only devices that have been seen to leave. An
             * always-on device never gets here, so it can never pin the
             * system to HOME. */
            if (edge_presence.persist.cand[idx].transitions > 0) {
                edge_presence.any_last_seen_ms = now_ms;
            }
        }
    }

    /* Count departures. "Absent" means past the timeout, not merely missing
     * from one poll -- otherwise a power-save gap would inflate the count and
     * a stationary device could qualify on noise alone. */
    for (int i = 0; i < EDGE_PRESENCE_MAX_CAND; i++) {
        edge_presence_learned_t *c = &edge_presence.persist.cand[i];
        if (!c->used) continue;

        bool absent = !present_now[i] &&
                      (now_ms - edge_presence.cand_last_seen_ms[i]) >= EDGE_PRESENCE_TIMEOUT_MS;

        if (edge_presence.present_last[i] && absent) {
            c->transitions++;
            edge_presence.present_last[i] = false;
            edge_presence.dirty = true;
            ESP_LOGI(EDGE_PRESENCE_TAG,
                     "%02X:%02X:%02X:%02X:%02X:%02X left (%u departures observed)",
                     c->mac[0], c->mac[1], c->mac[2], c->mac[3], c->mac[4], c->mac[5],
                     (unsigned) c->transitions);
        } else if (present_now[i]) {
            edge_presence.present_last[i] = true;
        }
    }

    if (edge_presence.reg_pending && now_ms >= edge_presence.reg_until_ms) {
        edge_presence.reg_pending = false;

        int n = 0, only = -1;
        for (int i = 0; i < EDGE_PRESENCE_MAX_CAND; i++) {
            if (edge_presence.reg_appeared[i]) { n++; only = i; }
        }

        if (n == 1) {
            const uint8_t *m = edge_presence.persist.cand[only].mac;
            memcpy(edge_presence.persist.adopted_mac, m, 6);
            edge_presence.persist.adopted    = true;
            edge_presence.persist.registered = true;
            edge_presence.adopted_last_seen_ms = edge_presence.cand_last_seen_ms[only];
            edge_presence.dirty = true;
            ESP_LOGI(EDGE_PRESENCE_TAG,
                     "registered %02X:%02X:%02X:%02X:%02X:%02X as the resident's device",
                     m[0], m[1], m[2], m[3], m[4], m[5]);
        } else if (n == 0) {
            ESP_LOGW(EDGE_PRESENCE_TAG,
                     "registration failed: no device re-joined. Nothing changed.");
        } else {
            /* Refuse rather than guess -- a wrong bind fails silently. */
            ESP_LOGW(EDGE_PRESENCE_TAG,
                     "registration failed: %d devices joined, cannot tell which is the "
                     "resident's. Power down other devices and run REGISTER again.", n);
            for (int i = 0; i < EDGE_PRESENCE_MAX_CAND; i++) {
                if (!edge_presence.reg_appeared[i]) continue;
                const uint8_t *m = edge_presence.persist.cand[i].mac;
                ESP_LOGW(EDGE_PRESENCE_TAG, "    joined: %02X:%02X:%02X:%02X:%02X:%02X",
                         m[0], m[1], m[2], m[3], m[4], m[5]);
            }
        }
    }

    /* Drop an adoption whose device has been gone far too long -- most likely
     * a rotated MAC or a replaced handset. Learning then restarts. */
    if (edge_presence.persist.adopted &&
        edge_presence.adopted_last_seen_ms != 0 &&
        (now_ms - edge_presence.adopted_last_seen_ms) >= EDGE_PRESENCE_READOPT_MS) {
        ESP_LOGW(EDGE_PRESENCE_TAG, "adopted device gone too long -- dropping, will re-learn");
        edge_presence.persist.adopted = false;
        edge_presence.dirty = true;
    }

    /*
     * Adopt the device that has proven it travels with someone -- and when
     * several qualify, the one with the MOST departures rather than whichever
     * reached the bar first. A visitor's phone can cross the threshold, but
     * the resident's phone leaves and returns every day and pulls ahead
     * permanently, so ranking is more robust than first-come. Registration
     * (above) outranks this entirely and is never overwritten by it.
     */
    if (!edge_presence.persist.adopted) {
        int best = -1;
        for (int i = 0; i < EDGE_PRESENCE_MAX_CAND; i++) {
            edge_presence_learned_t *c = &edge_presence.persist.cand[i];
            if (!c->used || c->transitions < EDGE_PRESENCE_ADOPT_TRANSITIONS) continue;
            if (best < 0 || c->transitions > edge_presence.persist.cand[best].transitions) {
                best = i;
            }
        }
        if (best >= 0) {
            edge_presence_learned_t *c = &edge_presence.persist.cand[best];
            memcpy(edge_presence.persist.adopted_mac, c->mac, 6);
            edge_presence.persist.adopted    = true;
            edge_presence.persist.registered = false;
            edge_presence.adopted_last_seen_ms = edge_presence.cand_last_seen_ms[best];
            edge_presence.dirty = true;
            ESP_LOGI(EDGE_PRESENCE_TAG,
                     "adopted %02X:%02X:%02X:%02X:%02X:%02X (%u departures) as the resident's device",
                     c->mac[0], c->mac[1], c->mac[2], c->mac[3], c->mac[4], c->mac[5],
                     (unsigned) c->transitions);
        }
    }

    /* Written only on a real change -- a few times a day at most, which keeps
     * this well clear of the flash erase budget. */
    if (edge_presence.dirty) {
        edge_presence_save();
        edge_presence.dirty = false;
    }
}

/* ---- query -------------------------------------------------------------- */

static inline bool edge_presence_seen_recently(int64_t last_seen_ms, int64_t now_ms) {
    if (last_seen_ms == 0) return true;   /* never seen yet -> assume HOME */
    return (now_ms - last_seen_ms) < EDGE_PRESENCE_TIMEOUT_MS;
}

static inline bool edge_presence_is_home(void) {
    int64_t now_ms = esp_timer_get_time() / 1000;

    if (edge_presence.pinned) {
        return edge_presence_seen_recently(edge_presence.pinned_last_seen_ms, now_ms);
    }
    if (edge_presence.persist.adopted) {
        return edge_presence_seen_recently(edge_presence.adopted_last_seen_ms, now_ms);
    }
    return edge_presence_seen_recently(edge_presence.any_last_seen_ms, now_ms);
}

/* ---- registration button ------------------------------------------------ */

static inline void edge_presence_button_init(void) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << EDGE_REG_BUTTON_GPIO;
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
}

/*
 * Call frequently (input_loop already ticks every 10 ms, so it rides along
 * there rather than costing a task). A hold of EDGE_REG_BUTTON_HOLD_MS opens
 * the registration window; the hold requirement is the debounce, and it also
 * keeps an accidental brush from re-binding a working installation.
 *
 * While the window is open the alert LED blinks, so an installer with no
 * laptop can still see that the device is listening. The blink only runs
 * when no alert is active -- a real alert owns the LED and must not be
 * masked by a setup indicator.
 */
static inline void edge_presence_button_poll(void) {
    int64_t now_ms = esp_timer_get_time() / 1000;
    bool pressed = (gpio_get_level(EDGE_REG_BUTTON_GPIO) == 0);

    if (pressed) {
        if (edge_presence.btn_down_since_ms == 0) {
            edge_presence.btn_down_since_ms = now_ms;
            edge_presence.btn_fired = false;
        } else if (!edge_presence.btn_fired &&
                   (now_ms - edge_presence.btn_down_since_ms) >= EDGE_REG_BUTTON_HOLD_MS) {
            edge_presence.btn_fired = true;
            edge_presence_begin_registration();
        }
    } else {
        edge_presence.btn_down_since_ms = 0;
        edge_presence.btn_fired = false;
    }

    if (alert_get_level() == ALERT_NONE) {
        /* Blink while listening, and drive it back off once the window
         * closes -- otherwise the LED could be left stuck on mid-blink. */
        gpio_set_level(ALERT_LED_GPIO,
                       (edge_presence.reg_pending && ((now_ms / 250) % 2)) ? 1 : 0);
    }
}

/* Which tier answered, for the status line. */
static inline const char *edge_presence_mode(void) {
    if (edge_presence.pinned)             return "pinned";
    if (edge_presence.reg_pending)        return "registering";
    if (edge_presence.persist.registered) return "registered";
    if (edge_presence.persist.adopted)    return "adopted";
    return "any";
}

#endif /* ESP32_CSI_EDGE_PRESENCE_H */
