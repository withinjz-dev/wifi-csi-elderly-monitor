#ifndef ESP32_CSI_ALERT_COMPONENT_H
#define ESP32_CSI_ALERT_COMPONENT_H

#include "driver/gpio.h"
#include "driver/ledc.h"

#define ALERT_LED_GPIO    GPIO_NUM_4
#define ALERT_BUZZER_GPIO GPIO_NUM_5

// Passive buzzers need an oscillating signal (not just a static HIGH) to
// produce sound, so the buzzer is driven with PWM at an audible tone.
#define ALERT_BUZZER_LEDC_TIMER   LEDC_TIMER_0
#define ALERT_BUZZER_LEDC_CHANNEL LEDC_CHANNEL_0
#define ALERT_BUZZER_TONE_HZ      2700
#define ALERT_BUZZER_DUTY_RES     LEDC_TIMER_10_BIT
#define ALERT_BUZZER_DUTY_ON      512  // 50% of 2^10

/*
 * Two distinct annunciations, not one.
 *
 * A live test surfaced why: when the TX board lost power the receiver saw
 * sustained NO_DATA, the streak hit the alert threshold, and the buzzer went
 * off as a full medical emergency. Nobody had stopped breathing -- the sensor
 * had simply gone blind. Treating "I cannot see" as "the person is in trouble"
 * trains a caregiver to ignore the alarm, which is the one failure that
 * silently disables the whole product.
 *
 *   EMERGENCY : LED + buzzer -- sustained absence of respiration in a room the
 *               device can still see.
 *   FAULT     : LED only, no buzzer -- the device cannot see. Needs attention,
 *               but it is a maintenance call, not a rescue.
 */
typedef enum {
    ALERT_NONE = 0,
    ALERT_FAULT,
    ALERT_EMERGENCY,
} alert_level_t;

static alert_level_t alert_state = ALERT_NONE;

void alert_init() {
    alert_state = ALERT_NONE;
    gpio_reset_pin(ALERT_LED_GPIO);
    gpio_set_direction(ALERT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALERT_LED_GPIO, 0);

    ledc_timer_config_t buzzer_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = ALERT_BUZZER_DUTY_RES,
            .timer_num = ALERT_BUZZER_LEDC_TIMER,
            .freq_hz = ALERT_BUZZER_TONE_HZ,
            .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&buzzer_timer);

    ledc_channel_config_t buzzer_channel = {
            .gpio_num = ALERT_BUZZER_GPIO,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = ALERT_BUZZER_LEDC_CHANNEL,
            .timer_sel = ALERT_BUZZER_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
    };
    ledc_channel_config(&buzzer_channel);
}

void alert_set_level(alert_level_t level) {
    alert_state = level;
    gpio_set_level(ALERT_LED_GPIO, (level != ALERT_NONE) ? 1 : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ALERT_BUZZER_LEDC_CHANNEL,
                  (level == ALERT_EMERGENCY) ? ALERT_BUZZER_DUTY_ON : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ALERT_BUZZER_LEDC_CHANNEL);
}

alert_level_t alert_get_level() {
    return alert_state;
}

/* Retained so the legacy serial ALERT_ON/ALERT_OFF commands still work in
 * capture mode and for bench-testing the buzzer. */
void alert_set(bool on) {
    alert_set_level(on ? ALERT_EMERGENCY : ALERT_NONE);
}

bool alert_is_on() {
    return alert_state != ALERT_NONE;
}

#endif //ESP32_CSI_ALERT_COMPONENT_H
