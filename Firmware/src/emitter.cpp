// emitter.cpp: IR emitter post.
//
// Drives the TSAL6200 through the 2N3904 with a 38 kHz carrier that is switched
// on and off in bursts (600 us on, 600 us off by default). The receiver module
// ignores continuous 38 kHz, so the bursts matter.
//
// Wiring (from the build plan, unverified until you test it):
//   D2 (GPIO4) -> 1k -> 2N3904 base
//   3V3 -> 39 ohm -> LED anode, LED cathode -> collector, emitter -> GND
//
// The carrier comes from the LEDC peripheral (hardware, no jitter). The burst
// on/off switching comes from a periodic esp_timer, which can wobble by tens of
// microseconds. The receiver does not care about that. If you ever see it
// matter on a scope, move the burst gating to the RMT peripheral.

#include <Arduino.h>
#include "driver/ledc.h"
#include "esp_timer.h"
#include "gate_core.h"

static const int LED_PIN = D2;
static const uint32_t CARRIER_HZ = 38000;
static const uint32_t DUTY_BITS = 8;
static const uint32_t DUTY_ON = 85;  // 85/256, about 33 %: standard for IR remotes, easy on the LED

static gate::Config cfg;
static esp_timer_handle_t burst_timer;
static volatile bool burst_on = false;

static void ledc_setup() {
    ledc_timer_config_t t = {};
    t.speed_mode = LEDC_LOW_SPEED_MODE;  // the C3 only has low-speed mode
    t.duty_resolution = LEDC_TIMER_8_BIT;
    t.timer_num = LEDC_TIMER_0;
    t.freq_hz = CARRIER_HZ;
    t.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&t);

    ledc_channel_config_t c = {};
    c.gpio_num = LED_PIN;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = LEDC_CHANNEL_0;
    c.timer_sel = LEDC_TIMER_0;
    c.duty = 0;
    c.hpoint = 0;
    ledc_channel_config(&c);
}

static void set_duty(uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void burst_tick(void*) {
    burst_on = !burst_on;
    set_duty(burst_on ? DUTY_ON : 0);
}

void setup() {
    Serial.begin(115200);
    ledc_setup();

    esp_timer_create_args_t args = {};
    args.callback = &burst_tick;
    args.name = "burst";
    args.dispatch_method = ESP_TIMER_TASK;
    esp_timer_create(&args, &burst_timer);
    esp_timer_start_periodic(burst_timer, cfg.burst_period_us / 2);

    Serial.printf("emitter: %u Hz carrier, %u us burst period, LED on GPIO%d\n",
                  (unsigned)CARRIER_HZ, (unsigned)cfg.burst_period_us, (int)LED_PIN);
}

void loop() {
    // Nothing to do. Print a heartbeat so you can tell it is alive.
    delay(5000);
    Serial.println("emitter: running");
}
