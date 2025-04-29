#include <Arduino.h>
#include <event_loop.h>

using namespace cesp32;

#define LED_PIN LED_BUILTIN
//#define DEBUG

#ifdef DEBUG
#define PRINTLN_DEBUG(msg) Serial.println(msg)
#else
#define PRINTLN_DEBUG(msg)
#endif

volatile uint8_t led_state = 0;
volatile int64_t event_counter = 0;

event_loop_t event_loop;

#pragma GCC optimize "Og"
void quick() {
    led_state = 1 - led_state;
    event_counter += 1;
}

void slow() {
    led_state = 1 - led_state;
    event_counter += 1;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    event_loop.setup();

    PRINTLN_DEBUG("Starting");

    event_loop.onRepeat(2000, quick);
    event_loop.onRepeat(4200, slow);

    PRINTLN_DEBUG("Setup done...");
}

volatile bool led_high = false;
void loop() {
    event_loop.tick();

    if (led_state == 1) {
        if (!led_high) {
            led_high = true;
            digitalWrite(LED_PIN, HIGH);
            PRINTLN_DEBUG("On");
        }
    } else if (led_state == 0) {
        if (led_high) {
            led_high = false;
            digitalWrite(LED_PIN, HIGH);
            PRINTLN_DEBUG("Off");
        }
    }
    delay(1);
}
