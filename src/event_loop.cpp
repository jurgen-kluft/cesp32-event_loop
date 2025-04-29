#include "event_loop.h"

#include <freertos/semphr.h>

namespace cesp32 {
enum event_type_t {
    EVENT_TYPE_NONE = 0,       // No event
    EVENT_TYPE_DELAY = 0x11,   // TimedEvent -> Event
    EVENT_TYPE_REPEAT = 0x12,  // TimedEvent -> Event
    EVENT_TYPE_STREAM = 0x21,  // UnTimedEvent -> Event
    EVENT_TYPE_TICK = 0x22,    // UnTimedEvent -> Event
    EVENT_TYPE_ISR = 0x31,     // Event
};

struct timed_event_t {
    callback_t m_callback;
    uint64_t m_last_trigger_time;  // microseconds
    uint64_t m_interval;           // microseconds
    uint8_t m_type;
    uint8_t m_enabled;
};

struct untimed_event_t {
    callback_t m_callback;
    Stream *m_stream;
    uint8_t m_type;
    uint8_t m_enabled;
};

#ifdef ESP32
// set to true once gpio_install_isr_service is called
static bool isr_service_installed;
static void global_isr_event(void *this_ptr);
#endif

struct isr_event_t {
    callback_t m_callback;
    int m_mode;
    uint8_t m_enabled;
    uint8_t m_pin_number;
};

bool isValid(event_handle_t h) { return (h.type != 0 && h.index != 0); }

/**
 * @brief Return the current time since the device restart in microseconds
 *
 * Returns the time since the device restart. Even though the time
 * is in microseconds, a 64-bit integer is all but guaranteed not to
 * wrap, ever.
 */
inline uint64_t ICACHE_RAM_ATTR getNowUs() { return esp_timer_get_time(); }

event_loop_t::event_loop_t() {
    m_timed_events_array = nullptr;
    m_timed_events_active_array = nullptr;
    m_timed_events_free_array = nullptr;
    m_timed_events_active_count = 0;
    m_timed_events_free_count = 0;
    m_timed_events_capacity = 0;

    m_untimed_events_array = nullptr;
    m_untimed_events_active_count = 0;
    m_untimed_events_cap = 0;

    m_isr_events_array = nullptr;
    m_isr_events_active_count = 0;
    m_isr_events_cap = 0;

    m_timed_mutex_ = nullptr;
    m_untimed_mutex_ = nullptr;
    m_isr_mutex_ = nullptr;

    m_timed_counter = 0;
    m_untimed_counter = 0;
    m_tick_counter = 0;
}

void event_loop_t::setup(int timed_cap, int untimed_cap, int isr_cap) {
    m_timed_mutex_ = xSemaphoreCreateRecursiveMutex();
    m_untimed_mutex_ = xSemaphoreCreateRecursiveMutex();
    m_isr_mutex_ = xSemaphoreCreateRecursiveMutex();

    // Initialize the mutexes
    xSemaphoreGiveRecursive(m_timed_mutex_);
    xSemaphoreGiveRecursive(m_untimed_mutex_);
    xSemaphoreGiveRecursive(m_isr_mutex_);

    m_timed_counter = 0;
    m_untimed_counter = 0;
    m_tick_counter = 0;

    // Allocate the events
    m_timed_events_array = new timed_event_t[timed_cap];
    m_timed_events_active_array = new int[timed_cap];
    m_timed_events_free_array = new int[timed_cap];
    m_timed_events_active_count = 0;
    m_timed_events_free_count = timed_cap;
    m_timed_events_capacity = timed_cap;
    for (int i = 0; i < timed_cap; ++i) {
        timed_event_t &ev = m_timed_events_array[i];
        ev.m_callback = nullptr;
        ev.m_type = EVENT_TYPE_NONE;
        ev.m_enabled = 0;
        m_timed_events_active_array[i] = 0;
        m_timed_events_free_array[i] = i;
    }

    m_untimed_events_array = new untimed_event_t[untimed_cap];
    m_untimed_events_active_count = 0;
    m_untimed_events_cap = untimed_cap;
    for (int i = 0; i < (untimed_cap + 7) / 8; ++i) {
        untimed_event_t &ev = m_untimed_events_array[i];
        ev.m_callback = nullptr;
        ev.m_stream = nullptr;
        ev.m_type = EVENT_TYPE_NONE;
        ev.m_enabled = 0;
    }

    m_isr_events_array = new isr_event_t[isr_cap];
    m_isr_events_active_count = 0;
    m_isr_events_cap = isr_cap;
    for (int i = 0; i < isr_cap; ++i) {
        isr_event_t &ev = m_isr_events_array[i];
        ev.m_callback = nullptr;
        ev.m_enabled = 0;
        ev.m_pin_number = 0;
        ev.m_mode = 0;
    }
}

void event_loop_t::tickTimed() {
    xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);

    if (m_timed_events_active_count == 0) {
        xSemaphoreGiveRecursive(m_timed_mutex_);
        return;
    }

    // Get the current time in microseconds
    const uint64_t now = getNowUs();

    int i = 0;
    int w = 0;
    while (i < m_timed_events_active_count) {
        const int ti = m_timed_events_active_array[i];
        timed_event_t *te = &m_timed_events_array[ti];

        if (te->m_enabled == 0) {
            // Event is disabled, remove it from the active list and add it to
            // the free list
            m_timed_events_free_array[m_timed_events_free_count++] = ti;
            i++;
            continue;
        }

        if (now < (te->m_last_trigger_time + te->m_interval)) {
            // This event is not ready yet
            if (i > w) {
                m_timed_events_active_array[w] = ti;
            }
            w++;
            i++;
            continue;
        }

        // Handle event specific tick logic
        event_type_t const type = (event_type_t)te->m_type;
        switch (type) {
            case EVENT_TYPE_DELAY:
                te->m_last_trigger_time = getNowUs();
                te->m_callback();
                // event is done
                // remove it from the active list and add it to the free list
                m_timed_events_free_array[m_timed_events_free_count++] = ti;
                break;
            case EVENT_TYPE_REPEAT:
                te->m_last_trigger_time += te->m_interval;
                if ((te->m_last_trigger_time + te->m_interval) < now) {
                    // we're lagging more than one full interval;
                    te->m_last_trigger_time = now;  // reset the time
                }
                te->m_callback();

                if (i > w) {
                    m_timed_events_active_array[w] = ti;
                }

                w++;
                break;
        }

        i++;
        m_timed_counter++;
    }
    m_timed_events_active_count = w;
    xSemaphoreGiveRecursive(m_timed_mutex_);
}

void event_loop_t::tickUntimed() {
    xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);
    {
        int w = 0;
        for (int i = 0; i < m_untimed_events_active_count; ++i) {
            untimed_event_t &ev = m_untimed_events_array[i];
            const bool enabled = ev.m_enabled != 0;
            if (enabled) {
                const event_type_t type = (event_type_t)ev.m_type;
                if (type == EVENT_TYPE_STREAM) {
                    Stream *stream = ev.m_stream;
                    if (stream != nullptr && stream->available()) {
                        ev.m_callback();
                    }
                } else if (type == EVENT_TYPE_TICK) {
                    ev.m_callback();
                }
                w++;
            }

            if (i > w) {
                m_untimed_events_array[w] = m_untimed_events_array[i];
            }
        }

        // Some events might have been removed/disabled, so we need to update
        // the size
        m_untimed_events_active_count = w;
    }
    xSemaphoreGiveRecursive(m_untimed_mutex_);
}

void event_loop_t::tick() {
    tickUntimed();
    tickTimed();
    m_tick_counter++;
}

event_handle_t event_loop_t::onDelay(uint32_t delay, callback_t callback) {
    return onDelayMicros((uint64_t)delay * 1000, callback);
}

event_handle_t event_loop_t::onDelayMicros(uint64_t delay,
                                           callback_t callback) {
    xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);

    if (m_timed_events_free_count == 0) {
        // No free space in the timed event list
        xSemaphoreGiveRecursive(m_timed_mutex_);
        return event_handle_t();
    }

    int const i = m_timed_events_free_array[--m_timed_events_free_count];
    m_timed_events_active_array[m_timed_events_active_count++] = i;

    timed_event_t *te = &m_timed_events_array[i];
    te->m_type = EVENT_TYPE_DELAY;
    te->m_enabled = 1;
    te->m_callback = callback;
    te->m_interval = delay;
    te->m_last_trigger_time = getNowUs();

    event_handle_t h;
    h.type = EVENT_TYPE_DELAY;
    h.index = i;

    xSemaphoreGiveRecursive(m_timed_mutex_);
    return h;
}

event_handle_t event_loop_t::onRepeat(uint32_t interval, callback_t callback) {
    return onRepeatMicros((uint64_t)interval * 1000, callback);
}

event_handle_t event_loop_t::onRepeatMicros(uint64_t interval,
                                            callback_t callback) {
    xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);

    if (m_timed_events_free_count == 0) {
        // No free space in the timed event list
        xSemaphoreGiveRecursive(m_timed_mutex_);
        return event_handle_t();
    }

    int const i = m_timed_events_free_array[m_timed_events_free_count - 1];
    m_timed_events_active_array[m_timed_events_active_count] = i;

    m_timed_events_free_count--;
    m_timed_events_active_count++;

    timed_event_t *te = &m_timed_events_array[i];
    te->m_type = EVENT_TYPE_REPEAT;
    te->m_enabled = 1;
    te->m_callback = callback;

    te->m_interval = interval;
    te->m_last_trigger_time = getNowUs();

    xSemaphoreGiveRecursive(m_timed_mutex_);

    Serial.println("Repeat Event: " + String(te->m_last_trigger_time) + String(te->m_interval));

    event_handle_t h;
    h.type = EVENT_TYPE_REPEAT;
    h.index = i;
    return h;
}

event_handle_t event_loop_t::onAvailable(Stream *stream, callback_t callback) {
    xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);

    if (m_untimed_events_active_count >= m_untimed_events_cap) {
        // No free space in the untimed event list
        xSemaphoreGiveRecursive(m_untimed_mutex_);
        return event_handle_t();
    }

    int const i = m_untimed_events_active_count++;

    untimed_event_t &ev = m_untimed_events_array[i];
    ev.m_callback = callback;
    ev.m_stream = stream;
    ev.m_type = EVENT_TYPE_STREAM;
    ev.m_enabled = 1;

    xSemaphoreGiveRecursive(m_untimed_mutex_);

    event_handle_t h;
    h.type = EVENT_TYPE_STREAM;
    h.index = i;
    return h;
}

static void global_isr_event(void *callback_ptr) {
    isr_event_t *ev = static_cast<isr_event_t *>(callback_ptr);
    if (ev->m_enabled != 0) {
        ev->m_callback();
    }
}

event_handle_t event_loop_t::onInterrupt(uint8_t pin_number, int mode,
                                         callback_t callback) {
    xSemaphoreTakeRecursive(m_isr_mutex_, portMAX_DELAY);

    int index = -1;
    for (int i = 0; i < m_isr_events_cap; ++i) {
        if (m_isr_events_array[i].m_enabled == 0) {
            index = i;
            break;
        }
    }

    if (index >= 0) {
        m_isr_events_array[index].m_pin_number = pin_number;
        m_isr_events_array[index].m_mode = mode;
        m_isr_events_array[index].m_callback = callback;
        m_isr_events_array[index].m_enabled = 1;
        m_isr_events_active_count++;

#ifdef ESP32
        gpio_int_type_t intr_type;
        switch (mode) {
            case RISING:
                intr_type = GPIO_INTR_POSEDGE;
                break;
            case FALLING:
                intr_type = GPIO_INTR_NEGEDGE;
                break;
            case CHANGE:
                intr_type = GPIO_INTR_ANYEDGE;
                break;
            default:
                intr_type = GPIO_INTR_DISABLE;
                break;
        }

        // configure the IO pin
        gpio_set_intr_type((gpio_num_t)pin_number, intr_type);

        if (!isr_service_installed) {
            isr_service_installed = true;
            gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
        }

        gpio_isr_handler_add((gpio_num_t)pin_number, global_isr_event,
                             (void *)&m_isr_events_array[index]);

#elif defined(ESP8266)
        attachInterrupt(digitalPinToInterrupt(pin_number), callback, mode);
#endif
    }
    xSemaphoreGiveRecursive(m_isr_mutex_);

    if (index == -1) {
        // No free space in the ISR event list
        return event_handle_t();
    }

    event_handle_t h;
    h.type = EVENT_TYPE_ISR;
    h.index = index;
    return h;
}

event_handle_t event_loop_t::onTick(callback_t callback) {
    xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);

    if (m_untimed_events_active_count >= m_untimed_events_cap) {
        // No free space in the untimed event list
        xSemaphoreGiveRecursive(m_untimed_mutex_);
        return event_handle_t();
    }

    int const i = m_untimed_events_active_count++;

    untimed_event_t &ev = m_untimed_events_array[i];
    ev.m_callback = callback;
    ev.m_stream = nullptr;
    ev.m_type = EVENT_TYPE_TICK;
    ev.m_enabled = 1;

    xSemaphoreGiveRecursive(m_untimed_mutex_);

    event_handle_t h;
    h.type = EVENT_TYPE_TICK;
    h.index = i;
    return h;
}

void event_loop_t::remove(event_handle_t eh) {
    if (!isValid(eh)) {
        return;
    }

    if (eh.type == EVENT_TYPE_DELAY || eh.type == EVENT_TYPE_REPEAT) {
        xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);
        timed_event_t &ev = m_timed_events_array[eh.index];
        ev.m_enabled = 0;
        ev.m_callback = nullptr;
        m_timed_events_active_count--;
        m_timed_events_free_array[m_timed_events_free_count++] = eh.index;
        xSemaphoreGiveRecursive(m_timed_mutex_);
    } else if (eh.type == EVENT_TYPE_STREAM || eh.type == EVENT_TYPE_TICK) {
        xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);
        // m_untimed_events_enabled_array[ev.index / 8] &= ~(1 << (ev.index &
        // 7));
        untimed_event_t &ev = m_untimed_events_array[eh.index];
        ev.m_enabled = 0;
        ev.m_callback = nullptr;
        m_untimed_events_active_count--;
        xSemaphoreGiveRecursive(m_untimed_mutex_);
    } else if (eh.type == EVENT_TYPE_ISR) {
        xSemaphoreTakeRecursive(m_isr_mutex_, portMAX_DELAY);
        m_isr_events_array[eh.index].m_enabled = 0;
        m_isr_events_active_count--;

        // Check if there are any other events using the same pin
        bool found = false;
        for (int i = 0; i < m_isr_events_cap; ++i) {
            if (m_isr_events_array[i].m_enabled != 0 &&
                m_isr_events_array[i].m_pin_number ==
                    m_isr_events_array[eh.index].m_pin_number) {
                found = true;
                break;
            }
        }

#ifdef ESP32
        if (!found) {
            gpio_isr_handler_remove(
                (gpio_num_t)m_isr_events_array[eh.index].m_pin_number);
        }
#elif defined(ESP8266)
        if (!found) {
            detachInterrupt(m_isr_events_array[eh.index].m_pin_number);
        }
#endif
        xSemaphoreGiveRecursive(m_isr_mutex_);
    }
}

}  // namespace cesp32