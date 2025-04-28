#include "event_loop.h"

#include <freertos/semphr.h>

namespace cesp32
{
    enum event_type_t
    {
        EVENT_TYPE_NONE = 0, // No event
        EVENT_TYPE_DELAY,    // TimedEvent -> Event
        EVENT_TYPE_REPEAT,   // TimedEvent -> Event
        EVENT_TYPE_STREAM,   // UnTimedEvent -> Event
        EVENT_TYPE_TICK,     // UnTimedEvent -> Event
        EVENT_TYPE_ISR,      // Event
    };

    struct timed_event_t
    {
        callback_t m_callback;
        uint64_t m_interval;
        uint64_t m_last_trigger_time;
        uint64_t getTriggerTime() const { return (m_last_trigger_time + m_interval) / 1000; }
        uint64_t getTriggerTimeMicros() const { return m_last_trigger_time + m_interval; }
        uint8_t m_type;
        uint8_t m_enabled;
    };

    struct untimed_event_t
    {
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

    struct isr_event_t
    {
        callback_t m_callback;
        int m_mode;
        uint8_t m_enabled;
        uint8_t m_pin_number;
    };

    bool isValid(event_handle_t h)
    {
        return (h.type != 0 && h.index != 0);
    }

    /**
     * @brief Return the current time since the device restart in microseconds
     *
     * Returns the time since the device restart. Even though the time
     * is in microseconds, a 64-bit integer is all but guaranteed not to
     * wrap, ever.
     */
    inline uint64_t ICACHE_RAM_ATTR micros64() { return esp_timer_get_time(); }

    event_loop_t::event_loop_t(int timed_queue_cap, int untimed_list_cap,
                               int isr_event_list_cap)
    {
        m_timed_mutex_ = xSemaphoreCreateRecursiveMutex();
        m_untimed_mutex_ = xSemaphoreCreateRecursiveMutex();
        m_isr_mutex_ = xSemaphoreCreateRecursiveMutex();

        // Initialize the mutexes
        xSemaphoreGiveRecursive(m_timed_mutex_);
        xSemaphoreGiveRecursive(m_untimed_mutex_);
        xSemaphoreGiveRecursive(m_isr_mutex_);

        // Allocate the events
        m_timed_events_array = new timed_event_t[timed_queue_cap];
        m_timed_events_active_array = new int[timed_queue_cap];
        m_timed_events_free_array = new int[timed_queue_cap];
        m_timed_events_active_count = 0;
        m_timed_events_free_count = timed_queue_cap;
        m_timed_events_capacity = timed_queue_cap;
        for (int i = 0; i < timed_queue_cap; ++i)
        {
            timed_event_t &ev = m_timed_events_array[i];
            ev.m_callback = nullptr;
            ev.m_type = EVENT_TYPE_NONE;
            ev.m_enabled = 0;
            m_timed_events_active_array[i] = 0;
            m_timed_events_free_array[i] = i;
        }

        m_untimed_events_array = new untimed_event_t[untimed_list_cap];
        m_untimed_events_size = 0;
        m_untimed_events_cap = untimed_list_cap;
        for (int i = 0; i < (untimed_list_cap + 7) / 8; ++i)
        {
            untimed_event_t &ev = m_untimed_events_array[i];
            ev.m_callback = nullptr;
            ev.m_stream = nullptr;
            ev.m_type = EVENT_TYPE_NONE;
            ev.m_enabled = 0;
        }

        m_isr_events_array = new isr_event_t[isr_event_list_cap];
        m_isr_events_count = 0;
        m_isr_events_cap = isr_event_list_cap;
        for (int i = 0; i < isr_event_list_cap; ++i)
        {
            m_isr_events_array[i].m_callback = nullptr;
            m_isr_events_array[i].m_enabled = 0;
            m_isr_events_array[i].m_pin_number = 0;
            m_isr_events_array[i].m_mode = 0;
        }
    }

    void event_loop_t::tickTimed()
    {
        xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);
        const uint64_t now = micros64();
        int top = -1;

        // Sort the timed queue by trigger time
        timed_event_t *timed_events_array = m_timed_events_array;
        std::sort(&m_timed_events_active_array[0], &m_timed_events_active_array[m_timed_events_active_count], [timed_events_array](const int &a, const int &b)
                  { 
            // return (this->last_trigger_time + this->interval) < (other.last_trigger_time + other.interval);            
            return timed_events_array[a].m_last_trigger_time + timed_events_array[a].m_interval < timed_events_array[b].m_last_trigger_time + timed_events_array[b].m_interval; });

        int i = 0;
        int w = 0;
        while (i < m_timed_events_active_count)
        {
            int const ti = m_timed_events_active_array[i];
            if (m_timed_events_array[ti].m_enabled == 0)
            {
                // Event is disabled, remove it from the active list and add it to the free list
                m_timed_events_free_array[m_timed_events_free_count++] = ti;
                ++i;
                continue;
            }

            timed_event_t *te = &m_timed_events_array[ti];
            const uint64_t trigger_t = te->getTriggerTimeMicros();
            if (now < trigger_t)
            {
                // This event (and the rest of the events) are not ready yet
                if (i > w)
                {
                    m_timed_events_active_array[w] = ti;
                }

                w++;
                i++;
                continue;
            }

            // Handle event specific tick logic
            event_type_t const type = (event_type_t)m_timed_events_array[ti].m_type;
            switch (type)
            {
            case EVENT_TYPE_DELAY:
                te->m_last_trigger_time = micros64();
                te->m_callback();

                i++;
                break;
            case EVENT_TYPE_REPEAT:
                te->m_last_trigger_time = te->m_last_trigger_time + te->m_interval;
                if (te->m_last_trigger_time + te->m_interval < now)
                {
                    te->m_last_trigger_time = now; // we're lagging more than one full interval; reset the time
                }
                te->m_callback();

                if (i > w)
                {
                    m_timed_events_active_array[w] = ti;
                }

                w++;
                i++;
                break;
            }

            m_timed_counter++;
        }
        m_timed_events_active_count = w;
        xSemaphoreGiveRecursive(m_timed_mutex_);
    }

    void event_loop_t::tickUntimed()
    {
        xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);
        {
            int w = 0;
            for (int i = 0; i < m_untimed_events_size; ++i, ++w)
            {
                const bool enabled = m_untimed_events_array[i].m_enabled != 0;
                if (enabled)
                {
                    const event_type_t type = (event_type_t)m_untimed_events_array[i].m_type;
                    if (type == EVENT_TYPE_STREAM)
                    {
                        Stream *stream = m_untimed_events_array[i].m_stream;
                        if (stream != nullptr && stream->available())
                        {
                            m_untimed_events_array[i].m_callback();
                        }
                    }
                    else if (type == EVENT_TYPE_TICK)
                    {
                        m_untimed_events_array[i].m_callback();
                    }
                }

                if (i > w)
                {
                    m_untimed_events_array[w] = m_untimed_events_array[i];
                }
            }

            // Some events might have been removed/disabled, so we need to update the size
            m_untimed_events_size = w;
        }
        xSemaphoreGiveRecursive(m_untimed_mutex_);
    }

    void event_loop_t::tick()
    {
        tickUntimed();
        tickTimed();
        m_tick_counter++;
    }

    event_handle_t event_loop_t::onDelay(uint32_t delay, callback_t callback)
    {
        return onDelayMicros(delay * 1000, callback);
    }

    event_handle_t event_loop_t::onDelayMicros(uint64_t delay, callback_t callback)
    {
        xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);

        if (m_timed_events_free_count == 0)
        {
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
        te->m_last_trigger_time = micros64();

        event_handle_t h;
        h.type = EVENT_TYPE_DELAY;
        h.index = i;

        xSemaphoreGiveRecursive(m_timed_mutex_);
        return h;
    }

    event_handle_t event_loop_t::onRepeat(uint32_t interval, callback_t callback)
    {
        return onRepeatMicros(interval * 1000, callback);
    }

    event_handle_t event_loop_t::onRepeatMicros(uint64_t interval,
                                                callback_t callback)
    {
        xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);

        if (m_timed_events_free_count == 0)
        {
            // No free space in the timed event list
            xSemaphoreGiveRecursive(m_timed_mutex_);
            return event_handle_t();
        }

        int const i = m_timed_events_free_array[--m_timed_events_free_count];
        m_timed_events_active_array[m_timed_events_active_count++] = i;

        timed_event_t *te = &m_timed_events_array[i];
        te->m_type = EVENT_TYPE_REPEAT;
        te->m_enabled = 1;
        te->m_callback = callback;

        te->m_interval = interval;
        te->m_last_trigger_time = micros64();

        xSemaphoreGiveRecursive(m_timed_mutex_);

        event_handle_t h;
        h.type = EVENT_TYPE_REPEAT;
        h.index = i;
        return h;
    }

    event_handle_t event_loop_t::onAvailable(Stream *stream, callback_t callback)
    {
        xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);

        if (m_untimed_events_size >= m_untimed_events_cap)
        {
            // No free space in the untimed event list
            xSemaphoreGiveRecursive(m_untimed_mutex_);
            return event_handle_t();
        }

        int const i = m_untimed_events_size++;

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

    static void global_isr_event(void *callback_ptr)
    {
        isr_event_t *ev = static_cast<isr_event_t *>(callback_ptr);
        if (ev->m_enabled != 0)
        {
            ev->m_callback();
        }
    }

    event_handle_t event_loop_t::onInterrupt(uint8_t pin_number, int mode,
                                             callback_t callback)
    {
        xSemaphoreTakeRecursive(m_isr_mutex_, portMAX_DELAY);

        int index = -1;
        for (int i = 0; i < m_isr_events_cap; ++i)
        {
            if (m_isr_events_array[i].m_enabled == 0)
            {
                index = i;
                break;
            }
        }

        if (index >= 0)
        {
            m_isr_events_array[index].m_pin_number = pin_number;
            m_isr_events_array[index].m_mode = mode;
            m_isr_events_array[index].m_callback = callback;
            m_isr_events_array[index].m_enabled = 1;
            m_isr_events_count++;

#ifdef ESP32
            gpio_int_type_t intr_type;
            switch (mode)
            {
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

            if (!isr_service_installed)
            {
                isr_service_installed = true;
                gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
            }

            gpio_isr_handler_add((gpio_num_t)pin_number, global_isr_event, (void *)&m_isr_events_array[index]);

#elif defined(ESP8266)
            attachInterrupt(digitalPinToInterrupt(pin_number), callback, mode);
#endif
        }
        xSemaphoreGiveRecursive(m_isr_mutex_);

        if (index == -1)
        {
            // No free space in the ISR event list
            return event_handle_t();
        }

        event_handle_t h;
        h.type = EVENT_TYPE_ISR;
        h.index = index;
        return h;
    }

    event_handle_t event_loop_t::onTick(callback_t callback)
    {
        xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);

        if (m_untimed_events_size >= m_untimed_events_cap)
        {
            // No free space in the untimed event list
            xSemaphoreGiveRecursive(m_untimed_mutex_);
            return event_handle_t();
        }

        int const i = m_untimed_events_size++;

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

    void event_loop_t::remove(event_handle_t eh)
    {
        if (!isValid(eh))
        {
            return;
        }

        if (eh.type == EVENT_TYPE_DELAY || eh.type == EVENT_TYPE_REPEAT)
        {
            xSemaphoreTakeRecursive(m_timed_mutex_, portMAX_DELAY);
            timed_event_t& ev = m_timed_events_array[eh.index];
            ev.m_enabled = 0;
            ev.m_callback = nullptr;
            m_timed_events_active_count--;
            m_timed_events_free_array[m_timed_events_free_count++] = eh.index;
            xSemaphoreGiveRecursive(m_timed_mutex_);
        }
        else if (eh.type == EVENT_TYPE_STREAM || eh.type == EVENT_TYPE_TICK)
        {
            xSemaphoreTakeRecursive(m_untimed_mutex_, portMAX_DELAY);
            //m_untimed_events_enabled_array[ev.index / 8] &= ~(1 << (ev.index & 7));
            untimed_event_t& ev = m_untimed_events_array[eh.index];
            ev.m_enabled = 0;
            ev.m_callback = nullptr;    
            m_untimed_events_size--;
            xSemaphoreGiveRecursive(m_untimed_mutex_);
        }
        else if (eh.type == EVENT_TYPE_ISR)
        {
            xSemaphoreTakeRecursive(m_isr_mutex_, portMAX_DELAY);
            m_isr_events_array[eh.index].m_enabled = 0;
            m_isr_events_count--;

            // Check if there are any other events using the same pin
            bool found = false;
            for (int i = 0; i < m_isr_events_cap; ++i)
            {
                if (m_isr_events_array[i].m_enabled != 0 && m_isr_events_array[i].m_pin_number == m_isr_events_array[eh.index].m_pin_number)
                {
                    found = true;
                    break;
                }
            }

#ifdef ESP32
            if (!found)
            {
                gpio_isr_handler_remove((gpio_num_t)m_isr_events_array[eh.index].m_pin_number);
            }
#elif defined(ESP8266)
            if (!found)
            {
                detachInterrupt(m_isr_events_array[eh.index].m_pin_number);
            }
#endif
            xSemaphoreGiveRecursive(m_isr_mutex_);
        }
    }

} // namespace reactesp