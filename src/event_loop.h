#ifndef CESP32_EVENT_LOOP_H_
#define CESP32_EVENT_LOOP_H_

#include <Arduino.h>

#include <functional>
#include <memory>

namespace cesp32
{
    struct timed_event_t;
    struct untimed_event_t;
    struct isr_event_t;

    struct event_handle_t
    {
        event_handle_t() : type(0), index(0) {}
        bool isValid() const;
        uint16_t type;
        uint16_t index;
    };

    using callback_t = std::function<void()>;
    using isr_callback_t = void (*)(void *);

    /**
     * @brief Asynchronous event loop supporting timed (repeating and
     * non-repeating), interrupt and stream events.
     */
    class event_loop_t
    {
    public:
        event_loop_t(int timed_queue_cap = 10,
                     int untimed_list_cap = 10,
                     int isr_event_list_cap = 10);

        // Disabling copy constructors
        event_loop_t(const event_loop_t &) = delete;
        event_loop_t(event_loop_t &&) = delete;

        int getTimedEventQueueSize() const { return m_timed_events_active_count; }
        int getUntimedEventQueueSize() const { return m_untimed_events_size; }
        int getISREventQueueSize() const { return m_isr_events_count; }
        int getEventQueueSize() const
        {
            return getTimedEventQueueSize() + getUntimedEventQueueSize() +
                   getISREventQueueSize();
        }

        uint64_t getTimedEventCount() const { return m_timed_counter; }
        uint64_t getUntimedEventCount() const { return m_untimed_counter; }
        uint64_t getEventCount() const
        {
            return getTimedEventCount() + getUntimedEventCount();
        }

        uint64_t getTickCount() const { return m_tick_counter; }

        void tick();

        /**
         * @brief Create a new event that will be called after a delay
         *
         * @param delay Delay, in milliseconds
         * @param callback Callback function
         * @return event id
         */
        event_handle_t onDelay(uint32_t delay, callback_t callback);

        /**
         * @brief Create a new event that will be called after a delay
         *
         * @param delay Delay, in microseconds
         * @param callback Callback function
         * @return event id
         */
        event_handle_t onDelayMicros(uint64_t delay, callback_t callback);

        /**
         * @brief Update the delay of an existing event
         *
         * @param eh Event handle of the event to update
         * @param delay Delay, in milliseconds
         */
        void onDelayChange(event_handle_t eh, uint32_t delay);

        /**
         * @brief Update the delay of an existing event
         *
         * @param eh Event handle of the event to update
         * @param delay Delay, in microseconds
         */
        void onDelayMicrosChange(event_handle_t eh, uint64_t delay);

        /**
         * @brief Create a new event that will be called at regular intervals
         *
         * @param delay Interval, in milliseconds
         * @param callback Callback function
         * @return event id
         */
        event_handle_t onRepeat(uint32_t interval, callback_t callback);

        /**
         * @brief Create a new event that will be called at regular intervals
         *
         * @param delay Interval, in microseconds
         * @param callback Callback function
         * @return event id
         */
        event_handle_t onRepeatMicros(uint64_t interval, callback_t callback);

        /**
         * @brief Update the interval of an existing event
         *
         * @param eh Event handle of the event to update
         * @param interval Interval, in milliseconds
         */
        void onRepeatIntervalChange(event_handle_t eh, uint32_t interval);

        /**
         * @brief Update the interval of an existing event
         *
         * @param eh Event handle of the event to update
         * @param interval Interval, in microseconds
         */
        void onRepeatMicrosIntervalChange(event_handle_t eh, uint64_t interval);

        /**
         * @brief Create a new event that will be called when data is available on a stream
         *
         * @param stream Arduino Stream object to monitor
         * @param callback Callback function
         * @return event id
         */
        event_handle_t onAvailable(Stream *stream, callback_t callback);

        /**
         * @brief Create a new event that will be called when an interrupt occurs
         *
         * @param pin_number GPIO pin number
         * @param mode One of CHANGE, RISING, FALLING
         * @param callback Interrupt handler to call. This should be a very simple
         * function, ideally setting a flag variable or incrementing a counter. The
         * function should be defined with ICACHE_RAM_ATTR.
         * @return event id
         */
        event_handle_t onInterrupt(uint8_t pin_number, int mode, callback_t callback);

        /**
         * @brief Create a new event that will be called at every loop execution
         *
         * @param callback Callback function to be called at every loop execution
         * @return event id
         */
        event_handle_t onTick(callback_t callback);

        /**
         * @brief Remove an event from the list of active events
         *
         * @param event Event to remove
         */
        void remove(event_handle_t event);

    protected:
        // Timed events are managed by C style array's.
        timed_event_t *m_timed_events_array;
        int *m_timed_events_active_array;
        int *m_timed_events_free_array;
        int m_timed_events_active_count;
        int m_timed_events_free_count;
        int m_timed_events_capacity;

        // Untimed events are stored in a C array, which is traversed in order.
        untimed_event_t *m_untimed_events_array;
        int m_untimed_events_size;
        int m_untimed_events_cap;

        // ISR events
        isr_event_t *m_isr_events_array;
        int m_isr_events_count;
        int m_isr_events_cap;

        // Semaphores for accessing the above queues and lists
        SemaphoreHandle_t m_timed_mutex_;
        SemaphoreHandle_t m_untimed_mutex_;
        SemaphoreHandle_t m_isr_mutex_;

        uint64_t m_timed_counter = 0;
        uint64_t m_untimed_counter = 0;
        uint64_t m_tick_counter = 0;

        void tickTimed();
        void tickUntimed();
    };

} // namespace reactesp

#endif // CESP32_EVENT_LOOP_H_