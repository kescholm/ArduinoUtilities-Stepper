#include <Arduino.h>

#include "ArdStepper.h"

#ifdef __AVR__
#include <TimerOne.h>
#elif defined(ESP8266)

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#include <ESP8266TimerInterrupt.h>
#endif

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// pulse train

volatile ArdStepperState s_pulse_state;
volatile ArdStepperPins s_pins;

static void IRAM_ATTR ard_stepper_isr_pulse_train();

static void ard_stepper_pulse_timer_setup(const uint32_t step_size_us);
static void ard_stepper_pulse_timer_update(const uint32_t step_size_us);

static void ard_stepper_pulse_timer_disable();
static void ard_stepper_pulse_timer_enable();

static void ard_stepper_guard_on();
static void ard_stepper_guard_off();

static void ard_stepper_pulse_train_reset(const bool enable);
static void ard_stepper_pulse_train_setup(const ArdStepperPins *pins,
                                   const uint32_t pulse_timer_step_size_us);

#ifdef __AVR__

// avr implementation
// ------------------

static void ard_stepper_isr_pulse_train() {
    // apply pulse train
    if (s_pulse_state.steps_remaining > 0) {
        // send pulse
        digitalWrite(s_pins.pulse, HIGH);
        // reset pulse
        delayMicroseconds(ARD_STEPPER_PULSE_WIDTH_MICROSECONDS);
        digitalWrite(s_pins.pulse, LOW);
        // decrement remaining steps and advance timing
        s_pulse_state.steps_remaining -= 1;
    }
}

static void ard_stepper_pulse_timer_setup(const uint32_t step_size_us) {
    // use TimeOne library for AVR

    Timer1.initialize(step_size_us);
    Timer1.attachInterrupt(ard_stepper_isr_pulse_train);
    Timer1.stop();
}

static void ard_stepper_pulse_timer_update(const uint32_t step_size_us) {
    Timer1.setPeriod(step_size_us);
}

static void ard_stepper_pulse_timer_disable() {
    s_pulse_state.enabled = false;
    digitalWrite(s_pins.enable, (ARD_STEPPER_ENABLE_POLARITY ? HIGH : LOW));
    Timer1.stop();
}

static void ard_stepper_pulse_timer_enable() {
    Timer1.start();
    digitalWrite(s_pins.enable, (ARD_STEPPER_ENABLE_POLARITY ? LOW : HIGH));
    s_pulse_state.enabled = true;
}

static void ard_stepper_guard_on() { noInterrupts(); }

static void ard_stepper_guard_off() { interrupts(); }

#elif defined(ESP_PLATFORM)

// ESP32 implementation
// --------------------

hw_timer_t *s_timer = NULL;
portMUX_TYPE s_timer_mutex = portMUX_INITIALIZER_UNLOCKED;

static void ard_stepper_guard_on() { portENTER_CRITICAL(&s_timer_mutex); }

static void ard_stepper_guard_off() { portEXIT_CRITICAL(&s_timer_mutex); }

static void IRAM_ATTR ard_stepper_isr_pulse_train() {
    portENTER_CRITICAL_ISR(&s_timer_mutex);
    // apply pulse train
    if (s_pulse_state.steps_remaining > 0) {
        // send pulse
        digitalWrite(s_pins.pulse, HIGH);
        // reset pulse
        delayMicroseconds(ARD_STEPPER_PULSE_WIDTH_MICROSECONDS);
        digitalWrite(s_pins.pulse, LOW);
        // decrement remaining steps and advance timing
        s_pulse_state.steps_remaining -= 1;
    }
    portEXIT_CRITICAL_ISR(&s_timer_mutex);
}

static void ard_stepper_pulse_timer_setup(const uint32_t step_size_us) {
    // use timerBegin with timeout for ESP32
    s_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(s_timer, &ard_stepper_isr_pulse_train, true);
    timerAlarmWrite(s_timer, step_size_us, true);
    timerAlarmEnable(s_timer);
}

static void ard_stepper_pulse_timer_update(const uint32_t step_size_us) {
    timerAlarmWrite(s_timer, step_size_us, true);
}

static void ard_stepper_pulse_timer_enable() {
    timerAlarmEnable(s_timer);
    digitalWrite(s_pins.enable, (ARD_STEPPER_ENABLE_POLARITY ? LOW : HIGH));
    s_pulse_state.enabled = true;
}

static void ard_stepper_pulse_timer_disable() {
    // use timerBegin with timeout for ESP32
    s_pulse_state.enabled = false;
    digitalWrite(s_pins.enable, (ARD_STEPPER_ENABLE_POLARITY ? HIGH : LOW));
    timerAlarmDisable(s_timer);
}

#elif defined(ESP8266)

// ESP6266 implementation
// ----------------------

// Init ESP8266Timer
ESP8266Timer s_timer;

static void ard_stepper_isr_pulse_train() {
    // apply pulse train
    if (s_pulse_state.steps_remaining > 0) {
        // send pulse
        digitalWrite(s_pins.pulse, HIGH);
        // reset pulse
        delayMicroseconds(ARD_STEPPER_PULSE_WIDTH_MICROSECONDS);
        digitalWrite(s_pins.pulse, LOW);
        // decrement remaining steps and advance timing
        s_pulse_state.steps_remaining -= 1;
    }
}

static void ard_stepper_pulse_timer_setup(const uint32_t step_size_us) {
    // use Timer library
    s_timer.attachInterruptInterval(step_size_us, ard_stepper_isr_pulse_train);
    s_timer.disableTimer();
}

static void ard_stepper_pulse_timer_update(const uint32_t step_size_us) {
    s_timer.setInterval(step_size_us, ard_stepper_isr_pulse_train);
}

static void ard_stepper_pulse_timer_disable() {
    s_pulse_state.enabled = false;
    digitalWrite(s_pins.enable, (ARD_STEPPER_ENABLE_POLARITY ? HIGH : LOW));
    s_timer.disableTimer();
}

static void ard_stepper_pulse_timer_enable() {
    s_timer.enableTimer();
    digitalWrite(s_pins.enable, (ARD_STEPPER_ENABLE_POLARITY ? LOW : HIGH));
    s_pulse_state.enabled = true;
}

static void ard_stepper_guard_on() { noInterrupts(); }

static void ard_stepper_guard_off() { interrupts(); }

#else

  #error ArdStepper is only compatible with AVR, ESP32, or ESP8266 platforms

#endif

// Common Implementation
// ---------------------

static void ard_stepper_pulse_train_reset(const bool enable) {
    ard_stepper_guard_on();

    // pulse state
    s_pulse_state.pulse_step_size = 0;
    s_pulse_state.steps_remaining = 0;
    s_pulse_state.direction = 0;

    digitalWrite(s_pins.pulse, LOW);
    if (enable) {
        // turn on: ready for pulse train
        ard_stepper_pulse_timer_enable();
    } else {
        // turn off pulse train
        ard_stepper_pulse_timer_disable();
    }

    ard_stepper_guard_off();
}

static void ard_stepper_pulse_train_setup(const ArdStepperPins *pins,
                                   const uint32_t pulse_timer_step_size_us) {
    ard_stepper_guard_on();

    // pulse state
    s_pulse_state.pulse_step_size = 0;
    s_pulse_state.steps_remaining = 0;
    s_pulse_state.direction = 0;

    // pins
    s_pins.enable = pins->enable;
    s_pins.direction = pins->direction;
    s_pins.direction_polarity = pins->direction_polarity;
    s_pins.pulse = pins->pulse;

    pinMode(s_pins.pulse, OUTPUT);
    digitalWrite(s_pins.pulse, LOW);

    pinMode(s_pins.enable, OUTPUT);
    digitalWrite(s_pins.enable, (ARD_STEPPER_ENABLE_POLARITY ? HIGH : LOW));

    pinMode(s_pins.pulse, OUTPUT);
    digitalWrite(s_pins.pulse, LOW);

    pinMode(s_pins.direction, OUTPUT);
    digitalWrite(s_pins.direction, (s_pins.direction_polarity ? HIGH : LOW));

    // pulse timer
    ard_stepper_pulse_timer_setup(pulse_timer_step_size_us);

    ard_stepper_guard_off();
}

// User-facing functions

void IRAM_ATTR ard_stepper_consume_run(ArdStepper *stepper) {
    if (stepper->enabled) {
        // get next command number of pulses

        // space available in buffer
        size_t queued_positions =
            (stepper->buffer_tail > stepper->buffer_index
                 ? (stepper->buffer_size - stepper->buffer_tail) + stepper->buffer_index
                 : stepper->buffer_index - stepper->buffer_tail);
        // consume next available command
        if (queued_positions > 0) {
            // consume at tail of queue
            const int32_t delta = stepper->buffer[stepper->buffer_tail];
            // advance
            stepper->buffer_tail = (stepper->buffer_tail + 1) % stepper->buffer_size;

            // convert command to pulse train
            if (delta != 0) {
                // check steps remaining
                ard_stepper_guard_on();
                uint32_t steps_remaining = s_pulse_state.steps_remaining;
                const uint32_t pulse_step_size = s_pulse_state.pulse_step_size;
                ard_stepper_guard_off();

                // wait for steps remaining
                // this should not happen often, and wait should be short if timing is respected.
                while (steps_remaining > 0) {
                    // wait to finish
                    delayMicroseconds(pulse_step_size * steps_remaining);
                    // check steps remaining
                    ard_stepper_guard_on();
                    steps_remaining = s_pulse_state.steps_remaining;
                    ard_stepper_guard_off();
                }

                // modify pulse state
                ard_stepper_guard_on();

                // direction
                if (delta < 0) {
                    s_pulse_state.steps_remaining = -delta;
                    s_pulse_state.direction = s_pins.direction_polarity ? LOW : HIGH;
                } else {
                    s_pulse_state.steps_remaining = delta;
                    s_pulse_state.direction = s_pins.direction_polarity ? HIGH : LOW;
                }
                // set pulse steps
                s_pulse_state.pulse_step_size =
                    stepper->command_step_size / s_pulse_state.steps_remaining;
                // set direction pin
                digitalWrite(s_pins.direction, s_pulse_state.direction);
                // ensure pulse timer is enabled
                if (!s_pulse_state.enabled) {
                    ard_stepper_pulse_timer_enable();
                }
                // update pulse timer step size
                ard_stepper_pulse_timer_update(s_pulse_state.pulse_step_size);

                // resume pulse train
                ard_stepper_guard_off();
            }
        } else {
            ard_stepper_guard_on();
            if (s_pulse_state.steps_remaining == 0) {
                // no more commands waiting, no steps: turn off pulse train
                ard_stepper_pulse_timer_disable();
            }
            ard_stepper_guard_off();
        }
    }
}

int ard_stepper_alloc(ArdStepper *stepper, const ArdStepperParameters *parameters) {
    if (parameters->buffer_size < 2) return -1;
    if (parameters->command_delay_steps >= parameters->buffer_size) return -2;
    if (parameters->min_pulse_period < (2 * ARD_STEPPER_PULSE_WIDTH_MICROSECONDS)) return -3;

    // buffer sizes
    stepper->buffer_tail = 0;
    stepper->buffer_index = 0;
    stepper->buffer_size = parameters->buffer_size;

    // initalize buffer as NULL
    stepper->buffer = NULL;

    // allocate buffer
    stepper->buffer = (int32_t *)calloc(parameters->buffer_size, sizeof(int32_t));
    if (stepper->buffer == NULL) {
        return -4;
    }

    // command step size and delay
    stepper->command_step_size = parameters->command_step_size;
    stepper->command_delay_steps = parameters->command_delay_steps;

    // initialize in disabled state
    stepper->enabled = false;

    // command encoder
    ard_encoder_set(&stepper->encoder, 32U, parameters->pulses_per_rev, parameters->units_per_rev);

    // check min period
    const uint32_t min_pulse_period =
        (uint32_t)(1.0e6 * parameters->units_per_rev /
                   (parameters->max_unit_velocity * ((double)parameters->pulses_per_rev)));
    if (parameters->min_pulse_period * 2 > min_pulse_period) {
        // min_pulse_period, max velocity, or pulses_per_rev too high
        return -5;
    }

    // check maximum pulses per command
    const uint64_t max_pulses_per_command =
        (uint64_t)(1.0e6 * ((double)stepper->command_step_size) / min_pulse_period);
    if (max_pulses_per_command > INT32_MAX) {
        // too many pulses per command for 32 bit signed integers
        return -6;
    }

    // pulse state
    const uint32_t pulse_step_size = min_pulse_period / 2;
    ard_stepper_pulse_train_setup(&parameters->pins, pulse_step_size);

    return 0;
}

void ard_stepper_free(ArdStepper *stepper) {
    ard_stepper_disable(stepper);

    free(stepper->buffer);
    stepper->buffer = NULL;

    stepper->buffer_tail = 0;
    stepper->buffer_index = 0;
    stepper->buffer_size = 0;
}

void ard_stepper_enable(ArdStepper *stepper) {
    ard_stepper_reset(stepper, stepper->encoder.value, true);
    stepper->enabled = true;
}

void ard_stepper_disable(ArdStepper *stepper) {
    stepper->enabled = false;
    ard_stepper_reset(stepper, stepper->encoder.value, false);
}

void ard_stepper_reset(ArdStepper *stepper, const double position, const bool enable) {
    // reset pulse state
    ard_stepper_pulse_train_reset(enable);

    stepper->buffer_tail = 0;
    stepper->buffer_index = 0;
    stepper->enabled = enable;

    // reset encoder
    ard_encoder_reset_value(&stepper->encoder, position);
}

int ard_stepper_produce_command(ArdStepper *stepper, const double *restrict position,
                                const size_t size) {
    // space available in buffer
    size_t available_in_queue =
        (stepper->buffer_tail > stepper->buffer_index
             ? stepper->buffer_tail - stepper->buffer_index
             : stepper->buffer_size - (stepper->buffer_index - stepper->buffer_tail));
    // add positions array to buffer
    size_t k = 0;
    while (k < size && k < available_in_queue) {
        // encode: set delta steps to buffer
        stepper->buffer[stepper->buffer_index] = ard_encoder_encode(&stepper->encoder, position[k]);
        // advance
        stepper->buffer_index = (stepper->buffer_index + 1) % stepper->buffer_size;
        k += 1;
    }
    return k;
}

int ard_stepper_produce_count_deltas(ArdStepper *stepper, const int32_t *restrict count_deltas,
                                    const size_t size) {
    // space available in buffer
    size_t available_in_queue =
        (stepper->buffer_tail > stepper->buffer_index
             ? stepper->buffer_tail - stepper->buffer_index
             : stepper->buffer_size - (stepper->buffer_index - stepper->buffer_tail));
    // add positions array to buffer
    size_t k = 0;
    while (k < size && k < available_in_queue) {
        // Set delta steps to buffer
        stepper->buffer[stepper->buffer_index] = count_deltas[k];
        // advance
        stepper->buffer_index = (stepper->buffer_index + 1) % stepper->buffer_size;
        k += 1;
    }
    return k;
}
