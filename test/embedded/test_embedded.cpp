#include <Arduino.h>
#include <unity.h>

#include "ArdStepper.h"

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

// Pins
// ----

#ifndef STEPPER_PIN_PULSE
#define STEPPER_PIN_PULSE 4
#endif

#ifndef STEPPER_PIN_DIRECTION
#define STEPPER_PIN_DIRECTION 5
#endif

#ifndef STEPPER_PIN_ENABLE
#define STEPPER_PIN_ENABLE 6
#endif

// test pin connected to stepper output
#ifndef STEPPER_TEST_PIN_PULSE
#define STEPPER_TEST_PIN_PULSE 2
#endif

// Test Pulse Counting
// -------------------

// Use IRAM_ATTR for ISR on esp32
#ifdef __AVR__
#define IRAM_ATTR
#endif

// stepper test pulse counting
volatile uint32_t s_pulse_count = 0;

static void IRAM_ATTR test_ard_isr_pulse_count() { s_pulse_count += 1; }

static uint32_t test_ard_get_pulses_counted()
{
    noInterrupts();
    uint32_t ret = s_pulse_count;
    interrupts();
    return ret;
}

static void test_ard_pulse_count_setup()
{
    noInterrupts();
    // on rising edge, set pin back to LOW after pulse width delay
    pinMode(STEPPER_TEST_PIN_PULSE, INPUT_PULLUP);
#ifdef __AVR__
    attachInterrupt(digitalPinToInterrupt(STEPPER_TEST_PIN_PULSE), test_ard_isr_pulse_count,
                    RISING);
#else
    attachInterrupt(STEPPER_TEST_PIN_PULSE, test_ard_isr_pulse_count, RISING);
#endif
    interrupts();
}

// Stepper
// -------

// global stepper object
ArdStepper g_stepper;

// get parameters
ArdStepperParameters get_stepper_parameters(void) {

    ArdStepperParameters parameters;
    parameters.buffer_size = 16;
    parameters.pulses_per_rev = 6400;
    parameters.units_per_rev = 360.0;
    parameters.max_unit_velocity = 180.0;
    parameters.min_pulse_period = 100U;
    parameters.command_step_size = 10000U;
    parameters.command_delay_steps = 2;
    parameters.pins.pulse = STEPPER_PIN_PULSE;
    parameters.pins.direction = STEPPER_PIN_DIRECTION;
    parameters.pins.enable = STEPPER_PIN_ENABLE;
    parameters.pins.direction_polarity = 1;

    return parameters;
}

void test_stepper_loop(void) {

}

// Timing
// ------

uint32_t g_timing_now = 0;
uint32_t g_timing_prev = 0;
uint32_t g_timing_step_size = 10000U;

// Spline trajectory
// -----------------

// #define TRAJ_SPLINE_START 0
// #define TRAJ_SPLINE_END 180
// #define TRAJ_SPLINE_DURATION_SECONDS 1.0

// bool g_traj_forward;
// ArdSplineTrajectory g_traj;
// ArdSplinePva g_traj_start;
// ArdSplinePva g_traj_end;

// static void generate_test_traj()
// {
//     g_traj_start.position = TRAJ_SPLINE_START;
//     g_traj_start.velocity = 0;
//     g_traj_start.acceleration = 0;

//     g_traj_end.position = TRAJ_SPLINE_END;
//     g_traj_end.velocity = 0;
//     g_traj_end.acceleration = 0;

//     int ret =
//         ard_spline_traj_generate(&g_traj, &g_traj_start, &g_traj_end, TRAJ_SPLINE_DURATION_SECONDS,
//                                  ARD_TIMING_100HZ_STEP_SIZE, EARD_SPLINE_QUINTIC);
//     TEST_ASSERT_EQUAL(0, ret);
//     g_traj_forward = true;
// }

// Tests
// -----

static void test_ard_stepper_alloc(void)
{
    // stepper test parameters
    ArdStepperParameters stepper_parameters = get_stepper_parameters();

    // too few commands in buffer
    stepper_parameters.buffer_size = 1;
    int result = ard_stepper_alloc(&g_stepper, &stepper_parameters);
    TEST_ASSERT_EQUAL(-1, result);
    ard_stepper_free(&g_stepper);

    // max velocity too high
    stepper_parameters = get_stepper_parameters();
    stepper_parameters.max_unit_velocity = 360.0;
    result = ard_stepper_alloc(&g_stepper, &stepper_parameters);
    TEST_ASSERT_EQUAL(-5, result);
    ard_stepper_free(&g_stepper);

    // good configuration
    stepper_parameters = get_stepper_parameters();
    result = ard_stepper_alloc(&g_stepper, &stepper_parameters);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_NOT_EQUAL(NULL, g_stepper.buffer);
}

static void test_ard_stepper_end_test(void)
{
    const uint32_t pulses = test_ard_get_pulses_counted();
    TEST_ASSERT_EQUAL(pulses, g_stepper.encoder.count);
    TEST_ASSERT(0 < g_stepper.encoder.count);
    ard_stepper_reset(&g_stepper, 0.0, false);
    TEST_ASSERT_EQUAL(0, g_stepper.encoder.count);
}


void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();  // IMPORTANT LINE!

    /*
        Start communications
        --------------------
    */

    // begin serial
    // ard_serial_begin(ARD_SERIAL_BAUD_RATE);
    // delay(100);

    // join i2c bus
    // ard_i2c_master_begin();
    // delay(100);

    // ArdStepper
    // ------

    RUN_TEST(test_ard_stepper_alloc);

    // Pulse counting
    // --------------
    test_ard_pulse_count_setup();

    // flush serial
    // ard_serial_write_flush();
    // ard_serial_read_flush();

    // Timing
    // ------

    g_timing_now = micros();
    g_timing_prev = g_timing_now;

    // Finish Setup
    // ------------

    // Reset stepper
    ard_stepper_reset(&g_stepper, 0.0, true);
}

// Command vector
// --------------

const size_t g_command_size = 200;
const double g_command[200] = {};
size_t g_command_count = 0;

void loop()
{
    g_timing_now = micros();

    // 100 HZ
    // -----
    if ((g_timing_now - g_timing_prev) >= g_timing_step_size)
    {
        // elapsed steps
        uint32_t elapsed_steps = (g_timing_now - g_timing_prev) / g_timing_step_size;
        // advance step time
        g_timing_prev += elapsed_steps * g_timing_step_size;
        // should not miss a step
        TEST_ASSERT_EQUAL(1, elapsed_steps);

        if (g_command_count < g_command_size)
        {
            // command from array
            const double cmd = g_command[g_command_count];
            // produce test commands
            ard_stepper_produce_command(&g_stepper, &cmd, 1);
        }
        // update command count
        g_command_count++;

        // run stepper
        ard_stepper_consume_run(&g_stepper);
    }

    if (g_command_count > (1 + g_command_size + g_stepper.command_delay_steps))
    {
        // done pulsing
        RUN_TEST(test_ard_stepper_end_test);
        UNITY_END();  // stop unit testing
    }
}

#ifdef __AVR__

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#endif
