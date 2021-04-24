/**
 * @file       ArdStepper.h
 * @author     Kyle Escholm
 * @brief      Stepper motor control
 *
 * @details
 *
 * See group @ref ArdStepper
 *
 */

#ifndef ARD_STEPPER_H
#define ARD_STEPPER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ArdEncoder.h"

#ifdef __cplusplus

#ifndef restrict
#define restrict __restrict
#endif

extern "C" {
#endif

/**
 * @defgroup   ArdStepper Stepper Motor Control
 * @brief      Stepper motor control
 *
 * @{
 *
 */

#ifndef ARD_STEPPER_PULSE_WIDTH_MICROSECONDS
/**
 * @brief Stepper pulse width in microseconds
 *
 */
#define ARD_STEPPER_PULSE_WIDTH_MICROSECONDS 20
#endif

#ifndef ARD_STEPPER_ENABLE_POLARITY
/**
 * @brief Polarity of enable pin, set to 0 if enable LOW is off
 *
 */
#define ARD_STEPPER_ENABLE_POLARITY 1
#endif

typedef struct ArdStepperPins {
    /**
     * @brief Pulse output pin
     *
     */
    uint8_t pulse;

    /**
     * @brief pulse direction pin
     *
     */
    uint8_t direction;

    /**
     * @brief enabled pin
     *
     */
    uint8_t enable;

    /**
     * @brief direction polarity
     *
     */
    uint8_t direction_polarity;

} ArdStepperPins;

/**
 * @brief Stepper pulse output state
 *
 */
typedef struct ArdStepperState {
    /**
     * @brief Period between pulses in microseconds
     *
     */
    uint32_t pulse_step_size;
    /**
     * @brief Remaining steps in command output
     *
     */
    uint32_t steps_remaining;
    /**
     * @brief Direction of pulse steps
     *
     */
    uint8_t direction;
    /**
     * @brief Enabled stepper
     *
     */
    bool enabled;
} ArdStepperState;

/**
 * @brief Stepper motor controller
 *
 */
typedef struct ArdStepper {
    /**
     * @brief Circular buffer of commands for stepper
     *
     */
    int32_t *buffer;
    /**
     * @brief Set to 1 if buffer is full
     *
     */
    size_t buffer_tail;
    /**
     * @brief Current index of command arrays
     *
     */
    size_t buffer_index;
    /**
     * @brief Number of commands in step_increment_commands buffer
     *
     */
    size_t buffer_size;
    /**
     * @brief Command step size in microseconds
     *
     */
    uint32_t command_step_size;
    /**
     * @brief Delay between producing and consuming commands
     *
     */
    size_t command_delay_steps;
    /**
     * @brief @c `true` if drive is enabled
     *
     */
    bool enabled;
    /**
     * @brief Stepper motor pulses
     *
     */
    ArdEncoder encoder;

} ArdStepper;

/**
 * @brief Stepper motor controller parameters
 *
 */
typedef struct ArdStepperParameters {
    /**
     * @brief Number of commands in step_increment_commands buffer
     *
     */
    size_t buffer_size;

    /**
     * @brief Stepper motor pulses per revolution
     *
     */
    uint32_t pulses_per_rev;

    /**
     * @brief Motor physical units per revolution (eg. degrees, mm, etc.)
     *
     */
    double units_per_rev;

    /**
     * @brief Maximum command velocity in physical units
     *
     */
    double max_unit_velocity;

    /**
     * @brief Minimum step size in microsepconds. High frequency sampling
     *
     */
    uint32_t min_pulse_period;

    /**
     * @brief Command loop step size
     *
     */
    uint32_t command_step_size;

    /**
     * @brief Delay between producing and consuming commands
     *
     */
    size_t command_delay_steps;
    /**
     * @brief Stepper pin assignments
     *
     */
    ArdStepperPins pins;

} ArdStepperParameters;

/**
 * @brief Allocate memory and initialize stepper controller
 *
 * @param controller Stepper controller
 * @param max_command_size maximum command array size
 * @param buffer_num_commands Number of command arrays in circular buffer
 * @param command_step_size Step size of controller
 * @return int
 */

/**
 * @brief Allocate memory and initialize stepper controller
 *
 * @param stepper Stepper object
 * @param parameters Stepper parameters
 * @return int Returns 0 on success error otherwise.
 */
int ard_stepper_alloc(ArdStepper *stepper, const ArdStepperParameters *parameters);

// /**
//  * @brief Controller delay
//  *
//  * @return uint32_t microseconds of controller delay
//  */
// uint32_t ard_stepper_get_delay_microseconds(ArdStepper *stepper);

/**
 * @brief Free memory allocated when no longer using stepper
 *
 * @param stepper
 */
void ard_stepper_free(ArdStepper *stepper);

/**
 * @brief Enable stepper
 *
 * @param stepper
 */
void ard_stepper_enable(ArdStepper *stepper);

/**
 * @brief Disable stepper
 *
 * @param stepper
 */
void ard_stepper_disable(ArdStepper *stepper);

/**
 * @brief
 *
 * @param controller
 * @return int
 */
void ard_stepper_reset(ArdStepper *stepper, const double position, const bool enable);

/**
 * @brief run stepper
 *
 * @param controller Stepper controller
 */
void ard_stepper_consume_run(ArdStepper *stepper);

/**
 * @brief Append desired positions to command buffer
 *
 * @param stepper Stepper object
 * @param position desired position array
 * @param size Size of position array
 * @return int Returns non-zero if error occurred
 */
int ard_stepper_produce_command(ArdStepper *stepper, const double *restrict position,
                                const size_t size);

/**
 * @brief Append desired change in pulse counts
 *
 * @param stepper Stepper object
 * @param count_deltas desired change in counts (pulses) array
 * @param size Size of count array
 * @return int Returns non-zero if error occurred
 */
int ard_stepper_produce_count_deltas(ArdStepper *stepper, const int32_t *restrict count_deltas,
                                    const size_t size);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
