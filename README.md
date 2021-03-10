# ArduinoUtilities-Stepper

An Arduino library to generate and send pulses to a stepper motor. Interrupts and timers are implemented to send out the pulses and ensure non-blocking operation. Step commands can be sent directly or a floating point value can be added with automatic conversion to pulses.

Only one stepper motor is supported. Support for multiple stepper motors would be  amazing, so if you want to tackle implementing this feature, please get in touch and get involved!

## Prerequisites

Install [Platform.IO](https://docs.platformio.org/en/latest/) for your IDE. If you want to use the command line, append `"$HOME/.platformio/penv/bin"` to your `PATH` environment variable as instructed by [platformio documentation](https://docs.platformio.org/en/latest/core/installation.html#install-shell-commands).

## Use the Library with Platform.IO or Arduino

In your project, simply add this library in your platform.ini file:

```ini
; Library dependencies
lib_deps =
    ArduinoUtilities-Stepper
```

For more information about libraries for Platform.IO, read up on the [Library Dependency Finder](https://docs.platformio.org/en/latest/librarymanager/ldf.html#ldf)

### Example Usage

It is recommended (but optional) to use preprocessor definitions with defaults to set pin assignments. This makes it easier to configure and build for different hardware.

```cpp
#ifndef STEPPER_PIN_PULSE
#define STEPPER_PIN_PULSE 1
#endif
#ifndef STEPPER_PIN_DIRECTION
#define STEPPER_PIN_DIRECTION 2
#endif
#ifndef STEPPER_PIN_ENABLE
#define STEPPER_PIN_ENABLE 3
#endif
```

In your setup, set all parameters to control the stepper motor.

```cpp
#include "ArdStepper.h"

// stepper parameters
// ------------------
ArdStepperParameters parameters;
// buffer to queue up pulses (minimum 2)
parameters.buffer_size = 3;
// stepper driver pulses per revolution and scaled unit conversion
parameters.pulses_per_rev = 2000;
parameters.units_per_rev = 360.0;
// maximum velocity in scaled units
parameters.max_unit_velocity = 50.0;
// minimum pulse period in microseconds for high speed pulse generation
// this must be greater than (2 * ARD_STEPPER_PULSE_WIDTH_MICROSECONDS)
parameters.min_pulse_step_size = 50;
// Command step size
// This is the interval at which counts are consumed from the buffer and pulse frequency is changed. You must provide your own timing facility to run the stepper at this rate.
// example: 200 Hz in microseconds is 5000
parameters.command_step_size = 5000U;
// delay in number of steps to ensure no missed steps.
// this must be less than buffer size
parameters.command_delay_steps = 2;
// pin definitions
parameters.pins.pulse = STEPPER_PIN_PULSE;
parameters.pins.direction = STEPPER_PIN_DIRECTION;
parameters.pins.enable = STEPPER_PIN_ENABLE;
// direction polarity can be set to non-zero to change positive direction
parameters.pins.direction_polarity = 0;

// create stepper object and initialize with parameters
// ----------------------------------------------------
ArdStepper stepper;
int result = ard_stepper_alloc(&stepper, &parameters, 1);
if (result != 0) {
    // Error occurred
}

// reset position and enable
// -------------------------
double start_position = 0.0;
ard_stepper_reset(&stepper, start_position, true);
```

The stepper must be run at regular intervals at the same rate as `parameters.command_step_size`. The following code snippet uses a busy-loop for simplicity but it's recommended to implement an interrupt timer for your main control loop.

```cpp
// initialize microsecond timer (can be done in setup())
uint32_t timing_now = micros();
uint32_t timing_prev = timing_now;
uint32_t timing_step_size = parameters.command_step_size;

// loop forever
void loop(void) {
    // get current time
    uint32_t timing_now = micros();

    // 'get_stepper_commands' is not implemented anywhere, just an example interface to get commands
    // and send to stepper with 'ard_stepper_produce_command'
    double command_buffer[MAX_COMMAND_VALUES];
    uint8_t buffer_size = 0;
    get_stepper_commands(MAX_COMMAND_VALUES, command_buffer, &buffer_size);
    if (buffer_size > 0) {
        // Send commands to stepper
        int result = ard_stepper_produce_command(&stepper, command_buffer, buffer_size);
        if (result != 0) {
            // Error
        }
    }

    // check if enough time elapsed (step size) to run controller
    if ((timing_now - timing_prev) >= timing_step_size)
    {
        // elapsed steps
        uint32_t elapsed_steps = (timing_now - timing_prev) / timing_step_size;
        // advance step time
        timing_prev += elapsed_steps * timing_step_size;
        if (elapsed_steps > 1) {
            // Error: We missed a step
        }
        // run stepper
        ard_stepper_consume_run(&stepper);
    }

}
```

## Setup for Development

If using VS Code, you can easily [install recommended extensions](https://code.visualstudio.com/docs/editor/extension-gallery#_recommended-extensions) for this project.

### Run Examples

Examples are useful when developing the library code. See example code in [examples](./examples/README) folder. Copy and paste source files from example directory to `src` folder then build using Platform.IO. For example:

```sh
cp -r examples/examples/ArdStepper-CountToValue/* src/
pio run -e atmega328
```

Do not commit any example files in the `src` directory. There must not be any `main` function in the source files for an Arduino library.

### Format Source Files

Source formatting uses LLVM's [Clang Format](https://clang.llvm.org/docs/ClangFormat.html) and the configuration is provided in [`.clang-format`](.clang-format). The configuration is based on default Google style (generated by clang-format version 10.0.0) with `IndentWidth` set to 4 and `ColumnLimit` set to 100.

### Unit Testing

Please read up on Platform.IO unit testing here: [https://docs.platformio.org/en/latest/plus/unit-testing.html](https://docs.platformio.org/en/latest/plus/unit-testing.html). Unit tests are in the `test` folder.

For embedded testing with hardware connected by serial ports, set proper upload ports and optionally modify platform.ini with different boards. For example:

```sh
pio test -e esp32 --upload-port /dev/ttyUSB0
pio test -e atmega328 --upload-port /dev/ttyUSB1
```
