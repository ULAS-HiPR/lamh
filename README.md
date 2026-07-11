# Lamh

Lamh is the servo/canards board in the Ogma stack. Its job is to drive servo outputs from the stack command path and provide a bench-controllable actuator target during bring-up.

## Role In Ogma

- Drives a PCA9685 servo controller over I2C.
- Provides actuator/canards control hardware.
- Shares the STM32F072 + CAN architecture used across the Ogma STM boards.
- Uses the common Ogma local power architecture.

## Firmware

Firmware lives in `firmware/`.

Useful targets:

```bash
cd firmware
pio run -e stm32f072c8t6
pio run -e stm32f072c8t6_input_probe
```

`stm32f072c8t6` is the flight image. It probes only the PCA9685 address hard-strapped by the schematic (`0x40`), drives all routed outputs to the configured safe angle at boot, then accepts Croi CAN commands.

## Ogma Console Support

Ogma Console can:

- identify Lamh over SWD using `ogma_board_identity`,
- flash the flight firmware,
- read `servo_debug`,
- command physical servo outputs 1-4 through `ogma_servo_command` during a temporary bench lease.

Hardware mapping from the schematic:

- PWM1 -> PCA9685 channel 0.
- PWM2 -> PCA9685 channel 2.
- PWM3 -> PCA9685 channel 4.
- PWM4 -> PCA9685 channel 6.

## Host-Visible Symbols

- `ogma_board_identity`
- `servo_debug`
- `ogma_servo_command`

## Notes

- SWD servo control is denied while a fresh Croi heartbeat is present.
- J7 pin 1 is STM32 PB2 and J7 pin 2 is GND. CAN flight commands require a debounced low PB2 arm input; SWD bench commands remain lease-gated.
- A bench command remains active only for its 60 s lease; expiry returns all outputs to the safe angle.
- Loss of Croi heartbeat or CAN command for 5 s returns all outputs to the safe angle.
- Each output has a separate build-time safe angle. Ogma Console sets the four values, builds, flashes, and verifies them over SWD. Defaults are 90 degrees and need mechanical sign-off before flight.
- Flight CAN commands use the unified leased actuator payload from `comheadan`.
- The hardware watchdog starts only after PCA9685 init and the initial safe-angle writes succeed.
- Rev1 hardware ties PCA9685 OE low, so PWM can remain driven during MCU reset until firmware rewrites outputs. Rev2 should route OE to the MCU or a hardware safe-disable path.

## Dependency Lock

Use the exact shared-library pins in `../dependencies.lock.json`:

- `braiteoiri`: `ogma/flight-hardening`
- `comheadan`: `ogma/flight-hardening`

Ogma Console doctor fails a board when these submodule SHAs do not match the lock file.
