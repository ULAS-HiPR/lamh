# Lamh Firmware

Firmware for Lamh, the Ogma servo/canards board.

## Build Targets

```bash
pio run -e stm32f072c8t6
pio run -e stm32f072c8t6_input_probe
```

Target purpose:

- `stm32f072c8t6`: flight PCA9685 servo firmware.
- `stm32f072c8t6_input_probe`: input-pin bring-up/probe target.

## Runtime Behavior

The flight target:

- probes the hard-strapped PCA9685 address `0x40`,
- initializes the PCA9685 at 50 Hz,
- drives each routed output to its configured build-time safe angle,
- accepts `CAN_ID_ACTUATOR_COMMAND` only while Croi is alive,
- requires the active-low J7/PB2 arm input before applying CAN flight commands,
- returns to the safe angle after 5 s without Croi or a valid command.

## Host Interface

Ogma Console talks to Lamh over SWD. No UART is required.

Host-visible symbols:

- `ogma_board_identity`
- `servo_debug`
- `ogma_servo_command`

`servo_debug` reports boot, PCA9685, CAN, heartbeat, command, and failsafe state. `ogma_servo_command` accepts a channel and angle only with an active 60 s bench lease and no fresh Croi heartbeat. Ogma Console maps physical outputs 1-4 onto the routed PCA channels below.

## Hardware Notes

The current hardware README maps the four populated servo connectors:

- J4: PCA9685 channel 0.
- J6: PCA9685 channel 2.
- J2: PCA9685 channel 4.
- J3: PCA9685 channel 6.

The PCA9685 address is hard-strapped to `0x40`; flight firmware does not scan other I2C addresses.

## Safe-Angle Configuration

`include/lamh_safety_config.h` contains separate safe angles for PWM1-PWM4. Ogma Console changes this file only as part of an explicit build-and-flash action, then verifies the readback values from `servo_debug`. The values are compiled into the flight image and cannot be changed by CAN or the SWD mailbox while the vehicle operates.
