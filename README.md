# Lámh

Lámh is the airbrake actuator node in [Ogma](https://sean-osullivan.com/projects/ogma/), the University of Limerick Aeronautics Society's modular avionics stack for high-powered rocketry.

## Role

- Drives four routed servo outputs through a PCA9685.
- Reads a physical arm input.
- Accepts time-limited actuator commands over the 500 kbit/s CAN bus.
- Enforces defined safe states when commands expire or communication is lost.

## Repository

- `hardware/` - KiCad design and manufacturing outputs.
- `firmware/` - embedded firmware, actuator probes, and tests.

## Status

Rev 1 firmware and hardware bring-up continue on [`ogma/flight-hardening`](https://github.com/ULAS-HiPR/lamh/tree/ogma/flight-hardening). Release-candidate firmware builds and software tests pass, but hardware-in-the-loop validation is still underway. Lámh is not yet flight-proven.

## Manufacturing support

Rev 1 PCB fabrication was sponsored through [EasyEDA Education](https://easyeda.com/education) and manufactured by [JLCPCB](https://jlcpcb.com/). The board was designed in KiCad and imported into EasyEDA Pro for the sponsorship and manufacturing workflow.

## More information

See the [Ogma project write-up](https://sean-osullivan.com/projects/ogma/) for the complete stack, bring-up work, and current status.
