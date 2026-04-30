# Hardware Notes

## PCA9685 servo outputs

The current Lamh PCB routes the PCA9685 outputs to four servo connectors:

| Connector | PCA9685 channel | Connector pin 1 | Connector pin 2 | Connector pin 3 |
| --- | ---: | --- | --- | --- |
| J4 | 0 | Signal | GND | 5v_SERVO |
| J6 | 2 | Signal | GND | 5v_SERVO |
| J2 | 4 | Signal | GND | 5v_SERVO |
| J3 | 6 | Signal | GND | 5v_SERVO |

The firmware demo currently sweeps channel 0, so plug the demo servo into J4.

## PCA9685 address

A0-A3 have pulldowns and solder jumpers to 3.3 V. For the default `0x40` address, leave those jumpers open. The firmware scans the PCA9685 hardware-address range at boot, so changed address jumpers should still work.

A4 and A5 are marked unconnected in the current PCB files. If I2C probing is unstable on a built board, strap U2 A4 and A5 low for a deterministic `0x40` address in the next hardware revision.
