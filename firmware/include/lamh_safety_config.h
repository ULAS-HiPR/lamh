#ifndef LAMH_SAFETY_CONFIG_H
#define LAMH_SAFETY_CONFIG_H

/*
 * Ogma Console owns these build-time fail-safe positions. Values are degrees
 * and are compiled into the flight image; they are not writable in flight.
 */
#define LAMH_SAFETY_CONFIG_MAGIC 0x4C534346U
#define LAMH_SAFETY_CONFIG_VERSION 1U

#define LAMH_SAFE_ANGLE_PWM1_DEG 90
#define LAMH_SAFE_ANGLE_PWM2_DEG 90
#define LAMH_SAFE_ANGLE_PWM3_DEG 90
#define LAMH_SAFE_ANGLE_PWM4_DEG 90
#define LAMH_FLIGHT_MAX_ANGLE_DEG 90

#endif
