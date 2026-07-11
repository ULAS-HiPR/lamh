#ifndef SERVO_DEBUG_H
#define SERVO_DEBUG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    SERVO_DEBUG_MAGIC = 0x53455256,
    SERVO_DEBUG_STAGE_BOOT = 1,
    SERVO_DEBUG_STAGE_CLOCK_READY = 2,
    SERVO_DEBUG_STAGE_I2C_READY = 3,
    SERVO_DEBUG_STAGE_SCAN_START = 4,
    SERVO_DEBUG_STAGE_SCAN_PROBE = 5,
    SERVO_DEBUG_STAGE_SCAN_FOUND = 6,
    SERVO_DEBUG_STAGE_SCAN_NOT_FOUND = 7,
    SERVO_DEBUG_STAGE_RTOS_START = 8,
    SERVO_DEBUG_STAGE_TASK_CREATE_FAILED = 9,
    SERVO_DEBUG_STAGE_TASK_START = 20,
    SERVO_DEBUG_STAGE_PCA_INIT_START = 21,
    SERVO_DEBUG_STAGE_PCA_RESET = 22,
    SERVO_DEBUG_STAGE_PCA_MODE2 = 23,
    SERVO_DEBUG_STAGE_PCA_FREQ_START = 24,
    SERVO_DEBUG_STAGE_PCA_FREQ_DONE = 25,
    SERVO_DEBUG_STAGE_PCA_INIT_DONE = 26,
    SERVO_DEBUG_STAGE_PCA_INIT_FAILED = 27,
    SERVO_DEBUG_STAGE_SERVO_SET = 30,
    SERVO_DEBUG_STAGE_PWM_WRITE = 31
};

enum {
    SERVO_DEBUG_I2C_OP_NONE = 0,
    SERVO_DEBUG_I2C_OP_WRITE = 1,
    SERVO_DEBUG_I2C_OP_READ_REG = 2,
    SERVO_DEBUG_I2C_OP_READ_DATA = 3,
    SERVO_DEBUG_I2C_OP_READY = 4
};

typedef struct {
    uint32_t magic;
    uint32_t stage;
    uint32_t ticks;

    uint8_t pca9685_found;
    uint8_t pca9685_address;
    uint8_t servo_channel;
    uint8_t reserved0;

    uint32_t scan_attempts;
    uint32_t scan_last_status;
    uint32_t scan_last_error;
    uint8_t scan_last_address;
    uint8_t reserved1[3];

    uint32_t i2c_last_op;
    uint32_t i2c_last_status;
    uint32_t i2c_last_error;
    uint8_t i2c_last_address;
    uint8_t i2c_last_register;
    uint16_t i2c_last_length;
    uint32_t i2c_write_count;
    uint32_t i2c_read_count;

    uint32_t pca9685_init_count;
    uint8_t pca9685_mode1_before_prescale;
    uint8_t pca9685_prescale;
    uint8_t reserved2[2];

    uint32_t servo_set_count;
    int16_t servo_angle;
    uint16_t servo_pwm;
    uint16_t pwm_on;
    uint16_t pwm_off;
    uint32_t can_init_ok;
    uint32_t can_bus_off;
    uint32_t can_error;
    uint32_t can_tx_count;
    uint32_t can_rx_count;
    uint32_t can_tx_drops;
    uint32_t can_tx_queue_depth;
    uint32_t heartbeat_tx_count;
    uint32_t command_rx_count;
    uint32_t croi_last_seen_ms;
    uint32_t failsafe_count;
    uint32_t can_esr;
    uint32_t safety_config_magic;
    uint16_t safety_config_version;
    uint16_t reserved3;
    int16_t safe_angles_deg[4];
    uint32_t arm_input_active;
    uint32_t arm_input_raw_active;
} ServoDebugStatus;

extern volatile ServoDebugStatus servo_debug;

#ifdef __cplusplus
}
#endif

#endif
