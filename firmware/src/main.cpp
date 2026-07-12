#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <I2C/I2C_STM.h>
#include <Servo/PCA9685.h>
#include <CAN/CAN_Frames.h>
#include <lamh_safety_config.h>
#include <servo_debug.h>
#include <stdint.h>

void SystemClock_Config(void);
void Error_Handler(void);

IWDG_HandleTypeDef hiwdg{};

extern "C" void vApplicationMallocFailedHook(void) {
    Error_Handler();
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *) {
    Error_Handler();
}

extern "C" void xPortSysTickHandler(void);

extern "C" void SysTick_Handler(void) {
    (void)SysTick->CTRL;
    HAL_IncTick();

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

extern "C" {

struct OgmaBoardIdentity {
    uint32_t magic;
    uint16_t schema_version;
    uint16_t struct_size;
    uint32_t board_id;
    uint32_t capabilities;
    uint32_t firmware_version;
    uint32_t firmware_build;
    uint32_t reserved0;
    uint32_t reserved1;
};

__attribute__((used)) volatile OgmaBoardIdentity ogma_board_identity{
    0x4F474944U,
    1U,
    sizeof(OgmaBoardIdentity),
    0x03U,
    0x05U,
    20260710U,
    0U,
    0U,
    0U,
};

struct OgmaServoCommand {
    uint32_t magic;
    uint32_t request_seq;
    uint32_t channel;
    uint32_t angle_deg;
    uint32_t applied_seq;
    uint32_t result;
    uint32_t last_channel;
    uint32_t last_angle_deg;
};

volatile OgmaServoCommand ogma_servo_command{};

struct OgmaDebugControl {
    uint32_t magic;
    uint32_t version;
    uint32_t request_seq;
    uint32_t unlock_key;
    uint32_t lease_ms;
    uint32_t unlocked_until_ms;
    uint32_t accepted_seq;
    uint32_t denied_count;
};

__attribute__((used)) volatile OgmaDebugControl ogma_debug_control{};

}

namespace {

constexpr uint32_t OGMA_SERVO_COMMAND_MAGIC = 0x4F475356U; // OGSV
constexpr uint32_t OGMA_SERVO_RESULT_OK = 0U;
constexpr uint32_t OGMA_SERVO_RESULT_BAD_CHANNEL = 1U;
constexpr uint32_t OGMA_SERVO_RESULT_WRITE_FAILED = 2U;
constexpr uint32_t OGMA_SERVO_RESULT_LOCKED = 3U;
constexpr uint32_t OGMA_SERVO_RESULT_CROI_ACTIVE = 4U;
constexpr uint32_t OGMA_SERVO_RESULT_ARMED = 5U;
constexpr uint8_t OGMA_SERVO_MAX_CHANNEL = 15U;
constexpr uint8_t LAMH_SERVO_CHANNELS[] = {0U, 2U, 4U, 6U};
constexpr int16_t LAMH_SAFE_ANGLES[] = {
    LAMH_SAFE_ANGLE_PWM1_DEG,
    LAMH_SAFE_ANGLE_PWM2_DEG,
    LAMH_SAFE_ANGLE_PWM3_DEG,
    LAMH_SAFE_ANGLE_PWM4_DEG,
};
static_assert(LAMH_SAFE_ANGLE_PWM1_DEG >= 0 && LAMH_SAFE_ANGLE_PWM1_DEG <= 90,
              "PWM1 safe angle must be within the servo range");
static_assert(LAMH_SAFE_ANGLE_PWM2_DEG >= 0 && LAMH_SAFE_ANGLE_PWM2_DEG <= 90,
              "PWM2 safe angle must be within the servo range");
static_assert(LAMH_SAFE_ANGLE_PWM3_DEG >= 0 && LAMH_SAFE_ANGLE_PWM3_DEG <= 90,
              "PWM3 safe angle must be within the servo range");
static_assert(LAMH_SAFE_ANGLE_PWM4_DEG >= 0 && LAMH_SAFE_ANGLE_PWM4_DEG <= 90,
              "PWM4 safe angle must be within the servo range");
static_assert(LAMH_FLIGHT_MAX_ANGLE_DEG >= 0 && LAMH_FLIGHT_MAX_ANGLE_DEG <= 90,
              "flight servo maximum must be within the mechanical range");
static_assert(sizeof(ServoDebugStatus) == 148U,
              "ServoDebugStatus layout changed; update Ogma Console parser");
constexpr uint32_t CAN_HEARTBEAT_PERIOD_MS = 1000U;
constexpr uint32_t CAN_CROI_TIMEOUT_MS = 5000U;
constexpr uint32_t CAN_BUS_RECOVERY_PERIOD_MS = 250U;
constexpr uint8_t CAN_TX_QUEUE_LEN = 8U;
constexpr uint8_t CAN_TX_DRAIN_BUDGET = 3U;
constexpr uint32_t OGMA_DEBUG_CONTROL_MAGIC = 0x4F474442U; // OGDB
constexpr uint32_t OGMA_DEBUG_CONTROL_VERSION = 1U;
constexpr uint32_t OGMA_DEBUG_UNLOCK_KEY = 0x0BEE11E5U;
constexpr uint32_t OGMA_DEBUG_MIN_LEASE_MS = 1000U;
constexpr uint32_t OGMA_DEBUG_MAX_LEASE_MS = 60000U;
constexpr uint32_t LAMH_ARM_DEBOUNCE_MS = 50U;

struct CANFrame {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

bool can_ready = false;
uint32_t can_tx_count = 0U;
uint32_t can_rx_count = 0U;
uint32_t can_tx_drops = 0U;
uint32_t heartbeat_tx_count = 0U;
uint32_t command_rx_count = 0U;
uint32_t croi_last_seen_ms = 0U;
uint32_t last_command_ms = 0U;
uint32_t last_command_timeout_ms = ACTUATOR_COMMAND_MIN_LEASE_MS;
uint32_t last_heartbeat_ms = 0U;
uint32_t last_bus_recovery_ms = 0U;
uint32_t failsafe_count = 0U;
bool failsafe_active = false;
bool arm_input_active = false;
bool arm_input_raw_active = false;
uint32_t arm_input_last_change_ms = 0U;
uint16_t last_actuator_sequence = 0U;
bool actuator_sequence_seen = false;
bool watchdog_refresh_allowed = true;
uint8_t active_output_index = 0xFFU;
CANFrame tx_queue[CAN_TX_QUEUE_LEN]{};
uint8_t tx_head = 0U;
uint8_t tx_tail = 0U;
uint8_t tx_count = 0U;

bool init_watchdog() {
#if defined(__HAL_DBGMCU_FREEZE_IWDG)
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 2499U;
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    return HAL_IWDG_Init(&hiwdg) == HAL_OK;
}

int16_t clamp_servo_angle(uint32_t angle) {
    if (angle > static_cast<uint32_t>(LAMH_FLIGHT_MAX_ANGLE_DEG)) {
        return LAMH_FLIGHT_MAX_ANGLE_DEG;
    }

    return static_cast<int16_t>(angle);
}

bool is_flight_servo_channel(uint32_t channel) {
    for (uint8_t flight_channel : LAMH_SERVO_CHANNELS) {
        if (channel == flight_channel) {
            return true;
        }
    }
    return false;
}

bool tick_after(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) > 0;
}

bool debug_lease_active(uint32_t now_ms) {
    if (ogma_debug_control.magic == OGMA_DEBUG_CONTROL_MAGIC &&
        ogma_debug_control.version == OGMA_DEBUG_CONTROL_VERSION &&
        ogma_debug_control.request_seq != 0U &&
        ogma_debug_control.request_seq != ogma_debug_control.accepted_seq &&
        ogma_debug_control.unlock_key == OGMA_DEBUG_UNLOCK_KEY) {
        uint32_t lease_ms = ogma_debug_control.lease_ms;
        if (lease_ms < OGMA_DEBUG_MIN_LEASE_MS) {
            lease_ms = OGMA_DEBUG_MIN_LEASE_MS;
        }
        if (lease_ms > OGMA_DEBUG_MAX_LEASE_MS) {
            lease_ms = OGMA_DEBUG_MAX_LEASE_MS;
        }
        ogma_debug_control.unlocked_until_ms = now_ms + lease_ms;
        ogma_debug_control.accepted_seq = ogma_debug_control.request_seq;
        ogma_debug_control.unlock_key = 0U;
    }

    return tick_after(ogma_debug_control.unlocked_until_ms, now_ms);
}

void deny_debug_command() {
    ogma_debug_control.denied_count++;
}

bool probe_pca9685(I2C_HandleTypeDef* hi2c, uint8_t address) {
    servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_START;
    servo_debug.pca9685_found = 0;
    servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_PROBE;
    servo_debug.scan_attempts++;
    servo_debug.scan_last_address = address;
    servo_debug.i2c_last_op = SERVO_DEBUG_I2C_OP_READY;
    servo_debug.i2c_last_address = address;

    const HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(
        hi2c, static_cast<uint16_t>(address << 1), 2, 10);
    servo_debug.scan_last_status = static_cast<uint32_t>(status);
    servo_debug.scan_last_error = HAL_I2C_GetError(hi2c);
    servo_debug.i2c_last_status = servo_debug.scan_last_status;
    servo_debug.i2c_last_error = servo_debug.scan_last_error;
    if (status != HAL_OK) {
        servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_NOT_FOUND;
        servo_debug.pca9685_address = address;
        return false;
    }

    servo_debug.stage = SERVO_DEBUG_STAGE_SCAN_FOUND;
    servo_debug.pca9685_found = 1;
    servo_debug.pca9685_address = address;
    return true;
}

template<typename Payload>
CANFrame make_can_frame(uint32_t id, const Payload& payload)
{
    static_assert(sizeof(Payload) <= 8U, "CAN payload too large");
    CANFrame frame{};
    frame.id = id;
    frame.dlc = 8U;
    const auto* bytes = reinterpret_cast<const uint8_t*>(&payload);
    for (uint8_t index = 0U; index < sizeof(Payload); ++index) {
        frame.data[index] = bytes[index];
    }
    return frame;
}

bool can_bus_off()
{
    return hcan.Instance != nullptr && (hcan.Instance->ESR & CAN_ESR_BOFF) != 0U;
}

uint32_t can_error()
{
    return hcan.Instance == nullptr ? HAL_CAN_ERROR_PARAM : HAL_CAN_GetError(&hcan);
}

bool can_send_now(CANFrame& frame)
{
    if (!can_ready || can_bus_off() || frame.id > 0x7FFU || frame.dlc > 8U) {
        return false;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
        return false;
    }

    CAN_TxHeaderTypeDef header{};
    header.StdId = frame.id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame.dlc;
    header.TransmitGlobalTime = DISABLE;

    uint32_t mailbox = 0U;
    return HAL_CAN_AddTxMessage(&hcan, &header, frame.data, &mailbox) == HAL_OK;
}

bool queue_can_frame(const CANFrame& frame)
{
    if (tx_count >= CAN_TX_QUEUE_LEN) {
        tx_head = static_cast<uint8_t>((tx_head + 1U) % CAN_TX_QUEUE_LEN);
        --tx_count;
        ++can_tx_drops;
    }
    tx_queue[tx_tail] = frame;
    tx_tail = static_cast<uint8_t>((tx_tail + 1U) % CAN_TX_QUEUE_LEN);
    ++tx_count;
    return true;
}

bool send_can_frame(CANFrame& frame)
{
    if (!can_ready) {
        return false;
    }
    if (tx_count != 0U) {
        return queue_can_frame(frame);
    }
    if (can_send_now(frame)) {
        ++can_tx_count;
        return true;
    }
    return queue_can_frame(frame);
}

void flush_can_queue()
{
    if (!can_ready) {
        return;
    }
    for (uint8_t sent = 0U; sent < CAN_TX_DRAIN_BUDGET && tx_count > 0U; ++sent) {
        CANFrame& frame = tx_queue[tx_head];
        if (!can_send_now(frame)) {
            return;
        }
        tx_head = static_cast<uint8_t>((tx_head + 1U) % CAN_TX_QUEUE_LEN);
        --tx_count;
        ++can_tx_count;
    }
}

void service_can_bus(uint32_t now_ms)
{
    if (!can_ready || !can_bus_off()) {
        return;
    }
    if ((now_ms - last_bus_recovery_ms) < CAN_BUS_RECOVERY_PERIOD_MS) {
        return;
    }
    (void)HAL_CAN_Stop(&hcan);
    HAL_CAN_ResetError(&hcan);
    (void)HAL_CAN_Start(&hcan);
    last_bus_recovery_ms = now_ms;
}

bool croi_alive(uint32_t now_ms)
{
    return croi_last_seen_ms != 0U && (now_ms - croi_last_seen_ms) < CAN_CROI_TIMEOUT_MS;
}

void update_can_debug()
{
    servo_debug.can_init_ok = can_ready ? 1U : 0U;
    servo_debug.can_bus_off = can_ready && can_bus_off() ? 1U : 0U;
    servo_debug.can_error = can_ready ? can_error() : 0U;
    servo_debug.can_tx_count = can_tx_count;
    servo_debug.can_rx_count = can_rx_count;
    servo_debug.can_tx_drops = can_tx_drops;
    servo_debug.can_tx_queue_depth = tx_count;
    servo_debug.heartbeat_tx_count = heartbeat_tx_count;
    servo_debug.command_rx_count = command_rx_count;
    servo_debug.croi_last_seen_ms = croi_last_seen_ms;
    servo_debug.failsafe_count = failsafe_count;
    servo_debug.can_esr = hcan.Instance != nullptr ? hcan.Instance->ESR : 0U;
    servo_debug.arm_input_active = arm_input_active ? 1U : 0U;
    servo_debug.arm_input_raw_active = arm_input_raw_active ? 1U : 0U;
}

void init_arm_input()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void update_arm_input(uint32_t now_ms)
{
    const bool sample_active = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET;
    if (sample_active != arm_input_raw_active) {
        arm_input_raw_active = sample_active;
        arm_input_last_change_ms = now_ms;
    }
    if ((now_ms - arm_input_last_change_ms) >= LAMH_ARM_DEBOUNCE_MS) {
        arm_input_active = arm_input_raw_active;
    }
}

void update_safety_config_debug()
{
    servo_debug.safety_config_magic = LAMH_SAFETY_CONFIG_MAGIC;
    servo_debug.safety_config_version = LAMH_SAFETY_CONFIG_VERSION;
    for (uint8_t index = 0U; index < 4U; ++index) {
        servo_debug.safe_angles_deg[index] = LAMH_SAFE_ANGLES[index];
    }
}

void send_heartbeat(uint32_t now_ms)
{
    uint8_t err = 0U;
    if (can_bus_off()) {
        err |= CAN_HEARTBEAT_ERR_BUS_OFF;
    }
    if (can_error() != 0U) {
        err |= CAN_HEARTBEAT_ERR_CAN_ERROR;
    }
    if (can_tx_drops != 0U) {
        err |= CAN_HEARTBEAT_ERR_TX_DROP;
    }
    if (!croi_alive(now_ms)) {
        err |= CAN_HEARTBEAT_ERR_NODE_TIMEOUT;
    }

    HEARTBEAT_Payload payload{
        static_cast<uint8_t>(NODE_LAMH),
        static_cast<uint8_t>(failsafe_active ? 0U : 1U),
        err,
        static_cast<uint8_t>((now_ms / 1000U) & 0xFFU),
    };
    CANFrame frame = make_can_frame(CAN_ID_HEARTBEAT_NODE(NODE_LAMH), payload);
    if (send_can_frame(frame)) {
        ++heartbeat_tx_count;
    }
}

} // namespace

namespace task {

class ServoFlight {
    public:
        ServoFlight(I2C_Handler* i2c, uint8_t pca9685_address)
            : _i2c(i2c), _pca9685_address(pca9685_address) {}

        bool run() {
            _taskHandle = osThreadNew(
                &ServoFlight::entry,
                this,
                &_attributes
            );
            return _taskHandle != nullptr;
        }

    private:
        static void entry(void* argument) {
            auto* self = static_cast<ServoFlight*>(argument);
            if (self) self->loop();
        }

        void loop() {
            servo_debug.stage = SERVO_DEBUG_STAGE_TASK_START;

            if (_i2c == nullptr) {
                for (;;) {
                    osDelay(1000);
                }
            }

            PCA9685Servo init_servo(*_i2c, LAMH_SERVO_CHANNELS[0], _pca9685_address);
            if (!init_servo.init()) {
                servo_debug.stage = SERVO_DEBUG_STAGE_PCA_INIT_FAILED;
                Error_Handler();
            }
            apply_safe_state();
            if (!failsafe_active) {
                Error_Handler();
            }
            last_heartbeat_ms = HAL_GetTick();
            last_bus_recovery_ms = last_heartbeat_ms;

            for (;;) {
                const uint32_t now_ms = HAL_GetTick();
                update_arm_input(now_ms);
                service_can_bus(now_ms);
                flush_can_queue();
                service_can_rx(now_ms);
                (void)apply_pending_debug_command();
                if (croi_alive(now_ms) && !arm_input_active && !failsafe_active) {
                    apply_safe_state();
                }
                if (should_failsafe(now_ms) && !failsafe_active) {
                    apply_safe_state();
                }
                if ((now_ms - last_heartbeat_ms) >= CAN_HEARTBEAT_PERIOD_MS) {
                    send_heartbeat(now_ms);
                    last_heartbeat_ms = now_ms;
                }
                update_can_debug();
                servo_debug.ticks = now_ms;
                if (watchdog_refresh_allowed) {
                    (void)HAL_IWDG_Refresh(&hiwdg);
                }
                osDelay(20);
            }
        }

        void set_all_outputs(int16_t angle) {
            for (uint8_t channel : LAMH_SERVO_CHANNELS) {
                PCA9685Servo servo(*_i2c, channel, _pca9685_address);
                (void)servo.set_position(angle);
                servo_debug.servo_channel = channel;
                servo_debug.servo_angle = angle;
            }
            servo_debug.servo_set_count++;
        }

        bool apply_safe_state() {
            bool all_writes_ok = true;
            for (uint8_t index = 0U; index < 4U; ++index) {
                PCA9685Servo servo(*_i2c, LAMH_SERVO_CHANNELS[index], _pca9685_address);
                all_writes_ok = servo.set_position(LAMH_SAFE_ANGLES[index]) && all_writes_ok;
                servo_debug.servo_channel = LAMH_SERVO_CHANNELS[index];
                servo_debug.servo_angle = LAMH_SAFE_ANGLES[index];
            }
            servo_debug.servo_set_count++;
            if (all_writes_ok && !failsafe_active) {
                ++failsafe_count;
            }
            failsafe_active = all_writes_ok;
            watchdog_refresh_allowed = all_writes_ok;
            if (all_writes_ok) {
                active_output_index = 0xFFU;
            }
            return all_writes_ok;
        }

        bool should_failsafe(uint32_t now_ms) const {
            if (!croi_alive(now_ms)) {
                return arm_input_active || !debug_lease_active(now_ms);
            }
            if (last_command_ms == 0U) {
                return true;
            }
            return (now_ms - last_command_ms) >= last_command_timeout_ms;
        }

        void apply_actuator_command(const CANFrame& frame, uint32_t now_ms) {
            if (frame.dlc < sizeof(ActuatorCommandPayload) || !croi_alive(now_ms)) {
                return;
            }

            ActuatorCommandPayload payload{};
            auto* bytes = reinterpret_cast<uint8_t*>(&payload);
            for (uint8_t index = 0U; index < sizeof(ActuatorCommandPayload); ++index) {
                bytes[index] = frame.data[index];
            }

            ++command_rx_count;
            if ((payload.flags & ACTUATOR_COMMAND_FLAG_ACTIVE) == 0U) {
                actuator_sequence_seen = true;
                last_actuator_sequence = payload.sequence;
                if (!failsafe_active) {
                    apply_safe_state();
                }
                return;
            }
            const bool sequence_fresh = !actuator_sequence_seen ||
                (now_ms - last_command_ms) >= last_command_timeout_ms ||
                static_cast<int16_t>(payload.sequence - last_actuator_sequence) > 0;
            if (!sequence_fresh) {
                return;
            }
            actuator_sequence_seen = true;
            last_actuator_sequence = payload.sequence;
            if (!arm_input_active || payload.output_index >= 4U ||
                payload.lease_ms < ACTUATOR_COMMAND_MIN_LEASE_MS ||
                payload.lease_ms > ACTUATOR_COMMAND_MAX_LEASE_MS) {
                apply_safe_state();
                return;
            }

            int32_t angle = payload.angle_cdeg / 100;
            if (angle < 0) {
                angle = 0;
            }
            if (angle > LAMH_FLIGHT_MAX_ANGLE_DEG) {
                angle = LAMH_FLIGHT_MAX_ANGLE_DEG;
            }
            if (active_output_index < 4U && active_output_index != payload.output_index) {
                PCA9685Servo previous(
                    *_i2c, LAMH_SERVO_CHANNELS[active_output_index], _pca9685_address);
                if (!previous.set_position(LAMH_SAFE_ANGLES[active_output_index])) {
                    apply_safe_state();
                    return;
                }
            }
            PCA9685Servo servo(*_i2c, LAMH_SERVO_CHANNELS[payload.output_index], _pca9685_address);
            if (!servo.set_position(static_cast<int16_t>(angle))) {
                apply_safe_state();
                return;
            }
            servo_debug.servo_channel = LAMH_SERVO_CHANNELS[payload.output_index];
            servo_debug.servo_angle = static_cast<int16_t>(angle);
            active_output_index = payload.output_index;
            failsafe_active = false;
            watchdog_refresh_allowed = true;
            last_command_ms = now_ms;
            last_command_timeout_ms = payload.lease_ms;
        }

        void process_rx_frame(const CANFrame& frame, uint32_t now_ms) {
            if ((frame.id & 0x7F0U) == CAN_ID_HEARTBEAT_BASE && frame.dlc >= 4U) {
                if (frame.data[0] == NODE_CROI) {
                    croi_last_seen_ms = now_ms;
                }
                return;
            }
            if (frame.id == CAN_ID_ACTUATOR_COMMAND) {
                apply_actuator_command(frame, now_ms);
            }
        }

        void service_can_rx(uint32_t now_ms) {
            if (!can_ready) {
                return;
            }
            if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0) != RESET) {
                __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FOV0);
            }
            while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0U) {
                CAN_RxHeaderTypeDef header{};
                CANFrame frame{};
                uint8_t data[8]{};
                if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
                    return;
                }
                ++can_rx_count;
                if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8U) {
                    continue;
                }
                frame.id = header.StdId;
                frame.dlc = header.DLC;
                for (uint8_t index = 0U; index < header.DLC; ++index) {
                    frame.data[index] = data[index];
                }
                process_rx_frame(frame, now_ms);
            }
        }

        bool apply_pending_debug_command() {
            const uint32_t request_seq = ogma_servo_command.request_seq;

            if (ogma_servo_command.magic != OGMA_SERVO_COMMAND_MAGIC ||
                request_seq == 0U ||
                request_seq == ogma_servo_command.applied_seq) {
                return false;
            }

            const uint32_t channel = ogma_servo_command.channel;
            const int16_t angle = clamp_servo_angle(ogma_servo_command.angle_deg);

            ogma_servo_command.last_channel = channel;
            ogma_servo_command.last_angle_deg = static_cast<uint32_t>(angle);

            const uint32_t now_ms = HAL_GetTick();
            if (!debug_lease_active(now_ms)) {
                deny_debug_command();
                ogma_servo_command.result = OGMA_SERVO_RESULT_LOCKED;
                ogma_servo_command.applied_seq = request_seq;
                return true;
            }

            if (croi_alive(now_ms)) {
                deny_debug_command();
                ogma_servo_command.result = OGMA_SERVO_RESULT_CROI_ACTIVE;
                ogma_servo_command.applied_seq = request_seq;
                return true;
            }

            if (arm_input_active) {
                deny_debug_command();
                ogma_servo_command.result = OGMA_SERVO_RESULT_ARMED;
                ogma_servo_command.applied_seq = request_seq;
                return true;
            }

            if (channel > OGMA_SERVO_MAX_CHANNEL || !is_flight_servo_channel(channel)) {
                ogma_servo_command.result = OGMA_SERVO_RESULT_BAD_CHANNEL;
                ogma_servo_command.applied_seq = request_seq;
                return true;
            }

            PCA9685Servo servo(*_i2c, static_cast<uint8_t>(channel), _pca9685_address);
            if (!servo.set_position(angle)) {
                ogma_servo_command.result = OGMA_SERVO_RESULT_WRITE_FAILED;
                ogma_servo_command.applied_seq = request_seq;
                return true;
            }

            ogma_servo_command.result = OGMA_SERVO_RESULT_OK;
            ogma_servo_command.applied_seq = request_seq;
            failsafe_active = false;
            watchdog_refresh_allowed = true;
            active_output_index = static_cast<uint8_t>(
                (channel == LAMH_SERVO_CHANNELS[0]) ? 0U :
                (channel == LAMH_SERVO_CHANNELS[1]) ? 1U :
                (channel == LAMH_SERVO_CHANNELS[2]) ? 2U : 3U);
            last_command_ms = now_ms;
            return true;
        }

        I2C_Handler* _i2c;
        uint8_t _pca9685_address;
        osThreadId_t    _taskHandle;
        StaticTask_t _task_control_block{};
        StackType_t _task_stack[256]{};

        const osThreadAttr_t _attributes = {
            "servoFlight",
            0,
            &_task_control_block,
            sizeof(_task_control_block),
            _task_stack,
            sizeof(_task_stack),
            osPriorityNormal,   // priority
            0,
            0
        };
};

} // namespace task

// ============================================================
// Main
// ============================================================
int main(void) {
    (void)ogma_board_identity.magic;
    servo_debug.magic = SERVO_DEBUG_MAGIC;
    update_safety_config_debug();
    servo_debug.stage = SERVO_DEBUG_STAGE_BOOT;
    HAL_Init();
    SystemClock_Config();
    servo_debug.stage = SERVO_DEBUG_STAGE_CLOCK_READY;
    if (!init_watchdog()) {
        Error_Handler();
    }
    init_arm_input();
    MX_I2C1_Init();
    servo_debug.stage = SERVO_DEBUG_STAGE_I2C_READY;
    can_ready = MX_CAN_Init() && HAL_CAN_Start(&hcan) == HAL_OK;
    osKernelInitialize();

    // create I2C handler for the active STM platform
    constexpr uint8_t pca9685_address = PCA9685Servo::DEFAULT_ADDRESS;
    if (!probe_pca9685(&hi2c1, pca9685_address)) {
        Error_Handler();
    }
    static I2C_STM i2c(&hi2c1, pca9685_address);

    static task::ServoFlight servo_task(&i2c, pca9685_address);
    if (!servo_task.run()) {
        servo_debug.stage = SERVO_DEBUG_STAGE_TASK_CREATE_FAILED;
        Error_Handler();
    }

    servo_debug.stage = SERVO_DEBUG_STAGE_RTOS_START;
    osKernelStart();

    // never get here
    while (1) {
        HAL_Delay(1000);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc{};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PREDIV = RCC_PREDIV_DIV1;
    osc.PLL.PLLMUL = RCC_PLL_MUL6;

    bool clock_ok = HAL_RCC_OscConfig(&osc) == HAL_OK;
    if (clock_ok) {
        RCC_ClkInitTypeDef clk{};
        clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
        clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
        clk.APB1CLKDivider = RCC_HCLK_DIV1;
        clock_ok = HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) == HAL_OK;
    }

    if (!clock_ok) {
#if defined(RCC_OSCILLATORTYPE_HSI48)
        osc = {};
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
        osc.HSI48State = RCC_HSI48_ON;
        osc.PLL.PLLState = RCC_PLL_NONE;
        clock_ok = HAL_RCC_OscConfig(&osc) == HAL_OK;
        if (clock_ok) {
            RCC_ClkInitTypeDef clk{};
            clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
            clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
            clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
            clk.APB1CLKDivider = RCC_HCLK_DIV1;
            clock_ok = HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) == HAL_OK;
        }
#endif
    }

    if (!clock_ok) {
        Error_Handler();
    }

    SystemCoreClockUpdate();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
#if defined(TIM6)
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
#else
    (void)htim;
#endif
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}
