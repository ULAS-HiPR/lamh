#include "CAN_task.h"

namespace task {

void CAN_task::run() {
    taskHandle_ = osThreadNew(&CAN_task::StartCANEntry, this, &task_attributes);
}

void CAN_task::StartCANEntry(void *argument) {
    auto *self = static_cast<CAN_task*>(argument);
    if (self) {
        self->StartCAN();
    }
}

void CAN_task::StartCAN() {
    printf("[lamh] CAN task started\n");

    uint8_t  uptime_s = 0;
    uint32_t last_1hz = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();

        // 1 Hz: HEARTBEAT TX
        if (can_ && now - last_1hz >= 1000) {
            last_1hz = now;
            uptime_s++;
            HEARTBEAT_Payload hb{};
            hb.node_id  = NODE_LAMH;
            hb.state    = 0;
            hb.err      = 0;
            hb.uptime_s = uptime_s;
            CAN_Frame hb_frame = pack_frame(CAN_ID_HEARTBEAT, hb);
            can_->send(&hb_frame);
            printf("[lamh] HB uptime=%u\n", uptime_s);
        }

        // RX: drain incoming CAN frames
        if (can_) {
            CAN_Frame rx{};
            while (can_->receive(&rx)) {
                if (rx.id == CAN_ID_FLIGHT_STATE) {
                    FLIGHT_STATE_Payload fs{};
                    unpack_frame(rx, fs);
                    printf("[lamh] FLIGHT_STATE state=%u ts=%u\n", fs.state, fs.timestamp_ms);
                    // Forward state to canards controller queue
                    char state_byte = static_cast<char>(fs.state);
                    osMessageQueuePut(can_queue_, &state_byte, 0, 0);
                } else if (rx.id == CAN_ID_IMU_ACCEL) {
                    IMU_ACCEL_Payload acc{};
                    unpack_frame(rx, acc);
                    // Available for canards PID — log at low priority
                } else if (rx.id == CAN_ID_IMU_GYRO) {
                    IMU_GYRO_Payload gyro{};
                    unpack_frame(rx, gyro);
                } else if (rx.id == CAN_ID_SYNC) {
                    SYNC_Payload sync{};
                    unpack_frame(rx, sync);
                    printf("[lamh] SYNC ts=%lu\n", (unsigned long)sync.timestamp_ms);
                }
            }
        }

        osDelay(5);
    }
}

}