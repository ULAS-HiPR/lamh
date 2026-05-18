#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"
#include <cstdio>
#include <CAN/CAN_Handler.h>
#include <CAN/CAN_Frames.h>


namespace task{
class CAN_task {
    public:
        CAN_task(osMessageQueueId_t can_queue, CAN_Handler* can = nullptr)
            : can_queue_(can_queue), can_(can), taskHandle_(nullptr) {}
        void run();

    private:
        void StartCAN();
        static void StartCANEntry(void *argument);

        osMessageQueueId_t can_queue_;
        CAN_Handler*       can_;
        osThreadId_t       taskHandle_;

        const osThreadAttr_t task_attributes {
            "CAN",
            0,
            nullptr,
            0,
            nullptr,
            512 * 4,
            osPriorityNormal,
            0,
            0
        };
    };

}

