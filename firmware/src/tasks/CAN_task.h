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
//#include <CAN/CanBus.h>


namespace task{
class CAN_task {
    public:
        CAN_task(osMessageQueueId_t can_queue_) : can_queue_(can_queue_), taskHandle_(nullptr) {};
        void run();

    private:
        void StartCAN();
        static void StartCANEntry(void *argument);
        char parse_message(char msg);

        //CAN& can_bus_;
        osMessageQueueId_t can_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "CAN",
            0,
            nullptr,
            0,
            nullptr,
            256,        // 2 KB stack
            osPriorityNormal,
            0,
            0
        };
    };

}

