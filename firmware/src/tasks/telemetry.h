#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>
#include <Radio/Radio.h>


namespace task{
class Telemetry {
    public:
        Telemetry(
        // Radio radio_in, 
            osMessageQueueId_t telem_queue) : 
            //radio_(radio_in),
             telem_queue_(telem_queue),taskHandle_(nullptr){};
        void run();

    private:
        void StartTelemetry();
        static void StartTelemetryEntry(void *argument);
        static task::State_Machine::State parse_message(char msg);

        //Radio& radio_;
        osMessageQueueId_t telem_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "Telemetry",
            0,
            nullptr,
            0,
            nullptr,
            512 * 4,        // 2 KB stack
            osPriorityNormal,
            0,
            0
        };
    };

}

