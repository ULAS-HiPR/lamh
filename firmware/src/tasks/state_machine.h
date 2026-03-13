#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>
#include <IMU>
#include <Servo>

namespace task{
class State_Machine {
    public:
        State_Machine(IMU& imu, Servo& servo,
                      osMessageQueueId_t telem_queue,
                      osMessageQueueId_t logger_queue)
            : imu_(imu),
              servo_(servo),
              telem_queue_(telem_queue),
              logger_queue_(logger_queue),
              taskHandle_(nullptr)
        {};

        void run();

        enum State {
            STOP,
            ROLL,
            UNROLL
        }

    private:
        void StartStateMachine();
        static void StartStateMachineEntry(void *argument);
        void do_roll_action();
        void do_unroll_action();
        void stop_action();

        IMU& imu_;
        Servo& servo_;
        osMessageQueueId_t telem_queue_;
        osMessageQueueId_t logger_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "StateMachine",
            0,
            nullptr,
            0,
            nullptr,
            512 * 4,        // 2 KB stack
            osPriorityHigh,
            0,
            0
        };
    };

}

