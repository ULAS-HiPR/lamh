#pragma once
#include <cstdint>
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "cmsis_os.h"
#include <math.h>
#include <cstdio>
#include <data.h>
#include <Servo/servo.h>

#define CANARDS_DELAY_MS 1000

namespace task{
class Canards_Controller {
    public:
        Canards_Controller(
                      Servo& servo,
                      osMessageQueueId_t can_queue,
                      osMessageQueueId_t logger_queue)
            : 
              servo_(servo),
              can_queue_(can_queue),
              logger_queue_(logger_queue),
              taskHandle_(nullptr){};
              
        void run();

    private:
        void StartCanardsController();
        static void StartCanardsControllerEntry(void *argument);
        canards_raw run_canards_controller(const imu_data& imu, const baro_data& baro, const prediction_data& pred);
        float get_rocket_angle(const imu_data& imu);
        void stop_action();
        bool safety_check(int state, const imu_data& imu);
        void get_up_direction();

        uint32_t last_time_ms{0};
        float rocket_angle{0.0f};
        float prev_roll_rate{0.0f};
        float output{0.0f};
        float output_degrees{0.0f};
        float servo_angle{0.0f};
        float rho{0.0f};
        float q{0.0f};

        const float omega_n = 6.0f;
        const float damp_ratio = 1.0f;
        const float I_xx = 0.00164f;
        const float N_canards = 2.0f;
        const float C_L_alpha_can = 2.86f;
        const float S_can = 0.0024f;
        const float Y_cp_can = 0.067f;

        Servo& servo_;
        osMessageQueueId_t can_queue_;
        osMessageQueueId_t logger_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "CanardsController",
            0,
            nullptr,
            0,
            nullptr,
            512,        // 512 byte stack
            osPriorityHigh,
            0,
            0
        };
    };

}

