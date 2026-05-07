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
#include <cstdio>
#include <data.h>
#include <cstdint>
#include <cmath>

//#include <IMU/IMU.h>
#include <Servo/servo.h>

namespace task{
class Canards_Controller {
    public:
        Canards_Controller(//Servo& servo, 
            osMessageQueueId_t can_queue)
            : 
            //servo_(servo),
              can_queue_(can_queue),
              taskHandle_(nullptr){};
              
        void run();

    private:
        void StartCanardsController();
        static void StartCanardsControllerEntry(void *argument);
        float run_canards_controller(imu_data imu, baro_data baro, prediction_data prediction);
        float get_rocket_angle(imu_data imu);
        void stop_action();
        bool safety_check(int state, imu_data imu);
        void get_up_direction();

        uint32_t last_time_ms{0};
        float rocket_angle{0.0f};
        float prev_roll_rate{0.0f};
        float output{0.0f};
        float output_degrees{0.0f};
        float servo_angle{0.0f};

        // Target dynamics
        const float omega_n       = 6.0f; // keep float!!!
        const float damp_ratio    = 1.0f;

        // Rocket properties
        const float I_xx          = 0.00164f;
        const float   N_canards     = 2.0f;
        const float C_L_alpha_can = 2.86f;

        // Canard CP offset
        const float S_can = 0.0024f;
        const float gamma_can = 0.6f;
        const float y_mac_can = 0.0275f;
        const float Y_cp_can = 0.067f; 

        //Servo& servo_;
        osMessageQueueId_t can_queue_;

        osThreadId_t taskHandle_;

        const osThreadAttr_t task_attributes {
            "CanardsController",
            0,
            nullptr,
            0,
            nullptr,
            256 ,       // 256 
            osPriorityHigh,
            0,
            0
        };
    };

}

