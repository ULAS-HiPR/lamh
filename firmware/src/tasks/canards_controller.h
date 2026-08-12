#pragma once
#include <atomic>
#include <cstdint>
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#if F0
#include "stm32f0xx_hal.h"
#include "platform/stm_f0.h"
#endif
#include "platform/hal_time.h"


#include "cmsis_os.h"
#include <math.h>
#include <cstdio>
#include <cstdint>
#include <data.h>
#include <Servo/servo.h>

#define CANARDS_DELAY_MS 50
// this is actully for airbrakes
//180 = in
//0 = out

namespace task{
class Canards_Controller {
    public:
        Canards_Controller(
                      Servo& servo,
                      int8_t active_pin,  // should make a pin wrapper but whatever
                      GPIO_TypeDef* active_port,
                      float apogeeDesired,
                      osMessageQueueId_t can_queue,
                      osMessageQueueId_t logger_queue)
            : 
              servo_(servo),
              active_pin_(active_pin),
              active_port_(active_port),
              apogeeDesired(apogeeDesired),
              can_queue_(can_queue),
              logger_queue_(logger_queue),
              taskHandle_(nullptr){};
              
        void run();

    private:
        Servo& servo_;
        int8_t active_pin_;
        GPIO_TypeDef* active_port_;
        float apogeeDesired{2000.0f}; // desired apogee in meters
        osMessageQueueId_t can_queue_;
        osMessageQueueId_t logger_queue_;

        void StartCanardsController();
        static void StartCanardsControllerEntry(void *argument);
        canards_raw run_canards_controller(const baro_data& baro, const prediction_data& pred);
        float get_rocket_angle(const imu_data& imu);
        void stop_action();
        bool safety_check(bool active, const imu_data& imu);
        void get_up_direction();

        // controller state
        float last_deployment_ = 0.0f;   // previous step's clamped output (d in Artem's formula)
        float previous_error_ = 0.0f;    // for ErrorRate
        uint32_t last_airbrake_time_ms_ = 0;
        float servo_angle = 180.0f;

        static constexpr float mass = 16.8f; //kg
        static constexpr float g = 9.80665f;   // m/s^2
        static constexpr float speedOfSound = 343.0f;  // m/s
        static constexpr float areaMax = 0.00388f;    // m^2 (maximum deployed area)
        static constexpr float zeta = 1; // damping ratio
        static constexpr float CD_FLOOR = 1e-3f;  // minimum drag coefficient
        
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

