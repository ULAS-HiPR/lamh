#include "canards_controller.h"
#include "logger.h"
#include <cstdint>
#include <cstdio>

namespace task {

void Canards_Controller::run() {
    taskHandle_ = osThreadNew(&Canards_Controller::StartCanardsControllerEntry,
                              this,
                              &task_attributes);
}

void Canards_Controller::StartCanardsControllerEntry(void *argument) {
    printf("Canards_Controller starting1\n");
    auto *self = static_cast<Canards_Controller*>(argument);
    printf("Canards_Controller starting1\n");
    if (self) {
        self->StartCanardsController();
    }
}

void Canards_Controller::StartCanardsController() {

    printf("Canards_Controller started\n");

    for (;;) {
        //get servo last position?
        char data;
        osStatus_t status;
        status = osMessageQueueGet(can_queue_, &data, NULL, 0U);   // wait for message

        if (status == osOK) {
            //data parsing logic here
            printf("Received CAN data: %c\n", data);

            //parse data
        //    if (safety_check(data.state, data.imu)) {
        //       // run_canards_controller(data.sampleCount, data.imu, data.baro);
        //    } else {
        //        stop_action();
        //    }
        }
      
        //uint32_t msg = HAL_GetTick();

        //task::Logger::LogMessage log_msg {
        //    .timestamp = msg,
        //    .imu = data,
        //    .fsm_state = active_state
        //    //add in servo position?
        //};
//
        //osMessageQueuePut(logger_queue_, &log_msg, 0, 0);

        osDelay(100);  
    }
}

void Canards_Controller::run_canards_controller(uint16_t sampleCount, imu_data imu, baro_data baro) {
    //T0 = baro.temperature;
    //float pressure = baro.pressure;
    //float rho = pressure / (287.05 * T0);
    //float q = 0.5 * rho * speedData[sampleCount] * speedData[sampleCount];
    //float M_alpha = N_CANARDS * q * S_CAN * CL_ALPHA * Y_CP_CAN; 
//
    //Kp = (I_XX * OMEGA_N * OMEGA_N) / M_alpha;  
    //Kd = (2.0 * DAMP_RATIO * OMEGA_N * I_XX) / M_alpha;
    osDelay(1000);
    //servo_.set_position(ROLL_HOLD_POSITION);
};


void Canards_Controller::stop_action() {
    //servo_.set_position(STOP_POSITION);
    printf("position");
};

bool Canards_Controller::safety_check(int state, imu_data imu) {
    // only use canards in coating
    //if (state != state::COASTING) {
    //    return false; 
    //}
    
    // check tilt angle from IMU, if too large, stop the canards to prevent further instability
    if (imu.acceleration.x > 50.0f || imu.acceleration.x < -50.0f) {
        return false; 
    }
    return true; // Placeholder for actual safety check logic

}
}