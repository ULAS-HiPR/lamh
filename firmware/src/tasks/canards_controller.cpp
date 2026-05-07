#include "canards_controller.h"

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
        flight_data data;
        osStatus_t status;
        status = osMessageQueueGet(can_queue_, &data, 0U, 0U);   // wait for message

        if (status == osOK) {
            //data parsing logic here
            printf("Received CAN data: %d\n", data.state);

            //parse data
            if (safety_check(data.state, data.core_data.imu)) {
                run_canards_controller(data.core_data.imu, data.core_data.barometer, data.prediction);
            } else {
                stop_action();
            }
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

        osDelay(10);  
    }
}

float Canards_Controller::get_rocket_angle(imu_data imu) {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time_ms) / 1000.0f;
    last_time_ms = now;

    // Trapezoidal integration
    float theta = rocket_angle + 0.5f * (prev_roll_rate + imu.gyro.x) * dt;
    prev_roll_rate = imu.gyro.x;

    // Clamp to ±20 degrees (±0.349066 rad)
    const float THETA_MAX_RAD = 0.349066f;
    theta = fmaxf(-THETA_MAX_RAD, fminf(THETA_MAX_RAD, theta));

    return theta;
}

float Canards_Controller::run_canards_controller(imu_data imu, baro_data baro, prediction_data prediction) {
    float rho = baro.pressure / (287.0f * baro.temperature);
    float q = 0.5f * rho * prediction.velocity * prediction.velocity;
    float M_alpha = N_canards * q * S_can * C_L_alpha_can * Y_cp_can; 

    float Kp = (I_xx * omega_n * omega_n) / M_alpha;
    float Kd = (2.0f * damp_ratio * omega_n * I_xx) / M_alpha;
    
    rocket_angle = get_rocket_angle(imu);
    output = Kp * rocket_angle + Kd * imu.gyro.x; // assuming gyro.x is the roll rate in rad/s
    output_degrees = output * (180.0f / 3.14159f); 
    servo_angle = output_degrees + 90.0f; 
    return servo_angle;
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