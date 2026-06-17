#include "canards_controller.h"
#include <ratio>

namespace task {

void Canards_Controller::run() {
    taskHandle_ = osThreadNew(&Canards_Controller::StartCanardsControllerEntry,
                              this,
                              &task_attributes);
}

void Canards_Controller::StartCanardsControllerEntry(void *argument) {
    auto *self = static_cast<Canards_Controller*>(argument);
    if (self) {
        self->StartCanardsController();
    }
}

void Canards_Controller::StartCanardsController() {
    servo_.init();

    // start at centre so servo doesn't snap on boot
    servo_.set_position(90);
    osDelay(500);

    flight_data flight_data_in;

    for (;;) {
        osStatus_t status;
        status = osMessageQueueGet(can_queue_, &flight_data_in, NULL, 10U);   // wait for message

        if (status == osOK) {
            printf("Received CAN data: %d\n", flight_data_in.state);
            canards_raw canards_data = run_canards_controller(flight_data_in.core_data.imu, flight_data_in.core_data.barometer, flight_data_in.prediction);
            HAL_GPIO_ReadPin(active_port_, active_pin_) ? canards_data.active = true : canards_data.active = false;

            if (safety_check(canards_data.active, flight_data_in.core_data.imu)) {
                servo_.set_position(canards_data.servo_angle);
            } else {
                stop_action();  // keep neutral
            }

        osMessageQueuePut(logger_queue_, &canards_data, 0, 0);
        }
        osDelay(CANARDS_DELAY_MS);  
    }
}

// to change to airbrakes algo
canards_raw Canards_Controller::run_canards_controller(const imu_data& imu, const baro_data& baro, const prediction_data& pred) {
    float rho = baro.pressure / (287.0f * baro.temperature);
    float q = 0.5f * rho * pred.velocity * pred.velocity;
    float M_alpha = N_canards * q * S_can * C_L_alpha_can * Y_cp_can; 
    if (fabs(M_alpha) < 1e-3f) {
        return {0.0f, 0.0f, 90.0f};
    }

    float Kp = (I_xx * omega_n * omega_n) / M_alpha;
    float Kd = (2.0f * damp_ratio * omega_n * I_xx) / M_alpha;
    
    get_rocket_angle(imu);
    printf("Rocket angle (rad): %.4f\n", rocket_angle);
    output = Kp * rocket_angle + Kd * imu.gyro.x; // assuming gyro.x is the roll rate in rad/s
    output_degrees = output * (180.0f / 3.14159f); 
    servo_angle = output_degrees + 90.0f; 
    servo_angle = fmaxf(82.0f, fminf(98.0f, servo_angle));
    return {Kp, Kd, servo_angle};
};

float Canards_Controller::get_rocket_angle(const imu_data& imu) {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time_ms) * 0.001f;
    last_time_ms = now;
    printf("Delta time (s): %.4f\n", dt);


    // Trapezoidal integration
    float theta = rocket_angle + 0.5f * (prev_roll_rate + imu.gyro.x) * dt;
    prev_roll_rate = imu.gyro.x;

    // Clamp to ±20 degrees (±0.349066 rad)
    const float THETA_MAX_RAD = 0.349066f;
    theta = fmaxf(-THETA_MAX_RAD, fminf(THETA_MAX_RAD, theta));
    rocket_angle = theta; 

    return theta;
}

void Canards_Controller::stop_action() {
    //servo_.set_position(STOP_POSITION);
    printf("position");
};


bool Canards_Controller::safety_check(bool active, const imu_data& imu) {
    // only use airbrakes when cots computer signals coasting
    if (active == false) {
        return false; 
    }
    
    // check tilt angle from IMU, if too large, stop the canards to prevent further instability
    if (imu.acceleration.x > 50.0f || imu.acceleration.x < -50.0f) {
        return false; 
    }
    return true;

}
}