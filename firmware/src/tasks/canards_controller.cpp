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
    bool servo_init = servo_.init();

    // start at centre so servo doesn't snap on boot
    servo_.set_position(180);
    osDelay(1000);
    servo_.set_position(15);

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
    // density of air = pressure / (R * temperature)
    float rho = baro.pressure / (287.058f * (baro.temperature + 273.15f));  // kg/m^3

    // dt since last call 
    uint32_t now = HAL_GetTick();
    float dt = (now - last_airbrake_time_ms_) * 0.001f;
    last_airbrake_time_ms_ = now;
    if (dt <= 0.0f) {
        dt = 0.02f;   
    }

    float mach = pred.velocity / speedOfSound;
    float machClamped = fmaxf(0.3f, fminf(1.1f, mach));


    float d = last_deployment_;
    float M = machClamped;
    float PreviousCd =
        (-0.0027f * d)
        + (0.0197f * d * M)
        - (0.0394f * d * powf(M, 2.0f))
        + (0.0246f * d * powf(M, 3.0f))
        + (0.0035f * powf(d, 2.0f))
        - (0.0086f * powf(d, 2.0f) * M)
        + (0.0179f * powf(d, 2.0f) * powf(M, 2.0f))
        - (0.0118f * powf(d, 2.0f) * powf(M, 3.0f))
        + 0.4 ;
    

    PreviousCd = fmaxf(PreviousCd, CD_FLOOR);

    float aDrag = (rho * PreviousCd * areaMax * pred.velocity * pred.velocity) / (2.0f * mass);

    if (fabsf(pred.velocity) < 1.0f) {
        servo_angle = 180.0f;  // airbrakes stowed
        return {0.0f, 0.0f, servo_angle};
    }

    float Kp = (4.0f * mass * powf(g + aDrag, 2.0f)) / (rho * PreviousCd * areaMax * powf(pred.velocity, 4.0f));
    float Kd = (8.0f * zeta * mass * powf(g + aDrag, 2.0f)) / (rho * PreviousCd * areaMax * powf(pred.velocity, 4.0f));
    
    // Apogee prediction
    // ApogeePredicted = CurrentAltitude + VerticalVelocity^2 / (2*(g+aDrag))
    float ApogeePredicted = baro.altitude + (pred.velocity * pred.velocity) / (2.0f * (g + aDrag));

    float Error = ApogeePredicted - apogeeDesired;
    float ErrorRate = (Error - previous_error_) / dt;
    previous_error_ = Error;

    float Proportional = Kp * Error;
    float Derivative = Kd * ErrorRate;
    float output = Proportional + Derivative;

    // Clamp output (deployment level) 0..1
    output = fmaxf(0.0f, fminf(1.0f, output));

    // Remember deployment level for next iteration's PreviousCd calc
    last_deployment_ = output;

    // Map deployment level [0,1] -> servo angle: 180 = fully in, 15 = fully out
    servo_angle = 180.0f + output * (15.0f - 180.0f);

    return {Kp, Kd, servo_angle};
};

void Canards_Controller::stop_action() {
    servo_.set_position(180);   //neutral position
};


bool Canards_Controller::safety_check(bool active, const imu_data& imu) {
    // only use airbrakes when cots computer signals coasting
    if (active == false) {
        return false; 
    }
    
    // check tilt angle from IMU, if too large, stop the canards to prevent further instability
    // for airbrakes indicated 1 has broken
    if (imu.acceleration.x > 50.0f || imu.acceleration.x < -50.0f) {
        return false; 
    }
    return true;

}
}