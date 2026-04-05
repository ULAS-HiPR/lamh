#include "canards_controller.h"
#include "logger.h"
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
          printf("data is %d", data);
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

        osDelay(CANARD_HZ);  
    }
}

bool Canards_Controller::init(){
    //check servo signal is good
    //servo.spin(0)
    //servo.spin(180)
    //check all other stuff
    return true;
}

void Canards_Controller::run_canards_controller() {
    //servo_.set_position(ROLL_POSITION);
    printf("position");
    osDelay(1000);
    //servo_.set_position(ROLL_HOLD_POSITION);
};


void Canards_Controller::stop_action() {
    //servo_.set_position(STOP_POSITION);
    printf("position");
};

}