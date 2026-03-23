#include "state_machine.h"
#include <cstdio>

namespace task {

void State_Machine::run() {
    taskHandle_ = osThreadNew(&State_Machine::StartStateMachineEntry,
                              this,
                              &task_attributes);
}

void State_Machine::StartStateMachineEntry(void *argument) {
    printf("State machine starting1\n");
    auto *self = static_cast<State_Machine*>(argument);
    printf("State machine starting1\n");
    if (self) {
        self->StartStateMachine();
    }
}

void State_Machine::StartStateMachine() {

    printf("State machine started\n");
    imu_.init();
    imu_data data;

    for (;;) {
        imu_.update(&data);
        State active_state;
        osMessageQueueGet(&telem_queue_, &active_state, 0, 0); //freezing here 
        switch (active_state)
        {
            case State::ROLL:
                do_roll_action();
            case State::UNROLL:
                do_unroll_action();
            case State::STOP:
                stop_action();
            default:
                stop_action();
        }

        uint32_t msg = HAL_GetTick();
        osMessageQueuePut(logger_queue_, &msg, 0, 0);

        osDelay(100);  
    }
}

void State_Machine::do_roll_action() {
    //servo_.set_position(ROLL_POSITION);
    printf("position");
    osDelay(1000);
    //servo_.set_position(ROLL_HOLD_POSITION);
};

void State_Machine::do_unroll_action() {
    //servo_.set_position(UNROLL_POSITION);
    printf("position");
    osDelay(1000);
    printf("position");
    //servo_.set_position(UNROLL_HOLD_POSITION);
};

void State_Machine::stop_action() {
    //servo_.set_position(STOP_POSITION);
    printf("position");
};

}