#include "state_machine.h"


namespace task {

void State_Machine::run() {
    taskHandle_ = osThreadNew(&State_Machine::StartStateMachineEntry,
                              this,
                              &task_attributes);
}

void State_Machine::StartStateMachineEntry(void *argument) {
    auto *self = static_cast<State_Machine*>(argument);
    if (self) {
        self->StartStateMachine();
    }
}

void State_Machine::StartStateMachine() {

    printf("State machine started\n");

    for (;;) {
        imu_.update();
        State active_state;
        osMessageQueueGet(&telem_queue_, &active_state, 0, 0);
        switch (active_state)
        {
            case State::ROLL:
                do_roll_action();
            case State::UNROLL:
                do_unroll_action();
            case State::STOP:
                stop_action():
            default:
                stop_action();
        }

        uint32_t msg = HAL_GetTick();
        osMessageQueuePut(logger_queue_, &msg, 0, 0);

        osDelay(100);  
    }
}

}
