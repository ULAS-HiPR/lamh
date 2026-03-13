#include "telemetry.h"


namespace task {

void Telemetry::run() {
    taskHandle_ = osThreadNew(&Telemetry::StartTelemetryEntry,
                              this,
                              &task_attributes);
}

void Telemetry::StartTelemetryEntry(void *argument) {
    auto *self = static_cast<Telemetry*>(argument);
    if (self) {
        self->StartStateMachine();
    }
}

void Telemetry::StartTelemetry() {

    printf("Telemetry started\n");

    for (;;) {
        char data = radio.read();
        State s_data = parse_message(data)
        osMessageQueuePut(telem_queue_, &s_data, 0, 0);
        osDelay(100);  
    }
}

static State Telemetry::parse_messge(char msg){
    if(msg == 'r'){
        return State::ROLL;
    } else if (msg == 'u'){
        return State::UNROLL;
    }else{
        return State::STOP;
    }
}

}