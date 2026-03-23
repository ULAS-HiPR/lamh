#include "state_machine.h"
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
        self->StartTelemetry();
    }
}

void Telemetry::StartTelemetry() {

    printf("Telemetry started\n");

    for (;;) {
        //char data = radio.read();
        char data = 'r'; // placeholder for testing
        task::State_Machine::State s_data = parse_message(data);
        printf("Parsed message: %d\n", s_data);
        osMessageQueuePut(telem_queue_, &s_data, 0, 0);
        osDelay(100);  
    }
}

task::State_Machine::State Telemetry::parse_message(char msg){
    if(msg == 'r'){
        return task::State_Machine::State::ROLL;
    } else if (msg == 'u'){
        return task::State_Machine::State::UNROLL;
    }else{
        return task::State_Machine::State::STOP;
    }
}

}