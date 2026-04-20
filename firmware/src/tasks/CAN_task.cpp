#include "CAN_task.h"


namespace task {

void CAN_task::run() {
    taskHandle_ = osThreadNew(&CAN_task::StartCANEntry,
                              this,
                              &task_attributes);
}

void CAN_task::StartCANEntry(void *argument) {
    auto *self = static_cast<CAN_task*>(argument);
    if (self) {
        self->StartCAN();
    }
}

void CAN_task::StartCAN() {

    printf("CAN started\n");

    for (;;) {
        //char data = radio.read();
        char data = 'r'; // placeholder for testing
        //  get data from can bus
        // parse data & send to controller
        printf("Parsed message: %d\n", data);
        osMessageQueuePut(can_queue_, &data, 0, 0);
        osDelay(1000);  
    }
}

char CAN_task::parse_message(char msg){
    // placeholder for parsing logic
    return msg;
}

}