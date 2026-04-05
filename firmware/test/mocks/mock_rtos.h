#pragma once

#include <queue>
#include <cstdint>

enum class State {
    ROLL,
    UNROLL,
    STOP
};

static std::queue<State> fakeQueue;

inline void osDelay(uint32_t) {}

inline uint32_t HAL_GetTick() {
    static uint32_t tick = 0;
    return tick += 100;
}

inline int osMessageQueueGet(void *, State *state, uint8_t, uint32_t) {
    if (fakeQueue.empty())
        return -1;

    *state = fakeQueue.front();
    fakeQueue.pop();
    return 0;
}

inline int osMessageQueuePut(void *, void *, uint8_t, uint32_t) {
    return 0;
}