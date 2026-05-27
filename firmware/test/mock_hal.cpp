#include "mock_hal.h"

uint32_t fake_tick = 0;

extern "C" uint32_t HAL_GetTick(void)
{
    return fake_tick;
}