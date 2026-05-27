#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t fake_tick;
uint32_t HAL_GetTick(void);

#ifdef __cplusplus
}
#endif