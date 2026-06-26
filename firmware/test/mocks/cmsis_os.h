#pragma message("USING MOCK CMSIS OS")
#pragma once
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// ==========================
// PRIORITY DEFINITIONS
// ==========================

#define osPriorityLow           1
#define osPriorityNormal        2
#define osPriorityHigh          3
#define osPriorityRealtime      4

// ==========================
// CMSIS-RTOS TYPES (STUBS)
// ==========================

typedef void* osMessageQueueId_t;
typedef void* osThreadId_t;

typedef struct {
    const char *name;
    uint32_t attr_bits;
    void *cb_mem;
    uint32_t cb_size;
    void *stack_mem;
    uint32_t stack_size;
    uint32_t priority;
    uint32_t tz_module;
    uint32_t reserved;
} osThreadAttr_t;

typedef int32_t osStatus_t;

#define osOK 0

// ==========================
// RTOS FUNCTIONS (NO-OP)
// ==========================

inline void osDelay(uint32_t) {}

inline osMessageQueueId_t osMessageQueueNew(
    uint32_t, uint32_t, const void*) {
    return nullptr;
}

inline osStatus_t osMessageQueuePut(
    osMessageQueueId_t, const void*, uint8_t, uint32_t) {
    return osOK;
}

inline osStatus_t osMessageQueueGet(
    osMessageQueueId_t, void*, uint8_t*, uint32_t) {
    return osOK;
}

using osThreadFunc_t = void (*)(void *);

inline osThreadId_t osThreadNew(
    osThreadFunc_t func,
    void *argument,
    const osThreadAttr_t *attr)
{
    // optional: call immediately for SIL
    // func(argument);

    return nullptr;
}

#ifdef __cplusplus
}
#endif

