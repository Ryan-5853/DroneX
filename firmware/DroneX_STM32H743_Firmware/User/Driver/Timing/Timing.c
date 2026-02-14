/**
 * @file Timing.c
 * @brief DWT 周期计数器实现。
 */

#include "Driver/Timing/Timing.h"
#include "Driver/Debug/Debug.h"
#include "main.h"

extern uint32_t SystemCoreClock;

void Timing_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    Debug_Printf("Timing initialized, SystemCoreClock=%lu Hz\r\n", (unsigned long)SystemCoreClock);
}

uint32_t Timing_GetCycles(void)
{
    return DWT->CYCCNT;
}

uint32_t Timing_CyclesToUs(uint32_t cycles)
{
    if (SystemCoreClock == 0) return 0;
    return (uint32_t)((uint64_t)cycles * 1000000ULL / SystemCoreClock);
}
