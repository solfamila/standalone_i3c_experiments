/*
 * Standalone LED smoke test for RT595 EVK RGB LEDs.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fsl_common.h"
#include "app.h"
#include "board.h"
#include "experiment_led.h"

static void semihost_write0(const char *message)
{
    register unsigned int operation asm("r0") = 0x04U;
    register const char *parameter asm("r1") = message;

    __asm volatile(
        "bkpt 0xAB"
        : "+r"(operation)
        : "r"(parameter)
        : "memory");
}

static void smoke_log(const char *message)
{
    semihost_write0(message);
}

static void show_raw_levels(uint8_t red, uint8_t green, uint8_t blue, uint32_t delay_us, const char *label)
{
    char message[96];

    EXP_LED_RawLevels(red, green, blue);
    (void)snprintf(message, sizeof(message), "%s\n", label);
    smoke_log(message);
    SDK_DelayAtLeastUs(delay_us, SystemCoreClock);
}

int main(void)
{
    BOARD_InitHardware();
    EXP_LED_Init();
    smoke_log("LED smoke starting\n");

    show_raw_levels(1U, 1U, 1U, 700000U, "raw 111: expected off");
    show_raw_levels(0U, 1U, 1U, 700000U, "raw 011: expected red");
    show_raw_levels(1U, 0U, 1U, 700000U, "raw 101: expected green");
    show_raw_levels(1U, 1U, 0U, 700000U, "raw 110: expected blue");
    show_raw_levels(0U, 0U, 0U, 700000U, "raw 000: expected white");

    smoke_log("logical final green\n");
    EXP_LED_Set(false, true, false);
    SDK_DelayAtLeastUs(2000000U, SystemCoreClock);
    smoke_log("LED smoke completed: steady green\n");
    return 0;
}