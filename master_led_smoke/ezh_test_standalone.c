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

static void show_color(bool red, bool green, bool blue, uint32_t delay_us, const char *label)
{
    char message[64];

    EXP_LED_Set(red, green, blue);
    (void)snprintf(message, sizeof(message), "master LED: %s\n", label);
    smoke_log(message);
    SDK_DelayAtLeastUs(delay_us, SystemCoreClock);
}

int main(void)
{
    BOARD_InitHardware();
    EXP_LED_Init();
    smoke_log("LED smoke starting\n");

    for (uint32_t cycle = 0U; cycle < 3U; cycle++)
    {
        show_color(true, false, false, 500000U, "red");
        show_color(false, true, false, 500000U, "green");
        show_color(false, false, true, 500000U, "blue");
        show_color(false, false, false, 500000U, "off");
    }

    EXP_LED_Set(false, false, false);
    smoke_log("LED smoke completed\n");
    return 0;
}