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

static void crude_delay(void)
{
    for (volatile uint32_t index = 0U; index < 20000000U; index++)
    {
        __NOP();
    }
}

static void dump_led_gpio(const char *label)
{
    char message[192];
    const uint32_t red = GPIO_PinRead(GPIO, EXP_LED_RED_PORT, EXP_LED_RED_PIN);
    const uint32_t green = GPIO_PinRead(GPIO, EXP_LED_GREEN_PORT, EXP_LED_GREEN_PIN);
    const uint32_t blue = GPIO_PinRead(GPIO, EXP_LED_BLUE_PORT, EXP_LED_BLUE_PIN);
    const uint32_t port0 = GPIO_PortRead(GPIO, 0U);
    const uint32_t port1 = GPIO_PortRead(GPIO, 1U);
    const uint32_t port3 = GPIO_PortRead(GPIO, 3U);

    (void)snprintf(message,
                   sizeof(message),
                   "%s readback: R=%lu G=%lu B=%lu P0=0x%08lx P1=0x%08lx P3=0x%08lx\n",
                   label,
                   (unsigned long)red,
                   (unsigned long)green,
                   (unsigned long)blue,
                   (unsigned long)port0,
                   (unsigned long)port1,
                   (unsigned long)port3);
    smoke_log(message);
}

static void show_raw_levels(uint8_t red, uint8_t green, uint8_t blue, const char *label)
{
    EXP_LED_RawLevels(red, green, blue);
    smoke_log(label);
    smoke_log("\n");
    dump_led_gpio(label);
    crude_delay();
}

int main(void)
{
    BOARD_InitHardware();
    EXP_LED_Init();
    smoke_log("LED smoke starting: raw GPIO forever\n");

    for (;;)
    {
        show_raw_levels(1U, 1U, 1U, "raw 111");
        show_raw_levels(0U, 1U, 1U, "raw 011");
        show_raw_levels(1U, 0U, 1U, "raw 101");
        show_raw_levels(1U, 1U, 0U, "raw 110");
        show_raw_levels(0U, 0U, 0U, "raw 000");
    }
}