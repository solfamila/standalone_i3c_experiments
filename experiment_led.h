#ifndef EXPERIMENT_LED_H_
#define EXPERIMENT_LED_H_

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iopctl.h"

#ifndef EXP_LED_ACTIVE_LOW
#define EXP_LED_ACTIVE_LOW 1
#endif

#if EXP_LED_ACTIVE_LOW != 1
#error "MIMXRT595 EVK D19 RGB LED is active-low; do not override EXP_LED_ACTIVE_LOW."
#endif

#define EXP_LED_RED_PORT BOARD_LED_RED_GPIO_PORT
#define EXP_LED_RED_PIN BOARD_LED_RED_GPIO_PIN
#define EXP_LED_GREEN_PORT BOARD_LED_GREEN_GPIO_PORT
#define EXP_LED_GREEN_PIN BOARD_LED_GREEN_GPIO_PIN
#define EXP_LED_BLUE_PORT BOARD_LED_BLUE_GPIO_PORT
#define EXP_LED_BLUE_PIN BOARD_LED_BLUE_GPIO_PIN

static inline uint8_t EXP_LED_Level(bool on)
{
    /* MIMXRT595-EVK D19 RGB LED is active-low: low = on, high = off. */
    return on ? 0U : 1U;
}

static inline void EXP_LED_RawLevels(uint8_t red, uint8_t green, uint8_t blue)
{
    GPIO_PinWrite(GPIO, EXP_LED_RED_PORT, EXP_LED_RED_PIN, red);
    GPIO_PinWrite(GPIO, EXP_LED_GREEN_PORT, EXP_LED_GREEN_PIN, green);
    GPIO_PinWrite(GPIO, EXP_LED_BLUE_PORT, EXP_LED_BLUE_PIN, blue);
}

static inline void EXP_LED_EnableClocks(void)
{
    CLOCK_EnableClock(kCLOCK_HsGpio0);
    CLOCK_EnableClock(kCLOCK_HsGpio1);
    CLOCK_EnableClock(kCLOCK_HsGpio3);
}

static inline void EXP_LED_PinMuxGpio(uint32_t port, uint32_t pin)
{
#if defined(IOPCTL_PIO_FUNC0)
    const uint32_t pinConfig = IOPCTL_PIO_FUNC0 | IOPCTL_PIO_PUPD_DI | IOPCTL_PIO_INBUF_DI |
                               IOPCTL_PIO_SLEW_RATE_NORMAL | IOPCTL_PIO_FULLDRIVE_DI |
                               IOPCTL_PIO_ANAMUX_DI | IOPCTL_PIO_PSEDRAIN_DI | IOPCTL_PIO_INV_DI;
#else
    const uint32_t pinConfig = IOPCTL_FUNC0 | IOPCTL_PUPD_EN | IOPCTL_PULLUP_EN | IOPCTL_INBUF_EN;
#endif

    IOPCTL_PinMuxSet(IOPCTL, port, pin, pinConfig);
}

static inline void EXP_LED_Set(bool red, bool green, bool blue)
{
    EXP_LED_RawLevels(EXP_LED_Level(red), EXP_LED_Level(green), EXP_LED_Level(blue));
}

static inline void EXP_LED_Init(void)
{
    gpio_pin_config_t offConfig = {
        kGPIO_DigitalOutput,
        EXP_LED_Level(false),
    };

    EXP_LED_EnableClocks();

    /* Preload GPIO outputs before switching the pads to GPIO to avoid a white flash. */
    GPIO_PinInit(GPIO, EXP_LED_RED_PORT, EXP_LED_RED_PIN, &offConfig);
    GPIO_PinInit(GPIO, EXP_LED_GREEN_PORT, EXP_LED_GREEN_PIN, &offConfig);
    GPIO_PinInit(GPIO, EXP_LED_BLUE_PORT, EXP_LED_BLUE_PIN, &offConfig);

    EXP_LED_PinMuxGpio(EXP_LED_RED_PORT, EXP_LED_RED_PIN);
    EXP_LED_PinMuxGpio(EXP_LED_GREEN_PORT, EXP_LED_GREEN_PIN);
    EXP_LED_PinMuxGpio(EXP_LED_BLUE_PORT, EXP_LED_BLUE_PIN);

    EXP_LED_Set(false, false, false);
}

static inline void EXP_LED_ToggleBlue(void)
{
    GPIO_PortToggle(GPIO, EXP_LED_BLUE_PORT, 1UL << EXP_LED_BLUE_PIN);
}

static inline void EXP_LED_Blink(bool red, bool green, bool blue, uint32_t count, uint32_t delayUs)
{
    for (uint32_t index = 0U; index < count; index++)
    {
        EXP_LED_Set(red, green, blue);
        SDK_DelayAtLeastUs(delayUs, SystemCoreClock);
        EXP_LED_Set(false, false, false);
        SDK_DelayAtLeastUs(delayUs, SystemCoreClock);
    }
}

#endif /* EXPERIMENT_LED_H_ */