/*
 * Standalone proof that a DMA0 completion IRQ can wake SmartDMA through INPUTMUX.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"
#include "fsl_reset.h"
#include "fsl_smartdma.h"
#include "app.h"
#include "board.h"

#define DMA_PROOF_CHANNEL 0U
#define DMA_PROOF_BYTES 8U
#define DMA_PROOF_TIMEOUT 10000000U
#define DMA_PROOF_API_INDEX 0U
#define SMART_DMA_TRIGGER_CHANNEL 0U

extern uint8_t __smartdma_start__[];
extern uint8_t __smartdma_end__[];

void keep_smartdma_api_alive(void);

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

static void experiment_log(const char *level, const char *format, ...)
{
    char buffer[256];
    int prefix_length;
    va_list args;
    size_t used;

    prefix_length = snprintf(buffer, sizeof(buffer), "%s dma-irq-probe ", level);
    if ((prefix_length < 0) || (prefix_length >= (int)sizeof(buffer)))
    {
        semihost_write0("ERR dma-irq-probe log prefix overflow\n");
        return;
    }

    va_start(args, format);
    (void)vsnprintf(buffer + prefix_length,
                    sizeof(buffer) - (size_t)prefix_length,
                    format,
                    args);
    va_end(args);

    used = 0U;
    while ((used < sizeof(buffer)) && (buffer[used] != '\0'))
    {
        used++;
    }

    if ((used == 0U) || (buffer[used - 1U] != '\n'))
    {
        if (used < (sizeof(buffer) - 1U))
        {
            buffer[used++] = '\n';
            buffer[used] = '\0';
        }
        else
        {
            buffer[sizeof(buffer) - 2U] = '\n';
            buffer[sizeof(buffer) - 1U] = '\0';
        }
    }

    semihost_write0(buffer);
}

#define EXP_LOG_INFO(...) experiment_log("INF", __VA_ARGS__)
#define EXP_LOG_ERROR(...) experiment_log("ERR", __VA_ARGS__)

typedef struct _dma_descriptor
{
    uint32_t xfercfg;
    const void *srcEndAddr;
    void *dstEndAddr;
    struct _dma_descriptor *linkToNextDesc;
} dma_descriptor_t;

typedef struct _dma_irq_probe_param
{
    volatile uint32_t *mailbox;
} dma_irq_probe_param_t;

AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_DMA_MAX_CHANNELS],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_dma_source[DMA_PROOF_BYTES], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_dma_destination[DMA_PROOF_BYTES], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_smartdma_mailbox, 4);
AT_NONCACHEABLE_SECTION_ALIGN(static dma_irq_probe_param_t s_dma_irq_probe_param, 4);

static volatile bool s_smartdma_irq_fired = false;

static void smartdma_completion_callback(void *param)
{
    (void)param;

    s_smartdma_irq_fired = true;
}

void SDMA_DriverIRQHandler(void)
{
    SMARTDMA_HandleIRQ();
    SDK_ISR_EXIT_BARRIER;
}

static void fill_dma_buffers(void)
{
    uint32_t index;

    for (index = 0U; index < DMA_PROOF_BYTES; index++)
    {
        s_dma_source[index] = (uint8_t)(0xA0U + index);
        s_dma_destination[index] = 0U;
    }
}

static void clear_dma0_channel_state(void)
{
    const uint32_t channel_mask = (1UL << DMA_PROOF_CHANNEL);

    DMA0->COMMON[0].INTA = channel_mask;
    DMA0->COMMON[0].INTB = channel_mask;
    DMA0->COMMON[0].ERRINT = channel_mask;
    DMA0->COMMON[0].INTENCLR = channel_mask;
    DMA0->COMMON[0].ENABLECLR = channel_mask;
}

static void start_dma0_memory_copy(void)
{
    const uint32_t channel_mask = (1UL << DMA_PROOF_CHANNEL);
    const uint32_t xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(1U) | DMA_CHANNEL_XFERCFG_SETINTA(1U) |
                             DMA_CHANNEL_XFERCFG_WIDTH(0U) | DMA_CHANNEL_XFERCFG_SRCINC(1U) |
                             DMA_CHANNEL_XFERCFG_DSTINC(1U) |
                             DMA_CHANNEL_XFERCFG_XFERCOUNT(DMA_PROOF_BYTES - 1U);

    CLOCK_EnableClock(kCLOCK_Dmac0);
    RESET_PeripheralReset(kDMAC0_RST_SHIFT_RSTn);

    DMA0->CTRL = DMA_CTRL_ENABLE(1U);
    DMA0->SRAMBASE = (uint32_t)(uintptr_t)s_dma_descriptor_table;

    memset((void *)s_dma_descriptor_table, 0, sizeof(s_dma_descriptor_table));

    s_dma_descriptor_table[DMA_PROOF_CHANNEL].xfercfg = xfercfg;
    s_dma_descriptor_table[DMA_PROOF_CHANNEL].srcEndAddr = &s_dma_source[DMA_PROOF_BYTES - 1U];
    s_dma_descriptor_table[DMA_PROOF_CHANNEL].dstEndAddr = &s_dma_destination[DMA_PROOF_BYTES - 1U];
    s_dma_descriptor_table[DMA_PROOF_CHANNEL].linkToNextDesc = NULL;

    clear_dma0_channel_state();

    DMA0->CHANNEL[DMA_PROOF_CHANNEL].CFG = 0U;
    DMA0->COMMON[0].INTENSET = channel_mask;
    DMA0->COMMON[0].ENABLESET = channel_mask;
    DMA0->CHANNEL[DMA_PROOF_CHANNEL].XFERCFG = xfercfg | DMA_CHANNEL_XFERCFG_SWTRIG(1U);
}

static bool wait_for_smartdma_wakeup(void)
{
    volatile uint32_t timeout = 0U;

    while ((!s_smartdma_irq_fired) && (++timeout < DMA_PROOF_TIMEOUT))
    {
        __NOP();
    }

    return s_smartdma_irq_fired;
}

static void dump_dma_status(void)
{
    PRINTF("DMA0 INTSTAT=0x%08lx\r\n", (unsigned long)DMA0->INTSTAT);
    PRINTF("DMA0 INTA=0x%08lx\r\n", (unsigned long)DMA0->COMMON[0].INTA);
    PRINTF("DMA0 ACTIVE=0x%08lx\r\n", (unsigned long)DMA0->COMMON[0].ACTIVE);
    PRINTF("DMA0 CHANNEL0 CTLSTAT=0x%08lx\r\n", (unsigned long)DMA0->CHANNEL[DMA_PROOF_CHANNEL].CTLSTAT);
    PRINTF("DMA0 CHANNEL0 XFERCFG=0x%08lx\r\n", (unsigned long)DMA0->CHANNEL[DMA_PROOF_CHANNEL].XFERCFG);
    PRINTF("SmartDMA mailbox=%lu\r\n", (unsigned long)s_smartdma_mailbox);

    EXP_LOG_INFO("DMA0 INTSTAT=0x%08lx", (unsigned long)DMA0->INTSTAT);
    EXP_LOG_INFO("DMA0 INTA=0x%08lx", (unsigned long)DMA0->COMMON[0].INTA);
    EXP_LOG_INFO("DMA0 ACTIVE=0x%08lx", (unsigned long)DMA0->COMMON[0].ACTIVE);
    EXP_LOG_INFO("DMA0 CHANNEL0 CTLSTAT=0x%08lx", (unsigned long)DMA0->CHANNEL[DMA_PROOF_CHANNEL].CTLSTAT);
    EXP_LOG_INFO("DMA0 CHANNEL0 XFERCFG=0x%08lx", (unsigned long)DMA0->CHANNEL[DMA_PROOF_CHANNEL].XFERCFG);
    EXP_LOG_INFO("SmartDMA mailbox=%lu", (unsigned long)s_smartdma_mailbox);
}

int main(void)
{
    size_t index;

    BOARD_InitHardware();

    PRINTF("\r\nDMA0 IRQ to SmartDMA probe -- master only.\r\n");
    EXP_LOG_INFO("DMA0 IRQ to SmartDMA probe -- master only.");

    POWER_DisablePD(kPDRUNCFG_APD_SMARTDMA_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_SMARTDMA_SRAM);
    POWER_ApplyPD();

    keep_smartdma_api_alive();
    fill_dma_buffers();

    s_smartdma_mailbox = 0U;
    s_smartdma_irq_fired = false;
    s_dma_irq_probe_param.mailbox = &s_smartdma_mailbox;

    NVIC_DisableIRQ(DMA0_IRQn);
    NVIC_ClearPendingIRQ(DMA0_IRQn);

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, SMART_DMA_TRIGGER_CHANNEL, kINPUTMUX_Dma0IrqToSmartDmaInput);
    INPUTMUX_Deinit(INPUTMUX);

    SMARTDMA_Init(
        SMARTDMA_SRAM_ADDR, __smartdma_start__, (uint32_t)((uintptr_t)__smartdma_end__ - (uintptr_t)__smartdma_start__));
    SMARTDMA_InstallCallback(smartdma_completion_callback, NULL);
    NVIC_ClearPendingIRQ(SDMA_IRQn);
    NVIC_SetPriority(SDMA_IRQn, 3);
    NVIC_EnableIRQ(SDMA_IRQn);
    SMARTDMA_Reset();
    SMARTDMA_Boot(DMA_PROOF_API_INDEX, &s_dma_irq_probe_param, 0);

    EXP_LOG_INFO("SmartDMA armed on DMA0 IRQ input.");

    start_dma0_memory_copy();
    EXP_LOG_INFO("DMA0 software-triggered transfer started.");

    if (!wait_for_smartdma_wakeup())
    {
        PRINTF("DMA0 IRQ to SmartDMA proof timed out.\r\n");
        EXP_LOG_ERROR("DMA0 IRQ to SmartDMA proof timed out.");
        dump_dma_status();
        return 1;
    }

    clear_dma0_channel_state();
    SMARTDMA_Reset();

    if (s_smartdma_mailbox != 1U)
    {
        PRINTF("SmartDMA mailbox mismatch: %lu\r\n", (unsigned long)s_smartdma_mailbox);
        EXP_LOG_ERROR("SmartDMA mailbox mismatch: %lu", (unsigned long)s_smartdma_mailbox);
        dump_dma_status();
        return 1;
    }

    if (memcmp((const void *)s_dma_source, (const void *)s_dma_destination, sizeof(s_dma_source)) != 0)
    {
        PRINTF("DMA memory copy mismatch.\r\n");
        EXP_LOG_ERROR("DMA memory copy mismatch.");
        for (index = 0U; index < DMA_PROOF_BYTES; index++)
        {
            PRINTF("  [%u] src=0x%02x dst=0x%02x\r\n",
                   (unsigned int)index,
                   s_dma_source[index],
                   s_dma_destination[index]);
            EXP_LOG_ERROR("[%u] src=0x%02x dst=0x%02x",
                          (unsigned int)index,
                          s_dma_source[index],
                          s_dma_destination[index]);
        }
        dump_dma_status();
        return 1;
    }

    PRINTF("DMA0 IRQ to SmartDMA proof successful.\r\n");
    EXP_LOG_INFO("DMA0 IRQ to SmartDMA proof successful.");
    return 0;
}