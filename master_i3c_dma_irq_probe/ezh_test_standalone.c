/*
 * Standalone proof that I3C0 TX DMA requests can drive DMA0, whose completion IRQ wakes SmartDMA.
 */

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "fsl_device_registers.h"
#include "fsl_i3c.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"
#include "fsl_reset.h"
#include "fsl_smartdma.h"
#include "app.h"
#include "board.h"
#include "experiment_led.h"

#define I3C_DMA_PROOF_LENGTH 8U
#define I3C_DMA_PROOF_TIMEOUT 100000000U
#define I3C_DMA_PROOF_API_INDEX 0U
#define I3C_DMA_TX_CHANNEL 25U
#define I3C_DMA_PROOF_STARTUP_WAIT 5000000U
#define I3C_DMA_PROOF_DAA_RETRY_ATTEMPTS 3U
#define I3C_DMA_PROOF_DAA_RETRY_DELAY 5000000U
#define I3C_DMA_PROOF_I2C_BAUDRATE EXAMPLE_I2C_BAUDRATE
#define I3C_DMA_PROOF_I3C_OD_BAUDRATE EXAMPLE_I3C_OD_BAUDRATE
#define I3C_DMA_PROOF_I3C_PP_BAUDRATE 4000000U
#define SMART_DMA_TRIGGER_CHANNEL 0U
#define I3C_DMA_PROOF_LED_BOOT_PULSE_COUNT 2U
#define I3C_DMA_PROOF_LED_BOOT_PULSE_US 100000U
#define I3C_DMA_PROOF_LED_SUCCESS_PULSE_COUNT 3U
#define I3C_DMA_PROOF_LED_SUCCESS_PULSE_US 120000U

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

    prefix_length = snprintf(buffer, sizeof(buffer), "%s i3c-dma-irq-probe ", level);
    if ((prefix_length < 0) || (prefix_length >= (int)sizeof(buffer)))
    {
        semihost_write0("ERR i3c-dma-irq-probe log prefix overflow\n");
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

typedef struct _i3c_dma_irq_probe_param
{
    volatile uint32_t *mailbox;
    uint32_t *i3cBaseAddress;
    uint32_t lastByte;
} i3c_dma_irq_probe_param_t;

AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_DMA_MAX_CHANNELS],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_tx_buffer[I3C_DMA_PROOF_LENGTH], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static volatile uint32_t s_smartdma_mailbox, 4);
AT_NONCACHEABLE_SECTION_ALIGN(static i3c_dma_irq_probe_param_t s_probe_param, 4);

static volatile bool s_smartdma_irq_fired = false;

static void init_status_led(void)
{
    EXP_LED_Init();
    EXP_LED_Blink(false, false, true, I3C_DMA_PROOF_LED_BOOT_PULSE_COUNT, I3C_DMA_PROOF_LED_BOOT_PULSE_US);
    EXP_LED_Set(false, false, true);
}

static void set_success_led(void)
{
    EXP_LED_Blink(false, true, false, I3C_DMA_PROOF_LED_SUCCESS_PULSE_COUNT, I3C_DMA_PROOF_LED_SUCCESS_PULSE_US);
    EXP_LED_Set(false, true, false);
}

static void set_failure_led(void)
{
    EXP_LED_Set(true, false, false);
}

static void smartdma_completion_callback(void *param)
{
    (void)param;
    s_smartdma_irq_fired = true;
}

void SDMA_IRQHandler(void)
{
    SMARTDMA_HandleIRQ();
    SDK_ISR_EXIT_BARRIER;
}

static void clear_dma0_channel_state(void)
{
    const uint32_t channel_mask = (1UL << I3C_DMA_TX_CHANNEL);

    DMA0->COMMON[0].INTA = channel_mask;
    DMA0->COMMON[0].INTB = channel_mask;
    DMA0->COMMON[0].ERRINT = channel_mask;
    DMA0->COMMON[0].INTENCLR = channel_mask;
    DMA0->COMMON[0].ENABLECLR = channel_mask;
}

static void fill_tx_buffer(void)
{
    uint32_t index;

    for (index = 0U; index < I3C_DMA_PROOF_LENGTH; index++)
    {
        s_tx_buffer[index] = (uint8_t)(index + 1U);
    }
}

static status_t wait_for_smartdma_completion(void)
{
    volatile uint32_t timeout = 0U;

    while ((!s_smartdma_irq_fired) && (++timeout < I3C_DMA_PROOF_TIMEOUT))
    {
        __NOP();
    }

    if (!s_smartdma_irq_fired)
    {
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

static status_t wait_for_i3c_complete(I3C_Type *base)
{
    volatile uint32_t timeout = 0U;

    while (++timeout < I3C_DMA_PROOF_TIMEOUT)
    {
        uint32_t error_status = I3C_MasterGetErrorStatusFlags(base);
        if (error_status != 0U)
        {
            I3C_MasterClearErrorStatusFlags(base, error_status);
            return kStatus_Fail;
        }

        if ((I3C_MasterGetStatusFlags(base) & (uint32_t)kI3C_MasterCompleteFlag) != 0U)
        {
            I3C_MasterClearStatusFlags(base, (uint32_t)kI3C_MasterCompleteFlag);
            return kStatus_Success;
        }
    }

    return kStatus_Timeout;
}

static status_t run_cpu_rstdaa_and_daa(I3C_Type *base, uint8_t *slaveAddr)
{
    i3c_master_transfer_t masterXfer;
    uint8_t addressList[8] = {0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U};
    uint8_t devCount = 0U;
    i3c_device_info_t *devList;
    status_t result;
    uint32_t attempt;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = 0x7EU;
    masterXfer.subaddress = 0x06U;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;

    for (attempt = 0U; attempt < I3C_DMA_PROOF_DAA_RETRY_ATTEMPTS; attempt++)
    {
        I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
        I3C_MasterClearStatusFlags(base,
                                   (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                       (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                       (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
        base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

        result = I3C_MasterTransferBlocking(base, &masterXfer);
        if (result == kStatus_Success)
        {
            break;
        }

        EXP_LOG_ERROR("RSTDAA failed on attempt %lu: %d", (unsigned long)(attempt + 1U), result);

        for (volatile uint32_t delay = 0U; delay < I3C_DMA_PROOF_DAA_RETRY_DELAY; delay++)
        {
            __NOP();
        }
    }

    if (result != kStatus_Success)
    {
        return result;
    }

    result = I3C_MasterProcessDAA(base, addressList, ARRAY_SIZE(addressList));
    if (result != kStatus_Success)
    {
        EXP_LOG_ERROR("DAA failed: %d", result);
        return result;
    }

    devList = I3C_MasterGetDeviceListAfterDAA(base, &devCount);
    for (uint8_t devIndex = 0U; devIndex < devCount; devIndex++)
    {
        if (devList[devIndex].vendorID == 0x123U)
        {
            *slaveAddr = devList[devIndex].dynamicAddr;
            return kStatus_Success;
        }
    }

    EXP_LOG_ERROR("Target slave not found after DAA.");
    return kStatus_Fail;
}

static void dump_debug_state(I3C_Type *base)
{
    EXP_LOG_INFO("DMA0 INTSTAT=0x%08lx", (unsigned long)DMA0->INTSTAT);
    EXP_LOG_INFO("DMA0 INTA=0x%08lx", (unsigned long)DMA0->COMMON[0].INTA);
    EXP_LOG_INFO("DMA0 ACTIVE=0x%08lx", (unsigned long)DMA0->COMMON[0].ACTIVE);
    EXP_LOG_INFO("DMA0 CH25 CTLSTAT=0x%08lx", (unsigned long)DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].CTLSTAT);
    EXP_LOG_INFO("DMA0 CH25 XFERCFG=0x%08lx", (unsigned long)DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].XFERCFG);
    EXP_LOG_INFO("I3C MSTATUS=0x%08lx", (unsigned long)base->MSTATUS);
    EXP_LOG_INFO("I3C MDATACTRL=0x%08lx", (unsigned long)base->MDATACTRL);
    EXP_LOG_INFO("SmartDMA mailbox=%lu", (unsigned long)s_smartdma_mailbox);
}

static status_t run_i3c_dma_irq_proof(I3C_Type *base, uint8_t slaveAddr)
{
    const uint32_t bulk_count = I3C_DMA_PROOF_LENGTH - 1U;
    const uint32_t channel_mask = (1UL << I3C_DMA_TX_CHANNEL);
    const uint32_t xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(1U) | DMA_CHANNEL_XFERCFG_SETINTA(1U) |
                             DMA_CHANNEL_XFERCFG_WIDTH(0U) | DMA_CHANNEL_XFERCFG_SRCINC(1U) |
                             DMA_CHANNEL_XFERCFG_DSTINC(0U) |
                             DMA_CHANNEL_XFERCFG_XFERCOUNT(bulk_count - 1U);
    status_t result;

    s_smartdma_mailbox = 0U;
    s_smartdma_irq_fired = false;
    s_probe_param.mailbox = &s_smartdma_mailbox;
    s_probe_param.i3cBaseAddress = (uint32_t *)(uintptr_t)base;
    s_probe_param.lastByte = s_tx_buffer[I3C_DMA_PROOF_LENGTH - 1U];

    CLOCK_EnableClock(kCLOCK_Dmac0);
    RESET_PeripheralReset(kDMAC0_RST_SHIFT_RSTn);
    DMA0->CTRL = DMA_CTRL_ENABLE(1U);
    DMA0->SRAMBASE = (uint32_t)(uintptr_t)s_dma_descriptor_table;
    memset((void *)s_dma_descriptor_table, 0, sizeof(s_dma_descriptor_table));

    s_dma_descriptor_table[I3C_DMA_TX_CHANNEL].xfercfg = xfercfg;
    s_dma_descriptor_table[I3C_DMA_TX_CHANNEL].srcEndAddr = &s_tx_buffer[bulk_count - 1U];
    s_dma_descriptor_table[I3C_DMA_TX_CHANNEL].dstEndAddr = (void *)&base->MWDATAB;
    s_dma_descriptor_table[I3C_DMA_TX_CHANNEL].linkToNextDesc = NULL;

    clear_dma0_channel_state();
    DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].CFG = DMA_CHANNEL_CFG_PERIPHREQEN(1U);
    DMA0->COMMON[0].INTENSET = channel_mask;
    DMA0->COMMON[0].ENABLESET = channel_mask;
    DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].XFERCFG = xfercfg;

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base,
                               (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                   (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                   (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, SMART_DMA_TRIGGER_CHANNEL, kINPUTMUX_Dma0IrqToSmartDmaInput);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_I3c0TxToDmac0Ch25RequestEna, true);
    INPUTMUX_Deinit(INPUTMUX);

    NVIC_DisableIRQ(DMA0_IRQn);
    NVIC_ClearPendingIRQ(DMA0_IRQn);

    SMARTDMA_Init(
        SMARTDMA_SRAM_ADDR, __smartdma_start__, (uint32_t)((uintptr_t)__smartdma_end__ - (uintptr_t)__smartdma_start__));
    SMARTDMA_InstallCallback(smartdma_completion_callback, NULL);
    NVIC_ClearPendingIRQ(SDMA_IRQn);
    NVIC_SetPriority(SDMA_IRQn, 3);
    NVIC_EnableIRQ(SDMA_IRQn);
    SMARTDMA_Reset();
    SMARTDMA_Boot(I3C_DMA_PROOF_API_INDEX, &s_probe_param, 0);

    I3C_MasterEnableDMA(base, true, false, 1U);

    result = I3C_MasterStart(base, kI3C_TypeI3CSdr, slaveAddr, kI3C_Write);
    if (result != kStatus_Success)
    {
        return result;
    }

    result = I3C_MasterWaitForCtrlDone(base, false);
    if (result != kStatus_Success)
    {
        return result;
    }

    result = wait_for_smartdma_completion();
    if (result != kStatus_Success)
    {
        return result;
    }

    result = wait_for_i3c_complete(base);
    if (result != kStatus_Success)
    {
        return result;
    }

    result = I3C_MasterStop(base);

    I3C_MasterEnableDMA(base, false, false, 1U);
    clear_dma0_channel_state();

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_I3c0TxToDmac0Ch25RequestEna, false);
    INPUTMUX_Deinit(INPUTMUX);

    return result;
}

int main(void)
{
    i3c_master_config_t masterConfig;
    status_t result;
    uint8_t slaveAddr = 0U;

    BOARD_InitHardware();
    init_status_led();

    for (volatile uint32_t startup_delay = 0U; startup_delay < I3C_DMA_PROOF_STARTUP_WAIT; startup_delay++)
    {
        __NOP();
    }

    PRINTF("\r\nI3C DMA request to DMA0 IRQ to SmartDMA probe -- master.\r\n");
    EXP_LOG_INFO("I3C DMA request to DMA0 IRQ to SmartDMA probe -- master.");

    POWER_DisablePD(kPDRUNCFG_APD_SMARTDMA_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_SMARTDMA_SRAM);
    POWER_ApplyPD();

    fill_tx_buffer();
    keep_smartdma_api_alive();

    I3C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz.i2cBaud = I3C_DMA_PROOF_I2C_BAUDRATE;
    masterConfig.baudRate_Hz.i3cPushPullBaud = I3C_DMA_PROOF_I3C_PP_BAUDRATE;
    masterConfig.baudRate_Hz.i3cOpenDrainBaud = I3C_DMA_PROOF_I3C_OD_BAUDRATE;
    masterConfig.enableOpenDrainStop = false;
    I3C_MasterInit(EXAMPLE_MASTER, &masterConfig, I3C_MASTER_CLOCK_FREQUENCY);

    result = run_cpu_rstdaa_and_daa(EXAMPLE_MASTER, &slaveAddr);
    if (result != kStatus_Success)
    {
        EXP_LOG_ERROR("DAA setup failed: %d", result);
        set_failure_led();
        return -1;
    }

    EXP_LOG_INFO("Selected slave address: 0x%x", slaveAddr);
    EXP_LOG_INFO("Starting I3C0 TX DMA request proof.");

    result = run_i3c_dma_irq_proof(EXAMPLE_MASTER, slaveAddr);
    if (result != kStatus_Success)
    {
        EXP_LOG_ERROR("I3C DMA request proof failed: %d", result);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_smartdma_mailbox != 1U)
    {
        EXP_LOG_ERROR("SmartDMA mailbox mismatch: %lu", (unsigned long)s_smartdma_mailbox);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    EXP_LOG_INFO("I3C DMA request to DMA0 IRQ to SmartDMA proof successful.");
    PRINTF("I3C DMA request to DMA0 IRQ to SmartDMA proof successful.\r\n");
    set_success_led();
    return 0;
}
