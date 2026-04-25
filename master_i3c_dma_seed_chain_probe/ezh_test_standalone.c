/*
 * Standalone proof that a prelinked chain of one-byte I3C TX DMA seed
 * descriptors can wake SmartDMA repeatedly without software rearming CH25.
 */

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
#include "fsl_iopctl.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"
#include "fsl_reset.h"
#include "fsl_smartdma.h"
#include "app.h"
#include "board.h"

#define I3C_DMA_SEED_CHAIN_LENGTH 8U
#define I3C_DMA_SEED_CHAIN_TIMEOUT 100000000U
#define I3C_DMA_SEED_CHAIN_API_INDEX 0U
#define I3C_DMA_TX_CHANNEL 25U
#define I3C_DMA_SEED_CHAIN_STARTUP_WAIT 5000000U
#define I3C_DMA_SEED_CHAIN_DAA_RETRY_ATTEMPTS 3U
#define I3C_DMA_SEED_CHAIN_DAA_RETRY_DELAY 5000000U
#define I3C_DMA_SEED_CHAIN_I2C_BAUDRATE EXAMPLE_I2C_BAUDRATE
#define I3C_DMA_SEED_CHAIN_I3C_OD_BAUDRATE EXAMPLE_I3C_OD_BAUDRATE
#define I3C_DMA_SEED_CHAIN_I3C_PP_BAUDRATE 4000000U
#define I3C_DMA_SEED_CHAIN_LED_TOGGLE_INTERVAL 250000U
#define I3C_DMA_SEED_CHAIN_LED_SELF_TEST_PULSE_US 300000U
#define I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_US 200000U
#define I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_COUNT 2U
#define SMART_DMA_TRIGGER_CHANNEL 0U

#define I3C_PROTOCOL_IRQ_MASK ((uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag | \
                               (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag | \
                               (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag)

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

    prefix_length = snprintf(buffer, sizeof(buffer), "%s i3c-dma-seed-chain-probe ", level);
    if ((prefix_length < 0) || (prefix_length >= (int)sizeof(buffer)))
    {
        semihost_write0("ERR i3c-dma-seed-chain-probe log prefix overflow\n");
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

typedef struct _i3c_dma_seed_chain_param
{
    volatile uint32_t mailbox;
    uint32_t expectedWakeCount;
    volatile uint32_t wakeCount;
    volatile uint32_t dmaSeedBytes;
    volatile uint32_t dmaIntaCount;
    uint32_t dmaIntaAddress;
    uint32_t dmaChannelMask;
} i3c_dma_seed_chain_param_t;

typedef struct _seed_chain_failure_snapshot
{
    bool valid;
    uint32_t dmaIntStat;
    uint32_t dmaInta;
    uint32_t dmaActive;
    uint32_t dmaCfg;
    uint32_t dmaCtlStat;
    uint32_t dmaXferCfg;
    uint32_t i3cMdmaCtrl;
    uint32_t i3cMstatus;
    uint32_t i3cMdataCtrl;
} seed_chain_failure_snapshot_t;

AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_DMA_MAX_CHANNELS],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_seed_chain[I3C_DMA_SEED_CHAIN_LENGTH - 1U],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_tx_buffer[I3C_DMA_SEED_CHAIN_LENGTH], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static i3c_dma_seed_chain_param_t s_probe_param, 4);

static volatile bool s_smartdma_irq_fired = false;
static volatile uint32_t s_i3c_irq_status_latched = 0U;
static volatile uint32_t s_cm33_i3c_irq_count = 0U;
static volatile uint32_t s_cm33_i3c_data_irq_count = 0U;
static volatile uint32_t s_cm33_i3c_protocol_irq_count = 0U;
static seed_chain_failure_snapshot_t s_failure_snapshot;
static bool s_transfer_led_active = false;
static uint32_t s_transfer_led_poll_count = 0U;
static uint32_t s_transfer_led_toggle_count = 0U;

static void master_led_red_on(void)
{
    LED_RED_ON();
}

static void master_led_red_off(void)
{
    LED_RED_OFF();
}

static void master_led_green_on(void)
{
    LED_GREEN_ON();
}

static void master_led_green_off(void)
{
    LED_GREEN_OFF();
}

static void master_led_blue_on(void)
{
    LED_BLUE_ON();
}

static void master_led_blue_off(void)
{
    LED_BLUE_OFF();
}

static void master_led_blue_toggle(void)
{
    LED_BLUE_TOGGLE();
}

static void master_led_all_off(void)
{
    master_led_red_off();
    master_led_green_off();
    master_led_blue_off();
}

static void configure_transfer_led_pins(void)
{
    const uint32_t led_pin_config = IOPCTL_FUNC0 | IOPCTL_PUPD_EN | IOPCTL_PULLUP_EN | IOPCTL_INBUF_EN;

    CLOCK_EnableClock(kCLOCK_HsGpio0);
    CLOCK_EnableClock(kCLOCK_HsGpio1);
    CLOCK_EnableClock(kCLOCK_HsGpio3);

    IOPCTL_PinMuxSet(IOPCTL, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, led_pin_config);
    IOPCTL_PinMuxSet(IOPCTL, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, led_pin_config);
    IOPCTL_PinMuxSet(IOPCTL, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, led_pin_config);
}

static void transfer_led_delay_us(uint32_t delay_us)
{
    SDK_DelayAtLeastUs(delay_us, SystemCoreClock);
}

static void run_boot_led_self_test(void)
{
    master_led_all_off();

    master_led_red_on();
    transfer_led_delay_us(I3C_DMA_SEED_CHAIN_LED_SELF_TEST_PULSE_US);
    master_led_red_off();
    transfer_led_delay_us(I3C_DMA_SEED_CHAIN_LED_SELF_TEST_PULSE_US);

    master_led_green_on();
    transfer_led_delay_us(I3C_DMA_SEED_CHAIN_LED_SELF_TEST_PULSE_US);
    master_led_green_off();
    transfer_led_delay_us(I3C_DMA_SEED_CHAIN_LED_SELF_TEST_PULSE_US);
}

static void ensure_visible_transfer_activity(void)
{
    uint32_t pulse_index;

    if (s_transfer_led_toggle_count != 0U)
    {
        return;
    }

    master_led_red_off();
    master_led_green_off();
    for (pulse_index = 0U; pulse_index < I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_COUNT; pulse_index++)
    {
        master_led_blue_on();
        transfer_led_delay_us(I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_US);
        master_led_blue_off();
        transfer_led_delay_us(I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_US);
    }
}

static void init_transfer_led(void)
{
    configure_transfer_led_pins();
    LED_RED_INIT(LOGIC_LED_OFF);
    LED_GREEN_INIT(LOGIC_LED_OFF);
    LED_BLUE_INIT(LOGIC_LED_OFF);
    master_led_all_off();
}

static void set_success_led(void)
{
    master_led_red_off();
    master_led_green_off();
    master_led_blue_on();
}

static void set_failure_led(void)
{
    master_led_green_off();
    master_led_blue_off();
    master_led_red_on();
}

static void begin_transfer_led(void)
{
    s_transfer_led_active = true;
    s_transfer_led_poll_count = 0U;
    s_transfer_led_toggle_count = 0U;
    master_led_red_off();
    master_led_green_off();
    master_led_blue_on();
}

static void service_transfer_led(void)
{
    if (!s_transfer_led_active)
    {
        return;
    }

    s_transfer_led_poll_count++;
    if (s_transfer_led_poll_count < I3C_DMA_SEED_CHAIN_LED_TOGGLE_INTERVAL)
    {
        return;
    }

    s_transfer_led_poll_count = 0U;
    s_transfer_led_toggle_count++;
    master_led_blue_toggle();
}

static void end_transfer_led(void)
{
    s_transfer_led_active = false;
    ensure_visible_transfer_activity();
    s_transfer_led_poll_count = 0U;
    s_transfer_led_toggle_count = 0U;
    master_led_all_off();
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

void I3C0_IRQHandler(void)
{
    uint32_t pending = I3C_MasterGetPendingInterrupts(EXAMPLE_MASTER);
    uint32_t clearable = pending & (uint32_t)kI3C_MasterClearFlags;

    if (pending == 0U)
    {
        SDK_ISR_EXIT_BARRIER;
        return;
    }

    s_cm33_i3c_irq_count++;

    if ((pending & ((uint32_t)kI3C_MasterTxReadyFlag | (uint32_t)kI3C_MasterRxReadyFlag)) != 0U)
    {
        s_cm33_i3c_data_irq_count++;
    }

    if ((pending & ~((uint32_t)kI3C_MasterTxReadyFlag | (uint32_t)kI3C_MasterRxReadyFlag)) != 0U)
    {
        s_cm33_i3c_protocol_irq_count++;
    }

    s_i3c_irq_status_latched |= pending;

    if (clearable != 0U)
    {
        I3C_MasterClearStatusFlags(EXAMPLE_MASTER, clearable);
    }

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

static void capture_failure_snapshot(I3C_Type *base)
{
    s_failure_snapshot.valid = true;
    s_failure_snapshot.dmaIntStat = DMA0->INTSTAT;
    s_failure_snapshot.dmaInta = DMA0->COMMON[0].INTA;
    s_failure_snapshot.dmaActive = DMA0->COMMON[0].ACTIVE;
    s_failure_snapshot.dmaCfg = DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].CFG;
    s_failure_snapshot.dmaCtlStat = DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].CTLSTAT;
    s_failure_snapshot.dmaXferCfg = DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].XFERCFG;
    s_failure_snapshot.i3cMdmaCtrl = base->MDMACTRL;
    s_failure_snapshot.i3cMstatus = base->MSTATUS;
    s_failure_snapshot.i3cMdataCtrl = base->MDATACTRL;
}

static void fill_tx_buffer(void)
{
    uint32_t index;

    for (index = 0U; index < I3C_DMA_SEED_CHAIN_LENGTH; index++)
    {
        s_tx_buffer[index] = (uint8_t)(index + 1U);
    }
}

static status_t wait_for_smartdma_completion(void)
{
    volatile uint32_t timeout = 0U;

    while ((!s_smartdma_irq_fired) && (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT))
    {
        service_transfer_led();
        __NOP();
    }

    if (!s_smartdma_irq_fired)
    {
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

static status_t wait_for_i3c_ctrl_done(I3C_Type *base)
{
    volatile uint32_t timeout = 0U;

    while (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        service_transfer_led();
        uint32_t error_status = I3C_MasterGetErrorStatusFlags(base);
        uint32_t latched_status = s_i3c_irq_status_latched;

        if ((error_status != 0U) || ((latched_status & (uint32_t)kI3C_MasterErrorFlag) != 0U))
        {
            I3C_MasterClearErrorStatusFlags(base, error_status);
            s_i3c_irq_status_latched &= ~((uint32_t)kI3C_MasterErrorFlag);
            return kStatus_Fail;
        }

        if ((latched_status & (uint32_t)kI3C_MasterControlDoneFlag) != 0U)
        {
            s_i3c_irq_status_latched &= ~((uint32_t)kI3C_MasterControlDoneFlag);
            return kStatus_Success;
        }

        if ((I3C_MasterGetStatusFlags(base) & (uint32_t)kI3C_MasterControlDoneFlag) != 0U)
        {
            I3C_MasterClearStatusFlags(base, (uint32_t)kI3C_MasterControlDoneFlag);
            return kStatus_Success;
        }
    }

    return kStatus_Timeout;
}

static status_t wait_for_i3c_complete(I3C_Type *base)
{
    volatile uint32_t timeout = 0U;

    while (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        service_transfer_led();
        uint32_t error_status = I3C_MasterGetErrorStatusFlags(base);
        uint32_t latched_status = s_i3c_irq_status_latched;

        if ((error_status != 0U) || ((latched_status & (uint32_t)kI3C_MasterErrorFlag) != 0U))
        {
            I3C_MasterClearErrorStatusFlags(base, error_status);
            s_i3c_irq_status_latched &= ~((uint32_t)kI3C_MasterErrorFlag);
            return kStatus_Fail;
        }

        if ((latched_status & (uint32_t)kI3C_MasterCompleteFlag) != 0U)
        {
            s_i3c_irq_status_latched &= ~((uint32_t)kI3C_MasterCompleteFlag);
            return kStatus_Success;
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

    for (attempt = 0U; attempt < I3C_DMA_SEED_CHAIN_DAA_RETRY_ATTEMPTS; attempt++)
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

        for (volatile uint32_t delay = 0U; delay < I3C_DMA_SEED_CHAIN_DAA_RETRY_DELAY; delay++)
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

static void configure_dma_seed_chain(I3C_Type *base)
{
    const uint32_t channel_mask = (1UL << I3C_DMA_TX_CHANNEL);
    const uint32_t one_byte_xfercfg_base = DMA_CHANNEL_XFERCFG_CFGVALID(1U) | DMA_CHANNEL_XFERCFG_SETINTA(1U) |
                                           DMA_CHANNEL_XFERCFG_WIDTH(0U) | DMA_CHANNEL_XFERCFG_SRCINC(1U) |
                                           DMA_CHANNEL_XFERCFG_DSTINC(0U) | DMA_CHANNEL_XFERCFG_XFERCOUNT(0U);
    dma_descriptor_t *descriptor = NULL;
    dma_descriptor_t *next_descriptor = NULL;
    uint32_t descriptor_index;

    CLOCK_EnableClock(kCLOCK_Dmac0);
    RESET_PeripheralReset(kDMAC0_RST_SHIFT_RSTn);
    DMA0->CTRL = DMA_CTRL_ENABLE(1U);
    DMA0->SRAMBASE = (uint32_t)(uintptr_t)s_dma_descriptor_table;
    memset((void *)s_dma_descriptor_table, 0, sizeof(s_dma_descriptor_table));
    memset((void *)s_dma_seed_chain, 0, sizeof(s_dma_seed_chain));

    clear_dma0_channel_state();
    DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].CFG = DMA_CHANNEL_CFG_PERIPHREQEN(1U);
    DMA0->COMMON[0].INTENSET = channel_mask;
    DMA0->COMMON[0].ENABLESET = channel_mask;

    for (descriptor_index = 0U; descriptor_index < I3C_DMA_SEED_CHAIN_LENGTH; descriptor_index++)
    {
        bool has_next = (descriptor_index + 1U) < I3C_DMA_SEED_CHAIN_LENGTH;
        uint32_t xfercfg = one_byte_xfercfg_base | (has_next ? DMA_CHANNEL_XFERCFG_RELOAD(1U) : 0U);

        descriptor = (descriptor_index == 0U) ? &s_dma_descriptor_table[I3C_DMA_TX_CHANNEL] :
                                                &s_dma_seed_chain[descriptor_index - 1U];

        if (has_next)
        {
            next_descriptor = (descriptor_index == 0U) ? &s_dma_seed_chain[0] : &s_dma_seed_chain[descriptor_index];
        }
        else
        {
            next_descriptor = NULL;
        }

        descriptor->xfercfg = xfercfg;
        descriptor->srcEndAddr = &s_tx_buffer[descriptor_index];
        descriptor->dstEndAddr = (void *)((descriptor_index + 1U < I3C_DMA_SEED_CHAIN_LENGTH) ?
                                              (uintptr_t)&base->MWDATAB :
                                              (uintptr_t)&base->MWDATABE);
        descriptor->linkToNextDesc = next_descriptor;
    }

    memset((void *)&s_probe_param, 0, sizeof(s_probe_param));
    s_probe_param.expectedWakeCount = I3C_DMA_SEED_CHAIN_LENGTH;
    s_probe_param.dmaIntaAddress = (uint32_t)(uintptr_t)&DMA0->COMMON[0].INTA;
    s_probe_param.dmaChannelMask = channel_mask;
}

static void arm_seed_chain(void)
{
    DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].XFERCFG = s_dma_descriptor_table[I3C_DMA_TX_CHANNEL].xfercfg;
    DMA0->COMMON[0].ENABLESET = s_probe_param.dmaChannelMask;
    DMA0->COMMON[0].SETVALID = s_probe_param.dmaChannelMask;
}

static void dump_probe_counters(void)
{
    EXP_LOG_INFO("dma_seed_bytes=%lu", (unsigned long)s_probe_param.dmaSeedBytes);
    EXP_LOG_INFO("dma0_inta_count=%lu", (unsigned long)s_probe_param.dmaIntaCount);
    EXP_LOG_INFO("smartdma_wake_count=%lu", (unsigned long)s_probe_param.wakeCount);
    EXP_LOG_INFO("cm33_i3c_irq_count=%lu", (unsigned long)s_cm33_i3c_irq_count);
    EXP_LOG_INFO("cm33_i3c_data_irq_count=%lu", (unsigned long)s_cm33_i3c_data_irq_count);
    EXP_LOG_INFO("cm33_i3c_protocol_irq_count=%lu", (unsigned long)s_cm33_i3c_protocol_irq_count);

    PRINTF("dma_seed_bytes=%lu\r\n", (unsigned long)s_probe_param.dmaSeedBytes);
    PRINTF("dma0_inta_count=%lu\r\n", (unsigned long)s_probe_param.dmaIntaCount);
    PRINTF("smartdma_wake_count=%lu\r\n", (unsigned long)s_probe_param.wakeCount);
    PRINTF("cm33_i3c_irq_count=%lu\r\n", (unsigned long)s_cm33_i3c_irq_count);
    PRINTF("cm33_i3c_data_irq_count=%lu\r\n", (unsigned long)s_cm33_i3c_data_irq_count);
    PRINTF("cm33_i3c_protocol_irq_count=%lu\r\n", (unsigned long)s_cm33_i3c_protocol_irq_count);
}

static void dump_debug_state(I3C_Type *base)
{
    uint32_t dma_intstat = DMA0->INTSTAT;
    uint32_t dma_inta = DMA0->COMMON[0].INTA;
    uint32_t dma_active = DMA0->COMMON[0].ACTIVE;
    uint32_t dma_cfg = DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].CFG;
    uint32_t dma_ctlstat = DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].CTLSTAT;
    uint32_t dma_xfercfg = DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].XFERCFG;
    uint32_t i3c_mdmactrl = base->MDMACTRL;
    uint32_t i3c_mstatus = base->MSTATUS;
    uint32_t i3c_mdatactrl = base->MDATACTRL;

    if (s_failure_snapshot.valid)
    {
        dma_intstat = s_failure_snapshot.dmaIntStat;
        dma_inta = s_failure_snapshot.dmaInta;
        dma_active = s_failure_snapshot.dmaActive;
        dma_cfg = s_failure_snapshot.dmaCfg;
        dma_ctlstat = s_failure_snapshot.dmaCtlStat;
        dma_xfercfg = s_failure_snapshot.dmaXferCfg;
        i3c_mdmactrl = s_failure_snapshot.i3cMdmaCtrl;
        i3c_mstatus = s_failure_snapshot.i3cMstatus;
        i3c_mdatactrl = s_failure_snapshot.i3cMdataCtrl;
    }

    EXP_LOG_INFO("DMA0 INTSTAT=0x%08lx", (unsigned long)dma_intstat);
    EXP_LOG_INFO("DMA0 INTA=0x%08lx", (unsigned long)dma_inta);
    EXP_LOG_INFO("DMA0 ACTIVE=0x%08lx", (unsigned long)dma_active);
    EXP_LOG_INFO("DMA0 CH25 CFG=0x%08lx", (unsigned long)dma_cfg);
    EXP_LOG_INFO("DMA0 CH25 CTLSTAT=0x%08lx", (unsigned long)dma_ctlstat);
    EXP_LOG_INFO("DMA0 CH25 XFERCFG=0x%08lx", (unsigned long)dma_xfercfg);
    EXP_LOG_INFO("I3C MDMACTRL=0x%08lx", (unsigned long)i3c_mdmactrl);
    EXP_LOG_INFO("I3C MSTATUS=0x%08lx", (unsigned long)i3c_mstatus);
    EXP_LOG_INFO("I3C MDATACTRL=0x%08lx", (unsigned long)i3c_mdatactrl);
    EXP_LOG_INFO("I3C latched IRQ mask=0x%08lx", (unsigned long)s_i3c_irq_status_latched);
    EXP_LOG_INFO("SmartDMA mailbox=%lu", (unsigned long)s_probe_param.mailbox);

    dump_probe_counters();
}

static status_t run_i3c_dma_seed_chain_probe(I3C_Type *base, uint8_t slaveAddr)
{
    status_t result = kStatus_Success;
    uint8_t saved_tx_trigger_level = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_TXTRIG_MASK) >> I3C_MDATACTRL_TXTRIG_SHIFT);
    uint8_t saved_rx_trigger_level = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_RXTRIG_MASK) >> I3C_MDATACTRL_RXTRIG_SHIFT);

    s_smartdma_irq_fired = false;
    s_i3c_irq_status_latched = 0U;
    s_cm33_i3c_irq_count = 0U;
    s_cm33_i3c_data_irq_count = 0U;
    s_cm33_i3c_protocol_irq_count = 0U;
    memset(&s_failure_snapshot, 0, sizeof(s_failure_snapshot));

    configure_dma_seed_chain(base);

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base,
                               (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                   (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                   (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;
    I3C_MasterSetWatermarks(base,
                            kI3C_TxTriggerOnEmpty,
                            (i3c_rx_trigger_level_t)saved_rx_trigger_level,
                            false,
                            false);

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, SMART_DMA_TRIGGER_CHANNEL, kINPUTMUX_Dma0IrqToSmartDmaInput);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_I3c0TxToDmac0Ch25RequestEna, true);
    INPUTMUX_Deinit(INPUTMUX);

    NVIC_DisableIRQ(DMA0_IRQn);
    NVIC_ClearPendingIRQ(DMA0_IRQn);
    NVIC_ClearPendingIRQ(I3C0_IRQn);
    NVIC_SetPriority(I3C0_IRQn, 3);
    NVIC_EnableIRQ(I3C0_IRQn);

    SMARTDMA_Init(
        SMARTDMA_SRAM_ADDR, __smartdma_start__, (uint32_t)((uintptr_t)__smartdma_end__ - (uintptr_t)__smartdma_start__));
    SMARTDMA_InstallCallback(smartdma_completion_callback, NULL);
    NVIC_ClearPendingIRQ(SDMA_IRQn);
    NVIC_SetPriority(SDMA_IRQn, 3);
    NVIC_EnableIRQ(SDMA_IRQn);
    SMARTDMA_Reset();
    SMARTDMA_Boot(I3C_DMA_SEED_CHAIN_API_INDEX, &s_probe_param, 0);

    I3C_MasterEnableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    I3C_MasterEnableDMA(base, true, false, 1U);
    arm_seed_chain();
    begin_transfer_led();

    result = I3C_MasterStart(base, kI3C_TypeI3CSdr, slaveAddr, kI3C_Write);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = wait_for_i3c_ctrl_done(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = wait_for_smartdma_completion();
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = wait_for_i3c_complete(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = I3C_MasterStop(base);

exit:
    end_transfer_led();

    if (result != kStatus_Success)
    {
        capture_failure_snapshot(base);
    }

    I3C_MasterEnableDMA(base, false, false, 1U);
    I3C_MasterDisableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    I3C_MasterSetWatermarks(base,
                            (i3c_tx_trigger_level_t)saved_tx_trigger_level,
                            (i3c_rx_trigger_level_t)saved_rx_trigger_level,
                            false,
                            false);
    clear_dma0_channel_state();
    SMARTDMA_Reset();

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
    init_transfer_led();
    run_boot_led_self_test();

    for (volatile uint32_t startup_delay = 0U; startup_delay < I3C_DMA_SEED_CHAIN_STARTUP_WAIT; startup_delay++)
    {
        __NOP();
    }

    PRINTF("\r\nI3C DMA seed chain probe -- master.\r\n");
    EXP_LOG_INFO("I3C DMA seed chain probe -- master.");

    POWER_DisablePD(kPDRUNCFG_APD_SMARTDMA_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_SMARTDMA_SRAM);
    POWER_ApplyPD();

    fill_tx_buffer();
    keep_smartdma_api_alive();

    I3C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz.i2cBaud = I3C_DMA_SEED_CHAIN_I2C_BAUDRATE;
    masterConfig.baudRate_Hz.i3cPushPullBaud = I3C_DMA_SEED_CHAIN_I3C_PP_BAUDRATE;
    masterConfig.baudRate_Hz.i3cOpenDrainBaud = I3C_DMA_SEED_CHAIN_I3C_OD_BAUDRATE;
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
    EXP_LOG_INFO("Starting I3C DMA seed chain probe.");

    result = run_i3c_dma_seed_chain_probe(EXAMPLE_MASTER, slaveAddr);
    if (result != kStatus_Success)
    {
        EXP_LOG_ERROR("I3C DMA seed chain probe failed: %d", result);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_probe_param.mailbox != 1U)
    {
        EXP_LOG_ERROR("SmartDMA mailbox mismatch: %lu", (unsigned long)s_probe_param.mailbox);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_probe_param.wakeCount != s_probe_param.expectedWakeCount)
    {
        EXP_LOG_ERROR("Wake count mismatch: expected=%lu actual=%lu",
                      (unsigned long)s_probe_param.expectedWakeCount,
                      (unsigned long)s_probe_param.wakeCount);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_probe_param.dmaSeedBytes != s_probe_param.expectedWakeCount)
    {
        EXP_LOG_ERROR("DMA seed byte mismatch: expected=%lu actual=%lu",
                      (unsigned long)s_probe_param.expectedWakeCount,
                      (unsigned long)s_probe_param.dmaSeedBytes);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_probe_param.dmaIntaCount != s_probe_param.expectedWakeCount)
    {
        EXP_LOG_ERROR("DMA INTA count mismatch: expected=%lu actual=%lu",
                      (unsigned long)s_probe_param.expectedWakeCount,
                      (unsigned long)s_probe_param.dmaIntaCount);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_probe_param.wakeCount <= 1U)
    {
        EXP_LOG_ERROR("Expected more than one SmartDMA wake.");
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_cm33_i3c_data_irq_count != 0U)
    {
        EXP_LOG_ERROR("Unexpected CM33 I3C data-ready IRQ count: %lu", (unsigned long)s_cm33_i3c_data_irq_count);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_cm33_i3c_protocol_irq_count == 0U)
    {
        EXP_LOG_ERROR("Expected protocol IRQs to remain visible on CM33.");
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    dump_probe_counters();
    EXP_LOG_INFO("I3C DMA seed chain probe successful.");
    PRINTF("I3C DMA seed chain probe successful.\r\n");
    set_success_led();
    return 0;
}
