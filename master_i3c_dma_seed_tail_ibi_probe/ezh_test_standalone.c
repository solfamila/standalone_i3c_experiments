/*
 * Standalone proof that one real I3C TX DMA seed byte can wake SmartDMA for a
 * one-FIFO-window tail fill while CM33 keeps protocol/IBI/error IRQ ownership.
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
#include "fsl_i3c_smartdma.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"
#include "fsl_reset.h"
#include "fsl_smartdma.h"
#include "app.h"
#include "board.h"
#include "experiment_led.h"

#define EXPERIMENT_ENABLE_SEMIHOST_LOG 0

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
#define I3C_DMA_SEED_CHAIN_LED_BOOT_PULSE_US 100000U
#define I3C_DMA_SEED_CHAIN_LED_SUCCESS_PULSE_US 120000U
#define I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_US 200000U
#define I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_COUNT 2U
#define SMART_DMA_TRIGGER_CHANNEL 0U
#define I3C_DMA_SEED_TAIL_IBI_PRE_READ_DELAY_US 50000U
#define EXPERIMENT_POST_IBI_READ_MODE_MANUAL 0U
#define EXPERIMENT_POST_IBI_READ_MODE_BLOCKING 1U
#define EXPERIMENT_POST_IBI_READ_MODE_SMARTDMA 2U

/* Keep the validated manual path as the default. The blocking variant is
 * available for exploratory runs by switching this mode.
 */
#ifndef EXPERIMENT_POST_IBI_READ_MODE
#define EXPERIMENT_POST_IBI_READ_MODE EXPERIMENT_POST_IBI_READ_MODE_MANUAL
#endif

#if (EXPERIMENT_POST_IBI_READ_MODE != EXPERIMENT_POST_IBI_READ_MODE_MANUAL) && \
    (EXPERIMENT_POST_IBI_READ_MODE != EXPERIMENT_POST_IBI_READ_MODE_BLOCKING) && \
    (EXPERIMENT_POST_IBI_READ_MODE != EXPERIMENT_POST_IBI_READ_MODE_SMARTDMA)
#error "Unsupported EXPERIMENT_POST_IBI_READ_MODE"
#endif

#define I3C_DMA_SEED_TAIL_IBI_PAYLOAD_BYTE 0xA5U
#define I3C_DMA_SEED_TAIL_IBI_MAX_PAYLOAD 8U
#define I3C_BROADCAST_ADDR 0x7EU
#define I3C_CCC_RSTDAA 0x06U
#define I3C_CCC_SETDASA 0x87U
#define I3C_TARGET_STATIC_ADDR 0x1EU
#define I3C_TARGET_DYNAMIC_ADDR 0x30U

#define I3C_PROTOCOL_IRQ_MASK ((uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag | \
                               (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag | \
                               (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag)

extern uint8_t __smartdma_start__[];
extern uint8_t __smartdma_end__[];

void keep_smartdma_api_alive(void);
void I3C0_DriverIRQHandler(void);

static void roundtrip_read_ibi_callback(I3C_Type *base,
                                        i3c_master_smartdma_handle_t *handle,
                                        i3c_ibi_type_t ibiType,
                                        i3c_ibi_state_t ibiState);
static void roundtrip_read_complete_callback(I3C_Type *base,
                                             i3c_master_smartdma_handle_t *handle,
                                             status_t status,
                                             void *userData);

#if EXPERIMENT_ENABLE_SEMIHOST_LOG
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
        semihost_write0("ERR i3c-dma-seed-tail-ibi-probe log prefix overflow\n");
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
#else
#define EXP_LOG_INFO(...) ((void)0)
#define EXP_LOG_ERROR(...) ((void)0)
#endif

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
    volatile uint32_t smartdmaBytes;
    volatile uint32_t dmaIntaCount;
    uint32_t nextTxByteAddress;
    uint32_t remainingCount;
    uint32_t i3cBaseAddress;
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

typedef struct _roundtrip_read_snapshot
{
    uint32_t magic;
    int32_t result;
    uint32_t completionStatus;
    uint32_t stage;
    uint32_t remaining;
    uint32_t status;
    uint32_t errStatus;
    uint32_t mdatactrl;
    uint32_t handleState;
    uint32_t transferCount;
    uint32_t smartdmaWindowIrqCount;
    uint32_t smartdmaFifoReadyBounceCount;
    uint32_t smartdmaProtocolBounceCount;
    uint32_t smartdmaMailboxProtocolCount;
    uint32_t smartdmaWindowPendingMask;
    uint32_t smartdmaWindowFifoMask;
    uint32_t smartdmaWindowProtocolMask;
    uint32_t smartdmaMailbox;
    uint32_t smartdmaBounceStatus;
    uint32_t smartdmaBounceErrStatus;
    uint32_t smartdmaBounceDataCtrl;
    uint32_t smartdmaMailboxMaskedStatus;
    uint32_t smartdmaMailboxStatus;
    uint32_t smartdmaMailboxErrStatus;
    uint32_t smartdmaMailboxDataCtrl;
    uint8_t data[I3C_DMA_SEED_CHAIN_LENGTH];
} roundtrip_read_snapshot_t;

typedef struct _post_ibi_handoff_sample
{
    uint32_t stage;
    int32_t result;
    uint32_t mctrl;
    uint32_t status;
    uint32_t errStatus;
    uint32_t mdatactrl;
    uint32_t pending;
    uint32_t masterState;
    uint32_t latchedPending;
    uint32_t ibiSeen;
    uint32_t ibiActive;
    uint32_t ibiAckEmitted;
} post_ibi_handoff_sample_t;

typedef struct _post_ibi_handoff_snapshot
{
    uint32_t magic;
    post_ibi_handoff_sample_t beforeFinalize;
    post_ibi_handoff_sample_t afterFinalize;
    post_ibi_handoff_sample_t beforeReadStart;
} post_ibi_handoff_snapshot_t;

typedef struct _post_ibi_read_trace_window
{
    uint32_t beginCount;
    uint32_t endCount;
    uint32_t mode;
    uint32_t stage;
    int32_t result;
    uint32_t remaining;
} post_ibi_read_trace_window_t;

typedef struct _post_ibi_drain_probe
{
    uint32_t beginCount;
    uint32_t endCount;
    uint32_t rxCount;
    uint32_t errStatus;
    uint32_t remaining;
} post_ibi_drain_probe_t;

typedef struct _daa_setup_snapshot
{
    uint32_t magic;
    uint32_t stage;
    uint32_t attempt;
    int32_t rstdaaResult;
    int32_t daaResult;
    uint32_t devCount;
    uint32_t matchedDynamicAddr;
    uint32_t status;
    uint32_t errStatus;
    uint32_t mdatactrl;
} daa_setup_snapshot_t;

#define ROUNDTRIP_READ_SNAPSHOT_MAGIC 0x52524453U
#define POST_IBI_HANDOFF_SNAPSHOT_MAGIC 0x50494248U
#define DAA_SETUP_SNAPSHOT_MAGIC 0x44414153U
#define ROUNDTRIP_STAGE_CLEARED 1U
#define ROUNDTRIP_STAGE_START_ISSUED 2U
#define ROUNDTRIP_STAGE_CTRL_DONE 3U
#define ROUNDTRIP_STAGE_DRAIN_LOOP 4U
#define ROUNDTRIP_STAGE_STOP_SENT 5U
#define ROUNDTRIP_STAGE_SMARTDMA_PREPARED 6U
#define ROUNDTRIP_STAGE_SMARTDMA_STARTED 7U
#define ROUNDTRIP_STAGE_SMARTDMA_COMPLETED 8U
#define ROUNDTRIP_STAGE_BLOCKING_COMPLETED 9U
#define ROUNDTRIP_STAGE_ERROR_START 0x101U
#define ROUNDTRIP_STAGE_ERROR_CTRL_DONE 0x102U
#define ROUNDTRIP_STAGE_ERROR_LOOP 0x103U
#define ROUNDTRIP_STAGE_ERROR_SMARTDMA_START 0x104U
#define ROUNDTRIP_STAGE_ERROR_SMARTDMA_WAIT 0x105U
#define ROUNDTRIP_STAGE_ERROR_BLOCKING 0x106U
#define POST_IBI_HANDOFF_STAGE_BEFORE_FINALIZE 1U
#define POST_IBI_HANDOFF_STAGE_AFTER_FINALIZE 2U
#define POST_IBI_HANDOFF_STAGE_BEFORE_READ_START 3U
#define DAA_SETUP_STAGE_CLEARED 1U
#define DAA_SETUP_STAGE_RSTDAA_ATTEMPT 2U
#define DAA_SETUP_STAGE_RSTDAA_OK 3U
#define DAA_SETUP_STAGE_DAA_OK 4U
#define DAA_SETUP_STAGE_TARGET_FOUND 5U
#define DAA_SETUP_STAGE_SETDASA_ATTEMPT 6U
#define DAA_SETUP_STAGE_SETDASA_CCC_OK 7U
#define DAA_SETUP_STAGE_SETDASA_DIRECT_OK 8U
#define DAA_SETUP_STAGE_ERROR_RSTDAA 0x101U
#define DAA_SETUP_STAGE_ERROR_DAA 0x102U
#define DAA_SETUP_STAGE_ERROR_NO_TARGET 0x103U
#define DAA_SETUP_STAGE_ERROR_SETDASA_CCC 0x104U
#define DAA_SETUP_STAGE_ERROR_SETDASA_DIRECT 0x105U

AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_DMA_MAX_CHANNELS],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_seed_chain[I3C_DMA_SEED_CHAIN_LENGTH - 1U],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_tx_buffer[I3C_DMA_SEED_CHAIN_LENGTH], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_rx_buffer[I3C_DMA_SEED_CHAIN_LENGTH], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static i3c_dma_seed_chain_param_t s_probe_param, 4);

static i3c_master_smartdma_handle_t s_roundtrip_read_handle;
static const i3c_master_smartdma_callback_t s_roundtrip_read_callbacks = {
    .slave2Master = NULL,
    .ibiCallback = roundtrip_read_ibi_callback,
    .transferComplete = roundtrip_read_complete_callback,
};
static volatile uint32_t s_i3c_irq_status_latched = 0U;
static volatile uint32_t s_cm33_i3c_irq_count = 0U;
static volatile uint32_t s_cm33_i3c_data_irq_count = 0U;
static volatile uint32_t s_cm33_i3c_protocol_irq_count = 0U;
static volatile uint32_t s_cm33_i3c_ibi_irq_count = 0U;
static volatile bool s_ibi_seen = false;
static volatile bool s_ibi_active = false;
static volatile bool s_ibi_ack_emitted = false;
static volatile uint32_t s_ibi_type = 0U;
static volatile uint32_t s_ibi_address = 0U;
static volatile uint32_t s_ibi_payload_count = 0U;
static volatile uint8_t s_ibi_payload[I3C_DMA_SEED_TAIL_IBI_MAX_PAYLOAD];
static volatile bool s_roundtrip_read_active = false;
static volatile bool s_roundtrip_read_complete = false;
static volatile bool s_roundtrip_read_ibi_won = false;
static volatile status_t s_roundtrip_read_status = kStatus_Success;
static uint8_t s_roundtrip_read_ibi_buffer[I3C_DMA_SEED_TAIL_IBI_MAX_PAYLOAD];
static seed_chain_failure_snapshot_t s_failure_snapshot;
static __NO_INIT roundtrip_read_snapshot_t s_roundtrip_read_snapshot;
static __NO_INIT post_ibi_handoff_snapshot_t s_post_ibi_handoff_snapshot;
static __NO_INIT volatile post_ibi_read_trace_window_t s_post_ibi_read_trace_window;
static __NO_INIT volatile post_ibi_drain_probe_t s_post_ibi_drain_probe;
static __NO_INIT daa_setup_snapshot_t s_daa_setup_snapshot;
static bool s_transfer_led_active = false;
static uint32_t s_transfer_led_poll_count = 0U;
static uint32_t s_transfer_led_toggle_count = 0U;

static void run_boot_led_self_test(void)
{
    EXP_LED_Blink(false, false, true, I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_COUNT,
                  I3C_DMA_SEED_CHAIN_LED_BOOT_PULSE_US);
    EXP_LED_Set(false, false, true);
}

static void ensure_visible_transfer_activity(void)
{
    if (s_transfer_led_toggle_count != 0U)
    {
        return;
    }

    EXP_LED_Blink(false, false, true, I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_COUNT,
                  I3C_DMA_SEED_CHAIN_LED_VISIBLE_PULSE_US);
}

static void init_transfer_led(void)
{
    EXP_LED_Init();
}

static void set_success_led(void)
{
    EXP_LED_Blink(false, true, false, 3U, I3C_DMA_SEED_CHAIN_LED_SUCCESS_PULSE_US);
    EXP_LED_Set(false, true, false);
}

static void set_failure_led(void)
{
    EXP_LED_Set(true, false, false);
}

static void begin_transfer_led(void)
{
    s_transfer_led_active = true;
    s_transfer_led_poll_count = 0U;
    s_transfer_led_toggle_count = 0U;
    EXP_LED_Set(false, false, true);
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
    EXP_LED_ToggleBlue();
}

static void end_transfer_led(bool show_completion_pulse)
{
    s_transfer_led_active = false;
    if (show_completion_pulse)
    {
        ensure_visible_transfer_activity();
    }
    s_transfer_led_poll_count = 0U;
    s_transfer_led_toggle_count = 0U;
    EXP_LED_Set(false, false, false);
}

void SDMA_IRQHandler(void)
{
    SMARTDMA_HandleIRQ();
    SDK_ISR_EXIT_BARRIER;
}

static void clear_ibi_state(void)
{
    s_ibi_seen = false;
    s_ibi_active = false;
    s_ibi_ack_emitted = false;
    s_ibi_type = 0U;
    s_ibi_address = 0U;
    s_ibi_payload_count = 0U;
    memset((void *)s_ibi_payload, 0, sizeof(s_ibi_payload));
}

static void clear_roundtrip_read_snapshot(void)
{
    s_roundtrip_read_snapshot.magic = 0U;
    s_roundtrip_read_snapshot.result = 0;
    s_roundtrip_read_snapshot.completionStatus = 0U;
    s_roundtrip_read_snapshot.stage = 0U;
    s_roundtrip_read_snapshot.remaining = 0U;
    s_roundtrip_read_snapshot.status = 0U;
    s_roundtrip_read_snapshot.errStatus = 0U;
    s_roundtrip_read_snapshot.mdatactrl = 0U;
    s_roundtrip_read_snapshot.handleState = 0U;
    s_roundtrip_read_snapshot.transferCount = 0U;
    s_roundtrip_read_snapshot.smartdmaWindowIrqCount = 0U;
    s_roundtrip_read_snapshot.smartdmaFifoReadyBounceCount = 0U;
    s_roundtrip_read_snapshot.smartdmaProtocolBounceCount = 0U;
    s_roundtrip_read_snapshot.smartdmaMailboxProtocolCount = 0U;
    s_roundtrip_read_snapshot.smartdmaWindowPendingMask = 0U;
    s_roundtrip_read_snapshot.smartdmaWindowFifoMask = 0U;
    s_roundtrip_read_snapshot.smartdmaWindowProtocolMask = 0U;
    s_roundtrip_read_snapshot.smartdmaMailbox = 0U;
    s_roundtrip_read_snapshot.smartdmaBounceStatus = 0U;
    s_roundtrip_read_snapshot.smartdmaBounceErrStatus = 0U;
    s_roundtrip_read_snapshot.smartdmaBounceDataCtrl = 0U;
    s_roundtrip_read_snapshot.smartdmaMailboxMaskedStatus = 0U;
    s_roundtrip_read_snapshot.smartdmaMailboxStatus = 0U;
    s_roundtrip_read_snapshot.smartdmaMailboxErrStatus = 0U;
    s_roundtrip_read_snapshot.smartdmaMailboxDataCtrl = 0U;
    memset(s_roundtrip_read_snapshot.data, 0, sizeof(s_roundtrip_read_snapshot.data));
}

static void clear_post_ibi_handoff_snapshot(void)
{
    memset(&s_post_ibi_handoff_snapshot, 0, sizeof(s_post_ibi_handoff_snapshot));
}

static void clear_daa_setup_snapshot(void)
{
    memset(&s_daa_setup_snapshot, 0, sizeof(s_daa_setup_snapshot));
}

static void capture_daa_setup_snapshot(I3C_Type *base,
                                       uint32_t stage,
                                       uint32_t attempt,
                                       status_t rstdaaResult,
                                       status_t daaResult,
                                       uint32_t devCount,
                                       uint32_t matchedDynamicAddr)
{
    s_daa_setup_snapshot.magic = DAA_SETUP_SNAPSHOT_MAGIC;
    s_daa_setup_snapshot.stage = stage;
    s_daa_setup_snapshot.attempt = attempt;
    s_daa_setup_snapshot.rstdaaResult = (int32_t)rstdaaResult;
    s_daa_setup_snapshot.daaResult = (int32_t)daaResult;
    s_daa_setup_snapshot.devCount = devCount;
    s_daa_setup_snapshot.matchedDynamicAddr = matchedDynamicAddr;
    s_daa_setup_snapshot.status = I3C_MasterGetStatusFlags(base);
    s_daa_setup_snapshot.errStatus = I3C_MasterGetErrorStatusFlags(base);
    s_daa_setup_snapshot.mdatactrl = base->MDATACTRL;
}

__attribute__((noinline, used)) void trace_post_ibi_read_window_begin(uint32_t mode)
{
    s_post_ibi_read_trace_window.beginCount++;
    s_post_ibi_read_trace_window.mode = mode;
    s_post_ibi_read_trace_window.stage = 0U;
    s_post_ibi_read_trace_window.result = 0;
    s_post_ibi_read_trace_window.remaining = 0U;
    __asm volatile("" ::: "memory");
}

__attribute__((noinline, used)) void trace_post_ibi_read_window_end(uint32_t stage,
                                                                    status_t result,
                                                                    uint32_t remaining)
{
    s_post_ibi_read_trace_window.endCount++;
    s_post_ibi_read_trace_window.stage = stage;
    s_post_ibi_read_trace_window.result = (int32_t)result;
    s_post_ibi_read_trace_window.remaining = remaining;
    __asm volatile("" ::: "memory");
}

__attribute__((noinline, used)) void trace_post_ibi_drain_probe_begin(uint32_t rxCount, uint32_t remaining)
{
    s_post_ibi_drain_probe.beginCount++;
    s_post_ibi_drain_probe.rxCount = rxCount;
    s_post_ibi_drain_probe.errStatus = 0U;
    s_post_ibi_drain_probe.remaining = remaining;
    __asm volatile("" ::: "memory");
}

__attribute__((noinline, used)) void trace_post_ibi_drain_probe_end(uint32_t rxCount,
                                                                     uint32_t errStatus,
                                                                     uint32_t remaining)
{
    s_post_ibi_drain_probe.endCount++;
    s_post_ibi_drain_probe.rxCount = rxCount;
    s_post_ibi_drain_probe.errStatus = errStatus;
    s_post_ibi_drain_probe.remaining = remaining;
    __asm volatile("" ::: "memory");
}

static void capture_post_ibi_handoff_sample(post_ibi_handoff_sample_t *sample,
                                            I3C_Type *base,
                                            uint32_t stage,
                                            status_t result)
{
    sample->stage = stage;
    sample->result = (int32_t)result;
    sample->mctrl = base->MCTRL;
    sample->status = I3C_MasterGetStatusFlags(base);
    sample->errStatus = I3C_MasterGetErrorStatusFlags(base);
    sample->mdatactrl = base->MDATACTRL;
    sample->pending = I3C_MasterGetPendingInterrupts(base);
    sample->masterState = (uint32_t)I3C_MasterGetState(base);
    sample->latchedPending = s_i3c_irq_status_latched;
    sample->ibiSeen = s_ibi_seen ? 1U : 0U;
    sample->ibiActive = s_ibi_active ? 1U : 0U;
    sample->ibiAckEmitted = s_ibi_ack_emitted ? 1U : 0U;
}

static void capture_post_ibi_handoff_snapshot(I3C_Type *base,
                                              uint32_t stage,
                                              status_t result)
{
    s_post_ibi_handoff_snapshot.magic = POST_IBI_HANDOFF_SNAPSHOT_MAGIC;

    if (stage == POST_IBI_HANDOFF_STAGE_BEFORE_FINALIZE)
    {
        capture_post_ibi_handoff_sample(&s_post_ibi_handoff_snapshot.beforeFinalize, base, stage, result);
    }
    else if (stage == POST_IBI_HANDOFF_STAGE_AFTER_FINALIZE)
    {
        capture_post_ibi_handoff_sample(&s_post_ibi_handoff_snapshot.afterFinalize, base, stage, result);
    }
    else if (stage == POST_IBI_HANDOFF_STAGE_BEFORE_READ_START)
    {
        capture_post_ibi_handoff_sample(&s_post_ibi_handoff_snapshot.beforeReadStart, base, stage, result);
    }
}

static void capture_roundtrip_read_snapshot(I3C_Type *base, status_t result)
{
    s_roundtrip_read_snapshot.magic = ROUNDTRIP_READ_SNAPSHOT_MAGIC;
    s_roundtrip_read_snapshot.result = (int32_t)result;
    s_roundtrip_read_snapshot.completionStatus = (uint32_t)s_roundtrip_read_status;
    s_roundtrip_read_snapshot.status = I3C_MasterGetStatusFlags(base);
    s_roundtrip_read_snapshot.errStatus = I3C_MasterGetErrorStatusFlags(base);
    s_roundtrip_read_snapshot.mdatactrl = base->MDATACTRL;
    s_roundtrip_read_snapshot.handleState = (uint32_t)s_roundtrip_read_handle.state;
    s_roundtrip_read_snapshot.transferCount = s_roundtrip_read_handle.transferCount;
    s_roundtrip_read_snapshot.smartdmaWindowIrqCount = s_roundtrip_read_handle.smartdmaWindowIrqCount;
    s_roundtrip_read_snapshot.smartdmaFifoReadyBounceCount = s_roundtrip_read_handle.smartdmaFifoReadyBounceCount;
    s_roundtrip_read_snapshot.smartdmaProtocolBounceCount = s_roundtrip_read_handle.smartdmaProtocolBounceCount;
    s_roundtrip_read_snapshot.smartdmaMailboxProtocolCount = s_roundtrip_read_handle.smartdmaMailboxProtocolCount;
    s_roundtrip_read_snapshot.smartdmaWindowPendingMask = s_roundtrip_read_handle.smartdmaWindowPendingMask;
    s_roundtrip_read_snapshot.smartdmaWindowFifoMask = s_roundtrip_read_handle.smartdmaWindowFifoMask;
    s_roundtrip_read_snapshot.smartdmaWindowProtocolMask = s_roundtrip_read_handle.smartdmaWindowProtocolMask;
    s_roundtrip_read_snapshot.smartdmaMailbox = s_roundtrip_read_handle.smartdmaMailbox;
    s_roundtrip_read_snapshot.smartdmaBounceStatus = s_roundtrip_read_handle.smartdmaBounceStatus;
    s_roundtrip_read_snapshot.smartdmaBounceErrStatus = s_roundtrip_read_handle.smartdmaBounceErrStatus;
    s_roundtrip_read_snapshot.smartdmaBounceDataCtrl = s_roundtrip_read_handle.smartdmaBounceDataCtrl;
    s_roundtrip_read_snapshot.smartdmaMailboxMaskedStatus = s_roundtrip_read_handle.smartdmaMailboxMaskedStatus;
    s_roundtrip_read_snapshot.smartdmaMailboxStatus = s_roundtrip_read_handle.smartdmaMailboxStatus;
    s_roundtrip_read_snapshot.smartdmaMailboxErrStatus = s_roundtrip_read_handle.smartdmaMailboxErrStatus;
    s_roundtrip_read_snapshot.smartdmaMailboxDataCtrl = s_roundtrip_read_handle.smartdmaMailboxDataCtrl;
    memcpy(s_roundtrip_read_snapshot.data, s_rx_buffer, sizeof(s_roundtrip_read_snapshot.data));
}

static void service_ibi_protocol_irq(I3C_Type *base, uint32_t pending)
{
    i3c_master_state_t masterState = I3C_MasterGetState(base);
    uint32_t rxCount;

    if ((pending & ((uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterArbitrationWonFlag)) != 0U)
    {
        s_ibi_active = true;
    }

    if ((pending & (uint32_t)kI3C_MasterSlaveStartFlag) != 0U)
    {
        I3C_MasterEmitRequest(base, kI3C_RequestAutoIbi);
        masterState = I3C_MasterGetState(base);
    }

    if (masterState == kI3C_MasterStateIbiAck)
    {
        s_ibi_type = (uint32_t)I3C_GetIBIType(base);
        I3C_MasterEmitIBIResponse(base, kI3C_IbiRespAckMandatory);
        s_ibi_ack_emitted = true;
        masterState = I3C_MasterGetState(base);
    }

    rxCount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT;
    while ((rxCount != 0U) && (s_ibi_payload_count < ARRAY_SIZE(s_ibi_payload)))
    {
        s_ibi_payload[s_ibi_payload_count++] = (uint8_t)(base->MRDATAB & 0xFFU);
        rxCount--;
    }

    if (((pending & (uint32_t)kI3C_MasterCompleteFlag) != 0U) &&
        (s_ibi_active || s_ibi_ack_emitted || (masterState == kI3C_MasterStateIbiAck) ||
         (masterState == kI3C_MasterStateIbiRcv)))
    {
        s_cm33_i3c_ibi_irq_count++;
        s_ibi_type = (uint32_t)I3C_GetIBIType(base);
        s_ibi_address = (uint32_t)I3C_GetIBIAddress(base);
        s_ibi_seen = true;
        s_ibi_active = false;
        s_ibi_ack_emitted = false;
    }
}

void I3C0_IRQHandler(void)
{
    i3c_master_state_t masterState = I3C_MasterGetState(EXAMPLE_MASTER);
    uint32_t pending = I3C_MasterGetPendingInterrupts(EXAMPLE_MASTER);
    uint32_t clearable;

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

    if (s_roundtrip_read_active)
    {
        I3C_MasterTransferSmartDMAHandleIRQ(EXAMPLE_MASTER, &s_roundtrip_read_handle);
        SDK_ISR_EXIT_BARRIER;
        return;
    }

    if ((pending & ((uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterArbitrationWonFlag)) != 0U ||
        (masterState == kI3C_MasterStateIbiAck) || (masterState == kI3C_MasterStateIbiRcv) || s_ibi_active)
    {
        service_ibi_protocol_irq(EXAMPLE_MASTER, pending);
        pending = I3C_MasterGetPendingInterrupts(EXAMPLE_MASTER);
    }

    clearable = pending & (uint32_t)kI3C_MasterClearFlags;
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

static void roundtrip_read_ibi_callback(I3C_Type *base,
                                        i3c_master_smartdma_handle_t *handle,
                                        i3c_ibi_type_t ibiType,
                                        i3c_ibi_state_t ibiState)
{
    (void)base;

    if ((ibiType == kI3C_IbiNormal) && (ibiState == kI3C_IbiDataBuffNeed))
    {
        handle->ibiBuff = s_roundtrip_read_ibi_buffer;
    }
}

static void roundtrip_read_complete_callback(I3C_Type *base,
                                             i3c_master_smartdma_handle_t *handle,
                                             status_t status,
                                             void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;

    if (status == kStatus_Success)
    {
        s_roundtrip_read_complete = true;
    }

    if (status == kStatus_I3C_IBIWon)
    {
        s_roundtrip_read_ibi_won = true;
    }

    s_roundtrip_read_active = false;
    s_roundtrip_read_status = status;
}

static void prepare_roundtrip_read_controller(I3C_Type *base)
{
    clear_dma0_channel_state();
    I3C_MasterEnableDMA(base, false, false, 1U);

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, SMART_DMA_TRIGGER_CHANNEL, kINPUTMUX_I3c0IrqToSmartDmaInput);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_I3c0TxToDmac0Ch25RequestEna, false);
    INPUTMUX_Deinit(INPUTMUX);

    NVIC_ClearPendingIRQ(SDMA_IRQn);
    NVIC_SetPriority(SDMA_IRQn, 3);
    NVIC_EnableIRQ(SDMA_IRQn);

    memset(&s_roundtrip_read_handle, 0, sizeof(s_roundtrip_read_handle));
    memset(s_roundtrip_read_ibi_buffer, 0, sizeof(s_roundtrip_read_ibi_buffer));
    s_roundtrip_read_active = false;
    s_roundtrip_read_complete = false;
    s_roundtrip_read_ibi_won = false;
    s_roundtrip_read_status = kStatus_Success;

    SMARTDMA_Reset();
    I3C_MasterTransferCreateHandleSmartDMA(base, &s_roundtrip_read_handle, &s_roundtrip_read_callbacks, NULL);
    NVIC_ClearPendingIRQ(I3C0_IRQn);
    NVIC_ClearPendingIRQ(SDMA_IRQn);
}

static void prepare_roundtrip_read_cpu_controller(I3C_Type *base)
{
    clear_dma0_channel_state();
    I3C_MasterEnableDMA(base, false, false, 1U);

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_I3c0TxToDmac0Ch25RequestEna, false);
    INPUTMUX_Deinit(INPUTMUX);

    NVIC_ClearPendingIRQ(I3C0_IRQn);
    NVIC_ClearPendingIRQ(SDMA_IRQn);
    NVIC_DisableIRQ(SDMA_IRQn);

    memset(&s_roundtrip_read_handle, 0, sizeof(s_roundtrip_read_handle));
    memset(s_roundtrip_read_ibi_buffer, 0, sizeof(s_roundtrip_read_ibi_buffer));
    s_roundtrip_read_active = false;
    s_roundtrip_read_complete = false;
    s_roundtrip_read_ibi_won = false;
    s_roundtrip_read_status = kStatus_Success;
}

static status_t wait_for_roundtrip_read_completion(void)
{
    volatile uint32_t timeout = 0U;

    while ((!s_roundtrip_read_ibi_won) && (!s_roundtrip_read_complete) && (s_roundtrip_read_status == kStatus_Success) &&
           (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT))
    {
        service_transfer_led();
        __NOP();
    }

    if (timeout == I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        return kStatus_Timeout;
    }

    return (status_t)s_roundtrip_read_status;
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

    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
}

static bool buffers_match(const uint8_t *expected, const uint8_t *actual, size_t length)
{
    for (size_t index = 0U; index < length; index++)
    {
        if (expected[index] != actual[index])
        {
            return false;
        }
    }

    return true;
}

static status_t wait_for_smartdma_completion(void)
{
    volatile uint32_t timeout = 0U;

    while ((s_probe_param.mailbox == 0U) && (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT))
    {
        service_transfer_led();
        __NOP();
    }

    if (s_probe_param.mailbox == 0U)
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

    }

    return kStatus_Timeout;
}

static status_t run_cpu_setdasa_fallback(I3C_Type *base, uint8_t *slaveAddr, status_t rstdaaResult, uint32_t attempt)
{
    i3c_master_transfer_t masterXfer;
    status_t setdasaResult;

    EXP_LOG_INFO("Falling back to SETDASA using static address 0x%x.", I3C_TARGET_STATIC_ADDR);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = I3C_BROADCAST_ADDR;
    masterXfer.subaddress = I3C_CCC_SETDASA;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferNoStopFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base,
                               (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                   (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                   (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    capture_daa_setup_snapshot(base,
                               DAA_SETUP_STAGE_SETDASA_ATTEMPT,
                               attempt,
                               rstdaaResult,
                               kStatus_Success,
                               0U,
                               0U);

    setdasaResult = I3C_MasterTransferBlocking(base, &masterXfer);
    if (setdasaResult != kStatus_Success)
    {
        capture_daa_setup_snapshot(base,
                                   DAA_SETUP_STAGE_ERROR_SETDASA_CCC,
                                   attempt,
                                   rstdaaResult,
                                   setdasaResult,
                                   0U,
                                   0U);
        EXP_LOG_ERROR("SETDASA CCC failed: %d", setdasaResult);
        return setdasaResult;
    }

    capture_daa_setup_snapshot(base,
                               DAA_SETUP_STAGE_SETDASA_CCC_OK,
                               attempt,
                               rstdaaResult,
                               setdasaResult,
                               0U,
                               0U);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = I3C_TARGET_STATIC_ADDR;
    masterXfer.subaddress = (uint32_t)(I3C_TARGET_DYNAMIC_ADDR << 1U);
    masterXfer.subaddressSize = 1U;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;

    setdasaResult = I3C_MasterTransferBlocking(base, &masterXfer);
    if (setdasaResult != kStatus_Success)
    {
        capture_daa_setup_snapshot(base,
                                   DAA_SETUP_STAGE_ERROR_SETDASA_DIRECT,
                                   attempt,
                                   rstdaaResult,
                                   setdasaResult,
                                   0U,
                                   0U);
        EXP_LOG_ERROR("SETDASA address transfer failed: %d", setdasaResult);
        return setdasaResult;
    }

    *slaveAddr = I3C_TARGET_DYNAMIC_ADDR;
    capture_daa_setup_snapshot(base,
                               DAA_SETUP_STAGE_SETDASA_DIRECT_OK,
                               attempt,
                               rstdaaResult,
                               setdasaResult,
                               1U,
                               *slaveAddr);
    EXP_LOG_INFO("SETDASA assigned dynamic address 0x%x.", *slaveAddr);

    return kStatus_Success;
}

static status_t run_cpu_rstdaa_and_daa(I3C_Type *base, uint8_t *slaveAddr)
{
    i3c_master_transfer_t masterXfer;
    uint8_t addressList[8] = {0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U};
    uint8_t devCount = 0U;
    i3c_device_info_t *devList;
    status_t rstdaaResult = kStatus_Success;
    status_t daaResult = kStatus_Success;
    uint32_t attempt;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = I3C_BROADCAST_ADDR;
    masterXfer.subaddress = I3C_CCC_RSTDAA;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;

    clear_daa_setup_snapshot();
    capture_daa_setup_snapshot(base, DAA_SETUP_STAGE_CLEARED, 0U, rstdaaResult, daaResult, 0U, 0U);

    for (attempt = 0U; attempt < I3C_DMA_SEED_CHAIN_DAA_RETRY_ATTEMPTS; attempt++)
    {
        I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
        I3C_MasterClearStatusFlags(base,
                                   (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                       (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                       (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
        base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

        capture_daa_setup_snapshot(base,
                                   DAA_SETUP_STAGE_RSTDAA_ATTEMPT,
                                   attempt + 1U,
                                   rstdaaResult,
                                   daaResult,
                                   devCount,
                                   0U);

        rstdaaResult = I3C_MasterTransferBlocking(base, &masterXfer);
        if (rstdaaResult == kStatus_Success)
        {
            break;
        }

        capture_daa_setup_snapshot(base,
                                   DAA_SETUP_STAGE_ERROR_RSTDAA,
                                   attempt + 1U,
                                   rstdaaResult,
                                   daaResult,
                                   devCount,
                                   0U);

        EXP_LOG_ERROR("RSTDAA failed on attempt %lu: %d", (unsigned long)(attempt + 1U), rstdaaResult);

        for (volatile uint32_t delay = 0U; delay < I3C_DMA_SEED_CHAIN_DAA_RETRY_DELAY; delay++)
        {
            __NOP();
        }
    }

    if (rstdaaResult != kStatus_Success)
    {
        return run_cpu_setdasa_fallback(base, slaveAddr, rstdaaResult, I3C_DMA_SEED_CHAIN_DAA_RETRY_ATTEMPTS);
    }

    capture_daa_setup_snapshot(base, DAA_SETUP_STAGE_RSTDAA_OK, attempt + 1U, rstdaaResult, daaResult, 0U, 0U);

    daaResult = I3C_MasterProcessDAA(base, addressList, ARRAY_SIZE(addressList));
    if (daaResult != kStatus_Success)
    {
        capture_daa_setup_snapshot(base,
                                   DAA_SETUP_STAGE_ERROR_DAA,
                                   attempt + 1U,
                                   rstdaaResult,
                                   daaResult,
                                   devCount,
                                   0U);
        EXP_LOG_ERROR("DAA failed: %d", daaResult);
        return daaResult;
    }

    capture_daa_setup_snapshot(base, DAA_SETUP_STAGE_DAA_OK, attempt + 1U, rstdaaResult, daaResult, devCount, 0U);

    devList = I3C_MasterGetDeviceListAfterDAA(base, &devCount);
    for (uint8_t devIndex = 0U; devIndex < devCount; devIndex++)
    {
        if (devList[devIndex].vendorID == 0x123U)
        {
            *slaveAddr = devList[devIndex].dynamicAddr;
            capture_daa_setup_snapshot(base,
                                       DAA_SETUP_STAGE_TARGET_FOUND,
                                       attempt + 1U,
                                       rstdaaResult,
                                       daaResult,
                                       devCount,
                                       *slaveAddr);
            return kStatus_Success;
        }
    }

    capture_daa_setup_snapshot(base,
                               DAA_SETUP_STAGE_ERROR_NO_TARGET,
                               attempt + 1U,
                               rstdaaResult,
                               daaResult,
                               devCount,
                               0U);

    EXP_LOG_ERROR("Target slave not found after DAA.");
    return kStatus_Fail;
}

static void configure_dma_seed_chain(I3C_Type *base)
{
    const uint32_t channel_mask = (1UL << I3C_DMA_TX_CHANNEL);
    const uint32_t one_byte_xfercfg_base = DMA_CHANNEL_XFERCFG_CFGVALID(1U) | DMA_CHANNEL_XFERCFG_SETINTA(1U) |
                                           DMA_CHANNEL_XFERCFG_WIDTH(0U) | DMA_CHANNEL_XFERCFG_SRCINC(1U) |
                                           DMA_CHANNEL_XFERCFG_DSTINC(0U) | DMA_CHANNEL_XFERCFG_XFERCOUNT(0U);
    dma_descriptor_t *bootstrap_descriptor = &s_dma_descriptor_table[I3C_DMA_TX_CHANNEL];

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

    bootstrap_descriptor->xfercfg = one_byte_xfercfg_base;
    bootstrap_descriptor->srcEndAddr = &s_tx_buffer[0];
    bootstrap_descriptor->dstEndAddr = (void *)&base->MWDATAB;
    bootstrap_descriptor->linkToNextDesc = NULL;

    memset((void *)&s_probe_param, 0, sizeof(s_probe_param));
    s_probe_param.expectedWakeCount = 1U;
    s_probe_param.nextTxByteAddress = (uint32_t)(uintptr_t)&s_tx_buffer[0U];
    s_probe_param.remainingCount = I3C_DMA_SEED_CHAIN_LENGTH;
    s_probe_param.i3cBaseAddress = (uint32_t)(uintptr_t)base;
    s_probe_param.dmaIntaAddress = (uint32_t)(uintptr_t)&DMA0->COMMON[0].INTA;
    s_probe_param.dmaChannelMask = channel_mask;
}

static void arm_seed_chain(void)
{
    DMA0->CHANNEL[I3C_DMA_TX_CHANNEL].XFERCFG = s_dma_descriptor_table[I3C_DMA_TX_CHANNEL].xfercfg;
    DMA0->COMMON[0].ENABLESET = s_probe_param.dmaChannelMask;
    DMA0->COMMON[0].SETVALID = s_probe_param.dmaChannelMask;
}

static void configure_slave_ibi_rule(I3C_Type *base, uint8_t slaveAddr)
{
    i3c_register_ibi_addr_t ibiRule;

    memset(&ibiRule, 0, sizeof(ibiRule));
    ibiRule.address[0] = slaveAddr;
    ibiRule.ibiHasPayload = true;
    ibiRule.i3cFastStart = ((slaveAddr & 0x40U) == 0U);

    I3C_MasterRegisterIBI(base, &ibiRule);
}

static status_t wait_for_ibi_notification(I3C_Type *base)
{
    volatile uint32_t timeout = 0U;

    while ((!s_ibi_seen) && (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT))
    {
        service_transfer_led();
        if (I3C_MasterGetErrorStatusFlags(base) != 0U)
        {
            I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
            return kStatus_Fail;
        }
        __NOP();
    }

    if (!s_ibi_seen)
    {
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

static status_t finalize_post_ibi_bus(I3C_Type *base)
{
    capture_post_ibi_handoff_snapshot(base, POST_IBI_HANDOFF_STAGE_BEFORE_FINALIZE, kStatus_Success);

    status_t result = I3C_MasterStop(base);

    if (result == kStatus_Success)
    {
        capture_post_ibi_handoff_snapshot(base, POST_IBI_HANDOFF_STAGE_AFTER_FINALIZE, result);
        return kStatus_Success;
    }

    if (result != kStatus_I3C_InvalidReq)
    {
        capture_post_ibi_handoff_snapshot(base, POST_IBI_HANDOFF_STAGE_AFTER_FINALIZE, result);
        return result;
    }

    if (I3C_MasterGetState(base) == kI3C_MasterStateIdle)
    {
        capture_post_ibi_handoff_snapshot(base, POST_IBI_HANDOFF_STAGE_AFTER_FINALIZE, kStatus_Success);
        return kStatus_Success;
    }

    I3C_MasterEmitRequest(base, kI3C_RequestForceExit);
    result = I3C_MasterWaitForCtrlDone(base, true);
    if ((result == kStatus_Success) && (I3C_MasterGetState(base) == kI3C_MasterStateIdle))
    {
        capture_post_ibi_handoff_snapshot(base, POST_IBI_HANDOFF_STAGE_AFTER_FINALIZE, result);
        return kStatus_Success;
    }

    capture_post_ibi_handoff_snapshot(base, POST_IBI_HANDOFF_STAGE_AFTER_FINALIZE, result);
    return result;
}

static status_t wait_for_post_ibi_read_ctrl_done(I3C_Type *base)
{
    volatile uint32_t timeout = 0U;

    while (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        uint32_t status = I3C_MasterGetStatusFlags(base);
        uint32_t errStatus = I3C_MasterGetErrorStatusFlags(base);

        service_transfer_led();

        if ((status & (uint32_t)kI3C_MasterControlDoneFlag) != 0U)
        {
            I3C_MasterClearStatusFlags(base, (uint32_t)kI3C_MasterControlDoneFlag);
            if (errStatus != 0U)
            {
                I3C_MasterClearErrorStatusFlags(base, errStatus);
            }
            return kStatus_Success;
        }

        if ((errStatus & ~((uint32_t)kI3C_MasterErrorNackFlag)) != 0U)
        {
            return I3C_MasterCheckAndClearError(base, errStatus);
        }
    }

    return kStatus_Timeout;
}

#if (EXPERIMENT_POST_IBI_READ_MODE == EXPERIMENT_POST_IBI_READ_MODE_MANUAL)
static status_t read_roundtrip_payload_manual(I3C_Type *base, uint8_t slaveAddr)
{
    static const uint32_t clear_flags = (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                        (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                        (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag;
    volatile uint32_t timeout = 0U;
    uint8_t *nextByte = s_rx_buffer;
    size_t remaining = sizeof(s_rx_buffer);
    status_t result;

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base, clear_flags);
    base->MSTATUS = I3C_MSTATUS_NACKED_MASK;
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;
    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_CLEARED;
    s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
    s_post_ibi_drain_probe.beginCount = 0U;
    s_post_ibi_drain_probe.endCount = 0U;
    s_post_ibi_drain_probe.rxCount = 0U;
    s_post_ibi_drain_probe.errStatus = 0U;
    s_post_ibi_drain_probe.remaining = (uint32_t)remaining;
    trace_post_ibi_read_window_begin(1U);

    result = I3C_MasterStart(base, kI3C_TypeI3CSdr, slaveAddr, kI3C_Read);
    if (result != kStatus_Success)
    {
        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_START;
        s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
        goto exit;
    }

    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_START_ISSUED;

    result = I3C_MasterWaitForCtrlDone(base, false);
    if (result != kStatus_Success)
    {
        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_CTRL_DONE;
        s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
        goto exit;
    }

    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_CTRL_DONE;
    while (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        uint32_t rxCount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT;
        uint32_t errStatus;

        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_DRAIN_LOOP;
        s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
        trace_post_ibi_drain_probe_begin(rxCount, (uint32_t)remaining);

        while ((remaining != 0U) && (rxCount != 0U))
        {
            *nextByte++ = (uint8_t)(base->MRDATAB & I3C_MRDATAB_VALUE_MASK);
            remaining--;
            rxCount--;
        }

        trace_post_ibi_drain_probe_end(
            (uint32_t)(sizeof(s_rx_buffer) - remaining), I3C_MasterGetErrorStatusFlags(base), (uint32_t)remaining);

        if (remaining == 0U)
        {
            result = I3C_MasterStop(base);
            s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_STOP_SENT;
            s_roundtrip_read_snapshot.remaining = 0U;
            if ((result == kStatus_Success) || (result == kStatus_I3C_InvalidReq))
            {
                result = kStatus_Success;
            }
            goto exit;
        }

        errStatus = I3C_MasterGetErrorStatusFlags(base);
        result = I3C_MasterCheckAndClearError(base, errStatus);
        if (result != kStatus_Success)
        {
            s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_LOOP;
            s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
            goto exit;
        }
    }

    result = kStatus_Timeout;
    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_LOOP;
    s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;

exit:
    trace_post_ibi_read_window_end(s_roundtrip_read_snapshot.stage, result, (uint32_t)remaining);
    return result;
}
#endif

#if (EXPERIMENT_POST_IBI_READ_MODE == EXPERIMENT_POST_IBI_READ_MODE_BLOCKING)
static status_t read_roundtrip_payload_blocking(I3C_Type *base, uint8_t slaveAddr)
{
    i3c_master_transfer_t masterXfer;
    status_t result;

    prepare_roundtrip_read_cpu_controller(base);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = slaveAddr;
    masterXfer.direction = kI3C_Read;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;
    masterXfer.data = s_rx_buffer;
    masterXfer.dataSize = sizeof(s_rx_buffer);

    trace_post_ibi_read_window_begin(2U);
    result = I3C_MasterTransferBlocking(base, &masterXfer);
    s_roundtrip_read_status = result;
    s_roundtrip_read_snapshot.stage =
        (result == kStatus_Success) ? ROUNDTRIP_STAGE_BLOCKING_COMPLETED : ROUNDTRIP_STAGE_ERROR_BLOCKING;
    s_roundtrip_read_snapshot.remaining = 0U;
    trace_post_ibi_read_window_end(s_roundtrip_read_snapshot.stage, result, 0U);

    return result;
}
#endif

static status_t run_roundtrip_read(I3C_Type *base, uint8_t slaveAddr)
{
    i3c_master_transfer_t masterXfer;
    status_t result;

    capture_post_ibi_handoff_snapshot(base, POST_IBI_HANDOFF_STAGE_BEFORE_READ_START, kStatus_Success);
    SDK_DelayAtLeastUs(I3C_DMA_SEED_TAIL_IBI_PRE_READ_DELAY_US, SystemCoreClock);

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base,
                               (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                   (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                   (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));

#if (EXPERIMENT_POST_IBI_READ_MODE == EXPERIMENT_POST_IBI_READ_MODE_MANUAL)
    clear_roundtrip_read_snapshot();
    result = read_roundtrip_payload_manual(base, slaveAddr);
    capture_roundtrip_read_snapshot(base, result);

    if (((result == kStatus_Success) || (result == kStatus_I3C_Nak) || (result == kStatus_I3C_Term)) &&
        buffers_match(s_tx_buffer, s_rx_buffer, sizeof(s_rx_buffer)))
    {
        return kStatus_Success;
    }

    return result;
#endif

#if (EXPERIMENT_POST_IBI_READ_MODE == EXPERIMENT_POST_IBI_READ_MODE_BLOCKING)
    clear_roundtrip_read_snapshot();
    result = read_roundtrip_payload_blocking(base, slaveAddr);
    capture_roundtrip_read_snapshot(base, result);

    if (((result == kStatus_Success) || (result == kStatus_I3C_Nak) || (result == kStatus_I3C_Term)) &&
        buffers_match(s_tx_buffer, s_rx_buffer, sizeof(s_rx_buffer)))
    {
        return kStatus_Success;
    }

    return result;
#endif

    clear_roundtrip_read_snapshot();
    prepare_roundtrip_read_controller(base);
    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_SMARTDMA_PREPARED;

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = slaveAddr;
    masterXfer.direction = kI3C_Read;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;
    masterXfer.data = s_rx_buffer;
    masterXfer.dataSize = sizeof(s_rx_buffer);

    s_roundtrip_read_active = true;
    result = I3C_MasterTransferSmartDMA(base, &s_roundtrip_read_handle, &masterXfer);
    if (result != kStatus_Success)
    {
        s_roundtrip_read_active = false;
        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_SMARTDMA_START;
        capture_roundtrip_read_snapshot(base, result);
        return result;
    }

    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_SMARTDMA_STARTED;
    result = wait_for_roundtrip_read_completion();
    if (result == kStatus_Timeout)
    {
        I3C_MasterTransferAbortSmartDMA(base, &s_roundtrip_read_handle);
        s_roundtrip_read_active = false;
        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_SMARTDMA_WAIT;
    }
    else
    {
        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_SMARTDMA_COMPLETED;
    }

    capture_roundtrip_read_snapshot(base, result);
    if (((result == kStatus_Success) || (result == kStatus_I3C_Nak) || (result == kStatus_I3C_Term)) &&
        buffers_match(s_tx_buffer, s_rx_buffer, sizeof(s_rx_buffer)))
    {
        return kStatus_Success;
    }

    return result;
}

static void dump_probe_counters(void)
{
    EXP_LOG_INFO("dma_seed_bytes=%lu", (unsigned long)s_probe_param.dmaSeedBytes);
    EXP_LOG_INFO("smartdma_bytes=%lu", (unsigned long)s_probe_param.smartdmaBytes);
    EXP_LOG_INFO("dma0_inta_count=%lu", (unsigned long)s_probe_param.dmaIntaCount);
    EXP_LOG_INFO("smartdma_wake_count=%lu", (unsigned long)s_probe_param.wakeCount);
    EXP_LOG_INFO("cm33_i3c_irq_count=%lu", (unsigned long)s_cm33_i3c_irq_count);
    EXP_LOG_INFO("cm33_i3c_data_irq_count=%lu", (unsigned long)s_cm33_i3c_data_irq_count);
    EXP_LOG_INFO("cm33_i3c_protocol_irq_count=%lu", (unsigned long)s_cm33_i3c_protocol_irq_count);
    EXP_LOG_INFO("cm33_i3c_ibi_irq_count=%lu", (unsigned long)s_cm33_i3c_ibi_irq_count);
    EXP_LOG_INFO("ibi_type=%lu", (unsigned long)s_ibi_type);
    EXP_LOG_INFO("ibi_address=0x%02lx", (unsigned long)s_ibi_address);
    EXP_LOG_INFO("ibi_payload_count=%lu", (unsigned long)s_ibi_payload_count);
    EXP_LOG_INFO("ibi_payload0=0x%02x", s_ibi_payload_count != 0U ? s_ibi_payload[0] : 0U);

    PRINTF("dma_seed_bytes=%lu\r\n", (unsigned long)s_probe_param.dmaSeedBytes);
    PRINTF("smartdma_bytes=%lu\r\n", (unsigned long)s_probe_param.smartdmaBytes);
    PRINTF("dma0_inta_count=%lu\r\n", (unsigned long)s_probe_param.dmaIntaCount);
    PRINTF("smartdma_wake_count=%lu\r\n", (unsigned long)s_probe_param.wakeCount);
    PRINTF("cm33_i3c_irq_count=%lu\r\n", (unsigned long)s_cm33_i3c_irq_count);
    PRINTF("cm33_i3c_data_irq_count=%lu\r\n", (unsigned long)s_cm33_i3c_data_irq_count);
    PRINTF("cm33_i3c_protocol_irq_count=%lu\r\n", (unsigned long)s_cm33_i3c_protocol_irq_count);
    PRINTF("cm33_i3c_ibi_irq_count=%lu\r\n", (unsigned long)s_cm33_i3c_ibi_irq_count);
    PRINTF("ibi_type=%lu\r\n", (unsigned long)s_ibi_type);
    PRINTF("ibi_address=0x%02lx\r\n", (unsigned long)s_ibi_address);
    PRINTF("ibi_payload_count=%lu\r\n", (unsigned long)s_ibi_payload_count);
    PRINTF("ibi_payload0=0x%02x\r\n", s_ibi_payload_count != 0U ? s_ibi_payload[0] : 0U);
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

static status_t run_i3c_dma_seed_tail_ibi_probe(I3C_Type *base, uint8_t slaveAddr)
{
    status_t result = kStatus_Success;
    size_t tx_fifo_size = 2UL << ((base->SCAPABILITIES & I3C_SCAPABILITIES_FIFOTX_MASK) >> I3C_SCAPABILITIES_FIFOTX_SHIFT);
    uint8_t saved_tx_trigger_level = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_TXTRIG_MASK) >> I3C_MDATACTRL_TXTRIG_SHIFT);
    uint8_t saved_rx_trigger_level = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_RXTRIG_MASK) >> I3C_MDATACTRL_RXTRIG_SHIFT);
    uint32_t saved_ibi_response = base->MCTRL & I3C_MCTRL_IBIRESP_MASK;

    s_i3c_irq_status_latched = 0U;
    s_cm33_i3c_irq_count = 0U;
    s_cm33_i3c_data_irq_count = 0U;
    s_cm33_i3c_protocol_irq_count = 0U;
    s_cm33_i3c_ibi_irq_count = 0U;
    memset(&s_failure_snapshot, 0, sizeof(s_failure_snapshot));
    clear_ibi_state();
    clear_post_ibi_handoff_snapshot();

    if (tx_fifo_size < I3C_DMA_SEED_CHAIN_LENGTH)
    {
        return kStatus_Fail;
    }

    configure_dma_seed_chain(base);
    configure_slave_ibi_rule(base, slaveAddr);

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base,
                               (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                   (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                   (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;
    base->MCTRL = (base->MCTRL & ~I3C_MCTRL_IBIRESP_MASK) | I3C_MCTRL_IBIRESP(kI3C_IbiRespAckMandatory);
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
    NVIC_ClearPendingIRQ(SDMA_IRQn);
    NVIC_DisableIRQ(SDMA_IRQn);
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
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = wait_for_i3c_ctrl_done(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = wait_for_ibi_notification(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    I3C_MasterDisableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    NVIC_DisableIRQ(I3C0_IRQn);
    NVIC_ClearPendingIRQ(I3C0_IRQn);

    result = finalize_post_ibi_bus(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = run_roundtrip_read(base, slaveAddr);
    if (result != kStatus_Success)
    {
        goto exit;
    }

exit:
    end_transfer_led(result == kStatus_Success);

    if (result != kStatus_Success)
    {
        capture_failure_snapshot(base);
    }

    I3C_MasterDisableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    NVIC_DisableIRQ(I3C0_IRQn);
    NVIC_DisableIRQ(SDMA_IRQn);
    NVIC_ClearPendingIRQ(SDMA_IRQn);
    I3C_MasterEnableDMA(base, false, false, 1U);
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
    base->MCTRL = (base->MCTRL & ~I3C_MCTRL_IBIRESP_MASK) | saved_ibi_response;

    return result;
}

int main(void)
{
    i3c_master_config_t masterConfig;
    status_t result;
    uint8_t slaveAddr = 0U;

    BOARD_InitHardware();
    init_transfer_led();

    for (volatile uint32_t startup_delay = 0U; startup_delay < I3C_DMA_SEED_CHAIN_STARTUP_WAIT; startup_delay++)
    {
        __NOP();
    }

    PRINTF("\r\nI3C DMA seed tail IBI probe -- master.\r\n");
    EXP_LOG_INFO("I3C DMA seed tail IBI probe -- master.");

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
    EXP_LOG_INFO("Starting I3C DMA seed tail IBI probe.");

    result = run_i3c_dma_seed_tail_ibi_probe(EXAMPLE_MASTER, slaveAddr);
    if (result != kStatus_Success)
    {
        EXP_LOG_ERROR("I3C DMA seed tail IBI probe failed: %d", result);
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

    if (s_probe_param.dmaSeedBytes != 1U)
    {
        EXP_LOG_ERROR("DMA seed byte mismatch: expected=1 actual=%lu",
                      (unsigned long)s_probe_param.dmaSeedBytes);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_probe_param.smartdmaBytes != I3C_DMA_SEED_CHAIN_LENGTH)
    {
        EXP_LOG_ERROR("SmartDMA replay byte mismatch: expected=%u actual=%lu",
                      I3C_DMA_SEED_CHAIN_LENGTH,
                      (unsigned long)s_probe_param.smartdmaBytes);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_probe_param.dmaIntaCount != 1U)
    {
        EXP_LOG_ERROR("DMA INTA count mismatch: expected=1 actual=%lu",
                      (unsigned long)s_probe_param.dmaIntaCount);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_cm33_i3c_data_irq_count > s_ibi_payload_count)
    {
        EXP_LOG_ERROR("Unexpected CM33 I3C data-ready IRQ count: actual=%lu ibi_payload=%lu",
                      (unsigned long)s_cm33_i3c_data_irq_count,
                      (unsigned long)s_ibi_payload_count);
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

    if (s_cm33_i3c_ibi_irq_count == 0U)
    {
        EXP_LOG_ERROR("Expected a provoked IBI to complete on CM33.");
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (!s_ibi_seen)
    {
        EXP_LOG_ERROR("IBI notification was not observed.");
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_ibi_payload_count != 1U)
    {
        EXP_LOG_ERROR("Unexpected IBI payload count: %lu", (unsigned long)s_ibi_payload_count);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (s_ibi_payload[0] != I3C_DMA_SEED_TAIL_IBI_PAYLOAD_BYTE)
    {
        EXP_LOG_ERROR("Unexpected IBI payload byte: 0x%02x", s_ibi_payload[0]);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    if (!buffers_match(s_tx_buffer, s_rx_buffer, sizeof(s_rx_buffer)))
    {
        EXP_LOG_ERROR("Roundtrip payload mismatch after seed-tail write.");
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    dump_probe_counters();
    EXP_LOG_INFO("I3C DMA seed tail IBI probe successful.");
    PRINTF("I3C DMA seed tail IBI probe successful.\r\n");
    set_success_led();
    return 0;
}
