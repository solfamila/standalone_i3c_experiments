/*
 * Standalone proof that the validated 8-byte seed-tail write + IBI + post-IBI
 * readback primitive survives a segmented logical-length sweep.
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
#define I3C_DMA_SEED_TAIL_IBI_PRE_READ_DELAY_US 0U
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

#ifndef EXPERIMENT_SLAVE_MIN_ECHO_COUNT
#define EXPERIMENT_SLAVE_MIN_ECHO_COUNT 0U
#endif

#ifndef I3C_SLAVE_RX_DATA_LENGTH
#define I3C_SLAVE_RX_DATA_LENGTH 255U
#endif

#define I3C_DMA_SEED_TAIL_IBI_MAX_PAYLOAD 8U
#define I3C_DMA_SEED_TAIL_SWEEP_MAX_LENGTH 256U
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
static void dump_debug_state(I3C_Type *base);
static status_t finalize_post_ibi_bus(I3C_Type *base);
static status_t ensure_master_idle(I3C_Type *base);

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

typedef struct _length_sweep_snapshot
{
    uint32_t stage;
    uint32_t logicalLength;
    uint32_t offset;
    uint32_t chunkLength;
    uint32_t chunkIndex;
    uint32_t totalSeedBytes;
    uint32_t totalSmartdmaBytes;
    uint32_t totalDataIrqs;
    uint32_t totalProtocolIrqs;
    uint32_t totalIbiIrqs;
    int32_t result;
} length_sweep_snapshot_t;

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
#define ROUNDTRIP_STAGE_ERROR_COMPLETE_NO_DATA 0x107U
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
#define LENGTH_SWEEP_STAGE_CLEARED 1U
#define LENGTH_SWEEP_STAGE_CASE_START 2U
#define LENGTH_SWEEP_STAGE_CHUNK_START 3U
#define LENGTH_SWEEP_STAGE_CHUNK_OK 4U
#define LENGTH_SWEEP_STAGE_CASE_OK 5U
#define LENGTH_SWEEP_STAGE_ERROR_CHUNK_TRANSFER 0x101U
#define LENGTH_SWEEP_STAGE_ERROR_CHUNK_VALIDATE 0x102U
#define LENGTH_SWEEP_STAGE_ERROR_LOGICAL_MISMATCH 0x103U
#define LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_SEED 0x104U
#define LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_SMARTDMA 0x105U
#define LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_DATA_IRQ 0x106U
#define LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_PROTOCOL_IBI 0x107U
#define CHUNK_VALIDATE_FAIL_NONE 0U
#define CHUNK_VALIDATE_FAIL_ONE_BYTE_SEED_BYTES 1U
#define CHUNK_VALIDATE_FAIL_ONE_BYTE_SMARTDMA_BYTES 2U
#define CHUNK_VALIDATE_FAIL_MAILBOX 3U
#define CHUNK_VALIDATE_FAIL_DMA_SEED_BYTES 4U
#define CHUNK_VALIDATE_FAIL_WAKE_COUNT 5U
#define CHUNK_VALIDATE_FAIL_SEED_ONLY_BOOKKEEPING 6U
#define CHUNK_VALIDATE_FAIL_SMARTDMA_REPLAY_BYTES 7U
#define CHUNK_VALIDATE_FAIL_DMA_INTA_COUNT 8U
#define CHUNK_VALIDATE_FAIL_DATA_IRQ_COUNT 9U
#define CHUNK_VALIDATE_FAIL_PROTOCOL_IRQ_COUNT 10U
#define CHUNK_VALIDATE_FAIL_IBI_IRQ_COUNT 11U
#define CHUNK_VALIDATE_FAIL_IBI_SEEN 12U
#define CHUNK_VALIDATE_FAIL_IBI_PAYLOAD_COUNT 13U
#define CHUNK_VALIDATE_FAIL_IBI_PAYLOAD_BYTE 14U
#define CHUNK_VALIDATE_FAIL_ROUNDTRIP_BUFFER 15U
#define ONE_BYTE_PROBE_STAGE_CLEARED 1U
#define ONE_BYTE_PROBE_STAGE_START_ISSUED 2U
#define ONE_BYTE_PROBE_STAGE_START_CTRL_DONE 3U
#define ONE_BYTE_PROBE_STAGE_TX_READY 4U
#define ONE_BYTE_PROBE_STAGE_BYTE_WRITTEN 5U
#define ONE_BYTE_PROBE_STAGE_COMPLETE 6U
#define ONE_BYTE_PROBE_STAGE_STOP_SENT 7U
#define ONE_BYTE_PROBE_STAGE_STOP_CTRL_DONE 8U
#define ONE_BYTE_PROBE_STAGE_IBI_SEEN 9U
#define ONE_BYTE_PROBE_STAGE_FINALIZED 10U
#define ONE_BYTE_PROBE_STAGE_READ_DONE 11U
#define DMA_PROBE_STAGE_CLEARED 1U
#define DMA_PROBE_STAGE_START_ISSUED 2U
#define DMA_PROBE_STAGE_START_CTRL_DONE 3U
#define DMA_PROBE_STAGE_SMARTDMA_DONE 4U
#define DMA_PROBE_STAGE_COMPLETE 5U
#define DMA_PROBE_STAGE_STOP_SENT 6U
#define DMA_PROBE_STAGE_STOP_CTRL_DONE 7U
#define DMA_PROBE_STAGE_IBI_SEEN 8U
#define DMA_PROBE_STAGE_FINALIZED 9U
#define DMA_PROBE_STAGE_IDLE 10U
#define DMA_PROBE_STAGE_READ_DONE 11U

#define POST_IBI_MANUAL_READ_STAGE_CLEARED 1U
#define POST_IBI_MANUAL_READ_STAGE_START_ISSUED 2U
#define POST_IBI_MANUAL_READ_STAGE_CTRL_DONE 3U
#define POST_IBI_MANUAL_READ_STAGE_DRAIN_LOOP 4U
#define POST_IBI_MANUAL_READ_STAGE_STOP_SENT 5U
#define POST_IBI_MANUAL_READ_STAGE_EXIT 6U
#define POST_IBI_MANUAL_READ_STAGE_COMPLETE_NO_DATA 7U

typedef enum _length_sweep_chunk_mode
{
    kLengthSweepChunkModeNone = 0,
    kLengthSweepChunkModeSeedTail = 1,
    kLengthSweepChunkModeOneByteCpu = 2,
} length_sweep_chunk_mode_t;

AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_descriptor_table[FSL_FEATURE_DMA_MAX_CHANNELS],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static dma_descriptor_t s_dma_seed_chain[I3C_DMA_SEED_CHAIN_LENGTH - 1U],
                              FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_tx_buffer[I3C_DMA_SEED_CHAIN_LENGTH], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_rx_buffer[I3C_DMA_SEED_CHAIN_LENGTH], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_logical_tx_buffer[I3C_DMA_SEED_TAIL_SWEEP_MAX_LENGTH], 4);
AT_NONCACHEABLE_SECTION_ALIGN(static uint8_t s_logical_rx_buffer[I3C_DMA_SEED_TAIL_SWEEP_MAX_LENGTH], 4);
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
static __NO_INIT length_sweep_snapshot_t s_length_sweep_snapshot;
static __NO_INIT volatile uint32_t s_length_sweep_probe_logical;
static __NO_INIT volatile uint32_t s_length_sweep_probe_offset;
static __NO_INIT volatile uint32_t s_length_sweep_probe_chunk;
static __NO_INIT volatile uint32_t s_length_sweep_probe_chunk_index;
static __NO_INIT volatile uint32_t s_length_sweep_probe_stage;
static __NO_INIT volatile int32_t s_length_sweep_probe_result;
static __NO_INIT volatile uint32_t s_one_byte_probe_stage;
static __NO_INIT volatile int32_t s_one_byte_probe_result;
static __NO_INIT volatile uint32_t s_dma_probe_stage;
static __NO_INIT volatile int32_t s_dma_probe_result;
static __NO_INIT volatile uint32_t s_no_ibi_probe_attempted;
static __NO_INIT volatile int32_t s_no_ibi_probe_result;
static __NO_INIT volatile uint8_t s_no_ibi_probe_data0;
static __NO_INIT volatile uint32_t s_post_ibi_manual_read_stage;
static __NO_INIT volatile uint32_t s_post_ibi_manual_read_state;
static __NO_INIT volatile uint32_t s_post_ibi_manual_read_mstatus;
static __NO_INIT volatile uint32_t s_post_ibi_manual_read_merrwarn;
static __NO_INIT volatile uint32_t s_post_ibi_manual_read_mdatactrl;
static __NO_INIT volatile uint32_t s_post_ibi_manual_read_remaining;
static __NO_INIT volatile uint32_t s_post_ibi_manual_read_rxcount;
static __NO_INIT volatile uint32_t s_post_ibi_start_pre_state;
static __NO_INIT volatile uint32_t s_post_ibi_start_pre_mstatus;
static __NO_INIT volatile int32_t s_post_ibi_start_result;
static __NO_INIT volatile uint32_t s_post_ibi_start_post_state;
static __NO_INIT volatile uint32_t s_post_ibi_start_post_mstatus;
static __NO_INIT volatile int32_t s_post_ibi_ctrl_done_result;
static __NO_INIT volatile uint32_t s_post_ibi_ctrl_done_state;
static __NO_INIT volatile uint32_t s_post_ibi_ctrl_done_mstatus;
static __NO_INIT volatile uint32_t s_protocol_trace_count;
static __NO_INIT volatile uint32_t s_protocol_trace_pending[4];
static __NO_INIT volatile uint32_t s_protocol_trace_state[4];
static __NO_INIT volatile uint32_t s_protocol_trace_mstatus[4];
static __NO_INIT volatile uint32_t s_chunk_validate_reason;
static __NO_INIT volatile uint32_t s_chunk_validate_value0;
static __NO_INIT volatile uint32_t s_chunk_validate_value1;
static __NO_INIT volatile uint32_t s_chunk_validate_tx0;
static __NO_INIT volatile uint32_t s_chunk_validate_rx0;
static __NO_INIT volatile uint32_t s_chunk_validate_ibi0;
static bool s_transfer_led_active = false;
static uint32_t s_transfer_led_poll_count = 0U;
static uint32_t s_transfer_led_toggle_count = 0U;
static size_t s_active_transfer_length = I3C_DMA_SEED_CHAIN_LENGTH;
static length_sweep_chunk_mode_t s_active_chunk_mode = kLengthSweepChunkModeSeedTail;

static const uint16_t s_length_sweep_cases[] = {
    0U, 1U, 2U, 7U, 8U, 9U, 15U, 16U, 31U, 32U, 63U, 64U, 255U, 256U,
};

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
    s_post_ibi_start_pre_state = 0U;
    s_post_ibi_start_pre_mstatus = 0U;
    s_post_ibi_start_result = 0;
    s_post_ibi_start_post_state = 0U;
    s_post_ibi_start_post_mstatus = 0U;
    s_post_ibi_ctrl_done_result = 0;
    s_post_ibi_ctrl_done_state = 0U;
    s_post_ibi_ctrl_done_mstatus = 0U;
}

static void clear_post_ibi_handoff_snapshot(void)
{
    memset(&s_post_ibi_handoff_snapshot, 0, sizeof(s_post_ibi_handoff_snapshot));
}

static void clear_protocol_trace(void)
{
    s_protocol_trace_count = 0U;
    memset((void *)s_protocol_trace_pending, 0, sizeof(s_protocol_trace_pending));
    memset((void *)s_protocol_trace_state, 0, sizeof(s_protocol_trace_state));
    memset((void *)s_protocol_trace_mstatus, 0, sizeof(s_protocol_trace_mstatus));
}

static void capture_protocol_trace(I3C_Type *base, uint32_t pending, i3c_master_state_t masterState)
{
    uint32_t index = s_protocol_trace_count;

    if (index < ARRAY_SIZE(s_protocol_trace_pending))
    {
        s_protocol_trace_pending[index] = pending;
        s_protocol_trace_state[index] = (uint32_t)masterState;
        s_protocol_trace_mstatus[index] = base->MSTATUS;
    }

    s_protocol_trace_count = index + 1U;
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

static void capture_post_ibi_manual_read_probe(I3C_Type *base, uint32_t stage, size_t remaining, uint32_t rxCount)
{
    s_post_ibi_manual_read_stage = stage;
    s_post_ibi_manual_read_state = (uint32_t)I3C_MasterGetState(base);
    s_post_ibi_manual_read_mstatus = base->MSTATUS;
    s_post_ibi_manual_read_merrwarn = base->MERRWARN;
    s_post_ibi_manual_read_mdatactrl = base->MDATACTRL;
    s_post_ibi_manual_read_remaining = (uint32_t)remaining;
    s_post_ibi_manual_read_rxcount = rxCount;
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
        capture_protocol_trace(EXAMPLE_MASTER, pending, masterState);
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

static void prepare_logical_payload(size_t length)
{
    uint32_t index;

    memset(s_logical_tx_buffer, 0, sizeof(s_logical_tx_buffer));
    memset(s_logical_rx_buffer, 0, sizeof(s_logical_rx_buffer));

    for (index = 0U; index < length; index++)
    {
        s_logical_tx_buffer[index] = (uint8_t)(index + 1U);
    }
}

static void load_transfer_window(const uint8_t *src, size_t length)
{
    memset(s_tx_buffer, 0, sizeof(s_tx_buffer));
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));

    if ((src != NULL) && (length != 0U))
    {
        memcpy(s_tx_buffer, src, length);
    }

    s_active_transfer_length = length;
    s_active_chunk_mode = (length == 1U) ? kLengthSweepChunkModeOneByteCpu : kLengthSweepChunkModeSeedTail;
}

static bool active_transfer_uses_seed_only_write(void)
{
    return s_active_transfer_length == 1U;
}

static size_t choose_chunk_length(size_t remaining)
{
    if (remaining == 0U)
    {
        return 0U;
    }

    if (remaining <= (size_t)I3C_DMA_SEED_CHAIN_LENGTH)
    {
        return remaining;
    }

    if ((remaining % (size_t)I3C_DMA_SEED_CHAIN_LENGTH) == 1U)
    {
        return (size_t)I3C_DMA_SEED_CHAIN_LENGTH - 1U;
    }

    return (size_t)I3C_DMA_SEED_CHAIN_LENGTH;
}

static void clear_length_sweep_snapshot(void)
{
    memset(&s_length_sweep_snapshot, 0, sizeof(s_length_sweep_snapshot));
    s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_CLEARED;
    s_length_sweep_probe_logical = 0U;
    s_length_sweep_probe_offset = 0U;
    s_length_sweep_probe_chunk = 0U;
    s_length_sweep_probe_chunk_index = 0U;
    s_length_sweep_probe_stage = s_length_sweep_snapshot.stage;
    s_length_sweep_probe_result = 0;
}

static void mirror_length_sweep_probe(void)
{
    s_length_sweep_probe_logical = s_length_sweep_snapshot.logicalLength;
    s_length_sweep_probe_offset = s_length_sweep_snapshot.offset;
    s_length_sweep_probe_chunk = s_length_sweep_snapshot.chunkLength;
    s_length_sweep_probe_chunk_index = s_length_sweep_snapshot.chunkIndex;
    s_length_sweep_probe_stage = s_length_sweep_snapshot.stage;
    s_length_sweep_probe_result = s_length_sweep_snapshot.result;
}

static void capture_chunk_validate_failure(uint32_t reason, uint32_t value0, uint32_t value1)
{
    s_chunk_validate_reason = reason;
    s_chunk_validate_value0 = value0;
    s_chunk_validate_value1 = value1;
    s_chunk_validate_tx0 = (s_active_transfer_length != 0U) ? s_tx_buffer[0] : 0U;
    s_chunk_validate_rx0 = (s_active_transfer_length != 0U) ? s_rx_buffer[0] : 0U;
    s_chunk_validate_ibi0 = (s_ibi_payload_count != 0U) ? s_ibi_payload[0] : 0U;
}

static uint8_t expected_post_rx_ibi_payload_byte(void)
{
    uint32_t echoedCount = (uint32_t)s_active_transfer_length;

#if defined(EXPERIMENT_SLAVE_ECHO_COUNT_PLUS_ONE)
    if (echoedCount < I3C_SLAVE_RX_DATA_LENGTH)
    {
        echoedCount++;
    }
#endif

    if ((EXPERIMENT_SLAVE_MIN_ECHO_COUNT != 0U) && (echoedCount < EXPERIMENT_SLAVE_MIN_ECHO_COUNT))
    {
        echoedCount = EXPERIMENT_SLAVE_MIN_ECHO_COUNT;
    }

    return (uint8_t)echoedCount;
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

static status_t wait_for_one_byte_tx_ready(I3C_Type *base)
{
    const size_t tx_fifo_size =
        2UL << ((base->SCAPABILITIES & I3C_SCAPABILITIES_FIFOTX_MASK) >> I3C_SCAPABILITIES_FIFOTX_SHIFT);
    volatile uint32_t timeout = 0U;

    while (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        size_t tx_count = 0U;
        uint32_t error_status = I3C_MasterGetErrorStatusFlags(base);

        service_transfer_led();

        if ((error_status != 0U) || ((s_i3c_irq_status_latched & (uint32_t)kI3C_MasterErrorFlag) != 0U))
        {
            I3C_MasterClearErrorStatusFlags(base, error_status);
            s_i3c_irq_status_latched &= ~((uint32_t)kI3C_MasterErrorFlag);
            return kStatus_Fail;
        }

        I3C_MasterGetFifoCounts(base, NULL, &tx_count);
        if ((tx_fifo_size - tx_count) != 0U)
        {
            return kStatus_Success;
        }
    }

    return kStatus_Timeout;
}

static status_t wait_for_master_idle(I3C_Type *base)
{
    volatile uint32_t timeout = 0U;

    while (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        uint32_t error_status = I3C_MasterGetErrorStatusFlags(base);
        i3c_master_state_t masterState = I3C_MasterGetState(base);

        service_transfer_led();

        if ((error_status != 0U) || ((s_i3c_irq_status_latched & (uint32_t)kI3C_MasterErrorFlag) != 0U))
        {
            I3C_MasterClearErrorStatusFlags(base, error_status);
            s_i3c_irq_status_latched &= ~((uint32_t)kI3C_MasterErrorFlag);
            return kStatus_Fail;
        }

        if ((masterState == kI3C_MasterStateIdle) && I3C_MasterGetBusIdleState(base))
        {
            return kStatus_Success;
        }

        if (masterState == kI3C_MasterStateSlvReq)
        {
            status_t result = finalize_post_ibi_bus(base);

            if (result != kStatus_Success)
            {
                return result;
            }
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

    setdasaResult = ensure_master_idle(base);
    if (setdasaResult != kStatus_Success)
    {
        capture_daa_setup_snapshot(base,
                                   DAA_SETUP_STAGE_ERROR_SETDASA_CCC,
                                   attempt,
                                   rstdaaResult,
                                   setdasaResult,
                                   0U,
                                   0U);
        return setdasaResult;
    }

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

    setdasaResult = ensure_master_idle(base);
    if (setdasaResult != kStatus_Success)
    {
        capture_daa_setup_snapshot(base,
                                   DAA_SETUP_STAGE_ERROR_SETDASA_DIRECT,
                                   attempt,
                                   rstdaaResult,
                                   setdasaResult,
                                   0U,
                                   0U);
        return setdasaResult;
    }

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

        rstdaaResult = ensure_master_idle(base);
        if (rstdaaResult != kStatus_Success)
        {
            capture_daa_setup_snapshot(base,
                                       DAA_SETUP_STAGE_ERROR_RSTDAA,
                                       attempt + 1U,
                                       rstdaaResult,
                                       daaResult,
                                       devCount,
                                       0U);
            return rstdaaResult;
        }

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
    const bool seed_only_write = active_transfer_uses_seed_only_write();
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
    bootstrap_descriptor->dstEndAddr = seed_only_write ? (void *)&base->MWDATABE : (void *)&base->MWDATAB;
    bootstrap_descriptor->linkToNextDesc = NULL;

    memset((void *)&s_probe_param, 0, sizeof(s_probe_param));
    s_probe_param.expectedWakeCount = 1U;
    s_probe_param.nextTxByteAddress = (uint32_t)(uintptr_t)&s_tx_buffer[seed_only_write ? 1U : 0U];
    s_probe_param.remainingCount = seed_only_write ? 0U : (uint32_t)s_active_transfer_length;
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

static status_t ensure_master_idle(I3C_Type *base)
{
    status_t result;

    if ((I3C_MasterGetState(base) == kI3C_MasterStateIdle) && I3C_MasterGetBusIdleState(base))
    {
        return kStatus_Success;
    }

    result = finalize_post_ibi_bus(base);

    if (result != kStatus_Success)
    {
        return result;
    }

    return wait_for_master_idle(base);
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
    size_t remaining = s_active_transfer_length;
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
    capture_post_ibi_manual_read_probe(base, POST_IBI_MANUAL_READ_STAGE_CLEARED, remaining, 0U);
    trace_post_ibi_read_window_begin(1U);

    s_post_ibi_start_pre_state = (uint32_t)I3C_MasterGetState(base);
    s_post_ibi_start_pre_mstatus = base->MSTATUS;
    result = I3C_MasterStartWithRxSize(base, kI3C_TypeI3CSdr, slaveAddr, kI3C_Read, (uint8_t)remaining);
    s_post_ibi_start_result = result;
    s_post_ibi_start_post_state = (uint32_t)I3C_MasterGetState(base);
    s_post_ibi_start_post_mstatus = base->MSTATUS;
    if (result != kStatus_Success)
    {
        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_START;
        s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
        goto exit;
    }

    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_START_ISSUED;
    capture_post_ibi_manual_read_probe(base, POST_IBI_MANUAL_READ_STAGE_START_ISSUED, remaining, 0U);

    result = wait_for_post_ibi_read_ctrl_done(base);
    s_post_ibi_ctrl_done_result = result;
    s_post_ibi_ctrl_done_state = (uint32_t)I3C_MasterGetState(base);
    s_post_ibi_ctrl_done_mstatus = base->MSTATUS;
    if (result != kStatus_Success)
    {
        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_CTRL_DONE;
        s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
        goto exit;
    }

    s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_CTRL_DONE;
    capture_post_ibi_manual_read_probe(base, POST_IBI_MANUAL_READ_STAGE_CTRL_DONE, remaining, 0U);
    while (++timeout < I3C_DMA_SEED_CHAIN_TIMEOUT)
    {
        uint32_t rxCount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT;
        uint32_t errStatus;
        uint32_t statusFlags;

        s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_DRAIN_LOOP;
        s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
        capture_post_ibi_manual_read_probe(base, POST_IBI_MANUAL_READ_STAGE_DRAIN_LOOP, remaining, rxCount);
        trace_post_ibi_drain_probe_begin(rxCount, (uint32_t)remaining);

        while ((remaining != 0U) && (rxCount != 0U))
        {
            *nextByte++ = (uint8_t)(base->MRDATAB & I3C_MRDATAB_VALUE_MASK);
            remaining--;
            rxCount--;
        }

        trace_post_ibi_drain_probe_end(
            (uint32_t)(s_active_transfer_length - remaining),
            I3C_MasterGetErrorStatusFlags(base),
            (uint32_t)remaining);

        statusFlags = I3C_MasterGetStatusFlags(base);

        if (remaining == 0U)
        {
            result = I3C_MasterStop(base);
            s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_STOP_SENT;
            s_roundtrip_read_snapshot.remaining = 0U;
            capture_post_ibi_manual_read_probe(base, POST_IBI_MANUAL_READ_STAGE_STOP_SENT, 0U, 0U);
            if ((result == kStatus_Success) || (result == kStatus_I3C_InvalidReq))
            {
                result = kStatus_Success;
            }
            goto exit;
        }

        if (((statusFlags & ((uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterNackDetectFlag)) ==
             ((uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterNackDetectFlag)) &&
            (rxCount == 0U))
        {
            s_roundtrip_read_snapshot.stage = ROUNDTRIP_STAGE_ERROR_COMPLETE_NO_DATA;
            s_roundtrip_read_snapshot.remaining = (uint32_t)remaining;
            capture_post_ibi_manual_read_probe(
                base, POST_IBI_MANUAL_READ_STAGE_COMPLETE_NO_DATA, remaining, rxCount);
            result = kStatus_I3C_Nak;
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
    capture_post_ibi_manual_read_probe(
        base,
        POST_IBI_MANUAL_READ_STAGE_EXIT,
        remaining,
        (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT);
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
    masterXfer.dataSize = s_active_transfer_length;

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
        buffers_match(s_tx_buffer, s_rx_buffer, s_active_transfer_length))
    {
        return ensure_master_idle(base);
    }

    return result;
#endif

#if (EXPERIMENT_POST_IBI_READ_MODE == EXPERIMENT_POST_IBI_READ_MODE_BLOCKING)
    clear_roundtrip_read_snapshot();
    result = read_roundtrip_payload_blocking(base, slaveAddr);
    capture_roundtrip_read_snapshot(base, result);

    if (((result == kStatus_Success) || (result == kStatus_I3C_Nak) || (result == kStatus_I3C_Term)) &&
        buffers_match(s_tx_buffer, s_rx_buffer, s_active_transfer_length))
    {
        return ensure_master_idle(base);
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
    masterXfer.dataSize = s_active_transfer_length;

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
        buffers_match(s_tx_buffer, s_rx_buffer, s_active_transfer_length))
    {
        return ensure_master_idle(base);
    }

    return result;
}

static status_t run_i3c_one_byte_cpu_write_ibi_probe(I3C_Type *base, uint8_t slaveAddr)
{
    status_t result = kStatus_Success;
    status_t no_ibi_probe_result = kStatus_Success;
    uint8_t saved_tx_trigger_level = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_TXTRIG_MASK) >> I3C_MDATACTRL_TXTRIG_SHIFT);
    uint8_t saved_rx_trigger_level = (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_RXTRIG_MASK) >> I3C_MDATACTRL_RXTRIG_SHIFT);
    uint32_t saved_ibi_response = base->MCTRL & I3C_MCTRL_IBIRESP_MASK;

    s_i3c_irq_status_latched = 0U;
    s_cm33_i3c_irq_count = 0U;
    s_cm33_i3c_data_irq_count = 0U;
    s_cm33_i3c_protocol_irq_count = 0U;
    s_cm33_i3c_ibi_irq_count = 0U;
    memset(&s_failure_snapshot, 0, sizeof(s_failure_snapshot));
    memset((void *)&s_probe_param, 0, sizeof(s_probe_param));
    clear_ibi_state();
    clear_post_ibi_handoff_snapshot();
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_CLEARED;
    s_one_byte_probe_result = (int32_t)kStatus_Success;
    s_no_ibi_probe_attempted = 0U;
    s_no_ibi_probe_result = (int32_t)kStatus_Success;
    s_no_ibi_probe_data0 = 0U;

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
    INPUTMUX_EnableSignal(INPUTMUX, kINPUTMUX_I3c0TxToDmac0Ch25RequestEna, false);
    INPUTMUX_Deinit(INPUTMUX);

    NVIC_ClearPendingIRQ(I3C0_IRQn);
    NVIC_SetPriority(I3C0_IRQn, 3);
    NVIC_EnableIRQ(I3C0_IRQn);
    I3C_MasterEnableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    begin_transfer_led();

    result = ensure_master_idle(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = I3C_MasterStart(base, kI3C_TypeI3CSdr, slaveAddr, kI3C_Write);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_START_ISSUED;

    result = wait_for_i3c_ctrl_done(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_START_CTRL_DONE;

    result = wait_for_one_byte_tx_ready(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_TX_READY;

    base->MWDATABE = s_tx_buffer[0];
    s_probe_param.dmaSeedBytes = 1U;
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_BYTE_WRITTEN;

    result = wait_for_i3c_complete(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_COMPLETE;

    result = I3C_MasterStop(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_STOP_SENT;

    result = wait_for_i3c_ctrl_done(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_STOP_CTRL_DONE;

    clear_protocol_trace();

    result = wait_for_ibi_notification(base);
    if (result != kStatus_Success)
    {
        if (result == kStatus_Timeout)
        {
            s_no_ibi_probe_attempted = 1U;
            I3C_MasterDisableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
            NVIC_DisableIRQ(I3C0_IRQn);
            NVIC_ClearPendingIRQ(I3C0_IRQn);

            no_ibi_probe_result = finalize_post_ibi_bus(base);
            if (no_ibi_probe_result == kStatus_Success)
            {
                no_ibi_probe_result = run_roundtrip_read(base, slaveAddr);
                s_no_ibi_probe_data0 = s_rx_buffer[0];
            }

            s_no_ibi_probe_result = (int32_t)no_ibi_probe_result;
        }
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_IBI_SEEN;

    I3C_MasterDisableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    NVIC_DisableIRQ(I3C0_IRQn);
    NVIC_ClearPendingIRQ(I3C0_IRQn);

    result = finalize_post_ibi_bus(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_FINALIZED;

    result = ensure_master_idle(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    result = run_roundtrip_read(base, slaveAddr);
    if (result == kStatus_Success)
    {
        s_one_byte_probe_stage = ONE_BYTE_PROBE_STAGE_READ_DONE;
    }

exit:
    s_one_byte_probe_result = (int32_t)result;
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

static status_t validate_chunk_result(I3C_Type *base)
{
    const bool one_byte_cpu_chunk = s_active_chunk_mode == kLengthSweepChunkModeOneByteCpu;
    const bool seed_only_write = active_transfer_uses_seed_only_write();
    const uint8_t expectedIbiPayloadByte = expected_post_rx_ibi_payload_byte();

    s_chunk_validate_reason = CHUNK_VALIDATE_FAIL_NONE;
    s_chunk_validate_value0 = 0U;
    s_chunk_validate_value1 = 0U;
    s_chunk_validate_tx0 = (s_active_transfer_length != 0U) ? s_tx_buffer[0] : 0U;
    s_chunk_validate_rx0 = (s_active_transfer_length != 0U) ? s_rx_buffer[0] : 0U;
    s_chunk_validate_ibi0 = (s_ibi_payload_count != 0U) ? s_ibi_payload[0] : 0U;

    if (one_byte_cpu_chunk)
    {
        if (s_probe_param.dmaSeedBytes != 1U)
        {
            capture_chunk_validate_failure(
                CHUNK_VALIDATE_FAIL_ONE_BYTE_SEED_BYTES, 1U, (uint32_t)s_probe_param.dmaSeedBytes);
            EXP_LOG_ERROR("One-byte CPU chunk count mismatch: expected=1 actual=%lu",
                          (unsigned long)s_probe_param.dmaSeedBytes);
            dump_debug_state(base);
            return kStatus_Fail;
        }

        if (s_probe_param.smartdmaBytes != 0U)
        {
            capture_chunk_validate_failure(
                CHUNK_VALIDATE_FAIL_ONE_BYTE_SMARTDMA_BYTES, 0U, (uint32_t)s_probe_param.smartdmaBytes);
            EXP_LOG_ERROR("One-byte CPU chunk should not use SmartDMA payload bytes: actual=%lu",
                          (unsigned long)s_probe_param.smartdmaBytes);
            dump_debug_state(base);
            return kStatus_Fail;
        }
    }
    else
    {
        if (s_probe_param.mailbox != 1U)
        {
            capture_chunk_validate_failure(CHUNK_VALIDATE_FAIL_MAILBOX, 1U, (uint32_t)s_probe_param.mailbox);
            EXP_LOG_ERROR("SmartDMA mailbox mismatch: %lu", (unsigned long)s_probe_param.mailbox);
            dump_debug_state(base);
            return kStatus_Fail;
        }

        if (s_probe_param.dmaSeedBytes != 1U)
        {
            capture_chunk_validate_failure(
                CHUNK_VALIDATE_FAIL_DMA_SEED_BYTES, 1U, (uint32_t)s_probe_param.dmaSeedBytes);
            EXP_LOG_ERROR("DMA seed byte mismatch: expected=1 actual=%lu", (unsigned long)s_probe_param.dmaSeedBytes);
            dump_debug_state(base);
            return kStatus_Fail;
        }

        if (s_probe_param.wakeCount != s_probe_param.expectedWakeCount)
        {
            capture_chunk_validate_failure(CHUNK_VALIDATE_FAIL_WAKE_COUNT,
                                           (uint32_t)s_probe_param.expectedWakeCount,
                                           (uint32_t)s_probe_param.wakeCount);
            EXP_LOG_ERROR("Wake count mismatch: expected=%lu actual=%lu",
                          (unsigned long)s_probe_param.expectedWakeCount,
                          (unsigned long)s_probe_param.wakeCount);
            dump_debug_state(base);
            return kStatus_Fail;
        }

        if (seed_only_write &&
            ((s_probe_param.smartdmaBytes != 0U) || (s_probe_param.dmaIntaCount != 1U)))
        {
            capture_chunk_validate_failure(CHUNK_VALIDATE_FAIL_SEED_ONLY_BOOKKEEPING,
                                           (uint32_t)s_probe_param.smartdmaBytes,
                                           (uint32_t)s_probe_param.dmaIntaCount);
            EXP_LOG_ERROR("Seed-only chunk bookkeeping mismatch: smartdma=%lu dma_inta=%lu",
                          (unsigned long)s_probe_param.smartdmaBytes,
                          (unsigned long)s_probe_param.dmaIntaCount);
            dump_debug_state(base);
            return kStatus_Fail;
        }

        if ((!seed_only_write) && (s_probe_param.smartdmaBytes != s_active_transfer_length))
        {
            capture_chunk_validate_failure(CHUNK_VALIDATE_FAIL_SMARTDMA_REPLAY_BYTES,
                                           (uint32_t)s_active_transfer_length,
                                           (uint32_t)s_probe_param.smartdmaBytes);
            EXP_LOG_ERROR("SmartDMA replay byte mismatch: expected=%u actual=%lu",
                          (unsigned int)s_active_transfer_length,
                          (unsigned long)s_probe_param.smartdmaBytes);
            dump_debug_state(base);
            return kStatus_Fail;
        }

        if ((!seed_only_write) && (s_probe_param.dmaIntaCount != 1U))
        {
            capture_chunk_validate_failure(
                CHUNK_VALIDATE_FAIL_DMA_INTA_COUNT, 1U, (uint32_t)s_probe_param.dmaIntaCount);
            EXP_LOG_ERROR("DMA INTA count mismatch: expected=1 actual=%lu", (unsigned long)s_probe_param.dmaIntaCount);
            dump_debug_state(base);
            return kStatus_Fail;
        }
    }

    if (s_cm33_i3c_data_irq_count > s_ibi_payload_count)
    {
        capture_chunk_validate_failure(
            CHUNK_VALIDATE_FAIL_DATA_IRQ_COUNT, (uint32_t)s_ibi_payload_count, (uint32_t)s_cm33_i3c_data_irq_count);
        EXP_LOG_ERROR("Unexpected CM33 I3C data-ready IRQ count: actual=%lu ibi_payload=%lu",
                      (unsigned long)s_cm33_i3c_data_irq_count,
                      (unsigned long)s_ibi_payload_count);
        dump_debug_state(base);
        return kStatus_Fail;
    }

    if ((!one_byte_cpu_chunk) && (s_cm33_i3c_protocol_irq_count == 0U))
    {
        capture_chunk_validate_failure(
            CHUNK_VALIDATE_FAIL_PROTOCOL_IRQ_COUNT, 1U, (uint32_t)s_cm33_i3c_protocol_irq_count);
        EXP_LOG_ERROR("Expected protocol IRQs to remain visible on CM33.");
        dump_debug_state(base);
        return kStatus_Fail;
    }

    if (s_cm33_i3c_ibi_irq_count == 0U)
    {
        capture_chunk_validate_failure(CHUNK_VALIDATE_FAIL_IBI_IRQ_COUNT, 1U, (uint32_t)s_cm33_i3c_ibi_irq_count);
        EXP_LOG_ERROR("Expected a provoked IBI to complete on CM33.");
        dump_debug_state(base);
        return kStatus_Fail;
    }

    if (!s_ibi_seen)
    {
        capture_chunk_validate_failure(CHUNK_VALIDATE_FAIL_IBI_SEEN, 1U, 0U);
        EXP_LOG_ERROR("IBI notification was not observed.");
        dump_debug_state(base);
        return kStatus_Fail;
    }

    if (s_ibi_payload_count != 1U)
    {
        capture_chunk_validate_failure(
            CHUNK_VALIDATE_FAIL_IBI_PAYLOAD_COUNT, 1U, (uint32_t)s_ibi_payload_count);
        EXP_LOG_ERROR("Unexpected IBI payload count: %lu", (unsigned long)s_ibi_payload_count);
        dump_debug_state(base);
        return kStatus_Fail;
    }

    if (s_ibi_payload[0] != expectedIbiPayloadByte)
    {
        capture_chunk_validate_failure(
            CHUNK_VALIDATE_FAIL_IBI_PAYLOAD_BYTE, expectedIbiPayloadByte, s_ibi_payload[0]);
        EXP_LOG_ERROR("Unexpected IBI payload byte: 0x%02x", s_ibi_payload[0]);
        dump_debug_state(base);
        return kStatus_Fail;
    }

    if (!buffers_match(s_tx_buffer, s_rx_buffer, s_active_transfer_length))
    {
        capture_chunk_validate_failure(CHUNK_VALIDATE_FAIL_ROUNDTRIP_BUFFER,
                                       (s_active_transfer_length != 0U) ? s_tx_buffer[0] : 0U,
                                       (s_active_transfer_length != 0U) ? s_rx_buffer[0] : 0U);
        EXP_LOG_ERROR("Roundtrip payload mismatch after seed-tail write.");
        dump_debug_state(base);
        return kStatus_Fail;
    }

    return kStatus_Success;
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
    s_dma_probe_stage = DMA_PROBE_STAGE_CLEARED;
    s_dma_probe_result = (int32_t)kStatus_Success;

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

    SMARTDMA_Init(
        SMARTDMA_SRAM_ADDR, __smartdma_start__, (uint32_t)((uintptr_t)__smartdma_end__ - (uintptr_t)__smartdma_start__));
    NVIC_ClearPendingIRQ(SDMA_IRQn);
    NVIC_DisableIRQ(SDMA_IRQn);
    SMARTDMA_Reset();
    SMARTDMA_Boot(I3C_DMA_SEED_CHAIN_API_INDEX, &s_probe_param, 0);

    begin_transfer_led();

    result = ensure_master_idle(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }

    base->MCTRL = (base->MCTRL & ~I3C_MCTRL_IBIRESP_MASK) | I3C_MCTRL_IBIRESP(kI3C_IbiRespNack);

    NVIC_ClearPendingIRQ(I3C0_IRQn);
    NVIC_EnableIRQ(I3C0_IRQn);
    I3C_MasterEnableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    I3C_MasterEnableDMA(base, true, false, 1U);
    arm_seed_chain();

    result = I3C_MasterStart(base, kI3C_TypeI3CSdr, slaveAddr, kI3C_Write);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_START_ISSUED;

    result = wait_for_i3c_ctrl_done(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_START_CTRL_DONE;

    result = wait_for_smartdma_completion();
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_SMARTDMA_DONE;

    result = wait_for_i3c_complete(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_COMPLETE;

    result = I3C_MasterStop(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_STOP_SENT;

    result = wait_for_i3c_ctrl_done(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_STOP_CTRL_DONE;

    base->MCTRL = (base->MCTRL & ~I3C_MCTRL_IBIRESP_MASK) | I3C_MCTRL_IBIRESP(kI3C_IbiRespAckMandatory);

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base,
                               (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                   (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                   (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
    s_i3c_irq_status_latched = 0U;
    clear_protocol_trace();

    result = wait_for_ibi_notification(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_IBI_SEEN;

    I3C_MasterDisableInterrupts(base, I3C_PROTOCOL_IRQ_MASK);
    NVIC_DisableIRQ(I3C0_IRQn);
    NVIC_ClearPendingIRQ(I3C0_IRQn);

    result = finalize_post_ibi_bus(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_FINALIZED;

    result = ensure_master_idle(base);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_IDLE;

    result = run_roundtrip_read(base, slaveAddr);
    if (result != kStatus_Success)
    {
        goto exit;
    }
    s_dma_probe_stage = DMA_PROBE_STAGE_READ_DONE;

exit:
    s_dma_probe_result = (int32_t)result;
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

static status_t run_i3c_sdma_seed_tail_len_sweep(I3C_Type *base, uint8_t slaveAddr)
{
    status_t result;

    clear_length_sweep_snapshot();

    for (size_t caseIndex = 0U; caseIndex < ARRAY_SIZE(s_length_sweep_cases); caseIndex++)
    {
        const size_t logicalLength = s_length_sweep_cases[caseIndex];
        size_t offset = 0U;
        uint32_t chunkCount = 0U;
        uint32_t seedOnlyChunkCount = 0U;
        uint32_t cpuOneByteChunkCount = 0U;
        uint32_t totalSeedBytes = 0U;
        uint32_t totalSmartdmaBytes = 0U;
        uint32_t totalDataIrqs = 0U;
        uint32_t totalProtocolIrqs = 0U;
        uint32_t totalIbiIrqs = 0U;

        prepare_logical_payload(logicalLength);
        s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_CASE_START;
        s_length_sweep_snapshot.logicalLength = (uint32_t)logicalLength;
        s_length_sweep_snapshot.offset = 0U;
        s_length_sweep_snapshot.chunkLength = 0U;
        s_length_sweep_snapshot.chunkIndex = 0U;
        s_length_sweep_snapshot.totalSeedBytes = 0U;
        s_length_sweep_snapshot.totalSmartdmaBytes = 0U;
        s_length_sweep_snapshot.totalDataIrqs = 0U;
        s_length_sweep_snapshot.totalProtocolIrqs = 0U;
        s_length_sweep_snapshot.totalIbiIrqs = 0U;
        s_length_sweep_snapshot.result = 0;
        mirror_length_sweep_probe();

        if (logicalLength == 0U)
        {
            PRINTF("len-sweep logical_length=0 chunks=0\r\n");
            EXP_LOG_INFO("len-sweep logical_length=0 chunks=0");
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_CASE_OK;
            mirror_length_sweep_probe();
            continue;
        }

        while (offset < logicalLength)
        {
            const size_t chunkLength = choose_chunk_length(logicalLength - offset);

            clear_roundtrip_read_snapshot();
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_CHUNK_START;
            s_length_sweep_snapshot.offset = (uint32_t)offset;
            s_length_sweep_snapshot.chunkLength = (uint32_t)chunkLength;
            s_length_sweep_snapshot.chunkIndex = chunkCount;
            mirror_length_sweep_probe();
            load_transfer_window(&s_logical_tx_buffer[offset], chunkLength);
            if (s_active_chunk_mode == kLengthSweepChunkModeOneByteCpu)
            {
                result = run_i3c_one_byte_cpu_write_ibi_probe(base, slaveAddr);
            }
            else
            {
                result = run_i3c_dma_seed_tail_ibi_probe(base, slaveAddr);
            }
            if (result != kStatus_Success)
            {
                s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_ERROR_CHUNK_TRANSFER;
                s_length_sweep_snapshot.result = result;
                mirror_length_sweep_probe();
                EXP_LOG_ERROR("Length sweep chunk failed: logical=%u offset=%u chunk=%u result=%d",
                              (unsigned int)logicalLength,
                              (unsigned int)offset,
                              (unsigned int)chunkLength,
                              result);
                return result;
            }

            result = validate_chunk_result(base);
            if (result != kStatus_Success)
            {
                s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_ERROR_CHUNK_VALIDATE;
                s_length_sweep_snapshot.result = result;
                mirror_length_sweep_probe();
                EXP_LOG_ERROR("Length sweep chunk validation failed: logical=%u offset=%u chunk=%u result=%d",
                              (unsigned int)logicalLength,
                              (unsigned int)offset,
                              (unsigned int)chunkLength,
                              result);
                return result;
            }

            memcpy(&s_logical_rx_buffer[offset], s_rx_buffer, chunkLength);
            chunkCount++;
            if (chunkLength == 1U)
            {
                seedOnlyChunkCount++;
                cpuOneByteChunkCount++;
            }
            totalSeedBytes += s_probe_param.dmaSeedBytes;
            totalSmartdmaBytes += s_probe_param.smartdmaBytes;
            totalDataIrqs += s_cm33_i3c_data_irq_count;
            totalProtocolIrqs += s_cm33_i3c_protocol_irq_count;
            totalIbiIrqs += s_cm33_i3c_ibi_irq_count;
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_CHUNK_OK;
            s_length_sweep_snapshot.totalSeedBytes = totalSeedBytes;
            s_length_sweep_snapshot.totalSmartdmaBytes = totalSmartdmaBytes;
            s_length_sweep_snapshot.totalDataIrqs = totalDataIrqs;
            s_length_sweep_snapshot.totalProtocolIrqs = totalProtocolIrqs;
            s_length_sweep_snapshot.totalIbiIrqs = totalIbiIrqs;
            s_length_sweep_snapshot.result = 0;
            mirror_length_sweep_probe();
            offset += chunkLength;
        }

        if (!buffers_match(s_logical_tx_buffer, s_logical_rx_buffer, logicalLength))
        {
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_ERROR_LOGICAL_MISMATCH;
            s_length_sweep_snapshot.result = kStatus_Fail;
            mirror_length_sweep_probe();
            EXP_LOG_ERROR("Logical payload mismatch after segmented sweep length=%u", (unsigned int)logicalLength);
            return kStatus_Fail;
        }

        if (totalSeedBytes != chunkCount)
        {
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_SEED;
            s_length_sweep_snapshot.result = kStatus_Fail;
            mirror_length_sweep_probe();
            EXP_LOG_ERROR("Aggregate DMA seed byte mismatch: length=%u expected=%lu actual=%lu",
                          (unsigned int)logicalLength,
                          (unsigned long)chunkCount,
                          (unsigned long)totalSeedBytes);
            return kStatus_Fail;
        }

        if (totalSmartdmaBytes != (logicalLength - seedOnlyChunkCount))
        {
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_SMARTDMA;
            s_length_sweep_snapshot.result = kStatus_Fail;
            mirror_length_sweep_probe();
            EXP_LOG_ERROR("Aggregate SmartDMA byte mismatch: length=%u expected=%u actual=%lu",
                          (unsigned int)logicalLength,
                          (unsigned int)(logicalLength - seedOnlyChunkCount),
                          (unsigned long)totalSmartdmaBytes);
            return kStatus_Fail;
        }

        if (totalDataIrqs > chunkCount)
        {
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_DATA_IRQ;
            s_length_sweep_snapshot.result = kStatus_Fail;
            mirror_length_sweep_probe();
            EXP_LOG_ERROR("Aggregate CM33 data-ready IRQ count exceeded chunk count: length=%u chunks=%lu data_irqs=%lu",
                          (unsigned int)logicalLength,
                          (unsigned long)chunkCount,
                          (unsigned long)totalDataIrqs);
            return kStatus_Fail;
        }

        if ((totalIbiIrqs == 0U) || ((chunkCount != cpuOneByteChunkCount) && (totalProtocolIrqs == 0U)))
        {
            s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_ERROR_AGGREGATE_PROTOCOL_IBI;
            s_length_sweep_snapshot.result = kStatus_Fail;
            mirror_length_sweep_probe();
            EXP_LOG_ERROR("Aggregate protocol/IBI IRQ visibility missing: length=%u protocol=%lu ibi=%lu",
                          (unsigned int)logicalLength,
                          (unsigned long)totalProtocolIrqs,
                          (unsigned long)totalIbiIrqs);
            return kStatus_Fail;
        }

        PRINTF("len-sweep logical_length=%u chunks=%lu seed_bytes=%lu smartdma_bytes=%lu data_irqs=%lu protocol_irqs=%lu ibi_irqs=%lu\r\n",
               (unsigned int)logicalLength,
               (unsigned long)chunkCount,
               (unsigned long)totalSeedBytes,
               (unsigned long)totalSmartdmaBytes,
               (unsigned long)totalDataIrqs,
               (unsigned long)totalProtocolIrqs,
               (unsigned long)totalIbiIrqs);
        EXP_LOG_INFO("len-sweep logical_length=%u chunks=%lu seed_bytes=%lu smartdma_bytes=%lu data_irqs=%lu protocol_irqs=%lu ibi_irqs=%lu",
                     (unsigned int)logicalLength,
                     (unsigned long)chunkCount,
                     (unsigned long)totalSeedBytes,
                     (unsigned long)totalSmartdmaBytes,
                     (unsigned long)totalDataIrqs,
                     (unsigned long)totalProtocolIrqs,
                     (unsigned long)totalIbiIrqs);
        s_length_sweep_snapshot.stage = LENGTH_SWEEP_STAGE_CASE_OK;
        mirror_length_sweep_probe();
    }

    return kStatus_Success;
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

    PRINTF("\r\nI3C SDMA seed tail length sweep -- master.\r\n");
    EXP_LOG_INFO("I3C SDMA seed tail length sweep -- master.");

    POWER_DisablePD(kPDRUNCFG_APD_SMARTDMA_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_SMARTDMA_SRAM);
    POWER_ApplyPD();

    load_transfer_window(NULL, I3C_DMA_SEED_CHAIN_LENGTH);
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
    EXP_LOG_INFO("Starting I3C SDMA seed tail length sweep.");

    result = run_i3c_sdma_seed_tail_len_sweep(EXAMPLE_MASTER, slaveAddr);
    if (result != kStatus_Success)
    {
        EXP_LOG_ERROR("I3C SDMA seed tail length sweep failed: %d", result);
        dump_debug_state(EXAMPLE_MASTER);
        set_failure_led();
        return -1;
    }

    dump_probe_counters();
    EXP_LOG_INFO("I3C SDMA seed tail length sweep successful.");
    PRINTF("I3C SDMA seed tail length sweep successful.\r\n");
    set_success_led();
    return 0;
}
