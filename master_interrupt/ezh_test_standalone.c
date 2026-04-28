/*
 * Standalone master_interrupt app.
 */

#define EXPERIMENT_USE_SMARTDMA 1
#define EZH_ROUNDTRIP_DATA_LENGTH 8U
#define EXPERIMENT_STARTUP_WAIT 5000000U
#define EXPERIMENT_POST_DAA_CLEANUP 1U
#define EXPERIMENT_POST_DAA_WAIT 0U
#define EXPERIMENT_I3C_PP_BAUDRATE 4000000U
#define EXPERIMENT_ENABLE_SEMIHOST_LOG 0
#define EXPERIMENT_FORCE_BLOCKING_WRITE 0
#define EXPERIMENT_FORCE_BLOCKING_READ 0
#define EXPERIMENT_FORCE_MANUAL_POST_IBI_READ 1
#define EXPERIMENT_FORCE_BLOCKING_POST_IBI_READ 1


#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_i3c_smartdma.h"
#include "fsl_smartdma.h"
#include "fsl_power.h"
#include "fsl_inputmux.h"
#include "fsl_i3c.h"
#include "experiment_led.h"

#if defined(EXPERIMENT_USE_PW_LOG)
#define PW_LOG_MODULE_NAME "archive-master"
#include "pw_log/log.h"
#define EXP_LOG_INFO(...) PW_LOG_INFO(__VA_ARGS__)
#define EXP_LOG_ERROR(...) PW_LOG_ERROR(__VA_ARGS__)
#elif EXPERIMENT_ENABLE_SEMIHOST_LOG
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

    prefix_length = snprintf(buffer, sizeof(buffer), "%s archive-master ", level);
    if ((prefix_length < 0) || (prefix_length >= (int)sizeof(buffer)))
    {
        semihost_write0("ERR archive-master log prefix overflow\n");
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

void keep_smartdma_api_alive(void);

static void i3c_master_ibi_callback(I3C_Type *base,
                                    i3c_master_smartdma_handle_t *handle,
                                    i3c_ibi_type_t ibiType,
                                    i3c_ibi_state_t ibiState);
static void i3c_master_callback(I3C_Type *base,
                                i3c_master_smartdma_handle_t *handle,
                                status_t status,
                                void *userData);
static void post_ibi_read_ibi_callback(I3C_Type *base,
                                       i3c_master_smartdma_handle_t *handle,
                                       i3c_ibi_type_t ibiType,
                                       i3c_ibi_state_t ibiState);
static void post_ibi_read_complete_callback(I3C_Type *base,
                                            i3c_master_smartdma_handle_t *handle,
                                            status_t status,
                                            void *userData);
static void clear_ibi_state(void);
static void configure_slave_ibi_rule(I3C_Type *base, uint8_t slaveAddr);
static void service_ibi_protocol_irq(I3C_Type *base, uint32_t pending);
static status_t wait_for_ibi_notification(I3C_Type *base);
static status_t finalize_post_ibi_bus(I3C_Type *base);

#define I3C_DMA_TX_CHANNEL 25U
#define SMART_DMA_TRIGGER_CHANNEL 0U
#define I3C_PROTOCOL_IRQ_MASK ((uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag | \
                               (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag | \
                               (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag)
#ifndef EZH_ROUNDTRIP_DATA_LENGTH
#define EZH_ROUNDTRIP_DATA_LENGTH 255U
#endif

#ifndef EXPERIMENT_COMPARE_LENGTH
#define EXPERIMENT_COMPARE_LENGTH EZH_ROUNDTRIP_DATA_LENGTH
#endif
#define I3C_TIME_OUT_INDEX 100000000U

#ifndef WAIT_TIME
#define WAIT_TIME 1000U
#endif

#ifndef EXPERIMENT_PRE_READ_DELAY_US
#define EXPERIMENT_PRE_READ_DELAY_US 50000U
#endif

#ifndef EXPERIMENT_STARTUP_WAIT
#define EXPERIMENT_STARTUP_WAIT 0U
#endif

#ifndef EXPERIMENT_POST_DAA_WAIT
#define EXPERIMENT_POST_DAA_WAIT 0U
#endif

#ifndef EXPERIMENT_POST_DAA_CLEANUP
#define EXPERIMENT_POST_DAA_CLEANUP 1U
#endif

#ifndef EXPERIMENT_I2C_BAUDRATE
#define EXPERIMENT_I2C_BAUDRATE EXAMPLE_I2C_BAUDRATE
#endif

#ifndef EXPERIMENT_I3C_OD_BAUDRATE
#define EXPERIMENT_I3C_OD_BAUDRATE EXAMPLE_I3C_OD_BAUDRATE
#endif

#ifndef EXPERIMENT_I3C_PP_BAUDRATE
#define EXPERIMENT_I3C_PP_BAUDRATE EXAMPLE_I3C_PP_BAUDRATE
#endif

#define EXPERIMENT_LED_BOOT_PULSE_COUNT 2U
#define EXPERIMENT_LED_BOOT_PULSE_US 100000U
#define EXPERIMENT_LED_SUCCESS_PULSE_COUNT 3U
#define EXPERIMENT_LED_SUCCESS_PULSE_US 120000U
#define EXPERIMENT_LED_FINAL_HOLD_US 2000000U

extern uint8_t __smartdma_start__[];
extern uint8_t __smartdma_end__[];

__attribute__((aligned(4))) static uint8_t ezh_data_buffer[EZH_ROUNDTRIP_DATA_LENGTH];
__attribute__((aligned(4))) static uint8_t ezh_data_buffer_rx[EZH_ROUNDTRIP_DATA_LENGTH];

static uint8_t g_master_ibiBuff[10];
static i3c_master_smartdma_handle_t g_i3c_m_handle;
static uint8_t g_postIbiReadIbiBuff[10];
static i3c_master_smartdma_handle_t g_postIbiReadHandle;
static const i3c_master_smartdma_callback_t masterCallback = {
    .slave2Master = NULL,
    .ibiCallback = i3c_master_ibi_callback,
    .transferComplete = i3c_master_callback,
};
static const i3c_master_smartdma_callback_t postIbiReadCallback = {
    .slave2Master = NULL,
    .ibiCallback = post_ibi_read_ibi_callback,
    .transferComplete = post_ibi_read_complete_callback,
};
static volatile bool g_masterCompletionFlag = false;
static volatile bool g_ibiWonFlag = false;
static volatile bool g_smartdmaTransferActive = false;
static volatile bool g_postIbiReadActive = false;
static volatile bool g_postIbiReadComplete = false;
static volatile bool g_postIbiReadIbiWon = false;
static volatile bool g_ibiSeen = false;
static volatile bool g_ibiActive = false;
static volatile bool g_ibiAckEmitted = false;
static volatile uint32_t g_ibiPayloadCount = 0U;
static volatile status_t g_completionStatus = kStatus_Success;
static volatile status_t g_postIbiReadStatus = kStatus_Success;

enum
{
    kMasterHandleIdleState = 0U,
    kMasterFailureStageNone = 0U,
    kMasterFailureStageRstdaa = 1U,
    kMasterFailureStageDaa = 2U,
    kMasterFailureStageWrite = 3U,
    kMasterFailureStageIbiWait = 4U,
    kMasterFailureStageIbiFinalize = 5U,
    kMasterFailureStageRead = 6U,
    kMasterFailureStageCompare = 7U,
};

enum
{
    kPostIbiManualReadStageNone = 0U,
    kPostIbiManualReadStageStartIssued = 1U,
    kPostIbiManualReadStageCtrlDone = 2U,
    kPostIbiManualReadStageDrainLoop = 3U,
    kPostIbiManualReadStageStopSent = 4U,
    kPostIbiManualReadStageErrorStart = 0x101U,
    kPostIbiManualReadStageErrorCtrlDone = 0x102U,
    kPostIbiManualReadStageErrorLoop = 0x103U,
};

typedef struct master_failure_snapshot
{
    uint32_t magic;
    uint32_t stage;
    uint32_t result;
    uint32_t completionStatus;
    uint32_t pendingMask;
    uint32_t statusMask;
    uint32_t errwarnMask;
    uint32_t nvicPending;
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
    uint32_t ibiSeen;
    uint32_t ibiActive;
    uint32_t ibiAckEmitted;
    uint32_t ibiPayloadCount;
    uint32_t prefixLength;
    uint8_t prefix[8];
} master_failure_snapshot_t;

static volatile master_failure_snapshot_t g_masterFailureSnapshot;

#define MASTER_FAILURE_SNAPSHOT_MAGIC 0x4D465331U

enum
{
    kPostIbiHandoffStageBeforeFinalize = 1U,
    kPostIbiHandoffStageAfterFinalize = 2U,
    kPostIbiHandoffStageBeforeReadStart = 3U,
};

typedef struct post_ibi_handoff_sample
{
    uint32_t stage;
    int32_t result;
    uint32_t mctrl;
    uint32_t status;
    uint32_t errStatus;
    uint32_t mdatactrl;
    uint32_t pending;
    uint32_t masterState;
    uint32_t ibiSeen;
    uint32_t ibiActive;
    uint32_t ibiAckEmitted;
} post_ibi_handoff_sample_t;

typedef struct post_ibi_handoff_snapshot
{
    uint32_t magic;
    post_ibi_handoff_sample_t beforeFinalize;
    post_ibi_handoff_sample_t afterFinalize;
    post_ibi_handoff_sample_t beforeReadStart;
} post_ibi_handoff_snapshot_t;

typedef struct post_ibi_manual_read_snapshot
{
    uint32_t stage;
    int32_t result;
    uint32_t status;
    uint32_t errStatus;
    uint32_t mdatactrl;
    uint32_t remaining;
} post_ibi_manual_read_snapshot_t;

static post_ibi_handoff_snapshot_t g_postIbiHandoffSnapshot;
static volatile post_ibi_manual_read_snapshot_t g_postIbiManualReadSnapshot;

#define POST_IBI_HANDOFF_SNAPSHOT_MAGIC 0x50494248U

static void init_status_led(void)
{
    EXP_LED_Init();
    EXP_LED_Blink(false, false, true, EXPERIMENT_LED_BOOT_PULSE_COUNT, EXPERIMENT_LED_BOOT_PULSE_US);
    EXP_LED_Set(false, false, true);
}

static void hold_status_led(void)
{
    SDK_DelayAtLeastUs(EXPERIMENT_LED_FINAL_HOLD_US, SystemCoreClock);
}

static void set_success_led(void)
{
    EXP_LED_Blink(false, true, false, EXPERIMENT_LED_SUCCESS_PULSE_COUNT, EXPERIMENT_LED_SUCCESS_PULSE_US);
    EXP_LED_Set(false, true, false);
    hold_status_led();
}

static void set_failure_led(void)
{
    EXP_LED_Set(true, false, false);
    hold_status_led();
}

static void reset_master_failure_snapshot(void)
{
    volatile uint32_t *words = (volatile uint32_t *)&g_masterFailureSnapshot;

    for (uint32_t index = 0U; index < (sizeof(g_masterFailureSnapshot) / sizeof(uint32_t)); index++)
    {
        words[index] = 0U;
    }
}

static void clear_ibi_state(void)
{
    g_ibiSeen = false;
    g_ibiActive = false;
    g_ibiAckEmitted = false;
    g_ibiPayloadCount = 0U;
    memset(g_master_ibiBuff, 0, sizeof(g_master_ibiBuff));
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
    sample->ibiSeen = g_ibiSeen ? 1U : 0U;
    sample->ibiActive = g_ibiActive ? 1U : 0U;
    sample->ibiAckEmitted = g_ibiAckEmitted ? 1U : 0U;
}

static void capture_post_ibi_handoff_snapshot(I3C_Type *base, uint32_t stage, status_t result)
{
    g_postIbiHandoffSnapshot.magic = POST_IBI_HANDOFF_SNAPSHOT_MAGIC;

    if (stage == kPostIbiHandoffStageBeforeFinalize)
    {
        capture_post_ibi_handoff_sample(&g_postIbiHandoffSnapshot.beforeFinalize, base, stage, result);
    }
    else if (stage == kPostIbiHandoffStageAfterFinalize)
    {
        capture_post_ibi_handoff_sample(&g_postIbiHandoffSnapshot.afterFinalize, base, stage, result);
    }
    else if (stage == kPostIbiHandoffStageBeforeReadStart)
    {
        capture_post_ibi_handoff_sample(&g_postIbiHandoffSnapshot.beforeReadStart, base, stage, result);
    }
}

static void capture_post_ibi_manual_read_snapshot(I3C_Type *base, uint32_t stage, status_t result, size_t remaining)
{
    g_postIbiManualReadSnapshot.stage = stage;
    g_postIbiManualReadSnapshot.result = (int32_t)result;
    g_postIbiManualReadSnapshot.status = I3C_MasterGetStatusFlags(base);
    g_postIbiManualReadSnapshot.errStatus = I3C_MasterGetErrorStatusFlags(base);
    g_postIbiManualReadSnapshot.mdatactrl = base->MDATACTRL;
    g_postIbiManualReadSnapshot.remaining = (uint32_t)remaining;
}

#ifndef EXPERIMENT_USE_SMARTDMA
#define EXPERIMENT_USE_SMARTDMA 1
#endif

static status_t wait_for_transfer_completion(void)
{
    volatile uint32_t timeout = 0U;

    while ((!g_ibiWonFlag) && (!g_masterCompletionFlag) && (g_completionStatus == kStatus_Success) &&
           (++timeout < I3C_TIME_OUT_INDEX))
    {
        __NOP();
    }

    if (timeout == I3C_TIME_OUT_INDEX)
    {
        return kStatus_Timeout;
    }

    return g_completionStatus;
}

static status_t run_transfer_blocking(i3c_master_transfer_t *transfer)
{
    status_t result;

#if !EXPERIMENT_USE_SMARTDMA
    return I3C_MasterTransferBlocking(EXAMPLE_MASTER, transfer);
#else
    g_masterCompletionFlag = false;
    g_ibiWonFlag = false;
    g_smartdmaTransferActive = true;
    g_completionStatus = kStatus_Success;

    result = I3C_MasterTransferSmartDMA(EXAMPLE_MASTER, &g_i3c_m_handle, transfer);
    if (result != kStatus_Success)
    {
        g_smartdmaTransferActive = false;
        return result;
    }

    result = wait_for_transfer_completion();
    g_masterCompletionFlag = false;
    g_ibiWonFlag = false;
    return result;
#endif
}

static status_t wait_for_post_ibi_read_completion(void)
{
    volatile uint32_t timeout = 0U;

    while ((!g_postIbiReadIbiWon) && (!g_postIbiReadComplete) && (g_postIbiReadStatus == kStatus_Success) &&
           (++timeout < I3C_TIME_OUT_INDEX))
    {
        __NOP();
    }

    if (timeout == I3C_TIME_OUT_INDEX)
    {
        return kStatus_Timeout;
    }

    return g_postIbiReadStatus;
}

static status_t run_read_transfer(i3c_master_transfer_t *transfer)
{
#if EXPERIMENT_FORCE_BLOCKING_READ
    return I3C_MasterTransferBlocking(EXAMPLE_MASTER, transfer);
#else
    return run_transfer_blocking(transfer);
#endif
}

static status_t run_write_transfer(i3c_master_transfer_t *transfer)
{
#if EXPERIMENT_FORCE_BLOCKING_WRITE
    return I3C_MasterTransferBlocking(EXAMPLE_MASTER, transfer);
#else
    return run_transfer_blocking(transfer);
#endif
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

static void prepare_post_ibi_read_controller(I3C_Type *base)
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

    SMARTDMA_Reset();
    memset(&g_postIbiReadHandle, 0, sizeof(g_postIbiReadHandle));
    memset(g_postIbiReadIbiBuff, 0, sizeof(g_postIbiReadIbiBuff));
    g_postIbiReadActive = false;
    g_postIbiReadComplete = false;
    g_postIbiReadIbiWon = false;
    g_postIbiReadStatus = kStatus_Success;
    I3C_MasterTransferCreateHandleSmartDMA(base, &g_postIbiReadHandle, &postIbiReadCallback, NULL);
    NVIC_ClearPendingIRQ(I3C0_IRQn);
    NVIC_ClearPendingIRQ(SDMA_IRQn);
}

static status_t run_post_ibi_read_manual(I3C_Type *base, uint8_t slaveAddr, uint8_t *buffer, size_t bufferSize)
{
    volatile uint32_t timeout = 0U;
    uint8_t *nextByte = buffer;
    size_t remaining = bufferSize;
    status_t result;

    capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageNone, kStatus_Success, remaining);

    result = I3C_MasterStart(base, kI3C_TypeI3CSdr, slaveAddr, kI3C_Read);
    if (result != kStatus_Success)
    {
        capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageErrorStart, result, remaining);
        return result;
    }

    capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageStartIssued, result, remaining);

    result = I3C_MasterWaitForCtrlDone(base, false);
    if (result != kStatus_Success)
    {
        capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageErrorCtrlDone, result, remaining);
        return result;
    }

    capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageCtrlDone, result, remaining);

    while (++timeout < I3C_TIME_OUT_INDEX)
    {
        uint32_t rxCount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT;
        uint32_t errStatus;

        capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageDrainLoop, kStatus_Success, remaining);

        while ((remaining != 0U) && (rxCount != 0U))
        {
            *nextByte++ = (uint8_t)(base->MRDATAB & I3C_MRDATAB_VALUE_MASK);
            remaining--;
            rxCount--;
        }

        if (remaining == 0U)
        {
            result = I3C_MasterStop(base);
            capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageStopSent, result, remaining);
            if ((result == kStatus_Success) || (result == kStatus_I3C_InvalidReq))
            {
                return kStatus_Success;
            }

            return result;
        }

        errStatus = I3C_MasterGetErrorStatusFlags(base);
        result = I3C_MasterCheckAndClearError(base, errStatus);
        if (result != kStatus_Success)
        {
            capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageErrorLoop, result, remaining);
            return result;
        }
    }

    capture_post_ibi_manual_read_snapshot(base, kPostIbiManualReadStageErrorLoop, kStatus_I3C_Timeout, remaining);
    return kStatus_I3C_Timeout;
}

static status_t run_post_ibi_read(I3C_Type *base, uint8_t slaveAddr, uint8_t *buffer, size_t bufferSize)
{
    i3c_master_transfer_t masterXfer;
    status_t result;

    capture_post_ibi_handoff_snapshot(base, kPostIbiHandoffStageBeforeReadStart, kStatus_Success);

    I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
    I3C_MasterClearStatusFlags(base,
                               (uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterControlDoneFlag |
                                   (uint32_t)kI3C_MasterCompleteFlag | (uint32_t)kI3C_MasterArbitrationWonFlag |
                                   (uint32_t)kI3C_MasterSlave2MasterFlag | (uint32_t)kI3C_MasterErrorFlag);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    memset(buffer, 0, bufferSize);

#if EXPERIMENT_FORCE_MANUAL_POST_IBI_READ
    return run_post_ibi_read_manual(base, slaveAddr, buffer, bufferSize);
#endif

    prepare_post_ibi_read_controller(base);

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = slaveAddr;
    masterXfer.direction = kI3C_Read;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;
    masterXfer.data = buffer;
    masterXfer.dataSize = bufferSize;

#if EXPERIMENT_FORCE_BLOCKING_POST_IBI_READ
    return I3C_MasterTransferBlocking(base, &masterXfer);
#endif

    g_postIbiReadActive = true;
    result = I3C_MasterTransferSmartDMA(base, &g_postIbiReadHandle, &masterXfer);
    if (result != kStatus_Success)
    {
        g_postIbiReadActive = false;
        return result;
    }

    result = wait_for_post_ibi_read_completion();
    if (result == kStatus_Timeout)
    {
        I3C_MasterTransferAbortSmartDMA(base, &g_postIbiReadHandle);
        g_postIbiReadActive = false;
    }

    return result;
}

static void log_buffer_prefix(const char *label, const uint8_t *buffer, uint32_t length)
{
    char line[8U * 5U + 1U];
    uint32_t dump_length = length;

    if (dump_length > 32U)
    {
        dump_length = 32U;
    }

    EXP_LOG_INFO("%s", label);
    for (uint32_t offset = 0U; offset < dump_length; offset += 8U)
    {
        uint32_t used = 0U;
        uint32_t chunk = dump_length - offset;
        if (chunk > 8U)
        {
            chunk = 8U;
        }

        for (uint32_t index = 0U; index < chunk; index++)
        {
            used += (uint32_t)snprintf(line + used,
                                       sizeof(line) - used,
                                       "0x%02X ",
                                       buffer[offset + index]);
        }

        if (used > 0U)
        {
            line[used - 1U] = '\0';
        }
        else
        {
            line[0] = '\0';
        }

        EXP_LOG_INFO("%s", line);
    }
}

static bool buffers_match(const uint8_t *expected, const uint8_t *actual, uint32_t length)
{
    for (uint32_t index = 0U; index < length; index++)
    {
        if (expected[index] != actual[index])
        {
            return false;
        }
    }

    return true;
}

static void capture_master_failure_snapshot(uint32_t stage, status_t result, const uint8_t *buffer, uint32_t length)
{
    const i3c_master_smartdma_handle_t *snapshotHandle =
        ((stage == kMasterFailureStageRead) || (stage == kMasterFailureStageCompare)) ? &g_postIbiReadHandle :
                                                                                         &g_i3c_m_handle;
    status_t completionStatus =
        ((stage == kMasterFailureStageRead) || (stage == kMasterFailureStageCompare)) ? g_postIbiReadStatus :
                                                                                         g_completionStatus;
    uint32_t prefixLength = length;

    if (prefixLength > sizeof(g_masterFailureSnapshot.prefix))
    {
        prefixLength = sizeof(g_masterFailureSnapshot.prefix);
    }

    reset_master_failure_snapshot();
    g_masterFailureSnapshot.magic = MASTER_FAILURE_SNAPSHOT_MAGIC;
    g_masterFailureSnapshot.stage = stage;
    g_masterFailureSnapshot.result = (uint32_t)result;
    g_masterFailureSnapshot.completionStatus = (uint32_t)completionStatus;
    g_masterFailureSnapshot.pendingMask = (uint32_t)I3C_MasterGetPendingInterrupts(EXAMPLE_MASTER);
    g_masterFailureSnapshot.statusMask = (uint32_t)EXAMPLE_MASTER->MSTATUS;
    g_masterFailureSnapshot.errwarnMask = (uint32_t)EXAMPLE_MASTER->MERRWARN;
    g_masterFailureSnapshot.nvicPending = NVIC_GetPendingIRQ(I3C0_IRQn) != 0U ? 1U : 0U;
    g_masterFailureSnapshot.handleState = (uint32_t)snapshotHandle->state;
    g_masterFailureSnapshot.transferCount = snapshotHandle->transferCount;
    g_masterFailureSnapshot.smartdmaWindowIrqCount = snapshotHandle->smartdmaWindowIrqCount;
    g_masterFailureSnapshot.smartdmaFifoReadyBounceCount = snapshotHandle->smartdmaFifoReadyBounceCount;
    g_masterFailureSnapshot.smartdmaProtocolBounceCount = snapshotHandle->smartdmaProtocolBounceCount;
    g_masterFailureSnapshot.smartdmaMailboxProtocolCount = snapshotHandle->smartdmaMailboxProtocolCount;
    g_masterFailureSnapshot.smartdmaWindowPendingMask = snapshotHandle->smartdmaWindowPendingMask;
    g_masterFailureSnapshot.smartdmaWindowFifoMask = snapshotHandle->smartdmaWindowFifoMask;
    g_masterFailureSnapshot.smartdmaWindowProtocolMask = snapshotHandle->smartdmaWindowProtocolMask;
    g_masterFailureSnapshot.smartdmaMailbox = snapshotHandle->smartdmaMailbox;
    g_masterFailureSnapshot.ibiSeen = g_ibiSeen ? 1U : 0U;
    g_masterFailureSnapshot.ibiActive = g_ibiActive ? 1U : 0U;
    g_masterFailureSnapshot.ibiAckEmitted = g_ibiAckEmitted ? 1U : 0U;
    g_masterFailureSnapshot.ibiPayloadCount = g_ibiPayloadCount;
    g_masterFailureSnapshot.prefixLength = prefixLength;

    for (uint32_t index = 0U; index < prefixLength; index++)
    {
        g_masterFailureSnapshot.prefix[index] = buffer[index];
    }
}

static void log_read_failure_context(void)
{
    const i3c_master_smartdma_handle_t *readHandle = &g_postIbiReadHandle;

    EXP_LOG_INFO("Read failure handle state=%u transferCount=%lu completionStatus=%ld",
                 (unsigned int)readHandle->state,
                 (unsigned long)readHandle->transferCount,
                 (long)g_postIbiReadStatus);
    EXP_LOG_INFO("Read failure pending=0x%08lx status=0x%08lx errwarn=0x%08lx nvicPending=%lu",
                 (unsigned long)I3C_MasterGetPendingInterrupts(EXAMPLE_MASTER),
                 (unsigned long)EXAMPLE_MASTER->MSTATUS,
                 (unsigned long)EXAMPLE_MASTER->MERRWARN,
                 (unsigned long)(NVIC_GetPendingIRQ(I3C0_IRQn) != 0U ? 1U : 0U));
    EXP_LOG_INFO("Read failure smartdma window irq=%lu fifoBounce=%lu protocolBounce=%lu mailboxProtocol=%lu",
                 (unsigned long)readHandle->smartdmaWindowIrqCount,
                 (unsigned long)readHandle->smartdmaFifoReadyBounceCount,
                 (unsigned long)readHandle->smartdmaProtocolBounceCount,
                 (unsigned long)readHandle->smartdmaMailboxProtocolCount);
    EXP_LOG_INFO("Read failure smartdma pending=0x%08lx fifo=0x%08lx protocol=0x%08lx mailbox=0x%08lx",
                 (unsigned long)readHandle->smartdmaWindowPendingMask,
                 (unsigned long)readHandle->smartdmaWindowFifoMask,
                 (unsigned long)readHandle->smartdmaWindowProtocolMask,
                 (unsigned long)readHandle->smartdmaMailbox);
    EXP_LOG_INFO("Read failure ibi seen=%lu active=%lu ack=%lu payloadCount=%lu",
                 (unsigned long)(g_ibiSeen ? 1U : 0U),
                 (unsigned long)(g_ibiActive ? 1U : 0U),
                 (unsigned long)(g_ibiAckEmitted ? 1U : 0U),
                 (unsigned long)g_ibiPayloadCount);
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

    while ((!g_ibiSeen) && (++timeout < I3C_TIME_OUT_INDEX))
    {
        if (I3C_MasterGetErrorStatusFlags(base) != 0U)
        {
            I3C_MasterClearErrorStatusFlags(base, I3C_MasterGetErrorStatusFlags(base));
            return kStatus_Fail;
        }
        __NOP();
    }

    if (!g_ibiSeen)
    {
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

static status_t finalize_post_ibi_bus(I3C_Type *base)
{
    capture_post_ibi_handoff_snapshot(base, kPostIbiHandoffStageBeforeFinalize, kStatus_Success);

    status_t result = I3C_MasterStop(base);

    if (result == kStatus_Success)
    {
        capture_post_ibi_handoff_snapshot(base, kPostIbiHandoffStageAfterFinalize, result);
        return kStatus_Success;
    }

    if (result != kStatus_I3C_InvalidReq)
    {
        capture_post_ibi_handoff_snapshot(base, kPostIbiHandoffStageAfterFinalize, result);
        return result;
    }

    if (I3C_MasterGetState(base) == kI3C_MasterStateIdle)
    {
        capture_post_ibi_handoff_snapshot(base, kPostIbiHandoffStageAfterFinalize, kStatus_Success);
        return kStatus_Success;
    }

    I3C_MasterEmitRequest(base, kI3C_RequestForceExit);
    result = I3C_MasterWaitForCtrlDone(base, true);
    if ((result == kStatus_Success) && (I3C_MasterGetState(base) == kI3C_MasterStateIdle))
    {
        capture_post_ibi_handoff_snapshot(base, kPostIbiHandoffStageAfterFinalize, result);
        return kStatus_Success;
    }

    capture_post_ibi_handoff_snapshot(base, kPostIbiHandoffStageAfterFinalize, result);
    return result;
}

static void i3c_master_ibi_callback(I3C_Type *base,
                                    i3c_master_smartdma_handle_t *handle,
                                    i3c_ibi_type_t ibiType,
                                    i3c_ibi_state_t ibiState)
{
    (void)base;

    switch (ibiType)
    {
        case kI3C_IbiNormal:
            if (ibiState == kI3C_IbiDataBuffNeed)
            {
                handle->ibiBuff = g_master_ibiBuff;
            }
            break;

        default:
            assert(false);
            break;
    }
}

static void i3c_master_callback(I3C_Type *base,
                                i3c_master_smartdma_handle_t *handle,
                                status_t status,
                                void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;

    if (status == kStatus_Success)
    {
        g_masterCompletionFlag = true;
    }

    if (status == kStatus_I3C_IBIWon)
    {
        g_ibiWonFlag = true;
    }

    g_smartdmaTransferActive = false;
    g_completionStatus = status;
}

static void post_ibi_read_ibi_callback(I3C_Type *base,
                                       i3c_master_smartdma_handle_t *handle,
                                       i3c_ibi_type_t ibiType,
                                       i3c_ibi_state_t ibiState)
{
    (void)base;

    if ((ibiType == kI3C_IbiNormal) && (ibiState == kI3C_IbiDataBuffNeed))
    {
        handle->ibiBuff = g_postIbiReadIbiBuff;
    }
}

static void post_ibi_read_complete_callback(I3C_Type *base,
                                            i3c_master_smartdma_handle_t *handle,
                                            status_t status,
                                            void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;

    if (status == kStatus_Success)
    {
        g_postIbiReadComplete = true;
    }

    if (status == kStatus_I3C_IBIWon)
    {
        g_postIbiReadIbiWon = true;
    }

    g_postIbiReadActive = false;
    g_postIbiReadStatus = status;
}

static void service_ibi_protocol_irq(I3C_Type *base, uint32_t pending)
{
    i3c_master_state_t masterState = I3C_MasterGetState(base);
    uint32_t rxCount;

    if ((pending & ((uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterArbitrationWonFlag)) != 0U)
    {
        g_ibiActive = true;
    }

    if ((pending & (uint32_t)kI3C_MasterSlaveStartFlag) != 0U)
    {
        I3C_MasterEmitRequest(base, kI3C_RequestAutoIbi);
        masterState = I3C_MasterGetState(base);
    }

    if (masterState == kI3C_MasterStateIbiAck)
    {
        I3C_MasterEmitIBIResponse(base, kI3C_IbiRespAckMandatory);
        g_ibiAckEmitted = true;
        masterState = I3C_MasterGetState(base);
    }

    rxCount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT;
    while ((rxCount != 0U) && (g_ibiPayloadCount < sizeof(g_master_ibiBuff)))
    {
        g_master_ibiBuff[g_ibiPayloadCount++] = (uint8_t)(base->MRDATAB & 0xFFU);
        rxCount--;
    }

    if (((pending & (uint32_t)kI3C_MasterCompleteFlag) != 0U) &&
        (g_ibiActive || g_ibiAckEmitted || (masterState == kI3C_MasterStateIbiAck) ||
         (masterState == kI3C_MasterStateIbiRcv)))
    {
        g_ibiSeen = true;
        g_ibiActive = false;
        g_ibiAckEmitted = false;
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

    if (g_postIbiReadActive)
    {
        I3C_MasterTransferSmartDMAHandleIRQ(EXAMPLE_MASTER, &g_postIbiReadHandle);
        SDK_ISR_EXIT_BARRIER;
        return;
    }

    if (g_smartdmaTransferActive)
    {
        I3C_MasterTransferSmartDMAHandleIRQ(EXAMPLE_MASTER, &g_i3c_m_handle);
        SDK_ISR_EXIT_BARRIER;
        return;
    }

    if ((pending & ((uint32_t)kI3C_MasterSlaveStartFlag | (uint32_t)kI3C_MasterArbitrationWonFlag)) != 0U ||
        (masterState == kI3C_MasterStateIbiAck) || (masterState == kI3C_MasterStateIbiRcv) || g_ibiActive)
    {
        service_ibi_protocol_irq(EXAMPLE_MASTER, pending);
        pending = I3C_MasterGetPendingInterrupts(EXAMPLE_MASTER);
    }

    clearable = pending & (uint32_t)kI3C_MasterClearFlags;
    if (clearable != 0U)
    {
        I3C_MasterClearStatusFlags(EXAMPLE_MASTER, clearable);
    }

    SDK_ISR_EXIT_BARRIER;
}

int main(void)
{
    i3c_master_config_t masterConfig;
    i3c_master_transfer_t masterXfer;
    uint8_t addressList[8] = {0x30U, 0x31U, 0x32U, 0x33U, 0x34U, 0x35U, 0x36U, 0x37U};
    uint8_t devCount = 0U;
    uint8_t slaveAddr = 0U;
    i3c_device_info_t *devList;
    status_t result;

    BOARD_InitHardware();
    init_status_led();

    for (volatile uint32_t startup_delay = 0U; startup_delay < EXPERIMENT_STARTUP_WAIT; startup_delay++)
    {
        __NOP();
    }

    PRINTF("MCUX SDK version: %s\r\n", MCUXSDK_VERSION_FULL_STR);
    PRINTF("\r\nI3C board2board Smartdma example -- Master transfer.\r\n");
    EXP_LOG_INFO("MCUX SDK version: %s", MCUXSDK_VERSION_FULL_STR);
    EXP_LOG_INFO("I3C board2board Smartdma example -- Master transfer.");

    POWER_DisablePD(kPDRUNCFG_APD_SMARTDMA_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_SMARTDMA_SRAM);
    POWER_ApplyPD();

    for (uint32_t index = 0U; index < sizeof(ezh_data_buffer); index++)
    {
        ezh_data_buffer[index] = (uint8_t)(index + 1U);
    }
    memset(ezh_data_buffer_rx, 0, sizeof(ezh_data_buffer_rx));
    reset_master_failure_snapshot();

    PRINTF("\r\nThe master send data to slave:\r\n");
    for (uint32_t index = 0U; index < sizeof(ezh_data_buffer); index++)
    {
        if ((index % 8U) == 0U)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", ezh_data_buffer[index]);
    }

    keep_smartdma_api_alive();

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, SMART_DMA_TRIGGER_CHANNEL, kINPUTMUX_I3c0IrqToSmartDmaInput);
    INPUTMUX_Deinit(INPUTMUX);

    SMARTDMA_Init(
        SMARTDMA_SRAM_ADDR, __smartdma_start__, (uint32_t)((uintptr_t)__smartdma_end__ - (uintptr_t)__smartdma_start__));
    NVIC_EnableIRQ(SDMA_IRQn);
    NVIC_SetPriority(SDMA_IRQn, 3);
    SMARTDMA_Reset();

    I3C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz.i2cBaud = EXPERIMENT_I2C_BAUDRATE;
    masterConfig.baudRate_Hz.i3cPushPullBaud = EXPERIMENT_I3C_PP_BAUDRATE;
    masterConfig.baudRate_Hz.i3cOpenDrainBaud = EXPERIMENT_I3C_OD_BAUDRATE;
    masterConfig.enableOpenDrainStop = false;
    I3C_MasterInit(EXAMPLE_MASTER, &masterConfig, I3C_MASTER_CLOCK_FREQUENCY);

    I3C_MasterTransferCreateHandleSmartDMA(EXAMPLE_MASTER, &g_i3c_m_handle, &masterCallback, NULL);
    NVIC_ClearPendingIRQ(I3C0_IRQn);
    NVIC_SetPriority(I3C0_IRQn, 3);
    NVIC_EnableIRQ(I3C0_IRQn);

    PRINTF("\r\nI3C master do dynamic address assignment to the I3C slaves on bus.\r\n");
    EXP_LOG_INFO("I3C master do dynamic address assignment to the I3C slaves on bus.");

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = 0x7EU;
    masterXfer.subaddress = 0x06U;
    masterXfer.subaddressSize = 1U;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;

    result = run_transfer_blocking(&masterXfer);
    if (result != kStatus_Success)
    {
        capture_master_failure_snapshot(kMasterFailureStageRstdaa, result, NULL, 0U);
        PRINTF("\r\nRSTDAA failed: %d\r\n", result);
        EXP_LOG_ERROR("RSTDAA failed: %d", result);
        set_failure_led();
        return -1;
    }

    result = I3C_MasterProcessDAA(EXAMPLE_MASTER, addressList, 8U);
    if (result != kStatus_Success)
    {
        capture_master_failure_snapshot(kMasterFailureStageDaa, result, NULL, 0U);
        PRINTF("\r\nDAA failed: %d\r\n", result);
        EXP_LOG_ERROR("DAA failed: %d", result);
        set_failure_led();
        return -1;
    }

    PRINTF("\r\nI3C master dynamic address assignment done.\r\n");
    PRINTF("\r\nFetching device list after DAA...\r\n");
    EXP_LOG_INFO("I3C master dynamic address assignment done.");

    devList = I3C_MasterGetDeviceListAfterDAA(EXAMPLE_MASTER, &devCount);
    PRINTF("\r\nDevice count after DAA: %u\r\n", devCount);
    for (uint8_t devIndex = 0U; devIndex < devCount; devIndex++)
    {
        PRINTF("\r\nDevice[%u] vendor=0x%x dynamic=0x%x\r\n",
               devIndex,
               devList[devIndex].vendorID,
               devList[devIndex].dynamicAddr);
        if (devList[devIndex].vendorID == 0x123U)
        {
            slaveAddr = devList[devIndex].dynamicAddr;
            break;
        }
    }

    if (slaveAddr == 0U)
    {
        PRINTF("\r\nTarget slave not found.\r\n");
        EXP_LOG_ERROR("Target slave not found.");
        set_failure_led();
        return -1;
    }

    PRINTF("\r\nSelected slave address: 0x%x\r\n", slaveAddr);
    PRINTF("\r\nStart to do I3C master transfer in I3C SDR mode.\r\n");
    EXP_LOG_INFO("Selected slave address: 0x%x", slaveAddr);
    EXP_LOG_INFO("Start to do I3C master transfer in I3C SDR mode.");
    configure_slave_ibi_rule(EXAMPLE_MASTER, slaveAddr);

    if (EXPERIMENT_POST_DAA_CLEANUP != 0U)
    {
        I3C_MasterClearErrorStatusFlags(EXAMPLE_MASTER,
                                        (uint32_t)kI3C_MasterErrorNackFlag |
                                            (uint32_t)kI3C_MasterErrorWriteAbortFlag |
                                            (uint32_t)kI3C_MasterErrorTermFlag |
                                            (uint32_t)kI3C_MasterErrorParityFlag |
                                            (uint32_t)kI3C_MasterErrorCrcFlag |
                                            (uint32_t)kI3C_MasterErrorReadFlag |
                                            (uint32_t)kI3C_MasterErrorWriteFlag |
                                            (uint32_t)kI3C_MasterErrorMsgFlag |
                                            (uint32_t)kI3C_MasterErrorInvalidReqFlag |
                                            (uint32_t)kI3C_MasterErrorTimeoutFlag);
        I3C_MasterClearStatusFlags(EXAMPLE_MASTER,
                                   (uint32_t)kI3C_MasterSlaveStartFlag |
                                       (uint32_t)kI3C_MasterControlDoneFlag |
                                       (uint32_t)kI3C_MasterCompleteFlag |
                                       (uint32_t)kI3C_MasterArbitrationWonFlag |
                                       (uint32_t)kI3C_MasterSlave2MasterFlag |
                                       (uint32_t)kI3C_MasterErrorFlag);
        EXAMPLE_MASTER->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;
    }

    for (volatile uint32_t post_daa_delay = 0U; post_daa_delay < EXPERIMENT_POST_DAA_WAIT; post_daa_delay++)
    {
        __NOP();
    }

    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = slaveAddr;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;
    masterXfer.data = ezh_data_buffer;
    masterXfer.dataSize = sizeof(ezh_data_buffer);

    clear_ibi_state();
    result = run_write_transfer(&masterXfer);
    if (result != kStatus_Success)
    {
        capture_master_failure_snapshot(kMasterFailureStageWrite, result, ezh_data_buffer, sizeof(ezh_data_buffer));
        PRINTF("\r\nWrite transfer failed: %d\r\n", result);
        EXP_LOG_ERROR("Write transfer failed: %d", result);
        set_failure_led();
        return -1;
    }

    result = wait_for_ibi_notification(EXAMPLE_MASTER);
    if (result != kStatus_Success)
    {
        capture_master_failure_snapshot(kMasterFailureStageIbiWait, result, NULL, 0U);
        PRINTF("\r\nIBI wait failed: %d\r\n", result);
        EXP_LOG_ERROR("IBI wait failed: %d", result);
        set_failure_led();
        return -1;
    }

    I3C_MasterDisableInterrupts(EXAMPLE_MASTER, I3C_PROTOCOL_IRQ_MASK);
    NVIC_DisableIRQ(I3C0_IRQn);
    NVIC_ClearPendingIRQ(I3C0_IRQn);

    result = finalize_post_ibi_bus(EXAMPLE_MASTER);
    if (result != kStatus_Success)
    {
        capture_master_failure_snapshot(kMasterFailureStageIbiFinalize, result, NULL, 0U);
        PRINTF("\r\nPost-IBI bus finalize failed: %d\r\n", result);
        EXP_LOG_ERROR("Post-IBI bus finalize failed: %d", result);
        set_failure_led();
        return -1;
    }

    SDK_DelayAtLeastUs(EXPERIMENT_PRE_READ_DELAY_US, SystemCoreClock);

    result = run_post_ibi_read(EXAMPLE_MASTER, slaveAddr, ezh_data_buffer_rx, sizeof(ezh_data_buffer_rx));
    if (result != kStatus_Success)
    {
        capture_master_failure_snapshot(kMasterFailureStageRead, result, ezh_data_buffer_rx, sizeof(ezh_data_buffer_rx));
        log_buffer_prefix("Received buffer prefix after read failure:", ezh_data_buffer_rx, sizeof(ezh_data_buffer_rx));
        log_read_failure_context();

        if ((result == kStatus_I3C_Nak) && buffers_match(ezh_data_buffer, ezh_data_buffer_rx, EXPERIMENT_COMPARE_LENGTH))
        {
            PRINTF("\r\nRead transfer reported NAK after matching payload; accepting transfer.\r\n");
            EXP_LOG_INFO("Read transfer reported NAK after matching payload; accepting transfer.");
        }
        else
        {
        PRINTF("\r\nRead transfer failed: %d\r\n", result);
        EXP_LOG_ERROR("Read transfer failed: %d", result);
        set_failure_led();
        return -1;
        }
    }

    for (uint32_t index = 0U; index < EXPERIMENT_COMPARE_LENGTH; index++)
    {
        if (ezh_data_buffer[index] != ezh_data_buffer_rx[index])
        {
            capture_master_failure_snapshot(kMasterFailureStageCompare, kStatus_Fail, ezh_data_buffer_rx, sizeof(ezh_data_buffer_rx));
            PRINTF("\r\nError occurred in the transfer!\r\n");
            EXP_LOG_ERROR("Roundtrip mismatch index=%u expected=0x%02X actual=0x%02X",
                          (unsigned int)index,
                          ezh_data_buffer[index],
                          ezh_data_buffer_rx[index]);
            log_buffer_prefix("Expected buffer prefix:", ezh_data_buffer, sizeof(ezh_data_buffer));
            log_buffer_prefix("Received buffer prefix:", ezh_data_buffer_rx, sizeof(ezh_data_buffer_rx));
            EXP_LOG_ERROR("Error occurred in the transfer!");
            set_failure_led();
            return -1;
        }
    }

    PRINTF("\r\nI3C master transfer successful in I3C SDR mode.\r\n");
    EXP_LOG_INFO("I3C master transfer successful in I3C SDR mode.");
    set_success_led();
    return 0;
}

void SDMA_IRQHandler(void)
{
    SMARTDMA_HandleIRQ();
}
