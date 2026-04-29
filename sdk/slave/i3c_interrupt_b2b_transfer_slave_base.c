/*
 * Local standalone copy of the validated slave implementation used by the
 * passing experiments. It mirrors the repo-side callback-driven flow so the
 * separate experiments repo does not depend on a modified tts worktree.
 */

/*  Standard C Included Files */
#include <string.h>
#include <stdint.h>
/*  SDK Included Files */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i3c.h"
#include "fsl_power.h"
#include "fsl_reset.h"
#include "experiment_led.h"

#ifndef APP_ENABLE_SEMIHOST
#define APP_ENABLE_SEMIHOST 1
#endif

#ifndef EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX
#define EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX 0
#endif

#ifndef EXPERIMENT_SLAVE_IBI_DATA
#define EXPERIMENT_SLAVE_IBI_DATA 0xA5U
#endif

#ifndef EXPERIMENT_SLAVE_MIN_ECHO_COUNT
#define EXPERIMENT_SLAVE_MIN_ECHO_COUNT 0U
#endif

#ifndef EXPERIMENT_SKIP_SLAVE_BOOT_LED_TEST
#define EXPERIMENT_SKIP_SLAVE_BOOT_LED_TEST 0
#endif

#ifndef EXPERIMENT_FORCE_ECHO_ON_ANY_TRANSMIT
#define EXPERIMENT_FORCE_ECHO_ON_ANY_TRANSMIT 0
#endif

#define EXPERIMENT_SLAVE_BCR_IBI_REQUEST_CAPABLE (1U << 1)
#define EXPERIMENT_SLAVE_BCR_IBI_PAYLOAD (1U << 2)

static void semihost_write0(const char *message);

static void semihost_write_hex32(uint32_t value)
{
    static const char hex_digits[] = "0123456789ABCDEF";
    char message[] = "0x00000000\n";

    for (uint32_t index = 0; index < 8U; index++)
    {
        uint32_t shift = (7U - index) * 4U;
        message[2U + index] = hex_digits[(value >> shift) & 0xFU];
    }

    semihost_write0(message);
}

static void semihost_write_label_hex32(const char *label, uint32_t value)
{
    semihost_write0(label);
    semihost_write_hex32(value);
}

enum
{
    kSlaveTraceTxPrepared = 1U,
    kSlaveTraceRxArmed,
    kSlaveTraceRxComplete,
    kSlaveTraceEchoArmed,
    kSlaveTraceTxComplete,
    kSlaveTraceCompletionError,
    kSlaveTraceIbiQueued,
    kSlaveTraceIbiIssued,
    kSlaveTraceIbiRequestSent,
};

#define SLAVE_RETAINED_TRACE_MAGIC 0x53545243U
#define SLAVE_RETAINED_TRACE_VERSION 6U

enum
{
    kSlavePostIbiEchoSourceNone = 0U,
    kSlavePostIbiEchoSourceAddressMatch = 1U,
    kSlavePostIbiEchoSourceTransmitEvent = 2U,
};

enum
{
    kSlaveRetainedTraceDynamicAddrSeen = 1U << 0,
    kSlaveRetainedTraceRxCompleteSeen  = 1U << 1,
    kSlaveRetainedTraceEchoArmedSeen   = 1U << 2,
    kSlaveRetainedTraceTxCompleteSeen  = 1U << 3,
    kSlaveRetainedTraceErrorSeen       = 1U << 4,
    kSlaveRetainedTraceIbiQueuedSeen   = 1U << 5,
    kSlaveRetainedTraceIbiIssuedSeen   = 1U << 6,
    kSlaveRetainedTraceIbiSentSeen     = 1U << 7,
};

typedef struct slave_retained_trace
{
    uint32_t magic;
    uint32_t version;
    uint32_t eventFlags;
    uint32_t dynamicAddrReg;
    uint32_t completionErrorStatus;
    uint32_t rxCompletionCount;
    uint32_t echoArmedCount;
    uint32_t txCompletionCount;
    uint32_t txPreparedCount;
    uint32_t completionErrorFlags;
    uint32_t completionErrorTransferredCount;
    uint32_t completionErrorWasReceive;
    uint32_t postEchoTxPreparedCount;
    uint32_t postEchoTxCompletionCount;
    uint32_t postEchoCompletionErrorStatus;
    uint32_t postEchoCompletionErrorFlags;
    uint32_t postEchoCompletionErrorWasReceive;
    uint32_t postIbiAddressMatchCount;
    uint32_t postIbiAddressMatchTxSize;
    uint32_t ibiQueuedCount;
    uint32_t ibiIssuedCount;
    uint32_t ibiRequestSentCount;
    uint32_t ibiStatusBeforeRequest;
    uint32_t ibiStatusAfterRequest;
    uint32_t ibiRequestSentStatus;
    uint32_t rxFirstByte;
    uint32_t rxLastByte;
    uint32_t echoFirstByte;
    uint32_t echoLastByte;
    uint32_t txFirstByte;
    uint32_t txLastByte;
    uint32_t lastTransmitIbiIssued;
    uint32_t lastTransmitIbiRequestSent;
    uint32_t lastTransmitPostIbiAddressMatched;
    uint32_t lastTransmitTxSizeBefore;
    uint32_t lastTransmitGTxSize;
    uint32_t lastTransmitPostEchoPrepared;
    uint32_t lastTransmitFirstByte;
    uint32_t lastTransmitTxSizeAfter;
    uint32_t lastTransmitTxDataIsNull;
    uint32_t postIbiEchoAddressMatchServeCount;
    uint32_t postIbiEchoTransmitServeCount;
    uint32_t postIbiEchoTxCompletionCount;
    uint32_t lastAddressMatchServedPostIbiEcho;
    uint32_t lastTransmitServedPostIbiEcho;
    uint32_t lastPostIbiEchoServeSource;
    uint32_t currentGeneration;
    uint32_t currentEchoedCount;
    uint32_t currentIbiDelayLoops;
    uint32_t currentIbiPending;
    uint32_t currentIbiIssued;
    uint32_t currentIbiRequestSent;
    uint32_t currentPostIbiAddressMatched;
    uint32_t currentPostIbiEchoPending;
    uint32_t lastQueuedGeneration;
    uint32_t lastIssuedGeneration;
    uint32_t lastRequestSentGeneration;
    uint32_t lastAddressMatchGeneration;
    uint32_t lastTransmitServeGeneration;
    uint32_t lastTxCompletionGeneration;
} slave_retained_trace_t;

#if APP_ENABLE_SEMIHOST

typedef struct slave_trace_entry
{
    uint32_t type;
    uint32_t count;
    uint32_t status;
    uint32_t firstByte;
    uint32_t lastByte;
} slave_trace_entry_t;

#define SLAVE_TRACE_ENTRY_COUNT 16U

static volatile uint32_t g_slaveTraceWriteIndex;
static volatile uint32_t g_slaveTraceReadIndex;
static slave_trace_entry_t g_slaveTraceEntries[SLAVE_TRACE_ENTRY_COUNT];

static void semihost_write_trace_entry(const char *prefix, const slave_trace_entry_t *entry)
{
    semihost_write_label_hex32(prefix, entry->count);
    if (entry->count != 0U)
    {
        semihost_write_label_hex32("slave: first byte=", entry->firstByte);
        semihost_write_label_hex32("slave: last byte=", entry->lastByte);
    }
}
#endif

static void semihost_write0(const char *message)
{
#if APP_ENABLE_SEMIHOST
    register uint32_t operation asm("r0") = 0x04U;
    register const char *parameter asm("r1") = message;

    __asm volatile(
        "bkpt 0xAB"
        : "+r"(operation)
        : "r"(parameter)
        : "memory");
#else
    (void)message;
#endif
}

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SLAVE              I3C0
#define I3C_SLAVE_CLOCK_FREQUENCY  CLOCK_GetLpOscFreq()
#define I3C_TIME_OUT_INDEX         0xFFFFFFFFU
#define I3C_MASTER_SLAVE_ADDR_7BIT 0x1EU

#define ENABLE_PRINTF

/*For MASTER DMA TX TEST*/
//#define I3C_MASTER_DMA_TX_TEST
#ifndef I3C_SLAVE_RX_DATA_LENGTH
#define I3C_SLAVE_RX_DATA_LENGTH            255U
#endif

///*For MASTER DMA RX TEST*/
#define I3C_MASTER_DMA_RX_TEST
#ifndef I3C_SLAVE_TX_DATA_LENGTH
#define I3C_SLAVE_TX_DATA_LENGTH            255U
#endif

#define I3C_SLAVE_LED_TOGGLE_INTERVAL 250000U
#define I3C_SLAVE_LED_VISIBLE_PULSE_US 200000U
#define I3C_SLAVE_LED_VISIBLE_PULSE_COUNT 2U
#define I3C_SLAVE_IBI_POST_STOP_DELAY_LOOPS 1U

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_slave_txBuff[I3C_SLAVE_TX_DATA_LENGTH] = {0};
uint8_t g_slave_rxBuff[I3C_SLAVE_RX_DATA_LENGTH] = {0};

volatile bool g_slaveCompletionFlag = false;
i3c_slave_handle_t g_i3c_s_handle;
uint8_t *g_txBuff;
uint32_t g_txSize = I3C_SLAVE_TX_DATA_LENGTH;
volatile uint8_t g_deviceAddress = 0U;
uint8_t *g_deviceBuff = NULL;
uint8_t g_deviceBuffSize = I3C_SLAVE_RX_DATA_LENGTH;
volatile bool g_lastTransferWasReceive = false;
__attribute__((section(".usb_ram"), used, aligned(4))) volatile slave_retained_trace_t g_slaveRetainedTrace;
static volatile bool g_slaveIbiPending = false;
static volatile bool g_slaveRearmAfterInvalidStart = false;
static volatile uint32_t g_slaveInvalidStartRearmCount = 0U;
volatile bool g_slaveIbiIssued = false;
volatile bool g_slaveIbiRequestSent = false;
volatile bool g_slavePostIbiAddressMatched = false;
static volatile bool g_slavePostIbiEchoPending = false;
static volatile bool g_slavePostIbiEchoArmed = false;
static volatile bool g_slavePostIbiEchoConsumed = false;
static volatile uint32_t g_slaveIbiDelayLoops = 0U;
static uint8_t g_slaveIbiPayload[1] = {EXPERIMENT_SLAVE_IBI_DATA};
static volatile bool g_slaveActivityLedActive = false;
static volatile bool g_slaveActivityLedCompletionPending = false;
static volatile bool g_slaveActivityLedFinal = false;
static volatile uint32_t g_slaveActivityLedPollCount = 0U;
static volatile uint32_t g_slaveActivityLedToggleCount = 0U;

static bool i3c_slave_roundtrip_done(void)
{
    const uint32_t doneFlags = kSlaveRetainedTraceRxCompleteSeen | kSlaveRetainedTraceTxCompleteSeen;

    return (g_slaveRetainedTrace.eventFlags & doneFlags) == doneFlags;
}

static void i3c_slave_init_activity_led(void)
{
    g_slaveActivityLedFinal = false;
    EXP_LED_Init();
    EXP_LED_Set(false, true, false);
}

static void i3c_slave_run_boot_led_self_test(void)
{
    EXP_LED_Blink(false, false, true, I3C_SLAVE_LED_VISIBLE_PULSE_COUNT, I3C_SLAVE_LED_VISIBLE_PULSE_US);
    EXP_LED_Set(false, true, false);
}

static void i3c_slave_begin_activity_led(void)
{
    if (g_slaveActivityLedFinal)
    {
        return;
    }

    g_slaveActivityLedActive = true;
    g_slaveActivityLedCompletionPending = false;
    g_slaveActivityLedPollCount = 0U;
    g_slaveActivityLedToggleCount = 0U;
    EXP_LED_Set(false, false, true);
}

static void i3c_slave_complete_activity_led(void)
{
    g_slaveActivityLedActive = false;
    g_slaveActivityLedCompletionPending = true;
}

static void i3c_slave_fail_activity_led(void)
{
    g_slaveActivityLedFinal = true;
    g_slaveActivityLedActive = false;
    g_slaveActivityLedCompletionPending = false;
    g_slaveActivityLedPollCount = 0U;
    g_slaveActivityLedToggleCount = 0U;
    EXP_LED_Set(true, false, false);
}

static void i3c_slave_service_activity_led(void)
{
    if (g_slaveActivityLedFinal)
    {
        return;
    }

    if (i3c_slave_roundtrip_done())
    {
        g_slaveActivityLedActive = false;
        g_slaveActivityLedCompletionPending = false;
        g_slaveActivityLedPollCount = 0U;
        g_slaveActivityLedToggleCount = 0U;
        g_slaveActivityLedFinal = true;
        EXP_LED_Set(false, true, false);
        return;
    }

    if (g_slaveActivityLedActive)
    {
        g_slaveActivityLedPollCount++;
        if (g_slaveActivityLedPollCount < I3C_SLAVE_LED_TOGGLE_INTERVAL)
        {
            return;
        }

        g_slaveActivityLedPollCount = 0U;
        g_slaveActivityLedToggleCount++;
        EXP_LED_ToggleBlue();
        return;
    }

    if (!g_slaveActivityLedCompletionPending)
    {
        return;
    }

    EXP_LED_Blink(false, true, false, I3C_SLAVE_LED_VISIBLE_PULSE_COUNT, I3C_SLAVE_LED_VISIBLE_PULSE_US);
    EXP_LED_Set(false, true, false);
    g_slaveActivityLedPollCount = 0U;
    g_slaveActivityLedToggleCount = 0U;
    g_slaveActivityLedCompletionPending = false;
}

/*******************************************************************************
 * Code
 ******************************************************************************/
static void i3c_slave_update_retained_ibi_state(void);

static void i3c_slave_reset_retained_trace(void)
{
    volatile uint32_t *traceWords = (volatile uint32_t *)&g_slaveRetainedTrace;

    for (uint32_t index = 0; index < (sizeof(g_slaveRetainedTrace) / sizeof(uint32_t)); index++)
    {
        traceWords[index] = 0U;
    }

    g_slaveRetainedTrace.magic = SLAVE_RETAINED_TRACE_MAGIC;
    g_slaveRetainedTrace.version = SLAVE_RETAINED_TRACE_VERSION;

#if APP_ENABLE_SEMIHOST
    g_slaveTraceWriteIndex = 0U;
    g_slaveTraceReadIndex = 0U;
    (void)memset(g_slaveTraceEntries, 0, sizeof(g_slaveTraceEntries));
#endif
}

static void i3c_slave_enable_retained_trace_ram(void)
{
    POWER_DisablePD(kPDRUNCFG_APD_USBHS_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_USBHS_SRAM);
    POWER_ApplyPD();

    RESET_PeripheralReset(kUSBHS_SRAM_RST_SHIFT_RSTn);
    CLOCK_EnableClock(kCLOCK_UsbhsSram);
}

static void i3c_slave_rearm_after_invalid_start(uint32_t eventMask)
{
    g_slaveRearmAfterInvalidStart = false;
    g_slaveInvalidStartRearmCount++;
    g_slaveCompletionFlag = false;
    g_lastTransferWasReceive = false;
    g_slaveIbiPending = false;
    g_slaveIbiIssued = false;
    g_slaveIbiRequestSent = false;
    g_slavePostIbiAddressMatched = false;
    g_slavePostIbiEchoPending = false;
    g_slavePostIbiEchoArmed = false;
    g_slavePostIbiEchoConsumed = false;
    g_slaveIbiDelayLoops = 0U;
    g_txBuff = g_slave_txBuff;
    g_txSize = I3C_SLAVE_TX_DATA_LENGTH;
    g_slaveRetainedTrace.currentEchoedCount = 0U;
    i3c_slave_update_retained_ibi_state();

    I3C_SlaveTransferAbort(EXAMPLE_SLAVE, &g_i3c_s_handle);
    I3C_SlaveClearErrorStatusFlags(EXAMPLE_SLAVE, I3C_SlaveGetErrorStatusFlags(EXAMPLE_SLAVE));
    I3C_SlaveClearStatusFlags(EXAMPLE_SLAVE, (uint32_t)kI3C_SlaveClearFlags);
    EXAMPLE_SLAVE->SDATACTRL |= I3C_SDATACTRL_FLUSHTB_MASK | I3C_SDATACTRL_FLUSHFB_MASK;

    (void)I3C_SlaveTransferNonBlocking(EXAMPLE_SLAVE, &g_i3c_s_handle, eventMask);
#ifdef ENABLE_PRINTF
    PRINTF("slave: rearmed after pre-DAA invalid start\r\n");
#endif
    semihost_write0("slave: rearmed after pre-DAA invalid start\n");
}

static void i3c_slave_reset_ibi_generation_state(void)
{
    g_slaveIbiPending = false;
    g_slaveIbiIssued = false;
    g_slaveIbiRequestSent = false;
    g_slavePostIbiAddressMatched = false;
    g_slavePostIbiEchoPending = false;
    g_slavePostIbiEchoArmed = false;
    g_slavePostIbiEchoConsumed = false;
    g_slaveIbiDelayLoops = 0U;
    g_txBuff = g_slave_txBuff;
    g_txSize = I3C_SLAVE_TX_DATA_LENGTH;
    g_slaveRetainedTrace.currentEchoedCount = 0U;
    i3c_slave_update_retained_ibi_state();
}

static void i3c_slave_record_trace(uint32_t type, const uint8_t *buffer, uint32_t count, uint32_t status)
{
    uint32_t firstByte = 0U;
    uint32_t lastByte = 0U;

    if ((buffer != NULL) && (count != 0U))
    {
        firstByte = buffer[0];
        lastByte = buffer[count - 1U];
    }

    switch (type)
    {
        case kSlaveTraceRxComplete:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceRxCompleteSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceRxCompleteSeen;
                g_slaveRetainedTrace.rxCompletionCount = count;
                g_slaveRetainedTrace.rxFirstByte = firstByte;
                g_slaveRetainedTrace.rxLastByte = lastByte;
            }
            break;

        case kSlaveTraceEchoArmed:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceEchoArmedSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceEchoArmedSeen;
                g_slaveRetainedTrace.echoArmedCount = count;
                g_slaveRetainedTrace.echoFirstByte = firstByte;
                g_slaveRetainedTrace.echoLastByte = lastByte;
            }
            break;

        case kSlaveTraceTxComplete:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceTxCompleteSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceTxCompleteSeen;
                g_slaveRetainedTrace.txCompletionCount = count;
                g_slaveRetainedTrace.txFirstByte = firstByte;
                g_slaveRetainedTrace.txLastByte = lastByte;
            }
            break;

        case kSlaveTraceCompletionError:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceErrorSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceErrorSeen;
                g_slaveRetainedTrace.completionErrorStatus = status;
            }
            break;

        case kSlaveTraceIbiQueued:
            g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceIbiQueuedSeen;
            g_slaveRetainedTrace.ibiQueuedCount = count;
            break;

        case kSlaveTraceIbiIssued:
            g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceIbiIssuedSeen;
            g_slaveRetainedTrace.ibiIssuedCount = count;
            g_slaveRetainedTrace.ibiStatusAfterRequest = status;
            break;

        case kSlaveTraceIbiRequestSent:
            g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceIbiSentSeen;
            g_slaveRetainedTrace.ibiRequestSentCount = count;
            g_slaveRetainedTrace.ibiRequestSentStatus = status;
            break;

        default:
            break;
    }

#if APP_ENABLE_SEMIHOST
    {
        uint32_t writeIndex = g_slaveTraceWriteIndex;
        slave_trace_entry_t *entry = &g_slaveTraceEntries[writeIndex % SLAVE_TRACE_ENTRY_COUNT];

        entry->type = type;
        entry->count = count;
        entry->status = status;
        if ((buffer != NULL) && (count != 0U))
        {
            entry->firstByte = buffer[0];
            entry->lastByte = buffer[count - 1U];
        }
        else
        {
            entry->firstByte = 0U;
            entry->lastByte = 0U;
        }

        g_slaveTraceWriteIndex = writeIndex + 1U;
        if ((g_slaveTraceWriteIndex - g_slaveTraceReadIndex) > SLAVE_TRACE_ENTRY_COUNT)
        {
            g_slaveTraceReadIndex = g_slaveTraceWriteIndex - SLAVE_TRACE_ENTRY_COUNT;
        }
    }
#endif
}

#if APP_ENABLE_SEMIHOST
static void i3c_slave_flush_trace(void)
{
    while (g_slaveTraceReadIndex != g_slaveTraceWriteIndex)
    {
        slave_trace_entry_t entry = g_slaveTraceEntries[g_slaveTraceReadIndex % SLAVE_TRACE_ENTRY_COUNT];

        switch (entry.type)
        {
            case kSlaveTraceTxPrepared:
                semihost_write_trace_entry("slave: tx prepared count=", &entry);
                break;

            case kSlaveTraceRxArmed:
                semihost_write_label_hex32("slave: rx armed size=", entry.count);
                break;

            case kSlaveTraceRxComplete:
                semihost_write_trace_entry("slave: rx completion count=", &entry);
                break;

            case kSlaveTraceEchoArmed:
                semihost_write_trace_entry("slave: echo armed count=", &entry);
                break;

            case kSlaveTraceTxComplete:
                semihost_write_trace_entry("slave: tx completion count=", &entry);
                break;

            case kSlaveTraceCompletionError:
                semihost_write0("slave: completion status ");
                semihost_write_hex32(entry.status);
                semihost_write_label_hex32("slave: completion flags=", entry.count);
                break;

            case kSlaveTraceIbiQueued:
                semihost_write_label_hex32("slave: ibi queued count=", entry.count);
                break;

            case kSlaveTraceIbiIssued:
                semihost_write_label_hex32("slave: ibi issued count=", entry.count);
                semihost_write_label_hex32("slave: ibi status after request=", entry.status);
                break;

            case kSlaveTraceIbiRequestSent:
                semihost_write_label_hex32("slave: ibi request-sent count=", entry.count);
                semihost_write_label_hex32("slave: ibi request-sent status=", entry.status);
                break;

            default:
                break;
        }

        g_slaveTraceReadIndex++;
    }
}
#else
static void i3c_slave_flush_trace(void)
{
}
#endif

static void i3c_slave_buildTxBuff(uint8_t *regAddr, uint8_t **txBuff, uint32_t *txBuffSize)
{
    if ((regAddr != NULL) && (g_slave_rxBuff[0] == g_deviceAddress) && (g_deviceBuff != NULL) &&
        (g_deviceBuffSize != 0U))
    {
        *txBuff = g_deviceBuff;
        *txBuffSize = g_deviceBuffSize;
    }
    else
    {
        *txBuff = g_txBuff;
        *txBuffSize = g_txSize;
    }
}

static void i3c_slave_update_retained_ibi_state(void)
{
    g_slaveRetainedTrace.currentIbiDelayLoops = g_slaveIbiDelayLoops;
    g_slaveRetainedTrace.currentIbiPending = g_slaveIbiPending ? 1U : 0U;
    g_slaveRetainedTrace.currentIbiIssued = g_slaveIbiIssued ? 1U : 0U;
    g_slaveRetainedTrace.currentIbiRequestSent = g_slaveIbiRequestSent ? 1U : 0U;
    g_slaveRetainedTrace.currentPostIbiAddressMatched = g_slavePostIbiAddressMatched ? 1U : 0U;
    g_slaveRetainedTrace.currentPostIbiEchoPending = g_slavePostIbiEchoPending ? 1U : 0U;
}

static bool i3c_slave_should_arm_post_ibi_echo(void)
{
    return g_slavePostIbiEchoPending && !g_slavePostIbiEchoConsumed && !g_slavePostIbiEchoArmed &&
           (g_txBuff != NULL) && (g_txSize != 0U);
}

static bool i3c_slave_post_ibi_echo_in_flight(void)
{
    return g_slavePostIbiEchoPending && !g_slavePostIbiEchoConsumed && g_slavePostIbiEchoArmed;
}

static void i3c_slave_arm_post_ibi_echo(i3c_slave_transfer_t *xfer, uint32_t source)
{
    xfer->txData = g_txBuff;
    xfer->txDataSize = g_txSize;

    g_slavePostIbiEchoArmed = true;
    g_slavePostIbiAddressMatched = true;
    g_slaveIbiPending = false;
    g_slaveIbiDelayLoops = 0U;

    g_slaveRetainedTrace.postIbiAddressMatchTxSize = (uint32_t)xfer->txDataSize;
    g_slaveRetainedTrace.lastTransmitTxSizeAfter = (uint32_t)xfer->txDataSize;
    g_slaveRetainedTrace.lastTransmitTxDataIsNull = (xfer->txData == NULL) ? 1U : 0U;
    g_slaveRetainedTrace.lastPostIbiEchoServeSource = source;

    if (source == kSlavePostIbiEchoSourceAddressMatch)
    {
        g_slaveRetainedTrace.postIbiEchoAddressMatchServeCount++;
        g_slaveRetainedTrace.lastAddressMatchServedPostIbiEcho = 1U;
        g_slaveRetainedTrace.lastAddressMatchGeneration = g_slaveRetainedTrace.currentGeneration;
    }
    else if (source == kSlavePostIbiEchoSourceTransmitEvent)
    {
        g_slaveRetainedTrace.postIbiEchoTransmitServeCount++;
        g_slaveRetainedTrace.lastTransmitServedPostIbiEcho = 1U;
        g_slaveRetainedTrace.lastTransmitServeGeneration = g_slaveRetainedTrace.currentGeneration;
    }

    i3c_slave_update_retained_ibi_state();
}

static void i3c_slave_callback(I3C_Type *base, i3c_slave_transfer_t *xfer, void *userData)
{
    (void)base;
    (void)userData;

    switch ((uint32_t)xfer->event)
    {
        case kI3C_SlaveAddressMatchEvent:
            g_slaveRetainedTrace.lastAddressMatchServedPostIbiEcho = 0U;
            if (i3c_slave_should_arm_post_ibi_echo())
            {
                g_slaveRetainedTrace.postIbiAddressMatchCount++;
                i3c_slave_arm_post_ibi_echo(xfer, kSlavePostIbiEchoSourceAddressMatch);
#ifdef ENABLE_PRINTF
                PRINTF("slave: post-ibi address match txsize=%lu first=0x%02x\r\n",
                       (unsigned long)xfer->txDataSize,
                       ((xfer->txData != NULL) && (xfer->txDataSize != 0U)) ? xfer->txData[0] : 0U);
#endif
            }
            break;

        case kI3C_SlaveTransmitEvent:
            g_lastTransferWasReceive = false;
            g_slaveRetainedTrace.lastTransmitIbiIssued = g_slaveIbiIssued ? 1U : 0U;
            g_slaveRetainedTrace.lastTransmitIbiRequestSent = g_slaveIbiRequestSent ? 1U : 0U;
            g_slaveRetainedTrace.lastTransmitPostIbiAddressMatched = g_slavePostIbiAddressMatched ? 1U : 0U;
            g_slaveRetainedTrace.lastTransmitTxSizeBefore = (uint32_t)xfer->txDataSize;
            g_slaveRetainedTrace.lastTransmitGTxSize = g_txSize;
            g_slaveRetainedTrace.lastTransmitPostEchoPrepared = g_slaveRetainedTrace.postEchoTxPreparedCount;
            g_slaveRetainedTrace.lastTransmitFirstByte =
                ((g_txBuff != NULL) && (g_txSize != 0U)) ? g_txBuff[0] : 0xEEU;
            g_slaveRetainedTrace.lastTransmitTxSizeAfter = 0U;
            g_slaveRetainedTrace.lastTransmitTxDataIsNull = 1U;
            g_slaveRetainedTrace.lastTransmitServedPostIbiEcho = 0U;

#if EXPERIMENT_FORCE_ECHO_ON_ANY_TRANSMIT
            if (i3c_slave_should_arm_post_ibi_echo())
            {
                i3c_slave_arm_post_ibi_echo(xfer, kSlavePostIbiEchoSourceTransmitEvent);
                if (g_slaveRetainedTrace.txPreparedCount == 0U)
                {
                    g_slaveRetainedTrace.txPreparedCount = (uint32_t)xfer->txDataSize;
                }
                if (((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceEchoArmedSeen) != 0U) &&
                    (g_slaveRetainedTrace.postEchoTxPreparedCount == 0U))
                {
                    g_slaveRetainedTrace.postEchoTxPreparedCount = (uint32_t)xfer->txDataSize;
                }
                i3c_slave_record_trace(kSlaveTraceTxPrepared, xfer->txData, (uint32_t)xfer->txDataSize, 0U);
                break;
            }
#endif

            if (i3c_slave_should_arm_post_ibi_echo())
            {
                i3c_slave_arm_post_ibi_echo(xfer, kSlavePostIbiEchoSourceTransmitEvent);
                if (g_slaveRetainedTrace.txPreparedCount == 0U)
                {
                    g_slaveRetainedTrace.txPreparedCount = (uint32_t)xfer->txDataSize;
                }
                if (((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceEchoArmedSeen) != 0U) &&
                    (g_slaveRetainedTrace.postEchoTxPreparedCount == 0U))
                {
                    g_slaveRetainedTrace.postEchoTxPreparedCount = (uint32_t)xfer->txDataSize;
                }
                i3c_slave_record_trace(kSlaveTraceTxPrepared, xfer->txData, (uint32_t)xfer->txDataSize, 0U);
                break;
            }

            if (i3c_slave_post_ibi_echo_in_flight())
            {
                xfer->txData = g_txBuff;
                xfer->txDataSize = g_txSize;
                g_slaveRetainedTrace.lastTransmitTxSizeAfter = (uint32_t)xfer->txDataSize;
                g_slaveRetainedTrace.lastTransmitTxDataIsNull = (xfer->txData == NULL) ? 1U : 0U;
                break;
            }

#if EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX
            if (g_slaveIbiIssued && !g_slaveIbiRequestSent && !g_slavePostIbiAddressMatched)
            {
                /* While the IBI request is still being sent, keep the normal
                 * post-echo data buffer out of the generic transmit path.
                 * Once the post-IBI read has address-matched, the real
                 * RequiredRead path needs the echo buffer immediately.
                 */
                xfer->txData = NULL;
                xfer->txDataSize = 0U;
                break;
            }
#endif
            i3c_slave_buildTxBuff(xfer->rxData, &xfer->txData, (uint32_t *)&xfer->txDataSize);
#ifdef ENABLE_PRINTF
            if (g_slaveIbiRequestSent || g_slavePostIbiAddressMatched)
            {
                PRINTF("slave: transmit event post-ibi txsize=%lu first=0x%02x\r\n",
                       (unsigned long)xfer->txDataSize,
                       ((xfer->txData != NULL) && (xfer->txDataSize != 0U)) ? xfer->txData[0] : 0U);
            }
#endif
            if (g_slaveRetainedTrace.txPreparedCount == 0U)
            {
                g_slaveRetainedTrace.txPreparedCount = (uint32_t)xfer->txDataSize;
            }
            if (((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceEchoArmedSeen) != 0U) &&
                (g_slaveRetainedTrace.postEchoTxPreparedCount == 0U))
            {
                g_slaveRetainedTrace.postEchoTxPreparedCount = (uint32_t)xfer->txDataSize;
            }
            g_slaveRetainedTrace.lastTransmitTxSizeAfter = (uint32_t)xfer->txDataSize;
            g_slaveRetainedTrace.lastTransmitTxDataIsNull = (xfer->txData == NULL) ? 1U : 0U;
            i3c_slave_record_trace(kSlaveTraceTxPrepared, xfer->txData, (uint32_t)xfer->txDataSize, 0U);
            break;

        case kI3C_SlaveReceiveEvent:
            i3c_slave_reset_ibi_generation_state();
            g_lastTransferWasReceive = true;
            xfer->rxData = g_slave_rxBuff;
            xfer->rxDataSize = I3C_SLAVE_RX_DATA_LENGTH;
            i3c_slave_record_trace(kSlaveTraceRxArmed, NULL, (uint32_t)xfer->rxDataSize, 0U);
            break;

        case kI3C_SlaveStartEvent:
            i3c_slave_begin_activity_led();
            break;

        case kI3C_SlaveReceivedCCCEvent:
#ifdef ENABLE_PRINTF
            PRINTF("slave: CCC event flags=0x%08lx\r\n", (unsigned long)I3C_SlaveGetStatusFlags(base));
#endif
            break;

        case (kI3C_SlaveTransmitEvent | kI3C_SlaveHDRCommandMatchEvent):
            g_lastTransferWasReceive = false;
            xfer->txData = g_slave_txBuff;
            xfer->txDataSize = I3C_SLAVE_TX_DATA_LENGTH;
            break;

        case (kI3C_SlaveReceiveEvent | kI3C_SlaveHDRCommandMatchEvent):
            i3c_slave_reset_ibi_generation_state();
            g_lastTransferWasReceive = true;
            xfer->rxData = g_slave_rxBuff;
            xfer->rxDataSize = I3C_SLAVE_RX_DATA_LENGTH;
            i3c_slave_record_trace(kSlaveTraceRxArmed, NULL, (uint32_t)xfer->rxDataSize, 0U);
            break;

        case kI3C_SlaveCompletionEvent:
            i3c_slave_complete_activity_led();
            if ((xfer->completionStatus == kStatus_Success) ||
                (g_lastTransferWasReceive && ((uint32_t)xfer->transferredCount != 0U) &&
                 (xfer->completionStatus == kStatus_I3C_Term)) ||
                ((!g_lastTransferWasReceive) && g_slavePostIbiEchoPending &&
                 ((uint32_t)xfer->transferredCount != 0U) && (xfer->completionStatus == kStatus_I3C_Term)))
            {
                if (g_lastTransferWasReceive)
                {
                    uint32_t echoedCount = (uint32_t)xfer->transferredCount;

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

                    i3c_slave_record_trace(
                        kSlaveTraceRxComplete, g_slave_rxBuff, (uint32_t)xfer->transferredCount, 0U);
                    g_txBuff = g_slave_rxBuff;
                    g_txSize = echoedCount;
                    g_slavePostIbiEchoPending = (echoedCount != 0U);
                    g_slavePostIbiEchoArmed = false;
                    g_slavePostIbiEchoConsumed = false;
                    g_slaveRetainedTrace.currentGeneration++;
                    g_slaveRetainedTrace.currentEchoedCount = echoedCount;
                    i3c_slave_record_trace(kSlaveTraceEchoArmed, g_txBuff, g_txSize, 0U);
#if EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX
                    g_slaveIbiPayload[0] = (uint8_t)echoedCount;
                    g_slaveIbiPending = true;
                    g_slaveIbiIssued = false;
                    g_slaveIbiRequestSent = false;
                    g_slavePostIbiAddressMatched = false;
                    g_slaveIbiDelayLoops = I3C_SLAVE_IBI_POST_STOP_DELAY_LOOPS;
                    g_slaveRetainedTrace.lastQueuedGeneration = g_slaveRetainedTrace.currentGeneration;
                    i3c_slave_update_retained_ibi_state();
                    i3c_slave_record_trace(
                        kSlaveTraceIbiQueued, NULL, g_slaveRetainedTrace.ibiQueuedCount + 1U, 0U);
#endif
                }
                else
                {
                    i3c_slave_record_trace(kSlaveTraceTxComplete, g_txBuff, (uint32_t)xfer->transferredCount, 0U);
                    if (g_slavePostIbiEchoPending && ((uint32_t)xfer->transferredCount != 0U))
                    {
                        g_slaveRetainedTrace.postIbiEchoTxCompletionCount++;
                        g_slaveRetainedTrace.lastTxCompletionGeneration = g_slaveRetainedTrace.currentGeneration;
                        g_slavePostIbiEchoConsumed = true;
                        g_slavePostIbiEchoArmed = false;
                        g_slavePostIbiEchoPending = false;
                        g_slaveIbiPending = false;
                        g_slaveIbiIssued = false;
                        g_slaveIbiRequestSent = false;
                        g_slavePostIbiAddressMatched = false;
                        i3c_slave_update_retained_ibi_state();
                    }
                    if (((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceEchoArmedSeen) != 0U) &&
                        (g_slaveRetainedTrace.postEchoTxCompletionCount == 0U))
                    {
                        g_slaveRetainedTrace.postEchoTxCompletionCount = (uint32_t)xfer->transferredCount;
                    }
                }
                g_slaveCompletionFlag = true;
            }
            else
            {
                if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceErrorSeen) == 0U)
                {
                    g_slaveRetainedTrace.completionErrorFlags = I3C_SlaveGetStatusFlags(base);
                    g_slaveRetainedTrace.completionErrorTransferredCount = (uint32_t)xfer->transferredCount;
                    g_slaveRetainedTrace.completionErrorWasReceive = g_lastTransferWasReceive ? 1U : 0U;
                }
                if ((xfer->completionStatus == kStatus_I3C_InvalidStart) &&
                    ((EXAMPLE_SLAVE->SDYNADDR & I3C_SDYNADDR_DAVALID_MASK) == 0U))
                {
#ifdef ENABLE_PRINTF
                    PRINTF("slave: invalid start before DAA status=%ld flags=0x%08lx\r\n",
                           (long)xfer->completionStatus,
                           (unsigned long)I3C_SlaveGetStatusFlags(base));
#endif
                    g_slaveRearmAfterInvalidStart = true;
                    break;
                }

#ifdef ENABLE_PRINTF
                PRINTF("slave: completion error status=%ld flags=0x%08lx\r\n",
                       (long)xfer->completionStatus,
                       (unsigned long)I3C_SlaveGetStatusFlags(base));
#endif
                i3c_slave_fail_activity_led();
                if (((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceEchoArmedSeen) != 0U) &&
                    (g_slaveRetainedTrace.postEchoCompletionErrorStatus == 0U))
                {
                    g_slaveRetainedTrace.postEchoCompletionErrorStatus = (uint32_t)xfer->completionStatus;
                    g_slaveRetainedTrace.postEchoCompletionErrorFlags = I3C_SlaveGetStatusFlags(base);
                    g_slaveRetainedTrace.postEchoCompletionErrorWasReceive = g_lastTransferWasReceive ? 1U : 0U;
                }
                i3c_slave_record_trace(
                    kSlaveTraceCompletionError, NULL, I3C_SlaveGetStatusFlags(base), (uint32_t)xfer->completionStatus);
            }
            break;

        case kI3C_SlaveRequestSentEvent:
            g_slaveIbiPending = false;
            g_slaveIbiRequestSent = true;
            g_slavePostIbiAddressMatched = false;
            g_slaveRetainedTrace.lastRequestSentGeneration = g_slaveRetainedTrace.currentGeneration;
            i3c_slave_update_retained_ibi_state();
            i3c_slave_record_trace(kSlaveTraceIbiRequestSent,
                                   NULL,
                                   g_slaveRetainedTrace.ibiRequestSentCount + 1U,
                                   I3C_SlaveGetStatusFlags(base));
#ifdef ENABLE_PRINTF
            PRINTF("slave: ibi request sent status=0x%08lx\r\n", (unsigned long)I3C_SlaveGetStatusFlags(base));
#endif
            break;

#if defined(I3C_ASYNC_WAKE_UP_INTR_CLEAR)
        case kI3C_SlaveAddressMatchEvent:
            I3C_ASYNC_WAKE_UP_INTR_CLEAR
            break;
#endif

        default:
            break;
    }
}

int main(void)
{
    i3c_slave_config_t slaveConfig;
    uint32_t eventMask = kI3C_SlaveCompletionEvent;
    bool dynamicAddrReported = false;
#if defined(I3C_ASYNC_WAKE_UP_INTR_CLEAR)
    eventMask |= kI3C_SlaveAddressMatchEvent;
#endif

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    i3c_slave_init_activity_led();

    i3c_slave_enable_retained_trace_ram();
    i3c_slave_reset_retained_trace();
    semihost_write0("slave: boot init done\n");

#ifdef ENABLE_PRINTF
    BOARD_InitDebugConsole();
#endif
    semihost_write0("slave: debug console init done\n");

    CLOCK_AttachClk(kMAIN_CLK_to_I3C_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivI3cClk, 8);
    CLOCK_AttachClk(kLPOSC_to_I3C_TC_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivI3cTcClk, 1);
    CLOCK_SetClkDiv(kCLOCK_DivI3cSlowClk, 1);

#ifdef ENABLE_PRINTF
    PRINTF("\r\nI3C board2board interrupt example -- Slave transfer.\r\n");
    PRINTF("\r\n CPU Freq = %d\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("\r\n I3C Freq = %d\r\n", CLOCK_GetFreq(kCLOCK_I3cClk));
#endif
    semihost_write0("slave: i3c clocks configured\n");

    I3C_SlaveGetDefaultConfig(&slaveConfig);

    slaveConfig.staticAddr = I3C_MASTER_SLAVE_ADDR_7BIT;
    slaveConfig.vendorID = 0x123U;
#if EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX
    slaveConfig.bcr |= EXPERIMENT_SLAVE_BCR_IBI_REQUEST_CAPABLE | EXPERIMENT_SLAVE_BCR_IBI_PAYLOAD;
#endif
    slaveConfig.maxWriteLength = I3C_SLAVE_RX_DATA_LENGTH;
    slaveConfig.maxReadLength = I3C_SLAVE_TX_DATA_LENGTH;
    slaveConfig.offline = false;

    I3C_SlaveInit(EXAMPLE_SLAVE, &slaveConfig, I3C_SLAVE_CLOCK_FREQUENCY);
    I3C_SlaveSetWatermarks(
        EXAMPLE_SLAVE, kI3C_TxTriggerUntilOneLessThanFull, kI3C_RxTriggerOnNotEmpty, true, true);
    semihost_write0("slave: i3c slave init done\n");

    eventMask = kI3C_SlaveAllEvents;

    I3C_SlaveTransferCreateHandle(EXAMPLE_SLAVE, &g_i3c_s_handle, i3c_slave_callback, NULL);

    memset(g_slave_txBuff, 0, sizeof(g_slave_txBuff));
    memset(g_slave_rxBuff, 0, sizeof(g_slave_rxBuff));
    g_txBuff = g_slave_txBuff;
    g_slaveIbiPending = false;
    g_slaveIbiIssued = false;
    g_slaveIbiRequestSent = false;
    g_slavePostIbiEchoPending = false;
    g_slavePostIbiEchoArmed = false;
    g_slavePostIbiEchoConsumed = false;
    g_slaveIbiDelayLoops = 0U;
    g_slaveRetainedTrace.currentEchoedCount = 0U;
    i3c_slave_update_retained_ibi_state();
    I3C_SlaveTransferNonBlocking(EXAMPLE_SLAVE, &g_i3c_s_handle, eventMask);
#ifdef ENABLE_PRINTF
    PRINTF("slave: armed\r\n");
    PRINTF("slave: waiting for master traffic\r\n");
#endif
    semihost_write0("slave: armed\n");
    semihost_write0("slave: waiting for master traffic\n");

#if !EXPERIMENT_SKIP_SLAVE_BOOT_LED_TEST
    i3c_slave_run_boot_led_self_test();
#endif

    while (1)
    {
        i3c_slave_service_activity_led();
        i3c_slave_flush_trace();

        if (g_slaveRearmAfterInvalidStart)
        {
            i3c_slave_rearm_after_invalid_start(eventMask);
        }

#if EXPERIMENT_SLAVE_REQUEST_IBI_AFTER_RX
        if (g_slaveIbiPending && !g_slaveIbiRequestSent)
        {
            if (g_slaveIbiDelayLoops != 0U)
            {
                g_slaveIbiDelayLoops--;
                g_slaveRetainedTrace.currentIbiDelayLoops = g_slaveIbiDelayLoops;
            }
            else
            {
                /* The request API only arms EVENT; keep retrying until the
                 * controller reports RequestSentEvent.
                 */
                g_slaveRetainedTrace.ibiStatusBeforeRequest = I3C_SlaveGetStatusFlags(EXAMPLE_SLAVE);
                I3C_SlaveRequestIBIWithData(EXAMPLE_SLAVE, g_slaveIbiPayload, 1U);
                i3c_slave_record_trace(kSlaveTraceIbiIssued,
                                       NULL,
                                       g_slaveRetainedTrace.ibiIssuedCount + 1U,
                                       I3C_SlaveGetStatusFlags(EXAMPLE_SLAVE));
                g_slaveIbiIssued = true;
                g_slaveIbiDelayLoops = 1U;
                g_slaveRetainedTrace.lastIssuedGeneration = g_slaveRetainedTrace.currentGeneration;
                i3c_slave_update_retained_ibi_state();
            }
        }
#endif

        if ((!dynamicAddrReported) && ((EXAMPLE_SLAVE->SDYNADDR & I3C_SDYNADDR_DAVALID_MASK) != 0U))
        {
            uint32_t dynamicAddrReg = EXAMPLE_SLAVE->SDYNADDR;

            dynamicAddrReported = true;
            g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceDynamicAddrSeen;
            g_slaveRetainedTrace.dynamicAddrReg = dynamicAddrReg;
#ifdef ENABLE_PRINTF
            PRINTF("slave: dynamic addr reg=0x%08lx\r\n", (unsigned long)dynamicAddrReg);
#endif
            semihost_write_label_hex32("slave: dynamic addr reg=", dynamicAddrReg);
        }

        __NOP();
    }
}