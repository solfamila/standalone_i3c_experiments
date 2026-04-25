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
    kSlaveTraceRxComplete,
    kSlaveTraceEchoArmed,
    kSlaveTraceTxComplete,
    kSlaveTraceCompletionError,
};

#define SLAVE_RETAINED_TRACE_MAGIC 0x53545243U
#define SLAVE_RETAINED_TRACE_VERSION 1U

enum
{
    kSlaveRetainedTraceDynamicAddrSeen = 1U << 0,
    kSlaveRetainedTraceRxCompleteSeen  = 1U << 1,
    kSlaveRetainedTraceEchoArmedSeen   = 1U << 2,
    kSlaveRetainedTraceTxCompleteSeen  = 1U << 3,
    kSlaveRetainedTraceErrorSeen       = 1U << 4,
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

#define SLAVE_TRACE_ENTRY_COUNT 8U

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
static volatile bool g_slaveActivityLedActive = false;
static volatile bool g_slaveActivityLedCompletionPending = false;
static volatile uint32_t g_slaveActivityLedPollCount = 0U;
static volatile uint32_t g_slaveActivityLedToggleCount = 0U;

static void i3c_slave_init_activity_led(void)
{
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
    g_slaveActivityLedActive = false;
    g_slaveActivityLedCompletionPending = false;
    g_slaveActivityLedPollCount = 0U;
    g_slaveActivityLedToggleCount = 0U;
    EXP_LED_Set(true, false, false);
}

static void i3c_slave_service_activity_led(void)
{
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

static void i3c_slave_record_trace(uint32_t type, const uint8_t *buffer, uint32_t count, uint32_t status)
{
    switch (type)
    {
        case kSlaveTraceRxComplete:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceRxCompleteSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceRxCompleteSeen;
                g_slaveRetainedTrace.rxCompletionCount = count;
            }
            break;

        case kSlaveTraceEchoArmed:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceEchoArmedSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceEchoArmedSeen;
                g_slaveRetainedTrace.echoArmedCount = count;
            }
            break;

        case kSlaveTraceTxComplete:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceTxCompleteSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceTxCompleteSeen;
                g_slaveRetainedTrace.txCompletionCount = count;
            }
            break;

        case kSlaveTraceCompletionError:
            if ((g_slaveRetainedTrace.eventFlags & kSlaveRetainedTraceErrorSeen) == 0U)
            {
                g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceErrorSeen;
                g_slaveRetainedTrace.completionErrorStatus = status;
            }
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
    if ((regAddr != NULL) && (g_slave_rxBuff[0] == g_deviceAddress))
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

static void i3c_slave_callback(I3C_Type *base, i3c_slave_transfer_t *xfer, void *userData)
{
    (void)base;
    (void)userData;

    switch ((uint32_t)xfer->event)
    {
        case kI3C_SlaveAddressMatchEvent:
            break;

        case kI3C_SlaveTransmitEvent:
            g_lastTransferWasReceive = false;
            i3c_slave_buildTxBuff(xfer->rxData, &xfer->txData, (uint32_t *)&xfer->txDataSize);
            i3c_slave_record_trace(kSlaveTraceTxPrepared, xfer->txData, (uint32_t)xfer->txDataSize, 0U);
            break;

        case kI3C_SlaveReceiveEvent:
            g_lastTransferWasReceive = true;
            xfer->rxData = g_slave_rxBuff;
            xfer->rxDataSize = I3C_SLAVE_RX_DATA_LENGTH;
            break;

        case kI3C_SlaveStartEvent:
            i3c_slave_begin_activity_led();
            break;

        case (kI3C_SlaveTransmitEvent | kI3C_SlaveHDRCommandMatchEvent):
            g_lastTransferWasReceive = false;
            xfer->txData = g_slave_txBuff;
            xfer->txDataSize = I3C_SLAVE_TX_DATA_LENGTH;
            break;

        case (kI3C_SlaveReceiveEvent | kI3C_SlaveHDRCommandMatchEvent):
            g_lastTransferWasReceive = true;
            xfer->rxData = g_slave_rxBuff;
            xfer->rxDataSize = I3C_SLAVE_RX_DATA_LENGTH;
            break;

        case kI3C_SlaveCompletionEvent:
            i3c_slave_complete_activity_led();
            if (xfer->completionStatus == kStatus_Success)
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

                    i3c_slave_record_trace(
                        kSlaveTraceRxComplete, g_slave_rxBuff, (uint32_t)xfer->transferredCount, 0U);
                    g_txBuff = g_slave_rxBuff;
                    g_txSize = echoedCount;
                    i3c_slave_record_trace(kSlaveTraceEchoArmed, g_txBuff, g_txSize, 0U);
                }
                else
                {
                    i3c_slave_record_trace(kSlaveTraceTxComplete, g_txBuff, (uint32_t)xfer->transferredCount, 0U);
                }
                g_slaveCompletionFlag = true;
            }
            else
            {
                i3c_slave_fail_activity_led();
                i3c_slave_record_trace(kSlaveTraceCompletionError, NULL, 0U, (uint32_t)xfer->completionStatus);
            }
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
    i3c_slave_run_boot_led_self_test();
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
    slaveConfig.maxWriteLength = I3C_SLAVE_RX_DATA_LENGTH;
    slaveConfig.maxReadLength = I3C_SLAVE_TX_DATA_LENGTH;
    slaveConfig.offline = false;

    I3C_SlaveInit(EXAMPLE_SLAVE, &slaveConfig, I3C_SLAVE_CLOCK_FREQUENCY);
    semihost_write0("slave: i3c slave init done\n");

    eventMask = kI3C_SlaveAllEvents;

    I3C_SlaveTransferCreateHandle(EXAMPLE_SLAVE, &g_i3c_s_handle, i3c_slave_callback, NULL);

    memset(g_slave_txBuff, 0, sizeof(g_slave_txBuff));
    memset(g_slave_rxBuff, 0, sizeof(g_slave_rxBuff));
    g_txBuff = g_slave_txBuff;
    I3C_SlaveTransferNonBlocking(EXAMPLE_SLAVE, &g_i3c_s_handle, eventMask);
    semihost_write0("slave: waiting for master traffic\n");

    while (1)
    {
        i3c_slave_service_activity_led();
        i3c_slave_flush_trace();

        if ((!dynamicAddrReported) && ((EXAMPLE_SLAVE->SDYNADDR & I3C_SDYNADDR_DAVALID_MASK) != 0U))
        {
            uint32_t dynamicAddrReg = EXAMPLE_SLAVE->SDYNADDR;

            dynamicAddrReported = true;
            g_slaveRetainedTrace.eventFlags |= kSlaveRetainedTraceDynamicAddrSeen;
            g_slaveRetainedTrace.dynamicAddrReg = dynamicAddrReg;
            semihost_write_label_hex32("slave: dynamic addr reg=", dynamicAddrReg);
        }

        __NOP();
    }
}