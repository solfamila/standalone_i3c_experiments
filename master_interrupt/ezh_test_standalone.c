/*
 * Standalone master_interrupt app.
 */

#define EXPERIMENT_USE_SMARTDMA 0
#define EZH_ROUNDTRIP_DATA_LENGTH 8U
#define EXPERIMENT_STARTUP_WAIT 5000000U
#define EXPERIMENT_POST_DAA_CLEANUP 0U
#define EXPERIMENT_POST_DAA_WAIT 50000000U
#define EXPERIMENT_I3C_PP_BAUDRATE 4000000U


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
#else
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

#define SMART_DMA_TRIGGER_CHANNEL 0U
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
static const i3c_master_smartdma_callback_t masterCallback = {
    .slave2Master = NULL,
    .ibiCallback = i3c_master_ibi_callback,
    .transferComplete = i3c_master_callback,
};
static volatile bool g_masterCompletionFlag = false;
static volatile bool g_ibiWonFlag = false;
static volatile status_t g_completionStatus = kStatus_Success;

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
    g_completionStatus = kStatus_Success;

    result = I3C_MasterTransferSmartDMA(EXAMPLE_MASTER, &g_i3c_m_handle, transfer);
    if (result != kStatus_Success)
    {
        return result;
    }

    result = wait_for_transfer_completion();
    g_masterCompletionFlag = false;
    g_ibiWonFlag = false;
    return result;
#endif
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

    g_completionStatus = status;
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
        PRINTF("\r\nRSTDAA failed: %d\r\n", result);
        EXP_LOG_ERROR("RSTDAA failed: %d", result);
        set_failure_led();
        return -1;
    }

    result = I3C_MasterProcessDAA(EXAMPLE_MASTER, addressList, 8U);
    if (result != kStatus_Success)
    {
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

    result = run_transfer_blocking(&masterXfer);
    if (result != kStatus_Success)
    {
        PRINTF("\r\nWrite transfer failed: %d\r\n", result);
        EXP_LOG_ERROR("Write transfer failed: %d", result);
        set_failure_led();
        return -1;
    }

    for (volatile uint32_t delay = 0U; delay < WAIT_TIME; delay++)
    {
        __NOP();
    }

    memset(ezh_data_buffer_rx, 0, sizeof(ezh_data_buffer_rx));
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = slaveAddr;
    masterXfer.direction = kI3C_Read;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse = kI3C_IbiRespAckMandatory;
    masterXfer.data = ezh_data_buffer_rx;
    masterXfer.dataSize = sizeof(ezh_data_buffer_rx);

    result = run_transfer_blocking(&masterXfer);
    if (result != kStatus_Success)
    {
        PRINTF("\r\nRead transfer failed: %d\r\n", result);
        EXP_LOG_ERROR("Read transfer failed: %d", result);
        set_failure_led();
        return -1;
    }

    for (uint32_t index = 0U; index < EXPERIMENT_COMPARE_LENGTH; index++)
    {
        if (ezh_data_buffer[index] != ezh_data_buffer_rx[index])
        {
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
