/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_smartdma.h"
#include "fsl_power.h"
#include "fsl_inputmux.h"
#include "fsl_i3c.h"
#include  "fsl_i3c_smartdma.h"

#ifndef APP_ENABLE_SEMIHOST
#define APP_ENABLE_SEMIHOST 1
#endif

static void semihost_write0(const char *message)
{
#if APP_ENABLE_SEMIHOST
    register uint32_t operation asm("r0") = 0x04U;
    register const char *parameter asm("r1") = message;

    __asm volatile("bkpt 0xAB" : "+r"(operation) : "r"(parameter) : "memory");
#else
    (void)message;
#endif
}

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void keep_smartdma_api_alive(void);

static void i3c_master_ibi_callback(I3C_Type *base,
		i3c_master_smartdma_handle_t *handle,
                                    i3c_ibi_type_t ibiType,
                                    i3c_ibi_state_t ibiState);
static void i3c_master_smartdma_callback(I3C_Type *base, i3c_master_smartdma_handle_t *handle, status_t status, void *userData);
/*******************************************************************************
 * Variables
 ******************************************************************************/
#define SMARTDMA_FIRMWARE_ADDR 0x20000000
#define SMARTDMA_FIRMWARE_SIZE 0x3F0
#define SMART_DMA_TRIGGER_CHANNEL   0U // Use channel 0 for Smart DMA trigger. Total 8 channels

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
extern void EZH_Callback(void *param);

__attribute__((aligned(4))) uint8_t ezh_data_buffer[255];
__attribute__((aligned(4))) uint8_t ezh_data_buffer_rx[255];


/*I3C related*/
uint8_t g_master_ibiBuff[10];
i3c_master_smartdma_handle_t g_i3c_m_handle;

const i3c_master_smartdma_callback_t masterCallback = {
    .slave2Master = NULL, .ibiCallback = i3c_master_ibi_callback, .transferComplete = i3c_master_smartdma_callback};
volatile bool g_masterCompletionFlag = false;
volatile bool g_ibiWonFlag           = false;
volatile status_t g_completionStatus = kStatus_Success;
#define I3C_TIME_OUT_INDEX 100000000U


static void i3c_master_ibi_callback(I3C_Type *base,
                                    i3c_master_smartdma_handle_t *handle,
                                    i3c_ibi_type_t ibiType,
                                    i3c_ibi_state_t ibiState)
{
    switch (ibiType)
    {
        case kI3C_IbiNormal:
            if (ibiState == kI3C_IbiDataBuffNeed)
            {
                handle->ibiBuff = g_master_ibiBuff;
            }
            else
            {
                /* Handle ibi data*/
            }
            break;

        default:
            assert(false);
            break;
    }
}

static void i3c_master_smartdma_callback(I3C_Type *base, i3c_master_smartdma_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
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
    char ch;
    i3c_master_config_t masterConfig;
    i3c_master_transfer_t masterXfer;
    status_t result = kStatus_Success;
    volatile uint32_t timeout = 0;

    /* Init board hardware. */
    BOARD_InitHardware();
    semihost_write0("archive-master: start\n");

    PRINTF("MCUX SDK version: %s\r\n", MCUXSDK_VERSION_FULL_STR);

    PRINTF("\r\nI3C board2board Smartdma example -- Master transfer.");

    POWER_DisablePD(kPDRUNCFG_APD_SMARTDMA_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_SMARTDMA_SRAM);
    POWER_ApplyPD();

    for(uint32_t i = 0; i< sizeof(ezh_data_buffer);i++)
    {
    	ezh_data_buffer[i] = i + 0x1;
    }

    PRINTF("\r\nThe master send data to slave:\r\n");

    for(uint32_t i = 0; i < sizeof(ezh_data_buffer);i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", ezh_data_buffer[i]);
    }

    keep_smartdma_api_alive();

    RESET_PeripheralReset(kINPUTMUX_RST_SHIFT_RSTn);

    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, SMART_DMA_TRIGGER_CHANNEL, kINPUTMUX_I3c0IrqToSmartDmaInput);
    INPUTMUX_Deinit(INPUTMUX);

    I3C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz.i2cBaud          = EXAMPLE_I2C_BAUDRATE;
    masterConfig.baudRate_Hz.i3cPushPullBaud  = EXAMPLE_I3C_PP_BAUDRATE;
    masterConfig.baudRate_Hz.i3cOpenDrainBaud = EXAMPLE_I3C_OD_BAUDRATE;
    masterConfig.enableOpenDrainStop          = false;
    I3C_MasterInit(EXAMPLE_MASTER, &masterConfig, I3C_MASTER_CLOCK_FREQUENCY);

    SMARTDMA_Init(SMARTDMA_SRAM_ADDR, (void *)SMARTDMA_FIRMWARE_ADDR, SMARTDMA_FIRMWARE_SIZE);
    NVIC_EnableIRQ(SDMA_IRQn);

    NVIC_SetPriority(SDMA_IRQn, 3);
    SMARTDMA_Reset();
    /* Create I3C handle. */
    I3C_MasterTransferCreateHandleSmartDMA(EXAMPLE_MASTER, &g_i3c_m_handle, &masterCallback, NULL);

    PRINTF("\r\nI3C master do dynamic address assignment to the I3C slaves on bus.");

    /* Reset dynamic address before DAA */
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress   = 0x7EU; /* Broadcast address */
    masterXfer.subaddress     = 0x06U; /* CCC command RSTDAA */
    masterXfer.subaddressSize = 1U;
    masterXfer.direction      = kI3C_Write;
    masterXfer.busType        = kI3C_TypeI3CSdr;
    masterXfer.flags          = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse    = kI3C_IbiRespAckMandatory;
    result                    = I3C_MasterTransferSmartDMA(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

    timeout = 0U;
    /* Wait for transfer completed. */
    while ((!g_ibiWonFlag) && (!g_masterCompletionFlag) && (g_completionStatus == kStatus_Success) &&
           (++timeout < I3C_TIME_OUT_INDEX))
    {
        __NOP();
    }

    result = g_completionStatus;
    if ((result != kStatus_Success) || (timeout == I3C_TIME_OUT_INDEX))
    {
        return -1;
    }
    g_masterCompletionFlag = false;

    uint8_t addressList[8] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37};
    result                 = I3C_MasterProcessDAA(EXAMPLE_MASTER, addressList, 8);
    if (result != kStatus_Success)
    {
        return -1;
    }

    PRINTF("\r\nI3C master dynamic address assignment done.\r\n");

    uint8_t devCount;
    i3c_device_info_t *devList;
    uint8_t slaveAddr = 0x0U;
    devList           = I3C_MasterGetDeviceListAfterDAA(EXAMPLE_MASTER, &devCount);
    for (uint8_t devIndex = 0; devIndex < devCount; devIndex++)
    {
        if (devList[devIndex].vendorID == 0x123U)
        {
            slaveAddr = devList[devIndex].dynamicAddr;
            break;
        }
    }
    PRINTF("\r\nStart to do I3C master transfer in I3C SDR mode.");

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = slaveAddr;
    masterXfer.data         = ezh_data_buffer;
    masterXfer.dataSize     = sizeof(ezh_data_buffer);
    masterXfer.direction    = kI3C_Write;
    masterXfer.busType      = kI3C_TypeI3CSdr;
    masterXfer.flags        = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse  = kI3C_IbiRespAckMandatory;
    result                  = I3C_MasterTransferSmartDMA(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

    /* Wait for transfer completed. */
    while (!g_masterCompletionFlag)
    {
        __NOP();
    }

    if (g_completionStatus != kStatus_Success)
    {
        PRINTF("\r\nTransfer error %u.\r\n", g_completionStatus);
        return -1;
    }
    g_masterCompletionFlag = false;

    /* Wait until the slave is ready for transmit, wait time depend on user's case.*/
    for (volatile uint32_t i = 0U; i < WAIT_TIME; i++)
    {
        __NOP();
    }


    memset(ezh_data_buffer_rx, 0, sizeof(ezh_data_buffer_rx));
    masterXfer.slaveAddress = slaveAddr;
    masterXfer.data         = ezh_data_buffer_rx;
    masterXfer.dataSize     = sizeof(ezh_data_buffer_rx);
    masterXfer.direction    = kI3C_Read;
    masterXfer.busType      = kI3C_TypeI3CSdr;
    masterXfer.flags        = kI3C_TransferDefaultFlag;
    masterXfer.ibiResponse  = kI3C_IbiRespAckMandatory;
    result                  = I3C_MasterTransferSmartDMA(EXAMPLE_MASTER, &g_i3c_m_handle, &masterXfer);
    if (kStatus_Success != result)
    {
        return result;
    }

    /* Wait for transfer completed. */
    while (!g_masterCompletionFlag)
    {
        __NOP();
    }

    if (g_completionStatus != kStatus_Success)
    {
        return -1;
    }
    g_masterCompletionFlag = false;

    for (uint32_t i = 0U; i < sizeof(ezh_data_buffer_rx); i++)
    {
        if (ezh_data_buffer[i] != ezh_data_buffer_rx[i])
        {
            PRINTF("\r\nError occurred in the transfer! \r\n");
            return -1;
        }
    }


    PRINTF("\r\nI3C master transfer successful in I3C SDR mode.\r\n");
    semihost_write0("I3C master transfer successful in I3C SDR mode.\n");




}

void SDMA_IRQHandler(void)
{
    SMARTDMA_HandleIRQ();
}

