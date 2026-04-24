/*
 * Copyright 2019, 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
#include <string.h>
/*  SDK Included Files */
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i3c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_SLAVE              I3C0
#define I3C_SLAVE_CLOCK_FREQUENCY  CLOCK_GetLpOscFreq()
#define I3C_TIME_OUT_INDEX         100000000
#define I3C_MASTER_SLAVE_ADDR_7BIT 0x1EU

#define ENABLE_PRINTF

#define I3C_MASTER_DMA_TX_TEST
#define I3C_SLAVE_RX_BUFFER_SIZE   8U
#define BYTES_PER_TRANSFER         4U
#define EXPECTED_TRANSFERS         (I3C_SLAVE_RX_BUFFER_SIZE / BYTES_PER_TRANSFER)

uint8_t g_slave_rxBuff[I3C_SLAVE_RX_BUFFER_SIZE] = {0};
uint8_t g_slave_rxBuff_combined[I3C_SLAVE_RX_BUFFER_SIZE] = {0};

volatile bool g_slaveCompletionFlag = false;
volatile uint32_t g_transferCount = 0;
volatile uint32_t g_totalBytesReceived = 0;

i3c_slave_handle_t g_i3c_s_handle;

static void i3c_slave_callback(I3C_Type *base, i3c_slave_transfer_t *xfer, void *userData)
{
    switch ((uint32_t)xfer->event)
    {
        case kI3C_SlaveReceiveEvent:
            xfer->rxData = g_slave_rxBuff;
            xfer->rxDataSize = BYTES_PER_TRANSFER;
            break;

        case kI3C_SlaveCompletionEvent:
            if (xfer->completionStatus == kStatus_Success)
            {
                if (xfer->transferredCount > 0)
                {
                    for (uint32_t i = 0; i < xfer->transferredCount; i++)
                    {
                        g_slave_rxBuff_combined[g_totalBytesReceived + i] = g_slave_rxBuff[i];
                    }
                    g_totalBytesReceived += xfer->transferredCount;
                    g_transferCount++;
                    
#ifdef ENABLE_PRINTF
                    PRINTF("[Slave] Transfer %d completed, received %d bytes\r\n", 
                           g_transferCount, xfer->transferredCount);
#endif
                    
                    if (g_transferCount >= EXPECTED_TRANSFERS)
                    {
                        g_slaveCompletionFlag = true;
                    }
                    
                    memset(g_slave_rxBuff, 0, I3C_SLAVE_RX_BUFFER_SIZE);
                }
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
    
#if defined(I3C_ASYNC_WAKE_UP_INTR_CLEAR)
    eventMask |= kI3C_SlaveAddressMatchEvent;
#endif

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    
#ifdef ENABLE_PRINTF
    BOARD_InitDebugConsole();
#endif
    
    CLOCK_AttachClk(kMAIN_CLK_to_I3C_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivI3cClk, 8);
    
    CLOCK_AttachClk(kLPOSC_to_I3C_TC_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivI3cTcClk, 1);
    CLOCK_SetClkDiv(kCLOCK_DivI3cSlowClk, 1);

#ifdef ENABLE_PRINTF
    PRINTF("\r\n");
    PRINTF("============================================\r\n");
    PRINTF("  I3C Slave - Ready to Receive Data\r\n");
    PRINTF("============================================\r\n");
    PRINTF("CPU Freq = %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("I3C Freq = %d Hz\r\n", CLOCK_GetFreq(kCLOCK_I3cClk));
    PRINTF("Expected: %d transfers x %dB = %dB total\r\n", 
           EXPECTED_TRANSFERS, BYTES_PER_TRANSFER, I3C_SLAVE_RX_BUFFER_SIZE);
    PRINTF("============================================\r\n\r\n");
#endif

    I3C_SlaveGetDefaultConfig(&slaveConfig);
    slaveConfig.staticAddr = I3C_MASTER_SLAVE_ADDR_7BIT;
    slaveConfig.vendorID = 0x123U;
    slaveConfig.offline = false;
    I3C_SlaveInit(EXAMPLE_SLAVE, &slaveConfig, I3C_SLAVE_CLOCK_FREQUENCY);

    I3C_SlaveTransferCreateHandle(EXAMPLE_SLAVE, &g_i3c_s_handle, i3c_slave_callback, NULL);

    I3C_SlaveTransferNonBlocking(EXAMPLE_SLAVE, &g_i3c_s_handle, eventMask);

#ifdef ENABLE_PRINTF
    PRINTF("[Slave] Waiting for DAA...\r\n");
#endif

    uint32_t timeout_i = 0U;
    while ((g_slaveCompletionFlag != true) && (++timeout_i < I3C_TIME_OUT_INDEX))
    {
    }

    if (timeout_i == I3C_TIME_OUT_INDEX)
    {
#ifdef ENABLE_PRINTF
        PRINTF("[Error] Transfer timeout\r\n");
        PRINTF("Received %d/%d transfers, %d/%d bytes\r\n", 
               g_transferCount, EXPECTED_TRANSFERS, 
               g_totalBytesReceived, I3C_SLAVE_RX_BUFFER_SIZE);
#endif
        return -1;
    }

#ifdef ENABLE_PRINTF
    PRINTF("\r\n============================================\r\n");
    PRINTF("  All Transfers Completed!\r\n");
    PRINTF("============================================\r\n");
    PRINTF("Total transfers: %d\r\n", g_transferCount);
    PRINTF("Total bytes: %d\r\n", g_totalBytesReceived);
    PRINTF("\r\nReceived data:\r\n  ");
    for (uint32_t i = 0; i < g_totalBytesReceived; i++)
    {
        PRINTF("0x%02x ", g_slave_rxBuff_combined[i]);
        if ((i + 1) % BYTES_PER_TRANSFER == 0)
            PRINTF("| ");
    }
    PRINTF("\r\n============================================\r\n");
#endif

    while (1)
    {
        __NOP();
    }
}
