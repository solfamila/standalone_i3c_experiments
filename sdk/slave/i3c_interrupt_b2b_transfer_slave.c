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


//#define ENABLE_PRINTF

/*For MASTER DMA TX TEST*/
#define I3C_MASTER_DMA_TX_TEST
#define I3C_SLAVE_RX_DATA_LENGTH            255U



///*For MASTER DMA RX TEST*/
#define I3C_MASTER_DMA_RX_TEST
#define I3C_SLAVE_TX_DATA_LENGTH            255U


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_slave_txBuff[I3C_SLAVE_TX_DATA_LENGTH] = {0};
uint8_t g_slave_rxBuff[I3C_SLAVE_RX_DATA_LENGTH] = {0};


volatile bool g_slaveCompletionFlag         = false;
i3c_slave_handle_t g_i3c_s_handle;
uint8_t *g_txBuff;
uint32_t g_txSize                = I3C_SLAVE_TX_DATA_LENGTH;
volatile uint8_t g_deviceAddress = 0U;
uint8_t *g_deviceBuff            = NULL;
uint8_t g_deviceBuffSize         = I3C_SLAVE_RX_DATA_LENGTH;
volatile bool g_lastTransferWasReceive = false;
/*******************************************************************************
 * Code
 ******************************************************************************/
static void i3c_slave_buildTxBuff(uint8_t *regAddr, uint8_t **txBuff, uint32_t *txBuffSize)
{
    /* If this is a combined frame and have device address information, send out the device buffer content and size.
     The received device address information is loaded in g_slave_rxBuff according to callback implementation of
     kI3C_SlaveReceiveEvent*/
    if ((regAddr != NULL) && (g_slave_rxBuff[0] == g_deviceAddress))
    {
        *txBuff     = g_deviceBuff;
        *txBuffSize = g_deviceBuffSize;
    }
    else
    {
        /* No valid register address information received, send default tx buffer. */
        *txBuff     = g_txBuff;
        *txBuffSize = g_txSize;
    }
}

static void i3c_slave_callback(I3C_Type *base, i3c_slave_transfer_t *xfer, void *userData)
{
    switch ((uint32_t)xfer->event)
    {
        /*  Transmit request */
        case kI3C_SlaveTransmitEvent:
            g_lastTransferWasReceive = false;
            /*  Update information for transmit process */
            i3c_slave_buildTxBuff(xfer->rxData, &xfer->txData, (uint32_t *)&xfer->txDataSize);
            break;

        /*  Receive request */
        case kI3C_SlaveReceiveEvent:
            g_lastTransferWasReceive = true;
            /*  Update information for received process */
            xfer->rxData     = g_slave_rxBuff;
            xfer->rxDataSize = I3C_SLAVE_RX_DATA_LENGTH;
            break;

        /*  Transmit request */
        case (kI3C_SlaveTransmitEvent | kI3C_SlaveHDRCommandMatchEvent):
            g_lastTransferWasReceive = false;
            /*  Update information for transmit process */
            xfer->txData     = g_slave_txBuff;
            xfer->txDataSize = I3C_SLAVE_TX_DATA_LENGTH;
            break;

        /*  Receive request */
        case (kI3C_SlaveReceiveEvent | kI3C_SlaveHDRCommandMatchEvent):
            g_lastTransferWasReceive = true;
            /*  Update information for received process */
            xfer->rxData     = g_slave_rxBuff;
            xfer->rxDataSize = I3C_SLAVE_RX_DATA_LENGTH;
            break;

        /*  Transfer done */
        case kI3C_SlaveCompletionEvent:
            if (xfer->completionStatus == kStatus_Success)
            {
                if (g_lastTransferWasReceive)
                {
                    g_txBuff = g_slave_rxBuff;
                    g_txSize = (uint32_t)xfer->transferredCount;
                }
                g_slaveCompletionFlag = true;
            }
            break;

#if defined(I3C_ASYNC_WAKE_UP_INTR_CLEAR)
        /*  Handle async wake up interrupt on specific platform. */
        case kI3C_SlaveAddressMatchEvent:
            I3C_ASYNC_WAKE_UP_INTR_CLEAR
            break;
#endif

        default:
            break;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    i3c_slave_config_t slaveConfig;
    uint32_t eventMask = kI3C_SlaveAllEvents;
#if defined(I3C_ASYNC_WAKE_UP_INTR_CLEAR)
    eventMask |= kI3C_SlaveAddressMatchEvent;
#endif

//    /* Attach main clock to I3C, 396MHz / 4 = 99MHz. */
//    CLOCK_AttachClk(kMAIN_CLK_to_I3C_CLK);
//    CLOCK_SetClkDiv(kCLOCK_DivI3cClk, 4);


    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    //BOARD_InitSimulationClocks();
    
#ifdef ENABLE_PRINTF    
    BOARD_InitDebugConsole();
#endif
    
    CLOCK_AttachClk(kMAIN_CLK_to_I3C_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivI3cClk, 8);
    /* Attach lposc_1m clock to I3C time control, clear halt for slow clock. */
    CLOCK_AttachClk(kLPOSC_to_I3C_TC_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivI3cTcClk, 1);
    CLOCK_SetClkDiv(kCLOCK_DivI3cSlowClk, 1);
    
//    IOPCTL->PIO[2][29] = 0x141;
//    IOPCTL->PIO[2][30] = 0x141;
//    IOPCTL->PIO[2][31] = 0x001;
//    SYSCTL0->AUTOCLKGATEOVERRIDE0 = 0x0000003F;
//    SYSCTL0->AUTOCLKGATEOVERRIDE1 = 0x3FFFFFFF;

#ifdef ENABLE_PRINTF    
    PRINTF("\r\nI3C board2board interrupt example -- Slave transfer.\r\n");
    PRINTF("\r\n CPU Freq = %d\r\n",CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("\r\n I3C Freq = %d\r\n",CLOCK_GetFreq(kCLOCK_I3cClk));
#endif


    I3C_SlaveGetDefaultConfig(&slaveConfig);

    slaveConfig.staticAddr = I3C_MASTER_SLAVE_ADDR_7BIT;
    slaveConfig.vendorID   = 0x123U;
    slaveConfig.maxWriteLength = I3C_SLAVE_RX_DATA_LENGTH;
    slaveConfig.maxReadLength  = I3C_SLAVE_TX_DATA_LENGTH;
    slaveConfig.offline    = false;

    I3C_SlaveInit(EXAMPLE_SLAVE, &slaveConfig, I3C_SLAVE_CLOCK_FREQUENCY);

    /* Create slave handle. */
    I3C_SlaveTransferCreateHandle(EXAMPLE_SLAVE, &g_i3c_s_handle, i3c_slave_callback, NULL);

    g_txBuff = g_slave_txBuff;
    /* Start slave non-blocking transfer. */
    I3C_SlaveTransferNonBlocking(EXAMPLE_SLAVE, &g_i3c_s_handle, eventMask);


    uint32_t timeout_i = 0U;

#ifdef ENABLE_PRINTF    
    PRINTF("\r\nCheck I3C master I3C SDR transfer.\r\n");
#endif
    
#ifdef I3C_MASTER_DMA_TX_TEST
      
    /* For I3C SDR transfer check, master board will not send subaddress(device address). The first transfer is a
    I3C SDR write transfer, master will send one byte transmit size + several bytes transmit buffer content. */
    /* Wait for master transmit completed. */
    memset(g_slave_rxBuff, 0, I3C_SLAVE_RX_DATA_LENGTH);
    timeout_i = 0U;

    while ((g_slaveCompletionFlag != true) && (++timeout_i < I3C_TIME_OUT_INDEX))
     {
     }
    
    g_slaveCompletionFlag = false;

#ifdef ENABLE_PRINTF
    PRINTF("Slave will receive:");
    if (timeout_i == I3C_TIME_OUT_INDEX)
    {
        PRINTF("\r\nTransfer timeout.\r\n");
        for(uint32_t i = 0; i < I3C_SLAVE_RX_DATA_LENGTH;i++)
        {
            if (i % 8 == 0)
            {
                PRINTF("\r\n");
            }
            PRINTF("0x%2x  ", g_slave_rxBuff[i]);
        }
        return -1;
    }  
    
    for(uint32_t i = 0; i < I3C_SLAVE_RX_DATA_LENGTH;i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", g_slave_rxBuff[i]);
    }
    
    PRINTF("\r\nI3C slave I3C SDR transfer finished.\r\n");
#endif    
    
#endif
    
    
#ifdef I3C_MASTER_DMA_RX_TEST
    /* The second transfer is a I3C SDR read transfer, master will read back the transmit buffer content just sent. */
    /* Wait for slave transmit completed. */


     for (uint32_t i = 0U; i < I3C_SLAVE_TX_DATA_LENGTH; i++)
     {
         g_txBuff[i] = i + 1U;
     }
    g_txSize = I3C_SLAVE_TX_DATA_LENGTH;
    I3C_SlaveTransferNonBlocking(EXAMPLE_SLAVE, &g_i3c_s_handle, eventMask);

#ifdef ENABLE_PRINTF    
    PRINTF("Slave will send:");
    for(uint32_t i = 0; i < I3C_SLAVE_TX_DATA_LENGTH;i++)
    {
        if (i % 8 == 0)
        {
            PRINTF("\r\n");
        }
        PRINTF("0x%2x  ", g_txBuff[i]);
    }
#endif
    
    timeout_i = 0U;
    
    while ((g_slaveCompletionFlag != true) && (++timeout_i < I3C_TIME_OUT_INDEX))
    {
    }
    
    g_slaveCompletionFlag = false;
    if (timeout_i == I3C_TIME_OUT_INDEX)
    {
#ifdef ENABLE_PRINTF
        PRINTF("\r\nTransfer timeout.\r\n");
#endif
        return -1;
    }

#ifdef ENABLE_PRINTF
    PRINTF("\r\nI3C master I3C SDR transfer finished.\r\n");
#endif

#endif
    
    while (1)
    {
    }
}
