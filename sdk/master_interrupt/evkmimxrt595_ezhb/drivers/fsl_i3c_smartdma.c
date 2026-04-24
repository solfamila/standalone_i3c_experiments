/*
 * Copyright 2022-2025, 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_i3c_smartdma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.i3c_smartdma"
#endif



/*! @brief States for the state machine used by transactional APIs. */
enum _i3c_smartdma_transfer_states
{
    kIdleState = 0,
    kIBIWonState,
    kSlaveStartState,
    kSendCommandState,
    kWaitRepeatedStartCompleteState,
    kTransmitDataState,
    kReceiveDataState,
    kStopState,
    kWaitForCompletionState,
    kAddressMatchState,
};

/*! @brief Common sets of flags used by the driver. */
enum _i3c_smartdma_flag_constants
{
    /*! All flags which are cleared by the driver upon starting a transfer. */
    kMasterClearFlags = kI3C_MasterSlaveStartFlag | kI3C_MasterControlDoneFlag | kI3C_MasterCompleteFlag |
                        kI3C_MasterArbitrationWonFlag | kI3C_MasterSlave2MasterFlag | kI3C_MasterErrorFlag,

    /*! IRQ sources enabled by the non-blocking transactional API. */
    kMasterDMAIrqFlags = kI3C_MasterSlaveStartFlag | kI3C_MasterControlDoneFlag | kI3C_MasterCompleteFlag |
                         kI3C_MasterArbitrationWonFlag | kI3C_MasterErrorFlag | kI3C_MasterSlave2MasterFlag,

    /*! Errors to check for. */
    kMasterErrorFlags = kI3C_MasterErrorNackFlag | kI3C_MasterErrorWriteAbortFlag |
#if !defined(FSL_FEATURE_I3C_HAS_NO_MERRWARN_TERM) || (!FSL_FEATURE_I3C_HAS_NO_MERRWARN_TERM)
                        kI3C_MasterErrorTermFlag |
#endif
                        kI3C_MasterErrorParityFlag | kI3C_MasterErrorCrcFlag | kI3C_MasterErrorReadFlag |
                        kI3C_MasterErrorWriteFlag | kI3C_MasterErrorMsgFlag | kI3C_MasterErrorInvalidReqFlag |
                        kI3C_MasterErrorTimeoutFlag,
    /*! All flags which are cleared by the driver upon starting a transfer. */
    kSlaveClearFlags = kI3C_SlaveBusStartFlag | kI3C_SlaveMatchedFlag | kI3C_SlaveBusStopFlag,

    /*! IRQ sources enabled by the non-blocking transactional API. */
    kSlaveDMAIrqFlags = kI3C_SlaveBusStartFlag | kI3C_SlaveMatchedFlag |
                        kI3C_SlaveBusStopFlag | /*kI3C_SlaveRxReadyFlag |*/
                        kI3C_SlaveDynamicAddrChangedFlag | kI3C_SlaveReceivedCCCFlag | kI3C_SlaveErrorFlag |
                        kI3C_SlaveHDRCommandMatchFlag | kI3C_SlaveCCCHandledFlag | kI3C_SlaveEventSentFlag,

    /*! Errors to check for. */
    kSlaveErrorFlags = kI3C_SlaveErrorOverrunFlag | kI3C_SlaveErrorUnderrunFlag | kI3C_SlaveErrorUnderrunNakFlag |
                       kI3C_SlaveErrorTermFlag | kI3C_SlaveErrorInvalidStartFlag | kI3C_SlaveErrorSdrParityFlag |
                       kI3C_SlaveErrorHdrParityFlag | kI3C_SlaveErrorHdrCRCFlag | kI3C_SlaveErrorS0S1Flag |
                       kI3C_SlaveErrorOverreadFlag | kI3C_SlaveErrorOverwriteFlag,
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Array to map I3C instance number to base pointer. */
static I3C_Type *const kI3cBases[] = I3C_BASE_PTRS;

/*! @brief Array to store the END byte of I3C teransfer. */
static uint8_t i3cEndByte[ARRAY_SIZE(kI3cBases)] = {0};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void I3C_MasterRunSmartDMATransfer(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, void *data, size_t dataSize, i3c_direction_t direction);

/*******************************************************************************
 * Code
 ******************************************************************************/
void EZH_Callback(void *param)
{
    i3c_master_smartdma_handle_t *i3cHandle = (i3c_master_smartdma_handle_t *)param;
    uint32_t instance;
    if (i3cHandle->transfer.direction == kI3C_Read)
    {
        /* Disable I3C Rx DMA. */
        //i3cHandle->base->MDMACTRL &= ~I3C_MDMACTRL_DMAFB_MASK;
        I3C_MasterDisableInterrupts(i3cHandle->base, (uint32_t)kI3C_MasterRxReadyFlag);
        /* Terminate following data if present. */
        i3cHandle->base->MCTRL |= I3C_MCTRL_RDTERM(1U);
        size_t rxCount;
        do
        {
            I3C_MasterGetFifoCounts(i3cHandle->base, &rxCount, NULL);
        } while (rxCount == 0U);

        ((uint8_t *)i3cHandle->transfer.data)[i3cHandle->transfer.dataSize - 1U] = (uint8_t)i3cHandle->base->MRDATAB;
    }
    else if (i3cHandle->transfer.direction == kI3C_Write)
    {
        //i3cHandle->base->MDMACTRL &= ~I3C_MDMACTRL_DMATB_MASK;
        I3C_MasterDisableInterrupts(i3cHandle->base, (uint32_t)kI3C_MasterTxReadyFlag);

        instance = I3C_GetInstance(i3cHandle->base);
        /* Ensure there's space in the Tx FIFO. */
        while ((i3cHandle->base->MDATACTRL & I3C_MDATACTRL_TXFULL_MASK) != 0U)
        {
        }
        i3cHandle->base->MWDATABE = i3cEndByte[instance];
    }
}

static void I3C_MasterRunSmartDMATransfer(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, void *data, size_t dataSize, i3c_direction_t direction)
{
    bool isEnableTxDMA = false;
    bool isEnableRxDMA = false;
    handle->transferCount = dataSize;
    uint8_t slaveAddress = handle->transfer.slaveAddress;
    uint32_t width = 1;
    uint32_t instance;
    SMARTDMA_Reset();

    handle->smartdmaParam.addr = (uint32_t *)data;
    handle->smartdmaParam.dataSize = dataSize;
    handle->smartdmaParam.i3cBaseAddress = (uint32_t *)(uintptr_t)base;
    handle->smartdmaParam.slave_address  = slaveAddress;
    
    if (direction == kI3C_Write)
    {
        instance = I3C_GetInstance(base);
        if (dataSize != 1U)
        {
            //Use smartdma to transfer size - 1. And use smartdma callback to send the last byte.

            i3cEndByte[instance] = ((uint8_t *)data)[dataSize - 1U];
            dataSize--;
            handle->smartdmaParam.dataSize = dataSize;
            //Smartdma API
            I3C_MasterEnableInterrupts(base, (uint32_t)kI3C_MasterTxReadyFlag);
            SMARTDMA_Boot(kI3C_Write, &handle->smartdmaParam, 0);            
        }
        else
        {
            //Use smartdma to transfer 1 Byte ->Smartdma API
            handle->smartdmaParam.dataSize = 0;
            i3cEndByte[instance] = ((uint8_t *)data)[0];
            SMARTDMA_Boot(kI3C_Write, &handle->smartdmaParam, 0);
        }
        isEnableTxDMA = true;
    }
    else
    {
        dataSize--;
        handle->smartdmaParam.dataSize = dataSize;
        //Smartdma API
        I3C_MasterEnableInterrupts(base, (uint32_t)kI3C_MasterRxReadyFlag);
        SMARTDMA_Boot(kI3C_Read, &handle->smartdmaParam, 0);
        isEnableRxDMA = true;
    }

    //I3C_MasterEnableDMA(base, isEnableTxDMA, isEnableRxDMA, width);
}

static status_t I3C_MasterInitTransferStateMachineSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle)
{
    i3c_master_transfer_t *xfer = &handle->transfer;
    status_t result             = kStatus_Success;
    i3c_direction_t direction   = xfer->direction;

    if (xfer->busType != kI3C_TypeI3CDdr)
    {
        direction = (0UL != xfer->subaddressSize) ? kI3C_Write : xfer->direction;
    }
    /* Calculate command count and put into command buffer. */
    handle->subaddressCount = 0U;
    if (xfer->subaddressSize != 0U)
    {
        for (uint32_t i = xfer->subaddressSize; i > 0U; i--)
        {
            handle->subaddressBuffer[handle->subaddressCount++] = (uint8_t)((xfer->subaddress) >> (8U * (i - 1U)));
        }
    }

    /* Start condition shall be ommited, switch directly to next phase */
    if (xfer->dataSize == 0U)
    {
        handle->state = (uint8_t)kStopState;
    }

    if (0UL != (xfer->flags & (uint32_t)kI3C_TransferStartWithBroadcastAddr))
    {
        if (0UL != (xfer->flags & (uint32_t)kI3C_TransferNoStartFlag))
        {
            return kStatus_InvalidArgument;
        }

        if (0UL != (xfer->flags & (uint32_t)kI3C_TransferRepeatedStartFlag))
        {
            return kStatus_InvalidArgument;
        }

        /* Issue 0x7E as start. */
        result = I3C_MasterStart(base, xfer->busType, 0x7E, kI3C_Write);
        if (result != kStatus_Success)
        {
            return result;
        }

        result = I3C_MasterWaitForCtrlDone(base, false);
        if (result != kStatus_Success)
        {
            return result;
        }
    }

    /* Handle no start option. */
    if (0U != (xfer->flags & (uint32_t)kI3C_TransferNoStartFlag))
    {
        /* No need to send start flag, directly go to send command or data */
        if (xfer->subaddressSize > 0UL)
        {
            handle->state = (uint8_t)kSendCommandState;
        }
        else
        {
            if (direction == kI3C_Write)
            {
                /* Next state, send data. */
                handle->state = (uint8_t)kTransmitDataState;
            }
            else
            {
                /* Only support write with no stop signal. */
                return kStatus_InvalidArgument;
            }
        }
    }
    else
    {
        if (xfer->subaddressSize != 0U)
        {
            handle->state = (uint8_t)kSendCommandState;
        }
        else
        {
            if (handle->transfer.direction == kI3C_Write)
            {
                handle->state = (uint8_t)kTransmitDataState;
            }
            else if (handle->transfer.direction == kI3C_Read)
            {
                handle->state = (uint8_t)kReceiveDataState;
            }
            else
            {
                return kStatus_InvalidArgument;
            }
        }

        if ((handle->transfer.direction == kI3C_Read) && (xfer->subaddressSize == 0U))
        {
            I3C_MasterRunSmartDMATransfer(base, handle, xfer->data, xfer->dataSize, kI3C_Read);
        }

        if (handle->state != (uint8_t)kStopState)
        {
            /* If repeated start is requested, send repeated start. */
            if (0U != (xfer->flags & (uint32_t)kI3C_TransferRepeatedStartFlag))
            {
                result = I3C_MasterRepeatedStart(base, xfer->busType, xfer->slaveAddress, direction);
            }
            else /* For normal transfer, send start. */
            {
                result = I3C_MasterStart(base, xfer->busType, xfer->slaveAddress, direction);
            }
        }
    }

    I3C_MasterTransferSmartDMAHandleIRQ(base, handle);
    return result;
}

static status_t I3C_MasterRunTransferStateMachineSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle, bool *isDone)
{
    status_t result     = kStatus_Success;
    bool state_complete = false;
    size_t rxCount      = 0;
    i3c_master_transfer_t *xfer;
    uint32_t status;
    uint32_t errStatus;
    uint32_t statusToHandle;

    /* Set default isDone return value. */
    *isDone = false;

    /* Check for errors. */
    status = (uint32_t)I3C_MasterGetPendingInterrupts(base);

    statusToHandle = status & ~((uint32_t)kI3C_MasterTxReadyFlag | (uint32_t)kI3C_MasterRxReadyFlag);

    I3C_MasterClearStatusFlags(base, statusToHandle);

    i3c_master_state_t masterState = I3C_MasterGetState(base);
    errStatus                      = I3C_MasterGetErrorStatusFlags(base);
    result                         = I3C_MasterCheckAndClearError(base, errStatus);
    if (kStatus_Success != result)
    {
        return result;
    }

    if (0UL != (statusToHandle  & (uint32_t)kI3C_MasterSlave2MasterFlag))
    {
        if (handle->callback.slave2Master != NULL)
        {
            handle->callback.slave2Master(base, handle->userData);
        }
    }

    if ((0UL != (statusToHandle & (uint32_t)kI3C_MasterSlaveStartFlag)) && (handle->transfer.busType != kI3C_TypeI2C))
    {
        handle->state = (uint8_t)kSlaveStartState;
    }

    if ((masterState == kI3C_MasterStateIbiRcv) || (masterState == kI3C_MasterStateIbiAck))
    {
        handle->state = (uint8_t)kIBIWonState;
    }

    if (handle->state == (uint8_t)kIdleState)
    {
        return result;
    }

    if (handle->state == (uint8_t)kIBIWonState)
    {
        /* Get Rx fifo counts. */
        rxCount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT;
    }

    /* Get pointer to private data. */
    xfer = &handle->transfer;

    while (!state_complete)
    {
        /* Execute the state. */
        switch (handle->state)
        {
            case (uint8_t)kSlaveStartState:
                /* Emit start + 0x7E */
                I3C_MasterEmitRequest(base, kI3C_RequestAutoIbi);
                handle->state  = (uint8_t)kIBIWonState;
                state_complete = true;
                break;

            case (uint8_t)kIBIWonState:
                if (masterState == kI3C_MasterStateIbiAck)
                {
                    handle->ibiType = I3C_GetIBIType(base);
                    if (handle->callback.ibiCallback != NULL)
                    {
                        handle->callback.ibiCallback(base, handle, handle->ibiType, kI3C_IbiAckNackPending);
                    }
                    else
                    {
                        I3C_MasterEmitIBIResponse(base, kI3C_IbiRespNack);
                    }
                }

                /* Make sure there is data in the rx fifo. */
                if (0UL != rxCount)
                {
                    if ((handle->ibiBuff == NULL) && (handle->callback.ibiCallback != NULL))
                    {
                        handle->callback.ibiCallback(base, handle, kI3C_IbiNormal, kI3C_IbiDataBuffNeed);
                    }
                    uint8_t tempData = (uint8_t)(base->MRDATAB & 0xFFU);
                    if (handle->ibiBuff != NULL)
                    {
                        handle->ibiBuff[handle->ibiPayloadSize++] = tempData;
                    }
                    rxCount--;
                    break;
                }
                else if (0UL != (statusToHandle & (uint32_t)kI3C_MasterCompleteFlag))
                {
                    handle->ibiType    = I3C_GetIBIType(base);
                    handle->ibiAddress = I3C_GetIBIAddress(base);
                    state_complete     = true;
                    result             = kStatus_I3C_IBIWon;
                }
                else
                {
                    state_complete = true;
                }
                break;
            
            
            case (uint8_t)kSendCommandState:
                for (uint32_t i = 0; i < handle->subaddressCount; i++)
                {
                    result = I3C_MasterWaitForTxReady(base, 1U);
                    if (result != kStatus_Success)
                    {
                        return result;
                    }
                    
                    if (i == handle->subaddressCount - 1U)
                    {
                        if ((xfer->direction == kI3C_Read) || (xfer->dataSize == 0U))
                        {
                            base->MWDATABE = handle->subaddressBuffer[i];
                        }
                        else
                        {
                            base->MWDATAB = handle->subaddressBuffer[i];
                        }
                    }
                    else
                    {
                        base->MWDATAB = handle->subaddressBuffer[i];
                    }
                }
                
                if ((xfer->direction == kI3C_Read) || (0UL == xfer->dataSize))
                {
                    if (0UL == xfer->dataSize)
                    {
                        handle->state = (uint8_t)kWaitForCompletionState;
                    }
                    else
                    {
                        handle->state = (uint8_t)kWaitRepeatedStartCompleteState;
                    }
                }
                else
                {
                    handle->state = (uint8_t)kTransmitDataState;
                }
                
                state_complete = true;
                break;
 

            case (uint8_t)kWaitRepeatedStartCompleteState:
                /* We stay in this state until the maste complete. */
                if (0UL != (statusToHandle & (uint32_t)kI3C_MasterCompleteFlag))
                {
                    handle->state = (uint8_t)kReceiveDataState;
                    I3C_MasterRunSmartDMATransfer(base, handle, xfer->data, xfer->dataSize, kI3C_Read);
                    /* Send repeated start and slave address. */
                    result = I3C_MasterRepeatedStart(base, xfer->busType, xfer->slaveAddress, kI3C_Read);
                }

                state_complete = true;
                break;

            case (uint8_t)kTransmitDataState:
                I3C_MasterRunSmartDMATransfer(base, handle, xfer->data, xfer->dataSize, kI3C_Write);
                handle->state = (uint8_t)kWaitForCompletionState;

                state_complete = true;
                break;

            case (uint8_t)kReceiveDataState:
                /* Do DMA read. */
                handle->state = (uint8_t)kWaitForCompletionState;

                state_complete = true;
                break;

            case (uint8_t)kWaitForCompletionState:
                /* We stay in this state until the master complete. */
                if (0UL != (statusToHandle & (uint32_t)kI3C_MasterCompleteFlag))
                {
                    handle->state = (uint8_t)kStopState;
                }
                else
                {
                    state_complete = true;
                }
                break;

            case (uint8_t)kStopState:
                /* Only issue a stop transition if the caller requested it. */
                if (0UL == (xfer->flags & (uint32_t)kI3C_TransferNoStopFlag))
                {

                    if (xfer->busType == kI3C_TypeI3CDdr)
                    {
                        I3C_MasterEmitRequest(base, kI3C_RequestForceExit);
                    }
                    else
                    {
                        I3C_MasterEmitRequest(base, kI3C_RequestEmitStop);
                        result = I3C_MasterWaitForCtrlDone(base, false);
                    }
                }

                *isDone        = true;
                state_complete = true;

                break;

            default:
                //assert(false);
                break;
        }
    }
    return result;
}

void I3C_MasterTransferCreateHandleSmartDMA(I3C_Type *base,
                                       i3c_master_smartdma_handle_t *handle,
                                       const i3c_master_smartdma_callback_t *callback,
                                       void *userData)
{
    uint32_t instance;

    assert(NULL != handle);

    /* Clear out the handle. */
    (void)memset(handle, 0, sizeof(*handle));

    /* Look up instance number */
    instance = I3C_GetInstance(base);

    handle->base        = base;
    handle->callback    = *callback;
    handle->userData    = userData;

    /* Save this handle for IRQ use. */
    s_i3cMasterHandle[instance] = handle;

    /* Set irq handler. */
    s_i3cMasterIsr = I3C_MasterTransferSmartDMAHandleIRQ;

    /* Clear all flags. */
    I3C_MasterClearErrorStatusFlags(base, (uint32_t)kMasterErrorFlags);
    I3C_MasterClearStatusFlags(base, (uint32_t)kMasterClearFlags);
    /* Reset fifos. These flags clear automatically. */
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    /* Enable NVIC IRQ, this only enables the IRQ directly connected to the NVIC.
     In some cases the I3C IRQ is configured through INTMUX, user needs to enable
     INTMUX IRQ in application code. */
    (void)EnableIRQ(kI3cIrqs[instance]);
    NVIC_SetPriority(kI3cIrqs[instance], 4);

    /* Clear internal IRQ enables and enable NVIC IRQ. */
    I3C_MasterEnableInterrupts(base, (uint32_t)kMasterDMAIrqFlags);

    SMARTDMA_InstallCallback(EZH_Callback, handle);
}

/*!
 * brief Performs a non-blocking DMA transaction on the I2C/I3C bus.
 *
 * param base The I3C peripheral base address.
 * param handle Pointer to the I3C master driver handle.
 * param transfer The pointer to the transfer descriptor.
 * retval #kStatus_Success The transaction was started successfully.
 * retval #kStatus_I3C_Busy Either another master is currently utilizing the bus, or a non-blocking
 *      transaction is already in progress.
 */
status_t I3C_MasterTransferSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle, i3c_master_transfer_t *transfer)
{
    assert(NULL != handle);
    assert(NULL != transfer);

    i3c_master_state_t masterState = I3C_MasterGetState(base);
    bool checkDdrState;
    status_t result;

    /* Return busy if another transaction is in progress. */
    if (handle->state != (uint8_t)kIdleState)
    {
        return kStatus_I3C_Busy;
    }

    /* Return an error if the bus is already in use not by us. */
    checkDdrState = (transfer->busType == kI3C_TypeI3CDdr) ? (masterState != kI3C_MasterStateDdr) : true;
    if ((masterState != kI3C_MasterStateIdle) && (masterState != kI3C_MasterStateNormAct) && checkDdrState)
    {
        return kStatus_I3C_Busy;
    }

    /* Save transfer into handle. */
    handle->transfer = *transfer;

    /* Configure IBI response type. */
    base->MCTRL &= ~I3C_MCTRL_IBIRESP_MASK;
    base->MCTRL |= I3C_MCTRL_IBIRESP(transfer->ibiResponse);

    /* Clear all flags. */
    I3C_MasterClearErrorStatusFlags(base, (uint32_t)kMasterErrorFlags);
    I3C_MasterClearStatusFlags(base, (uint32_t)kMasterClearFlags);
    /* Reset fifos. These flags clear automatically. */
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    I3C_MasterEnableInterrupts(base, (uint32_t)kMasterDMAIrqFlags);

    /* Generate commands to send. */
    result = I3C_MasterInitTransferStateMachineSmartDMA(base, handle);
    if (result != kStatus_Success)
    {
        return result;
    }

    if (transfer->busType == kI3C_TypeI2C)
    {
        I3C_MasterDisableInterrupts(base, (uint32_t)kI3C_MasterSlaveStartFlag);
    }

    return kStatus_Success;
}

void I3C_MasterTransferSmartDMAHandleIRQ(I3C_Type *base, void *i3cHandle)
{
    i3c_master_smartdma_handle_t *handle = (i3c_master_smartdma_handle_t *)i3cHandle;
    status_t result;
    bool isDone;

    /* Don't do anything if the valid handle is not created. */
    if (NULL == handle)
    {
        return;
    }

    result = I3C_MasterRunTransferStateMachineSmartDMA(base, handle, &isDone);

    if (handle->state == (uint8_t)kIdleState)
    {
        return;
    }

    if (isDone || (result != kStatus_Success))
    {
        /* Terminate xfer when error or IBI event occurs */
        if ((result == kStatus_I3C_Nak) || (result == kStatus_I3C_IBIWon))
        {
            I3C_MasterEmitRequest(base, kI3C_RequestEmitStop);
            (void)I3C_MasterWaitForCtrlDone(base, false);
        }

        /* Set handle to idle state. */
        handle->state = (uint8_t)kIdleState;

        /* Invoke IBI user callback. */
        if ((result == kStatus_I3C_IBIWon) && (handle->callback.ibiCallback != NULL))
        {
            handle->callback.ibiCallback(base, handle, handle->ibiType, kI3C_IbiReady);
            handle->ibiPayloadSize = 0;
        }

        /* Invoke callback. */
        if (NULL != handle->callback.transferComplete)
        {
            handle->callback.transferComplete(base, handle, result, handle->userData);
        }
    }
}

/*!
 * brief Abort a master dma non-blocking transfer in a early time
 *
 * param base I3C peripheral base address
 * param handle pointer to i2c_master_dma_handle_t structure
 */
void I3C_MasterTransferAbortSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle)
{
    if (handle->state != (uint8_t)kIdleState)
    {
        SMARTDMA_Reset();

        /* Reset fifos. These flags clear automatically. */
        base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

        /* Send a stop command to finalize the transfer. */
        (void)I3C_MasterStop(base);

        /* Reset handle. */
        handle->state = (uint8_t)kIdleState;
    }
}
