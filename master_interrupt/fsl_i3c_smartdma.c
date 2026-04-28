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
    kSlaveDMAIrqFlags = kI3C_SlaveBusStartFlag | kI3C_SlaveMatchedFlag | kI3C_SlaveBusStopFlag |
                        kI3C_SlaveDynamicAddrChangedFlag | kI3C_SlaveReceivedCCCFlag | kI3C_SlaveErrorFlag |
                        kI3C_SlaveHDRCommandMatchFlag | kI3C_SlaveCCCHandledFlag | kI3C_SlaveEventSentFlag,

    /*! Errors to check for. */
    kSlaveErrorFlags = kI3C_SlaveErrorOverrunFlag | kI3C_SlaveErrorUnderrunFlag | kI3C_SlaveErrorUnderrunNakFlag |
                       kI3C_SlaveErrorTermFlag | kI3C_SlaveErrorInvalidStartFlag | kI3C_SlaveErrorSdrParityFlag |
                       kI3C_SlaveErrorHdrParityFlag | kI3C_SlaveErrorHdrCRCFlag | kI3C_SlaveErrorS0S1Flag |
                       kI3C_SlaveErrorOverreadFlag | kI3C_SlaveErrorOverwriteFlag,

    kMasterFifoReadyFlags = kI3C_MasterTxReadyFlag | kI3C_MasterRxReadyFlag,

    kMasterProtocolBounceFlags =
        kI3C_MasterSlaveStartFlag | kI3C_MasterArbitrationWonFlag | kI3C_MasterErrorFlag | kI3C_MasterSlave2MasterFlag,

    kSmartDMAMailboxIdle = 0U,
    kSmartDMAMailboxCompletion = 1U,
    kSmartDMAMailboxProtocol = 2U,
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Array to map I3C instance number to base pointer. */
static I3C_Type *const kI3cBases[] = I3C_BASE_PTRS;

/*! @brief Array to store the END byte of I3C transfer. */
static uint8_t i3cEndByte[ARRAY_SIZE(kI3cBases)] = {0};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t I3C_MasterSmartDMAWaitForTxReady(I3C_Type *base, uint8_t byteCounts);

static status_t I3C_MasterCompleteSmartDMAReadTail(i3c_master_smartdma_handle_t *handle);

static void I3C_MasterSmartDMASetDataWindowActive(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, bool isActive);

static void I3C_MasterSmartDMAStopDataPhase(I3C_Type *base, i3c_master_smartdma_handle_t *handle);

static void I3C_MasterSmartDMAEnterDataPhase(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, uint32_t dataIrqMask);

static void I3C_MasterSmartDMAExitDataPhase(I3C_Type *base, i3c_master_smartdma_handle_t *handle);

static bool I3C_MasterSmartDMAHandleActiveDataWindowIRQ(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, uint32_t pendingInterrupts);

static void I3C_MasterSmartDMASetTxTriggerLevel(I3C_Type *base, i3c_tx_trigger_level_t txLevel);

static void I3C_MasterRunSmartDMATransfer(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, void *data, size_t dataSize, i3c_direction_t direction);

static status_t I3C_MasterInitTransferStateMachineSmartDMA(I3C_Type *base,
                                                           i3c_master_smartdma_handle_t *handle);

static status_t I3C_MasterRunTransferStateMachineSmartDMA(I3C_Type *base,
                                                          i3c_master_smartdma_handle_t *handle,
                                                          bool *isDone);

/*******************************************************************************
 * Code
 ******************************************************************************/
static void I3C_MasterSmartDMASetDataWindowActive(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, bool isActive)
{
    handle->smartdmaDataWindowActive = isActive;
    I3C_MasterSetIrqDataWindowActive(base, isActive);
}

static void I3C_MasterSmartDMAStopDataPhase(I3C_Type *base, i3c_master_smartdma_handle_t *handle)
{
    I3C_MasterSmartDMASetDataWindowActive(base, handle, false);

    if (handle->dataIrqMask != 0U)
    {
        I3C_MasterDisableInterrupts(base, handle->dataIrqMask);
        handle->dataIrqMask = 0U;
    }

    if (handle->txTriggerAdjusted)
    {
        I3C_MasterSmartDMASetTxTriggerLevel(base, (i3c_tx_trigger_level_t)handle->savedTxTriggerLevel);
        handle->txTriggerAdjusted = false;
    }
}

static void I3C_MasterSmartDMAEnterDataPhase(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, uint32_t dataIrqMask)
{
    if (dataIrqMask == 0U)
    {
        return;
    }

    handle->dataIrqMask = dataIrqMask;
    I3C_MasterSmartDMASetDataWindowActive(base, handle, true);
    I3C_MasterDisableInterrupts(base, dataIrqMask);
}

static void I3C_MasterSmartDMAExitDataPhase(I3C_Type *base, i3c_master_smartdma_handle_t *handle)
{
    I3C_MasterSmartDMAStopDataPhase(base, handle);
}

static bool I3C_MasterSmartDMAHandleActiveDataWindowIRQ(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, uint32_t pendingInterrupts)
{
    uint32_t fifoReady;
    uint32_t protocol;

    if (!handle->smartdmaDataWindowActive)
    {
        return false;
    }

    fifoReady = pendingInterrupts & (uint32_t)kMasterFifoReadyFlags;
    protocol = pendingInterrupts & (uint32_t)kMasterProtocolBounceFlags;
    handle->smartdmaWindowIrqCount++;
    handle->smartdmaWindowPendingMask |= pendingInterrupts;
    handle->smartdmaWindowFifoMask |= fifoReady;
    handle->smartdmaWindowProtocolMask |= protocol;

    if ((handle->smartdmaMailbox == (uint32_t)kSmartDMAMailboxProtocol) || (protocol != 0U))
    {
        handle->smartdmaProtocolBounceCount++;
        if (handle->smartdmaMailbox == (uint32_t)kSmartDMAMailboxProtocol)
        {
            handle->smartdmaMailboxProtocolCount++;
        }
        handle->smartdmaMailbox = (uint32_t)kSmartDMAMailboxIdle;
        I3C_MasterSmartDMAStopDataPhase(base, handle);
        return false;
    }

    if (fifoReady != 0U)
    {
        handle->smartdmaFifoReadyBounceCount++;
    }

    return fifoReady != 0U;
}

static void I3C_MasterSmartDMASetTxTriggerLevel(I3C_Type *base, i3c_tx_trigger_level_t txLevel)
{
    uint32_t rxTriggerLevel;

    rxTriggerLevel = (base->MDATACTRL & I3C_MDATACTRL_RXTRIG_MASK) >> I3C_MDATACTRL_RXTRIG_SHIFT;
    I3C_MasterSetWatermarks(base, txLevel, (i3c_rx_trigger_level_t)rxTriggerLevel, false, false);
}

void EZH_Callback(void *param)
{
    i3c_master_smartdma_handle_t *i3cHandle = (i3c_master_smartdma_handle_t *)param;
    uint32_t instance;

    if (i3cHandle->smartdmaMailbox == (uint32_t)kSmartDMAMailboxProtocol)
    {
        I3C_MasterTransferSmartDMAHandleIRQ(i3cHandle->base, i3cHandle);
        return;
    }

    i3cHandle->smartdmaMailbox = (uint32_t)kSmartDMAMailboxIdle;

    I3C_MasterSmartDMAExitDataPhase(i3cHandle->base, i3cHandle);

    if (i3cHandle->transfer.direction == kI3C_Read)
    {
        /* Disable I3C Rx DMA. */
        i3cHandle->base->MDMACTRL &= ~I3C_MDMACTRL_DMAFB_MASK;
        if (i3cHandle->transfer.dataSize == 1U)
        {
            /* Single-byte reads do not pass through the EZH loop that arms RDTERM. */
            i3cHandle->base->MCTRL |= I3C_MCTRL_RDTERM(1U);
        }
        i3cHandle->smartdmaReadTailPending = true;
    }
    else if (i3cHandle->transfer.direction == kI3C_Write)
    {
        i3cHandle->base->MDMACTRL &= ~I3C_MDMACTRL_DMATB_MASK;

        instance = I3C_GetInstance(i3cHandle->base);
        while ((i3cHandle->base->MDATACTRL & I3C_MDATACTRL_TXFULL_MASK) != 0U)
        {
        }
        i3cHandle->base->MWDATABE = i3cEndByte[instance];

#if defined(EXPERIMENT_MASTER_SMARTDMA_TAIL_WAIT)
        {
            uint32_t waitCount = 0U;

            while (((I3C_MasterGetStatusFlags(i3cHandle->base) & (uint32_t)kI3C_MasterCompleteFlag) == 0U) &&
                   (waitCount < 1000000U))
            {
                waitCount++;
            }
        }
#endif
    }

    i3cHandle->smartdmaCompletionPending = false;
    I3C_MasterTransferSmartDMAHandleIRQ(i3cHandle->base, i3cHandle);
}

static status_t I3C_MasterSmartDMAWaitForTxReady(I3C_Type *base, uint8_t byteCounts)
{
    uint32_t errStatus;
    status_t result;
    size_t txCount;
    size_t txFifoSize =
        2UL << ((base->SCAPABILITIES & I3C_SCAPABILITIES_FIFOTX_MASK) >> I3C_SCAPABILITIES_FIFOTX_SHIFT);

#if I3C_RETRY_TIMES
    uint32_t waitTimes = I3C_RETRY_TIMES;
#endif

    do
    {
        I3C_MasterGetFifoCounts(base, NULL, &txCount);
        assert(txFifoSize >= txCount);
        txCount = txFifoSize - txCount;

        errStatus = I3C_MasterGetErrorStatusFlags(base);
        result = I3C_MasterCheckAndClearError(base, errStatus);
        if (kStatus_Success != result)
        {
            return result;
        }
#if I3C_RETRY_TIMES
    } while ((txCount < byteCounts) && (--waitTimes));

    if (waitTimes == 0U)
    {
        return kStatus_I3C_Timeout;
    }
#else
    } while (txCount < byteCounts);
#endif

    return kStatus_Success;
}

static status_t I3C_MasterCompleteSmartDMAReadTail(i3c_master_smartdma_handle_t *handle)
{
    size_t rxCount;
    uint32_t waitCount = 0U;

    do
    {
        I3C_MasterGetFifoCounts(handle->base, &rxCount, NULL);
        waitCount++;
    } while ((rxCount == 0U) && (waitCount < 1000000U));

    if (rxCount == 0U)
    {
        return kStatus_I3C_Timeout;
    }

    ((uint8_t *)handle->transfer.data)[handle->transfer.dataSize - 1U] = (uint8_t)handle->base->MRDATAB;

    handle->smartdmaReadTailPending = false;
    return kStatus_Success;
}

static void I3C_MasterRunSmartDMATransfer(
    I3C_Type *base, i3c_master_smartdma_handle_t *handle, void *data, size_t dataSize, i3c_direction_t direction)
{
    bool isEnableTxDMA = false;
    bool isEnableRxDMA = false;
    uint32_t dataIrqMask = 0U;
    uint32_t width = 1U;
    uint32_t instance;

    handle->transferCount = dataSize;
    handle->smartdmaCompletionStatus = kStatus_Success;
    handle->smartdmaCompletionPending = true;
    handle->smartdmaReadTailPending = false;
    handle->dataIrqMask = 0U;
    handle->smartdmaDataWindowActive = false;
    handle->txTriggerAdjusted = false;
    handle->smartdmaMailbox = (uint32_t)kSmartDMAMailboxIdle;
    handle->smartdmaWindowIrqCount = 0U;
    handle->smartdmaFifoReadyBounceCount = 0U;
    handle->smartdmaProtocolBounceCount = 0U;
    handle->smartdmaMailboxProtocolCount = 0U;
    handle->smartdmaWindowPendingMask = 0U;
    handle->smartdmaWindowFifoMask = 0U;
    handle->smartdmaWindowProtocolMask = 0U;
    handle->smartdmaParam.addr = (uint32_t *)data;
    handle->smartdmaParam.dataSize = dataSize;
    handle->smartdmaParam.i3cBaseAddress = (uint32_t *)(uintptr_t)base;
    handle->smartdmaParam.slave_address = handle->transfer.slaveAddress;
    handle->smartdmaParam.mailbox = &handle->smartdmaMailbox;

    SMARTDMA_Reset();

    if (direction == kI3C_Write)
    {
        instance = I3C_GetInstance(base);
        dataIrqMask = (uint32_t)kI3C_MasterTxReadyFlag;
        handle->savedTxTriggerLevel =
            (uint8_t)((base->MDATACTRL & I3C_MDATACTRL_TXTRIG_MASK) >> I3C_MDATACTRL_TXTRIG_SHIFT);
        I3C_MasterSmartDMASetTxTriggerLevel(base, kI3C_TxTriggerOnEmpty);
        handle->txTriggerAdjusted = true;
        if (dataSize != 1U)
        {
            i3cEndByte[instance] = ((uint8_t *)data)[dataSize - 1U];
            dataSize--;
            handle->smartdmaParam.dataSize = dataSize;
        }
        else
        {
            handle->smartdmaParam.dataSize = 0U;
            i3cEndByte[instance] = ((uint8_t *)data)[0];
        }
        isEnableTxDMA = true;
    }
    else
    {
        dataIrqMask = (uint32_t)kI3C_MasterRxReadyFlag;
        dataSize--;
        handle->smartdmaParam.dataSize = dataSize;
        isEnableRxDMA = true;
    }

    if (direction == kI3C_Write)
    {
        /* Writes enter the SmartDMA-owned FIFO pacing window immediately. */
        I3C_MasterSmartDMAEnterDataPhase(base, handle, dataIrqMask);
    }
    else
    {
        /* Reads defer the FIFO pacing window until the repeated-start receive phase is active. */
        handle->dataIrqMask = dataIrqMask;
    }

    if (direction == kI3C_Write)
    {
        SMARTDMA_Boot(kI3C_Write, &handle->smartdmaParam, 0);
    }
    else
    {
        SMARTDMA_Boot(kI3C_Read, &handle->smartdmaParam, 0);
    }

    I3C_MasterEnableDMA(base, isEnableTxDMA, isEnableRxDMA, width);
}

static status_t I3C_MasterInitTransferStateMachineSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle)
{
    i3c_master_transfer_t *xfer = &handle->transfer;
    status_t result = kStatus_Success;
    i3c_direction_t direction = xfer->direction;

    if (xfer->busType != kI3C_TypeI3CDdr)
    {
        direction = (0UL != xfer->subaddressSize) ? kI3C_Write : xfer->direction;
    }

    handle->subaddressCount = 0U;
    if (xfer->subaddressSize != 0U)
    {
        for (uint32_t i = xfer->subaddressSize; i > 0U; i--)
        {
            handle->subaddressBuffer[handle->subaddressCount++] = (uint8_t)(xfer->subaddress >> (8U * (i - 1U)));
        }
    }

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

        result = I3C_MasterStart(base, xfer->busType, 0x7EU, kI3C_Write);
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

    if (0U != (xfer->flags & (uint32_t)kI3C_TransferNoStartFlag))
    {
        if (xfer->subaddressSize > 0UL)
        {
            handle->state = (uint8_t)kSendCommandState;
        }
        else if (direction == kI3C_Write)
        {
            handle->state = (uint8_t)kTransmitDataState;
        }
        else
        {
            return kStatus_InvalidArgument;
        }
    }
    else
    {
        if (xfer->subaddressSize != 0U)
        {
            handle->state = (uint8_t)kSendCommandState;
        }
        else if (handle->transfer.direction == kI3C_Write)
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

        if ((handle->transfer.direction == kI3C_Read) && (xfer->subaddressSize == 0U))
        {
            I3C_MasterRunSmartDMATransfer(base, handle, xfer->data, xfer->dataSize, kI3C_Read);
            I3C_MasterSmartDMAEnterDataPhase(base, handle, handle->dataIrqMask);
        }

        if (handle->state != (uint8_t)kStopState)
        {
            if (0U != (xfer->flags & (uint32_t)kI3C_TransferRepeatedStartFlag))
            {
                result = I3C_MasterRepeatedStart(base, xfer->busType, xfer->slaveAddress, direction);
            }
            else
            {
                result = I3C_MasterStart(base, xfer->busType, xfer->slaveAddress, direction);
            }

            if (result != kStatus_Success)
            {
                return result;
            }
        }
    }

    if (handle->state == (uint8_t)kAddressMatchState)
    {
        if (0UL == (xfer->flags & (uint32_t)kI3C_TransferNoStopFlag))
        {
            result = I3C_MasterWaitForCtrlDone(base, false);
            if (result != kStatus_Success)
            {
                return result;
            }
        }

        I3C_MasterTransferSmartDMAHandleIRQ(base, handle);
    }

    return result;
}

static status_t I3C_MasterRunTransferStateMachineSmartDMA(I3C_Type *base,
                                                          i3c_master_smartdma_handle_t *handle,
                                                          bool *isDone)
{
    status_t result = kStatus_Success;
    bool state_complete = false;
    size_t rxCount = 0U;
    i3c_master_transfer_t *xfer;
    uint32_t status;
    uint32_t errStatus;
    uint32_t statusToHandle;
    i3c_master_state_t masterState;

    *isDone = false;

    status = (uint32_t)I3C_MasterGetPendingInterrupts(base);
    statusToHandle = status & ~((uint32_t)kI3C_MasterTxReadyFlag | (uint32_t)kI3C_MasterRxReadyFlag);
    I3C_MasterClearStatusFlags(base, statusToHandle);

    masterState = I3C_MasterGetState(base);
    errStatus = I3C_MasterGetErrorStatusFlags(base);
    result = I3C_MasterCheckAndClearError(base, errStatus);
    if (kStatus_Success != result)
    {
        return result;
    }

    if ((handle->state == (uint8_t)kWaitForCompletionState) && handle->smartdmaCompletionPending &&
        handle->smartdmaDataWindowActive &&
        (statusToHandle == 0U) && (errStatus == 0U))
    {
        return result;
    }

    if (0UL != (statusToHandle & (uint32_t)kI3C_MasterSlave2MasterFlag))
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
        rxCount = (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK) >> I3C_MDATACTRL_RXCOUNT_SHIFT;
    }

    xfer = &handle->transfer;

    while (!state_complete)
    {
        switch (handle->state)
        {
            case (uint8_t)kSlaveStartState:
                I3C_MasterEmitRequest(base, kI3C_RequestAutoIbi);
                handle->state = (uint8_t)kIBIWonState;
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
                    handle->ibiType = I3C_GetIBIType(base);
                    handle->ibiAddress = I3C_GetIBIAddress(base);
                    state_complete = true;
                    result = kStatus_I3C_IBIWon;
                }
                else
                {
                    state_complete = true;
                }
                break;

            case (uint8_t)kSendCommandState:
                for (uint32_t i = 0U; i < handle->subaddressCount; i++)
                {
                    result = I3C_MasterSmartDMAWaitForTxReady(base, 1U);
                    if (result != kStatus_Success)
                    {
                        return result;
                    }

                    if (i == (uint32_t)(handle->subaddressCount - 1U))
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
                if (0UL != (statusToHandle & (uint32_t)kI3C_MasterCompleteFlag))
                {
                    handle->state = (uint8_t)kReceiveDataState;
                    I3C_MasterRunSmartDMATransfer(base, handle, xfer->data, xfer->dataSize, kI3C_Read);
                    I3C_MasterSmartDMAEnterDataPhase(base, handle, handle->dataIrqMask);
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
                if ((handle->transfer.direction == kI3C_Read) && !handle->smartdmaDataWindowActive &&
                    (handle->dataIrqMask != 0U))
                {
                    /* The receive data window starts only after repeated start completes. */
                    I3C_MasterSmartDMAEnterDataPhase(base, handle, handle->dataIrqMask);
                }
                handle->state = (uint8_t)kWaitForCompletionState;
                state_complete = true;
                break;

            case (uint8_t)kWaitForCompletionState:
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

                *isDone = true;
                state_complete = true;
                break;

            default:
                assert(false);
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

    (void)memset(handle, 0, sizeof(*handle));

    instance = I3C_GetInstance(base);
    handle->base = base;
    handle->callback = *callback;
    handle->userData = userData;

    s_i3cMasterHandle[instance] = handle;
    s_i3cMasterIsr = I3C_MasterTransferSmartDMAHandleIRQ;

    I3C_MasterClearErrorStatusFlags(base, (uint32_t)kMasterErrorFlags);
    I3C_MasterClearStatusFlags(base, (uint32_t)kMasterClearFlags);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    (void)EnableIRQ(kI3cIrqs[instance]);
    I3C_MasterEnableInterrupts(base, (uint32_t)kMasterDMAIrqFlags);

    SMARTDMA_InstallCallback(EZH_Callback, handle);
}

status_t I3C_MasterTransferSmartDMA(I3C_Type *base,
                                    i3c_master_smartdma_handle_t *handle,
                                    i3c_master_transfer_t *transfer)
{
    i3c_master_state_t masterState;
    bool checkDdrState;
    status_t result;

    assert(NULL != handle);
    assert(NULL != transfer);

    masterState = I3C_MasterGetState(base);

    if (handle->state != (uint8_t)kIdleState)
    {
        return kStatus_I3C_Busy;
    }

    checkDdrState = (transfer->busType == kI3C_TypeI3CDdr) ? (masterState != kI3C_MasterStateDdr) : true;
    if ((masterState != kI3C_MasterStateIdle) && (masterState != kI3C_MasterStateNormAct) && checkDdrState)
    {
        return kStatus_I3C_Busy;
    }

    handle->transfer = *transfer;

    base->MCTRL &= ~I3C_MCTRL_IBIRESP_MASK;
    base->MCTRL |= I3C_MCTRL_IBIRESP(transfer->ibiResponse);

    I3C_MasterClearErrorStatusFlags(base, (uint32_t)kMasterErrorFlags);
    I3C_MasterClearStatusFlags(base, (uint32_t)kMasterClearFlags);
    base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;

    I3C_MasterEnableInterrupts(base, (uint32_t)kMasterDMAIrqFlags);

    result = I3C_MasterInitTransferStateMachineSmartDMA(base, handle);
    if (result != kStatus_Success)
    {
        I3C_MasterSmartDMAExitDataPhase(base, handle);
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
    uint32_t pendingInterrupts;

    if (NULL == handle)
    {
        return;
    }

    pendingInterrupts = I3C_MasterGetPendingInterrupts(base);
    if (I3C_MasterSmartDMAHandleActiveDataWindowIRQ(base, handle, pendingInterrupts))
    {
        return;
    }

    result = I3C_MasterRunTransferStateMachineSmartDMA(base, handle, &isDone);

    if ((result == kStatus_Success) && (handle->smartdmaCompletionStatus != kStatus_Success))
    {
        result = handle->smartdmaCompletionStatus;
        isDone = true;
    }

    /* The I3C completion IRQ can arrive before SmartDMA has handled the tail byte. */
    if (isDone && (result == kStatus_Success) && handle->smartdmaCompletionPending &&
        handle->smartdmaDataWindowActive)
    {
        return;
    }

    /*
     * On read transfers the protocol-side NAK can be observed before EZH has
     * returned and armed the deferred tail handling. Do not finalize the
     * transfer on that first NAK while SmartDMA completion is still pending.
     */
    if ((result == kStatus_I3C_Nak) && handle->smartdmaCompletionPending &&
        !handle->smartdmaReadTailPending && (handle->transfer.direction == kI3C_Read))
    {
        handle->smartdmaCompletionStatus = kStatus_I3C_Nak;

        /* Once the SmartDMA data window has already closed, there is no later
         * EZH completion edge left to retire this read-side NAK. Finalize it
         * now instead of leaving the transfer stuck in WaitForCompletion.
         */
        if (handle->smartdmaDataWindowActive)
        {
            return;
        }

        isDone = true;
    }

    if (isDone && (result == kStatus_Success) && handle->smartdmaReadTailPending)
    {
        result = I3C_MasterCompleteSmartDMAReadTail(handle);
    }

    if ((result == kStatus_I3C_Nak) && handle->smartdmaReadTailPending &&
        (handle->smartdmaCompletionStatus == kStatus_Success))
    {
        result = I3C_MasterCompleteSmartDMAReadTail(handle);
        isDone = true;
    }

    if ((handle->state == (uint8_t)kIdleState) && !isDone && (result == kStatus_Success) &&
        !handle->smartdmaReadTailPending)
    {
        return;
    }

    if (isDone || (result != kStatus_Success))
    {
        I3C_MasterSmartDMAExitDataPhase(base, handle);

        if ((result == kStatus_I3C_Nak) || (result == kStatus_I3C_IBIWon))
        {
            I3C_MasterEmitRequest(base, kI3C_RequestEmitStop);
            (void)I3C_MasterWaitForCtrlDone(base, false);
        }

        handle->state = (uint8_t)kIdleState;
        handle->smartdmaCompletionPending = false;
        handle->smartdmaReadTailPending = false;

        if ((result == kStatus_I3C_IBIWon) && (handle->callback.ibiCallback != NULL))
        {
            handle->callback.ibiCallback(base, handle, handle->ibiType, kI3C_IbiReady);
            handle->ibiPayloadSize = 0U;
        }

        if (NULL != handle->callback.transferComplete)
        {
            handle->callback.transferComplete(base, handle, result, handle->userData);
        }

        handle->smartdmaCompletionStatus = kStatus_Success;
    }
}

void I3C_MasterTransferAbortSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle)
{
    if (handle->state != (uint8_t)kIdleState)
    {
        SMARTDMA_Reset();
        base->MDATACTRL |= I3C_MDATACTRL_FLUSHTB_MASK | I3C_MDATACTRL_FLUSHFB_MASK;
        I3C_MasterSmartDMAExitDataPhase(base, handle);
        (void)I3C_MasterStop(base);
        handle->state = (uint8_t)kIdleState;
    }
}