/*
 * Copyright 2022-2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef FSL_I3C_SMARTDMA_H_
#define FSL_I3C_SMARTDMA_H_

#include "fsl_i3c.h"
#include "fsl_smartdma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*! @{ */
/*! @brief I3C SMARTDMA driver version. */
#define FSL_I3C_SMARTDMA_DRIVER_VERSION (MAKE_VERSION(2, 1, 9))
/*@}*/

/*!
 * @addtogroup i3c_master_smartdma_driver
 * @{
 */

typedef struct _smartdma_transfer_param
{
    uint32_t *addr;
    uint32_t dataSize;
    uint32_t *i3cBaseAddress;
    uint32_t slave_address;
    volatile uint32_t *mailbox;
} smartdma_transfer_param_t;

/* Forward declaration of the transfer descriptor and handle typedefs. */
typedef struct _i3c_master_smartdma_handle i3c_master_smartdma_handle_t;

/*! @brief i3c master callback functions. */
typedef struct _i3c_master_smartdma_callback
{
    void (*slave2Master)(I3C_Type *base, void *userData); /*!< Transfer complete callback */
    void (*ibiCallback)(I3C_Type *base,
                        i3c_master_smartdma_handle_t *handle,
                        i3c_ibi_type_t ibiType,
                        i3c_ibi_state_t ibiState); /*!< IBI event callback */
    void (*transferComplete)(I3C_Type *base,
                             i3c_master_smartdma_handle_t *handle,
                             status_t status,
                             void *userData); /*!< Transfer complete callback */
} i3c_master_smartdma_callback_t;

/*!
 * @brief Driver handle for master DMA APIs.
 * @note The contents of this structure are private and subject to change.
 */
struct _i3c_master_smartdma_handle
{
    I3C_Type *base;                         /*!< I3C base pointer. */
    uint8_t state;                          /*!< Transfer state machine current state. */
    uint32_t transferCount;                 /*!< Indicates progress of the transfer */
    status_t smartdmaCompletionStatus;      /*!< Status reported by the SmartDMA completion callback. */
    uint32_t dataIrqMask;                   /*!< FIFO-ready IRQ source temporarily handed to SmartDMA. */
    bool smartdmaCompletionPending;         /*!< Tail-byte SmartDMA completion has not run yet. */
    bool smartdmaReadTailPending;           /*!< The final read byte still needs to be drained from the FIFO. */
    bool smartdmaDataWindowActive;          /*!< SmartDMA currently owns the FIFO-ready pacing source. */
    bool txTriggerAdjusted;                 /*!< Write-side TX watermark is temporarily overridden for SmartDMA pacing. */
    uint8_t savedTxTriggerLevel;            /*!< Original TX watermark restored when the SmartDMA data phase ends. */
    volatile uint32_t smartdmaMailbox;      /*!< SmartDMA-to-CM33 handoff reason written by the EZH program. */
    uint32_t smartdmaWindowIrqCount;        /*!< CM33 IRQ entries observed while the SmartDMA data window was active. */
    uint32_t smartdmaFifoReadyBounceCount;  /*!< Active-window IRQs treated as FIFO-ready fast returns. */
    uint32_t smartdmaProtocolBounceCount;   /*!< Active-window IRQs escalated back into the CM33 state machine. */
    uint32_t smartdmaMailboxProtocolCount;  /*!< Protocol escalations triggered by the SmartDMA mailbox reason. */
    uint32_t smartdmaWindowPendingMask;     /*!< OR'd pending interrupt mask observed during the SmartDMA data window. */
    uint32_t smartdmaWindowFifoMask;        /*!< OR'd FIFO-ready bits observed during the SmartDMA data window. */
    uint32_t smartdmaWindowProtocolMask;    /*!< OR'd protocol/escalation bits observed during the SmartDMA data window. */
    uint32_t smartdmaBounceStatus;          /*!< MSTATUS captured on the first active-window protocol bounce. */
    uint32_t smartdmaBounceErrStatus;       /*!< MERRWARN captured on the first active-window protocol bounce. */
    uint32_t smartdmaBounceDataCtrl;        /*!< MDATACTRL captured on the first active-window protocol bounce. */
    uint32_t smartdmaMailboxMaskedStatus;   /*!< MINTMASKED captured when EZH escalates with mailbox=protocol. */
    uint32_t smartdmaMailboxStatus;         /*!< MSTATUS captured when EZH escalates with mailbox=protocol. */
    uint32_t smartdmaMailboxErrStatus;      /*!< MERRWARN captured when EZH escalates with mailbox=protocol. */
    uint32_t smartdmaMailboxDataCtrl;       /*!< MDATACTRL captured when EZH escalates with mailbox=protocol. */
    uint8_t subaddressBuffer[4];            /*!< Saving subaddress command. */
    uint8_t subaddressCount;                /*!< Saving command count. */
    i3c_master_transfer_t transfer;         /*!< Copy of the current transfer info. */
    i3c_master_smartdma_callback_t callback; /*!< Callback function pointer. */
    void *userData;                         /*!< Application data passed to callback. */
    uint8_t ibiAddress;                     /*!< Slave address which request IBI. */
    uint8_t *ibiBuff;                       /*!< Pointer to IBI buffer to keep ibi bytes. */
    size_t ibiPayloadSize;                  /*!< IBI payload size. */
    i3c_ibi_type_t ibiType;                 /*!< IBI type. */
    smartdma_transfer_param_t smartdmaParam;
};

/*! @} */
/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @addtogroup i3c_master_smartdma_driver
 * @{
 */

/*! @name Master DMA */
/*! @{ */

/*!
 * @brief Create a new handle for the I3C master SmartDMA APIs.
 *
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master driver handle.
 * @param callback User provided pointer to the asynchronous callback function.
 * @param userData User provided pointer to the application callback data.
 */
void I3C_MasterTransferCreateHandleSmartDMA(I3C_Type *base,
                                            i3c_master_smartdma_handle_t *handle,
                                            const i3c_master_smartdma_callback_t *callback,
                                            void *userData);

/*!
 * @brief Performs a non-blocking SmartDMA-based transaction on the I3C bus.
 *
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master driver handle.
 * @param transfer The pointer to the transfer descriptor.
 * @retval kStatus_Success The transaction was started successfully.
 * @retval #kStatus_I3C_Busy Either another master is currently utilizing the bus, or another SmartDMA
 *      transaction is already in progress.
 */
status_t I3C_MasterTransferSmartDMA(I3C_Type *base,
                                    i3c_master_smartdma_handle_t *handle,
                                    i3c_master_transfer_t *transfer);

/*!
 * @brief Terminates a non-blocking I3C master transmission early.
 *
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master driver handle.
 */
void I3C_MasterTransferAbortSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle);

/*!
 * @brief Reusable routine to handle master interrupts.
 *
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master SmartDMA driver handle.
 */
void I3C_MasterTransferSmartDMAHandleIRQ(I3C_Type *base, void *i3cHandle);
/*! @} */

/*! @} */
#if defined(__cplusplus)
}
#endif

#endif /* FSL_I3C_SMARTDMA_H_ */