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
    I3C_Type *base;                     /*!< I3C base pointer. */
    uint8_t state;                      /*!< Transfer state machine current state. */
    uint32_t transferCount;             /*!< Indicates progress of the transfer */
    uint8_t subaddressBuffer[4];        /*!< Saving subaddress command. */
    uint8_t subaddressCount;            /*!< Saving command count. */
    i3c_master_transfer_t transfer;     /*!< Copy of the current transfer info. */
    i3c_master_smartdma_callback_t callback; /*!< Callback function pointer. */
    void *userData;                     /*!< Application data passed to callback. */
    uint8_t ibiAddress;                 /*!< Slave address which request IBI. */
    uint8_t *ibiBuff;                   /*!< Pointer to IBI buffer to keep ibi bytes. */
    size_t ibiPayloadSize;              /*!< IBI payload size. */
    i3c_ibi_type_t ibiType;             /*!< IBI type. */
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
 * The creation of a handle is for use with the DMA APIs. Once a handle
 * is created, there is not a corresponding destroy handle. If the user wants to
 * terminate a transfer, the I3C_MasterTransferAbortDMA() API shall be called.
 *
 * For devices where the I3C send and receive DMA requests are OR'd together, the @a txDmaHandle
 * parameter is ignored and may be set to NULL.
 *
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master driver handle.
 * @param callback User provided pointer to the asynchronous callback function.
 * @param userData User provided pointer to the application callback data.
 * @param rxDmaHandle Handle for the DMA receive channel. Created by the user prior to calling this function.
 * @param txDmaHandle Handle for the DMA transmit channel. Created by the user prior to calling this function.
 */
void I3C_MasterTransferCreateHandleSmartDMA(I3C_Type *base,
                                       i3c_master_smartdma_handle_t *handle,
                                       const i3c_master_smartdma_callback_t *callback,
                                       void *userData);


/*!
 * @brief Performs a non-blocking DMA-based transaction on the I3C bus.
 *
 * The callback specified when the @a handle was created is invoked when the transaction has
 * completed.
 *
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master driver handle.
 * @param transfer The pointer to the transfer descriptor.
 * @retval kStatus_Success The transaction was started successfully.
 * @retval #kStatus_I3C_Busy Either another master is currently utilizing the bus, or another DMA
 *      transaction is already in progress.
 */
status_t I3C_MasterTransferSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle, i3c_master_transfer_t *transfer);

/*!
 * @brief Terminates a non-blocking I3C master transmission early.
 *
 * @note It is not safe to call this function from an IRQ handler that has a higher priority than the
 *      DMA peripheral's IRQ priority.
 *
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master driver handle.
 */
void I3C_MasterTransferAbortSmartDMA(I3C_Type *base, i3c_master_smartdma_handle_t *handle);

/*!
 * @brief Reusable routine to handle master interrupts.
 * @note This function does not need to be called unless you are reimplementing the
 *  nonblocking API's interrupt handler routines to add special functionality.
 * @param base The I3C peripheral base address.
 * @param handle Pointer to the I3C master DMA driver handle.
 */
void I3C_MasterTransferSmartDMAHandleIRQ(I3C_Type *base, void *i3cHandle);
/*! @} */



/*! @} */
#if defined(__cplusplus)
}
#endif

#endif /* FSL_I3C_DMA_H_ */
