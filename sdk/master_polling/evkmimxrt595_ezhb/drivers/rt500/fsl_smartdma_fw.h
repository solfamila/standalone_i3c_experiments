/*
 * Copyright 2019-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef FSL_SMARTDMA_RT500_H_
#define FSL_SMARTDMA_RT500_H_

#include "fsl_common.h"
typedef struct
{
    uint8_t RESERVED_0[32];
    __IO uint32_t BOOTADR;    /* 0x20 */
    __IO uint32_t CTRL;       /* 0x24 */
    __I uint32_t PC;          /* 0x28 */
    __I uint32_t SP;          /* 0x2C */
    __IO uint32_t BREAK_ADDR; /* 0x30 */
    __IO uint32_t BREAK_VECT; /* 0x34 */
    __IO uint32_t EMER_VECT;  /* 0x38 */
    __IO uint32_t EMER_SEL;   /* 0x3C */
    __IO uint32_t ARM2EZH;    /* 0x40 */
    __IO uint32_t EZH2ARM;    /* 0x44 */
    __IO uint32_t PENDTRAP;   /* 0x48 */
} SMARTDMA_Type;

#define SMARTDMA_BASE 0x40027000
#define SMARTDMA      ((volatile SMARTDMA_Type *)SMARTDMA_BASE)

#define SMARTDMA_SRAM_ADDR 0x24100000U


/*******************************************************************************
 * APIs
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* FSL_SMARTDMA_RT500_H_ */
