#include "fsl_SMARTDMA_armclang.h"

#define SMARTDMA_CODE __attribute__((used, section(".smartdma_code"), aligned(4)))
#define SMARTDMA_DATA __attribute__((used, section(".smartdma_data"), aligned(4)))

#define EZH_ARM2EZH 0x27040

void SMARTDMA_CODE EZHB_DmaIrqProof(void);

SMARTDMA_DATA void (*g_SMARTDMA_api[16])(void);

void SMARTDMA_CODE EZHB_DmaIrqProof(void)
{
    E_NOP;
    E_NOP;

    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);
    E_LDR(R0, R6, 0); /* mailbox address */

    E_LOAD_IMM(CFS, 0x0);
    E_LOAD_IMM(CFM, 0x101);

    E_HOLD;
    E_BCLR_IMM(CFM, CFM, 0);

    E_LDR(R1, R0, 0);
    E_ADD_IMM(R1, R1, 1);
    E_STR(R0, R1, 0);
    E_INT_TRIGGER(0);

E_LABEL("end");
    E_NOP;
    E_GOSUB(end);
}

void keep_smartdma_api_alive(void)
{
    volatile void *ptr = &g_SMARTDMA_api;

    g_SMARTDMA_api[0] = (void (*)(void))(((uint32_t)EZHB_DmaIrqProof + 4U) & (~3U));

    (void)ptr;
}