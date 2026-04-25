#include "fsl_SMARTDMA_armclang.h"

#define SMARTDMA_CODE __attribute__((used, section(".smartdma_code"), aligned(4)))
#define SMARTDMA_DATA __attribute__((used, section(".smartdma_data"), aligned(4)))

#define EZH_ARM2EZH 0x27040

#define I3C_MDATACTRL_OFFSET 0xAC
#define I3C_MWDATABE_OFFSET 0xB4

typedef struct _i3c_dma_irq_probe_param
{
    volatile uint32_t *mailbox;
    uint32_t *i3cBaseAddress;
    uint32_t lastByte;
} i3c_dma_irq_probe_param_t;

void SMARTDMA_CODE EZHB_I3cDmaTxComplete(void);

SMARTDMA_DATA void (*g_SMARTDMA_api[16])(void);

void SMARTDMA_CODE EZHB_I3cDmaTxComplete(void)
{
    E_NOP;
    E_NOP;

    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);

    E_LDR(R0, R6, 0); /* mailbox address */
    E_LDR(R1, R6, 1); /* I3C base address */
    E_LDR(R2, R6, 2); /* last byte */

    E_LOAD_IMM(CFS, 0x0);
    E_LOAD_IMM(CFM, 0x101);

    E_HOLD;
    E_BCLR_IMM(CFM, CFM, 0);

    E_LOAD_SIMM(R5, I3C_MDATACTRL_OFFSET, 0);
    E_ADD(R3, R1, R5);
    E_LOAD_SIMM(R4, 0x40, 24);
    E_ADD_IMM(R7, PC, -4);

E_LABEL("wait_tx_room");
    E_LDR(R5, R3, 0);
    E_ANDS(R5, R5, R4);
    E_COND_GOTO_REGL(NZ, R7);

    E_LOAD_SIMM(R5, I3C_MWDATABE_OFFSET, 0);
    E_ADD(R3, R1, R5);
    E_STR(R3, R2, 0);

    E_LDR(R5, R0, 0);
    E_ADD_IMM(R5, R5, 1);
    E_STR(R0, R5, 0);

    E_INT_TRIGGER(0);

E_LABEL("end");
    E_NOP;
    E_GOSUB(end);
}

void keep_smartdma_api_alive(void)
{
    volatile void *ptr = &g_SMARTDMA_api;

    g_SMARTDMA_api[0] = (void (*)(void))(((uint32_t)EZHB_I3cDmaTxComplete + 4U) & (~3U));

    (void)ptr;
}