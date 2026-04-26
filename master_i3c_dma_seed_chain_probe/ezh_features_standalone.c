#include "fsl_SMARTDMA_armclang.h"

#define SMARTDMA_CODE __attribute__((used, section(".smartdma_code"), aligned(4)))
#define SMARTDMA_DATA __attribute__((used, section(".smartdma_data"), aligned(4)))

#define EZH_ARM2EZH 0x27040

#define I3C_MWDATAB_OFFSET 0xB0
#define I3C_MWDATABE_OFFSET 0xB4

#define PARAM_MAILBOX_INDEX 0
#define PARAM_EXPECTED_WAKE_COUNT_INDEX 1
#define PARAM_WAKE_COUNT_INDEX 2
#define PARAM_DMA_SEED_BYTES_INDEX 3
#define PARAM_SMARTDMA_BYTES_INDEX 4
#define PARAM_DMA_INTA_COUNT_INDEX 5
#define PARAM_NEXT_TX_BYTE_INDEX 6
#define PARAM_REMAINING_COUNT_INDEX 7
#define PARAM_I3C_BASE_ADDRESS_INDEX 8
#define PARAM_DMA_INTA_ADDRESS_INDEX 9
#define PARAM_DMA_CHANNEL_MASK_INDEX 10

void SMARTDMA_CODE EZHB_I3cDmaSeedChainProbe(void);

SMARTDMA_DATA void (*g_SMARTDMA_api[16])(void);

void SMARTDMA_CODE EZHB_I3cDmaSeedChainProbe(void)
{
    E_NOP;
    E_NOP;

    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);

    E_LOAD_IMM(CFS, 0x0);
    E_LOAD_IMM(CFM, 0x101);

    E_ADD_IMM(R7, PC, 0);
    E_ADD_IMM(PC, PC, 3 * 4);

    E_DCD(wait_dma_irq);
    E_DCD(fill_tail_bytes);
    E_DCD(signal_completion);
    E_DCD(end);

    E_LDR(R5, R7, 0);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("wait_dma_irq");
    E_HOLD;
    E_BCLR_IMM(CFM, CFM, 0);

    E_LDR(R0, R6, PARAM_WAKE_COUNT_INDEX);
    E_ADD_IMM(R0, R0, 1);
    E_STR(R6, R0, PARAM_WAKE_COUNT_INDEX);

    E_LDR(R1, R6, PARAM_DMA_SEED_BYTES_INDEX);
    E_ADD_IMM(R1, R1, 1);
    E_STR(R6, R1, PARAM_DMA_SEED_BYTES_INDEX);

    E_LDR(R1, R6, PARAM_DMA_INTA_COUNT_INDEX);
    E_ADD_IMM(R1, R1, 1);
    E_STR(R6, R1, PARAM_DMA_INTA_COUNT_INDEX);

    E_LDR(R1, R6, PARAM_DMA_INTA_ADDRESS_INDEX);
    E_LDR(R2, R6, PARAM_DMA_CHANNEL_MASK_INDEX);
    E_STR(R1, R2, 0);

    E_LDR(R0, R6, PARAM_NEXT_TX_BYTE_INDEX);
    E_LDR(R1, R6, PARAM_REMAINING_COUNT_INDEX);
    E_LDR(R2, R6, PARAM_I3C_BASE_ADDRESS_INDEX);

    E_SUB_IMMS(R5, R1, 0);
    E_LDR(R5, R7, 1);
    E_COND_GOTO_REGL(ZE, R5);

E_LABEL("fill_tail_bytes");
    E_SUB_IMMS(R5, R1, 1);
    E_LDR(R5, R7, 2);
    E_COND_GOTO_REGL(ZE, R5);

    E_LOAD_SIMM(R5, I3C_MWDATAB_OFFSET, 0);
    E_ADD(R3, R2, R5);
    E_LDRB(R4, R0, 0);
    E_STR(R3, R4, 0);

    E_ADD_IMM(R0, R0, 1);
    E_SUB_IMMS(R1, R1, 1);

    E_LDR(R5, R6, PARAM_SMARTDMA_BYTES_INDEX);
    E_ADD_IMM(R5, R5, 1);
    E_STR(R6, R5, PARAM_SMARTDMA_BYTES_INDEX);

    E_LDR(R5, R7, 1);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("signal_completion");
    E_SUB_IMMS(R5, R1, 0);
    E_LDR(R5, R7, 3);
    E_COND_GOTO_REGL(ZE, R5);

    E_LOAD_SIMM(R5, I3C_MWDATABE_OFFSET, 0);
    E_ADD(R3, R2, R5);
    E_LDRB(R4, R0, 0);
    E_STR(R3, R4, 0);

    E_ADD_IMM(R0, R0, 1);
    E_LOAD_IMM(R1, 0x0);
    E_STR(R6, R0, PARAM_NEXT_TX_BYTE_INDEX);
    E_STR(R6, R1, PARAM_REMAINING_COUNT_INDEX);

    E_LDR(R5, R6, PARAM_SMARTDMA_BYTES_INDEX);
    E_ADD_IMM(R5, R5, 1);
    E_STR(R6, R5, PARAM_SMARTDMA_BYTES_INDEX);

E_LABEL("complete_mailbox");
    E_LDR(R5, R6, PARAM_MAILBOX_INDEX);
    E_ADD_IMM(R5, R5, 1);
    E_STR(R6, R5, PARAM_MAILBOX_INDEX);

    E_LDR(R5, R7, 3);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("end");
    E_NOP;
    E_GOSUB(end);
}

void keep_smartdma_api_alive(void)
{
    volatile void *ptr = &g_SMARTDMA_api;

    g_SMARTDMA_api[0] = (void (*)(void))(((uint32_t)EZHB_I3cDmaSeedChainProbe + 4U) & (~3U));

    (void)ptr;
}