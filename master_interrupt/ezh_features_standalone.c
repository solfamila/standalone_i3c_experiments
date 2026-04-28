#include "fsl_SMARTDMA_armclang.h"

#define SMARTDMA_CODE                   __attribute__((used, section(".smartdma_code"), aligned(4)))
#define SMARTDMA_DATA                   __attribute__((used, section(".smartdma_data"), aligned(4)))

#include "fsl_SMARTDMA_armclang.h"

#define SMARTDMA_CODE __attribute__((used, section(".smartdma_code"), aligned(4)))
#define SMARTDMA_DATA __attribute__((used, section(".smartdma_data"), aligned(4)))

#define EZH_BREAKADDR 0x27030
#define EZH_BREAKVECT 0x27034
#define EZH_EMERVECT 0x27038
#define EZH_EMERSEL 0x2703C
#define EZH_ARM2EZH 0x27040
#define EZH_EZH2ARM 0x27044

#define I3C_MCONFIG_OFFSET 0x0
#define I3C_MCTRL_OFFSET 0x84
#define I3C_MSTATUS_OFFSET 0x88
#define I3C_MINTSET_OFFSET 0x90
#define I3C_MINTCLR_OFFSET 0x94
#define I3C_MINTMASKED_OFFSET 0x98
#define I3C_MDATACTRL_OFFSET 0xAC
#define I3C_MWDATAB_OFFSET 0xB0
#define I3C_MWDATABE_OFFSET 0xB4
#define I3C_MRDATAB_OFFSET 0xC0

#define EZH_SMARTDMA_MAILBOX_COMPLETION 0x1U
#define EZH_SMARTDMA_MAILBOX_PROTOCOL 0x2U

#define EZH_I3C_READY_INT_RX_MASK 0x800U
#define EZH_I3C_READY_INT_TX_MASK 0x1000U
#define EZH_I3C_RDTERM_ONE 0x10000U
#define EZH_I3C_STATUS_PROTOCOL_LOW_MASK 0xA100U
#define EZH_I3C_STATUS_PROTOCOL_HIGH_MASK 0x80000U

void SMARTDMA_CODE EZHB_I3CWriting(void);
void SMARTDMA_CODE EZHB_I3CReading(void);

SMARTDMA_DATA void (*g_SMARTDMA_api[16])(void);

void SMARTDMA_CODE EZHB_I3CWriting(void)
{
    E_NOP;
    E_NOP;
    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);
    E_LDR(R0, R6, 0); /* src address */
    E_LDR(R1, R6, 1); /* src size */
    E_LDR(R2, R6, 2); /* I3C base address */
    E_LDR(GPO, R6, 4); /* mailbox address */

    E_LOAD_IMM(CFS, 0x0);
    E_LOAD_IMM(CFM, 0x101);

    E_ADD_IMM(R7, PC, 0);
    E_ADD_IMM(PC, PC, 4 * 4);

    E_DCD(send_loop);
    E_DCD(burst_loop);
    E_DCD(write_data);
    E_DCD(done_irq);
    E_DCD(protocol_irq);
    E_DCD(end);

    E_LDR(R4, R7, 3);
    E_SUB_IMMS(R5, R1, 0);
    E_COND_GOTO_REGL(ZE, R4);

    E_LOAD_IMM(R5, I3C_MWDATAB_OFFSET);
    E_ADD(R3, R2, R5);
    E_LOAD_IMM(R5, I3C_MDATACTRL_OFFSET);
    E_ADD(R6, R2, R5);

E_LABEL("send_loop");
    E_LOAD_SIMM(R5, I3C_MINTSET_OFFSET, 0);
    E_ADD(R4, R2, R5);
    E_LOAD_SIMM(R5, (EZH_I3C_READY_INT_TX_MASK >> 8), 8);
    E_STR(R4, R5, 0);
    E_HOLD;
    E_LOAD_SIMM(R5, I3C_MINTCLR_OFFSET, 0);
    E_ADD(R4, R2, R5);
    E_LOAD_SIMM(R5, (EZH_I3C_READY_INT_TX_MASK >> 8), 8);
    E_STR(R4, R5, 0);
    E_BCLR_IMM(CFM, CFM, 0);
    E_LOAD_SIMM(R5, I3C_MINTMASKED_OFFSET, 0);
    E_ADD(R4, R2, R5);
    E_LDR(GPD, R4, 0);
    E_LOAD_SIMM(R4, (EZH_I3C_STATUS_PROTOCOL_LOW_MASK >> 8), 8);
    E_LOAD_SIMM(R5, (EZH_I3C_STATUS_PROTOCOL_HIGH_MASK >> 16), 16);
    E_ORS(R4, R4, R5);
    E_ANDS(GPD, GPD, R4);
    E_LDR(R4, R7, 4);
    E_COND_GOTO_REGL(NZ, R4);

    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R4, R2, R5);
    E_LDR(R5, R4, 0);
    E_LOAD_SIMM(R4, 0x10, 8);
    E_ANDS(R5, R5, R4);
    E_LDR(R4, R7, 0);
    E_COND_GOTO_REGL(ZE, R4);

    E_LDR(R5, R6, 0);
    E_LOAD_SIMM(R4, 0x1F, 16);
    E_AND_LSRS(R5, R5, R4, 16);

    E_LOAD_IMM(R4, 8);
    E_SUB(R4, R4, R5);

    E_SUB_IMMS(R5, R4, 0);
    E_LDR(R5, R7, 0);
    E_COND_GOTO_REGL(ZE, R5);

    E_LDR(GPD, R7, 1);
    E_COND_GOTO_REGL(EU, GPD);

E_LABEL("burst_loop");
    E_SUB_IMMS(R5, R1, 0);
    E_LDR(R5, R7, 3);
    E_COND_GOTO_REGL(ZE, R5);

    E_SUB_IMMS(R5, R4, 0);
    E_LDR(R5, R7, 0);
    E_COND_GOTO_REGL(ZE, R5);

    E_LDR(R5, R7, 2);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("write_data");
    E_LDRB(R5, R0, 0);
    E_STR(R3, R5, 0);
    E_ADD_IMM(R0, R0, 1);
    E_SUB_IMMS(R1, R1, 1);
    E_LDR(R5, R7, 3);
    E_COND_GOTO_REGL(ZE, R5);
    E_SUB_IMMS(R4, R4, 1);
    E_SUB_IMMS(R5, R4, 0);
    E_LDR(R5, R7, 0);
    E_COND_GOTO_REGL(ZE, R5);
    E_COND_GOTO_REGL(EU, GPD);

E_LABEL("done_irq");
    E_LOAD_IMM(R4, EZH_SMARTDMA_MAILBOX_COMPLETION);
    E_STR(GPO, R4, 0);
    E_INT_TRIGGER(0);
    E_LDR(R5, R7, 5);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("protocol_irq");
    E_LOAD_IMM(R4, EZH_SMARTDMA_MAILBOX_PROTOCOL);
    E_STR(GPO, R4, 0);
    E_INT_TRIGGER(0);
    E_LDR(R5, R7, 5);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("end");
    E_NOP;
    E_GOSUB(end);
}

void SMARTDMA_CODE EZHB_I3CReading(void)
{
    E_NOP;
    E_NOP;
    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);
    E_LDR(R0, R6, 0); /* dst address */
    E_LDR(R1, R6, 1); /* dst size */
    E_LDR(R2, R6, 2); /* I3C base address */
    E_LDR(GPO, R6, 4); /* mailbox address */

    E_LOAD_IMM(CFS, 0x0);
    E_LOAD_IMM(CFM, 0x101);

    E_LOAD_IMM(R5, I3C_MRDATAB_OFFSET);
    E_ADD(R3, R2, R5);
    E_LOAD_IMM(R5, I3C_MDATACTRL_OFFSET);
    E_ADD(R6, R2, R5);
    E_ADD_IMM(R7, PC, 0);
    E_ADD_IMM(PC, PC, 4 * 4);

    E_DCD(read_wait_for_irq);
    E_DCD(read_process_ready);
    E_DCD(read_done_irq);
    E_DCD(read_byte_loop);
    E_DCD(read_protocol_irq);
    E_DCD(read_arm_rdterm);
    E_DCD(read_end0);

    E_SUB_IMMS(R4, R1, 0);
    E_LDR(R4, R7, 2);
    E_COND_GOTO_REGL(ZE, R4);

E_LABEL("read_wait_for_irq");
    E_LOAD_SIMM(R5, I3C_MINTSET_OFFSET, 0);
    E_ADD(R4, R2, R5);
    E_LOAD_SIMM(R5, (EZH_I3C_READY_INT_RX_MASK >> 8), 8);
    E_STR(R4, R5, 0);
    E_HOLD;
    E_LOAD_SIMM(R5, I3C_MINTCLR_OFFSET, 0);
    E_ADD(R4, R2, R5);
    E_LOAD_SIMM(R5, (EZH_I3C_READY_INT_RX_MASK >> 8), 8);
    E_STR(R4, R5, 0);
    E_BCLR_IMM(CFM, CFM, 0);
    E_LOAD_SIMM(R5, I3C_MINTMASKED_OFFSET, 0);
    E_ADD(R5, R2, R5);
    E_LDR(R4, R5, 0);

    E_LOAD_SIMM(R5, (EZH_I3C_STATUS_PROTOCOL_LOW_MASK >> 8), 8);
    E_LOAD_SIMM(GPD, (EZH_I3C_STATUS_PROTOCOL_HIGH_MASK >> 16), 16);
    E_ORS(R5, R5, GPD);
    E_ANDS(R4, R4, R5);
    E_LDR(R5, R7, 4);
    E_COND_GOTO_REGL(NZ, R5);

    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R5, R2, R5);
    E_LDR(GPD, R5, 0);

    E_LOAD_SIMM(R5, (EZH_I3C_READY_INT_RX_MASK >> 8), 8);
    E_ANDS(R5, GPD, R5);
    E_LDR(R4, R7, 1);
    E_COND_GOTO_REGL(NZ, R4);

    E_LDR(R4, R7, 0);
    E_COND_GOTO_REGL(EU, R4);

E_LABEL("read_process_ready");
    E_LDR(R5, R6, 0);
    E_LOAD_SIMM(R4, 0x1F, 24);
    E_AND_LSRS(R5, R5, R4, 24);
    E_SUB_IMMS(R4, R5, 0);
    E_LDR(R4, R7, 0);
    E_COND_GOTO_REGL(ZE, R4);

    E_LDR(R4, R7, 3);
    E_COND_GOTO_REGL(EU, R4);

E_LABEL("read_byte_loop");
    E_SUB_IMMS(R4, R1, 0);
    E_LDR(R4, R7, 2);
    E_COND_GOTO_REGL(ZE, R4);

    E_SUB_IMMS(R4, R5, 0);
    E_LDR(R4, R7, 0);
    E_COND_GOTO_REGL(ZE, R4);

    E_LDR(R4, R3, 0);
    E_STRB(R0, R4, 0);
    E_ADD_IMM(R0, R0, 1);
    E_SUB_IMMS(R1, R1, 1);
    E_SUB_IMMS(R5, R5, 1);
    E_SUB_IMMS(R4, R1, 0);
    E_LDR(R4, R7, 5);
    E_COND_GOTO_REGL(ZE, R4);
    E_LDR(R4, R7, 3);
    E_COND_GOTO_REGL(EU, R4);

E_LABEL("read_arm_rdterm");
    E_LOAD_SIMM(R4, I3C_MCTRL_OFFSET, 0);
    E_ADD(R4, R2, R4);
    E_LDR(GPD, R4, 0);
    E_LOAD_SIMM(R5, (EZH_I3C_RDTERM_ONE >> 16), 16);
    E_ORS(GPD, GPD, R5);
    E_STR(R4, GPD, 0);
    E_LDR(R4, R7, 3);
    E_COND_GOTO_REGL(EU, R4);

E_LABEL("read_done_irq");
    E_LOAD_IMM(R4, EZH_SMARTDMA_MAILBOX_COMPLETION);
    E_STR(GPO, R4, 0);
    E_INT_TRIGGER(0);
    E_LDR(R5, R7, 6);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("read_protocol_irq");
    E_LOAD_IMM(R4, EZH_SMARTDMA_MAILBOX_PROTOCOL);
    E_STR(GPO, R4, 0);
    E_INT_TRIGGER(0);
    E_LDR(R5, R7, 6);
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("read_end0");
    E_NOP;
    E_GOSUB(read_end0);
}

void keep_smartdma_api_alive(void)
{
    volatile void *ptr = &g_SMARTDMA_api;

    g_SMARTDMA_api[0] = (void (*)(void))(((uint32_t)EZHB_I3CWriting + 4U) & (~3U));
    g_SMARTDMA_api[1] = (void (*)(void))(((uint32_t)EZHB_I3CReading + 4U) & (~3U));
    (void)ptr;
}
