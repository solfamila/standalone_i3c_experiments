#include "fsl_SMARTDMA_armclang.h"

#define SMARTDMA_CODE                   __attribute__((used, section(".smartdma_code"), aligned(4)))
#define SMARTDMA_DATA                   __attribute__((used, section(".smartdma_data"), aligned(4)))


#define EZH_BREAKADDR              0x27030
#define EZH_BREAKVECT              0x27034
#define EZH_EMERVECT               0x27038
#define EZH_EMERSEL                0x2703C
#define EZH_ARM2EZH                0x27040
#define EZH_EZH2ARM                0x27044


#define I3C_MCONFIG_OFFSET         0x0
#define I3C_MCTRL_OFFSET           0x84
#define I3C_MSTATUS_OFFSET         0x88
#define I3C_MDATACTRL_OFFSET       0xAC
#define I3C_MWDATAB_OFFSET         0xB0
#define I3C_MWDATABE_OFFSET        0xB4
#define I3C_MRDATAB_OFFSET         0xC0


void SMARTDMA_CODE EZHB_I3CWriting(void);
void SMARTDMA_CODE EZHB_I3CReading(void);

SMARTDMA_DATA void (*g_SMARTDMA_api[16])(void);


//For Google、
#if 0 
void SMARTDMA_CODE EZHB_I3CWriting(void)
{
    E_NOP;
    E_NOP;
    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);
    E_LDR(R0, R6, 0); /*src address */
    E_LDR(R1, R6, 1); /*src size */
    E_LDR(R2, R6, 2); /*I3C base address - 0x40036000*/
 
    //[MCTTL] = 0x6001 -> Start
    E_LOAD_SIMM(R5, I3C_MCTRL_OFFSET, 0);
    E_LOAD_SIMM(R6, 0x60, 8);
    E_LOAD_SIMM(R7, 0x01, 0);
    E_OR(R6, R6, R7);
    E_ADD(R3, R2, R5); //R3 = 0x40036084
    E_STR(R3, R6,0);//R3 is the register address, and R6 is the value to write
 
    //I3C_MSTATUS_OFFSET -> while ((I3C0->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) == 0)
    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R3, R2, R5); //R3 = 0x40036088
    E_LOAD_IMM(R6, 0x200U); //R6 = I3C_MSTATUS_MCTRLDONE_MASK, R6 is for the mask value to check
    E_ADD_IMM(R7, PC, -4); // R7 is the loop address to jump back
E_LABEL("start_loop");
    E_LDR(R4, R3, 0); // R4 = [MSTATUS]
    E_ANDS(R4, R4, R6);
    E_COND_GOTO_REGL(ZE,R7);
 
#if 0
    E_LOAD_IMM(R5, I3C_MWDATAB_OFFSET);
    E_ADD(R3, R2, R5); //R3 = I3C_MWDATAB, the address the data to write  
    E_LOAD_IMM(R5, I3C_MDATACTRL_OFFSET);  
    E_ADD(R6, R2, R5); //R6 = I3C_MDATACTRL
    E_ADD_IMM(R7, PC, -4);
E_LABEL("send_loop");
    /*Send data if the TX FIFO has Room*/
    /**R5 - txCount = (base->MDATACTRL & I3C_MDATACTRL_TXCOUNT_MASK) >> I3C_MDATACTRL_TXCOUNT_SHIFT;*/
    E_LDR(R5, R6, 0); //R5 = [MDATACTRL]
    E_LOAD_SIMM(R4, 0x1F, 16); //R4 = 0x1F0000, I3C_MDATACTRL_TXCOUNT_MASK
    E_AND_LSRS(R5, R5, R4, 16); // I3C_MDATACTRL_TXCOUNT_SHIFT - 16
    E_LOAD_IMM(R4, 8); //txFifoSize - Need to check further
    E_SUB(R5,R4, R5); /*txCount = txFifoSize - txCount;*/
    E_SUB_IMMS(R5, R5, 1); // R5 = R5 - 1, to check if R5 > 1
    E_COND_GOTO_REGL(ZB,R7); //Negative - if R5 < 0, that is txCount < 1, then jump back to "send loop"
#else
    E_LOAD_IMM(R5, I3C_MWDATAB_OFFSET);
    E_ADD(R3, R2, R5); //R3 = I3C_MWDATAB, the address the data to write
    E_LOAD_IMM(R5, I3C_MDATACTRL_OFFSET);  
    E_ADD(R6, R2, R5); //R6 = I3C_MDATACTRL
    E_LOAD_IMM(CFS, 0x0); //CFS[10:8]=000, Bit-slice 0 - BS_INPUT0
    E_LOAD_IMM(CFM, 0x101); // edge trigger detection
    E_ADD_IMM(R7, PC, -8);
 
E_LABEL("send_loop");
    E_HOLD; /*Hold for the event */
    //I3C_MSTATUS_OFFSET -> to check if the event is I3C_MSTATUS_TXNOTFULL_MASK
    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R4, R2, R5); //R4 = 0x40036088
    E_LDR(R5, R4, 0); // R5 = [MSTATUS]
    E_LOAD_SIMM(R6, 0x10, 8); //R6 = 0x1000, I3C_MSTATUS_TXNOTFULL_MASK, R6 is for the mask value to check
    E_ANDS(R5, R5, R6); // Check if TXNOTFULL bit is set
    E_COND_GOTO_REGL(ZE,R7); //Negative - if R5 == 0, that is I3C_MSTATUS_TXNOTFULL_MASK is not set, then jump back to "send loop"
    E_STR(R4, R6, 0); //clear I3C_MSTATUS_TXNOTFULL_MASK
#endif
 
    //R0 is the src address, and R1 is the src size
    E_LDRB(R4,R0,0); //The first byte of src. ezh_data_buffer[0]
    E_STR(R3, R4, 0);
    E_ADD_IMM(R0, R0, 1); // R0 = R0 + 1, increment src address
    E_SUB_IMMS(R1, R1, 1); // R1 = R1 - 1, decrement size counter
    E_SUB_IMMS(R4, R1, 1); // Update condition flags based on R1
    E_COND_GOTO_REGL(NZ,R7); // if R4 != 0, jump back to "send loop"
   
    E_LOAD_SIMM(R5, I3C_MWDATABE_OFFSET, 0);
    E_ADD(R3, R2, R5); //R3 = I3C_MWDATABE
    E_LDRB(R6, R0, 0); //R6 = [R0], get the last byte of src data
    E_STR(R3, R6, 0); //[I3C_MWDATABE]  0x21
 
    //I3C_MSTATUS_OFFSET -> while ((I3C0->MSTATUS & I3C_MSTATUS_COMPLETE_MASK) == 0)
    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R3, R2, R5); //R3 = 0x40036088
    E_LOAD_SIMM(R6, 0x40, 4); //R6 = I3C_MSTATUS_COMPLETE_MASK 0x400
    E_ADD_IMM(R7, PC, -4);
E_LABEL("complete_loop");
    E_LDR(R4, R3, 0); // R4 = [MSTATUS]
    E_ANDS(R4, R4, R6);
    E_COND_GOTO_REGL(ZE,R7);
 
    //[MCTTL] =  0x82 -> Stop
    E_LOAD_SIMM(R5, I3C_MCTRL_OFFSET, 0);
    E_LOAD_SIMM(R6, 0x82,0);
    E_ADD(R3, R2, R5); //R3 = 0x40036084
    E_STR(R3, R6,0);  
 
    E_INT_TRIGGER(0);
 
 
E_LABEL("end");
    E_NOP;
    E_GOSUB(end);
}
#else

#define EZH_TRACE_IMM(val32)     \
    do {                         \
        E_LOAD_IMM(GPO, (val32)); \
        E_PER_WRITE(GPO, EZH_EZH2ARM); \
    } while (0)



void SMARTDMA_CODE EZHB_I3CWriting(void)
{
    E_NOP;
    E_NOP;
    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);
    E_LDR(R0, R6, 0); /*src address */
    E_LDR(R1, R6, 1); /*src size */
    E_LDR(R2, R6, 2); /*I3C base address - 0x40036000*/

    E_LOAD_IMM(CFS, 0x0); //CFS[10:8]=000, Bit-slice 0 - BS_INPUT0
    E_LOAD_IMM(CFM, 0x101);
    E_LOAD_IMM(GPI, 1);     // first_round = 1

    E_ADD_IMM(R7, PC, 0);      /* R7 = address of DCD[0] (jump table base) */
    E_ADD_IMM(PC, PC, 5 * 4);     /* Skip 6 DCD entries */

    /* Jump table (6 entries, 1 word each) */
    E_DCD(send_loop);           /* [0] */
    E_DCD(burst_loop);          /* [1] */
    E_DCD(write_data);          /* [2] */
    E_DCD(done_irq);            /* [3] */
    E_DCD(end);                 /* [4] */
    E_DCD(after_hold);          /* [5] */

    /* If initial len == 0: trigger IRQ then end */
    E_LDR(R4, R7, 3);                    // R4 = DCD[3] = done_irq
    E_SUB_IMMS(R5, R1, 0);
    E_COND_GOTO_REGL(ZE, R4);
 
    E_LOAD_IMM(R5, I3C_MWDATAB_OFFSET);
    E_ADD(R3, R2, R5); //R3 = I3C_MWDATAB, the address the data to write
    E_LOAD_IMM(R5, I3C_MDATACTRL_OFFSET);  
    E_ADD(R6, R2, R5); //R6 = I3C_MDATACTRL

E_LABEL("send_loop");
    E_LDR(R4, R7, 5);   // R4 = after_hold address (jump target), keep it intact
    E_SUB_IMMS(R5, GPI, 0);          // flags from first_round (dest=R5, avoid clobbering R4)
    E_COND_GOTO_REGL(NZ, R4);        // if first_round!=0 -> skip HOLD/CLEAR

    E_HOLD;
    E_BCLR_IMM(CFM, CFM, 0);


E_LABEL("after_hold");
    E_LOAD_IMM(GPI, 0);              // first_round = 0 (only affects first entry)

    /* Check TXNOTFULL in MSTATUS */
    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R4, R2, R5);                   // R4 = MSTATUS address
    E_LDR(R5, R4, 0);                    // R5 = [MSTATUS]
    E_LOAD_SIMM(R4, 0x10, 8);            // R4 = 0x1000 (TXNOTFULL mask)  <-- mask uses R4, NOT R6
    E_ANDS(R5, R5, R4);                  // R5 = MSTATUS & TXNOTFULL

    /* If not TXNOTFULL, jump back to send_loop via DCD[1] */
    E_LDR(R4, R7, 0);                    // R4 = DCD[1] = send_loop
    E_COND_GOTO_REGL(ZE, R4);

    /* Compute fifo_space = txFifoSize - TXCOUNT */
    E_LDR(R5, R6, 0);                    // R5 = [MDATACTRL]
    E_LOAD_SIMM(R4, 0x1F, 16);           // R4 = 0x1F0000 (TXCOUNT mask)
    E_AND_LSRS(R5, R5, R4, 16);          // R5 = TXCOUNT

    E_LOAD_IMM(R4, 8);                   // R4 = txFifoSize (assumed 8 for now)
    E_SUB(R4, R4, R5);                   // R4 = fifo_space

    /* If fifo_space == 0, back to send_loop */
    E_LDR(R5, R7, 0);                    // R5 = send_loop
    E_SUB_IMMS(R4, R4, 0);               // flags from (fifo_space)
    E_COND_GOTO_REGL(ZE, R5);

    /* Loopback for burst_loop (keep as-is for now) */
    E_LDR(GPD, R7, 1);
    E_COND_GOTO_REGL(EU, GPD);  /* EU: unconditional  */

E_LABEL("burst_loop");
    E_SUB_IMMS(R5, R1, 0);               // flags from len
    E_LDR(R5, R7, 3);                    // R5 = DCD[3] = done_irq
    E_COND_GOTO_REGL(ZE, R5);

    /* If fifo_space == 0 -> send_loop */
    E_SUB_IMMS(R5, R4, 0);      /* flags from fifo_space */
    E_LDR(R5, R7, 0);           /* R5 = send_loop */
    E_COND_GOTO_REGL(ZE, R5);
    /* else -> write_data */
    E_LDR(R5, R7, 2);           /* R5 = write_data */
    E_COND_GOTO_REGL(EU, R5);


E_LABEL("write_data");
    /* Write one byte to TX FIFO */
    E_LDRB(R5, R0, 0);                   // R5 = *src
    E_STR(R3, R5, 0);                    // [MWDATAB] = byte
    E_ADD_IMM(R0, R0, 1);                // src++
    /* If len becomes 0 -> done_irq (IRQ then end) */
    E_LDR(R5, R7, 3);           /* R5 = done_irq */
    E_SUB_IMMS(R1, R1, 1);               // len--
    E_COND_GOTO_REGL(ZE, R5);
    E_SUB_IMMS(R4, R4, 1);               // fifo_space--
    E_LDR(R5, R7, 0);                    // R5 = send_loop
    /* If fifo_space == 0, go back to send_loop */
    E_SUB_IMMS(R4, R4, 0);               // flags from fifo_space

    E_COND_GOTO_REGL(ZE, R5);
    /* Continue burst loop */
    E_COND_GOTO_REGL(EU, GPD);

E_LABEL("done_irq");
    E_INT_TRIGGER(0);           /* does not set flags  */
    E_LDR(R5, R7, 4);           /* R5 = end */
    E_COND_GOTO_REGL(EU, R5);
  
 
E_LABEL("end");
    E_NOP;
    E_GOSUB(end);
}
#endif



void SMARTDMA_CODE EZHB_I3CReading(void)
{
    E_NOP;
    E_NOP;
    E_PER_READ(R6, EZH_ARM2EZH);
    E_LSR(R6, R6, 2);
    E_LSL(R6, R6, 2);
    E_LDR(R0, R6, 0); /*dst address */
    E_LDR(R1, R6, 1); /*dst size */
    E_LDR(R2, R6, 2); /*I3C base address - 0x40036000*/

    E_LOAD_IMM(CFS, 0x0); //CFS[10:8]=000, Bit-slice 0 - BS_INPUT0
    E_LOAD_IMM(CFM, 0x101);
    E_LOAD_IMM(GPI, 1);     // first_round = 1
    E_ADD_IMM(R7, PC, 0);      /* R7 = address of DCD[0] (jump table base) */
    E_ADD_IMM(PC, PC, 5 * 4); 
    /* Skip 6 DCD entries */
    /* Jump table (6 entries, 1 word each) */
    E_DCD(receive_loop);           /* [0] */
    E_DCD(burst_loop_rx);          /* [1] */
    E_DCD(read_data);              /* [2] */
    E_DCD(done_irq_rx);            /* [3] */
    E_DCD(end_rx);                 /* [4] */
    E_DCD(after_hold_rx);          /* [5] */


    E_LOAD_IMM(R5, I3C_MRDATAB_OFFSET);
    E_ADD(R3, R2, R5); //R3 = I3C_MRDATAB, the address the data to read  
    E_LOAD_IMM(R5, I3C_MDATACTRL_OFFSET);  
    E_ADD(R6, R2, R5); //R6 = I3C_MDATACTRL -> is to check the rx data count

E_LABEL("receive_loop");
    E_LDR(R4, R7, 5);   // R4 = after_hold address (jump target), keep it intact
    E_SUB_IMMS(R5, GPI, 0);          // flags from first_round (dest=R5, avoid clobbering R4)
    E_COND_GOTO_REGL(NZ, R4);        // if first_round!=0 -> skip HOLD/CLEAR
    E_HOLD;
    E_BCLR_IMM(CFM, CFM, 0);

E_LABEL("after_hold_rx");
    E_LOAD_IMM(GPI, 0);              // first_round = 0 (only affects first entry)
    /* Check RXPEND in MSTATUS */
    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R4, R2, R5);                   // R4 = MSTATUS address
    E_LDR(R5, R4, 0);                    // R5 = [MSTATUS]
    E_LOAD_SIMM(R4, 0x8, 8);           // R4 = 0x800U (RXPEND mask)
    E_ANDS(R5, R5, R4);                  // R5 = MSTATUS & RXPEND
    /* If not RXPEND, jump back to receive_loop via DCD[0] */
    E_LDR(R4, R7, 0);                    // R4 = DCD[0] = receive_loop
    // R4 = DCD[0] = receive_loop
    E_COND_GOTO_REGL(ZE, R4);

    /*Check if the RX FIFO has data*/
    /**R5 - if rxCount is zero -> (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)*/
    E_LDR(R5, R6, 0); //R5 = [MDATACTRL]
    E_LOAD_SIMM(R4, 0x1F, 24); //R4 = 0x1F000000U, I3C_MDATACTRL_RXCOUNT_MASK
    E_AND_LSRS(R4, R5, R4, 24); // I3C_MDATACTRL_RXCOUNT_SHIFT - 24. R5 is the rxCount
    /* If rxCount == 0, back to receive_loop */
    E_LDR(R5, R7, 0); 
    E_SUB_IMMS(R4, R4, 0);               // flags from rxCount
    E_COND_GOTO_REGL(ZE, R5);           /* Loopback for burst_loop (keep as-is for now) */

    E_LDR(GPD, R7, 1);
    E_COND_GOTO_REGL(EU, GPD);  /* EU: unconditional  */ 

E_LABEL("burst_loop_rx");
    E_SUB_IMMS(R5, R1, 0);               // flags from len
    E_LDR(R5, R7, 3);                    // R5 = DCD[3] = done_irq
    E_COND_GOTO_REGL(ZE, R5);
    /* If rxCount == 0 -> receive_loop */
    E_SUB_IMMS(R5, R4, 0);      /* flags from rxCount */
    E_LDR(R5, R7, 0);           /* R5 = receive_loop */
    E_COND_GOTO_REGL(ZE, R5);
    /* else -> read_data */
    E_LDR(R5, R7, 2);           /* R5 = read_data */
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("read_data");
    /* Read one byte from RX FIFO */
    E_LDR(R5, R3, 0);                    // R5 = [MRDATAB]
    E_STRB(R0, R5, 0);                   // *dst = byte
    E_ADD_IMM(R0, R0, 1);                // dst++
    /* If len becomes 0 -> done_irq (IRQ then end) */
    E_LDR(R5, R7, 3);           /* R5 = done_irq */
    E_SUB_IMMS(R1, R1, 1);               // len--
    E_COND_GOTO_REGL(ZE, R5);
    E_SUB_IMMS(R4, R4, 1);               // rxCount--
    E_LDR(R5, R7, 0);                    // R5 = receive_loop
    /* If rxCount == 0, go back to receive_loop */
    E_SUB_IMMS(R4, R4, 0);               // flags from rxCount
    E_COND_GOTO_REGL(ZE, R5);
    /* Continue burst loop */
    E_COND_GOTO_REGL(EU, GPD);

E_LABEL("done_irq_rx");
    E_INT_TRIGGER(0);           /* does not set flags  */
    E_LDR(R5, R7, 4);           /* R5 = end */
    E_COND_GOTO_REGL(EU, R5);

E_LABEL("end_rx");
    E_NOP;
    E_GOSUB(end_rx);
}



/**
 * @brief Prevent linker optimization of SmartDMA API table
 * @note Forces linker to keep g_SMARTDMA_api symbol in binary
 */
void keep_smartdma_api_alive(void)
{
    volatile void *ptr = &g_SMARTDMA_api; 
    g_SMARTDMA_api[0] = (void (*)(void))(((uint32_t)EZHB_I3CWriting + 4) & (~3U)); 
    g_SMARTDMA_api[1] = (void (*)(void))(((uint32_t)EZHB_I3CReading + 4) & (~3U));
    (void)ptr;
}
