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


//For Google
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

    //R0 is the src address, and R1 is the src size
    E_LDRB(R4,R0,0); //The first byte of src. ezh_data_buffer[0]
    E_STR(R3, R4, 0);
    E_ADD_IMM(R0, R0, 1); // R0 = R0 + 1, increment src address
    E_SUB_IMMS(R1, R1, 1); // R1 = R1 - 1, decrement size counter
    E_COND_GOTO_REGL(NZ,R7); // if R1 != 0, jump back to "send loop"
    
    //I3C0->MWDATABE = 0x20;
    E_LOAD_SIMM(R5, I3C_MWDATABE_OFFSET, 0);
    E_ADD(R3, R2, R5); //R3 = I3C_MWDATABE
    //E_LOAD_IMM(R6, 0x21U); //R6 = 0x21 ->The data will be sent
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

    //E_LOAD_IMM(CFS, 0x0); //CFS[10:8]=000, Bit-slice 0 - BS_INPUT0
    //E_LOAD_IMM(CFM, 0x101); // rising-edge trigger detection
    //E_HOLD; /*Hold for the event */

    E_INT_TRIGGER(0);


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
    E_LDR(R0, R6, 0); /*dst address */
    E_LDR(R1, R6, 1); /*dst size */
    E_LDR(R2, R6, 2); /*I3C base address - 0x40036000*/


    //E_ADD_IMM(R0, PC, 0); /* Load current PC to jump base address register */
    //E_ADD_IMM(PC, PC, 4);                   /* Skip legacy E_DCD table */
    //E_DCD(loop); E_DCD(end);

    //Just for test to write a fixed value in fix address
    E_LOAD_SIMM(R3, 0x20, 24); //R3 = 0x20000000
    E_LOAD_SIMM(R4, 0x28, 16); //R4 = 0x280000
    E_ADD(R3,R3, R4);          //R3 = 0x20280000
    E_LOAD_SIMM(R4, 0x18, 8); //R4 = 0x1800
    E_ADD(R3,R3, R4);          //R3 = 0x20281800
    E_LOAD_SIMM(R4, 0xD8, 0); //R4 = 0xD8
    E_ADD(R3,R3, R4);         //R3 = 0x202818D8， R3 is a fixed address for test
    //E_PUSH(R3);
    //E_LOAD_IMM(R4, 8);
    //E_STR(R3, R4,0);
    //E_STR(R3, R2, 0);

    //[MCTTL] = 0x6101 -> Start
    E_LOAD_SIMM(R5, I3C_MCTRL_OFFSET, 0);
    E_LOAD_SIMM(R6, 0x61, 8);
    E_LOAD_SIMM(R7, 0x01, 0);
    E_OR(R6, R6, R7);
    E_ADD(R3, R2, R5); //R3 = 0x40036084
    E_STR(R3, R6,0);//R3 is the register address, and R6 is the value to write

    //I3C_MSTATUS_OFFSET -> while ((I3C0->MSTATUS & I3C_MSTATUS_MCTRLDONE_MASK) == 0)
    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R3, R2, R5); //R3 = 0x40036088
    E_LOAD_IMM(R6, 0x200U); //R6 = I3C_MSTATUS_MCTRLDONE_MASK, R6 is for the mask value to check
    E_ADD_IMM(R7, PC, -4); // R7 is the loop address to jump back
E_LABEL("loop");
    E_LDR(R4, R3, 0); // R4 = [MSTATUS]
    E_ANDS(R4, R4, R6);
    E_COND_GOTO_REGL(ZE,R7);

    E_LOAD_IMM(R5, I3C_MRDATAB_OFFSET);
    E_ADD(R3, R2, R5); //R3 = I3C_MRDATAB, the address the data to read  
    E_LOAD_IMM(R5, I3C_MDATACTRL_OFFSET);  
    E_ADD(R6, R2, R5); //R6 = I3C_MDATACTRL -> is to check the rx data count
    E_ADD_IMM(R7, PC, -4);
E_LABEL("receive_loop");
    /*Check if the RX FIFO has data*/
    /**R5 - if rxCount is zero -> (base->MDATACTRL & I3C_MDATACTRL_RXCOUNT_MASK)*/
    E_LDR(R5, R6, 0); //R5 = [MDATACTRL]
    E_LOAD_SIMM(R4, 0x1F, 24); //R4 = 0x1F000000U, I3C_MDATACTRL_RXCOUNT_MASK
    E_AND_LSRS(R5, R5, R4, 24); // I3C_MDATACTRL_RXCOUNT_SHIFT - 24. R5 is the rxCount
    E_COND_GOTO_REGL(ZE,R7); //Zero - if R5 = 0, that is no data in RX FIFO, then jump back to "send loop"


    //R0 is the dst address, and R1 is the dst size
    E_LDR(R4,R3,0); //The data of RX FIFO. Read from I3C_MRDATAB
    E_STRB(R0, R4, 0);
    E_ADD_IMM(R0, R0, 1); // R0 = R0 + 1, increment src address
    E_SUB_IMMS(R1, R1, 1); // R1 = R1 - 1, decrement size counter

    E_COND_GOTO_REGL(NZ,R7); // if R1 != 0, jump back to "send loop"
    

    //I3C_MSTATUS_OFFSET -> while ((I3C0->MSTATUS & I3C_MSTATUS_COMPLETE_MASK) == 0)
    E_LOAD_SIMM(R5, I3C_MSTATUS_OFFSET, 0);
    E_ADD(R3, R2, R5); //R3 = 0x40036088
    E_LOAD_SIMM(R6, 0x40, 4); //R6 = I3C_MSTATUS_COMPLETE_MASK 0x400
    E_ADD_IMM(R7, PC, -4);
E_LABEL("loop1");
    E_LDR(R4, R3, 0); // R4 = [MSTATUS]
    E_ANDS(R4, R4, R6);
    E_COND_GOTO_REGL(ZE,R7);

    //[MCTTL] =  0x82 -> Stop
    E_LOAD_SIMM(R5, I3C_MCTRL_OFFSET, 0);
    E_LOAD_SIMM(R6, 0x82,0);
    E_ADD(R3, R2, R5); //R3 = 0x40036084
    E_STR(R3, R6,0);  


    E_INT_TRIGGER(0);


E_LABEL("end0");
    E_NOP;
    E_GOSUB(end0);
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
