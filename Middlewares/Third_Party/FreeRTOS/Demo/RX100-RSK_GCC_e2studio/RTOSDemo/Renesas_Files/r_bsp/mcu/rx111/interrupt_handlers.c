/***********************************************************************/
/*  													               */
/*      PROJECT NAME :  RTOSDemo_GCC                                   */
/*      FILE         :  interrupt_handlers.c                           */
/*      DESCRIPTION  :  Interrupt Handler                              */
/*      CPU SERIES   :  RX100                                          */
/*      CPU TYPE     :  RX111                                          */
/*  													               */
/*      This file is generated by e2studio.                        */
/*  													               */
/***********************************************************************/




#include "interrupt_handlers.h"

// INT_Exception(Supervisor Instruction)
void INT_Excep_SuperVisorInst(void){/* brk(); */}

// Exception(Undefined Instruction)
void INT_Excep_UndefinedInst(void){/* brk(); */}

// Exception(Floating Point)
void INT_Excep_FloatingPoint(void){/* brk(); */}

// NMI
void INT_NonMaskableInterrupt(void){/* brk(); */}

// Dummy
void Dummy(void)
{
	for( ;; );
}

// BRK
void INT_Excep_BRK(void){/* wait(); */}
//;0x0000  Reserved

void  INT_Excep_BUSERR(void){ }

// ICU SWINT
void INT_Excep_ICU_SWINT(void){ }

// CMT0 CMI0
void INT_Excep_CMT0_CMI0(void){ }

// CMT1 CMI1
void INT_Excep_CMT1_CMI1(void){ }

// CAC FERRF
void INT_Excep_CAC_FERRF(void){ }

// CAC MENDF
void INT_Excep_CAC_MENDF(void){ }

// CAC OVFF
void INT_Excep_CAC_OVFF(void){ }

// USB0 D0FIFO0
void INT_Excep_USB0_D0FIFO0(void){ }

// USB0 D1FIFO0
void INT_Excep_USB0_D1FIFO0(void){ }

// USB0 USBI0
void INT_Excep_USB0_USBI0(void){ }

// RSPI0 SPEI0
void INT_Excep_RSPI0_SPEI0(void){ }

// RSPI0 SPRI0
void INT_Excep_RSPI0_SPRI0(void){ }

// RSPI0 SPTI0
void INT_Excep_RSPI0_SPTI0(void){ }

// RSPI0 SPII0
void INT_Excep_RSPI0_SPII0(void){ }

// DOC DOPCF
void INT_Excep_DOC_DOPCF(void){ }

// RTC CUP
void INT_Excep_RTC_CUP(void){ }

// ICU IRQ0
void INT_Excep_ICU_IRQ0(void){ }

// ICU IRQ1
void INT_Excep_ICU_IRQ1(void){ }

// ICU IRQ2
void INT_Excep_ICU_IRQ2(void){ }

// ICU IRQ3
void INT_Excep_ICU_IRQ3(void){ }

// ICU IRQ4
void INT_Excep_ICU_IRQ4(void){ }

// ICU IRQ5
void INT_Excep_ICU_IRQ5(void){ }

// ICU IRQ6
void INT_Excep_ICU_IRQ6(void){ }

// ICU IRQ7
void INT_Excep_ICU_IRQ7(void){ }

// LVD LVD1
void INT_Excep_LVD_LVD1(void){ }

// LVD LVD2
void INT_Excep_LVD_LVD2(void){ }

// USB0 USBR0
void INT_Excep_USB0_USBR0(void){ }

// RTC ALM
void INT_Excep_RTC_ALM(void){ }

// RTC PRD
void INT_Excep_RTC_PRD(void){ }

// S12AD S12ADI0
void INT_Excep_S12AD_S12ADI0(void){ }

// S12AD GBADI
void INT_Excep_S12AD_GBADI(void){ }

// ELC ELSR18I
void INT_Excep_ELC_ELSR18I(void){ }

// MTU0 TGIA0
void INT_Excep_MTU0_TGIA0(void){ }

// MTU0 TGIB0
void INT_Excep_MTU0_TGIB0(void){ }

// MTU0 TGIC0
void INT_Excep_MTU0_TGIC0(void){ }

// MTU0 TGID0
void INT_Excep_MTU0_TGID0(void){ }

// MTU0 TCIV0
void INT_Excep_MTU0_TCIV0(void){ }

// MTU0 TGIE0
void INT_Excep_MTU0_TGIE0(void){ }

// MTU0 TGIF0
void INT_Excep_MTU0_TGIF0(void){ }

// MTU1 TGIA1
void INT_Excep_MTU1_TGIA1(void){ }

// MTU1 TGIB1
void INT_Excep_MTU1_TGIB1(void){ }

// MTU1 TCIV1
void INT_Excep_MTU1_TCIV1(void){ }

// MTU1 TCIU1
void INT_Excep_MTU1_TCIU1(void){ }

// MTU2 TGIA2
void INT_Excep_MTU2_TGIA2(void){ }

// MTU2 TGIB2
void INT_Excep_MTU2_TGIB2(void){ }

// MTU2 TCIV2
void INT_Excep_MTU2_TCIV2(void){ }

// MTU2 TCIU2
void INT_Excep_MTU2_TCIU2(void){ }

// MTU3 TGIA3
void INT_Excep_MTU3_TGIA3(void){ }

// MTU3 TGIB3
void INT_Excep_MTU3_TGIB3(void){ }

// MTU3 TGIC3
void INT_Excep_MTU3_TGIC3(void){ }

// MTU3 TGID3
void INT_Excep_MTU3_TGID3(void){ }

// MTU3 TCIV3
void INT_Excep_MTU3_TCIV3(void){ }

// MTU4 TGIA4
void INT_Excep_MTU4_TGIA4(void){ }

// MTU4 TGIB4
void INT_Excep_MTU4_TGIB4(void){ }

// MTU4 TGIC4
void INT_Excep_MTU4_TGIC4(void){ }

// MTU4 TGID4
void INT_Excep_MTU4_TGID4(void){ }

// MTU4 TCIV4
void INT_Excep_MTU4_TCIV4(void){ }

// MTU5 TGIU5
void INT_Excep_MTU5_TGIU5(void){ }

// MTU5 TGIV5
void INT_Excep_MTU5_TGIV5(void){ }

// MTU5 TGIW5
void INT_Excep_MTU5_TGIW5(void){ }

// POE OEI1
void INT_Excep_POE_OEI1(void){ }

// POE OEI2
void INT_Excep_POE_OEI2(void){ }

// SCI1 ERI1
void INT_Excep_SCI1_ERI1(void){ }

// SCI1 RXI1
void INT_Excep_SCI1_RXI1(void){ }

// SCI1 TXI1
void INT_Excep_SCI1_TXI1(void){ }

// SCI1 TEI1
void INT_Excep_SCI1_TEI1(void){ }

// SCI5 ERI5
void INT_Excep_SCI5_ERI5(void){ }

// SCI5 RXI5
void INT_Excep_SCI5_RXI5(void){ }

// SCI5 TXI5
void INT_Excep_SCI5_TXI5(void){ }

// SCI5 TEI5
void INT_Excep_SCI5_TEI5(void){ }

// SCI12 ERI12
void INT_Excep_SCI12_ERI12(void){ }

// SCI12 RXI12
void INT_Excep_SCI12_RXI12(void){ }

// SCI12 TXI12
void INT_Excep_SCI12_TXI12(void){ }

// SCI12 TEI12
void INT_Excep_SCI12_TEI12(void){ }

// SCI12 SCIX0
void INT_Excep_SCI12_SCIX0(void){ }

// SCI12 SCIX1
void INT_Excep_SCI12_SCIX1(void){ }

// SCI12 SCIX2
void INT_Excep_SCI12_SCIX2(void){ }

// SCI12 SCIX3
void INT_Excep_SCI12_SCIX3(void){ }

// RIIC0 EEI0
void INT_Excep_RIIC0_EEI0(void){ }

// RIIC0 RXI0
void INT_Excep_RIIC0_RXI0(void){ }

// RIIC0 TXI0
void INT_Excep_RIIC0_TXI0(void){ }

// RIIC0 TEI0
void INT_Excep_RIIC0_TEI0(void){ }
