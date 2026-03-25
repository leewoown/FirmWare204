

/**
 * main.c
 */

#include "DSP28x_Project.h"
#include "parameter.h"
#include "SysVariable.h"
#include "ProtectRelay.h"
#include "BATAlgorithm.h"
#include "BATCalc.h"
#include "SysSpiCan.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
//#include "BATCellModel.h"
/*
 *
 */
void InitGpio(void);

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

/*
 *
 */
void InitECanaGpio(void);
void InitECana(void);
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);
void CANATX_29bit(Uint32 ID, Uint16 Length,Uint16 Data0, Uint16 Data1, Uint16 Data2, Uint16 Data3);

void InitSpiGpio();
//void InitSpiBATIC(void);
void InitSpiCAN(void);
void SPI_Write(unsigned int WRData);
unsigned int SPI_Read(void);
float MeasureSPISpeedHandle(Uint16 testLen);

void TestCANSPIWriteBytesHandle(void);
void CANSPIReadBytesHandle (Uint16 cmd, Uint16 addr, char RxBuf[], Uint16 len);
void CANSPIWriteBytesHandle(Uint16 cmd, Uint16 addr, char TxBuf[], Uint16 len);

void MCP2515ResetHandle(void);
void MCP2515SetCNFHandle(char cnf1, char cnf2, char cnf3);
void MCP2515SetNormalModeHandle(void);

void MCP2515InitHandle(CANBReg *p);

/*BAT MODE*/




//void SPI_Write(unsigned int WRData);
//unsigned int SPI_Read(void);
//void BAT_InitSPI(void);
//void SPI_BATWrite(unsigned int WRData);

/*
 *
 */
void SysTimerINIT(SystemReg *s);
void SysVarINIT(SystemReg *s);
void CANRegVarINIT(CANAReg *P);
void ModuleInit(ModulemReg *P);
void DigitalInput(SystemReg *sys);
void DigitalOutput(SystemReg *sys);
/*
 *
 */
//void ProtectRlySateCheck(PrtectRelayReg *P);
void ProtectRlyVarINIT(PrtectRelayReg *P);
void ProtectRelayHandle(PrtectRelayReg *P);
void ProtectRlyOnInit(PrtectRelayReg *P);
void ProtectRlyOffInit(PrtectRelayReg *P);



void CalKokam100AhRegsInit(SocReg *P);
void CalKokam100AhSocInit(SocReg *P);
void Calkokam100AhSocHandle(SocReg *P);

/*
 *
 */
void SysCurrentHandle(SystemReg *s);

/*
 *
 */
void SysCommErrHandle(SystemReg *P);
void CalSysAlarmtCheck(SystemReg *s);
void CalSysFaultCheck(SystemReg *s);
void CalSysProtectCheck(SystemReg *s);
/*
 *
 */
extern void BatCalcRegsInit(BatCalcReg *P);
extern void BatCalcVoltHandle(BatCalcReg *P);
extern void BatCalcTempsHandle(BatCalcReg *P);

/*
 *
 */
int float32ToInt(float32 Vaule, Uint32 Num);
/*
 *
 */

/*
 *
 */
float32 GetDischargeLimit30s_A(float32 tempC_f, float32 socPct_f);
float32 GetDischargeLimitCont_A(float32 tempC_f, float32 socPct_f);
float32 GetChargeLimit10s_A(float32 tempC_f, float32 socPct_f);
float32 GetChargeLimitCont_A(float32 tempC_f, float32 socPct_f);
void TestCurrentlimt_StepUpdate_200ms(SystemReg *p);
float RandVaule(float inputVaule);
/*
 *
 */
//int LTC6804_read_cmd(char address, short command, char data[], int len);
//int LTC6804_write_cmd(char address, short command, char data[], int len);
//void init_PEC15_Table(void);
//unsigned short pec15(char *data, int len);
//int SlaveBMSIint(SlaveReg *s);
//void SlaveVoltagHandler(SlaveReg *s);

/*
 *  인터럽트 함수 선언
 */
interrupt void cpu_timer0_isr(void);
interrupt void ISR_CANRXINTA(void);
//interrupt void cpu_timer2_isr(void);

SystemReg       SysRegs;
ModulemReg      ModRegs;
PrtectRelayReg  PrtectRelayRegs;
CANAReg         CANARegs;
CANBReg         CANBRegs;
SocReg          Kam100AHSocRegs;
BatCalcReg      BatCalcRegs;
float32         NCMsocTestVoltAGV =3.210;

extern unsigned int   CANTXFALAG=0;
extern unsigned int   CANTXFALAG1=0;
extern unsigned int   CANTXFALAG12=0;
//extern unsigned int    CellVoltUnBalaneFaulCount=0;
void main(void)
{
//    struct ECAN_REGS ECanaShadow;
    InitSysCtrl();
    /*
     * To check the clock status of the C2000 in operation
     */
  //  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
  //  SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 0; //XCLOCKOUT = 1/2* SYSCLK

// Step 2. Initalize GPIO:
// This example function is found in the DSP2803x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// For this example use the following configuration:
// Step 3. Clear all interrupts and initialize PIE vector table:
    DINT;
// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2803x_PieCtrl.c file.
    InitPieCtrl();
// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    EALLOW;  // This is needed to write to EALLOW protected registers

    /*
     *  인터럽트 함수 선언
     */
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.ECAN0INTA  = &ISR_CANRXINTA;
//    PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    InitGpio();
   // GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 3; //enable XCLOCKOUT through GPIO mux
  //  SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2; //XCLOCKOUT = SYSCLK
    InitSpiGpio();
    InitSpiCAN();
    // TEST
    InitECanGpio();
    InitECan();

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();

    ConfigCpuTimer(&CpuTimer0, 80, 1000);
    CpuTimer0Regs.PRD.all = 90000;// 90000 is 1msec
    //   ConfigCpuTimer(&CpuTimer1, 80, 1000000);
    //   ConfigCpuTimer(&CpuTimer2, 80, 1000000);
    CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
    //  CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
//    InitAdc();
//    AdcOffsetSelfCal();
    EALLOW;
    EDIS;    // This is needed to disable write to EALLOW protected registers
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT9;//test
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;      // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;      // Enable ECAN-A interrupt of PIE group 9
//  PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // SCIA RX interrupt of PIE group
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    SysRegs.SysMachine =System_STATE_INIT;
    PrtectRelayRegs.StateMachine=STATERly_INIT;
    SysRegs.PackStateReg.bit.CANCOMEnable=0;
    SysRegs.PackStateReg.bit.INITOK=0;
    while(1)
    {
        SysRegs.Maincount++;

        switch(SysRegs.SysMachine)
        {
            case System_STATE_INIT:
                  SysTimerINIT(&SysRegs);
                  SysVarINIT(&SysRegs);
                  CANRegVarINIT(&CANARegs);
                  ModuleInit(&ModRegs);
                  BatCalcRegsInit(&BatCalcRegs);
                  ProtectRlyVarINIT(&PrtectRelayRegs);
                  CalKokam100AhRegsInit(&Kam100AHSocRegs);
                 // BatteryModel_UserInit();
                  // Function SysMachine
                  Kam100AHSocRegs.state=SOC_STATE_IDLE;
                  PrtectRelayRegs.StateMachine=STATERly_INIT;
                  SysRegs.SysMachine=System_STATE_STANDBY;
                  CANARegs.PackSate= 1;
                  if(SysRegs.PackStateReg.bit.SysPrtct==1)
                  {
                    //SysRegs.SysMachine=System_STATE_PROTECTER;
                  }

                //  SysRegs.PackStateReg.bit.SysSeqState=1;
            break;
            case System_STATE_STANDBY:
                  CANARegs.PackSate= 2;
                  SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=0;
                  PrtectRelayRegs.StateMachine=STATERly_STANDBY;
                  SysRegs.PackStateReg.bit.INITOK=1;
                  if(SysRegs.PackStateReg.bit.SysPrtct==1)
                  {
                    //  SysRegs.PackStateReg.bit.INITOK=1;
                  }
                  delay_ms(1000);
                  // Function SysMachine
                  Kam100AHSocRegs.state= SOC_STATE_RUNNING;
                  SysRegs.SysMachine=System_STATE_READY;
                  PrtectRelayRegs.StateMachine=STATERly_Ready;
                  SysRegs.PackStateReg.bit.CANCOMEnable=1;
              //    SysRegs.PackStateReg.bit.SysSTATE =2;
            break;
            case System_STATE_READY:
                 CANARegs.PackSate= 3;
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=0;
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus01==1)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=1;
                 }
                 // Function SysMachine
                 if(PrtectRelayRegs.State.bit.WakeuPOnEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_RUNING;
                 }
                // SysRegs.PackStateReg.bit.SysSTATE = 3;
            break;
            case System_STATE_RUNING:
                 CANARegs.PackSate= 4;
                 Kam100AHSocRegs.state= SOC_STATE_RUNNING;
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 SysRegs.DigitalOutPutReg.bit.PWRLAMPOUT=1;
                 if(CANARegs.PMSCMDRegs.bit.RUNStatus01==0)
                 {
                     PrtectRelayRegs.State.bit.WakeUpEN=0;
                 }
                 if(PrtectRelayRegs.State.bit.WakeuPOffEND==1)
                 {
                     SysRegs.SysMachine=System_STATE_STANDBY;
                 }
             //    SysRegs.PackStateReg.bit.SysSTATE = 4;
            break;
            case System_STATE_PROTECTER:
                CANARegs.PackSate= 5;
                 SysRegs.PackStateReg.bit.CANCOMEnable=1;
                 CANARegs.PMSCMDRegs.all=0;
                 if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
                 {
                     CANARegs.PMSCMDRegs.bit.PrtctReset01=0;
                     SysTimerINIT(&SysRegs);
                     SysVarINIT(&SysRegs);
                     CANRegVarINIT(&CANARegs);
                     ModuleInit(&ModRegs);
                     BatCalcRegsInit(&BatCalcRegs);
                     ProtectRlyVarINIT(&PrtectRelayRegs);
                     CalKokam100AhRegsInit(&Kam100AHSocRegs);
                     delay_ms(200);
                     SysRegs.SysMachine=System_STATE_INIT;
                 }
           //      SysRegs.PackStateReg.bit.SysSTATE =5;
            break;
            case System_STATE_MANUALMode:

            break;
            case System_STATE_ProtectHistory:

            break;

            case System_STATE_CLEAR:

            break;
            default :
            break;
        }
        if(SysRegs.Maincount>3000){SysRegs.Maincount=0;}
        if(CANTXFALAG==1)
        {
         //   MCP2515InitHandle(&CANBRegs);
            CANTXFALAG=0;
       }
        PrtectRelayRegs.State.bit.NRlyDI=SysRegs.DigitalInputReg.bit.NAUX;
        PrtectRelayRegs.State.bit.PRlyDI=SysRegs.DigitalInputReg.bit.PAUX;
        ProtectRelayHandle(&PrtectRelayRegs);
        SysRegs.DigitalOutPutReg.bit.NRlyOUT=PrtectRelayRegs.State.bit.NRlyDO;
        SysRegs.DigitalOutPutReg.bit.PRlyOUT=PrtectRelayRegs.State.bit.PRlyDO;
        SysRegs.DigitalOutPutReg.bit.ProRlyOUT=PrtectRelayRegs.State.bit.PreRlyDO;
        SysRegs.PackProtectReg.bit.PackRly_Err=PrtectRelayRegs.State.bit.RlyFaulttSate;

        SysRegs.PackStateReg.bit.NRlyDOStatus=SysRegs.DigitalInputReg.bit.NAUX;
        SysRegs.PackStateReg.bit.PRlyDOStatus=SysRegs.DigitalInputReg.bit.PAUX;
        SysRegs.PackStateReg.bit.PreRlyDOStatus=PrtectRelayRegs.State.bit.PreRlyDO;


   //     CANBRegs.SPISpeedHz=MeasureSPISpeedHandle(100);
    }
  /*  if(SysRegs.SysMachine==System_STATE_READY)//||(SysRegs.SysMachine==System_STATE_RUNING)||(SysRegs.SysMachine==System_STATE_PROTECTER))
    {
        SysRegs.SysMachine=System_STATE_STANDBY;
    }*/

}

interrupt void cpu_timer0_isr(void)
{
   SysRegs.MainIsr1++;
   SysRegs.SysRegTimer5msecCount++;
   SysRegs.SysRegTimer10msecCount++;
   SysRegs.SysRegTimer50msecCount++;
   SysRegs.SysRegTimer100msecCount++;
   SysRegs.SysRegTimer300msecCount++;
   SysRegs.SysRegTimer500msecCount++;
   SysRegs.SysRegTimer1000msecCount++;
   SysRegs.CellVoltsampling++;
   if(SysRegs.SysRegTimer5msecCount   >=SysRegTimer5msec)    {SysRegs.SysRegTimer5msecCount=0;}
   if(SysRegs.SysRegTimer10msecCount  >=SysRegTimer10msec)   {SysRegs.SysRegTimer10msecCount=0;}
   if(SysRegs.SysRegTimer50msecCount  >=SysRegTimer50msec)   {SysRegs.SysRegTimer50msecCount=0;}
   if(SysRegs.SysRegTimer100msecCount >=SysRegTimer100msec)  {SysRegs.SysRegTimer100msecCount=0;}
   if(SysRegs.SysRegTimer300msecCount >SysRegTimer300msec)   {SysRegs.SysRegTimer300msecCount=0;}
   if(SysRegs.SysRegTimer500msecCount >SysRegTimer500msec)   {SysRegs.SysRegTimer500msecCount=0;}
   if(SysRegs.SysRegTimer1000msecCount>SysRegTimer1000msec)  {SysRegs.SysRegTimer1000msecCount=0;}
   //if(SysRegs.PackStateReg.bit.CANRXCountReset==1)
   //{
   //    ModuleInit(&ModRegs);
   //    SysRegs.PackStateReg.bit.CANRXCountReset=0;
   //}
   /*
    * DigitalInput detection
    */
   DigitalInput(&SysRegs);
   //BatteryModel_1msTask();
   /*
    *
    */



  /*
   * current sensing detection
  */
   SysCurrentHandle(&SysRegs);

   /*
    * SOC Algorithm
    */
   Kam100AHSocRegs.CellAgvVoltageF = SysRegs.PackCellAgvVoltageF;
   Kam100AHSocRegs.SysSoCCTF       = SysRegs.PackCurrentF;
   Kam100AHSocRegs.SysSoCCTAbsF    = SysRegs.PackCurrentAsbF;
   Calkokam100AhSocHandle(&Kam100AHSocRegs);
   if(Kam100AHSocRegs.SoCStateRegs.bit.CalMeth==0)
   {
       SysRegs.PackSOCF=Kam100AHSocRegs.SysSocInitF;
   }
   if(Kam100AHSocRegs.SoCStateRegs.bit.CalMeth==1)
   {
       SysRegs.PackSOCF=Kam100AHSocRegs.SysSOCF;
   }


   /* ModRegs.MDCellMinTemps
    *
    *
    *
    * ModRegs.MDCellMaxTemps
    * ModRegs.MDCellMinTemp
    * ModRegs.MDMinTempsPo
    * ModRegs.MDMaxTempsPo
    */


   /*
    * 에러 검출
    */
  // SysCommErrHandle(&SysRegs);
   if(SysRegs.DigitalInputReg.bit.EMGSWStauts==1)
   {
       SysRegs.PackProtectReg.bit.PackEMSSWErr=1;
   }
   else
   {
       SysRegs.PackProtectReg.bit.PackEMSSWErr=0;
   }
   CANARegs.PackProtetSate=0;
   CalSysAlarmtCheck(&SysRegs);
   if(SysRegs.PackAlarmReg.all != 0)
   {
     CANARegs.PackProtetSate=1;
     SysRegs.PackStateReg.bit.SysAalarm=1;
    // SysRegs.PackStateReg.bit.SysProtectSate=1;
   }
   else
   {
     SysRegs.PackStateReg.bit.SysAalarm=0;
   //  SysRegs.PackStateReg.bit.SysProtectSate=0;
   }
   CalSysFaultCheck(&SysRegs);
   if(SysRegs.PackFaultReg.all != 0)
   {
       CANARegs.PackProtetSate=2;
     SysRegs.PackStateReg.bit.SysFault=1;
    // SysRegs.PackStateReg.bit.SysProtectSate=2;
   }
   else
   {
     SysRegs.PackStateReg.bit.SysFault=0;
    // SysRegs.PackStateReg.bit.SysProtectSate=0;
   }

   CalSysProtectCheck(&SysRegs);
   if(SysRegs.PackProtectReg.all != 0)
   {
       CANARegs.PackProtetSate=3;
       SysRegs.PackStateReg.bit.SysPrtct=1;
   //    SysRegs.PackStateReg.bit.SysProtectSate=3;
   }

   if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
   {
       SysRegs.SysMachine=System_STATE_INIT;
   }

   switch(SysRegs.SysRegTimer5msecCount)
   {
       case 1:

       break;
       default :
       break;

   }
   switch(SysRegs.SysRegTimer10msecCount)
   {
       case 1:
       break;
       case 2:
               /*
                *
                */
               memcpy(&BatCalcRegs.MDCellMaxVolt[0], &ModRegs.MDCellMaxVolt[0],sizeof(Uint16)* 9);
               memcpy(&BatCalcRegs.MDCellMinVolt[0], &ModRegs.MDCellMinVolt[0],sizeof(Uint16)* 9);
               memcpy(&BatCalcRegs.MDTotalVolt[0],   &ModRegs.MDTotalVolt[0],  sizeof(Uint16)* 9);
               memcpy(&BatCalcRegs.MDMaxVoltPo[0],   &ModRegs.MDMaxVoltPo[0],  sizeof(Uint16)* 9);
               memcpy(&BatCalcRegs.MDMinVoltPo[0],   &ModRegs.MDMinVoltPo[0],  sizeof(Uint16)* 9);
               BatCalcVoltHandle(&BatCalcRegs);

               SysRegs.PackVoltageF= BatCalcRegs.PackPTCANF;
               SysRegs.PackCellMaxVoltageF= BatCalcRegs.PackCellMaxVoltF;
               SysRegs.PackCellMinVoltageF= BatCalcRegs.PackCellMinVoltF;
               SysRegs.PackCellAgvVoltageF= BatCalcRegs.PackCellAgvVoltF;
               SysRegs.PackCellDivVoltageF= BatCalcRegs.PackCellDivVoltF;
               SysRegs.PackCellMaxVoltPos = BatCalcRegs.PackCellMaxVoltPos;
               SysRegs.PackCellMinVoltPos = BatCalcRegs.PackCellMinVoltPos;
               /*
               //SysRegs.PackVoltageF=gTrace_OutV*198.0f;
               //SysRegs.PackCellMaxVoltageF=gTrace_OutV;
               if(SysRegs.PackCurrentF>0)
               {
    //               SysRegs.PackCellMaxVoltageF=gTrace_OutV+0.05;
               }
               else if(SysRegs.PackCurrentF<0)
               {
     //              SysRegs.PackCellMaxVoltageF=gTrace_OutV+0.08;
               }
     //          SysRegs.PackCellMinVoltageF=gTrace_OutV-0.05;
               if(SysRegs.PackCurrentF>0)
               {
          //         SysRegs.PackCellMinVoltageF=gTrace_OutV-0.03;
               }
               else if(SysRegs.PackCurrentF<0)
               {
           //        SysRegs.PackCellMinVoltageF=gTrace_OutV-0.09;
               }*/
         //      SysRegs.PackCellDivVoltageF = SysRegs.PackCellMaxVoltageF-SysRegs.PackCellMinVoltageF;
        //       SysRegs.PackCellAgvVoltageF=  SysRegs.PackVoltageF/198.0f;
        //       SysRegs.PackCellMaxVoltPos = 56;
        //       SysRegs.PackCellMinVoltPos = 192;

       break;
       case 3:

       break;

       default :
       break;
   }
   switch(SysRegs.SysRegTimer50msecCount)
   {
       case 1:

       break;
       case 5:
               /*
                *
                */

               memcpy(&BatCalcRegs.MDCellMaxTemps[0], &ModRegs.MDCellMaxTemps[0],sizeof(Uint32)*7);
               memcpy(&BatCalcRegs.MDCellMinTemps[0], &ModRegs.MDCellMinTemps[0],sizeof(Uint32)*7);
               memcpy(&BatCalcRegs.MDMaxTempsPo[0],   &ModRegs.MDMaxTempsPo[0],sizeof(Uint32)*7);
               memcpy(&BatCalcRegs.MDMinTempsPo[0],   &ModRegs.MDMinTempsPo[0],sizeof(Uint32)*7);
               BatCalcTempsHandle(&BatCalcRegs);
               SysRegs.PackCellMaxTemperatureF= BatCalcRegs.PackCellMaxTempsF;
               SysRegs.PackCellMinTemperatureF= BatCalcRegs.PackCellMinTempsF;
               SysRegs.PackCellAgvTemperatureF= BatCalcRegs.PackCellAgvTempsF;
               SysRegs.PackCellDivTemperatureF= BatCalcRegs.PackCellDivTempsF;
               SysRegs.PackCellMaxTmepsPos    = BatCalcRegs.PackCellMaxTempsPos;
               SysRegs.PackCellMinTmepsPos    = BatCalcRegs.PackCellMinTempsPos;

              // SysRegs.PackCellMaxTemperatureF= RandVaule(gTrace_CellTempC+1.5);
               //SysRegs.PackCellMinTemperatureF= RandVaule(gTrace_CellTempC-0.5);

             //  SysRegs.PackCellDivTemperatureF= SysRegs.PackCellMaxTemperatureF-SysRegs.PackCellMinTemperatureF;
             //  SysRegs.PackCellAgvTemperatureF= RandVaule(gTrace_CellTempC);
             //  SysRegs.PackCellMaxTmepsPos    = RandVaule(74);
             //  SysRegs.PackCellMinTmepsPos    = RandVaule(10);
       break;
       case 10:
               if(SysRegs.PackStateReg.bit.INITOK==1)
               {
                   SysRegs.PackDisCHAPWRPeakF       = GetDischargeLimit30s_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
                   SysRegs.PackDisCHAPWRContintyF   = GetDischargeLimitCont_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
                   SysRegs.PackCHAPWRPeakF          = GetChargeLimit10s_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
                   SysRegs.PackCHAPWRContintyF      = GetChargeLimitCont_A(SysRegs.PackCellAgvTemperatureF, SysRegs.PackSOCF);
               }

       break;
       case 20:

       break;
       case 30 :


       break;
       default :
       break;
   }

   switch(SysRegs.SysRegTimer100msecCount)
   {

       case 1:
               #if(PackNum==1)
                   if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                   {
                       SysRegs.PackSOHF = 100.0;
                       //SysRegs.PackCurrentF=RandVaule(gBattery_OutputCurrentA);
                      // SysRegs.PackSOCF = gTrace_SOC;
                       CANARegs.PackPT = (unsigned int)(SysRegs.PackVoltageF*10);
                       CANARegs.PackCT = (int)(SysRegs.PackCurrentF*10.0);
                       CANARegs.PackSOC =(unsigned int)(SysRegs.PackSOCF*10);
                       CANARegs.PackSOH =(unsigned int)(SysRegs.PackSOHF*10);
                       CANARegs.PackID =0X601;
                       CANATX_29bit(CANARegs.PackID ,8,CANARegs.PackPT,CANARegs.PackCT,CANARegs.PackSOC,CANARegs.PackSOH);
                   }
               #endif
       break;
       case 3:
               #if(PackNum==1)
                   if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                   {
                        //CANARegs.PackSate
                        //CANARegs.PackProtetSate

                        CANARegs.PackSateInfo                    = ComBine(CANARegs.PackProtetSate, CANARegs.PackSate);
                        CANARegs.PackStatus.bit.PackNeg_Rly      = SysRegs.PackStateReg.bit.NRlyDOStatus;
                        CANARegs.PackStatus.bit.PackPos_Rly      = SysRegs.PackStateReg.bit.PRlyDOStatus;
                        CANARegs.PackStatus.bit.PackPreChar_Rly  = SysRegs.PackStateReg.bit.PreRlyDOStatus;
                        CANARegs.PackStatus.bit.PackMSD_AUX      = 1;//SysRegs.PackProtectReg.bit.MSD_Err;
                        CANARegs.PackStatus.bit.PackEMG_SW       = SysRegs.PackProtectReg.bit.PackEMSSWErr;
                        CANARegs.PackStatus.bit.PackWaterleak    = SysRegs.PackProtectReg.bit.PackWaterleakErr;
                        SysRegs.PackStateReg.bit.SysSeqState     = SysRegs.SysMachine;
                        SysRegs.PackStateReg.bit.RlySeqState     = PrtectRelayRegs.StateMachine;
                        SysRegs.PackStateReg.bit.SocSeqState     = Kam100AHSocRegs.state;
                        SysRegs.PackStateReg.bit.VCURlyWakeUp    = CANARegs.PMSCMDRegs.bit.RUNStatus01;
                        SysRegs.PackStateReg.bit.SysSocMode      = Kam100AHSocRegs.SoCStateRegs.bit.CalMeth;
                        SysRegs.PackStateReg.bit.SysAalarm       = 0;
                        SysRegs.PackStateReg.bit.SysAalarm       = 0;
                        SysRegs.PackStateReg.bit.SysPrtct        = 0;
                        CANARegs.PackAh = (int)(SysRegs.PackAhF*10);
                        CANARegs.PackID =0X602;
                        CANATX_29bit(CANARegs.PackID ,8,CANARegs.PackSateInfo,CANARegs.PackStatus.all,SysRegs.PackStateReg.Word.DataL,SysRegs.PackStateReg.Word.DataH);
                   }
                #endif

       break;

       case 5:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   //Cell MiN Voltage Offset 10mV
                   //SysRegs.PackCellMinVoltageF=3.0;
                   CANARegs.PackBalanVolt= (unsigned int)((SysRegs.PackCellMinVoltageF+C_BalanceVoltOffset)*1000);
                   #if(PackNum==1)
                       CANARegs.PackID =0X510;
                       CANATX(CANARegs.PackID,8,0X000,0X000,CANARegs.PackTemperatureAVG,CANARegs.PackBalanVolt);
                   #endif

               }
       break;
       case 8:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                #if(PackNum==1)
                   CANARegs.PackID =0X603;
                   CANATX_29bit(CANARegs.PackID,8,SysRegs.PackAlarmReg.Word.DataL,SysRegs.PackFaultReg.Word.DataL,SysRegs.PackProtectReg.Word.DataL,SysRegs.PackProtectReg.Word.DataH);
                #endif

               }

       break;
       case 11:

                //At 80MHZ, operation time is 0.151msec
               #if(PackNum==1)
                   if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                   {
                     //SysRegs.PackCHAPWRContintyF    =  43.0;
                     //SysRegs.PackDisCHAPWRContintyF =  92.7;
                     //SysRegs.PackCHAPWRPeakF        =  60.0;
                     //SysRegs.PackDisCHAPWRPeakF     =  150,0;
                     CANARegs.PackCHAPWRContinty    = (unsigned int)(SysRegs.PackCHAPWRContintyF*10);
                     CANARegs.PackDisCHAPWRContinty = (unsigned int)(SysRegs.PackDisCHAPWRContintyF*10);
                     CANARegs.PackCHAPWRPeak        = (unsigned int)(SysRegs.PackCHAPWRPeakF*10);
                     CANARegs.PackDisCHAPWRPeak     = (unsigned int)(SysRegs.PackDisCHAPWRPeakF*10);
                     CANARegs.PackID =0X604;
                     CANATX_29bit(CANARegs.PackID,8,CANARegs.PackCHAPWRContinty,CANARegs.PackDisCHAPWRContinty,CANARegs.PackCHAPWRPeak,CANARegs.PackDisCHAPWRPeak);
                   }
                #endif

       break;
       case 14:
                #if(PackNum==1)
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                   CANARegs.PackVoltageMax = (unsigned int)(SysRegs.PackCellMaxVoltageF*1000);
                   CANARegs.PackVoltageMin = (unsigned int)(SysRegs.PackCellMinVoltageF*1000);
                   CANARegs.PackVoltageAgv = (unsigned int)(SysRegs.PackCellAgvVoltageF*1000);
                   CANARegs.PackVoltageDiv = (unsigned int)(SysRegs.PackCellDivVoltageF*1000);
                   CANARegs.PackID =0X605;
                   CANATX_29bit(CANARegs.PackID,8,CANARegs.PackVoltageMax,CANARegs.PackVoltageMin,CANARegs.PackVoltageAgv,CANARegs.PackVoltageDiv);
                }
                #endif

       break;
       case 17:
                #if(PackNum==1)
                if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                    SysRegs.PackCellAgvTemperatureF= SysRegs.tempC_f;
                    CANARegs.PackTemperaturelMAX    = (unsigned int)(SysRegs.PackCellMaxTemperatureF*10);
                    CANARegs.PackTemperaturelMIN    = (unsigned int)(SysRegs.PackCellMinTemperatureF*10);
                    CANARegs.PackTemperatureAVG     = (int)(SysRegs.PackCellAgvTemperatureF*10);
                    CANARegs.PackTemperatureDiv     = (unsigned int)(SysRegs.PackCellDivTemperatureF*10);
                    CANARegs.PackID =0X606;
                    CANATX_29bit(CANARegs.PackID,8,CANARegs.PackTemperaturelMAX,CANARegs.PackTemperaturelMIN,CANARegs.PackTemperatureAVG,CANARegs.PackTemperatureDiv);
                }
                #endif

       break;
       case 20:
               #if(PackNum==1)
                    if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                    {
                        CANARegs.PackVoltageMaxNum          = SysRegs.PackCellMaxVoltPos;
                        CANARegs.PackVoltageMinNum          = SysRegs.PackCellMinVoltPos;
                        CANARegs.PackTemperatureMaxNUM      = SysRegs.PackCellMaxTmepsPos;
                        CANARegs.PackTemperatureMinNUM      = SysRegs.PackCellMinTmepsPos;
                        CANARegs.PackID =0X607;
                        CANATX_29bit(CANARegs.PackID,8,CANARegs.PackVoltageMaxNum,CANARegs.PackVoltageMinNum,CANARegs.PackTemperatureMaxNUM ,CANARegs.PackTemperatureMinNUM );
                    }
               #endif
       break;
       case 23:

       break;
       case 26:

       break;
       case 30:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {

               }
       break;
       case 35:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {

               }
       break;
       case 38:
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {

               }
       break;
       default:
       break;
   }
   switch(SysRegs.SysRegTimer300msecCount)
   {
       case 1:
            #if(PackNum==1)
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.MDTotalVolt[CANARegs.MDNumCountA]        = (Uint16)((float32)ModRegs.MDTotalVolt[CANARegs.MDNumCountA]*0.1);
                   CANARegs.MDstatusbit[CANARegs.MDNumCountA]        = ModRegs.MDstatusbit[CANARegs.MDNumCountA];
                   CANARegs.PackMinVolteRec[CANARegs.MDNumCountA]    = ModRegs.PackMinVolteRec[CANARegs.MDNumCountA];

                   CANARegs.PackID =0X608;
                   CANATX_29bit(CANARegs.PackID,8,CANARegs.MDNumCountA,CANARegs.MDTotalVolt[CANARegs.MDNumCountA],CANARegs.MDstatusbit[CANARegs.MDNumCountA],CANARegs.PackMinVolteRec[CANARegs.MDNumCountA]);
                   if(++CANARegs.MDNumCountA>=7)
                   {
                       CANARegs.MDNumCountA=0;
                   }
               }
               else
               {
                   CANARegs.MDNumCountA=0;
               }
            #endif
       break;
       case 10:
               #if(PackNum==1)
                   if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                   {

                       CANARegs.MDCellMaxVolt[CANARegs.MDNumCountB] = ModRegs.MDCellMaxVolt[CANARegs.MDNumCountB];
                       CANARegs.MDCellMinVolt[CANARegs.MDNumCountB] = ModRegs.MDCellMinVolt[CANARegs.MDNumCountB];
                       CANARegs.MDCellAgvVolt[CANARegs.MDNumCountB] = ModRegs.MDCellAgvVolt[CANARegs.MDNumCountB];

                       CANARegs.PackID =0X609;
                       CANATX_29bit(CANARegs.PackID,8,CANARegs.MDNumCountB,CANARegs.MDCellMaxVolt[CANARegs.MDNumCountB],CANARegs.MDCellMinVolt[CANARegs.MDNumCountB] ,CANARegs.MDCellAgvVolt[CANARegs.MDNumCountB]);

                       if(++CANARegs.MDNumCountB>=7)
                       {
                           CANARegs.MDNumCountB=0;
                       }
                   }
                   else
                   {
                       CANARegs.MDNumCountB=0;
                   }
              #endif

       break;
       case 20:
               #if(PackNum==1)
                   if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                   {
                       CANARegs.MDCellMaxTemps[CANARegs.MDNumCountB] = ModRegs.MDCellMaxTemps[CANARegs.MDNumCountB];
                       CANARegs.MDCellMinTemps[CANARegs.MDNumCountB] = ModRegs.MDCellMinTemps[CANARegs.MDNumCountB];
                       CANARegs.MDCellAgvTemps[CANARegs.MDNumCountB] = ModRegs.MDCellAgvTemps[CANARegs.MDNumCountB];

                       CANARegs.PackID =0X60A;
                       CANATX_29bit(CANARegs.PackID,8,CANARegs.MDNumCountC,CANARegs.MDCellMaxTemps[CANARegs.MDNumCountC],CANARegs.MDCellMinTemps[CANARegs.MDNumCountC] ,CANARegs.MDCellAgvTemps[CANARegs.MDNumCountC]);
                       if(++CANARegs.MDNumCountC>=7)
                       {
                          CANARegs.MDNumCountC=0;
                       }
                   }
                   else
                   {
                       CANARegs.MDNumCountC=0;
                   }
               #endif

       break;

       case 30:
               #if(PackNum==1)
                   if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                   {

                       CANARegs.MDCellDivVolt[CANARegs.MDNumCountD]    =  ModRegs.MDCellDivVolt[CANARegs.MDNumCountD];
                       CANARegs.MDCellDivTemps[CANARegs.MDNumCountD]   =  ModRegs.MDCellDivTemps[CANARegs.MDNumCountD];
                       CANARegs.MDInResis[CANARegs.MDNumCountD]        =  ModRegs.MDInResis[CANARegs.MDNumCountD];

                       CANARegs.PackID =0X60B;
                       CANATX_29bit(CANARegs.PackID,8,CANARegs.MDNumCountD,CANARegs.MDCellDivVolt[CANARegs.MDNumCountD],CANARegs.MDCellDivTemps[CANARegs.MDNumCountD],CANARegs.MDInResis[CANARegs.MDNumCountD]);
                       if(++CANARegs.MDNumCountD>=7)
                       {
                           CANARegs.MDNumCountD=0;
                       }
                       CANARegs.PackID =0X60B;
                   }
                   else
                   {
                       CANARegs.MDNumCountD=0;
                   }
               #endif

       break;
       case 40:
            #if(PackNum==1)
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                    if(ModRegs.MailBox0Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox0Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox1Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox1Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox2Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox2Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox3Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox3Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox4Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox4Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox5Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox5Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox6Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox6Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox7Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox7Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox8Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox8Rxcount[CANARegs.MDNumCountE]=0;}
                    if(ModRegs.MailBox9Rxcount[CANARegs.MDNumCountE]>200){ModRegs.MailBox9Rxcount[CANARegs.MDNumCountE]=0;}
                    ModRegs.MDN1CanRxCntVaule = ComBine(ModRegs.MailBox0Rxcount[CANARegs.MDNumCountE],CANARegs.MDNumCountE);
                    ModRegs.MD23CanRxCntVaule = ComBine(ModRegs.MailBox2Rxcount[CANARegs.MDNumCountE],ModRegs.MailBox1Rxcount[CANARegs.MDNumCountE]);
                    ModRegs.MD45CanRxCntVaule = ComBine(ModRegs.MailBox4Rxcount[CANARegs.MDNumCountE],ModRegs.MailBox3Rxcount[CANARegs.MDNumCountE]);
                    ModRegs.MD67CanRxCntVaule = ComBine(ModRegs.MailBox6Rxcount[CANARegs.MDNumCountE],ModRegs.MailBox5Rxcount[CANARegs.MDNumCountE]);
                    CANARegs.PackID =0X60C;
                    CANATX_29bit(CANARegs.PackID,8,ModRegs.MDN1CanRxCntVaule,ModRegs.MD23CanRxCount,ModRegs.MD45CanRxCount,ModRegs.MD67CanRxCount);

                    if(++CANARegs.MDNumCountE>=9)
                    {
                        CANARegs.MDNumCountE=0;
                    }
                }
                else
                {
                    CANARegs.MDNumCountE=0;
                }
            #endif

       break;
       case 50:
            #if(PackNum==1)
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
                {
                    CANARegs.PackID =0X60D;
                    CANATX_29bit(CANARegs.PackID,8,SysRegs.Maincount,SysRegs.MainIsr1,CANARegs.MailBox1RxCount,CANARegs.MailBox2RxCount);

                }
            #endif

       break;
       case 60:
            #if(PackNum==1)
               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.PackID =0X60E;
                   CANARegs.DebugStaueA.all=(unsigned int)(SysRegs.PackCurrentAsbF*10);
                   CANATX_29bit(CANARegs.PackID,8,CANARegs.DebugStaueA.all,CANARegs.DebugStaueB.all,CANARegs.DebugStaueC.all,CANARegs.DebugStaueD.all);
               }
            #endif

       break;
       case 65:
               CANARegs.PackID =0X60F;
               CANARegs.DebugStaueE.all = CANARegs.PMSCMDRegs.all;
               CANATX_29bit(CANARegs.PackID,8,CANARegs.DebugStaueE.all,CANARegs.DebugStaueF.all,CANARegs.DebugStaueG.all ,CANARegs.DebugStaueH.all);
       break;
       case 80:

               TestCurrentlimt_StepUpdate_200ms(&SysRegs);

       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer500msecCount)
   {
       case 1:
       break;

       case 40:

               SysRegs.PackModule1Regs.all = ModRegs.MDstatusbit[0];
               SysRegs.PackModule2Regs.all = ModRegs.MDstatusbit[1];
               SysRegs.PackModule3Regs.all = ModRegs.MDstatusbit[2];
               SysRegs.PackModule4Regs.all = ModRegs.MDstatusbit[3];
               SysRegs.PackModule5Regs.all = ModRegs.MDstatusbit[4];
               SysRegs.PackModule6Regs.all = ModRegs.MDstatusbit[5];
               SysRegs.PackModule7Regs.all = ModRegs.MDstatusbit[6];
               SysRegs.PackModule8Regs.all = ModRegs.MDstatusbit[7];
               SysRegs.PackModule9Regs.all = ModRegs.MDstatusbit[7];
       break;
       case 60:

       break;
       case 80:

       break;
       case 100:

       break;
       case 150:

       break;
       case 200:

       break;
       case 250:

       break;
       default :
       break;
   }
   switch(SysRegs.SysRegTimer1000msecCount)
   {
       case 1:

               if(SysRegs.PackStateReg.bit.CANCOMEnable==1)
               {
                   CANARegs.SwVerProducttype      = ComBine(Product_Version,Product_Type);
                   CANARegs.BatConfParallelSerial = ComBine(Product_SysCellVauleP,Product_SysCellVauleS);
                   #if(PackNum==1)
                       CANARegs.PackID =0X600;
                       CANATX_29bit(CANARegs.PackID,8,CANARegs.SwVerProducttype,(unsigned int)(Product_Voltage*10),(unsigned int)(Product_Capacity*10),CANARegs.BatConfParallelSerial);
                   #endif

               }

       break;
       default :
       break;
   }
   DigitalOutput(&SysRegs);

   //

   // Acknowledge this interrupt to receive more interrupts from group 1
   if(SysRegs.MainIsr1>3000) {SysRegs.MainIsr1=0;}
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
interrupt void ISR_CANRXINTA(void)
{
 //   struct ECAN_REGS ECanaShadow;
    CANARegs.MailBoxRxCount++;
    if(CANARegs.MailBoxRxCount>3000){CANARegs.MailBoxRxCount=0;}
    if(ECanaRegs.CANGIF0.bit.GMIF0 == 1U)
    {
        if(ECanaRegs.CANRMP.bit.RMP0==1U)
        {
            ModRegs.MailBox1Rxcount[0]++;
            SysRegs.MD1CANRxCount=0;
             switch(ECanaMboxes.MBOX0.MSGID.bit.STDMSGID)
             {
                #if(PackNum==1)
                 case (0x112)://1

                         ModRegs.MailBox1Rxcount[1]++;
                         ModRegs.MDCellVoltQty[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE0;
                         ModRegs.MDFirmwareVer[0]    = ECanaMboxes.MBOX0.MDL.byte.BYTE1;
                         ModRegs.MDNorVolt[0]        = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                         ModRegs.MDNorCapacity[0]    = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                         ModRegs.PackMinVolteRec[0]  = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);
                 break;
                 case (0x113)://2
                         ModRegs.MailBox1Rxcount[2]++;
                         ModRegs.MDCellMaxVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                         ModRegs.MDCellMinVolt[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                         ModRegs.MDCellAgvVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                         ModRegs.MDCellDivVolt[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                 break;
                 case (0x114)://3

                         ModRegs.MailBox1Rxcount[3]++;
                         ModRegs.MDCellMaxTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                         ModRegs.MDCellMinTemps[0] = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE3,ECanaMboxes.MBOX0.MDL.byte.BYTE2);
                         ModRegs.MDCellAgvTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE5,ECanaMboxes.MBOX0.MDH.byte.BYTE4);
                         ModRegs.MDCellDivTemps[0] = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);

                break;
                case (0x115)://4
                         ModRegs.MailBox1Rxcount[4]++;
                         ModRegs.MDTotalVolt[0]      = ComBine(ECanaMboxes.MBOX0.MDL.byte.BYTE1,ECanaMboxes.MBOX0.MDL.byte.BYTE0);
                         ModRegs.MDMaxVoltPo[0]      = ECanaMboxes.MBOX0.MDL.byte.BYTE2;
                         ModRegs.MDMinVoltPo[0]      = ECanaMboxes.MBOX0.MDL.byte.BYTE3;
                         ModRegs.MDMaxTempsPo[0]     = ECanaMboxes.MBOX0.MDH.byte.BYTE4;
                         ModRegs.MDMinTempsPo[0]     = ECanaMboxes.MBOX0.MDH.byte.BYTE5;
                         ModRegs.MDstatusbit[0]      = ComBine(ECanaMboxes.MBOX0.MDH.byte.BYTE7,ECanaMboxes.MBOX0.MDH.byte.BYTE6);


                 break;


                 default:
                 break;
                #endif

             }
               ECanaRegs.CANRMP.bit.RMP0 = 1;
            // ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            // ECanaShadow.CANRMP.all=0;
            // ECanaShadow.CANRMP.bit.RMP0 = 1;
            // ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP1==1) //0XR2~4 , R1~4)
        {
            ModRegs.MailBox2Rxcount[0]++;
            SysRegs.MD2CANRxCount=0;
            switch(ECanaMboxes.MBOX1.MSGID.bit.STDMSGID)
            {
             #if(PackNum==1)
                case (0x122):
                                ModRegs.MailBox2Rxcount[1]++;
                                ModRegs.MDCellVoltQty[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE0;
                                ModRegs.MDFirmwareVer[1]    = ECanaMboxes.MBOX1.MDL.byte.BYTE1;
                                ModRegs.MDNorVolt[1]        = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                                ModRegs.MDNorCapacity[1]    = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                                ModRegs.PackMinVolteRec[1]  = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

                break;
                case (0x123):
                                ModRegs.MailBox2Rxcount[2]++;
                                ModRegs.MDCellMaxVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                                ModRegs.MDCellMinVolt[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                                ModRegs.MDCellDivVolt[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

                break;
                case (0x124):

                                ModRegs.MailBox2Rxcount[3]++;
                                ModRegs.MDCellMaxTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                                ModRegs.MDCellMinTemps[1] = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE3,ECanaMboxes.MBOX1.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE5,ECanaMboxes.MBOX1.MDH.byte.BYTE4);
                                ModRegs.MDCellDivTemps[1] = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);

                break;
                case (0x125):
                                ModRegs.MailBox2Rxcount[4]++;
                                ModRegs.MDTotalVolt[1]     = ComBine(ECanaMboxes.MBOX1.MDL.byte.BYTE1,ECanaMboxes.MBOX1.MDL.byte.BYTE0);
                                ModRegs.MDMaxVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE2;
                                ModRegs.MDMinVoltPo[1]     = ECanaMboxes.MBOX1.MDL.byte.BYTE3;
                                ModRegs.MDMaxTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE4;
                                ModRegs.MDMinTempsPo[1]    = ECanaMboxes.MBOX1.MDH.byte.BYTE5;
                                ModRegs.MDstatusbit[1]     = ComBine(ECanaMboxes.MBOX1.MDH.byte.BYTE7,ECanaMboxes.MBOX1.MDH.byte.BYTE6);
                break;
             #endif


               default:
               break;
            }
              ECanaRegs.CANRMP.bit.RMP1 = 1;
          //  ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
          //  ECanaShadow.CANRMP.all=0;
          //  ECanaShadow.CANRMP.bit.RMP1 = 1;
          //  ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
        }
        if(ECanaRegs.CANRMP.bit.RMP2==1) //0XR31~4 , R1~4)
        {
            ModRegs.MailBox3Rxcount[0]++;
            SysRegs.MD3CANRxCount=0;
            switch(ECanaMboxes.MBOX2.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                case (0x132):
                            ModRegs.MailBox3Rxcount[1]++;
                            ModRegs.MDCellVoltQty[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE0;
                            ModRegs.MDFirmwareVer[2]    = ECanaMboxes.MBOX2.MDL.byte.BYTE1;
                            ModRegs.MDNorVolt[2]        = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                            ModRegs.MDNorCapacity[2]    = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                            ModRegs.PackMinVolteRec[2]  = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                break;
                case (0x133):
                            ModRegs.MailBox3Rxcount[2]++;
                            ModRegs.MDCellMaxVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                            ModRegs.MDCellMinVolt[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                            ModRegs.MDCellDivVolt[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                break;
                case (0x134):

                            ModRegs.MailBox3Rxcount[3]++;
                            ModRegs.MDCellMaxTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                            ModRegs.MDCellMinTemps[2] = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE3,ECanaMboxes.MBOX2.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE5,ECanaMboxes.MBOX2.MDH.byte.BYTE4);
                            ModRegs.MDCellDivTemps[2] = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                break;
                case (0x135):
                            ModRegs.MailBox3Rxcount[4]++;
                            ModRegs.MDTotalVolt[2]     = ComBine(ECanaMboxes.MBOX2.MDL.byte.BYTE1,ECanaMboxes.MBOX2.MDL.byte.BYTE0);
                            ModRegs.MDMaxVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE2;
                            ModRegs.MDMinVoltPo[2]     = ECanaMboxes.MBOX2.MDL.byte.BYTE3;
                            ModRegs.MDMaxTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE4;
                            ModRegs.MDMinTempsPo[2]    = ECanaMboxes.MBOX2.MDH.byte.BYTE5;
                            ModRegs.MDstatusbit[2]     = ComBine(ECanaMboxes.MBOX2.MDH.byte.BYTE7,ECanaMboxes.MBOX2.MDH.byte.BYTE6);
                break;

                #endif
                default:
                break;
            }
            ECanaRegs.CANRMP.bit.RMP2= 1;
           // ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
           // ECanaShadow.CANRMP.all=0;
           // ECanaShadow.CANRMP.bit.RMP2= 1;
           // ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP3==1)
        {
            ModRegs.MailBox4Rxcount[0]++;
            SysRegs.MD4CANRxCount=0;
            switch(ECanaMboxes.MBOX3.MSGID.bit.STDMSGID)
            {
             #if(PackNum==1)
                case (0x142):
                            ModRegs.MailBox4Rxcount[1]++;
                            ModRegs.MDCellVoltQty[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE0;
                            ModRegs.MDFirmwareVer[3]    = ECanaMboxes.MBOX3.MDL.byte.BYTE1;
                            ModRegs.MDNorVolt    [3]    = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                            ModRegs.MDNorCapacity[3]    = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                            ModRegs.PackMinVolteRec[3]  = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

                break;
                case (0x143):
                            ModRegs.MailBox4Rxcount[2]++;
                            ModRegs.MDCellMaxVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                            ModRegs.MDCellMinVolt[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                            ModRegs.MDCellDivVolt[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

                break;
                case (0x144):

                            ModRegs.MailBox4Rxcount[3]++;
                            ModRegs.MDCellMaxTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                            ModRegs.MDCellMinTemps[3] = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE3,ECanaMboxes.MBOX3.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE5,ECanaMboxes.MBOX3.MDH.byte.BYTE4);
                            ModRegs.MDCellDivTemps[3] = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);

                break;
                case (0x145):
                            ModRegs.MailBox4Rxcount[4]++;
                            ModRegs.MDTotalVolt[3]     = ComBine(ECanaMboxes.MBOX3.MDL.byte.BYTE1,ECanaMboxes.MBOX3.MDL.byte.BYTE0);
                            ModRegs.MDMaxVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE2;
                            ModRegs.MDMinVoltPo[3]     = ECanaMboxes.MBOX3.MDL.byte.BYTE3;
                            ModRegs.MDMaxTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE4;
                            ModRegs.MDMinTempsPo[3]    = ECanaMboxes.MBOX3.MDH.byte.BYTE5;
                            ModRegs.MDstatusbit[3]     = ComBine(ECanaMboxes.MBOX3.MDH.byte.BYTE7,ECanaMboxes.MBOX3.MDH.byte.BYTE6);
                break;
            #endif

                default:
                break;
            }
              ECanaRegs.CANRMP.bit.RMP3 = 1;
          //  ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
          //  ECanaShadow.CANRMP.all=0;
          //  ECanaShadow.CANRMP.bit.RMP3 = 1;
          //  ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP4==1)
        {
            ModRegs.MailBox5Rxcount[0]++;
            SysRegs.MD5CANRxCount=0;
            switch(ECanaMboxes.MBOX4.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                    case (0x152):
                                ModRegs.MailBox5Rxcount[1]++;
                                ModRegs.MDCellVoltQty[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE0;
                                ModRegs.MDFirmwareVer[4]    = ECanaMboxes.MBOX4.MDL.byte.BYTE1;
                                ModRegs.MDNorVolt[4]        = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                                ModRegs.MDNorCapacity[4]    = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                                ModRegs.PackMinVolteRec[4]  = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                    break;
                    case (0x153):
                                ModRegs.MailBox5Rxcount[2]++;
                                ModRegs.MDCellMaxVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                                ModRegs.MDCellMinVolt[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                                ModRegs.MDCellDivVolt[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                    break;
                    case (0x154):
                                ModRegs.MailBox5Rxcount[3]++;
                                ModRegs.MDCellMaxTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                                ModRegs.MDCellMinTemps[4] = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE3,ECanaMboxes.MBOX4.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE5,ECanaMboxes.MBOX4.MDH.byte.BYTE4);
                                ModRegs.MDCellDivTemps[4] = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                    break;
                    case (0x155):
                                ModRegs.MailBox5Rxcount[4]++;
                                ModRegs.MDTotalVolt[4]     = ComBine(ECanaMboxes.MBOX4.MDL.byte.BYTE1,ECanaMboxes.MBOX4.MDL.byte.BYTE0);
                                ModRegs.MDMaxVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE2;
                                ModRegs.MDMinVoltPo[4]     = ECanaMboxes.MBOX4.MDL.byte.BYTE3;
                                ModRegs.MDMaxTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE4;
                                ModRegs.MDMinTempsPo[4]    = ECanaMboxes.MBOX4.MDH.byte.BYTE5;
                                ModRegs.MDstatusbit[4]     = ComBine(ECanaMboxes.MBOX4.MDH.byte.BYTE7,ECanaMboxes.MBOX4.MDH.byte.BYTE6);
                    break;
                #endif

                default:
                break;
            }
              ECanaRegs.CANRMP.bit.RMP4 = 1;
           // ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
           // ECanaShadow.CANRMP.all=0;
           // ECanaShadow.CANRMP.bit.RMP4 = 1;
           // ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
         }
        if(ECanaRegs.CANRMP.bit.RMP5==1)
        {
            ModRegs.MailBox6Rxcount[0]++;
            SysRegs.MD6CANRxCount=0;
            switch(ECanaMboxes.MBOX5.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                    case (0x162):
                                ModRegs.MailBox6Rxcount[1]++;
                                ModRegs.MDCellVoltQty[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE0;
                                ModRegs.MDFirmwareVer[5]    = ECanaMboxes.MBOX5.MDL.byte.BYTE1;
                                ModRegs.MDNorVolt[5]        = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                                ModRegs.MDNorCapacity[5]    = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                                ModRegs.PackMinVolteRec[5]  = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                    break;
                    case (0x163):
                                ModRegs.MailBox6Rxcount[2]++;
                                ModRegs.MDCellMaxVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                                ModRegs.MDCellMinVolt[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                                ModRegs.MDCellDivVolt[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                    break;
                    case (0x164):

                                ModRegs.MailBox6Rxcount[3]++;
                                ModRegs.MDCellMaxTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                                ModRegs.MDCellMinTemps[5] = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE3,ECanaMboxes.MBOX5.MDL.byte.BYTE2);
                                ModRegs.MDCellAgvTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE5,ECanaMboxes.MBOX5.MDH.byte.BYTE4);
                                ModRegs.MDCellDivTemps[5] = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                    break;
                    case (0x165):
                                ModRegs.MailBox6Rxcount[4]++;
                                ModRegs.MDTotalVolt[5]     = ComBine(ECanaMboxes.MBOX5.MDL.byte.BYTE1,ECanaMboxes.MBOX5.MDL.byte.BYTE0);
                                ModRegs.MDMaxVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE2;
                                ModRegs.MDMinVoltPo[5]     = ECanaMboxes.MBOX5.MDL.byte.BYTE3;
                                ModRegs.MDMaxTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE4;
                                ModRegs.MDMinTempsPo[5]    = ECanaMboxes.MBOX5.MDH.byte.BYTE5;
                                ModRegs.MDstatusbit[5]     = ComBine(ECanaMboxes.MBOX5.MDH.byte.BYTE7,ECanaMboxes.MBOX5.MDH.byte.BYTE6);
                    break;
                #endif

                default:
                break;
            }
              ECanaRegs.CANRMP.bit.RMP5 = 1;
            //ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            //ECanaShadow.CANRMP.all=0;
            //ECanaShadow.CANRMP.bit.RMP5 = 1;
            //ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP6==1)
        {
            ModRegs.MailBox7Rxcount[0]++;
            SysRegs.MD7CANRxCount=0;
            switch(ECanaMboxes.MBOX6.MSGID.bit.STDMSGID)
            {
              #if(PackNum==1)
                case (0x172):
                            ModRegs.MailBox7Rxcount[1]++;
                            ModRegs.MDCellVoltQty[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE0;
                            ModRegs.MDFirmwareVer[6]    = ECanaMboxes.MBOX6.MDL.byte.BYTE1;
                            ModRegs.MDNorVolt[6]        = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                            ModRegs.MDNorCapacity[6]    = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                            ModRegs.PackMinVolteRec[6]  = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

                break;
                case (0x173):
                            ModRegs.MailBox7Rxcount[2]++;
                            ModRegs.MDCellMaxVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                            ModRegs.MDCellMinVolt[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                            ModRegs.MDCellDivVolt[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

                break;
                case (0x174):

                            ModRegs.MailBox7Rxcount[3]++;
                            ModRegs.MDCellMaxTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                            ModRegs.MDCellMinTemps[6] = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE3,ECanaMboxes.MBOX6.MDL.byte.BYTE2);
                            ModRegs.MDCellAgvTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE5,ECanaMboxes.MBOX6.MDH.byte.BYTE4);
                            ModRegs.MDCellDivTemps[6] = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);

                break;
                case (0x175):
                            ModRegs.MailBox7Rxcount[4]++;
                            ModRegs.MDTotalVolt[6]     = ComBine(ECanaMboxes.MBOX6.MDL.byte.BYTE1,ECanaMboxes.MBOX6.MDL.byte.BYTE0);
                            ModRegs.MDMaxVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE2;
                            ModRegs.MDMinVoltPo[6]     = ECanaMboxes.MBOX6.MDL.byte.BYTE3;
                            ModRegs.MDMaxTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE4;
                            ModRegs.MDMinTempsPo[6]    = ECanaMboxes.MBOX6.MDH.byte.BYTE5;
                            ModRegs.MDstatusbit[6]     = ComBine(ECanaMboxes.MBOX6.MDH.byte.BYTE7,ECanaMboxes.MBOX6.MDH.byte.BYTE6);
                break;
              #endif
                default:
                break;

            }
            ECanaRegs.CANRMP.bit.RMP6 = 1;
            //ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            //ECanaShadow.CANRMP.all=0;
            //ECanaShadow.CANRMP.bit.RMP6 = 1;
            //ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
        if(ECanaRegs.CANRMP.bit.RMP7==1)
        {
            ModRegs.MailBox8Rxcount[0]++;
            SysRegs.MD7CANRxCount=0;
            switch(ECanaMboxes.MBOX7.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                  case (0x182):
                              ModRegs.MailBox8Rxcount[1]++;
                              ModRegs.MDCellVoltQty[7]    = ECanaMboxes.MBOX7.MDL.byte.BYTE0;
                              ModRegs.MDFirmwareVer[7]    = ECanaMboxes.MBOX7.MDL.byte.BYTE1;
                              ModRegs.MDNorVolt[7]        = ComBine(ECanaMboxes.MBOX7.MDL.byte.BYTE3,ECanaMboxes.MBOX7.MDL.byte.BYTE2);
                              ModRegs.MDNorCapacity[7]    = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE5,ECanaMboxes.MBOX7.MDH.byte.BYTE4);
                              ModRegs.PackMinVolteRec[7]  = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE7,ECanaMboxes.MBOX7.MDH.byte.BYTE6);

                  break;
                  case (0x183):
                              ModRegs.MailBox8Rxcount[2]++;
                              ModRegs.MDCellMaxVolt[7] = ComBine(ECanaMboxes.MBOX7.MDL.byte.BYTE1,ECanaMboxes.MBOX7.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[7] = ComBine(ECanaMboxes.MBOX7.MDL.byte.BYTE3,ECanaMboxes.MBOX7.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[7] = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE5,ECanaMboxes.MBOX7.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[7] = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE7,ECanaMboxes.MBOX7.MDH.byte.BYTE6);

                  break;
                  case (0x184):

                              ModRegs.MailBox8Rxcount[3]++;
                              ModRegs.MDCellMaxTemps[7] = ComBine(ECanaMboxes.MBOX7.MDL.byte.BYTE1,ECanaMboxes.MBOX7.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[7] = ComBine(ECanaMboxes.MBOX7.MDL.byte.BYTE3,ECanaMboxes.MBOX7.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[7] = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE5,ECanaMboxes.MBOX7.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[7] = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE7,ECanaMboxes.MBOX7.MDH.byte.BYTE6);

                  break;
                  case (0x185):
                              ModRegs.MailBox8Rxcount[4]++;
                              ModRegs.MDTotalVolt[7]     = ComBine(ECanaMboxes.MBOX7.MDL.byte.BYTE1,ECanaMboxes.MBOX7.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[7]     = ECanaMboxes.MBOX7.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[7]     = ECanaMboxes.MBOX7.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[7]    = ECanaMboxes.MBOX7.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[7]    = ECanaMboxes.MBOX7.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[7]     = ComBine(ECanaMboxes.MBOX7.MDH.byte.BYTE7,ECanaMboxes.MBOX7.MDH.byte.BYTE6);
                  break;
                #endif

                default:
                break;
            }
            ECanaRegs.CANRMP.bit.RMP7 = 1;
        }
        if(ECanaRegs.CANRMP.bit.RMP8==1)
        {
            ModRegs.MailBox9Rxcount[0]++;
            SysRegs.MD8CANRxCount=0;
            switch(ECanaMboxes.MBOX8.MSGID.bit.STDMSGID)
            {
                #if(PackNum==1)
                  case (0x192):
                              ModRegs.MailBox9Rxcount[1]++;
                              ModRegs.MDCellVoltQty[8]    = ECanaMboxes.MBOX8.MDL.byte.BYTE0;
                              ModRegs.MDFirmwareVer[8]    = ECanaMboxes.MBOX8.MDL.byte.BYTE1;
                              ModRegs.MDNorVolt[8]        = ComBine(ECanaMboxes.MBOX8.MDL.byte.BYTE3,ECanaMboxes.MBOX8.MDL.byte.BYTE2);
                              ModRegs.MDNorCapacity[8]    = ComBine(ECanaMboxes.MBOX8.MDH.byte.BYTE5,ECanaMboxes.MBOX8.MDH.byte.BYTE4);
                              ModRegs.PackMinVolteRec[8]  = ComBine(ECanaMboxes.MBOX8.MDH.byte.BYTE7,ECanaMboxes.MBOX8.MDH.byte.BYTE6);

                  break;
                  case (0x193):
                              ModRegs.MailBox9Rxcount[2]++;
                              ModRegs.MDCellMaxVolt[8] = ComBine(ECanaMboxes.MBOX8.MDL.byte.BYTE1,ECanaMboxes.MBOX8.MDL.byte.BYTE0);
                              ModRegs.MDCellMinVolt[8] = ComBine(ECanaMboxes.MBOX8.MDL.byte.BYTE3,ECanaMboxes.MBOX8.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvVolt[8] = ComBine(ECanaMboxes.MBOX8.MDH.byte.BYTE5,ECanaMboxes.MBOX8.MDH.byte.BYTE4);
                              ModRegs.MDCellDivVolt[8] = ComBine(ECanaMboxes.MBOX8.MDH.byte.BYTE7,ECanaMboxes.MBOX8.MDH.byte.BYTE6);

                  break;
                  case (0x194):

                              ModRegs.MailBox9Rxcount[3]++;
                              ModRegs.MDCellMaxTemps[8] = ComBine(ECanaMboxes.MBOX8.MDL.byte.BYTE1,ECanaMboxes.MBOX8.MDL.byte.BYTE0);
                              ModRegs.MDCellMinTemps[8] = ComBine(ECanaMboxes.MBOX8.MDL.byte.BYTE3,ECanaMboxes.MBOX8.MDL.byte.BYTE2);
                              ModRegs.MDCellAgvTemps[8] = ComBine(ECanaMboxes.MBOX8.MDH.byte.BYTE5,ECanaMboxes.MBOX8.MDH.byte.BYTE4);
                              ModRegs.MDCellDivTemps[8] = ComBine(ECanaMboxes.MBOX8.MDH.byte.BYTE7,ECanaMboxes.MBOX8.MDH.byte.BYTE6);

                  break;
                  case (0x195):
                              ModRegs.MailBox9Rxcount[4]++;
                              ModRegs.MDTotalVolt[8]     = ComBine(ECanaMboxes.MBOX8.MDL.byte.BYTE1,ECanaMboxes.MBOX8.MDL.byte.BYTE0);
                              ModRegs.MDMaxVoltPo[8]     = ECanaMboxes.MBOX8.MDL.byte.BYTE2;
                              ModRegs.MDMinVoltPo[8]     = ECanaMboxes.MBOX8.MDL.byte.BYTE3;
                              ModRegs.MDMaxTempsPo[8]    = ECanaMboxes.MBOX8.MDH.byte.BYTE4;
                              ModRegs.MDMinTempsPo[8]    = ECanaMboxes.MBOX8.MDH.byte.BYTE5;
                              ModRegs.MDstatusbit[8]     = ComBine(ECanaMboxes.MBOX8.MDH.byte.BYTE7,ECanaMboxes.MBOX8.MDH.byte.BYTE6);
                  break;
                #endif

                default:
                break;
            }
            ECanaRegs.CANRMP.bit.RMP8 = 1;
        }
        if(ECanaRegs.CANRMP.bit.RMP29==1)
        {
            CANARegs.MailBox1RxCount++;
            #if(PackNum==1)
            if(ECanaMboxes.MBOX29.MSGID.bit.STDMSGID==0x3C2)
            {
                SysRegs.CTRxCount=0;
                if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
                SysRegs.CurrentData.byte.CurrentH   = (ECanaMboxes.MBOX29.MDL.byte.BYTE0<<8)|(ECanaMboxes.MBOX29.MDL.byte.BYTE1);
                SysRegs.CurrentData.byte.CurrentL   = (ECanaMboxes.MBOX29.MDL.byte.BYTE2<<8)|(ECanaMboxes.MBOX29.MDL.byte.BYTE3);
            }
            #endif

            ECanaRegs.CANRMP.bit.RMP29 = 1;
            //ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            //ECanaShadow.CANRMP.all=0;
            //ECanaShadow.CANRMP.bit.RMP29 = 1;
            //ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;
        }
        if(ECanaRegs.CANRMP.bit.RMP30==1)
        {

            CANARegs.MailBox2RxCount++;
            #if(PackNum==1)
           // if(ECanaMboxes.MBOX30.MSGID.bit.STDMSGID==0x700)
           // {
                SysRegs.MasterRxCount=0;
               if(CANARegs.MailBox1RxCount>3000){CANARegs.MailBox1RxCount=0;}
               CANARegs.PMSCMDRegs.all = (ECanaMboxes.MBOX30.MDL.byte.BYTE1<<8)|(ECanaMboxes.MBOX30.MDL.byte.BYTE0);
              if(CANARegs.PMSCMDRegs.bit.PrtctReset01==1)
              {
                //   CANARegs.PMSCMDRegs.bit.RUNStatus01=0;
              }
               SysRegs.SysCanRxCount=0;
           // }
            #endif

            ECanaRegs.CANRMP.bit.RMP30 = 1;
            //ECanaShadow.CANRMP.all =ECanaRegs.CANRMP.all;
            //ECanaShadow.CANRMP.all=0;
            //ECanaShadow.CANRMP.bit.RMP30 = 1;
            //ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all ;

        }
    }

    ECanaRegs.CANGIF0.all = ECanaRegs.CANGIF0.all;
    ECanaRegs.CANGIF1.all = ECanaRegs.CANGIF1.all;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}//EOF
/*
interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
  // A_OVCHACurrent;
   EDIS;
}
*/
