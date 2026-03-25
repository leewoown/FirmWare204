
#include "parameter.h"
#include "SysVariable.h"
#include "DSP28x_Project.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

extern void SysTimerINIT(SystemReg *s);
extern void CANRegVarINIT(CANAReg *P);
extern void SysVarINIT(SystemReg *s);
extern void ModuleInit(ModulemReg *P);
extern void DigitalInput(SystemReg *sys);
extern void DigitalOutput(SystemReg *sys);
extern void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3);
extern void CANATX_29bit(Uint32 ID, Uint16 Length,Uint16 Data0, Uint16 Data1, Uint16 Data2, Uint16 Data3);
extern void SysCommErrHandle(SystemReg *P);
extern void SysCurrentHandle(SystemReg *s);

extern void CalSysAlarmtCheck(SystemReg *s);
extern void CalSysFaultCheck(SystemReg *s);
extern void CalSysProtectCheck(SystemReg *s);

extern int float32ToInt(float32 Vaule, Uint32 Num);
extern void TestCurrentlimt_StepUpdate_200ms(SystemReg *p);

//extern SystemReg       SysRegs;
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3)
{
    struct ECAN_REGS ECanaShadow;
    unsigned int CANWatchDog;
    unsigned int Data0Low, Data0High, Data1Low, Data1High;
    unsigned int Data2Low, Data2High, Data3Low, Data3High;

    CANWatchDog=0;

    Data0Low  = 0x00ff&Data0;
    Data0High = 0x00ff&(Data0>>8);
    Data1Low  = 0x00ff&Data1;
    Data1High = 0x00ff&(Data1>>8);
    Data2Low  = 0x00ff&Data2;
    Data2High = 0x00ff&(Data2>>8);
    Data3Low  = 0x00ff&Data3;
    Data3High = 0x00ff&(Data3>>8);



    EALLOW;
    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME31=0;
    ECanaRegs.CANME.bit.ME31= ECanaShadow.CANME.bit.ME31;

    ECanaMboxes.MBOX31.MSGID.all = 0UL;
    ECanaMboxes.MBOX31.MSGID.bit.IDE = 0U;                 // 표준 프레임
    ECanaMboxes.MBOX31.MSGID.bit.STDMSGID = (Uint16)(ID & 0x07FFU); // 11-bit

    ECanaMboxes.MBOX31.MSGCTRL.bit.RTR = 0U;
    ECanaMboxes.MBOX31.MSGCTRL.bit.DLC=Length;

    ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;
    ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;

    ECanaShadow.CANME.all = ECanaRegs.CANME.all;
    ECanaShadow.CANME.bit.ME31 = 1U;
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    EDIS;
    ECanaRegs.CANTRS.bit.TRS31 = 1U;

    while (ECanaRegs.CANTA.bit.TA31 == 0U)
    {
        if (++CANWatchDog > 2000U)
        {
            break; // 타임아웃
        }
    }
    // TA31 플래그 클리어
    if (ECanaRegs.CANTA.bit.TA31 == 1U)
    {
        ECanaRegs.CANTA.bit.TA31 = 1U;
    }

}
void CANATX_29bit(Uint32 ID, Uint16 Length,Uint16 Data0, Uint16 Data1, Uint16 Data2, Uint16 Data3)
{
    struct ECAN_REGS ECanaShadow;

    unsigned int CANWatchDog;
    unsigned int Data0Low, Data0High, Data1Low, Data1High;
    unsigned int Data2Low, Data2High, Data3Low, Data3High;

    CANWatchDog=0;

    Data0Low  = 0x00ff&Data0;
    Data0High = 0x00ff&(Data0>>8);
    Data1Low  = 0x00ff&Data1;
    Data1High = 0x00ff&(Data1>>8);
    Data2Low  = 0x00ff&Data2;
    Data2High = 0x00ff&(Data2>>8);
    Data3Low  = 0x00ff&Data3;
    Data3High = 0x00ff&(Data3>>8);

    EALLOW;

     // MBOX31 비활성 → MSGID 설정 → 활성
     ECanaShadow.CANME.all = ECanaRegs.CANME.all;
     ECanaShadow.CANME.bit.ME31 = 0U;
     ECanaRegs.CANME.all = ECanaShadow.CANME.all;

     ECanaMboxes.MBOX31.MSGID.all = 0UL;
     ECanaMboxes.MBOX31.MSGID.bit.IDE = 1U;   // 확장 프레임

     ECanaMboxes.MBOX31.MSGID.bit.STDMSGID   = (Uint16)((ID >> 18) & 0x7FFUL); // [28:18]
     ECanaMboxes.MBOX31.MSGID.bit.EXTMSGID_H = (Uint16)((ID >> 16) & 0x3UL);   // [17:16]
     ECanaMboxes.MBOX31.MSGID.bit.EXTMSGID_L = (Uint16)( ID        & 0xFFFFUL);// [15:0]

     // RTR=0(데이터 프레임)
     ECanaMboxes.MBOX31.MSGCTRL.bit.RTR = 0U;

     // 재활성
     ECanaShadow.CANME.all = ECanaRegs.CANME.all;
     ECanaShadow.CANME.bit.ME31 = 1U;
     ECanaRegs.CANME.all = ECanaShadow.CANME.all;

     EDIS;


    ECanaMboxes.MBOX31.MSGCTRL.bit.DLC=Length;

    ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;
    ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
    ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;
    ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
    ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;

    //CAN Tx Request
    ECanaShadow.CANTRS.all=0;
    ECanaShadow.CANTRS.bit.TRS31= 1;
    ECanaRegs.CANTRS.all = ECanaShadow.CANTRS.all;
    for (;;)
    {
        ECanaShadow.CANTA.all = ECanaRegs.CANTA.all;
        if (ECanaShadow.CANTA.bit.TA31 == 1U)
        {
            // 완료 → TA31 클리어
            ECanaShadow.CANTA.all      = 0UL;
            ECanaShadow.CANTA.bit.TA31 = 1U;
            ECanaRegs.CANTA.all        = ECanaShadow.CANTA.all;
            break;
        }
        if (++CANWatchDog > 2000U)
        {
            // 타임아웃: 일단 플래그 정리만 하고 탈출 (필요시 에러 카운트 추가)
            ECanaShadow.CANTA.all       = 0UL;
            ECanaShadow.CANTA.bit.TA31  = 1U;
            ECanaRegs.CANTA.all         = ECanaShadow.CANTA.all;
            break;
        }
    }

    //Tx Flag Clear
    //InitECan();
  //  ECanaShadow.CANTA.all = 0;
  //  ECanaShadow.CANTA.bit.TA31=1;                   // Clear TA5
  //  ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;

}
void SysTimerINIT(SystemReg *s)
{
    s->SysMachine=System_STATE_INIT;
    s->Maincount=0;
    s->MainIsr1=0;
    s->CANRXCOUNT=0;
    s->CANRXMailBox00Count=0;
    s->CANRXMailBox01Count=0;
    s->CANRXMailBox02Count=0;
    s->CANRXMailBox03Count=0;
    s->CANRXMailBox04Count=0;
    s->SysRegTimer5msecCount=0;
    s->SysRegTimer10msecCount=0;
    s->SysRegTimer50msecCount=0;
    s->SysRegTimer100msecCount=0;
    s->SysRegTimer300msecCount=0;
    s->SysRegTimer500msecCount=0;
    s->BalanceModeCount=0;
    s->BalanceTimeCount=0;

    s->LEDSycCount=0;
    s->LEDFaultCount=0;
    s->LEDCanCount=0;
    s->PWRLAMPCount=0;
    s->StartBATOUTOnCount=0;
    s->StartBATOUTOffCount=0;
}
void SysVarINIT(SystemReg *s)
{

    s->PMSysCMDResg.all=0;
    s->PackStateReg.all=0;
    s->PackAlarmReg.all=0;
    s->PackFaultReg.all=0;
    s->PackProtectReg.all=0;
    s->PackFaulBuftReg.all=0;
    s->DigitalInputReg.all=0;
    s->DigitalOutPutReg.all=0;
    s->CurrentData.all=0x80000000;
  //  s->PackCOMERR.all=0;

    s->Test=0;
    s->Maincount=0;
    s->MainIsr1=0;
    s->CANRXCOUNT=0;
    s->CANRXMailBox00Count=0;
    s->CANRXMailBox01Count=0;
    s->CANRXMailBox02Count=0;
    s->CANRXMailBox03Count=0;
    s->CANRXMailBox04Count=0;
    s->SysRegTimer5msecCount=0;
    s->SysRegTimer10msecCount=0;
    s->SysRegTimer50msecCount=0;
    s->SysRegTimer100msecCount=0;
    s->SysRegTimer300msecCount=0;
    s->SysRegTimer500msecCount=0;
    s->SysRegTimer1000msecCount=0;
    s->CellVoltsampling=0;
    s->SysCanRxCount=0;
    s->AlarmStatecount=0;
    s->PackFaultStatecount=0;
    s->Bat12VFaultStatecount=0;
    s->ProtectStatecount=0;
    s->RelayCheck=0;
    s->PackCellMaxVoltPos=0;
    s->PackCellMinVoltPos=0;
    s->PackVoltageF=0;
    s->PackCurrentF=0;
    s->PackCurrentAsbF=0;
    s->PackCellMaxVoltageF=0;
    s->PackCellMinVoltageF=0;
    s->PackCellDivVoltageF=0;
    s->PackCellAgvVoltageF=0;
    s->PackCellMaxTemperatureF=0;
    s->PackCellMinTemperatureF=0;
    s->PackCellDivTemperatureF=0;
    s->PackCellAgvTemperatureF=0;
    s->PackCellMaxTmepsPos=0;
    s->PackCellMinTmepsPos=0;

    s->PackCHAPWRContintyF=5;//5
    s->PackDisCHAPWRContintyF=5;
    s->PackCHAPWRPeakF=5;
    s->PackDisCHAPWRPeakF=5;
    s->PackSOCF=0;
    s->PackSOHF=0;
    s->PackAhF=0;
    s->PackISOResisF=0;

    s->PackModule1Regs.all=0;
    s->PackModule2Regs.all=0;
    s->PackModule3Regs.all=0;
    s->PackModule4Regs.all=0;
    s->PackModule5Regs.all=0;
    s->PackModule6Regs.all=0;
    s->PackModule7Regs.all=0;

    s->MD1CANRxCount=0;
    s->MD2CANRxCount=0;
    s->MD3CANRxCount=0;
    s->MD4CANRxCount=0;
    s->MD5CANRxCount=0;
    s->MD6CANRxCount=0;
    s->MD7CANRxCount=0;
    s->MD8CANRxCount=0;
    s->CTRxCount=0;
    s->MasterRxCount=0;

    s->PackInMDCANrxReg.all=0;
    s->ModuleWaterLeakReg.all=0;
    s->ModuleBatICReg.all=0;
    s->ModuleCTCANErrCReg.all=0;
    s->ModuleSubCANErrCReg.all=0;
    s->PackProtectReg.all=0;
    s->PackFaultReg.all=0;

}
void CANRegVarINIT(CANAReg *P)
{
    P->SWTypeVer=0;
    P->PackID=0;
    P->PackSate=0;
    P->PackProtetSate=0;
    P->PackSateInfo=0;
    /*
     *
     */
    P->CellNumStart=0;
    P-> NumberShift=0;;
    P->CellVotlageNumber=0;;
    P->CellVotlageMaxNumber=0;;
    P->CellVoltageNum=0;
    P->PMSCMDRegs.all=0;
    P->PackDigitalOutPutReg.all=0;
    P->CellNumTStart=0;
    P->NumberTShift=0;
    P->CellTemperatureNumber=0;
    P->CellTemperatureMaxNumber=0;
    P->CellTemperatureNum=0;
    /*
     *
     */
    P->PackConfing=0;
    P->PackStatus.all=0;
    P->PackConFig=0;
    P->PackSOC=0;
    P->PackSOH=0;
    P->PackAh=0;
    P->PackCT=0;
    P->PackPT=0;
    P->PackCHAPWRContinty=0;
    P->PackCHAPWRPeak=0;
    P->PackDisCHAPWRContinty=0;
    P->PackDisCHAPWRPeak=0;
    P->PackVoltageMax=0;
    P->PackVoltageMin=0;
    P->PackBalanVolt=4500;
    P->PackVoltageAgv=0;
    P->PackVoltageDiv=0;
    P->PackVoltageMaxNum=0;
    P->PackVoltageMinNum=0;
    P->PackPackVotageBuf=0;

    P->PackTemperaturelMAX=0;
    P->PackTemperaturelMIN=0;
    P->PackTemperatureAVG=0;
    P->PackTemperatureDiv=0;
    P->PackTemperatureMaxNUM=0;
    P->PackTemperatureMinNUM=0;


    P->MailBoxRxCount=0;
    P->MailBox0RxCount=0;
    P->MailBox1RxCount=0;
    P->MailBox2RxCount=0;
    P->MailBox3RxCount=0;

    P->SwVerProducttype=0;
    P->BatConfParallelSerial=0;
    memset(&P->MDTotalVolt[0],0,7);

}
void ModuleInit(ModulemReg *P)
{
    memset(&P->MDCellVoltQty[0],0,7);
    memset(&P->MDCellTempsQty[0],0,7);
    memset(&P->MDFirmwareVer[0],0,7);
    memset(&P->MDFirmwareRev[0],0,7);
    memset(&P->MDNorVolt[0],0,7);
    memset(&P->MDNorCapacity[0],0,7);
    memset(&P->PackMinVolteRec[0],0,7);

    memset(&P->MDCellMaxVolt[0],0,7);
    memset(&P->MDCellMinVolt[0],0,7);
    memset(&P->MDCellAgvVolt[0],0,7);
    memset(&P->MDCellDivVolt[0],0,7);

    memset(&P->MDCellMaxTemps[0],0,7);
    memset(&P->MDCellMinTemps[0],0,7);
    memset(&P->MDCellAgvTemps[0],0,7);
    memset(&P->MDCellDivTemps[0],0,7);

    memset(&P->MDCTComErr[0],0,7);
    memset(&P->MDSubComErr[0],0,7);
    memset(&P->MDBatICCOMErr[0],0,7);
    memset(&P->MDWaterLeakErr[0],0,7);

    memset(&P->MDTotalVolt[0],0,7);
    memset(&P->MDMaxVoltPo[0],0,7);
    memset(&P->MDMinVoltPo[0],0,7);
    memset(&P->MDMaxTempsPo[0],0,7);
    memset(&P->MDMinTempsPo[0],0,7);
    memset(&P->MDstatusbit[0],0,7);



    memset(&P->MailBox0Rxcount[0],0,7);
    memset(&P->MailBox1Rxcount[0],0,7);
    memset(&P->MailBox2Rxcount[0],0,7);
    memset(&P->MailBox3Rxcount[0],0,7);
    memset(&P->MailBox4Rxcount[0],0,7);
    memset(&P->MailBox5Rxcount[0],0,7);
    memset(&P->MailBox6Rxcount[0],0,7);
    memset(&P->MailBox7Rxcount[0],0,7);
    memset(&P->MailBox8Rxcount[0],0,7);
    memset(&P->MailBox9Rxcount[0],0,7);
  //  P->MDCTComErrStaute.all=0;
  //  P->MDSubComErrStaute.all=0;
  //  P->MDBatICCOMErrStaute.all=0;
  //  P->MDWaterLeakErrStaute.all=0;
}
void SysModuleStauteCheckHandle(SystemReg *P)
{
    /*
     * SysRegs.PackModule1Regs = ModRegs.MDstatusbit[0];
     * SysRegs.PackModule2Regs = ModRegs.MDstatusbit[1];
     * SysRegs.PackModule3Regs = ModRegs.MDstatusbit[2];
     * SysRegs.PackModule4Regs = ModRegs.MDstatusbit[3];
     * SysRegs.PackModule5Regs = ModRegs.MDstatusbit[4];
     * SysRegs.PackModule6Regs = ModRegs.MDstatusbit[5];
     * SysRegs.PackModule7Regs = ModRegs.MDstatusbit[6];
     */
    //Module 1
    if(P->PackModule1Regs.bit.INITOK==1)
    {
        P->ModuleStatusInitok.bit.MD01=1;
    }
    else
    {
        P->ModuleStatusInitok.bit.MD01=0;
    }
    //Module 2
    if(P->PackModule2Regs.bit.INITOK==1)
    {
        P->ModuleStatusInitok.bit.MD02=1;
    }
    else
    {
        P->ModuleStatusInitok.bit.MD02=0;
    }
    //Module 3
    if(P->PackModule3Regs.bit.INITOK==1)
    {
        P->ModuleStatusInitok.bit.MD03=1;
    }
    else
    {
        P->ModuleStatusInitok.bit.MD03=0;
    }
    //Module 4
    if(P->PackModule4Regs.bit.INITOK==1)
    {
        P->ModuleStatusInitok.bit.MD04=1;
    }
    else
    {
        P->ModuleStatusInitok.bit.MD04=0;
    }
    //Module 5
    if(P->PackModule5Regs.bit.INITOK==1)
    {
        P->ModuleStatusInitok.bit.MD05=1;
    }
    else
    {
        P->ModuleStatusInitok.bit.MD05=0;
    }
    //Module 6
    if(P->PackModule6Regs.bit.INITOK==1)
    {
        P->ModuleStatusInitok.bit.MD06=1;
    }
    else
    {
        P->ModuleStatusInitok.bit.MD06=0;
    }
    //Module 7
    if(P->PackModule7Regs.bit.INITOK==1)
    {
        P->ModuleStatusInitok.bit.MD07=1;
    }
    else
    {
        P->ModuleStatusInitok.bit.MD07=0;
    }
    /*
     * Module Fault Check
     */
    if(P->PackModule1Regs.bit.Fault==1)
    {
        P->ModuleStatusFault.bit.MD01=1;
    }
    else
    {
        P->ModuleStatusFault.bit.MD01=0;
    }
    if(P->PackModule2Regs.bit.Fault==1)
    {
        P->ModuleStatusFault.bit.MD02=1;
    }
    else
    {
        P->ModuleStatusFault.bit.MD02=0;
    }
    //Module 3
    if(P->PackModule3Regs.bit.Fault==1)
    {
        P->ModuleStatusFault.bit.MD03=1;
    }
    else
    {
        P->ModuleStatusFault.bit.MD03=0;
    }
    //Module 4
    if(P->PackModule4Regs.bit.Fault==1)
    {
        P->ModuleStatusFault.bit.MD04=1;
    }
    else
    {
        P->ModuleStatusFault.bit.MD04=0;
    }
    //Module 5
    if(P->PackModule5Regs.bit.Fault==1)
    {
        P->ModuleStatusFault.bit.MD05=1;
    }
    else
    {
        P->ModuleStatusFault.bit.MD05=0;
    }
    //Module 6
    if(P->PackModule6Regs.bit.Fault==1)
    {
        P->ModuleStatusFault.bit.MD06=1;
    }
    else
    {
        P->ModuleStatusFault.bit.MD06=0;
    }
    //Module 7
    if(P->PackModule7Regs.bit.Fault==1)
    {
        P->ModuleStatusFault.bit.MD07=1;
    }
    else
    {
        P->ModuleStatusFault.bit.MD07=0;
    }
    /*
     *  Module ModuleStatusWleagErr
     */
    if(P->PackModule1Regs.bit.Fault==1)
    {
        P->ModuleStatusWleagErr.bit.MD01=1;
    }
    else
    {
        P->ModuleStatusWleagErr.bit.MD01=0;
    }
    //Module 2
    if(P->PackModule2Regs.bit.Fault==1)
    {
        P->ModuleStatusWleagErr.bit.MD02=1;
    }
    else
    {
        P->ModuleStatusWleagErr.bit.MD02=0;
    }
    //Module 3
    if(P->PackModule3Regs.bit.Fault==1)
    {
        P->ModuleStatusWleagErr.bit.MD03=1;
    }
    else
    {
        P->ModuleStatusWleagErr.bit.MD03=0;
    }
    //Module 4
    if(P->PackModule4Regs.bit.Fault==1)
    {
        P->ModuleStatusWleagErr.bit.MD04=1;
    }
    else
    {
        P->ModuleStatusWleagErr.bit.MD04=0;
    }
    //Module 5
    if(P->PackModule5Regs.bit.Fault==1)
    {
        P->ModuleStatusWleagErr.bit.MD05=1;
    }
    else
    {
        P->ModuleStatusWleagErr.bit.MD05=0;
    }
    //Module 6
    if(P->PackModule6Regs.bit.Fault==1)
    {
        P->ModuleStatusWleagErr.bit.MD06=1;
    }
    else
    {
        P->ModuleStatusWleagErr.bit.MD06=0;
    }
    //Module 7
    if(P->PackModule7Regs.bit.Fault==1)
    {
        P->ModuleStatusWleagErr.bit.MD07=1;
    }
    else
    {
        P->ModuleStatusWleagErr.bit.MD07=0;
    }
}
void SysDIErrHandle(SystemReg *P)
{
    /*
     * SysRegs.PackModule1Regs = ModRegs.MDstatusbit[0];
     * SysRegs.PackModule2Regs = ModRegs.MDstatusbit[1];
     * SysRegs.PackModule3Regs = ModRegs.MDstatusbit[2];
     * SysRegs.PackModule4Regs = ModRegs.MDstatusbit[3];
     * SysRegs.PackModule5Regs = ModRegs.MDstatusbit[4];
     * SysRegs.PackModule6Regs = ModRegs.MDstatusbit[5];
     * SysRegs.PackModule7Regs = ModRegs.MDstatusbit[6];
     */

   // if(P->PackModule1Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD01=1;}
    if(P->PackModule1Regs.bit.WaterleakFault==1){P->ModuleWaterLeakReg.bit.MD01=1;}
    if(P->PackModule2Regs.bit.WaterleakFault==1){P->ModuleWaterLeakReg.bit.MD02=1;}
    if(P->PackModule3Regs.bit.WaterleakFault==1){P->ModuleWaterLeakReg.bit.MD03=1;}
    if(P->PackModule4Regs.bit.WaterleakFault==1){P->ModuleWaterLeakReg.bit.MD04=1;}
    if(P->PackModule5Regs.bit.WaterleakFault==1){P->ModuleWaterLeakReg.bit.MD05=1;}
    if(P->PackModule6Regs.bit.WaterleakFault==1){P->ModuleWaterLeakReg.bit.MD06=1;}
    if(P->PackModule7Regs.bit.WaterleakFault==1){P->ModuleWaterLeakReg.bit.MD07=1;}

    if(P->ModuleWaterLeakReg.all>0)             {P->PackProtectReg.bit.PackWaterleakErr=1;}
    if(P->DigitalInputReg.bit.EMGSWStauts==1)   {P->PackProtectReg.bit.PackEMSSWErr=1;}

    if(P->DigitalInputReg.bit.EMGSWStauts==1)
    {
        P->DigitalOutPutReg.bit.EMGSWLAMPOUT=1;
    }
    else
    {
        P->DigitalOutPutReg.bit.EMGSWLAMPOUT=0;
    }

}
void SysUnitBMSStaus(SystemReg *P)
{

}
void SysCommErrHandle(SystemReg *P)
{
    P->MD1CANRxCount++;
    P->MD2CANRxCount++;
    P->MD3CANRxCount++;
    P->MD4CANRxCount++;
    P->MD5CANRxCount++;
    P->MD6CANRxCount++;
    P->MD7CANRxCount++;
    P->CTRxCount++;
    P->MasterRxCount++;
    if(P->MasterRxCount>5000)
    {
        P->MasterRxCount =5100;
        P->PackProtectReg.bit.PackUnitCom_Err=1;
    }
    if(P->CTRxCount>5000)
    {
        P->CTRxCount=5100;
        P->PackProtectReg.bit.PackCTCom_Err=1;
    }
    if(P->PackInMDCANrxReg.all>0)
    {
        P->PackStateReg.bit.CANCOMERR=1;

    }
   // if(P->ModuleBatICReg.all>0)     {P->PackStateReg.bit.MDBATICErr=1;}
   // if(P->ModuleCTCANErrCReg.all>0) {P->PackStateReg.bit.MDCTCANErr=1;}
   // if(P->ModuleSubCANErrCReg.all>0){P->PackStateReg.bit.MDSUBCANErr=1;}

    if(P->MD1CANRxCount>1000)
    {
        P->MD1CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD01=1;
    }
    if(P->MD2CANRxCount>1000)
    {
        P->MD2CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD02=1;
    }
    if(P->MD3CANRxCount>1000)
    {
        P->MD3CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD03=1;
    }
    if(P->MD4CANRxCount>1000)
    {
        P->MD4CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD04=1;
    }
    if(P->MD5CANRxCount>1000)
    {
        P->MD5CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD05=1;
    }
    if(P->MD6CANRxCount>1000)
    {
        P->MD6CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD06=1;
    }
    if(P->MD7CANRxCount>1000)
    {
        P->MD7CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD07=1;
    }
    if(P->MD8CANRxCount>1000)
    {
        P->MD8CANRxCount=1100;
        P->PackInMDCANrxReg.bit.MD08=1;
    }

    if(P->PackModule1Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD01=1;}
    if(P->PackModule2Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD02=1;}
    if(P->PackModule3Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD03=1;}
    if(P->PackModule4Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD04=1;}
    if(P->PackModule5Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD05=1;}
    if(P->PackModule6Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD06=1;}
    if(P->PackModule7Regs.bit.BATIC1ErrFault==1){P->ModuleBatICReg.bit.MD07=1;}
    //
    if(P->PackModule1Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD01=1;}
    if(P->PackModule2Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD02=1;}
    if(P->PackModule3Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD03=1;}
    if(P->PackModule4Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD04=1;}
    if(P->PackModule5Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD05=1;}
    if(P->PackModule6Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD06=1;}
    if(P->PackModule7Regs.bit.CTCOMErrFault==1){P->ModuleCTCANErrCReg.bit.MD07=1;}

    if(P->PackModule1Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD01=1;}
    if(P->PackModule2Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD02=1;}
    if(P->PackModule3Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD03=1;}
    if(P->PackModule4Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD04=1;}
    if(P->PackModule5Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD05=1;}
    if(P->PackModule6Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD06=1;}
    if(P->PackModule7Regs.bit.MBCOMErrFault==1){P->ModuleSubCANErrCReg.bit.MD07=1;}

    P->PackStateReg.bit.CANCOMERR=  P->PackProtectReg.bit.PackVCUCom_Err | P->PackProtectReg.bit.PackCharCom_Err|
                                    P->PackProtectReg.bit.PackUnitCom_Err| P->PackProtectReg.bit.PackCTCom_Err  ;



}
void SysCurrentHandle(SystemReg *s)
{
    long  CurrentCT  = 0;
    float32 Currentbuf = 0;
    CurrentCT  = s->CurrentData.all;
    CurrentCT  =  CurrentCT - 0x80000000;

    Currentbuf      =  ((float32)CurrentCT)/1000;     // (mA to A) CAB500 resolution 1mA
    s->PackCurrentF  = C_CTDirection * Currentbuf;    // Decide Current sensor's direction

    if(s->PackCurrentF>=500.0)
    {
        s->PackCurrentF=500.0;
    }
    if(s->PackCurrentF<=-500.0)
    {
        s->PackCurrentF=-500.0;
    }
    if(s->PackCurrentF < 0)
    {
        s->PackCurrentAsbF =-1.0 * s->PackCurrentF;
    }
    else
    {
        s->PackCurrentAsbF =s->PackCurrentF;
    }

}
void CalSysAlarmtCheck(SystemReg *s)
{

}
unsigned int    CellVoltUnBalaneFaulCount=0;

void CalSysFaultCheck(SystemReg *s)
{

}

void CalSysProtectCheck(SystemReg *s)
{

}

int float32ToInt(float32 Vaule, Uint32 Num)
{
    Uint32 intVaule=0;
    intVaule = roundf(Vaule*10)/10;

    return (Uint32)intVaule;
}
void DigitalInput(SystemReg *sys)
{
    if((IDSW02==0)&&(IDSW01==0))
    {
        sys->DigitalInputReg.bit.IDSW=0;
    }
    if((IDSW02==0)&&(IDSW01==1))
    {
        sys->DigitalInputReg.bit.IDSW=1;
    }
    if((IDSW02==1)&&(IDSW01==0))
    {
        sys->DigitalInputReg.bit.IDSW=2;
    }
    if((IDSW01==1)&&(IDSW01==1))
    {
        sys->DigitalInputReg.bit.IDSW=3;
    }
    if(CANRX0INT==0)
    {
        sys->DigitalInputReg.bit.CANRX0=1;
    }
    else
    {
        sys->DigitalInputReg.bit.CANRX0=0;
    }
    if(CANRX1INT==0)
    {
        sys->DigitalInputReg.bit.CANRX1=1;
    }
    else
    {
        sys->DigitalInputReg.bit.CANRX1=0;
    }

    if(PRlyState==0)
    {
        sys->DigitalInputReg.bit.PAUX=1;
    }
    else
    {
        sys->DigitalInputReg.bit.PAUX=0;
    }
    if(NRlyState==0)
    {
        sys->DigitalInputReg.bit.NAUX=1;
    }
    else
    {
        sys->DigitalInputReg.bit.NAUX=0;
    }
    if(EMGSWDI==0)
    {
        sys->DigitalInputReg.bit.EMGSWStauts=1;
    }
    else
    {
        sys->DigitalInputReg.bit.EMGSWStauts=0;
    }

}
void DigitalOutput(SystemReg *sys)
{

    if(sys->DigitalOutPutReg.bit.NRlyOUT==1)
    {
        NRlyOn;
    }
    else
    {
        NRlyOff;
    }
    if(sys->DigitalOutPutReg.bit.ProRlyOUT==1)
    {
        PRORlyOn;
    }
    else
    {
        PRORlyOff;
    }
    if(sys->DigitalOutPutReg.bit.PRlyOUT==1)
    {
        PRlyOn;
    }
    else
    {
        PRlyOff;
    }
    if(sys->DigitalOutPutReg.bit.PWRLAMPOUT==1)
    {
        PWRLAMPOn;
    }
    else
    {
        sys->PWRLAMPCount++;
        if(sys->PWRLAMPCount>1200)
        {
            PWRLAMPTog;
            sys->PWRLAMPCount=0;
        }
    }
    if(sys->DigitalOutPutReg.bit.EMGSWLAMPOUT==1)
    {
        EMGSWLAMPOn;
    }
    else
    {
        EMGSWLAMPOff;
    }
    if(sys->DigitalOutPutReg.bit.LEDAlarmOUT==1)
    {
        sys->LEDFaultCount++;
        if(sys->LEDFaultCount>1200)
        {
            LEDFault_T;
            sys->LEDFaultCount=0;
        }
    }
    if(sys->DigitalOutPutReg.bit.LEDFaultOUT==1)
    {
        sys->LEDFaultCount++;

    }

    if(sys->DigitalOutPutReg.bit.LEDProtectOUT==1)
    {
        sys->LEDFaultCount++;
        if(sys->LEDFaultCount>200)
        {
            LEDFault_T;
            sys->LEDFaultCount=0;
        }
    }
    if((sys->DigitalOutPutReg.bit.LEDAlarmOUT==0)&&(sys->DigitalOutPutReg.bit.LEDFaultOUT==0)&&(sys->DigitalOutPutReg.bit.LEDProtectOUT==0))
    {
        LEDFault_H;
    }
    if(sys->DigitalOutPutReg.bit.LEDSysOUT==1)
    {
        sys->LEDSycCount++;
        if(sys->LEDSycCount>500)
        {
      //      LEDSysState_T;
            sys->LEDSycCount=0;
        }
    }
    else
    {
       // LEDSysState_H;
    }
    if(sys->DigitalOutPutReg.bit.LEDCAnOUT==1)
    {
  -
        sys->LEDCanCount++;
        if(sys->LEDCanCount>500)
        {
            LEDCANState_T;
            sys->LEDCanCount=0;
        }
    }
    else
    {
        LEDCANState_H;
    }
}


/*
 *
 */

void TimerinitHandle(TimerReg *timer)
{
  timer->state = TIMER_STATE_IDLE;
  timer->TimeCount = 0;
  timer->Start=0;
  timer->Stop=0;
  timer->OutState=0;
  timer->Reset=0;
  timer->TimerVaule=0;
}
void ProtectRelayTimerHandle(TimerReg *timer)
{
  switch (timer->state)
  {
    case TIMER_STATE_IDLE:
      // 타이머 시작
      if (timer->Start==1)
      {
        timer->state = TIMER_STATE_RUNNING;
      }
      if (timer->Reset==1)
      {
         timer->state = TIMER_STATE_CLEAR;
      }
      break;
    case TIMER_STATE_RUNNING:
      // 타이머 만료 확인
      if (timer->TimeCount >= timer->TimerVaule)
      {
        timer->state = TIMER_STATE_EXPIRED;
      }
      // 타이머 증가
      timer->TimeCount++;
      break;
    case TIMER_STATE_EXPIRED:
         // 타이머 만료 처리
         timer->OutState = 1;
         // 타이머 재시작
         if (timer->Reset==1)
         {
            timer->state = TIMER_STATE_CLEAR;
         }
    break;
    case TIMER_STATE_CLEAR:
         timer->state = TIMER_STATE_IDLE;
         timer->TimeCount = 0;
         timer->Start=0;
         timer->Stop=0;
         timer->OutState=0;
         timer->TimerVaule=0;
         timer->Reset=0;
    break;

  }
}
/*
 *
 */
void TestCurrentlimt_StepUpdate_200ms(SystemReg *p)
{
    /* SOC 1% 증가 */
    p->socPct_f += 1.0f;

    if (p->socPct_f > 100.0f)
    {
        p->socPct_f = 0.0f;

        /* Temp 1℃ 증가 */
        p->tempC_f += 1.0f;

        if (p->tempC_f > 60.0f)
        {
            p->tempC_f = -30.0f;
        }
    }
}

//extern MODReg MODRegs;
/*
extern void InitECan(void);
extern void SciaTxchar(char Txchar);

extern void SPI_Write(unsigned int WRData);
extern unsigned int SPI_READ(void);
extern void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3 );
 
extern void ModuleCellMIN(SlaveReg *S);
extern void ModuleCellMAX(SlaveReg *S);
//extern void BatteryTeperaureCal(BatteryReg *bat,int Count);




extern void TEST_LTC6804(void);
extern int LTC6804_Init(void);

extern void SlaveBMSSPIEnable_low(void);
extern void SlaveBMSSPIEnable_high(void);
extern void SlaveBms_WakeUp(void);
extern int SlaveBMSCellReadCommand(SlaveReg *s);
extern void SlaveBmsBalance(SlaveReg *s);


extern float32 AbsVaule(float32 input);
extern void CellVoltagetoFloat(SlaveReg *s, ModuleReg *m);

extern void SystemAlarm(PackReg *bat,SystemReg *sys);
extern void SystemRegsInit(SystemReg *s);
extern void systemParmRead(SystemReg *sys,PackReg *bat);
extern void sysDigitalInput(SystemReg *sys);
extern void sysDigitalOutput(SystemReg *sys);
extern void SysCurrentOffset(SystemReg *sys);
extern void SystemforceBalance(ModuleReg *m);


extern void DigitalInput(SystemReg *sys);
extern void DigitalOutput(SystemReg *sys);
extern void SlaveRegsInit(SlaveReg *s);
extern void ModuleRegsInit(ModuleReg *m);
extern void PackRegsInit(PackReg *b);
extern void SystemInit(SystemReg *s);
extern void CellTempCalFloat(SlaveReg *s, ModuleReg *m);
extern void CellVOLTFloatTOInt(ModuleReg *s, CANTXReg *t);
extern void CellTempFloatToInt(ModuleReg *s, CANTXReg *t);

void CellTempFloatToInt(ModuleReg *s, CANTXReg *t)
{
	t->BatteryTempCell[t->CellNumer]  =(int)(s->CellTempF[0]*10.0);
	t->BatteryTempCell[t->CellNumer+1]=(int)(s->CellTempF[1]*10.0);
	t->BatteryTempCell[t->CellNumer+2]=(int)(s->CellTempF[2]*10.0);
	t->BatteryTempCell[t->CellNumer+3]=(int)(s->CellTempF[3]*10.0);
}
void CellVOLTFloatTOInt(ModuleReg *s, CANTXReg *t)
{
	t->BatteryVoltageCell[t->CellNumer]=(unsigned int)(s->CellVoltageF[0]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+1]=(unsigned int)(s->CellVoltageF[1]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+2]=(unsigned int)(s->CellVoltageF[2]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+3]=(unsigned int)(s->CellVoltageF[3]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+4]=(unsigned int)(s->CellVoltageF[4]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+5]=(unsigned int)(s->CellVoltageF[5]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+6]=(unsigned int)(s->CellVoltageF[6]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+7]=(unsigned int)(s->CellVoltageF[7]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+8]=(unsigned int)(s->CellVoltageF[8]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+9]=(unsigned int)(s->CellVoltageF[9]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+10]=(unsigned int)(s->CellVoltageF[10]*1000.0);
	t->BatteryVoltageCell[t->CellNumer+11]=(unsigned int)(s->CellVoltageF[11]*1000.0);


}
void TEST_LTC6804(void)
{
	LTC680x_CS;
	SPI_Write(0X55);
	SPI_Write(0X55);
	LTC680X_DS;
}
void Alarm_LED_handle(SystemReg *sys)
{

}
void Emergency_LED_handle(SystemReg *sys)
{
 
}
void SlaveRegsInit(SlaveReg *s)
{
	
}
void ModuleRegsInit(ModuleReg *m)
{

}
void PackRegsInit(PackReg *b)
{
//	b->PackVoltage=0.0;
//	b->PackTemp=0.0;
//	b->PackSOC=0.0; 

}
void SystemRegsInit(SystemReg *s)
{
	LTC680X_DS;
	s->initCount=0;
	s->DigitalOutPutReg.all=0;
	s->DigitalInputReg.all=0;
	s->SystemStateARegs.all=0x0000;
	s->Timer50msec=0;
	s->Timer250msec=0;
	s->Timer500msec=0;
	s->Timer1000msec=0;
	s->Timer1500msec=0;
	s->Timer2000msec=0;
	s->Timer2500msec=0;
 
}
void SystemInit(SystemReg *s)
{
	
	if(s->initCount<500)
	{
		s->SystemStateARegs.bit.INITOK=0;
	}
	if(s->initCount>=500)
	{
		s->SystemStateARegs.bit.INITOK=1;
		s->initCount=501;
	}
}

void SystemAlarm(PackReg *bat,SystemReg *sys)
{

}
void SystemFault(PackReg *bat, SystemReg *sys)
{

}
void SysCurrentOffset(SystemReg *sys)
{
}
void SocBalance(ModuleReg *m) 
{
	
}
void AutoBalance(ModuleReg *m) 
{
	if(m->CellVoltageF[0]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell00=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell00=1;
	}
	if(m->CellVoltageF[1]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell01=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell01=1;
	}
	if(m->CellVoltageF[2]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell02=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell02=1;
	}
	if(m->CellVoltageF[3]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell03=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell03=1;
	}
	if(m->CellVoltageF[4]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell04=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell04=1;
	}
	if(m->CellVoltageF[5]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell05=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell05=1;
	}
	if(m->CellVoltageF[6]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell06=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell06=1;
	}
	if(m->CellVoltageF[7]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell07=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell07=1;
	}
	if(m->CellVoltageF[8]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell08=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell08=1;
	}
	if(m->CellVoltageF[9]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell09=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell09=1;
	}
	if(m->CellVoltageF[10]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell10=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell10=1;
	}
	if(m->CellVoltageF[11]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell11=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell11=1;
	}
}
void SystemforceBalance(ModuleReg *m)
{
	if(m->CellVoltageF[0]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell00=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell00=1;
	}
	
	if(m->CellVoltageF[1]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell01=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell01=1;
	}
	
	if(m->CellVoltageF[2]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell02=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell02=1;
	}
	
	if(m->CellVoltageF[3]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell03=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell03=1;
	}
	
	if(m->CellVoltageF[4]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell04=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell04=1;
	}
	
	if(m->CellVoltageF[5]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell05=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell05=1;
	}
	
	if(m->CellVoltageF[6]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell06=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell06=1;
	}
	
	if(m->CellVoltageF[7]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell07=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell07=1;
	}
	
	if(m->CellVoltageF[8]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell08=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell08=1;
	}
	
	if(m->CellVoltageF[9]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell09=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell09=1;
	}
	
	if(m->CellVoltageF[10]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell10=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell10=1;
	}
	
	if(m->CellVoltageF[11]<= m->ForceBalanceVoltage)
	{
		m->forceBalance.bit.B_Cell11=0;
	}
	else 
	{
		m->forceBalance.bit.B_Cell11=1;
	}

}

float32 AbsVaule(float32 input)
{
	float32 AbsCurrent=0.0;
	if(input>=0)
	{
		AbsCurrent=input;
	}
	else if(input<0)
	{
		AbsCurrent=-1.0*input;
	}	
	return AbsCurrent;
}

void CellTempCalFloat(SlaveReg *s, ModuleReg *m)
{
	float32 tmpF0=0.0;
	float32 tmpF1=0.0;
	float32 tmpF2=0.0;
	float32 tmpF3=0.0;

	float32 x02=0.0;
	float32 x0=0.0;
	float32 x12=0.0;
	float32 x1=0.0;
	float32 x22=0.0;
	float32 x2=0.0;
	float32 x32=0.0;
	float32 x3=0.0;

	tmpF0=(float)(s->CellTempBuffer[0]*0.0001);
	tmpF1=(float)(s->CellTempBuffer[1]*0.0001);
	tmpF2=(float)(s->CellTempBuffer[2]*0.0001);
	tmpF3=(float)(s->CellTempBuffer[3]*0.0001);

	x02= tmpF0*tmpF0;
	x0=tmpF0;
	m->CellTempF[0]= 3.5081*x02-41.81*x0+107.59;

	x12= tmpF1*tmpF1;
	x1=tmpF1;
	m->CellTempF[1]= 3.5081*x12-41.81*x1+107.59;


	x22= tmpF2*tmpF2;
	x2=tmpF2;
	m->CellTempF[2]= 3.5081*x22-41.81*x2+107.59;

	x32= tmpF3*tmpF3;
	x3=tmpF3;
	m->CellTempF[3]= 3.5081*x32-41.81*x3+107.59;

}
void CellVoltagetoFloat(SlaveReg *s, ModuleReg *m)
{
	int i;
	float32 sum=0.0;
	
	for (i = 0; i < ModuleCellNum; i++)
	{	
	//	m->CellVoltageF[i]=(float32)s->CellVoltage[i]/10000;
		s->CellVoltagebuffer[i]=s->CellVoltage[i]/10;
		m->CellVoltageF[i]=(float32)(s->CellVoltagebuffer[i]*0.001);

	//	m->CellVoltageF[i]=m->CellVoltageF[i]+0.0005;
		sum=sum+m->CellVoltageF[i];
	}
	m->ModuleVoltageCellMAX=(float32)(s->CellVoltageMax*0.001);
	m->ModuleVoltageCellMIN=(float32)(s->CellVoltageMin*0.001);
	m->ModuleVoltageCellAVG=m->ModuleVoltageF/12.0;
	m->ModuleVoltageCellCap=m->ModuleVoltageCellMAX-m->ModuleVoltageCellMIN;
	m->ModuleVoltageF=sum;
	
}
// 입력 인자
// S->CellNumber : 
void ModuleCellMAX(SlaveReg *S)
{
	int i;
	S->CellVoltageMax=S->CellVoltage[0];
	S->CellVoltageMaxNum=0;
	for(i=0; i<S->CellNumber;i++)
	{
		if(S->CellVoltageMax< S->CellVoltage[i])
		{
			S->CellVoltageMax = S->CellVoltage[i];	
			S->CellVoltageMaxNum=i;
		}
	}
	
}
// 입력 인자
// S->CellNumber : 
void ModuleCellMIN(SlaveReg *S)
{
	int i;
	S->CellVoltageMin=S->CellVoltage[0];
	S->CellVoltageMinNum=0;
	for(i=0; i<S->CellNumber;i++)
	{
		if(S->CellVoltageMin> S->CellVoltage[i])
		{
			S->CellVoltageMin = S->CellVoltage[i];	
			S->CellVoltageMinNum=i;
		}
	}
}
void BatteryTeperaureCal(PackReg *bat,int Count)
{
	//bat->BatteryTempCellF[i]=(-4.67*num3+35.688*num2)-102.18*num1+91.477;
}
void SlaveBMSSPIEnable_low(void)
{
	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
}

void SlaveBMSSPIEnable_high(void)
{
	GpioDataRegs.GPASET.bit.GPIO10 = 1;
}
void SlaveBms_WakeUp(void)
{
	// need to check
	SlaveBMSSPIEnable_low();
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SPI_Write(0);
	SlaveBMSSPIEnable_high();
}
int SlaveBMSCellReadCommand(SlaveReg *s)
{
	int i;
	SlaveBms_WakeUp();
	SlaveBMSSPIEnable_low();
//	s->WError= LTC6804_write(s->ID, s->Command, 0, 0);

	if((s->WError!=0)&&(s->len !=0))
	{
		for (i = 0; i < s->len; i++) 
		{
			s->ADCX[i]= SPI_READ();
		}
		s->pecr = (SPI_READ() << 8) & 0xff00;
		s->pecr |= (SPI_READ() & 0x00ff);
  //		s->pecg = pec15(s->ADCX[i], s->len);
		if (s->pecr ==s->pecg) 
		{
			s->RError=0;
			s->ErrorCount= 0;
		} 
		else 
		{
			s->RError=1;
			s->ErrorCount++;
		}
	}
	if(s->RError==1)
	{
		if(s->Command==LTC6804_CMD_RDCVA)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVA;
		}
		if(s->Command==LTC6804_CMD_RDCVB)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVB;
		}
		if(s->Command==LTC6804_CMD_RDCVC)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVC;
		}
		if(s->Command==LTC6804_CMD_RDCVD)
		{
			s->ErrorCode=LTC6804_ERROR_RDCVD;
		}
	}
	SlaveBMSSPIEnable_high();
	return s->ErrorCode;	
}

int CellBalanceSet(int pack_id, CellBalance_t cell)
{
	int ret;
	char addr = LTC6804.address[pack_id];
	char *tab = (char *)LTC6804_init_table;
	tab[4] = (cell.all & 0xff);
	tab[5] = (tab[5] & 0xf0) | ((cell.all >> 8) & 0x0f);
	ret = LTC6804_write_cmd(addr, LTC6804_CMD_WRCFG, tab, sizeof(LTC6804_init_table));
	return ret;
}


//-----------------------------------------------------------------------

void SciaTxchar(char Txchar)
{
	while(!(SciaTxReadyFlag));
	SciaRegs.SCITXBUF=Txchar;
}
void CANATX(unsigned int ID, unsigned char Length, unsigned int Data0, unsigned int Data1,unsigned int Data2,unsigned int Data3)
{
	unsigned int Data0Low, Data0High, Data1Low, Data1High;
	unsigned int Data2Low, Data2High, Data3Low, Data3High;
	struct ECAN_REGS ECanaShadow;


	Data0Low  = 0x00ff&Data0;
	Data0High = 0x00ff&(Data0>>8);

	Data1Low  = 0x00ff&Data1;
	Data1High = 0x00ff&(Data1>>8);

	Data2Low  = 0x00ff&Data2;
	Data2High = 0x00ff&(Data2>>8);

	Data3Low  = 0x00ff&Data3;
	Data3High = 0x00ff&(Data3>>8);


	// 현재 CAN-A MBOX31가 전송 중이면 리턴함
	if(ECanaRegs.CANTRS.bit.TRS31== 1) return;
	//현재 CAN-A MBOX31가 전송 상태 체크함.
  	if(ECanaShadow.CANTA.bit.TA31== 0) // 0 : Send Fail,  1 : Send OK
	{
		ECanaShadow.CANTA.all = 0;
      	ECanaShadow.CANTA.bit.TA31= 1;     	         	// Clear TA5
       	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
   	}


	// CAN-A 송수신 체크
//	if(COM_FAULT_MODE)	CanDataCnt++;			// reset at Rx Routine
//	if(CanDataCnt > COMER_set)	COMER_FS=1;
//	Can_TX_ERR_CNT = ECanaRegs.CANTEC.bit.TEC;
//	Can_RX_ERR_CNT = ECanaRegs.CANREC.bit.REC;


	// 전송하는 데이터 ID 설정함.
	ECanaRegs.CANME.bit.ME31= 0;
	ECanaMboxes.MBOX31.MSGID.bit.STDMSGID=ID;
	ECanaRegs.CANME.bit.ME31= 1;
	// 전송하는 데이터 길이?8 설정함.


	ECanaMboxes.MBOX31.MSGCTRL.bit.DLC=Length; 	// 전송하는 데이터 길이
	// DATA 전송
	ECanaMboxes.MBOX31.MDL.byte.BYTE0=Data0Low;
	ECanaMboxes.MBOX31.MDL.byte.BYTE1=Data0High;

	ECanaMboxes.MBOX31.MDL.byte.BYTE2=Data1Low;
	ECanaMboxes.MBOX31.MDL.byte.BYTE3=Data1High;

	ECanaMboxes.MBOX31.MDH.byte.BYTE4=Data2Low;
	ECanaMboxes.MBOX31.MDH.byte.BYTE5=Data2High;

	ECanaMboxes.MBOX31.MDH.byte.BYTE6=Data3Low;
	ECanaMboxes.MBOX31.MDH.byte.BYTE7=Data3High;

	// CAN-A MBOX31 전송 시작함.
	ECanaRegs.CANTRS.bit.TRS31= 1;
	// CAN-A MBOX31전송 완료 체크함.
	while(!ECanaShadow.CANTA.bit.TA31);
	ECanaShadow.CANTA.all = 0;
   	ECanaShadow.CANTA.bit.TA31=1;     	         	// Clear TA5
   	ECanaRegs.CANTA.all = ECanaShadow.CANTA.all;
	ECanaMboxes.MBOX31.MSGID.all	= 0;

	// From Here 해당 MailBox를 CAN Enable 시킴  이것을 왜 다시 해 주어야 하�?
	EALLOW;     // EALLOW enables access to protected bits

	ECanaShadow.CANME.all = ECanaRegs.CANME.all;

	ECanaShadow.CANME.bit.ME0= 1;
	ECanaShadow.CANME.bit.ME1= 1;
	ECanaShadow.CANME.bit.ME2= 1;
	ECanaShadow.CANME.bit.ME3= 1;
	ECanaShadow.CANME.bit.ME4= 1;
	ECanaShadow.CANME.bit.ME5= 1;
	ECanaShadow.CANME.bit.ME6= 1;
	ECanaShadow.CANME.bit.ME7= 1;
	ECanaShadow.CANME.bit.ME8= 1;
	ECanaShadow.CANME.bit.ME9= 1;
	ECanaShadow.CANME.bit.ME10= 1;
	ECanaShadow.CANME.bit.ME11= 1;
	ECanaShadow.CANME.bit.ME12= 1;
	ECanaShadow.CANME.bit.ME13= 1;
	ECanaShadow.CANME.bit.ME14= 1;
	ECanaShadow.CANME.bit.ME15= 1;
	ECanaShadow.CANME.bit.ME16= 1;
	ECanaShadow.CANME.bit.ME17= 1;
	ECanaShadow.CANME.bit.ME18= 1;
	ECanaShadow.CANME.bit.ME19= 1;
	ECanaShadow.CANME.bit.ME20= 1;
	ECanaShadow.CANME.bit.ME21= 1;
	ECanaShadow.CANME.bit.ME22= 1;
	ECanaShadow.CANME.bit.ME23= 1;
	ECanaShadow.CANME.bit.ME24= 1;
	ECanaShadow.CANME.bit.ME25= 1;
	ECanaShadow.CANME.bit.ME26= 1;
	ECanaShadow.CANME.bit.ME27= 1;
	ECanaShadow.CANME.bit.ME28= 1;
	ECanaShadow.CANME.bit.ME29= 1;
	ECanaShadow.CANME.bit.ME30= 1;
	ECanaShadow.CANME.bit.ME31= 1;
	ECanaRegs.CANME.all = ECanaShadow.CANME.all;
	InitECan();

	EDIS;

	// To Here 해당 MailBox를 CAN Enable 시킴  이것을 왜 다시 해 주어야 하지?

}//EOF

//----------------------------------------------------------------------------------디지털 INPUT/OUTPUT 관련 함수


void DigitalOutput(SystemReg *sys)
{
	if(sys->DigitalOutPutReg.bit.Relay==1)
  	{
  		Relay_ON; 
  	}
	else 
	{
		Relay_OFF;
	}
}

void DataConversion(SystemReg *sys)
{

}
*/


