#include "DSP28x_Project.h"
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "BATCalc.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#if ShipPack_168S1P
extern void BatCalcRegsInit(BatCalcReg *P);
extern void BatCalcVoltHandle(BatCalcReg *P);
extern void BatCalcTempsHandle(BatCalcReg *P);
extern float RandVaule(float inputVaule);
float RandVaule(float inputVaule)
{
    static Uint32 seed = 1234567UL;
    float randNorm;
    float randOffset;

    /* LCG Random Generator */
    seed = (Uint32)(1664525UL * seed + 1013904223UL);

    /* 0 ~ 1 */
    randNorm = (float)(seed & 0xFFFFU) / 65535.0f;

    /* -0.4 ~ +0.5 */
    randOffset = -0.4f + (randNorm * 0.9f);

    /* 입력 전류 + 랜덤 */
    return (inputVaule + randOffset);
}

void BatCalcRegsInit(BatCalcReg *P)
{
    memset(&P->MDCellMaxVolt[0],0,7);
    memset(&P->MDCellMinVolt[0],0,7);
    memset(&P->MDCellAgvVolt[0],0,7);
    memset(&P->MDCellDivVolt[0],0,7);
    memset(&P->MDCellMaxTemps[0],0.0,7);
    memset(&P->MDCellMinTemps[0],0.0,7);
    memset(&P->MDCellAgvTemps[0],0.0,7);
    memset(&P->MDCellDivTemps[0],0.0,7);
    memset(&P->MDCellMaxVoltF[0],0.0,7);
    memset(&P->MDCellMinVoltF[0],0.0,7);
    memset(&P->MDCellAgvVoltF[0],0.0,7);
    memset(&P->MDCellDivVoltF[0],0.0,7);
    memset(&P->MDCellMaxTempsF[0],0.0,7);
    memset(&P->MDCellMaxTempsF[0],0.0,7);
    memset(&P->MDCellMaxTempsF[0],0.0,7);
    memset(&P->MDCellMaxTempsF[0],0.0,7);
    memset(&P->MDTotalVolt[0],0,7);
    memset(&P->MDTotalVoltF[0],0.0,7);
    memset(&P->MDMaxVoltPo[0],0.0,7);
    memset(&P->MDMinVoltPo[0],0.0,7);
    memset(&P->MDMaxTempsPo[0],0.0,7);
    memset(&P->MDMinTempsPo[0],0.0,7);
    /*
     *
     */
    P->PackPTADC=0;
    P->PackPTCAN=0;
    P->PackPTADCF=0;
    P->PackPTCANF=0;
    /*
     *
     */
    P->PackCTADC=0;
    P->PackCTCAN=0;
    P->PackCTAbs=0;
    P->PackCTAgv=0;
    P->PackCTADCF=0.0;
    P->PackCTCANF=0.0;
    P->PackCTAbsF=0.0;
    P->PackCTAgvF=0.0;
    /*
     *
     */
;
    P->PackCellMaxVoltF=0.0;
    P->PackCellMinVoltF=0.0;
    P->PackCellDivVoltF=0.0;
    P->PackCellAgvVoltF=0.0;

    P->PackCellMaxTempsF=0.0;
    P->PackCellMinTempsF=0.0;
    P->PackCellAgvTempsF=0.0;
    P->PackCellDivTempsF=0.0;


    P->PackCellMaxVoltPos=0;
    P->PackCellMinVoltPos=0;
    P->PackCellMaxTempsPos=0;
    P->PackCellMinTempsPos=0;
}
void BatCalcVoltHandle(BatCalcReg *P)
{
    Uint16  Count;
    Uint16  const MoudleEa =9;
    Uint16  const PackCellEa =22*9;
    /*
     * 정수를 소수점 변환하는 루틴
     */
    for(Count=0; Count<MoudleEa; Count++)
    {
        P->MDCellMaxVoltF[Count] =(float32) P->MDCellMaxVolt[Count]  *0.001f;
        P->MDCellMinVoltF[Count] =(float32) P->MDCellMinVolt[Count]  *0.001f;
        P->MDCellAgvVoltF[Count] =(float32) P->MDCellAgvVolt[Count]  *0.001f;
        P->MDCellDivVoltF[Count] =(float32) P->MDCellDivVoltF[Count] *0.001f;
        P->MDTotalVoltF[Count]   =(float32) P->MDTotalVolt[Count]    *0.01f;
    }
    float32 PackVoltageBufF=0;
    for(Count=0; Count<MoudleEa; Count++)
    {
        PackVoltageBufF = PackVoltageBufF+P->MDTotalVoltF[Count];
    }
    /*
     *
     */
    float32 CellMaxVoltF=-1.0;
    float32 CellMinVoltF= 5.0;
    Uint16 MDCellMaxVoltPos =0;
    Uint16 MDCellMinVoltPos =0;

    for(Count=0; Count<MoudleEa; Count++)
    {
        const float32 Vmax = P->MDCellMaxVoltF[Count];
        const float32 Vmin = P->MDCellMinVoltF[Count];
        if(Vmax > CellMaxVoltF )
        {
            CellMaxVoltF=Vmax;
            MDCellMaxVoltPos = Count+1;
        }
        if(Vmin < CellMinVoltF)
        {
            CellMinVoltF=Vmin;
            MDCellMinVoltPos = Count+1;
        }

    }
    P->PackPTCANF        = PackVoltageBufF;
    P->PackCellMaxVoltF  = CellMaxVoltF;
    P->PackCellMinVoltF  = CellMinVoltF;
    P->PackCellAgvVoltF  = P->PackPTCANF/(float32)PackCellEa;
    P->PackCellDivVoltF  = CellMaxVoltF-CellMinVoltF;
    P->PackCellMaxVoltPos =  (MDCellMaxVoltPos*22)+P->MDMaxVoltPo[MDCellMinVoltPos-1];
    P->PackCellMinVoltPos =  (MDCellMinVoltPos*22)+P->MDMinVoltPo[MDCellMinVoltPos-1];

}
void BatCalcTempsHandle(BatCalcReg *P)
{
    Uint16  Count=0;
    Uint16  BreakCountA=0;
    Uint16  BreakCountB=0;

    float32 CellMaxTempsF=0;
    float32 CellMaxTempsadF=0;
    float32 CellMinTempsF=0;
    float32 CellMinTempsadF=0;
     Uint16 MDCellMaxTempsPos =0;
     Uint16 MDCellMinTempsPos =0;


    /*
     * 정수를 소수점 변환하는 루틴
     */
    for(Count=0; Count<=ModuleEA; Count++)
    {
        BreakCountA++;
        P->MDCellMaxTempsF[Count]=(float32) P->MDCellMaxTemps[Count]*0.1;
        P->MDCellMinTempsF[Count]=(float32) P->MDCellMinTemps[Count]*0.1;
        if(BreakCountA>10) {break;}
    }

    CellMaxTempsF  = P->MDCellMaxTempsF[0];
    CellMinTempsF  = P->MDCellMinTempsF[0];

    for(Count=0; Count<=ModuleEA; Count++)
    {
        BreakCountB++;
        if(CellMaxTempsF <=P->MDCellMaxTempsF[Count])
        {
            CellMaxTempsF=P->MDCellMaxTempsF[Count];
            MDCellMaxTempsPos = Count+1;
        }
        if(CellMinTempsF >=P->MDCellMinTempsF[Count])
        {
            CellMinTempsF=P->MDCellMinTempsF[Count];
            MDCellMinTempsPos = Count+1;
        }
        if(BreakCountB>10) {break;}
    }
    /*
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 20, 25)){CellMaxTempsadF=CellMaxTempsF-4.2f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 25, 30)){CellMaxTempsadF=CellMaxTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 30, 35)){CellMaxTempsadF=CellMaxTempsF-3.8f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 35, 40)){CellMaxTempsadF=CellMaxTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 40, 45)){CellMaxTempsadF=CellMaxTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 45, 50)){CellMaxTempsadF=CellMaxTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 50, 55)){CellMaxTempsadF=CellMaxTempsF-4.8f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 55, 60)){CellMaxTempsadF=CellMaxTempsF-6.0f;}
    if(IS_ABOVE_AND_UNDER(CellMaxTempsF, 60, 65)){CellMaxTempsadF=CellMaxTempsF-6.0f;}

    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 20, 25)){CellMinTempsadF=CellMinTempsF-4.2f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 25, 30)){CellMinTempsadF=CellMinTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 30, 35)){CellMinTempsadF=CellMinTempsF-3.8f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 35, 40)){CellMinTempsadF=CellMinTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 40, 45)){CellMinTempsadF=CellMinTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 45, 50)){CellMinTempsadF=CellMinTempsF-4.7f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 50, 55)){CellMinTempsadF=CellMinTempsF-4.8f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 55, 60)){CellMinTempsadF=CellMinTempsF-6.0f;}
    if(IS_ABOVE_AND_UNDER(CellMinTempsF, 60, 65)){CellMinTempsadF=CellMinTempsF-6.0f;}
    */
    P->PackCellMaxTempsF  = CellMaxTempsadF;
    P->PackCellMinTempsF  = CellMinTempsadF;
    P->PackCellAgvTempsF  = (CellMaxTempsadF+CellMinTempsadF)*0.5;
    P->PackCellDivTempsF  = CellMaxTempsadF-CellMinTempsadF;

    P->PackCellMaxTempsPos =  (MDCellMaxTempsPos*24)+P->MDMaxTempsPo[MDCellMaxTempsPos];
    P->PackCellMinTempsPos =  (MDCellMinTempsPos*24)+P->MDMinTempsPo[MDCellMinTempsPos];
}
#endif


