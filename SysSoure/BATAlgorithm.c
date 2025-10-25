#include "DSP28x_Project.h"
#include "BATAlgorithm.h"
#include "stdio.h"
#include "math.h"
#include "string.h"

#if FarasisP52Ah
//extern void CalFarasis52AhRegsInit(SocReg *P);
//extern void CalFarasis52AhSocInit(SocReg *P);
//extern void CalFarasis52AhSocHandle(SocReg *P);
#define C_Farasis52Ah_SOCX2     -138.71
#define C_Farasis52Ah_SOCX1     1195.70
#define C_Farasis52Ah_SOCX0    -2472.30
#define C_FarasisP52AhNorm      0.0238//1/42Ah
#endif
#if Frey60Ah
extern void CalFrey60AhRegsInit(SocReg *P);
extern void CalFrey60AhSocInit(SocReg *P);
extern void CalFrey60AhSocHandle(SocReg *P);

#define C_Frey60Ah_SOCX1A    62.112
#define C_Frey60Ah_SOCX0A   -189.44
#define C_Frey60Ah_SOCX2B    2566.7
#define C_Frey60Ah_SOCX1B    -16384
#define C_Frey60Ah_SOCX0B     26156
#define C_Frey60Ah_SOCX1C    1313.1
#define C_Frey60Ah_SOCX0C   -4276.6
#define C_Frey60Ah_SOCX1D    939.26
#define C_Frey60Ah_SOCX0D   -3043.6
#define C_Frey60AhNorm      0.0208//1/48Ah
#endif

#if Kokam100Ah
#define C_SOCX2         -198.04
#define C_SOCX1         1646.6
#define C_SOCX0         -3324.3
#define C_NegAh         -90.0
#define C_PogAh          90.0
#define AhNorm          0.0125//1/42Ah
extern void CalKokam100AhRegsInit(SocReg *P);
extern void CalKokam100AhSocInit(SocReg *P);
extern void Calkokam100AhSocHandle(SocReg *P);
#endif

#if Kokam100Ah

void CalKokam100AhRegsInit(SocReg *P)
{
    P->SysSOCdtF=0.0;
    P->SysSoCCTF=0.0;
    P->SysAhNewF=0.0;
    P->SysAhOldF=0.0;
    P->SysAhF=0.0;
    P->SysSOCBufF1=0.0;
    P->SysSOCBufF2=0.0;
    P->SysSOCF=5.0;
    P->AVGXF=0.0;
    P->SOCX4InF=0.0;
    P->SOCX3InF=0.0;
    P->SOCX2InF=0.0;
    P->SOCX1InF=0.0;
    P->SOCX4OutF=0.0;
    P->SOCX3OutF=0.0;
    P->SOCX2OutF=0.0;
    P->SOCX1OutF=0.0;
    P->SOCbufF=0.0;
    P->SysSocInitF=0.0;
    P->CellAgvVoltageF=0.0;
    P->SoCStateRegs.all=0;
    P->CTCount=0;
    P->SysTime=0;
    P->SysSoCCTAbsF=0;
    P->state=SOC_STATE_IDLE;
}
void CalKokam100AhSocInit(SocReg *P)
{
     P->AVGXF         =   P->CellAgvVoltageF;
     P->SOCX2InF      =   P->AVGXF  * P->AVGXF;
     P->SOCX1InF      =   P->AVGXF;
     P->SOCX2OutF     =   C_SOCX2 * P->SOCX2InF;
     P->SOCX1OutF     =   C_SOCX1 * P->SOCX1InF;
     P->SOCbufF       =   P->SOCX2OutF + P->SOCX1OutF + C_SOCX0;
     /*
      *
      */
     if(P->SOCbufF <=0.0)
     {
         P->SOCbufF = 0.0;
     }
     else if(P->SOCbufF > 98.0)
     {
         P->SOCbufF = 100.0;
     }
     P->SysSocInitF = P->SOCbufF;

}

void Calkokam100AhSocHandle(SocReg *P)
{
    P->SysTime++;
    if(P->SysTime>=C_SocSamPleCount)
    {
        if(P->SysSoCCTAbsF>=C_SocInitCTVaule)
        {
            P->SoCStateRegs.bit.CalMeth=1;
            P->CTCount=0;
        }
        else
        {
            P->CTCount++;
            if(P->CTCount>6000)
            {
                P->CTCount=6001;
                P->SoCStateRegs.bit.CalMeth=0;
            }
        }
        switch (P->state)
        {

            case SOC_STATE_RUNNING:
                 if(P->SoCStateRegs.bit.CalMeth==0)
                 {
                     P->AVGXF         =   P->CellAgvVoltageF;
                     P->SOCX2InF      =   P->AVGXF  * P->AVGXF;
                     P->SOCX1InF      =   P->AVGXF;
                     P->SOCX2OutF     =   C_SOCX2 * P->SOCX2InF;
                     P->SOCX1OutF     =   C_SOCX1 * P->SOCX1InF;
                     P->SOCbufF       =   P->SOCX2OutF + P->SOCX1OutF + C_SOCX0;
                     P->SOCbufF       =   P->SOCbufF+3.0;
                     if(P->SOCbufF <=0.0)
                     {
                         P->SOCbufF = 0.0;
                     }
                     else if(P->SOCbufF > 98.0)
                     {
                         P->SOCbufF = 100.0;
                     }
                      if(P->SOCbufF <=0.0)
                      {
                          P->SOCbufF = 0.0;
                      }
                      else if(P->SOCbufF > 98.0)
                      {
                          P->SOCbufF = 100.0;
                      }
                      P->SysSocInitF = P->SOCbufF;
                 }
                 if(P->SoCStateRegs.bit.CalMeth==1)
                 {
                     P->SysSOCdtF = C_CTSampleTime*C_SocCumulativeTime; // CumulativeTime(1/3600) -> 누적시간
                     P->SysAhNewF = P->SysSoCCTF * P->SysSOCdtF;
                     P->SysAhF    = P->SysAhNewF + P->SysAhOldF;
                     P->SysAhOldF = P->SysAhF;
                     if(P->SysAhF <= C_NegAh)
                     {
                        P->SysAhF =C_NegAh;
                     }
                     if(P->SysAhF> C_PogAh)
                     {
                         P->SysAhF= C_PogAh;
                     }
                     /*
                     * SOC 변환
                     */
                     P->SysSOCBufF1 = P->SysAhF *AhNorm;//0.0125 ;// 1/80 --> 0.0125--> 일반화
                     P->SysSOCBufF2 = P->SysSOCBufF1*100.0; //--> 단위 변환 %
                     P->SysSOCF     = P->SysSocInitF+P->SysSOCBufF2;
                 }
                 P->state = SOC_STATE_Save;

            break;
            case SOC_STATE_Save:

                P->state = SOC_STATE_RUNNING;

            break;
            case SOC_STATE_CLEAR:

            break;
        }
        P->SysTime=0;
    }
}
#endif

#if Kokam60Ah
void CalKokam60AhRegs(void)
{

}
void CalKokam60AhSocInit(void)
{

}

void Calkokam60AhSocHandle(void)
{

}

#endif



#if FarasisP52Ah
void CalFarasis52AhRegsInit(SocReg *P)
{
    P->SysSOCdtF=0.0;
    P->SysSoCCTF=0.0;
    P->SysAhNewF=0.0;
    P->SysAhOldF=0.0;
    P->SysAhF=0.0;
    P->SysSOCBufF1=0.0;
    P->SysSOCBufF2=0.0;
    P->SysSOCF=5.0;
    P->AVGXF=0.0;
    P->SOCX4InF=0.0;
    P->SOCX3InF=0.0;
    P->SOCX2InF=0.0;
    P->SOCX1InF=0.0;
    P->SOCX4OutF=0.0;
    P->SOCX3OutF=0.0;
    P->SOCX2OutF=0.0;
    P->SOCX1OutF=0.0;
    P->SOCbufF=0.0;
    P->SysSocInitF=0.0;
    P->CellAgvVoltageF=0.0;
    P->SoCStateRegs.all=0;
    P->CTCount=0;
    P->SysTime=0;
    P->SysSoCCTAbsF=0;
    P->state=SOC_STATE_IDLE;
}
void CalFarasis52AhSocInit(SocReg *P)
{

    // 60Ah
     P->AVGXF         =   P->CellAgvVoltageF;
     P->SOCX2InF      =   P->AVGXF  * P->AVGXF;
     P->SOCX1InF      =   P->AVGXF;
     P->SOCX2OutF     =   C_Farasis52Ah_SOCX2 * P->SOCX2InF;
     P->SOCX1OutF     =   C_Farasis52Ah_SOCX1 * P->SOCX1InF;
     P->SOCbufF       =   P->SOCX2OutF + P->SOCX1OutF + C_Farasis52Ah_SOCX0;
     /*
      *  보관법 계산식 필요함
      */
     if((P->SOCbufF >= 0.0)&&(P->SOCbufF < 20.0))
     {
         P->SOCbufF  =   P->SOCbufF-2;
     }
     if((P->SOCbufF >= 20)&&(P->SOCbufF < 40.0))
     {
         P->SOCbufF  =   P->SOCbufF+3;
     }
     if((P->SOCbufF >= 40)&&(P->SOCbufF < 80.0))
     {
         P->SOCbufF  =   P->SOCbufF+3.0;
     }
     if(P->SOCbufF >= 80.0)
     {
         P->SOCbufF  =   P->SOCbufF-3;
     }
     /*
      *
      */
     if(P->SOCbufF <=0.0)
     {
         P->SOCbufF = 0.0;
     }
     else if(P->SOCbufF > 98.0)
     {
         P->SOCbufF = 100.0;
     }
     P->SysSocInitF = P->SOCbufF;
}

//CalSocKokamInit(&KokamSocRegs);
//KokamSocRegs.state = SOC_STATE_RUNNING;
void CalFarasis52AhSocHandle(SocReg *P)
{
    P->SysTime++;
    if(P->SysTime>=C_SocSamPleCount)
    {
        if(P->SysSoCCTAbsF>=C_SocInitCTVaule)
        {
            P->SoCStateRegs.bit.CalMeth=1;
            P->CTCount=0;
        }
        else
        {
            P->CTCount++;
            if(P->CTCount>6000)
            {
                P->CTCount=6001;
                P->SoCStateRegs.bit.CalMeth=0;
            }
        }
        switch (P->state)
        {

            case SOC_STATE_RUNNING:
                 if(P->SoCStateRegs.bit.CalMeth==0)
                 {
                     /*
                      *
                      */
                     //52Ah

                     P->AVGXF         =   P->CellAgvVoltageF;
                     P->SOCX2InF      =   P->AVGXF  * P->AVGXF;
                     P->SOCX1InF      =   P->AVGXF;
                     P->SOCX2OutF     =   C_Farasis52Ah_SOCX2 * P->SOCX2InF;
                     P->SOCX1OutF     =   C_Farasis52Ah_SOCX1 * P->SOCX1InF;
                     P->SOCbufF       =   P->SOCX2OutF + P->SOCX1OutF + C_Farasis52Ah_SOCX0;
                     P->SOCbufF       =   P->SOCbufF+3.0;
                     /*
                      *  보관법 계산식 필요함
                      */
                     if((P->SOCbufF >= 0.0)&&(P->SOCbufF < 20.0))
                     {
                         P->SOCbufF  =   P->SOCbufF-2;
                     }
                     if((P->SOCbufF >= 20)&&(P->SOCbufF < 40.0))
                     {
                         P->SOCbufF  =   P->SOCbufF+3;
                     }
                     if((P->SOCbufF >= 40)&&(P->SOCbufF < 80.0))
                     {
                         P->SOCbufF  =   P->SOCbufF+3.0;
                     }
                     if(P->SOCbufF >= 80.0)
                     {
                         P->SOCbufF  =   P->SOCbufF-3;
                     }
                     /*
                      *
                      */
                     if(P->SOCbufF <=0.0)
                     {
                         P->SOCbufF = 0.0;
                     }
                     else if(P->SOCbufF > 98.0)
                     {
                         P->SOCbufF = 100.0;
                     }
                      if(P->SOCbufF <=0.0)
                      {
                          P->SOCbufF = 0.0;
                      }
                      else if(P->SOCbufF > 98.0)
                      {
                          P->SOCbufF = 100.0;
                      }
                      P->SysSocInitF = P->SOCbufF;
                 }
                 if(P->SoCStateRegs.bit.CalMeth==1)
                 {
                     /*
                      *
                      */
                     P->SysSOCdtF = C_CTSampleTime*C_SocCumulativeTime; // CumulativeTime(1/3600) -> 누적시간
                     P->SysAhNewF = P->SysSoCCTF * P->SysSOCdtF;
                     P->SysAhF    = P->SysAhNewF + P->SysAhOldF;
                     P->SysAhOldF = P->SysAhF;
                     if(P->SysAhF <= -52.0)
                     {
                        P->SysAhF =-52.0;
                     }
                     if(P->SysAhF> 52.0)
                     {
                         P->SysAhF= 52.0;
                     }
                     /*
                     * SOC 변환
                     */
                     P->SysSOCBufF1 = P->SysAhF *C_FarasisP52AhNorm;//0.0125 ;// 1/80 --> 0.0125--> 일반화
                     P->SysSOCBufF2 = P->SysSOCBufF1*100.0; //--> 단위 변환 %
                     P->SysSOCF     = P->SysSocInitF+P->SysSOCBufF2;
                 }
                 P->state = SOC_STATE_Save;

            break;
            case SOC_STATE_Save:

                P->state = SOC_STATE_RUNNING;

            break;
            case SOC_STATE_CLEAR:

            break;
        }
        P->SysTime=0;
    }
}
#endif

#if Frey60Ah
void CalFrey60AhRegsInit(SocReg *P)
{
    P->SysSOCdtF=0.0;
    P->SysSoCCTF=0.0;
    P->SysAhNewF=0.0;
    P->SysAhOldF=0.0;
    P->SysAhF=0.0;
    P->SysSOCBufF1=0.0;
    P->SysSOCBufF2=0.0;
    P->SysSOCF=5.0;
    P->AVGXF=0.0;
    P->SOCX2InF=0.0;
    P->SOCX1InF=0.0;
    P->SOCX3InF=0.0;
    P->SOCX4InF=0.0;
    P->SOCX2OutF=0.0;
    P->SOCX1OutF=0.0;
    P->SOCX3OutF=0.0;
    P->SOCX4OutF=0.0;

    P->SOCX4InFAZore=0.0;
    P->SOCX3InFAZore=0.0;
    P->SOCX2InFAZore=0.0;
    P->SOCX1InFAZore=0.0;
    P->SOCX4OutFAZore=0.0;
    P->SOCX3OutFAZore=0.0;
    P->SOCX2OutFAZore=0.0;
    P->SOCX1OutFAZore=0.0;
    P->AZoreCalCout=0;



    P->SOCX4InFBZore=0.0;
    P->SOCX3InFBZore=0.0;
    P->SOCX2InFBZore=0.0;
    P->SOCX1InFBZore=0.0;
    P->SOCX4OutFBZore=0.0;
    P->SOCX3OutFBZore=0.0;
    P->SOCX2OutFBZore=0.0;
    P->SOCX1OutFBZore=0.0;
    P->BZoreCalCout=0.0;


    P->SOCX4InFCZore=0.0;
    P->SOCX3InFCZore=0.0;
    P->SOCX2InFCZore=0.0;
    P->SOCX1InFCZore=0.0;
    P->SOCX4OutFCZore=0.0;
    P->SOCX3OutFCZore=0.0;
    P->SOCX2OutFCZore=0.0;
    P->SOCX1OutFCZore=0.0;
    P->CZoreCalCout=0;

    P->SOCX4InFDZore=0.0;
    P->SOCX3InFDZore=0.0;
    P->SOCX2InFDZore=0.0;
    P->SOCX1InFDZore=0.0;
    P->SOCX4OutFDZore=0.0;
    P->SOCX3OutFDZore=0.0;
    P->SOCX2OutFDZore=0.0;
    P->SOCX1OutFDZore=0.0;
    P->DZoreCalCout=0;

    P->SOCbufF=0.0;
    P->SysSocInitF=0.0;
    P->CellAgvVoltageF=0.0;
    P->SoCStateRegs.all=0;
    P->CTCount=0;
    P->SysTime=0;
    P->SysSoCCTAbsF=0;
    P->state=SOC_STATE_IDLE;
}
void CalFrey60AhSocInit(SocReg *P)
{
    // 60Ah
      P->AVGXF         =   P->CellAgvVoltageF;
      if((LFP_VOLT_A_BOT< P->AVGXF)&&(P->AVGXF<=LFP_VOLT_A_TOP))
      {
          //#define C_Frey60Ah_SOCX1A    62.112
          //#define C_Frey60Ah_SOCX0A   -189.44
          P->AZoreCalCout++;
          P->SOCX4InFAZore = 0;
          P->SOCX3InFAZore = 0;
          P->SOCX2InFAZore = 0;
          P->SOCX1InFAZore = P->AVGXF;

          P->SOCX4OutFAZore = 0;
          P->SOCX3OutFAZore = 0;
          P->SOCX2OutFAZore = 0;
          P->SOCX1OutFAZore = C_Frey60Ah_SOCX1A*P->SOCX1InFAZore;
          P->SOCbufF        = P->SOCX1OutFAZore + C_Frey60Ah_SOCX0A;
          if(P->AZoreCalCout>3600)
          {
              P->AZoreCalCout=0;
          }
      }
      if((LFP_VOLT_B_BOT<P->AVGXF)&&(P->AVGXF<=LFP_VOLT_B_TOP))
      {
          //#define C_Frey60Ah_SOCX2B    2566.7
          //#define C_Frey60Ah_SOCX1B    -16384
          //#define C_Frey60Ah_SOCX0B     26156
          P->BZoreCalCout++;
          P->SOCX4InFBZore = 0;
          P->SOCX3InFBZore = 0;
          P->SOCX2InFBZore = P->AVGXF*P->AVGXF;
          P->SOCX1InFBZore = P->AVGXF;

          P->SOCX4OutFBZore = 0;
          P->SOCX3OutFBZore = 0;
          P->SOCX2OutFBZore = C_Frey60Ah_SOCX2B*P->SOCX2InFBZore;
          P->SOCX1OutFBZore = C_Frey60Ah_SOCX1B*P->SOCX1InFBZore;
          P->SOCbufF        = P->SOCX2OutFBZore + P->SOCX1OutFBZore+C_Frey60Ah_SOCX0B;
          if(P->BZoreCalCout>3600)
           {
               P->BZoreCalCout=0;
           }
      }
      if((LFP_VOLT_C_BOT<P->AVGXF)&&(P->AVGXF<=LFP_VOLT_C_TOP))
      {

          P->CZoreCalCout++;
          //#define C_Frey60Ah_SOCX1C    1313.1
          //#define C_Frey60Ah_SOCX0C   -4276.6
          P->SOCX4InFCZore = 0;
          P->SOCX3InFCZore = 0;
          P->SOCX2InFCZore = 0;
          P->SOCX1InFCZore = P->AVGXF;

          P->SOCX4OutFCZore = 0;
          P->SOCX3OutFCZore = 0;
          P->SOCX2OutFCZore = 0;
          P->SOCX1OutFCZore = C_Frey60Ah_SOCX1C*P->SOCX1InFAZore;
          P->SOCbufF        = P->SOCX1OutFCZore + C_Frey60Ah_SOCX0C;

          if(P->CZoreCalCout>3600)
           {
               P->CZoreCalCout=0;
           }
      }
      if((LFP_VOLT_D_BOT<P->AVGXF)&&(P->AVGXF<=LFP_VOLT_D_TOP))
      {
          P->DZoreCalCout++;
          P->SOCX4InFDZore = 0;
          P->SOCX3InFDZore = 0;
          P->SOCX2InFDZore = 0;
          P->SOCX1InFDZore = P->AVGXF;

          P->SOCX4OutFDZore = 0;
          P->SOCX3OutFDZore = 0;
          P->SOCX2OutFDZore = 0;
          P->SOCX1OutFDZore = C_Frey60Ah_SOCX1D*P->SOCX1InFAZore;
          P->SOCbufF        = P->SOCX1OutFCZore + C_Frey60Ah_SOCX0D;
          if(P->DZoreCalCout>3600)
           {
               P->DZoreCalCout=0;
           }
      }
     if(P->SOCbufF <=0.0)
     {
         P->SOCbufF = 0.0;
     }
     else if(P->SOCbufF > 90.0)
     {
         P->SOCbufF = 100.0;
     }
     P->SysSocInitF = P->SOCbufF;
}

void CalFrey60AhSocHandle(SocReg *P)
{
    P->SysTime++;
    P->AVGXF         =   P->CellAgvVoltageF;
    if(P->SysTime>=C_SocSamPleCount)
     {
         if(P->SysSoCCTAbsF>=C_SocInitCTVaule)
         {
             P->SoCStateRegs.bit.CalMeth=1;
             P->CTCount=0;
         }
         else
         {
             P->CTCount++;
             if(P->CTCount>6000)
             {
                 P->CTCount=6001;
                 P->SoCStateRegs.bit.CalMeth=0;
             }
         }
         switch (P->state)
         {

             case SOC_STATE_RUNNING:
                  if(P->SoCStateRegs.bit.CalMeth==0)
                  {
                      // 60Ah
                       if((LFP_VOLT_A_BOT< P->AVGXF)&&(P->AVGXF<=LFP_VOLT_A_TOP))
                       {
                           //#define C_Frey60Ah_SOCX1A    62.112
                           //#define C_Frey60Ah_SOCX0A   -189.44
                           P->AZoreCalCout++;
                           P->SOCX4InFAZore = 0;
                           P->SOCX3InFAZore = 0;
                           P->SOCX2InFAZore = 0;
                           P->SOCX1InFAZore = P->AVGXF;

                           P->SOCX4OutFAZore = 0;
                           P->SOCX3OutFAZore = 0;
                           P->SOCX2OutFAZore = 0;
                           P->SOCX1OutFAZore = C_Frey60Ah_SOCX1A*P->SOCX1InFAZore;
                           P->SOCbufF        = P->SOCX1OutFAZore + C_Frey60Ah_SOCX0A;
                           P->SOCbufF = P->SOCbufF +3.0;
                           if(P->AZoreCalCout>3600)
                           {
                               P->AZoreCalCout=0;
                           }
                       }
                       if((LFP_VOLT_B_BOT<P->AVGXF)&&(P->AVGXF<=LFP_VOLT_B_TOP))
                       {
                           //#define C_Frey60Ah_SOCX2B    2566.7
                           //#define C_Frey60Ah_SOCX1B    -16384
                           //#define C_Frey60Ah_SOCX0B     26156
                           P->BZoreCalCout++;
                           P->SOCX4InFBZore = 0;
                           P->SOCX3InFBZore = 0;
                           P->SOCX2InFBZore = P->AVGXF*P->AVGXF;
                           P->SOCX1InFBZore = P->AVGXF;

                           P->SOCX4OutFBZore = 0;
                           P->SOCX3OutFBZore = 0;
                           P->SOCX2OutFBZore = C_Frey60Ah_SOCX2B*P->SOCX2InFBZore;
                           P->SOCX1OutFBZore = C_Frey60Ah_SOCX1B*P->SOCX1InFBZore;
                           P->SOCbufF        = P->SOCX2OutFBZore + P->SOCX1OutFBZore+C_Frey60Ah_SOCX0B;
                           if(P->AVGXF>3.292)
                           {
                               P->SOCbufF =39.5;
                           }
                           if(P->BZoreCalCout>3600)
                            {
                                P->BZoreCalCout=0;
                            }
                       }
                       if((LFP_VOLT_C_BOT<P->AVGXF)&&(P->AVGXF<=LFP_VOLT_C_TOP))
                       {

                          P->CZoreCalCout++;
                         /*
                           //#define C_Frey60Ah_SOCX1C    1313.1
                           //#define C_Frey60Ah_SOCX0C   -4276.6
                           P->SOCX4InFCZore = 0;
                           P->SOCX3InFCZore = 0;
                           P->SOCX2InFCZore = 0;
                           P->SOCX1InFCZore = P->AVGXF;

                           P->SOCX4OutFCZore = 0;
                           P->SOCX3OutFCZore = 0;
                           P->SOCX2OutFCZore = 0;
                           P->SOCX1OutFCZore = C_Frey60Ah_SOCX1C*P->SOCX1InFAZore;
                           P->SOCbufF        = P->SOCX1OutFCZore + C_Frey60Ah_SOCX0C;*/

                           if((3.294<=P->AVGXF)&&(P->AVGXF<3.295))
                           {
                               P->SOCbufF =55;
                           }
                           if((3.295<=P->AVGXF)&&(P->AVGXF<3.304))
                           {
                               P->SOCbufF =65;
                           }
                           if((3.304<=P->AVGXF)&&(P->AVGXF<3.312))
                           {
                               P->SOCbufF =70;
                           }
                           if((3.312<=P->AVGXF)&&(P->AVGXF<=3.317))
                           {
                               P->SOCbufF =75;
                           }
                           if(P->CZoreCalCout>3600)
                            {
                                P->CZoreCalCout=0;
                            }
                       }
                       if((LFP_VOLT_D_BOT<P->AVGXF)&&(P->AVGXF<=LFP_VOLT_D_TOP))
                       {
                           P->DZoreCalCout++;
                         /*
                           P->SOCX4InFDZore = 0;
                           P->SOCX3InFDZore = 0;
                           P->SOCX2InFDZore = 0;
                           P->SOCX1InFDZore = P->AVGXF;

                           P->SOCX4OutFDZore = 0;
                           P->SOCX3OutFDZore = 0;
                           P->SOCX2OutFDZore = 0;
                           P->SOCX1OutFDZore = C_Frey60Ah_SOCX1D*P->SOCX1InFAZore;
                           P->SOCbufF        = P->SOCX1OutFCZore + C_Frey60Ah_SOCX0D;
                           P->SOCbufF        = P->SOCbufF+7;
                           */
                           if((3.317<=P->AVGXF)&&(P->AVGXF<3.334))
                           {
                               P->SOCbufF =75;
                           }
                           if((3.334<=P->AVGXF)&&(P->AVGXF<3.335))
                           {
                               P->SOCbufF =85;
                           }
                           if((3.335<=P->AVGXF)&&(P->AVGXF<3.336))
                           {
                               P->SOCbufF =95;
                           }
                           if((3.336<=P->AVGXF)&&(P->AVGXF<3.337))
                           {
                               P->SOCbufF =100;
                           }
                           if(P->CZoreCalCout>3600)
                            {
                                P->CZoreCalCout=0;
                            }
                           if(P->DZoreCalCout>3600)
                            {
                                P->DZoreCalCout=0;
                            }
                       }
                       if(P->SOCbufF <=0.0)
                       {
                           P->SOCbufF = 0.0;
                       }
                       else if(P->SOCbufF > 90.0)
                       {
                           P->SOCbufF = 100.0;
                       }
                       P->SysSocInitF = P->SOCbufF;
                  }
                  if(P->SoCStateRegs.bit.CalMeth==1)
                  {
                      /*
                       *
                       */
                      P->SysSOCdtF = C_CTSampleTime*C_SocCumulativeTime; // CumulativeTime(1/3600) -> 누적시간
                      P->SysAhNewF = P->SysSoCCTF * P->SysSOCdtF;
                      P->SysAhF    = P->SysAhNewF + P->SysAhOldF;
                      P->SysAhOldF = P->SysAhF;
                      if(P->SysAhF <= -52.0)
                      {
                         P->SysAhF =-52.0;
                      }
                      if(P->SysAhF> 52.0)
                      {
                          P->SysAhF= 52.0;
                      }
                      /*
                      * SOC 변환
                      */
                      P->SysSOCBufF1 = P->SysAhF *C_Frey60AhNorm;// 1/48(0.0208)
                      P->SysSOCBufF2 = P->SysSOCBufF1*100.0; //--> 단위 변환 %
                      P->SysSOCF     = P->SysSocInitF+P->SysSOCBufF2;
                  }
                  P->state = SOC_STATE_Save;

             break;
             case SOC_STATE_Save:

                 P->state = SOC_STATE_RUNNING;

             break;
             case SOC_STATE_CLEAR:

             break;
         }
         P->SysTime=0;
     }
}

#endif
