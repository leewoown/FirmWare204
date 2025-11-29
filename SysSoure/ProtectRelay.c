


#include "DSP28x_Project.h"
#include "parameter.h"
#include "ProtectRelay.h"
#include <stdio.h>
#include <math.h>
#include <string.h>


extern void ProtectRlySateCheck(PrtectRelayReg *P);
extern void ProtectRlyVarINIT(PrtectRelayReg *P);
extern void ProtectRlyOnInit(PrtectRelayReg *P);
extern void ProtectRlyOnHandle(PrtectRelayReg *P);
extern void ProtectRlyOffInit(PrtectRelayReg *P);
extern void ProtectRlyOffHandle(PrtectRelayReg *P);
extern void ProtectRlyEMSHandle(PrtectRelayReg *P);

void ProtectRlyVarINIT(PrtectRelayReg *P)
{
    P->State.all=0;
    P->WakeupOn_ProRlyOnCount=0;
    P->WakeupOn_ProRlyOffCount=0;
    P->WakeupOn_PRlyOnCount=0;
    P->WakeupOn_NRlyOnCount=0;
    P->WakeupOff_PRlyOffCount=0;
    P->WakeupOff_NRlyOffCount=0;
    P->Protect_ProRlyOnCount=0;
    P->Protect_ProRlyOffCount=0;
    P->Protect_PRlyOffCount=0;
    P->Protect_NRlyOffCount=0;
    P->WakeupOn_TimeCount=0;
    P->WakeupOff_TimeCount=0;
    P->StateMachine=STATERly_INIT;
}
void ProtectRelayHandle(PrtectRelayReg *P)
{
    switch(P->StateMachine)
    {
        case STATERly_INIT :
              ProtectRlyVarINIT(P);
        break;
        case STATERly_STANDBY :
             if((P->State.bit.PRlyDI==1)||(P->State.bit.NRlyDI==1))
             {
                    P->State.bit.RlyFaulttSate =1;
             }
             if((P->State.bit.PRlyDI==0)&&(P->State.bit.NRlyDI==0))
             {
                    P->State.bit.RlyFaulttSate =0;
             }
             P->State.bit.WakeuPOffEND=0;
             P->State.bit.WakeuPOnEND=0;
        break;
        case STATERly_Ready :
             P->State.bit.WakeuPOnEND=0;
             P->State.bit.WakeuPOffEND=0;
             if(P->State.bit.WakeUpEN==1)
             {
                 P->StateMachine = STATERly_SeqRlyOn;
             }
        break;
        case STATERly_SeqRlyOn :
             P->State.bit.NRlyDO=1;
             delay_ms(50);
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==0))
             {
                 P->State.bit.PreRlyDO=1;
                 P->State.bit.ProRlyDI=1;
             }
             delay_ms(100);
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==1)&&(P->State.bit.PRlyDI==0))
             {
                 P->State.bit.PRlyDO=1;
             }
             delay_ms(50);
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==1)&&(P->State.bit.PRlyDI==1))
             {
                 P->State.bit.PreRlyDO=0;
                 P->State.bit.ProRlyDI=0;
             }
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==1))
             {
                 P->StateMachine = STATERly_SeqRlyRun;
             }
        break;
        case STATERly_SeqRlyRun :

              P->State.bit.WakeuPOffEND=0;
              P->State.bit.WakeuPOnEND=1;
              if(P->State.bit.WakeUpEN==0)
              {
                P->StateMachine = STATERly_SeqRlyOff;
              }
        break;
        case STATERly_SeqRlyOff :
             P->State.bit.NRlyDO=0;
             delay_ms(50);
             if((P->State.bit.NRlyDI==0)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==1))
             {
                 P->State.bit.PRlyDO=0;
             }
             if((P->State.bit.NRlyDI==0)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==0))
             {
                 P->State.bit.WakeuPOffEND=1;
                 P->State.bit.WakeuPOnEND=0;
                 P->StateMachine = STATERly_Ready;
             }

        break;
        case STATERly_Protect:
             P->State.bit.NRlyDO=0;
             delay_ms(10);
             P->State.bit.PRlyDO=0;

        break;
        case STATERly_Manual:

        break;
        case STATERly_Reset:
             P->State.bit.PRlyDO=0;
             P->State.bit.NRlyDO=0;
             P->State.bit.PreRlyDO=0;
             P->State.bit.WakeuPOnEND=0;
             P->State.bit.WakeuPOffEND=0;
             if((P->State.bit.NRlyDI==1)&&(P->State.bit.ProRlyDI==0)&&(P->State.bit.PRlyDI==1))
             {
                 P->State.bit.WakeuPOnEND=1;
                 P->State.bit.WakeuPOffEND=0;
          //       P->StateMachine = STATERly_OffSeqReady;
             }
        break;

        break;
        default :
        break;

    }
}

void ProtectRlyOnInit(PrtectRelayReg *P)
{
    P->WakeupOn_ProRlyOnCount=0;
    P->WakeupOn_PRlyOnCount=0;
    P->WakeupOn_ProRlyOffCount=0;
    P->State.bit.WakeuPOffEND=0;
   // P->State.bit.WakeuPOnEND=0;
    P->WakeupOff_TimeCount=0;
}
void ProtectRlyOffInit(PrtectRelayReg *P)
{
    P->WakeupOff_NRlyOffCount=0;
    P->State.bit.WakeuPOnEND=0;
    P->WakeupOn_TimeCount=0;
    P->WakeupOff_TimeCount=0;
}





