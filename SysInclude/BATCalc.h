/* ==============================================================================
System Name:  현대자동차 수소 지게차 80V

File Name:		PARAMETER.H

Description:	현대
          	    Orientation Control for a Three Phase AC Induction Motor. 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DMC1500 board.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
=================================================================================  */

//#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
//#include "F2806x_Device.h"      // F2806x Headerfile Include File
//#include "F2806x_Examples.h"    // F2806x Examples Include File
//#include "DSP28x_Project.h"

#ifndef BAT_Calc_H
#define BAT_Calc_H

#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"
#define  ShipPack_168S1P       1

#if ShipPack_168S1P

#define ModuleEA               9
#define PackCellEA             168

typedef enum
{
  BATCALC_STATE_IDLE,
  BATCALC_STATE_RUNNING,
  BATCALC_STATE_Save,
  BATCALC_STATE_CLEAR
} BatCalcMachine;
struct BatCalcStauts_BIT
{       // bits   description
   unsigned int     State00         :1; // 0
   unsigned int     State01         :1; // 1
   unsigned int     State02         :1; // 2
   unsigned int     State03         :1; // 3
   unsigned int     State04         :1; // 4
   unsigned int     State05         :1; // 5
   unsigned int     State06         :1; // 6
   unsigned int     State07         :1; // 7
   unsigned int     State08         :1; // 8
   unsigned int     State09         :1; // 9
   unsigned int     State10         :1; // 10
   unsigned int     State11         :1; // 11
   unsigned int     State12         :1; // 12
   unsigned int     State13         :1; // 13
   unsigned int     State14         :1; // 14
   unsigned int     State15         :1; // 15
};
union BatCalcStauts_REG
{
   unsigned int     all;

   struct BatCalcStauts_BIT bit;
};
typedef struct
{
   /*
    *
    */
    Uint16  MDCellMaxVolt     [ModuleEA];
    Uint16  MDCellMinVolt     [ModuleEA];
    Uint16  MDCellAgvVolt     [ModuleEA];
    Uint16  MDCellDivVolt     [ModuleEA];

    int16   MDCellMaxTemps    [ModuleEA];
    int16   MDCellMinTemps    [ModuleEA];
    int16   MDCellAgvTemps    [ModuleEA];
    int16   MDCellDivTemps    [ModuleEA];

    float32  MDCellMaxVoltF   [ModuleEA];
    float32  MDCellMinVoltF   [ModuleEA];
    float32  MDCellAgvVoltF   [ModuleEA];
    float32  MDCellDivVoltF   [ModuleEA];

    float32  MDCellMaxTempsF  [ModuleEA];
    float32  MDCellMinTempsF  [ModuleEA];
    float32  MDCellAgvTempsF  [ModuleEA];
    float32  MDCellDivTempsF  [ModuleEA];



    Uint16  MDTotalVolt [ModuleEA];
    float32 MDTotalVoltF[ModuleEA];

    Uint16  MDMaxVoltPo[ModuleEA];
    Uint16  MDMinVoltPo[ModuleEA];
    Uint16  MDMaxTempsPo[ModuleEA];
    Uint16  MDMinTempsPo[ModuleEA];

    Uint16  MDstatusbit[ModuleEA];
    /*
     *
     */
    Uint16 PackPTADC;
    Uint16 PackPTCAN;
    Uint16 PackPT;
    float32 PackPTADCF;
    float32 PackPTCANF;
    float32 PackPTF;
    /*
     *
     */
    int PackCTADC;
    int PackCTCAN;
    int PackCTAbs;
    int PackCTAgv;
    float32 PackCTADCF;
    float32 PackCTCANF;
    float32 PackCTAbsF;
    float32 PackCTAgvF;
    /*
     *
     */
    float32 PackCellMaxVoltF;
    float32 PackCellMinVoltF;
    float32 PackCellAgvVoltF;
    float32 PackCellDivVoltF;

    float32 PackCellMaxTempsF;
    float32 PackCellMinTempsF;
    float32 PackCellAgvTempsF;
    float32 PackCellDivTempsF;

    Uint16 PackCellMaxVoltPos;
    Uint16 PackCellMinVoltPos;
    Uint16 PackCellMaxTempsPos;
    Uint16 PackCellMinTempsPos;


  union BatCalcStauts_REG BatCalcStautsRegs;
}BatCalcReg;


#endif //ShipPack_168S1P


#endif  // end of PARAMETER.H definition


//===========================================================================
// No more.
//===========================================================================
