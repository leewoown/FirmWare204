/* ==============================================================================
*/
#ifndef BATCellModel_H

#define BATCellModel_H

#include "F2806x_Cla_typedefs.h"// F2806x CLA Type definitions
#include "F2806x_Device.h"      // F2806x Headerfile Include File
#include "F2806x_Examples.h"    // F2806x Examples Include File
#include "DSP28x_Project.h"

#ifdef __cplusplus
 extern "C"
 {
#endif
 /*-----------------------------------------------------------
  상수 정의
 -----------------------------------------------------------*/
 #define BATTERY_MODEL_DT_SEC              (0.001f)   /* 1ms */
 #define BATTERY_CURRENT_DEADBAND_A        (0.5f)

 #define BATTERY_SOC_MIN_PCT               (0.0f)
 #define BATTERY_SOC_MAX_PCT               (100.0f)

 #define BATTERY_CELL_MAX_V                (4.250f)
 #define BATTERY_CELL_MIN_V                (2.500f)

 #define BATTERY_TEMP_MAX_C                (100.0f)
 #define BATTERY_TEMP_MIN_C                (-40.0f)

 #define BATTERY_TEMP_REF_C                (25.0f)
 #define BATTERY_TEMP_COEF_PER_C           (0.010f)   /* 1℃당 1% */

 #define BATTERY_CURRENT_RISE_A_PER_SEC    (70.0f)       /* 상승: 0 -> 70A, 1초 */
 #define BATTERY_CURRENT_FALL_A_PER_SEC    (233.3333f)   /* 하강: 70 -> 0A, 0.3초 */

 #define BATTERY_CURRENT_HUNTING_PERIOD_MS (100u)
 #define BATTERY_CURRENT_HUNTING_MIN_A     (-0.5f)
 #define BATTERY_CURRENT_HUNTING_MAX_A     (0.5f)

 /*-----------------------------------------------------------
  Lookup Table 구조체
 -----------------------------------------------------------*/
 typedef struct
 {
     float socPct;     /* SOC [%] */
     float value;      /* OCV[V] 또는 IR[mOhm] */
 } BatteryTablePoint;

 /*-----------------------------------------------------------
  배터리 모델 구조체
 -----------------------------------------------------------*/
 typedef struct
 {
     float initSocPct;      /* 초기 SOC [%] */
     float socPct;          /* 현재 SOC [%] */

     float initTempC;       /* 초기 온도 [degC] */
     float tempC;           /* 현재 온도 [degC] */
     float tempMaxC;        /* 최대 온도 [degC] */
     float tempMinC;        /* 최소 온도 [degC] */

     float capacityAh;      /* 셀 용량 [Ah] */
     float capacityAs;      /* 셀 용량 [As] */

     float chargeAh;        /* 충전 누적 Ah */
     float dischargeAh;     /* 방전 누적 Ah */
     float netAh;           /* 순 Ah */

     float currentA;        /* 현재 모델 입력 전류 [A] */

     float ocvV;            /* OCV [V] */
     float r0BaseOhm;       /* 보정 전 IR [Ohm] */
     float r0Ohm;           /* 보정 후 IR [Ohm] */

     float irV;             /* IR 전압 성분 [V] */
     float tvV;             /* 단자 전압 [V] */
     float outV;            /* 최종 출력 전압 [V] */

     float cellVoltMaxV;    /* 최대 셀 전압 [V] */
     float cellVoltMinV;    /* 최소 셀 전압 [V] */

     const BatteryTablePoint *pOcvTable;
     Uint16 ocvTableSize;

     const BatteryTablePoint *pR0Table;
     Uint16 r0TableSize;

 } BatteryModel;

 /*-----------------------------------------------------------
  전역 변수
  CCS Trace / Expressions용
 -----------------------------------------------------------*/
 extern float gBattery_InitSocPct;
 extern float gBattery_InitTempC;
 extern float gBattery_CapacityAh;

 extern float gBattery_InputCurrentA;       /* 외부 목표 전류 */
 extern float gBattery_OutputCurrentA;      /* Ramp/Hunting 적용 전류 */

 extern float gBattery_RuntimeTempC;        /* 실시간 온도 */
 extern Uint16 gBattery_UseRuntimeTemp;     /* 0: 초기온도 사용, 1: 실시간온도 사용 */

 extern float gBattery_CurrentHuntingA;
 extern Uint16 gBattery_CurrentHuntingTimerMs;
 extern Uint32 gBattery_CurrentRandSeed;

 /* Trace 변수 */
 extern float gTrace_SOC;
 extern float gTrace_OCV;
 extern float gTrace_R0Base;
 extern float gTrace_R0;
 extern float gTrace_IR;
 extern float gTrace_TV;
 extern float gTrace_OutV;

 extern float gTrace_AhCharge;
 extern float gTrace_AhDischarge;
 extern float gTrace_AhNet;

 extern float gTrace_CellVoltMaxV;
 extern float gTrace_CellVoltMinV;
 extern float gTrace_CellTempC;
 extern float gTrace_CellTempMaxC;
 extern float gTrace_CellTempMinC;

 extern BatteryModel gBatteryModel;

 /*-----------------------------------------------------------
  함수 선언
 -----------------------------------------------------------*/
 float BatteryModel_Lookup(const BatteryTablePoint *pTable,
                           Uint16 tableSize,
                           float socPct);

 void BatteryModel_Init(BatteryModel *pModel,
                        float initSocPct,
                        float initTempC,
                        float capacityAh,
                        const BatteryTablePoint *pOcvTable,
                        Uint16 ocvTableSize,
                        const BatteryTablePoint *pR0Table,
                        Uint16 r0TableSize);

 void BatteryModel_Reset(BatteryModel *pModel,
                         float initSocPct,
                         float initTempC);

 void BatteryCurrentModel_Step(float inputCurrentA);
 void BatteryModel_Step(BatteryModel *pModel, float currentA);

 void BatteryModel_UserInit(void);
 void BatteryModel_1msTask(void);

#ifdef __cplusplus
 }
#endif

#endif  // end of BATCellModel_H definition


//===========================================================================
// No more.
//===========================================================================
