#include "DSP28x_Project.h"
#include "BATCellModel.h"
#include "stdio.h"
#include "math.h"
#include "string.h"


/*-----------------------------------------------------------
 내부 함수 선언
-----------------------------------------------------------*/
static float BatteryModel_Abs(float x);
static float BatteryModel_Clamp(float value, float minVal, float maxVal);
static float BatteryModel_GetTempFactor(float tempC);
static float BatteryCurrent_RandUniform(float minVal, float maxVal);

/*-----------------------------------------------------------
 전역 변수
-----------------------------------------------------------*/
float gBattery_InitSocPct             = 59.9f;
float gBattery_InitTempC              = 19.8f;
float gBattery_CapacityAh             = 80.0f;

float gBattery_InputCurrentA          = 0.0f;
float gBattery_OutputCurrentA         = 0.0f;

float gBattery_RuntimeTempC           = 19.0;
Uint16 gBattery_UseRuntimeTemp        = 1u;

float gBattery_CurrentHuntingA        = 0.0f;
Uint16 gBattery_CurrentHuntingTimerMs = 100u;
Uint32 gBattery_CurrentRandSeed       = 12345UL;

/* Trace 변수 */
float gTrace_SOC                      = 0.0f;
float gTrace_OCV                      = 0.0f;
float gTrace_R0Base                   = 0.0f;
float gTrace_R0                       = 0.0f;
float gTrace_IR                       = 0.0f;
float gTrace_TV                       = 0.0f;
float gTrace_OutV                     = 0.0f;

float gTrace_AhCharge                 = 0.0f;
float gTrace_AhDischarge              = 0.0f;
float gTrace_AhNet                    = 0.0f;

float gTrace_CellVoltMaxV             = 0.0f;
float gTrace_CellVoltMinV             = 0.0f;
float gTrace_CellTempC                = 0.0f;
float gTrace_CellTempMaxC             = 0.0f;
float gTrace_CellTempMinC             = 0.0f;

BatteryModel gBatteryModel;

/*-----------------------------------------------------------
 OCV Table (10% 간격)
-----------------------------------------------------------*/
static const BatteryTablePoint OcvTable[] =
{
    {  0.0f, 3.458f },
    { 10.0f, 3.526f },
    { 20.0f, 3.594f },
    { 30.0f, 3.627f },
    { 40.0f, 3.659f },
    { 50.0f, 3.708f },
    { 60.0f, 3.809f },
    { 70.0f, 3.902f },
    { 80.0f, 3.996f },
    { 90.0f, 4.095f },
    {100.0f, 4.194f }
};

/*-----------------------------------------------------------
 R0 Table (10% 간격, 20sec 기준, mOhm)
-----------------------------------------------------------*/
static const BatteryTablePoint R0Table[] =
{
    {  0.0f, 0.900f },
    { 10.0f, 0.890f },
    { 20.0f, 0.890f },
    { 30.0f, 0.890f },
    { 40.0f, 0.860f },
    { 50.0f, 0.850f },
    { 60.0f, 0.880f },
    { 70.0f, 0.910f },
    { 80.0f, 0.970f },
    { 90.0f, 1.150f },
    {100.0f, 1.330f }
};

#define OCV_TABLE_SIZE   ((Uint16)(sizeof(OcvTable) / sizeof(OcvTable[0])))
#define R0_TABLE_SIZE    ((Uint16)(sizeof(R0Table)  / sizeof(R0Table[0])))

/*-----------------------------------------------------------
 내부 함수
-----------------------------------------------------------*/
static float BatteryModel_Abs(float x)
{
    if (x < 0.0f)
    {
        return -x;
    }

    return x;
}

static float BatteryModel_Clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }

    if (value > maxVal)
    {
        return maxVal;
    }

    return value;
}

static float BatteryModel_GetTempFactor(float tempC)
{
    float factor;

    factor = 1.0f + ((BATTERY_TEMP_REF_C - tempC) * BATTERY_TEMP_COEF_PER_C);

    if (factor < 0.2f)
    {
        factor = 0.2f;
    }

    if (factor > 5.0f)
    {
        factor = 5.0f;
    }

    return factor;
}

static float BatteryCurrent_RandUniform(float minVal, float maxVal)
{
    float norm;

    gBattery_CurrentRandSeed =
        (Uint32)(1664525UL * gBattery_CurrentRandSeed + 1013904223UL);

    norm = (float)(gBattery_CurrentRandSeed & 0x0000FFFFUL) / 65535.0f;

    return minVal + ((maxVal - minVal) * norm);
}

/*-----------------------------------------------------------
 Lookup Table 선형 보간
-----------------------------------------------------------*/
float BatteryModel_Lookup(const BatteryTablePoint *pTable,
                          Uint16 tableSize,
                          float socPct)
{
    Uint16 i;
    float x0;
    float x1;
    float y0;
    float y1;
    float ratio;

    if ((pTable == 0) || (tableSize == 0u))
    {
        return 0.0f;
    }

    if (tableSize == 1u)
    {
        return pTable[0].value;
    }

    if (socPct <= pTable[0].socPct)
    {
        return pTable[0].value;
    }

    if (socPct >= pTable[tableSize - 1u].socPct)
    {
        return pTable[tableSize - 1u].value;
    }

    for (i = 0u; i < (tableSize - 1u); i++)
    {
        x0 = pTable[i].socPct;
        x1 = pTable[i + 1u].socPct;

        if ((socPct >= x0) && (socPct <= x1))
        {
            y0 = pTable[i].value;
            y1 = pTable[i + 1u].value;

            if ((x1 - x0) == 0.0f)
            {
                return y0;
            }

            ratio = (socPct - x0) / (x1 - x0);
            return y0 + ((y1 - y0) * ratio);
        }
    }

    return pTable[tableSize - 1u].value;
}

/*-----------------------------------------------------------
 모델 초기화
-----------------------------------------------------------*/
void BatteryModel_Init(BatteryModel *pModel,
                       float initSocPct,
                       float initTempC,
                       float capacityAh,
                       const BatteryTablePoint *pOcvTable,
                       Uint16 ocvTableSize,
                       const BatteryTablePoint *pR0Table,
                       Uint16 r0TableSize)
{
    if (pModel == 0)
    {
        return;
    }

    pModel->initSocPct = BatteryModel_Clamp(initSocPct,
                                            BATTERY_SOC_MIN_PCT,
                                            BATTERY_SOC_MAX_PCT);
    pModel->socPct = pModel->initSocPct;

    pModel->initTempC = BatteryModel_Clamp(initTempC,
                                           BATTERY_TEMP_MIN_C,
                                           BATTERY_TEMP_MAX_C);
    pModel->tempC = pModel->initTempC;
    pModel->tempMaxC = pModel->initTempC;
    pModel->tempMinC = pModel->initTempC;

    pModel->capacityAh = capacityAh;
    pModel->capacityAs = capacityAh * 3600.0f;

    pModel->chargeAh = 0.0f;
    pModel->dischargeAh = 0.0f;
    pModel->netAh = 0.0f;

    pModel->currentA = 0.0f;
    pModel->ocvV = 0.0f;
    pModel->r0BaseOhm = 0.0f;
    pModel->r0Ohm = 0.0f;
    pModel->irV = 0.0f;
    pModel->tvV = 0.0f;
    pModel->outV = 0.0f;

    pModel->cellVoltMaxV = 0.0f;
    pModel->cellVoltMinV = 0.0f;

    pModel->pOcvTable = pOcvTable;
    pModel->ocvTableSize = ocvTableSize;
    pModel->pR0Table = pR0Table;
    pModel->r0TableSize = r0TableSize;

    pModel->ocvV = BatteryModel_Lookup(pModel->pOcvTable,
                                       pModel->ocvTableSize,
                                       pModel->socPct);

    pModel->tvV = pModel->ocvV;
    pModel->outV = pModel->ocvV;

    pModel->cellVoltMaxV = pModel->outV;
    pModel->cellVoltMinV = pModel->outV;
}

/*-----------------------------------------------------------
 모델 리셋
-----------------------------------------------------------*/
void BatteryModel_Reset(BatteryModel *pModel,
                        float initSocPct,
                        float initTempC)
{
    if (pModel == 0)
    {
        return;
    }

    pModel->initSocPct = BatteryModel_Clamp(initSocPct,
                                            BATTERY_SOC_MIN_PCT,
                                            BATTERY_SOC_MAX_PCT);
    pModel->socPct = pModel->initSocPct;

    pModel->initTempC = BatteryModel_Clamp(initTempC,
                                           BATTERY_TEMP_MIN_C,
                                           BATTERY_TEMP_MAX_C);
    pModel->tempC = pModel->initTempC;
    pModel->tempMaxC = pModel->initTempC;
    pModel->tempMinC = pModel->initTempC;

    pModel->chargeAh = 0.0f;
    pModel->dischargeAh = 0.0f;
    pModel->netAh = 0.0f;

    pModel->currentA = 0.0f;
    pModel->irV = 0.0f;

    pModel->ocvV = BatteryModel_Lookup(pModel->pOcvTable,
                                       pModel->ocvTableSize,
                                       pModel->socPct);

    pModel->tvV = pModel->ocvV;
    pModel->outV = pModel->ocvV;

    pModel->cellVoltMaxV = pModel->outV;
    pModel->cellVoltMinV = pModel->outV;
}

/*-----------------------------------------------------------
 전류 Ramp + Hunting
 - 상승: 70A/s
 - 하강: 233.333A/s  -> 70A -> 0A, 약 0.3초
-----------------------------------------------------------*/
void BatteryCurrentModel_Step(float inputCurrentA)
{
    float currentErrorA;
    float rampStepA;

    currentErrorA = inputCurrentA - gBattery_OutputCurrentA;

    if (currentErrorA >= 0.0f)
    {
        rampStepA = BATTERY_CURRENT_RISE_A_PER_SEC * BATTERY_MODEL_DT_SEC;
    }
    else
    {
        rampStepA = BATTERY_CURRENT_FALL_A_PER_SEC * BATTERY_MODEL_DT_SEC;
    }

    if (currentErrorA > rampStepA)
    {
        gBattery_OutputCurrentA += rampStepA;
    }
    else if (currentErrorA < (-rampStepA))
    {
        gBattery_OutputCurrentA -= rampStepA;
    }
    else
    {
        gBattery_OutputCurrentA = inputCurrentA;
    }

    if (BatteryModel_Abs(inputCurrentA - gBattery_OutputCurrentA) <= 0.1f)
    {
        gBattery_CurrentHuntingTimerMs++;

        if (gBattery_CurrentHuntingTimerMs >= BATTERY_CURRENT_HUNTING_PERIOD_MS)
        {
            gBattery_CurrentHuntingTimerMs = 0u;

            gBattery_CurrentHuntingA = BatteryCurrent_RandUniform(
                                           BATTERY_CURRENT_HUNTING_MIN_A,
                                           BATTERY_CURRENT_HUNTING_MAX_A);
        }
    }
    else
    {
        gBattery_CurrentHuntingTimerMs = 0u;
        gBattery_CurrentHuntingA = 0.0f;
    }

    gBattery_OutputCurrentA += gBattery_CurrentHuntingA;
}

/*-----------------------------------------------------------
 배터리 모델 1ms 계산
-----------------------------------------------------------*/
void BatteryModel_Step(BatteryModel *pModel, float currentA)
{
    float deltaSocPct;
    float r0_mOhm;
    float tempFactor;
    float deltaAh;

    if (pModel == 0)
    {
        return;
    }

    pModel->currentA = currentA;

    if (gBattery_UseRuntimeTemp != 0u)
    {
        pModel->tempC = BatteryModel_Clamp(gBattery_RuntimeTempC,
                                           BATTERY_TEMP_MIN_C,
                                           BATTERY_TEMP_MAX_C);
    }
    else
    {
        pModel->tempC = BatteryModel_Clamp(pModel->initTempC,
                                           BATTERY_TEMP_MIN_C,
                                           BATTERY_TEMP_MAX_C);
    }

    if (pModel->capacityAs > 0.0f)
    {
        deltaSocPct = (currentA * BATTERY_MODEL_DT_SEC / pModel->capacityAs) * 100.0f;
        pModel->socPct += deltaSocPct;
        pModel->socPct = BatteryModel_Clamp(pModel->socPct,
                                            BATTERY_SOC_MIN_PCT,
                                            BATTERY_SOC_MAX_PCT);
    }

    /* Ah 적산 */
    deltaAh = currentA * BATTERY_MODEL_DT_SEC / 3600.0f;
    pModel->netAh += deltaAh;

    if (currentA > 0.0f)
    {
        pModel->chargeAh += deltaAh;
    }
    else if (currentA < 0.0f)
    {
        pModel->dischargeAh += (-deltaAh);
    }

    pModel->ocvV = BatteryModel_Lookup(pModel->pOcvTable,
                                       pModel->ocvTableSize,
                                       pModel->socPct);

    r0_mOhm = BatteryModel_Lookup(pModel->pR0Table,
                                  pModel->r0TableSize,
                                  pModel->socPct);

    pModel->r0BaseOhm = r0_mOhm * 0.001f;

    tempFactor = BatteryModel_GetTempFactor(pModel->tempC);
    pModel->r0Ohm = pModel->r0BaseOhm * tempFactor;

    if (BatteryModel_Abs(currentA) <= BATTERY_CURRENT_DEADBAND_A)
    {
        pModel->irV = 0.0f;
        pModel->tvV = pModel->ocvV;
    }
    else
    {
        pModel->irV = currentA * pModel->r0Ohm;
        pModel->tvV = pModel->ocvV + pModel->irV;
    }

    pModel->tvV = BatteryModel_Clamp(pModel->tvV,
                                     BATTERY_CELL_MIN_V,
                                     BATTERY_CELL_MAX_V);

    pModel->outV = pModel->tvV;

    if (pModel->outV > pModel->cellVoltMaxV)
    {
        pModel->cellVoltMaxV = pModel->outV;
    }

    if (pModel->outV < pModel->cellVoltMinV)
    {
        pModel->cellVoltMinV = pModel->outV;
    }

    if (pModel->tempC > pModel->tempMaxC)
    {
        pModel->tempMaxC = pModel->tempC;
    }

    if (pModel->tempC < pModel->tempMinC)
    {
        pModel->tempMinC = pModel->tempC;
    }
}

/*-----------------------------------------------------------
 사용자 초기화
-----------------------------------------------------------*/
void BatteryModel_UserInit(void)
{
    gBattery_OutputCurrentA = 0.0f;
    gBattery_CurrentHuntingA = 0.0f;
    gBattery_CurrentHuntingTimerMs = 0u;
    gBattery_CurrentRandSeed = 12345UL;

    BatteryModel_Init(&gBatteryModel,
                      gBattery_InitSocPct,
                      gBattery_InitTempC,
                      gBattery_CapacityAh,
                      OcvTable,
                      OCV_TABLE_SIZE,
                      R0Table,
                      R0_TABLE_SIZE);

    gTrace_SOC = gBatteryModel.socPct;
    gTrace_OCV = gBatteryModel.ocvV;
    gTrace_R0Base = gBatteryModel.r0BaseOhm;
    gTrace_R0 = gBatteryModel.r0Ohm;
    gTrace_IR = gBatteryModel.irV;
    gTrace_TV = gBatteryModel.tvV;
    gTrace_OutV = gBatteryModel.outV;

    gTrace_AhCharge = gBatteryModel.chargeAh;
    gTrace_AhDischarge = gBatteryModel.dischargeAh;
    gTrace_AhNet = gBatteryModel.netAh;

    gTrace_CellVoltMaxV = gBatteryModel.cellVoltMaxV;
    gTrace_CellVoltMinV = gBatteryModel.cellVoltMinV;
    gTrace_CellTempC = gBatteryModel.tempC;
    gTrace_CellTempMaxC = gBatteryModel.tempMaxC;
    gTrace_CellTempMinC = gBatteryModel.tempMinC;
}

/*-----------------------------------------------------------
 1ms Task
-----------------------------------------------------------*/
void BatteryModel_1msTask(void)
{
    BatteryCurrentModel_Step(gBattery_InputCurrentA);
    BatteryModel_Step(&gBatteryModel, gBattery_OutputCurrentA);

    gTrace_SOC = gBatteryModel.socPct;
    gTrace_OCV = gBatteryModel.ocvV;
    gTrace_R0Base = gBatteryModel.r0BaseOhm;
    gTrace_R0 = gBatteryModel.r0Ohm;
    gTrace_IR = gBatteryModel.irV;
    gTrace_TV = gBatteryModel.tvV;
    gTrace_OutV = gBatteryModel.outV;

    gTrace_AhCharge = gBatteryModel.chargeAh;
    gTrace_AhDischarge = gBatteryModel.dischargeAh;
    gTrace_AhNet = gBatteryModel.netAh;

    gTrace_CellVoltMaxV = gBatteryModel.cellVoltMaxV;
    gTrace_CellVoltMinV = gBatteryModel.cellVoltMinV;
    gTrace_CellTempC = gBatteryModel.tempC;
    gTrace_CellTempMaxC = gBatteryModel.tempMaxC;
    gTrace_CellTempMinC = gBatteryModel.tempMinC;
}
