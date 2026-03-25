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
#define TEMP_NODE_CNT   (9u)
#define SOC_NODE_CNT    (11u)
/* Temp 노드(비균일) */
static const float32 kTempNodesC_af[TEMP_NODE_CNT] =
{
    -30.0f, -20.0f, -10.0f, 0.0f, 10.0f, 25.0f, 40.0f, 50.0f, 60.0f
};

/* SOC 노드(균일 10% 간격) */
static const float32 kSocNodesPct_af[SOC_NODE_CNT] =
{
    0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f, 70.0f, 80.0f, 90.0f, 100.0f
};

/* ========== 테이블 데이터(변환 배율 적용된 값) ========== */
/* 1) 30 sec Discharge Current Limit (A) – ×9.5 */
static const float32 kTbl_Dis30s_A[TEMP_NODE_CNT][SOC_NODE_CNT] =
{
/*   0     10     20     30     40     50     60     70     80     90     100 */
 { 0.0f,  0.0f,  0.0f,   9.5f,  19.0f,  19.0f,  19.0f,  28.5f,  38.0f,  38.0f,  38.0f }, /* -30 */
 { 0.0f,  9.5f, 38.0f,  47.5f,  57.0f,  57.0f,  66.5f,  66.5f,  76.0f,  76.0f,  76.0f }, /* -20 */
 { 0.0f, 19.0f, 57.0f,  95.0f, 114.0f, 123.5f, 133.0f, 142.5f, 142.5f, 152.0f, 161.5f }, /* -10 */
 { 0.0f, 38.0f,104.5f, 133.0f, 152.0f, 161.5f, 180.5f, 190.0f, 199.5f, 218.5f, 218.5f }, /*  0 */
 { 0.0f, 66.5f,142.5f, 180.5f, 190.0f, 209.0f, 228.0f, 247.0f, 256.5f, 294.5f, 294.5f }, /* 10 */
 { 0.0f,104.5f,199.5f, 256.5f, 285.0f, 304.0f, 342.0f, 361.0f, 370.5f, 418.0f, 418.0f }, /* 25 */
 { 0.0f,104.5f,199.5f, 256.5f, 285.0f, 304.0f, 342.0f, 361.0f, 370.5f, 418.0f, 418.0f }, /* 40 */
 { 0.0f,104.5f,199.5f, 256.5f, 285.0f, 304.0f, 342.0f, 361.0f, 370.5f, 418.0f, 418.0f }, /* 50 */
 { 0.0f, 71.25f,133.0f,171.0f, 190.0f, 190.0f, 237.5f, 237.5f, 237.5f, 285.0f, 285.0f }  /* 60 */
};

/* 2) Continuous (>120s) Discharge Current Limit (A) – ×9 */
static const float32 kTbl_DisCont_A[TEMP_NODE_CNT][SOC_NODE_CNT] =
{
/*   0     10     20     30     40     50     60     70     80     90     100 */
 { 0.0f,  0.0f,  0.0f,  4.5f,  7.2f,  7.2f, 13.5f, 13.5f, 13.5f, 13.5f, 13.5f }, /* -30 */
 { 0.0f,  7.2f, 18.0f, 27.0f, 27.0f, 27.0f, 27.0f, 27.0f, 27.0f, 27.0f, 27.0f }, /* -20 */
 { 0.0f,  9.0f, 27.0f, 27.0f, 40.5f, 40.5f, 40.5f, 40.5f, 40.5f, 40.5f, 40.5f }, /* -10 */
 { 0.0f, 54.0f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f }, /*  0 */
 { 0.0f, 72.0f, 99.0f,108.0f,108.0f,108.0f,108.0f,108.0f,108.0f,108.0f,108.0f }, /* 10 */
 { 0.0f, 54.0f, 99.0f,135.0f,135.0f,135.0f,162.0f,162.0f,162.0f,162.0f,162.0f }, /* 25 */
 { 0.0f, 54.0f, 99.0f,135.0f,135.0f,135.0f,162.0f,162.0f,162.0f,162.0f,162.0f }, /* 40 */
 { 0.0f, 54.0f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f, 67.5f }, /* 50 */
 { 0.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f, 45.0f }  /* 60 */
};

/* 3) 10 sec Charge Current Limit (A) – ×9.6 */
static const float32 kTbl_Chg10s_A[TEMP_NODE_CNT][SOC_NODE_CNT] =
{
/*   0      10      20      30      40      50      60      70      80      90      100 */
 { 2.40f,  2.40f,  2.40f,  2.40f,  2.40f,  2.40f,  2.40f,  2.40f,  1.92f,  1.44f, 0.00f }, /* -30 */
 { 9.60f,  9.60f,  9.60f,  9.60f,  7.20f,  7.20f,  7.20f,  4.80f,  2.88f,  2.88f, 0.00f }, /* -20 */
 { 52.80f, 52.80f, 52.80f, 48.00f, 38.40f, 28.80f, 19.20f,  9.60f,  9.60f,  5.76f, 0.00f }, /* -10 */
 { 96.00f, 96.00f, 96.00f, 86.40f, 76.80f, 57.60f, 48.00f, 28.80f, 19.20f,  9.60f, 0.00f }, /*  0 */
 { 134.40f,134.40f,134.40f,124.80f,115.20f, 96.00f, 67.20f, 48.00f, 28.80f, 19.20f, 0.00f }, /* 10 */
 { 201.60f,201.60f,201.60f,201.60f,172.80f,153.60f,115.20f, 86.40f, 57.60f, 38.40f, 0.00f }, /* 25 */
 { 201.60f,201.60f,201.60f,201.60f,172.80f,153.60f,115.20f, 86.40f, 57.60f, 38.40f, 0.00f }, /* 40 */
 { 201.60f,201.60f,201.60f,201.60f,172.80f,153.60f,115.20f, 86.40f, 57.60f, 38.40f, 0.00f }, /* 50 */
 { 201.60f,201.60f,201.60f,201.60f,172.80f,153.60f,115.20f, 86.40f, 57.60f, 38.40f, 0.00f }  /* 60 */
};

/* 4) Continuous Charge Current Limit (A) – ×9.6 */
static const float32 kTbl_ChgCont_A[TEMP_NODE_CNT][SOC_NODE_CNT] =
{
/*   0      10      20      30      40      50      60      70      80      90      100 */
 { 1.248f, 1.248f, 1.248f, 1.248f, 1.248f, 1.248f, 1.248f, 0.480f, 0.288f, 0.096f, 0.000f }, /* -30 (0.13*9.6 etc) */
 { 2.400f, 2.400f, 2.400f, 2.400f, 2.400f, 2.400f, 2.400f, 0.768f, 0.480f, 0.288f, 0.000f }, /* -20 */
 { 28.80f,14.40f,14.40f, 4.80f, 4.80f, 2.88f, 2.88f, 0.96f, 0.96f, 0.48f, 0.00f }, /* -10 */
 { 48.00f,28.80f,28.80f, 9.60f, 9.60f, 4.80f, 4.80f, 2.40f, 2.40f, 0.96f, 0.00f }, /*  0 */
 { 72.00f,72.00f,72.00f,33.60f,33.60f,24.00f,24.00f,24.00f,24.00f,24.00f, 0.00f }, /* 10 */
 { 144.00f,144.00f,144.00f,67.20f,67.20f,48.00f,28.80f,28.80f,28.80f,28.80f, 0.00f }, /* 25 */
 { 172.80f,172.80f,172.80f,172.80f,153.60f,115.20f,76.80f,57.60f,57.60f,57.60f, 0.00f }, /* 40 */
 { 48.00f,48.00f,48.00f,48.00f,48.00f,48.00f,28.80f,28.80f,28.80f,28.80f, 0.00f }, /* 50 */
 { 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f }  /* 60 */
};
#endif

#if Kokam100Ah

static float32 clampf(float32 x_f, float32 lo_f, float32 hi_f)
{
    if (x_f < lo_f) return lo_f;
    if (x_f > hi_f) return hi_f;
    return x_f;
}
/* 비균일 Temp 노드에서 (lowIdx, highIdx) 찾기 */
static void findTempBracket(float32 tC_f, Uint16 *pLow_u, Uint16 *pHigh_u, float32 *pBeta_f)
{
    Uint16 i_u;
    float32 t_f;

    t_f = tC_f;

    if (t_f <= kTempNodesC_af[0u])
    {
        *pLow_u = 0u;
        *pHigh_u = 0u;
        *pBeta_f = 0.0f;
        return;
    }
    if (t_f >= kTempNodesC_af[TEMP_NODE_CNT - 1u])
    {
        *pLow_u = (TEMP_NODE_CNT - 1u);
        *pHigh_u = (TEMP_NODE_CNT - 1u);
        *pBeta_f = 0.0f;
        return;
    }

    /* 구간 탐색: kTempNodes[i] <= t < kTempNodes[i+1] */
    i_u = 0u;
    while (i_u < (TEMP_NODE_CNT - 1u))
    {
        if ((t_f >= kTempNodesC_af[i_u]) && (t_f <= kTempNodesC_af[i_u + 1u]))
        {
            float32 tl_f = kTempNodesC_af[i_u];
            float32 th_f = kTempNodesC_af[i_u + 1u];
            float32 denom_f = (th_f - tl_f);

            *pLow_u = i_u;
            *pHigh_u = (i_u + 1u);

            if (denom_f <= 0.0f)
            {
                *pBeta_f = 0.0f;
            }
            else
            {
                *pBeta_f = (t_f - tl_f) / denom_f;
                *pBeta_f = clampf(*pBeta_f, 0.0f, 1.0f);
            }
            return;
        }
        i_u++;
    }

    /* fallback */
    *pLow_u = 0u;
    *pHigh_u = 0u;
    *pBeta_f = 0.0f;
}

/* SOC(0~100)에서 10% 스텝 구간 찾기 */
static void findSocBracket(float32 socPct_f, Uint16 *pLow_u, Uint16 *pHigh_u, float32 *pAlpha_f)
{
    float32 s_f = clampf(socPct_f, 0.0f, 100.0f);
    Uint16 idx_u;

    if (s_f >= 100.0f)
    {
        *pLow_u = (SOC_NODE_CNT - 1u);
        *pHigh_u = (SOC_NODE_CNT - 1u);
        *pAlpha_f = 0.0f;
        return;
    }

    idx_u = (Uint16)(s_f / 10.0f); /* 0~9 */
    if (idx_u >= (SOC_NODE_CNT - 1u)) idx_u = (SOC_NODE_CNT - 2u);

    *pLow_u = idx_u;
    *pHigh_u = (idx_u + 1u);

    *pAlpha_f = (s_f - kSocNodesPct_af[idx_u]) / (kSocNodesPct_af[idx_u + 1u] - kSocNodesPct_af[idx_u]);
    *pAlpha_f = clampf(*pAlpha_f, 0.0f, 1.0f);
}

/* 2D 쌍선형 보간 */
static float32 lookup2D_Bilinear(const float32 tbl_af[TEMP_NODE_CNT][SOC_NODE_CNT], float32 tempC_f, float32 socPct_f)
{
    Uint16 tL_u, tH_u;
    Uint16 sL_u, sH_u;
    float32 beta_f, alpha_f;

    float32 I00_f, I01_f, I10_f, I11_f;
    float32 I0_f, I1_f, I_f;

    findTempBracket(tempC_f, &tL_u, &tH_u, &beta_f);
    findSocBracket(socPct_f, &sL_u, &sH_u, &alpha_f);

    I00_f = tbl_af[tL_u][sL_u];
    I01_f = tbl_af[tL_u][sH_u];
    I10_f = tbl_af[tH_u][sL_u];
    I11_f = tbl_af[tH_u][sH_u];

    /* SOC 방향 보간 */
    I0_f = I00_f + alpha_f * (I01_f - I00_f);
    I1_f = I10_f + alpha_f * (I11_f - I10_f);

    /* Temp 방향 보간 */
    I_f = I0_f + beta_f * (I1_f - I0_f);

    return I_f;
}

/* ========== 공개 API 구현 ========== */
float32 GetDischargeLimit30s_A(float32 tempC_f, float32 socPct_f)
{
    return lookup2D_Bilinear(kTbl_Dis30s_A, tempC_f, socPct_f);
}

float32 GetDischargeLimitCont_A(float32 tempC_f, float32 socPct_f)
{
    return lookup2D_Bilinear(kTbl_DisCont_A, tempC_f, socPct_f);
}

float32 GetChargeLimit10s_A(float32 tempC_f, float32 socPct_f)
{
    return lookup2D_Bilinear(kTbl_Chg10s_A, tempC_f, socPct_f);
}

float32 GetChargeLimitCont_A(float32 tempC_f, float32 socPct_f)
{
    return lookup2D_Bilinear(kTbl_ChgCont_A, tempC_f, socPct_f);
}
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
