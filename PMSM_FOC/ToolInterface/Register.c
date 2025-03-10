/**
 * @file Register.c
 *
 * @cond
 *********************************************************************************************************************
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *@endcond
 ***********************************************************************************************************************/

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "Register.h"


#include "../Configuration/pmsm_foc_gatedrv_config.h"
#include "../MCUInit/6EDL_gateway.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../Configuration/pmsm_foc_common.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "uCProbe.h"
#include "xmc1_flash.h"
#include "../ToolInterface/Register.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
PI_CONFIG_t PI_CONFIG;

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/
MC_INFO_t MC_INFO;          /* MC_INFO data structure */
SYSTEM_VAR_t SystemVar;
MOTOR_VAR_t MotorVar;
MOTOR_PAR_t MotorParam;
volatile int32_t Test1, Test2, Test3, Test4;


/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
void MC_Info_init(void)
{
    MC_INFO.chip_id = SCU_GENERAL->IDCHIP;
    MC_INFO.parameter_version = PARAMETER_VERSION;
    MC_INFO.firmware_version = FIRMWARE_VERSION;
    MC_INFO.kit_id = read_kit_id();        /* Hardware kit ID, initialize with 0 (Unidentified). After power up, firmware will read analog input to get actual kit id */
}


void MOTOR_PARAM_set_default(void)
{
  MotorParam.ParamVersion = PARAMETER_VERSION;

  /*********************************************************************************************************************
   * Hardware group
   ********************************************************************************************************************/
  /* HwConfig */
#if (XMC_ON_CHIP_GAIN == X1)
  MotorParam.InternalGain = 0;
#elif (XMC_ON_CHIP_GAIN == X3)
  MotorParam.InternalGain = 1;
#elif (XMC_ON_CHIP_GAIN == X6)
  MotorParam.InternalGain = 2;
#else //if (XMC_ON_CHIP_GAIN == X12)
  MotorParam.InternalGain = 3;
#endif

  /*********************************************************************************************************************
   * System group
   ********************************************************************************************************************/
  /* SysConfig */
  MotorParam.AnalogControl = USER_REF_SETTING;
  MotorParam.ControlScheme = USER_FOC_CTRL_SCHEME;
  MotorParam.BiDirectional = USER_MOTOR_BI_DIRECTION_CTRL;
  MotorParam.SvmScheme = USER_SVM_SWITCHING_SCHEME;
  MotorParam.SwapDirection = 0;
  MotorParam.PwmModulationType = 0;
  MotorParam.DcBusCompensation = 0;
  MotorParam.StartupMethod = USER_MOTOR_STARTUP_METHOD;
  MotorParam.InitPosDetect = USER_ROTOR_PRE_ALIGN_METHOD;
  MotorParam.Vadc_Ref_Voltage = USER_MAX_ADC_VDD_V * 10.0F;

  /*********************************************************************************************************************
   * Protection group
   ********************************************************************************************************************/
  /* EnableFault */
  MotorParam.EnableFault.Trap = 0;
  MotorParam.EnableFault.OverCurrent = 0;
  MotorParam.EnableFault.ShortCircuit = 0;
  MotorParam.EnableFault.TorqueOverLimit = 0;
  MotorParam.EnableFault.TempOverLimit = 0;
  MotorParam.EnableFault.Stall = 0;
  MotorParam.EnableFault.OverVoltage = USER_VDC_UV_OV_PROTECTION;
  MotorParam.EnableFault.FreeRunning = 0;
  MotorParam.EnableFault.FreeRevRunning = 0;
  MotorParam.EnableFault.StartupFailure = 0;
  MotorParam.EnableFault.UnderVoltage = USER_VDC_UV_OV_PROTECTION;
  MotorParam.EnableFault.SpiError = 0;
  MotorParam.EnableFault.Nfault = 1;
  MotorParam.EnableFault.OffState = 1;

  /*********************************************************************************************************************
   * Motor group
   ********************************************************************************************************************/
#if defined (__ICCARM__)
#pragma diag_suppress=Pa092
#endif

#if defined (__ICCARM__)
#pragma diag_suppress=Pa093
#endif
  MotorParam.G_OPAMP_PER_PHASECURRENT = ((uint32_t) USER_CURRENT_AMPLIFIER_GAIN);
  MotorParam.I_MAX_A = ((USER_MAX_ADC_VDD_V / (USER_R_SHUNT_OHM * USER_CURRENT_AMPLIFIER_GAIN)) / 2U);
  MotorParam.I_MIN_A = (1.0f); /* Minimum current which can be sensed by adc */
  MotorParam.CCU8_PERIOD_REG_T = ((uint32_t) ((USER_PCLK_FREQ_MHz * 1000000) / USER_CCU8_PWM_FREQ_HZ) - 1);
  MotorParam.CCU4_PERIOD_REG = ((uint32_t) ((USER_PCLK_FREQ_MHz * 1000) / USER_CCU4_DEBUG_KHZ) - 1);
  MotorParam.DRIVERIC_DELAY = (uint32_t) ((USER_DRIVERIC_DELAY_US * USER_PCLK_FREQ_MHz) - 0.5);
  MotorParam.SPEED_REF_HIGH_LIMIT_RPM = (uint32_t) USER_SPEED_REF_HIGH_LIMIT_RPM;
  MotorParam.SPEED_REF_LOW_LIMIT_RPM = (uint32_t) USER_SPEED_REF_LOW_LIMIT_RPM;
  MotorParam.SPEED_REF_HIGH_LIMIT_TS = (uint32_t)((1<<15)-1);
  MotorParam.SPEED_REF_LOW_LIMIT_TS = (uint32_t)((float)USER_SPEED_LOW_LIMIT_RPM / USER_SPEED_HIGH_LIMIT_RPM * ((1<<15)-1));
  MotorParam.CCU8_DEADTIME_RISE_T = (uint32_t) ((USER_DEAD_TIME_US * USER_PCLK_FREQ_MHz) - 0.5);
  MotorParam.CCU8_DEADTIME_FALL_T = (uint32_t) ((USER_DEAD_TIME_US * USER_PCLK_FREQ_MHz) - 0.5);
  MotorParam.PMSM_FOC_SYSTICK_COUNT_T = (USER_MCLK_FREQ_MHz * USER_SLOW_CTRL_LOOP_PERIOD_uS);
  MotorParam.IDC_OVER_CURRENT_LIMIT = (uint32_t) (4096 * USER_IDC_OVER_CURRENT_LIMIT_A / MotorParam.I_MAX_A);
  MotorParam.MOTOR_BRAKE_DUTY_VAL = (uint32_t) ((USER_MOTOR_BRAKE_DUTY * (((USER_PCLK_FREQ_MHz * 1000000) / USER_CCU8_PWM_FREQ_HZ) - 1)) / 100);
  MotorParam.PWM_PERIOD_TS_US = (1000000.0f / USER_CCU8_PWM_FREQ_HZ);
  MotorParam.BOOTSTRAP_BRAKE_TIME = (uint32_t) ((USER_BOOTSTRAP_PRECHARGE_TIME_MS * 1000) / (MotorParam.PWM_PERIOD_TS_US));
  MotorParam.STARTUP_VF_TRANSITION_SPEED = (uint32_t) (float)(USER_VF_TRANSITION_SPEED_RPM * 32767 / (float)USER_SPEED_HIGH_LIMIT_RPM);
  MotorParam.N_VREF_SVM_V = (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT);
  MotorParam.STARTUP_VF_OFFSET = (uint32_t) (USER_VF_OFFSET_V * 32767.0f * USER_SQRT_3_CONSTANT /USER_VDC_LINK_V);

  MotorParam.STARTUP_VF_V_PER_HZ_CONST = (uint32_t) (USER_STARTUP_VF_V_PER_HZ_CONST > 1.0f? USER_STARTUP_VF_V_PER_HZ_CONST: 1.0f);
  MotorParam.STARTUP_VF_SPEED_RAMP_UP_RATE = (uint32_t) (USER_VF_SPEED_RAMPUP_SLEWRATE);
  MotorParam.STARTUP_VF_SPEED_RAMP_UP_STEP = (uint32_t) ((float)USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S * 32767 * USER_VF_SPEED_RAMPUP_SLEWRATE/((float)USER_SPEED_HIGH_LIMIT_RPM * USER_CCU8_PWM_FREQ_HZ));
  MotorParam.STARTUP_VF_STABILIZATION_COUNT = (750U); /* After V/F ramp up Motor will wait this time before go to close loop */
  MotorParam.VQ_INITIAL_VALUE = (uint32_t) (32767U * USER_VQ_INITIAL_VALUE_V / (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)); /* Vq value during startup */
  MotorParam.ROTOR_PRE_ALIGNMENT_VREF = (uint32_t) ((USER_ROTOR_PRE_ALIGNMENT_VOLTAGE_V * 32767.0f) / (MotorParam.N_VREF_SVM_V));
  MotorParam.ROTOR_PRE_ALIGNMENT_COUNT = ((USER_ROTOR_PRE_ALIGNMENT_TIME_MS * 1000.0F) / MotorParam.PWM_PERIOD_TS_US);
  MotorParam.ROTOR_PRE_ALIGNMENT_RAMP_RATE = (((USER_ROTOR_PRE_ALIGNMENT_V_RAMP_RATE * MotorParam.PWM_PERIOD_TS_US) / 1000000.0F) / USER_VDC_LINK_V) * 32767.0f;
  MotorParam.BASE_VOLTAGE = (uint32_t) USER_VDC_LINK_V;
  MotorParam.VADC_DCLINK_T = (uint32_t) (((USER_VDC_LINK_V * USER_VDC_LINK_DIVIDER_RATIO) / USER_MAX_ADC_VDD_V) * (1 << 12));
  MotorParam.BRAKING_VDC_MAX_LIMIT = ((uint16_t) ((MotorParam.VADC_DCLINK_T * USER_BRAKING_VDC_MAX_LIMIT) / 100)); /* only for braking usage, voltage clamping */
  MotorParam.ELECTRICAL_SPEED_LOW_LIMIT_TS = (uint32_t)((float)USER_SPEED_LOW_LIMIT_RPM / USER_SPEED_HIGH_LIMIT_RPM * ((1<<15)-1));
  MotorParam.ELECTRICAL_SPEED_HIGH_LIMIT_TS = (uint32_t)((1<<15)-1);
  MotorParam.ELECTRICAL_SPEED_FREQ_HZ = ((float) USER_SPEED_HIGH_LIMIT_RPM * USER_MOTOR_POLE_PAIR / 60);
  MotorParam.VQ_REF_HIGH_LIMIT = (uint32_t) (32767U * USER_VQ_REF_HIGH_LIMIT_V / (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT)); /* Max Limit for Vq reference */
  MotorParam.VQ_REF_LOW_LIMIT = (uint32_t) (32767U * USER_VQ_REF_LOW_LIMIT_V / (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT));   /* Min Limit for Vq reference */
  MotorParam.RAMP_UP_SPEED = (uint32_t)(SPEED_RAMP_UP_STEP);
  MotorParam.RAMP_DOWN_SPEED = (uint32_t)(SPEED_RAMP_DOWN_STEP);
  MotorParam.SPEED_RAMP_SLEWRATE = USER_SPEED_RAMP_SLEWRATE;
  MotorParam.SPEED_TO_ANGLE_CONV_RAW = (uint32_t)((N_ESPEED_RAD_FCL/N_ESPEED_T)*(N_EROTORANGLE_T/N_EROTORANGLE_RAD));
  MotorParam.DEFAULT_SCALE_OF_L =
      (uint32_t) MIN(15-ROUNDUP(log2(1.5f * N_ESPEED_RAD_FCL * USER_CCU8_PWM_FREQ_HZ * N_I_A / N_V_V * USER_MOTOR_LS_PER_PHASE_uH / 1000000 )),15);
  MotorParam.DEFAULT_LS_SCALEDUP = ROUND(1.5f * N_ESPEED_RAD_FCL * USER_CCU8_PWM_FREQ_HZ * N_I_A / N_V_V * USER_MOTOR_LS_PER_PHASE_uH / 1000000*(1<<MotorParam.DEFAULT_SCALE_OF_L));
  MotorParam.USER_CURRENTCTRL_CUTOFF_FREQ_HZ = (MotorParam.ELECTRICAL_SPEED_FREQ_HZ);
  MotorParam.SPEED_TO_ANGLE_CONV_FACTOR_SCALE = ROUND(MIN((float)(14 - (int16_t)log2(MotorParam.SPEED_TO_ANGLE_CONV_RAW)),14));

#if defined (__ICCARM__)
#pragma diag_suppress=Pe186
#endif
    MotorParam.SPEED_TO_ANGLE_CONV_FACTOR = ROUND(MotorParam.SPEED_TO_ANGLE_CONV_RAW*(1<<MotorParam.SPEED_TO_ANGLE_CONV_FACTOR_SCALE));
#if defined (__ICCARM__)
#pragma diag_default=Pe186
#endif

  MotorParam.CONVERT_SPEED_TO_RPM = (uint32_t) ((USER_SPEED_HIGH_LIMIT_RPM * 2048.0f) / MotorParam.ELECTRICAL_SPEED_HIGH_LIMIT_TS);
  MotorParam.SVM_LUTTABLE_SCALE = ((uint32_t) ((((((USER_PCLK_FREQ_MHz * 1000000) / USER_CCU8_PWM_FREQ_HZ) - 1) * 1.0f / MAX_VREF_AMPLITUDE) * 32767.0f)
      * (1 << USER_SCALE_UP_SINE_LUT)));
  MotorParam.KS_SCALE_SVM = ((((USER_PCLK_FREQ_MHz * 1000000) / USER_CCU8_PWM_FREQ_HZ) - 1) / MAX_VREF_AMPLITUDE);
  MotorParam.PMSM_FOC_CCU8_SYNC_START = ((uint32_t) 1 << (uint32_t) (8U + CCU8_MODULE_NUM));

  MotorParam.UNDERVOLTAGE_THRESHOLD = ((uint32_t) (((USER_VDC_LINK_V * USER_VDC_LINK_DIVIDER_RATIO) / USER_MAX_ADC_VDD_V) * (1 << 12) * 60.0 / 100.0));
  MotorParam.OVERVOLTAGE_THRESHOLD = ((uint32_t) (((USER_VDC_LINK_V * USER_VDC_LINK_DIVIDER_RATIO) / USER_MAX_ADC_VDD_V) * (1 << 12) * 140.0 / 100.0));
  MotorParam.DQ_DECOUPLING = (uint32_t) (USER_DQ_DECOUPLING);

#if defined (__ICCARM__)
#pragma diag_default=Pa093
#endif

#if defined (__ICCARM__)
#pragma diag_default=Pa092
#endif
}

/*************************************************************************************************************************************
 *  User configuration initialization before motor start
 * ***********************************************************************************************************************************/
void PI_set_default(void)
{
  /* Speed PI Controller */
  PI_CONFIG.PI_SPEED_KP = USER_PI_SPEED_KP;
  PI_CONFIG.PI_SPEED_KI = USER_PI_SPEED_KI;
  PI_CONFIG.PI_SPEED_SCALE = USER_PI_SPEED_SCALE_KPKI;
  PI_CONFIG.PI_SPEED_IK_LIMIT_MAX = (1<<(30 - 0));       /* (1<<30). I[k] output limit HIGH. */
  PI_CONFIG.PI_SPEED_IK_LIMIT_MIN = (-(1<<(30 - 0)));    /* (-(1<<30 - scale)). I[k] output limit LOW. */
  PI_CONFIG.PI_SPEED_UK_LIMIT_MAX = USER_PI_SPEED_UK_LIMIT_MAX;
  PI_CONFIG.PI_SPEED_UK_LIMIT_MIN = USER_PI_SPEED_UK_LIMIT_MIN;
  
  /* Flux PI Controller */
  PI_CONFIG.PI_FLUX_SCALE = MIN((13-ROUNDUP((log2((USER_PI_FLUX_KP_BW*N_I_A/N_V_V))))),13);
  PI_CONFIG.PI_FLUX_KP = ROUND(USER_PI_FLUX_KP_BW*N_I_A/N_V_V*(1<<PI_CONFIG.PI_FLUX_SCALE));
  PI_CONFIG.PI_FLUX_KI = ROUND(USER_PI_FLUX_KI_BW*N_I_A/N_V_V*(1<<PI_CONFIG.PI_FLUX_SCALE));

  PI_CONFIG.PI_FLUX_IK_LIMIT_MAX = (1<<(30 - PI_CONFIG.PI_FLUX_SCALE));       /* (1<<30 - scale). I[k] output limit HIGH. Normally no need change. */
  PI_CONFIG.PI_FLUX_IK_LIMIT_MIN = (-(1<<(30 - PI_CONFIG.PI_FLUX_SCALE)));    /* (-(1<<30)). I[k] output limit LOW. Normally no need change. */
  PI_CONFIG.PI_FLUX_UK_LIMIT_MAX = USER_PI_FLUX_UK_LIMIT_MAX;
  PI_CONFIG.PI_FLUX_UK_LIMIT_MIN = USER_PI_FLUX_UK_LIMIT_MIN;

  /* Torque PI Controller */
  PI_CONFIG.PI_TORQUE_SCALE = PI_CONFIG.PI_FLUX_SCALE;
  PI_CONFIG.PI_TORQUE_KP = PI_CONFIG.PI_FLUX_KP;
  PI_CONFIG.PI_TORQUE_KI =  PI_CONFIG.PI_FLUX_KI;

  PI_CONFIG.PI_TORQUE_IK_LIMIT_MAX = (1<<(30 - PI_CONFIG.PI_TORQUE_SCALE));       /* (1<<30 - scale). I[k] output limit HIGH. Normally no need change. */
  PI_CONFIG.PI_TORQUE_IK_LIMIT_MIN = (-(1<<(30 - PI_CONFIG.PI_TORQUE_SCALE)));    /* (-(1<<30)). I[k] output limit LOW. Normally no need change. */
  PI_CONFIG.PI_TORQUE_UK_LIMIT_MAX = USER_PI_TORQUE_UK_LIMIT_MAX;
  PI_CONFIG.PI_TORQUE_UK_LIMIT_MIN = USER_PI_TORQUE_UK_LIMIT_MIN;

  /* PLL Observer PI Controller */
  PI_CONFIG.PI_PLL_KP = USER_PI_PLL_KP;
  PI_CONFIG.PI_PLL_KI = USER_PI_PLL_KI;
  PI_CONFIG.PI_PLL_SCALE = USER_PI_PLL_SCALE_KPKI;
  PI_CONFIG.PI_PLL_IK_LIMIT_MAX = USER_PI_PLL_IK_LIMIT_MAX;          /* I[k] output limit HIGH. */
  PI_CONFIG.PI_PLL_IK_LIMIT_MIN = USER_PI_PLL_IK_LIMIT_MIN;          /* I[k] output limit LOW. */  PI_CONFIG.PI_PLL_UK_LIMIT_MAX = MotorParam.ELECTRICAL_SPEED_HIGH_LIMIT_TS+3000;
  PI_CONFIG.PI_PLL_UK_LIMIT_MAX = MotorParam.ELECTRICAL_SPEED_HIGH_LIMIT_TS+3000;
  PI_CONFIG.PI_PLL_UK_LIMIT_MIN = MotorParam.ELECTRICAL_SPEED_LOW_LIMIT_TS>>1;

}

void FLASH_Parameter_load(void)
{
    /* MC_INFO structure contains expected parameter version for the firmware. Head of parameter block should contain
     * same value, otherwise parameter block won't be loaded
     */
#if (PROGRAM_DEFAULT_PARAM == 1)
        MOTOR_PARAM_set_default();  /* configure MotorParam parameters*/
        XMC_FLASH_ProgramVerifyPage(ParameterBlock_Addr, (uint32_t *)&MotorParam);

        PI_set_default(); /* configure PI parameters */
        XMC_FLASH_ProgramVerifyPage(motor_PI_config_addr, (uint32_t *)&PI_CONFIG);
        SystemVar.ParamConfigured = 1;
#else
    uint16_t parameter_version = (uint16_t)*ParameterBlock_Addr;
    if (parameter_version != MC_INFO.parameter_version)     /* if there is no valid parameter block, do nothing */
    {
        SystemVar.ParamConfigured = 0;
    }
    else    /* Restore parameter block in flash into MotorParam structure */
    {
        uint8_t *parameter_ptr = (uint8_t *)&MotorParam;
        uint8_t *flash_ptr = (uint8_t *)ParameterBlock_Addr;
        for (int32_t i=0; i<PARAMETER_SIZE; i++)
        {
            *parameter_ptr = *flash_ptr;
            parameter_ptr++;
            flash_ptr++;
        }

        uint8_t *pi_conf_ptr = (uint8_t *)&PI_CONFIG;
        uint8_t *pi_flash_ptr = (uint8_t *)PMSM_SL_FOC_PI_CONFIG_ADDR;
        for (int32_t i=0; i<PI_CONFIG_SIZE; i++)
        {
            *pi_conf_ptr = *pi_flash_ptr;
            pi_conf_ptr++;
            pi_flash_ptr++;
        }
        SystemVar.ParamConfigured = 1;
    }
#endif
}


uint32_t Reserved = 0;
void* const RegisterAddrMap[REGISTER_MAP_SIZE] =
{
/*************************** BLDC motor control registers ***********************/
#if ((EDL7141_CHIP_VERSION == 21) || (VALID_FLASH_DATA == 0x1))
  /*0*/   (void*)&PMSM_FOC_INPUT.ref_speed,
  /*1*/   (void*)&PMSM_FOC_INPUT.i_v,
  /*2*/   (void*)&PMSM_FOC_INPUT.i_w,
  /*3*/   (void*)&PMSM_FOC_INPUT.i_u,
  /*4*/   (void*)&PMSM_FOC_OUTPUT.rotor_speed,  /* Actual rotor speed*/
  /*5*/   (void*)&Reserved,
  /*6*/   (void*)&Reserved,
  /*7*/   (void*)&Reserved,
  /*8*/   (void*)&Reserved,
  /*9*/   (void*)&Reserved,
  /*10*/  (void*)&Reserved,
  /*11*/  (void*)&Reserved,
  /*12*/  (void*)&Reserved,
  /*13*/  (void*)&Reserved,
  /*14*/  (void*)&Reserved,
  /*15*/  (void*)&Reserved,
  /*16*/  (void*)&Reserved,
  /*17*/  (void*)&Reserved,
  /*18*/  (void*)&Reserved,
  /*19*/  (void*)&Reserved,
  /*20*/  (void*)&Reserved,
  /*21*/  (void*)&Reserved,
  /*22*/  (void*)&Reserved,
  /*23*/  (void*)&Reserved,
  /*24*/  (void*)&Reserved,
  /*25*/  (void*)&Reserved,
  /*26*/  (void*)&Reserved,
  /*27*/  (void*)&Reserved,
  /*28*/  (void*)&Reserved,
  /*29*/  (void*)&Reserved,
  /*30*/  (void*)&Reserved,
  /*31*/  (void*)&Reserved,
  /*32*/  (void*)&Reserved,
  /*33*/  (void*)&Reserved,
  /*34*/  (void*)&Reserved,
  /*35*/  (void*)&Reserved,
  /*36*/  (void*)&Reserved,
  /*37*/  (void*)&Reserved,
  /*38*/  (void*)&Reserved,
  /*39*/  (void*)&Reserved,
  /*40*/  (void*)&Reserved,
  /*41*/  (void*)&Reserved,
  /*42*/  (void*)&Reserved,
  /*43*/  (void*)&Reserved,
  /*44*/  (void*)&Reserved,
  /*45*/  (void*)&Reserved,
  /*46*/  (void*)&Test1,
  /*47*/  (void*)&Test2,
  /*48*/  (void*)&Test3,
  /*49*/  (void*)&Test4,
  
  #if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
  /*************************** 6EDL7151 registers ***********************/
  /*50*/  (void*)&Edl7151Reg.FAULT_ST,        /* 0. Read FAULT_ST register */
  /*51*/  (void*)&Edl7151Reg.TEMP_ST,         /* 1. Read TEMP_ST register */
  /*52*/  (void*)&Edl7151Reg.SUPPLY_ST,       /* 2. Read SUPPLY_ST register */
  /*53*/  (void*)&Edl7151Reg.FUNCT_ST,        /* 3. Read FUNCT_ST register */
  /*54*/  (void*)&Edl7151Reg.OTP_ST,          /* 4. Read OTP_ST register */
  /*55*/  (void*)&Edl7151Reg.ADC_ST,          /* 5. Read ADC_ST register */
  /*56*/  (void*)&Edl7151Reg.CP_ST,           /* 6. Read CP_ST register */
  /*57*/  (void*)&Edl7151Reg.DEVICE_ID,       /* 7. Read DEVICE_ID register */
  /*58*/  (void*)&Edl7151Reg.FAULTS_CLR,      /* 0. Read HS_GATE_CFG register */
  /*59*/  (void*)&Edl7151Reg.SUPPLY_CFG,      /* 1. Read SUPPLY_CFG register */
  /*60*/  (void*)&Edl7151Reg.ADC_CFG,         /* 2. Read ADC_CFG register */
  /*61*/  (void*)&Edl7151Reg.PWM_CFG,         /* 3. Read PWM_CFG register */
  /*62*/  (void*)&Edl7151Reg.SENSOR_CFG,      /* 4. Read SENSOR_CFG register */
  /*63*/  (void*)&Edl7151Reg.WD_CFG,          /* 5. Read WD_CFG register */
  /*64*/  (void*)&Edl7151Reg.WD_CFG2,         /* 6. Read WD_CFG2 register */
  /*65*/  (void*)&Edl7151Reg.IDRIVE_CFG,      /* 7. Read IDRIVE_CFG register */
  /*66*/  (void*)&Edl7151Reg.IDRIVE_PRE_CFG,  /* 8. Read IDRIVE_PRE_CFG register */
  /*67*/  (void*)&Edl7151Reg.TDRIVE_SRC_CFG,  /* 9. Read TDRIVE_SRC_CFG register */
  /*68*/  (void*)&Edl7151Reg.TDRIVE_SINK_CFG, /* 10. Read TDRIVE_SINK_CFG register */
  /*69*/  (void*)&Edl7151Reg.DT_CFG,          /* 11. Read DT_CFG register */
  /*70*/  (void*)&Edl7151Reg.CP_CFG,          /* 11. Read CP_CFG register */
  /*71*/  (void*)&Edl7151Reg.CSAMP_CFG,       /* 12. Read CSAMP_CFG register */
  /*72*/  (void*)&Edl7151Reg.CSAMP_CFG2,      /* 13. Read CSAMP_CFG2 register */
  /*73*/  (void*)&Edl7151Reg.OTP_PROG,        /* 14. Read OTP_PROG register */
  
  #else
  /*************************** 6EDL7141 registers ***********************/
  /*50*/  (void*)&Edl7141Reg.FAULT_ST,        /* 0. Read FAULT_ST register */
  /*51*/  (void*)&Edl7141Reg.TEMP_ST,         /* 1. Read TEMP_ST register */
  /*52*/  (void*)&Edl7141Reg.SUPPLY_ST,       /* 2. Read SUPPLY_ST register */
  /*53*/  (void*)&Edl7141Reg.FUNCT_ST,        /* 3. Read FUNCT_ST register */
  /*54*/  (void*)&Edl7141Reg.OTP_ST,          /* 4. Read OTP_ST register */
  /*55*/  (void*)&Edl7141Reg.ADC_ST,          /* 5. Read ADC_ST register */
  /*56*/  (void*)&Edl7141Reg.CP_ST,           /* 6. Read CP_ST register */
  /*57*/  (void*)&Edl7141Reg.DEVICE_ID,       /* 7. Read DEVICE_ID register */
  /*58*/  (void*)&Edl7141Reg.FAULTS_CLR,      /* 0. Read HS_GATE_CFG register */
  /*59*/  (void*)&Edl7141Reg.SUPPLY_CFG,      /* 1. Read SUPPLY_CFG register */
  /*60*/  (void*)&Edl7141Reg.ADC_CFG,         /* 2. Read ADC_CFG register */
  /*61*/  (void*)&Edl7141Reg.PWM_CFG,         /* 3. Read PWM_CFG register */
  /*62*/  (void*)&Edl7141Reg.SENSOR_CFG,      /* 4. Read SENSOR_CFG register */
  /*63*/  (void*)&Edl7141Reg.WD_CFG,          /* 5. Read WD_CFG register */
  /*64*/  (void*)&Edl7141Reg.WD_CFG2,         /* 6. Read WD_CFG2 register */
  /*65*/  (void*)&Edl7141Reg.IDRIVE_CFG,      /* 7. Read IDRIVE_CFG register */
  /*66*/  (void*)&Edl7141Reg.IDRIVE_PRE_CFG,  /* 8. Read IDRIVE_PRE_CFG register */
  /*67*/  (void*)&Edl7141Reg.TDRIVE_SRC_CFG,  /* 9. Read TDRIVE_SRC_CFG register */
  /*68*/  (void*)&Edl7141Reg.TDRIVE_SINK_CFG, /* 10. Read TDRIVE_SINK_CFG register */
  /*69*/  (void*)&Edl7141Reg.DT_CFG,          /* 11. Read DT_CFG register */
  /*70*/  (void*)&Edl7141Reg.CP_CFG,          /* 11. Read CP_CFG register */
  /*71*/  (void*)&Edl7141Reg.CSAMP_CFG,       /* 12. Read CSAMP_CFG register */
  /*72*/  (void*)&Edl7141Reg.CSAMP_CFG2,      /* 13. Read CSAMP_CFG2 register */
  /*73*/  (void*)&Edl7141Reg.OTP_PROG,        /* 14. Read OTP_PROG register */
  #endif /* #if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH) */
  
#endif /* ((EDL7141_CHIP_VERSION == 21) || (VALID_FLASH_DATA == 0x1)) */
};

enum {SCOPE_INT8U, SCOPE_INT8S, SCOPE_INT16U, SCOPE_INT16S, SCOPE_INT32U, SCOPE_INT32S, SCOPE_FP32};
REGISTER_TYPE_MAP_t const RegisterTypeMap[REGISTER_MAP_SIZE] =
{
/*************************** SL FOC motor control registers ***********************/
  /*0*/   {SCOPE_INT32S, DISABLED, 0},
  /*1*/   {SCOPE_INT32U, DISABLED, 0},
  /*2*/   {SCOPE_INT32U, DISABLED, 0},
  /*3*/   {SCOPE_INT32U, DISABLED, 0},
  /*4*/   {SCOPE_INT32S, DISABLED, 0},
  /*5*/   {SCOPE_INT16S, DISABLED, 0},
  /*6*/   {SCOPE_INT32U, DISABLED, 0},
  /*7*/   {SCOPE_INT32U, DISABLED, 0},
  /*8*/   {SCOPE_INT32U, DISABLED, 0},
  /*9*/   {SCOPE_INT32U, DISABLED, 0},
  /*10*/  {SCOPE_INT32U, DISABLED, 0},
  /*11*/  {SCOPE_INT32U, DISABLED, 0},
  /*12*/  {SCOPE_INT32U, DISABLED, 0},
  /*13*/  {SCOPE_INT32U, DISABLED, 0},
  /*14*/  {SCOPE_INT32U, DISABLED, 0},
  /*15*/  {SCOPE_INT32U, DISABLED, 0},
  /*16*/  {SCOPE_INT32U, DISABLED, 0},
  /*17*/  {SCOPE_INT32U, DISABLED, 0},
  /*18*/  {SCOPE_INT32U, DISABLED, 0},
  /*19*/  {SCOPE_INT32U, DISABLED, 0},
  /*20*/  {SCOPE_INT32U, DISABLED, 0},
  /*21*/  {SCOPE_INT32U, DISABLED, 0},
  /*22*/  {SCOPE_INT32U, DISABLED, 0},
  /*23*/  {SCOPE_INT32U, DISABLED, 0},
  /*24*/  {SCOPE_INT32U, DISABLED, 0},
  /*25*/  {SCOPE_INT32U, DISABLED, 0},
  /*26*/  {SCOPE_INT32U, DISABLED, 0},
  /*27*/  {SCOPE_INT32U, DISABLED, 0},
  /*28*/  {SCOPE_INT32U, DISABLED, 0},
  /*29*/  {SCOPE_INT32U, DISABLED, 0},
  /*30*/  {SCOPE_INT32U, DISABLED, 0},
  /*31*/  {SCOPE_INT32U, DISABLED, 0},
  /*32*/  {SCOPE_INT32U, DISABLED, 0},
  /*33*/  {SCOPE_INT32U, DISABLED, 0},
  /*34*/  {SCOPE_INT32U, DISABLED, 0},
  /*35*/  {SCOPE_INT32U, DISABLED, 0},
  /*36*/  {SCOPE_INT32U, DISABLED, 0},
  /*37*/  {SCOPE_INT32U, DISABLED, 0},
  /*38*/  {SCOPE_INT32U, DISABLED, 0},
  /*39*/  {SCOPE_INT32U, DISABLED, 0},
  /*40*/  {SCOPE_INT32U, DISABLED, 0},
  /*41*/  {SCOPE_INT32U, DISABLED, 0},
  /*42*/  {SCOPE_INT32U, DISABLED, 0},
  /*43*/  {SCOPE_INT32U, DISABLED, 0},
  /*44*/  {SCOPE_INT32U, DISABLED, 0},
  /*45*/  {SCOPE_INT32U, DISABLED, 0},
  /*46*/  {SCOPE_INT32U, DISABLED, 0},
  /*47*/  {SCOPE_INT32U, DISABLED, 0},
  /*48*/  {SCOPE_INT32U, DISABLED, 0},
  /*49*/  {SCOPE_INT32U, DISABLED, 0},
  /*50*/  {SCOPE_INT32U, DISABLED, 0},
  /*************************** 6EDL7141 registers ***********************/
  /*51*/  {SCOPE_INT16U, DISABLED, 0},
  /*52*/  {SCOPE_INT16U, DISABLED, 0},
  /*53*/  {SCOPE_INT16U, DISABLED, 0},
  /*54*/  {SCOPE_INT16U, DISABLED, 0},
  /*55*/  {SCOPE_INT16U, DISABLED, 0},
  /*56*/  {SCOPE_INT16U, DISABLED, 0},
  /*57*/  {SCOPE_INT16U, DISABLED, 0},
  /*58*/  {SCOPE_INT16U, DISABLED, 0},
  /*59*/  {SCOPE_INT16U, DISABLED, 0},
  /*60*/  {SCOPE_INT16U, DISABLED, 0},
  /*61*/  {SCOPE_INT16U, DISABLED, 0},
  /*62*/  {SCOPE_INT16U, DISABLED, 0},
  /*63*/  {SCOPE_INT16U, DISABLED, 0},
  /*64*/  {SCOPE_INT16U, DISABLED, 0},
  /*65*/  {SCOPE_INT16U, DISABLED, 0},
  /*66*/  {SCOPE_INT16U, DISABLED, 0},
  /*67*/  {SCOPE_INT16U, DISABLED, 0},
  /*68*/  {SCOPE_INT16U, DISABLED, 0},
  /*69*/  {SCOPE_INT16U, DISABLED, 0},
  /*70*/  {SCOPE_INT16U, DISABLED, 0},
  /*71*/  {SCOPE_INT16U, DISABLED, 0},
  /*72*/  {SCOPE_INT16U, DISABLED, 0},
  /*73*/  {SCOPE_INT16U, DISABLED, 0},
};

