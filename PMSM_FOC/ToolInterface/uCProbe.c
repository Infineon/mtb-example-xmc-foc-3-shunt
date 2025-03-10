/**
 * @file pmsm_foc_ucProbe.c
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
 * @endcond
 ***********************************************************************************************************************/
 /*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "xmc1_flash.h"
#include "uCProbe.h"
#include "ProbeScope/cfg/probe_scope_cfg.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../ControlModules/pmsm_foc_state_machine.h"
#include "../MCUInit/6EDL_gateway.h"

#if(USER_UCPROBE_GUI == ENABLED)
/*********************************************************************************************************************
 * DEFINATIONS
 ********************************************************************************************************************/
#define SPEED_RPM_LPF                (4U)
/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
volatile uint32_t ucProbe_cmd;
extern PI_CONFIG_t PI_CONFIG;
extern PMSM_FOC_CTRL_t PMSM_FOC_CTRL;

/*********************************************************************************************************************
 * Local DATA
 ********************************************************************************************************************/
static uint32_t   Speed_RPM;                    /* uC_Probe variable */
static uint32_t   Speed_RPM_LPF;                /* uC_Probe variable */

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

/***********************************************************************************************************************/
void PMSM_FOC_uCProbe_Init(void)
{
  /* uC Probe initialization */
  #if (USER_UCPROBE_OSCILLOSCOPE == ENABLED)
  ProbeScope_Init(PROBE_SCOPE_SAMPLING_CLK_HZ);
  #endif
  if ((*PMSM_SL_FOC_PI_CONFIG_ADDR)== PMSM_SL_FOC_UCPROBE_FLASH_VALID_ID)
  {
  }
  else
  {
  }
}

/***********************************************************************************************************************/
void PMSM_FOC_ucProbe_ReadFlash(void)
{
    uint8_t *pi_conf_ptr = (uint8_t *)&PI_CONFIG;
    uint8_t *pi_flash_ptr = (uint8_t *)PMSM_SL_FOC_PI_CONFIG_ADDR;
    for (int32_t i=0; i<PI_CONFIG_SIZE; i++)
    {
        *pi_conf_ptr = *pi_flash_ptr;
        pi_conf_ptr++;
        pi_flash_ptr++;
    }
}

/***********************************************************************************************************************/
void PMSM_FOC_ucProbe_WriteFlash(void)
{
    PI_CONFIG.CONFIG_VALID_ID = PMSM_SL_FOC_UCPROBE_FLASH_VALID_ID;
  /* Write only when motor is in STOP state */
  if (PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_STOP_MOTOR)
  {
    /*Erase and write 256 byte of data*/
    XMC_FLASH_ProgramVerifyPage(motor_PI_config_addr, (uint32_t *)&PI_CONFIG); /*Address, data*/
  }
}

/***********************************************************************************************************************/
/* This function process the commands received cmd from the GUI  and updates FOC input configurations */
void PMSM_FOC_ucProbe_CmdProcessing(void)
{
  /* uC Probe Processing Handling */
  /* uC Speed Display */
  Speed_RPM = ((PMSM_FOC_OUTPUT.rotor_speed * MotorParam.CONVERT_SPEED_TO_RPM ) >> SPEED_TO_RPM_SCALE);
  Speed_RPM_LPF = (Speed_RPM_LPF * ((1 << SPEED_RPM_LPF) - 1) + Speed_RPM) >> SPEED_RPM_LPF;

  switch (ucProbe_cmd)
  {
   case UCPROBE_CMD_CLEAR_ERROR:  /*Clear Error state */
        /* STOP motor */
        MotorVar.fault_clear = 1;
        /* Command is processed */
        ucProbe_cmd = 0;
//        PMSM_FOC_MotorStart();  /* cannot use MotorStart() to reset error because this function check PMSM_FOC_CTRL.msm_state != error, set pot to zero will clear the error status */
//        PMSM_FOC_CTRL.motor_start_flag = 1;
        break;
   case UCPROBE_CMD_WRITE_USERPARAM_FLASH:  /* Program parameter RAM value into flash */
      /* STOP motor */
      PMSM_FOC_MotorStop();
      /* Write User Parameters to FLASH */
      /* Disable CCU8 interrupt to enable update of user parameter */
      NVIC_DisableIRQ(CCU80_0_IRQn);
      /* comment out below code if using with BPA infineon toolbox */
//      PI_CONFIG.PI_PLL_KP = PMSM_FOC_PLL_PI.kp;
//      PI_CONFIG.PI_PLL_KI = PMSM_FOC_PLL_PI.ki;
//      PI_CONFIG.PI_PLL_SCALE = PMSM_FOC_PLL_PI.scale_kp_ki;
//
//      PI_CONFIG.PI_FLUX_KP = PMSM_FOC_FLUX_PI.kp;
//      PI_CONFIG.PI_FLUX_KI = PMSM_FOC_FLUX_PI.ki;
//      PI_CONFIG.PI_FLUX_SCALE = PMSM_FOC_FLUX_PI.scale_kp_ki;
//      PI_CONFIG.PI_FLUX_IK_LIMIT_MAX = (1<<(30 - PI_CONFIG.PI_FLUX_SCALE));       /* (1<<30 - scale). I[k] output limit HIGH. Normally no need change. */
//      PI_CONFIG.PI_FLUX_IK_LIMIT_MIN = (-(1<<(30 - PI_CONFIG.PI_FLUX_SCALE)));    /* (-(1<<30)). I[k] output limit LOW. Normally no need change. */
//
//      PI_CONFIG.PI_TORQUE_KP = PMSM_FOC_TORQUE_PI.kp;
//      PI_CONFIG.PI_TORQUE_KI = PMSM_FOC_TORQUE_PI.ki;
//      PI_CONFIG.PI_TORQUE_SCALE = PMSM_FOC_TORQUE_PI.scale_kp_ki;
//      PI_CONFIG.PI_TORQUE_IK_LIMIT_MAX = (1<<(30 - PI_CONFIG.PI_TORQUE_SCALE));       /* (1<<30 - scale). I[k] output limit HIGH. Normally no need change. */
//      PI_CONFIG.PI_TORQUE_IK_LIMIT_MIN = (-(1<<(30 - PI_CONFIG.PI_TORQUE_SCALE)));    /* (-(1<<30)). I[k] output limit LOW. Normally no need change. */
//
//      PI_CONFIG.PI_SPEED_KP = PMSM_FOC_SPEED_PI.kp;
//      PI_CONFIG.PI_SPEED_KI = PMSM_FOC_SPEED_PI.ki;
//      PI_CONFIG.PI_SPEED_SCALE = PMSM_FOC_SPEED_PI.scale_kp_ki;
      /* comment out above code if using with BPA infineon toolbox */
      /*Erase and write 256 byte of data from MotorParam, PI_CONFIG and Edl7141Reg */
      XMC_FLASH_ProgramVerifyPage(ParameterBlock_Addr, (uint32_t *)&MotorParam); /*Address, data*/
      XMC_FLASH_ProgramVerifyPage(motor_PI_config_addr, (uint32_t *)&PI_CONFIG); /*Address, data*/
#if (MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
      XMC_FLASH_ProgramVerifyPage(Edl7151ParameterBlock_Addr, (uint32_t *)&Edl7151Reg); /*Address, data*/
#elif (MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7141_FOC_3SH)
      XMC_FLASH_ProgramVerifyPage(Edl7141ParameterBlock_Addr, (uint32_t *)&Edl7141Reg); /*Address, data*/
#endif
      /* Enable IRQ */
      NVIC_EnableIRQ(CCU80_0_IRQn);
      /* Command is processed */
      ucProbe_cmd = 0;
      PMSM_FOC_MotorStart();
      break;

   case UCPROBE_CMD_ERASE_USERPARAM_FLASH:  /* Erase parameter block in the flash */
     /* STOP motor */
     PMSM_FOC_MotorStop();
     /* Disable CCU8 interrupt to enable update of user parameter */
     NVIC_DisableIRQ(CCU80_0_IRQn);
     /*Erase 256 byte of data*/
     XMC_FLASH_ErasePage(ParameterBlock_Addr);
     XMC_FLASH_ErasePage(motor_PI_config_addr);
#if (MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
     XMC_FLASH_ErasePage(Edl7151ParameterBlock_Addr);
#elif (MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7141_FOC_3SH)
     XMC_FLASH_ErasePage(Edl7141ParameterBlock_Addr);
#endif
     /* Enable IRQ */
     NVIC_EnableIRQ(CCU80_0_IRQn);
     /* Command is processed */
     ucProbe_cmd = 0;
     break;

   case UCPROBE_CMD_LOAD_DFLT_PARAM:  /* load default value of MotorParam, PI parameter and 6edl7141 regs to Flash */
     /* configure MotorParam parameters*/
     MOTOR_PARAM_set_default();
      /* configure PI parameters */
      PI_set_default();
      SystemVar.ParamConfigured = 1;
      /* Configure 6edl7141 regs with default parameter */
#if (MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
      EDL7151_MOTOR_PARAM_set_default();
      SystemVar.Edl7151Configured = 1;
#elif (MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7141_FOC_3SH)
      EDL7141_MOTOR_PARAM_set_default();
      SystemVar.Edl7141Configured = 1;
#endif
     break;


    case UCPROBE_CMD_READ_USERPARAM_FLASH:  /* Reload motor parameter RAM value from flash */
      /* STOP motor */
      PMSM_FOC_MotorStop();
      /* Read User Parameters from FLASH */
      /* Disable CCU8 interrupt to enable update of user parameter */
      NVIC_DisableIRQ(CCU80_0_IRQn);
      /* Update to flash */
      FLASH_Parameter_load();
      SystemVar.ReInitParam = 1;   /* request to re-init peripheral because key parameters are changed */
      /* Enable IRQ */
      NVIC_EnableIRQ(CCU80_0_IRQn);
      /* Command is processed */
      ucProbe_cmd = 0;
      PMSM_FOC_MotorStart();
      break;

    case UCPROBE_CMD_RESET:
      /* Reset MCU */
      NVIC_SystemReset();
      break;

    case UCPROBE_CMD_WRITE_DFLT_PIPARAM_FLASH:
      /* STOP motor */
      PMSM_FOC_MotorStop();
      /* Update Default PI Values in Flash with user configured PI Values. */
      /* Disable CCU8 interrupt to enable update of user parameter */
      NVIC_DisableIRQ(CCU80_0_IRQn);

      PI_CONFIG.PI_PLL_KP = PMSM_FOC_PLL_PI.kp;
      PI_CONFIG.PI_PLL_KI = PMSM_FOC_PLL_PI.ki;
      PI_CONFIG.PI_PLL_SCALE = PMSM_FOC_PLL_PI.scale_kp_ki;

      PI_CONFIG.PI_FLUX_KP = PMSM_FOC_FLUX_PI.kp;
      PI_CONFIG.PI_FLUX_KI = PMSM_FOC_FLUX_PI.ki;
      PI_CONFIG.PI_FLUX_SCALE = PMSM_FOC_FLUX_PI.scale_kp_ki;

      PI_CONFIG.PI_TORQUE_KP = PMSM_FOC_TORQUE_PI.kp;
      PI_CONFIG.PI_TORQUE_KI = PMSM_FOC_TORQUE_PI.ki;
      PI_CONFIG.PI_TORQUE_SCALE = PMSM_FOC_TORQUE_PI.scale_kp_ki;

      PI_CONFIG.PI_SPEED_KP = PMSM_FOC_SPEED_PI.kp;
      PI_CONFIG.PI_SPEED_KI = PMSM_FOC_SPEED_PI.ki;
      PI_CONFIG.PI_SPEED_SCALE = PMSM_FOC_SPEED_PI.scale_kp_ki;

      /* Update to flash */
      PMSM_FOC_ucProbe_WriteFlash();
      /* Enable IRQ */
      NVIC_EnableIRQ(CCU80_0_IRQn);
      /* Command is processed */
      ucProbe_cmd = 0;
      PMSM_FOC_MotorStart();
      break;

    case UCPROBE_CMD_READ_DFLT_PIPARAM_FLASH:
      /* STOP motor */
      PMSM_FOC_MotorStop();
      /* Restore configured PI values to default PI values */
      /* Disable CCU8 interrupt to enable update of user parameter */
      NVIC_DisableIRQ(CCU80_0_IRQn);

      /* Update to flash */
      PMSM_FOC_ucProbe_ReadFlash();

      PMSM_FOC_SPEED_PI.kp = PI_CONFIG.PI_SPEED_KP;
      PMSM_FOC_SPEED_PI.ki = PI_CONFIG.PI_SPEED_KI;
      PMSM_FOC_SPEED_PI.scale_kp_ki = PI_CONFIG.PI_SPEED_SCALE;

      PMSM_FOC_TORQUE_PI.kp = PI_CONFIG.PI_TORQUE_KP;
      PMSM_FOC_TORQUE_PI.ki = PI_CONFIG.PI_TORQUE_KI;
      PMSM_FOC_TORQUE_PI.scale_kp_ki = PI_CONFIG.PI_TORQUE_SCALE;

      PMSM_FOC_FLUX_PI.kp = PI_CONFIG.PI_FLUX_KP;
      PMSM_FOC_FLUX_PI.ki = PI_CONFIG.PI_FLUX_KI;
      PMSM_FOC_FLUX_PI.scale_kp_ki = PI_CONFIG.PI_FLUX_SCALE;

      PMSM_FOC_PLL_PI.kp = PI_CONFIG.PI_PLL_KP;
      PMSM_FOC_PLL_PI.ki = PI_CONFIG.PI_PLL_KI;
      PMSM_FOC_PLL_PI.scale_kp_ki = PI_CONFIG.PI_PLL_SCALE;

      /* Enable IRQ */
      NVIC_EnableIRQ(CCU80_0_IRQn);
      /* Command is processed */
      ucProbe_cmd = 0;
      PMSM_FOC_MotorStart();
      break;

    case UCPROBE_CMD_MOTOR_STOP:
      /* set a CTRAP to stop motor*/
      CCU8_SLICE_PHASE_U->INS2 &= (0xFFFFFBFF); /* to set CTRAP trigger level EV2LM to active high bit24 */
      CCU8_SLICE_PHASE_V->INS2 &= (0xFFFFFBFF); /* to set CTRAP trigger level EV2LM to active high bit24 */
      CCU8_SLICE_PHASE_W->INS2 &= (0xFFFFFBFF); /* to set CTRAP trigger level EV2LM to active hign bit24 */
      PMSM_FOC_CTRL.motor_start_flag = 0;
      PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
      break;

    default:
      break;
  }
}

#endif  /* #if(USER_UCPROBE_GUI == ENABLED) */
