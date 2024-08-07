/**
 * @file pmsm_foc_state_machine.h
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
#ifndef PMSM_FOC_STATE_MACHINE_H_
#define PMSM_FOC_STATE_MACHINE_H_
/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../Configuration/pmsm_foc_common.h"
#include "../ToolInterface/Register.h"

#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_speed_current_ctrl.h"
#include "../ControlModules/pmsm_foc_vq_voltage_ctrl.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"


/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/**
 * @brief This enumerates the motor control state machine.
 */
typedef enum
{
  PMSM_FOC_MSM_CLOSED_LOOP         = (1 << 0), /*!< 0001 - FOC CLOSED LOOP  */
  PMSM_FOC_MSM_BRAKE_BOOTSTRAP     = (1 << 1), /*!< 0002 - BRAKE BOOTSTRAP  */
  PMSM_FOC_MSM_STOP_MOTOR          = (1 << 2), /*!< 0004 - Motor STOP  */
  PMSM_FOC_MSM_VF_OPENLOOP         = (1 << 3), /*!< 0008 - V/F OPEN LOOP */
  PMSM_FOC_MSM_PRE_POSITIONING     = (1 << 4), /*!< 0016 - ROTOR ALIGNMENT  */
  PMSM_FOC_MSM_ERROR               = (1 << 5), /*!< 0032 - ERROR  */
  PMSM_FOC_MSM_IDLE                = (1 << 6), /*!< 0064 - IDLE  */
  PMSM_FOC_MSM_IDLE_EXIT           = (1 << 7), /*!< 0127 - IDLE_EXIT  */
} PMSM_FOC_MSM_t;


/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * LOCAL API PROTOTYPES
 ********************************************************************************************************************/
__STATIC_INLINE void PMSM_FOC_MSM_VF_OPENLOOP_Func(PMSM_FOC_VF_OPEN_LOOP_t* const handle_ptr);
__STATIC_INLINE void PMSM_FOC_MSM_BRAKE_BOOTSTRAP_Func(void);
__STATIC_INLINE void PMSM_FOC_MSM_STOP_MOTOR_Func(void);
__STATIC_INLINE void PMSM_FOC_MSM_ERROR_Func(void);
__STATIC_INLINE void PMSM_FOC_MSM_CLOSED_LOOP_Func(void);
__STATIC_INLINE void PMSM_FOC_MSM_PRE_POSITIONING_Func(void);


/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * FUNCTION DEFINATION
 ********************************************************************************************************************/
/**
 * @brief Motor control state machine.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM(void)
{
  switch (PMSM_FOC_CTRL.msm_state)
  {
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif
    case PMSM_FOC_MSM_CLOSED_LOOP:
      PMSM_FOC_MSM_CLOSED_LOOP_Func();
      break;

    case PMSM_FOC_MSM_VF_OPENLOOP:
      PMSM_FOC_MSM_VF_OPENLOOP_Func(&PMSM_FOC_VF_OPEN_LOOP_CTRL);
      break;

    case PMSM_FOC_MSM_BRAKE_BOOTSTRAP:
      PMSM_FOC_MSM_BRAKE_BOOTSTRAP_Func();
      break;

    case PMSM_FOC_MSM_STOP_MOTOR:
      PMSM_FOC_MSM_STOP_MOTOR_Func();
      break;

    case PMSM_FOC_MSM_PRE_POSITIONING:
      PMSM_FOC_MSM_PRE_POSITIONING_Func();
      break;

    case PMSM_FOC_MSM_ERROR:
      PMSM_FOC_MSM_ERROR_Func();   /*comment out for PI flux control tuning*/
      break;

    default:
      break;
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif
  }

}

/*********************************************************************************************************************
 * STATE MACHINE Functions
 ********************************************************************************************************************/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_CLOSED_LOOP.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_CLOSED_LOOP_Func(void)
{
  /* Call control scheme - Vq voltage control or Speed inner current control as per user configurations */
  PMSM_FOC_CTRL.ctrl_scheme_fun_ptr();

#if(USER_VDC_VOLT_COMPENSATION == ENABLED)
#define COMPENSATION_FACTOR              (4U)   /* 0 to 8, adjustable for best performance */
  /* Unsigned mode,After division the result will be shifted by COMPENSATION_FACTOR */
  MATH->DIVCON = (0x00008004 | (COMPENSATION_FACTOR << MATH_DIVCON_QSCNT_Pos));
  MATH->DVD = PMSM_FOC_OUTPUT.svm_vref_16 * (MotorParam.VADC_DCLINK_T << COMPENSATION_FACTOR); // Dividend
  MATH->DVS = ADC.adc_res_vdc;//Divisor
#endif

  /* Miscellaneous works in FOC, such as ramp up, speed adjustment, stop Motor, etc. Execution time: 2.8us*/
  PMSM_FOC_MiscWorksOfFOC();

  /* Wait for division operation to get over */
#if(USER_VDC_VOLT_COMPENSATION == ENABLED)
  while(IS_MATH_DIV_BUSY);
  PMSM_FOC_OUTPUT.svm_vref_16 = MATH->QUOT;
#endif

  /* check svm_vref_16 amplitude should be limited to 37836 and Update SVM PWM. */
  if (PMSM_FOC_OUTPUT.svm_vref_16 > 37836) PMSM_FOC_OUTPUT.svm_vref_16 =  37836;  /* SVM vref_16 output limit set for over modulation (2/3)/(1/sqrt(3))*32767=37836 */
  PMSM_FOC_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16, PMSM_FOC_OUTPUT.svm_angle_16);
}

/**********************************************************************************************************************
 Stop the Motor, check PWM or POT ADC (for adjusting motor speed)
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_STOP_MOTOR.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_STOP_MOTOR_Func(void)
{
  if (MotorVar.MaskedFault.Value == 0)
  {
    if (SYSTEM_BE_IDLE)
    {
      /* If system is idle, i.e.: Reference or POT ADC too low.*/
        /* if user modify dead time value, the value is updated here  */
        CCU8_SLICE_PHASE_U->DC1R = MotorParam.CCU8_DEADTIME_RISE_T << CCU8_CC8_DC1R_DT1R_Pos |
                            MotorParam.CCU8_DEADTIME_FALL_T << CCU8_CC8_DC1R_DT1F_Pos;
        CCU8_SLICE_PHASE_V->DC1R = MotorParam.CCU8_DEADTIME_RISE_T << CCU8_CC8_DC1R_DT1R_Pos |
                            MotorParam.CCU8_DEADTIME_FALL_T << CCU8_CC8_DC1R_DT1F_Pos;
        CCU8_SLICE_PHASE_W->DC1R = MotorParam.CCU8_DEADTIME_RISE_T << CCU8_CC8_DC1R_DT1R_Pos |
                            MotorParam.CCU8_DEADTIME_FALL_T << CCU8_CC8_DC1R_DT1F_Pos;
    }
    else
    {
      /* Configure motor direction based upon GPIO(if any) */
  #ifdef MOTOR_DIR_INPUT_PIN
      if ((XMC_GPIO_GetInput(MOTOR_DIR_INPUT_PIN)) == IS_GPIO_LOW)
      {
        PMSM_FOC_CTRL.rotation_dir = DIRECTION_INC;
      }
      else
      {
        PMSM_FOC_CTRL.rotation_dir = DIRECTION_DEC;
      }
  #else
      PMSM_FOC_CTRL.rotation_dir = DIRECTION_INC;
  #endif
      /* Next go to the bootstrap state */
      PMSM_FOC_CTRL.braking_counter = 0;
      PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_BRAKE_BOOTSTRAP;
    }
  }
  else
  {
#ifdef INVERTER_EN_PIN
    /* Disable gate driver. */
    XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_DISABLE_LEVEL);
#endif
    PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_ERROR;
  }

  PMSM_FOC_SVPWM_Update(0, 0);
}

/**********************************************************************************************************************
 Rotor initial preposition/alignment
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_PRE_POSITIONING.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_PRE_POSITIONING_Func(void)
{
  if (PMSM_FOC_CTRL.alignment_counter == 0)
  {
    PMSM_FOC_OUTPUT.svm_angle_16 = 0; /* Align to SVM 0 degree*/
    /* Rotor preposition/alignment counter ++. */
    PMSM_FOC_CTRL.alignment_counter++;
  }
  else if (PMSM_FOC_CTRL.alignment_counter < MotorParam.ROTOR_PRE_ALIGNMENT_COUNT)
  {
    if (PMSM_FOC_OUTPUT.svm_vref_16 < MotorParam.ROTOR_PRE_ALIGNMENT_VREF)
    {
      /* Vref increases gradually. */
      PMSM_FOC_OUTPUT.svm_vref_16 += MotorParam.ROTOR_PRE_ALIGNMENT_RAMP_RATE;
    }

    /* check svm_vref_16 amplitude should be limited to 37836 and Update SVM PWM. */
    if (PMSM_FOC_OUTPUT.svm_vref_16 > 37836) PMSM_FOC_OUTPUT.svm_vref_16 =  37836;  /* SVM vref_16 output limit set for over modulation (2/3)/(1/sqrt(3))*32767=37836 */
    PMSM_FOC_SVPWM_Update(PMSM_FOC_OUTPUT.svm_vref_16, PMSM_FOC_OUTPUT.svm_angle_16);

    /* Rotor preposition/alignment counter ++. */
    PMSM_FOC_CTRL.alignment_counter++;
  }
  else
  {
    /* Clear counter. */
    PMSM_FOC_CTRL.alignment_counter = 0U;
    PMSM_FOC_OUTPUT.rotor_angle_q31 = PMSM_FOC_ANGLE_000_DEGREE_Q31;
    PMSM_FOC_VADC_PhCurrentInit();

    if((MotorParam.StartupMethod == MOTOR_STARTUP_DIRECT_FOC) && (MotorParam.ControlScheme != VF_OPEN_LOOP_CTRL))
    {
        PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_CLOSED_LOOP;
    }
    else
    {
        /* V/F open loop initialization */
        PMSM_FOC_VF_OpenLoopInit();
    }
  }
}

/**********************************************************************************************************************
 V/f Open Loop Ramp Up with rotor initial preposition/alignment
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_VF_OPENLOOP.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_VF_OPENLOOP_Func(PMSM_FOC_VF_OPEN_LOOP_t* const handle_ptr)
{
  PMSM_FOC_VADC_GetPhasecurrent(SVPWM.previous_sector_num, SVPWM.current_sector_num, &ADC);
  PMSM_FOC_CurrentReconstruction(ADC.adc_res_iu, ADC.adc_res_iv, ADC.adc_res_iw, &PMSM_FOC_INPUT);

  /* To get I_Alpha and I_Beta of last PWM cycle, scale down I_Mag (i.e.: |I|) by 2/3. */
  PMSM_FOC_ClarkeTransform(PMSM_FOC_INPUT.i_u, PMSM_FOC_INPUT.i_v, PMSM_FOC_INPUT.i_w, &PMSM_FOC_OUTPUT.clarke_tansform);

  /* Update V/f voltage amplitude, Vref = Offset + Kω. */
  handle_ptr->vref_mag = ((handle_ptr->vf_constant * handle_ptr->vf_motor_speed)>>15);
  handle_ptr->vref_mag += handle_ptr->vf_offset;

  /* Limit vref */
  if (handle_ptr->vref_mag > MAX_U_Q15)
  {
    /*  Limit |Vref| maximum value.*/
    handle_ptr->vref_mag = MAX_U_Q15;
  }

  /* θ[k] = θ[k-1] + ω[k]. */
   handle_ptr->vref_angle += ((int32_t)((handle_ptr->vf_motor_speed * handle_ptr->speed_angle_conversion_factor)>>handle_ptr->speed_angle_conversion_factor_scale) >> 16);

  if (PMSM_FOC_CTRL.transition_status == PMSM_FOC_MOTOR_STATUS_TRANSITION)
  {
    /* check if motor speed is reached to transition speed */
    if (handle_ptr->vf_motor_speed < handle_ptr->vf_transition_speed)
    {
        /* Speed ramp counter ++. */
        handle_ptr->vf_speed_ramp_counter++;

        if (handle_ptr->vf_speed_ramp_counter > handle_ptr->vf_speed_ramp_up_rate)
        {
          /* Increment motor speed. */
          handle_ptr->vf_motor_speed += handle_ptr->vf_speed_ramp_up_step;

          /* Clear ramp counter.*/
          handle_ptr->vf_speed_ramp_counter = 0;
      }
    }
    else
    {
      if (handle_ptr->dont_exit_open_loop_flag == FALSE)
      {
        /* motor run at V/f constant speed for a while.*/
        handle_ptr->stablization_counter++;

        if (handle_ptr->stablization_counter > handle_ptr->stablization_count)
        {
          /* Change flag: motor in stable mode of V/f ramp-up. */
               PMSM_FOC_CTRL.transition_status = PMSM_FOC_MOTOR_STATUS_STABLE;
        }
      }
    }
  }
  else
  {
    /* Init rotor angle for first FOC PWM cycle, Lag/lead current angle γ by a 90° angle. */
    PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 = handle_ptr->vref_angle - PMSM_FOC_ANGLE_090_DEGREE_Q31
        + (int32_t)((handle_ptr->vf_motor_speed * handle_ptr->speed_angle_conversion_factor)>>handle_ptr->speed_angle_conversion_factor_scale);

    /* To init PI controllers' Ik for first FOC PWM cycle. */
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif
    PMSM_FOC_PI_InitIks();
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif

    /* Init FOC rotor speed ωr = PI_PLL.Uk, needed for ωL|I|, ωLId, ωLIq,and FG frequency calculation.*/
    PMSM_FOC_PLL_PI.uk = handle_ptr->vf_motor_speed;

    /* motor reference speed of FOC close loop. */
    PMSM_FOC_INPUT.ref_speed = handle_ptr->vf_motor_speed;

    /* Next, go to FOC closed-loop. */
    PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_CLOSED_LOOP;

    /* Check if user control scheme if speed_currrent control, it need to enter Vq control first, then transit to speed_current coontrol */
    if (PMSM_FOC_INPUT.user_ctrl_scheme == SPEED_INNER_CURRENT_CTRL_SCHEME)
    {
        PMSM_FOC_CTRL.transition_status = PMSM_FOC_MOTOR_STATUS_TRANSITION;
    }

  }

  /* Check if Motor stop is requested */
  if (SYSTEM_BE_IDLE)
  {
    /* Next, go to Motor Stop. */
    PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_BRAKE_BOOTSTRAP;
  }

  /* Assign open loop speed to FOC output */
  //PMSM_FOC_OUTPUT.rotor_speed = (int32_t)((handle_ptr->vf_motor_speed*(int32_t)handle_ptr->openloop_to_closedloop_conv_factor)>>handle_ptr->openloop_to_closedloop_conv_factor_scale);
    PMSM_FOC_OUTPUT.rotor_speed = handle_ptr->vf_motor_speed;


  /*  Update SVM PWM. */
  PMSM_FOC_SVPWM_Update(handle_ptr->vref_mag, handle_ptr->vref_angle);
}

/**********************************************************************************************************************
 To brake the Motor, charge gate driver bootstrap capacitors (if any)
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_BRAKE_BOOTSTRAP.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_BRAKE_BOOTSTRAP_Func(void)
{
  uint32_t brake_cmp_val_1;
  uint32_t brake_cmp_val_2;

  /* set PMSM_FOC_INPUT.ref_speed to 0 while braking in speed inner current control scheme */
  if (PMSM_FOC_CTRL.ctrl_scheme_fun_ptr == &PMSM_FOC_SpeedCurrentCtrl) PMSM_FOC_INPUT.ref_speed=0;

  if (PMSM_FOC_CTRL.braking_counter == 0)
  {
    /* Pull CCUCON signal to low to stop PWM */
    XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);

#if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    brake_cmp_val_1 = (SVPWM.pwm_period_reg_val - PMSM_FOC_INPUT.brake_duty_val );
#else
    brake_cmp_val_1 = PMSM_FOC_INPUT.brake_duty_val;
#endif

#if(USER_MOTOR_BRAKE_DUTY == 100U)
    brake_cmp_val_2 = SVPWM.pwm_period_reg_val + 1;
#else
    brake_cmp_val_2 = SVPWM.pwm_period_reg_val;
#endif

    /* Turn OFF high side MOSFETS and Turn ON all lower MOSFETS */
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_U, brake_cmp_val_1);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_U, brake_cmp_val_2);

    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_V, brake_cmp_val_1);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_V, brake_cmp_val_2);

    XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_W, brake_cmp_val_1);
    XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_W, brake_cmp_val_2);

    /* Trigger shadow transfer */
    CCU8_MODULE->GCSS |= (uint32_t) (XMC_CCU8_SHADOW_TRANSFER_SLICE_0 | XMC_CCU8_SHADOW_TRANSFER_SLICE_1
        | XMC_CCU8_SHADOW_TRANSFER_SLICE_2 | XMC_CCU8_SHADOW_TRANSFER_SLICE_3);

    /* Enable GPIO mode to enable PWM on lower SW and turn off upper SW. */
    XMC_GPIO_SetMode(PHASE_U_HS_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(PHASE_V_HS_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetMode(PHASE_W_HS_PIN, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);

#if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
    XMC_GPIO_SetOutputLow(PHASE_U_HS_PIN);
    XMC_GPIO_SetOutputLow(PHASE_V_HS_PIN);
    XMC_GPIO_SetOutputLow(PHASE_W_HS_PIN);
#else
    XMC_GPIO_SetOutputHigh(PHASE_U_HS_PIN);
    XMC_GPIO_SetOutputHigh(PHASE_V_HS_PIN);
    XMC_GPIO_SetOutputHigh(PHASE_W_HS_PIN);
#endif

#ifdef INVERTER_EN_PIN
  /* ENABLE gate driver. */
  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_ENABLE_LEVEL);
#endif

    /* Start PWM channels synchronously */
    PMSM_FOC_CCUx_SyncStart();
  }

  PMSM_FOC_VADC_GetPhasecurrent(SVPWM.previous_sector_num, SVPWM.current_sector_num, &ADC);

#if(USER_MOTOR_PH_OFFSET_CALIBRATION == ENABLED)
  /* Read Iu ADC bias */
  ADC.adc_bias_iu += (int32_t)((ADC.adc_res_iu - ADC.adc_bias_iu)>> SHIFT_BIAS_LPF);
  /* Read Iv ADC bias */
  ADC.adc_bias_iv += (int32_t)((ADC.adc_res_iv - ADC.adc_bias_iv)>> SHIFT_BIAS_LPF);
  /* Read Iw ADC bias */
  ADC.adc_bias_iw += (int32_t)((ADC.adc_res_iw - ADC.adc_bias_iw)>> SHIFT_BIAS_LPF);
#endif

  PMSM_FOC_CTRL.braking_counter++;

  if (PMSM_FOC_CTRL.braking_counter > MotorParam.BOOTSTRAP_BRAKE_TIME)
  {
    if (!SYSTEM_BE_IDLE)
    {
      XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
      XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
      XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
      NVIC_ClearPendingIRQ(TRAP_IRQn);
      PMSM_FOC_VariablesInit();

      if (MotorParam.InitPosDetect == ROTOR_PRE_ALIGN_NONE)
      {
        if((MotorParam.StartupMethod == MOTOR_STARTUP_DIRECT_FOC) && (MotorParam.ControlScheme != VF_OPEN_LOOP_CTRL))
        /* Next, directly go to the close loop. */
        {
          PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_CLOSED_LOOP;
          /* Clear counters. */
          PMSM_FOC_CTRL.braking_counter = 0U;
          /* Update SVM PWM. */
          PMSM_FOC_SVPWM_Update(PMSM_FOC_INPUT.ref_vq, (PMSM_FOC_OUTPUT.svm_angle_16));
        }
        else
        {
          /* V/F open loop initialization */
          PMSM_FOC_VF_OpenLoopInit();
        }
      }
      else
      {
           /* Next, go to the rotor pre-position startup. */
         PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_PRE_POSITIONING;
      }

      /* Pull CCUCON signal to low to stop PWM */
      XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);

      /* revert back normal pwm configuration */
      XMC_GPIO_SetMode(PHASE_U_HS_PIN, PHASE_U_HS_ALT_SELECT);
      XMC_GPIO_SetMode(PHASE_V_HS_PIN, PHASE_V_HS_ALT_SELECT);
      XMC_GPIO_SetMode(PHASE_W_HS_PIN, PHASE_W_HS_ALT_SELECT);

      /* Set all upper switches duty to zer0 */
      PMSM_FOC_SVPWM_Update(0, 0);

      /* Start PWM channels synchronously */
      PMSM_FOC_CCUx_SyncStart();

      PMSM_FOC_CTRL.ramp_counter = 0U;
      PMSM_FOC_CTRL.motor_start_flag = 1U;
    }
    else
    {
      /* If Motor is IDLE for long time then shut down the inverter */
      if (PMSM_FOC_CTRL.braking_counter > (10 * MotorParam.BOOTSTRAP_BRAKE_TIME ))
      {
#ifdef INVERTER_EN_PIN
        /* Disable gate driver. */
        XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_DISABLE_LEVEL);
#endif
        /* Clear counters. */
        PMSM_FOC_CTRL.braking_counter = 0U;
        PMSM_FOC_CTRL.ramp_counter = 0U;
        PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
        PMSM_FOC_OUTPUT.rotor_speed = 0;
      }
    }
  }
}

/**********************************************************************************************************************
 Error handling function
 -----------------------------------------------------------------------------------------------------------------------*/
/**
 * @brief This function is called when Motor control state machine is set to PMSM_FOC_MSM_ERROR.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_MSM_ERROR_Func(void)
{
  /* Reset variables */
  PMSM_FOC_INPUT.ref_id = 0;
  PMSM_FOC_INPUT.ref_iq = 0;
  PMSM_FOC_INPUT.ref_speed = 0;
  PMSM_FOC_INPUT.ref_vq = 0;

  PMSM_FOC_OUTPUT.rotor_speed = 0;
  PMSM_FOC_OUTPUT.rotor_angle_q31 = 0;

  PMSM_FOC_CTRL.motor_start_flag = 0;  /* replace PMSM_FOC_MotorStop() function with direct cmd */
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif
  PMSM_FOC_ErrorHandling();
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif

}

/**
 * @}
 */

/**
 * @}
 */

#endif

