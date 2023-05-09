/**
 * @file pmsm_foc_functions.c
 *
 * @cond
 *********************************************************************************************************************
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "../Configuration/pmsm_foc_common.h"
#include <xmc_common.h>                   /* SFR declarations of the selected device */

#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../MCUInit/pmsm_foc_systick.h"
#include "../MCUInit/pmsm_foc_gpio.h"
#include "../MCUInit/pmsm_foc_adc.h"
#include "../MCUInit/6EDL_spi.h"
#include "../MCUInit/pmsm_foc_adc.h"
#include "../MCUInit/pmsm_foc_eru.h"
#include "../ToolInterface/Register.h"

#include "../ToolInterface/uCProbe.h"
#include "../MCUInit/6EDL_gateway.h"

#include "../MIDSys/pmsm_foc_svpwm.h"

#include "pmsm_foc_error_handling.h"
#include "pmsm_foc_functions.h"
#include "pmsm_foc_state_machine.h"
#include "pmsm_foc_functions.h"
#include "pmsm_foc_vq_voltage_ctrl.h"
#include "pmsm_foc_interface.h"
#include "../MCUInit/pmsm_foc_ccu4.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
PMSM_FOC_INPUT_t PMSM_FOC_INPUT;
PMSM_FOC_OUTPUT_t PMSM_FOC_OUTPUT;
PMSM_FOC_CTRL_t PMSM_FOC_CTRL;
PMSM_FOC_VF_OPEN_LOOP_t PMSM_FOC_VF_OPEN_LOOP_CTRL;
/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define PMSM_FOC_SETTLING_TIME   (0x7FFF)
/*********************************************************************************************************************
 * LOCAL API PROTOTYPES
 ********************************************************************************************************************/
/* Peripheral initialization of VADC module  */
static void PMSM_FOC_Measurement_Init(void);
/* SVPWM module initialization */
static void PMSM_FOC_SVPWM_Init(void);
/* Initializes motor control required peripherals */
static void PMSM_FOC_PeripheralsInit(void);

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
/* PLL observer initialization */
void PMSM_FOC_PLL_Init(PMSM_FOC_PLL_ESTIMATOR_t * const pll_estimator_ptr);

/* Adjust FOC parameter dynamically */
__STATIC_INLINE void PMSM_FOC_AdjustParameters(void);

/*********************************************************************************************************************
 * API DEFINATION
 ********************************************************************************************************************/
/**
 * @brief  Variables initialization before motor start
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VariablesInit(void)
{
  /* FOC Configuration initialization */
  PMSM_FOC_INPUT.ref_id = 0;
  PMSM_FOC_INPUT.ref_vq = MotorParam.VQ_INITIAL_VALUE;
  PMSM_FOC_INPUT.ref_speed = 0;
  PMSM_FOC_INPUT.ref_iq = 0;
  PMSM_FOC_INPUT.brake_duty_val = MotorParam.MOTOR_BRAKE_DUTY_VAL;
#if(USER_OVERCURRENT_PROTECTION == ENABLED)
  PMSM_FOC_INPUT.idc_over_current_limit = MotorParam.IDC_OVER_CURRENT_LIMIT;
#endif

#if(USER_TORQUE_LIMITER == ENABLED)
  PMSM_FOC_INPUT.limit_max_iq = USER_IQ_LIMIT_Q15;
#else
  PMSM_FOC_INPUT.limit_max_iq = MAX_U_Q15;       // Reset to max limit
#endif

  if (MotorParam.ControlScheme == VQ_VOLTAGE_CTRL)
  {
    PMSM_FOC_INPUT.user_ctrl_scheme = VQ_VOLTAGE_CTRL_SCHEME;
  }
  else if (MotorParam.ControlScheme == SPEED_INNER_CURRENT_CTRL)
  {
    PMSM_FOC_INPUT.user_ctrl_scheme = SPEED_INNER_CURRENT_CTRL_SCHEME;
  }
  else
  {
    PMSM_FOC_INPUT.user_ctrl_scheme = VF_OPEN_LOOP_CTRL_SCHEME;
  }

  PMSM_FOC_CTRL.transition_status = PMSM_FOC_MOTOR_STATUS_TRANSITION;
  /* For Speed inner current control also first start with Vq voltage control and then switch. */
  PMSM_FOC_CTRL.ctrl_scheme_fun_ptr = &PMSM_FOC_VqVoltageCtrl;
  PMSM_FOC_CTRL.braking_counter = 0;
  PMSM_FOC_CTRL.ramp_counter = 0;
  PMSM_FOC_CTRL.alignment_counter = 0;
  MotorVar.error_status = 0; /* Clear error if any before start*/
  PMSM_FOC_CTRL.iq_limit_blanking_counter = 0;

  PMSM_FOC_OUTPUT.decoupling_id = 0;
  PMSM_FOC_OUTPUT.decoupling_iq = 0;

  PMSM_FOC_OUTPUT.svm_vref_16 = 0;
  PMSM_FOC_OUTPUT.svm_angle_16 = 0;		// Init Vref angle θ = X°.

  PMSM_FOC_OUTPUT.car2pol.vref_angle_q31 = PMSM_FOC_ANGLE_090_DEGREE_Q31;

  PMSM_FOC_OUTPUT.park_transform.flux_id = 0;
  PMSM_FOC_OUTPUT.park_transform.torque_iq = 0;
  PMSM_FOC_OUTPUT.torque_vq = PMSM_FOC_INPUT.ref_vq;

  ADC.adc_res_vdc = MotorParam.VADC_DCLINK_T;
  ADC.adc_res_idc = 0;

  /* PLL observer initialization */
  PMSM_FOC_PLL_Init(&PMSM_FOC_PLL_ESTIMATOR);

  /* SVPWM module initialization */
  PMSM_FOC_SVPWM_Init();
}

/**
 * @brief Initialization before motor start
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_MotorControl_Init(void)
{
  /* Init variables. */
  PMSM_FOC_VariablesInit ();

  /* Next go to stop motor state and wait for the motor start command */
  PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
}

/**
 * @brief  PLL observer initialization before motor start
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_PLL_Init(PMSM_FOC_PLL_ESTIMATOR_t * const pll_estimator_ptr)
{
  pll_estimator_ptr->phase_inductance_Ls = MotorParam.DEFAULT_LS_SCALEDUP;
  pll_estimator_ptr->phase_inductance_scale = MotorParam.DEFAULT_SCALE_OF_L;

  /* Resolution increase, use (16 + USER_RES_INC) bit to represent 360 deg. */
  pll_estimator_ptr->res_inc = USER_RES_INC;

  /* Filter coefficients used inside PLL observer */
  pll_estimator_ptr->lpf_n_bemf = USER_PLL_LPF;
  pll_estimator_ptr->lpf_n_speed = USER_PLL_SPEED_LPF;

  /* Reset PLL observer parameters */
  pll_estimator_ptr->rotor_angle_q31 = 0;
  pll_estimator_ptr->rotor_speed = PMSM_FOC_INPUT.ref_speed;

  pll_estimator_ptr->current_i_mag = 0;
  pll_estimator_ptr->current_i_mag_filtered = 0;

  pll_estimator_ptr->bemf_1 = 0;
  pll_estimator_ptr->bemf_2 = 0;

  PMSM_FOC_PLL_PI.ik = PMSM_FOC_OUTPUT.rotor_speed << PMSM_FOC_PLL_PI.scale_kp_ki;
}

/**
 * @brief  API to initialize MCU and peripherals for motor control
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_Init(void)
{
  /* Initializes motor control peripherals */
  PMSM_FOC_PeripheralsInit();

  /*Init parameters (Kp / Ki, limits) of PI controllers from PI_CONFIG . */
  PMSM_FOC_PI_Init();

  /* Initialization for motor control.  */
  PMSM_FOC_MotorControl_Init();

#if(USER_UCPROBE_GUI == ENABLED)
  PMSM_FOC_uCProbe_Init();
#endif

  /* Clear Trap event before start */
  XMC_CCU8_SLICE_ClearEvent (CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
  XMC_CCU8_SLICE_ClearEvent (CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
  XMC_CCU8_SLICE_ClearEvent (CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
  NVIC_ClearPendingIRQ (TRAP_IRQn);

  /* Synchronous start of CAPCOM modules, e.g.: CCU8x, and or CCU4x */
  PMSM_FOC_CCUx_SyncStart();
}

/**
 * @brief  API to initialize peripherals for motor control
 *
 * @param None
 *
 * @retval None
 */
static void PMSM_FOC_PeripheralsInit(void)
{
  /* Reset configuration, clock configuration */
  PMSM_FOC_Clock_Init();

  /* Enable prefetch unit */
  XMC_SCU_EnablePrefetchUnit();

  /* Init GPIOs */
  PMSM_FOC_GPIO_Init();

  /* Init CCU8 */
  PMSM_FOC_CCU8_Init();

  /* Init VADC */
  PMSM_FOC_Measurement_Init();

  /* Init ERU */
  PMSM_FOC_PIN_INTERRUPT_Init();

}

/**
 * @brief  Initializes measurement module along with the peripherals - VADC
 *
 * @param None
 *
 * @retval None
 */
static void PMSM_FOC_Measurement_Init(void)
{
  /* Motor phase current measurement VADC peripheral initialization */
  PMSM_FOC_VADC_PhCurrentInit();

  /* DC bus voltage measurement VADC peripheral initialization */
  PMSM_FOC_VADC_VDCInit();

  /* Potentiometer measurement VADC peripheral initialization */
  PMSM_FOC_VADC_PotInit();

  /* Temperature measurement VADC peripheral initialization */
  PMSM_FOC_VADC_TEMPInit();

  /* Init CCU4 for debug */
#if((DEBUG_PWM_0_ENABLE == 1U) || (DEBUG_PWM_1_ENABLE == 1U))
  PMSM_FOC_CCU4_Init();
#endif

  /* Init MATH Unit (i.e.: CORDIC Co-processor and Divider Unit DIV) */
  PMSM_FOC_MATH_Init();

#if(USER_WATCH_DOG_TIMER == ENABLED)
  /* Init WDT */
  PMSM_FOC_WDT_Init();
#endif

}

/**
 * @brief To init PI controllers' integral terms (Ik) for first FOC PWM cycle after V/F open loop
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_PI_InitIks(void)
{
  PMSM_FOC_SPEED_PI.ik = PMSM_FOC_PLL_ESTIMATOR.current_i_mag; /* Init PI integral terms for smooth transition to FOC. */
  PMSM_FOC_SPEED_PI.ik <<= PMSM_FOC_SPEED_PI.scale_kp_ki; /* All PI integral terms left shift by PI_data->Scale_KpKi. */

  /* Init Vq of torque / Iq PI controller, |Vref|cos(Î³-Î¸) = |Vref|cos(Î¸-Î³).  */
  PMSM_FOC_TORQUE_PI.ik = PMSM_FOC_PLL_ESTIMATOR.vref_x_cos_delta;
  PMSM_FOC_TORQUE_PI.ik <<= PMSM_FOC_TORQUE_PI.scale_kp_ki;

  /* Init Vd of flux / Id PI controller, |Vref|sin(Î³-Î¸) < 0 typically. */
  PMSM_FOC_FLUX_PI.ik = PMSM_FOC_PLL_ESTIMATOR.vref_x_sin_delta;
  PMSM_FOC_FLUX_PI.ik <<= PMSM_FOC_FLUX_PI.scale_kp_ki;

  /* Init rotor speed Ï‰r of PLL Observer PI controller. */
  PMSM_FOC_PLL_PI.ik = PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_motor_speed;
  PMSM_FOC_PLL_PI.ik <<= PMSM_FOC_PLL_PI.scale_kp_ki;

}

/**
 * @brief Miscellaneous works in FOC, such as ramp up, speed adjustment, stop Motor, etc
 * Do NOT add any CORDIC calculations in this function.
 *
 * @param None
 *
 * @retval None
 */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_MiscWorksOfFOC(void)
{
  /* Check if system is idle then stop the Motor as per use configuration */
  if (SYSTEM_BE_IDLE)
  {
    /* Clear counters. */
    PMSM_FOC_CTRL.braking_counter = 0;
    PMSM_FOC_CTRL.ramp_counter = 0;
    PMSM_FOC_OUTPUT.rotor_speed = 0;
    PMSM_FOC_OUTPUT.rotor_angle_q31 = 0;
    PMSM_FOC_PLL_ESTIMATOR.rotor_speed = 0;
    PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31 = 0;
    PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_BRAKE_BOOTSTRAP;
  }

  /* FOC is in stable mode. Adjust FOC parameters if any */
  if (PMSM_FOC_CTRL.transition_status == PMSM_FOC_MOTOR_STATUS_STABLE)
  {
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif
    PMSM_FOC_AdjustParameters();
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif
  }
}

/* ***********************************************************************************************************************************
 * STATIC API IMPLEMENTATION
 * ***********************************************************************************************************************************/

/**
 * @brief Adjust parameters, e.g.: for PI controllers, in FOC stable state
 * Scheduling - using different parameters in different operating regions.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_AdjustParameters(void)
{
  /* To indicate that adjustment of this parameter is done. */
  PMSM_FOC_CTRL.adjust_para_flag = ADJUST_DONE;
}

/**
 * @brief Initialize V/F open loop parameters
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VF_OpenLoopInit(void)
{
  /* V/F configuration */
  PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_offset = MotorParam.STARTUP_VF_OFFSET;
  PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_constant = MotorParam.STARTUP_VF_V_PER_HZ_CONST;
  PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_transition_speed = MotorParam.STARTUP_VF_TRANSITION_SPEED;
  PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_speed_ramp_up_rate = MotorParam.STARTUP_VF_SPEED_RAMP_UP_RATE;
  PMSM_FOC_VF_OPEN_LOOP_CTRL.stablization_count = MotorParam.STARTUP_VF_STABILIZATION_COUNT;

  if (MotorParam.ControlScheme == VF_OPEN_LOOP_CTRL)
  {
    PMSM_FOC_VF_OPEN_LOOP_CTRL.dont_exit_open_loop_flag = TRUE;
  }
  else
  {
    PMSM_FOC_VF_OPEN_LOOP_CTRL.dont_exit_open_loop_flag = FALSE;
  }

  /* Reset V/F control variables  */
  PMSM_FOC_VF_OPEN_LOOP_CTRL.vf_motor_speed = 0;
  PMSM_FOC_VF_OPEN_LOOP_CTRL.stablization_counter = 0;
  PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_angle = 0;
  PMSM_FOC_VF_OPEN_LOOP_CTRL.vref_mag = 0;

  /* Change the motor control state and status */
  PMSM_FOC_CTRL.transition_status = PMSM_FOC_MOTOR_STATUS_TRANSITION;
  PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_VF_OPENLOOP;
}

/**
 * @brief SVPWM module initialization
 *
 * @param None
 *
 * @retval None
 */
static void PMSM_FOC_SVPWM_Init(void)
{
  SVPWM.previous_sector_num = 0;
  SVPWM.flag_3or2_adc = USE_ALL_ADC;		/* Init to use all (e.g.: three) ADC samplings for current reconstruction, for 2or3-shunt.*/
  SVPWM.pwm_period_reg_val = MotorParam.CCU8_PERIOD_REG_T;
  SVPWM.vadc_trigger_point = (MotorParam.CCU8_PERIOD_REG_T >> 1) + MotorParam.DRIVERIC_DELAY;
  SVPWM.t_min = 200;
  SVPWM.t_max = MotorParam.CCU8_PERIOD_REG_T - SVPWM.t_min;

  if (MotorParam.SvmScheme == STANDARD_SVM_5_SEGMENT)
  {
    SVPWM.modulation_func_ptr = PMSM_FOC_SVPWM_5Seg;
  }
  else
  {
    SVPWM.modulation_func_ptr = PMSM_FOC_SVPWM_7Seg;
  }

  if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_INC)
  {
    /* motor rotation direction - rotor angle increasing. */
    SVPWM.ccu8_phu_module_ptr = CCU8_SLICE_PHASE_U;
    SVPWM.ccu8_phv_module_ptr = CCU8_SLICE_PHASE_V;
    SVPWM.ccu8_phw_module_ptr = CCU8_SLICE_PHASE_W;
  }
  else
  {
    /* motor rotation direction - rotor angle decrementing. */
    SVPWM.ccu8_phu_module_ptr = CCU8_SLICE_PHASE_U;
    SVPWM.ccu8_phv_module_ptr = CCU8_SLICE_PHASE_W;
    SVPWM.ccu8_phw_module_ptr = CCU8_SLICE_PHASE_V;
  }

  SVPWM.invalid_current_sample_flag = 0;
}

