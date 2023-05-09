/**
 * @file pmsm_foc_vq_voltage_ctrl.c
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

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../IPLib/pmsm_foc_ip.h"
#include "../MIDSys/pmsm_foc_pi.h"
#include "pmsm_foc_vq_voltage_ctrl.h"
#include "pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_error_handling.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ******************************************************************************************************************/

/**
 * @brief Vq Voltage control. Motor is driven per Vq voltage reference.
 *
 * @param None
 *
 * @retval None
 */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_VqVoltageCtrl(void)
{
  /* Read phase current from ADC */
  PMSM_FOC_VADC_GetPhasecurrent(SVPWM.previous_sector_num, SVPWM.current_sector_num, &ADC);
  /* Remove offset and target scaling */
  PMSM_FOC_CurrentReconstruction(ADC.adc_res_iu, ADC.adc_res_iv, ADC.adc_res_iw, &PMSM_FOC_INPUT);

  if (MotorParam.BiDirectional == ENABLED)
  {
    /* May need swap Iv and Iw, so that Motor reverses */
    if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
    {
	  SWAP(PMSM_FOC_INPUT.i_v, PMSM_FOC_INPUT.i_w);
    }
  }

  PMSM_FOC_ClarkeTransform(PMSM_FOC_INPUT.i_u, PMSM_FOC_INPUT.i_v, PMSM_FOC_INPUT.i_w,&PMSM_FOC_OUTPUT.clarke_tansform);
  /**************************************************************************************************************/
  /* Parallel computation #1 - ------ and CORDIC(PARK Transform) */
  /**************************************************************************************************************/
  PMSM_FOC_ParkTransform(&PMSM_FOC_OUTPUT.clarke_tansform, PMSM_FOC_OUTPUT.rotor_angle_q31);
  PMSM_FOC_ParkTransform_GetResult(&PMSM_FOC_OUTPUT.park_transform);

  /**************************************************************************************************************/
  /* Parallel computation #2 - ------ and CORDIC(PLL Imag) */
  /**************************************************************************************************************/
  PMSM_FOC_PLL_Imag(PMSM_FOC_OUTPUT.car2pol.vref_angle_q31, PMSM_FOC_OUTPUT.clarke_tansform.i_alpha_1q31,PMSM_FOC_OUTPUT.clarke_tansform.i_beta_1q31);
  PMSM_FOC_PLL_ImagGetResult(&PMSM_FOC_PLL_ESTIMATOR);

  PMSM_FOC_OUTPUT.torque_vq = PMSM_FOC_INPUT.ref_vq;
  /**************************************************************************************************************/
  /* Parallel computation #3 - CPU(Flux PI) and CORDIC(PMSM_FOC_PLL_ESTIMATOR) */
  /**************************************************************************************************************/
  PMSM_FOC_PLL_Vref(PMSM_FOC_OUTPUT.car2pol.vref32, &PMSM_FOC_PLL_ESTIMATOR);

  /* For constant Vq reference DQ decoupling is done only for the Flux PI current(Id) control output. */
  if(MotorParam.DQ_DECOUPLING == ENABLED)
  {
  /* Cross coupling components for Flux & Torque PI control */
  int32_t pll_estimated_speed = (PMSM_FOC_PLL_ESTIMATOR.rotor_speed >> PMSM_FOC_PLL_ESTIMATOR.res_inc);

  PMSM_FOC_OUTPUT.decoupling_id = -(((pll_estimated_speed * (PMSM_FOC_PLL_ESTIMATOR.phase_inductance_Ls))
      * (PMSM_FOC_OUTPUT.park_transform.torque_iq)) >> (PMSM_FOC_PLL_ESTIMATOR.phase_inductance_scale));

  PMSM_FOC_OUTPUT.decoupling_iq = (((pll_estimated_speed * (PMSM_FOC_PLL_ESTIMATOR.phase_inductance_Ls))
      * (PMSM_FOC_OUTPUT.park_transform.flux_id)) >> (PMSM_FOC_PLL_ESTIMATOR.phase_inductance_scale));
  }

  /* PI Controller #3 - Flux / Id PI controller of FOC */
  if (SVPWM.invalid_current_sample_flag == 0U)
  {
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif
	PMSM_FOC_PI_AntiWindup(PMSM_FOC_INPUT.ref_id, PMSM_FOC_OUTPUT.park_transform.flux_id,
	        PMSM_FOC_OUTPUT.decoupling_id, &PMSM_FOC_FLUX_PI);
#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif
    PMSM_FOC_OUTPUT.flux_vd = PMSM_FOC_FLUX_PI.uk;
  }

  PMSM_FOC_PLL_VrefGetResult(&PMSM_FOC_PLL_ESTIMATOR);

  /**************************************************************************************************************/
  /* Parallel computation #4 - PLL Observer and CORDIC(cart2polar) */
  /**************************************************************************************************************/
  PMSM_FOC_Cart2Polar(PMSM_FOC_OUTPUT.torque_vq, PMSM_FOC_OUTPUT.flux_vd, PMSM_FOC_OUTPUT.rotor_angle_q31);

  /* Execute PLL observer to estimate the rotor position & speed */
  PMSM_FOC_PLL_GetPosSpeed(&PMSM_FOC_PLL_ESTIMATOR, &PMSM_FOC_PLL_PI);

  /* Assign to FOC output structure */
  PMSM_FOC_OUTPUT.rotor_speed = PMSM_FOC_PLL_ESTIMATOR.rotor_speed;
  PMSM_FOC_OUTPUT.rotor_angle_q31 = PMSM_FOC_PLL_ESTIMATOR.rotor_angle_q31;

  PMSM_FOC_Cart2Polar_GetResult(&PMSM_FOC_OUTPUT.car2pol);
  /**************************************************************************************************************/
  /* Update PMSM_FOC_OUTPUT structure */
  PMSM_FOC_OUTPUT.svm_vref_16 = (uint16_t) (PMSM_FOC_OUTPUT.car2pol.vref32 >> CORDIC_SHIFT);
  PMSM_FOC_OUTPUT.svm_angle_16 = (uint16_t) (PMSM_FOC_OUTPUT.car2pol.vref_angle_q31 >> 16);



}
