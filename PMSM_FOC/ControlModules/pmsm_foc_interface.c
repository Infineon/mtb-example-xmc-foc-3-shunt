/**
 * @file pmsm_foc_interface.c
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

#include <PMSM_FOC/Configuration/pmsm_foc_common.h>
#include <PMSM_FOC/Configuration/pmsm_foc_user_input_config.h>
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../MIDSys/pmsm_foc_pi.h"
#include "../ToolInterface/ProbeScope/probe_scope.h"
#include "../ToolInterface/uCProbe.h"
//#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../MCUInit/pmsm_foc_clock.h"
//#include "../ControlModules/pmsm_foc_state_machine.h"
#include "../MIDSys/pmsm_foc_ramp_generator.h"
#include "pmsm_foc_interface.h"

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * DATA
 ********************************************************************************************************************/
volatile uint16_t adc_res_vdc_temp;
volatile uint16_t adc_res_tempSense_raw;
volatile uint16_t adc_res_tempSense_degreeC;


/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API Implementation
 ********************************************************************************************************************/
/**
 * @brief This function trigger ADC conversion and read the ADC.adc_res_pot value,
 * follow by ramping in speed or Vq depending on the motor control scheme selected
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_MiscWorks(void)
{
  XMC_VADC_GROUP_ScanTriggerConversion(VADC_G0);
  XMC_VADC_GROUP_ScanTriggerConversion(VADC_G1);
  if (MotorParam.AnalogControl == ENABLED)
  {
    volatile uint16_t pot_adc_result;
    pot_adc_result = XMC_VADC_GROUP_GetResult(VADC_POT_GROUP, VADC_POT_RESULT_REG);
    ADC.adc_res_pot = (int32_t) ((pot_adc_result * 4505) >> 12); /* 1.1 scaling for the 1kohm resistor place before 10Kohm potentiometer */
    /* POT ADC LPF. Read RES7 for ADC result (Previous ADC result). */
    ADC.adc_res_pot += (int32_t) ((pot_adc_result - ADC.adc_res_pot) >> USER_POT_ADC_LPF);
    if (ADC.adc_res_pot < (USER_TH_POT_ADC>>1))
    {
    	ADC.adc_res_pot = 0;  /* prevent the GUI's Target Set bar from moving around the 0% */
    }
    if (ADC.adc_res_pot > 4000)
    {
    	ADC.adc_res_pot = 4096; /* prevent the GUI's Target Set bar from moving around the 100% */
    }
  }

  if (PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_CLOSED_LOOP)
  {
    if (PMSM_FOC_CTRL.ctrl_scheme_fun_ptr == &PMSM_FOC_SpeedCurrentCtrl)
    {
      /* POT ADC values 0 ~ 2^12 represent Motor target speed of ELECTRICAL_SPEED_LOW_LIMIT_TS ~ ELECTRICAL_SPEED_HIGH_LIMIT_TS:*/
      PMSM_FOC_CTRL.set_val_pot = MotorParam.ELECTRICAL_SPEED_LOW_LIMIT_TS
          + ((uint32_t) ((MotorParam.ELECTRICAL_SPEED_HIGH_LIMIT_TS - MotorParam.ELECTRICAL_SPEED_LOW_LIMIT_TS) * ADC.adc_res_pot) >> 12);
      /* Limit speed, in case ADC values not 0 ~ 2^12.*/
      PMSM_FOC_CTRL.set_val_pot = MIN_MAX_LIMIT(PMSM_FOC_CTRL.set_val_pot, MotorParam.SPEED_REF_HIGH_LIMIT_TS, MotorParam.SPEED_REF_LOW_LIMIT_TS);
      /* Ramp generates the Motor reference speed */
      PMSM_FOC_LinearRampGenerator(PMSM_FOC_CTRL.set_val_pot, MotorParam.RAMP_UP_SPEED, MotorParam.RAMP_DOWN_SPEED,
      USER_SPEED_RAMP_SLEWRATE, &PMSM_FOC_INPUT.ref_speed);
    }

    else if (PMSM_FOC_CTRL.ctrl_scheme_fun_ptr == &PMSM_FOC_VqVoltageCtrl)
    {
      /* POT ADC values 0 ~ 2^12 represent motor target speed of ELECTRICAL_SPEED_LOW_LIMIT_TS ~ ELECTRICAL_SPEED_HIGH_LIMIT_TS:*/
      PMSM_FOC_CTRL.set_val_pot = MotorParam.VQ_REF_LOW_LIMIT + ((uint32_t) ((MotorParam.VQ_REF_HIGH_LIMIT - MotorParam.VQ_REF_LOW_LIMIT) * ADC.adc_res_pot) >> 12);
      /* Limit speed, in case ADC values not 0 ~ 2^12.*/
      PMSM_FOC_CTRL.set_val_pot = MIN_MAX_LIMIT(PMSM_FOC_CTRL.set_val_pot, MotorParam.VQ_REF_HIGH_LIMIT, MotorParam.VQ_REF_LOW_LIMIT);
      /* Ramp generates the Motor reference voltage */
      PMSM_FOC_LinearRampGenerator(PMSM_FOC_CTRL.set_val_pot, USER_VQ_RAMPUP_STEP, USER_VQ_RAMPDOWN_STEP, USER_VQ_RAMP_SLEWRATE, &PMSM_FOC_INPUT.ref_vq);
    }
    else
    {
      /* V/F open loop */
    }
  }

#if(USER_WATCH_DOG_TIMER == ENABLED)
  /* Service watchdog. Without WDT service regularly , it will reset system.*/
  XMC_WDT_Service();
#endif

}

/**
 * @brief API to start the Motor. If motor is not in error state then only it will start
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_MotorStart(void)
{
  if (PMSM_FOC_CTRL.msm_state != PMSM_FOC_MSM_ERROR)
  {
    PMSM_FOC_CTRL.motor_start_flag = 1;
  }
}

/**
 * @brief API to stop the Motor.
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_MotorStop(void)
{
  PMSM_FOC_CTRL.motor_start_flag = 0;
}
