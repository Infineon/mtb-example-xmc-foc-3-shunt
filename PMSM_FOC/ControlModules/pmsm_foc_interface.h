/**
 * @file pmsm_foc_interface.h
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
#ifndef PMSM_FOC_INTERFACE_H_
#define PMSM_FOC_INTERFACE_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

#include "../MIDSys/pmsm_foc_svpwm.h"
#include "../MIDSys/pmsm_foc_current_sense_3S.h"
#include "../MIDSys/pmsm_foc_debug.h"
#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_state_machine.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
void PMSM_FOC_MotorStart(void);
void PMSM_FOC_MotorStop(void);
void PMSM_FOC_MiscWorks(void);

extern volatile uint16_t adc_res_vdc_temp;
extern volatile uint16_t adc_res_tempSense_raw;
extern volatile uint16_t adc_res_tempSense_degreeC;

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 * @brief This function monitors the system periodically
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE void PMSM_FOC_SysMonitoring(void)
{

  /* DC link ADC LPF. Read RES5 for ADC result (Previous ADC result) */
  adc_res_vdc_temp = XMC_VADC_GROUP_GetResult(VADC_VDC_GROUP, VADC_VDC_RESULT_REG);
#if((USER_ADC_CALIBRATION == ENABLED) && (UC_SERIES == XMC13))
  /* Clear offset calibration values*/
  CLEAR_OFFSET_CALIB_VALUES;
#endif
  ADC.adc_res_vdc+= (int32_t) ((adc_res_vdc_temp - ADC.adc_res_vdc) >> ADCLPF);

  /***************** Over/Under voltage protection ************************************/

  if ((MotorParam.EnableFault.OverVoltage == 1) && (MotorParam.EnableFault.UnderVoltage == 1))
    {
      if (ADC.adc_res_vdc > MotorParam.OVERVOLTAGE_THRESHOLD)
      {
        /* Next Go to error state */
          MotorVar.error_status |= PMSM_FOC_EID_OVER_VOLT;
      }
      else if (ADC.adc_res_vdc < MotorParam.UNDERVOLTAGE_THRESHOLD)
      {
        /* Next Go to error state */
          MotorVar.error_status |= PMSM_FOC_EID_UNDER_VOLT;
      }
    }
// End of Over/Under voltage protection

  /***************** Over Current protection *****************************************/
#if(USER_OVERCURRENT_PROTECTION == ENABLED)
  int32_t idc_temp;
  idc_temp = (int32_t)XMC_VADC_GROUP_GetResult(VADC_IDC_GROUP,VADC_IDC_RESULT_REG);
#if((USER_ADC_CALIBRATION == ENABLED) && (UC_SERIES == XMC13))
  /* Clear offset calibration values*/
  CLEAR_OFFSET_CALIB_VALUES;
#endif
  ADC.adc_res_idc += (int32_t)((idc_temp - ADC.adc_res_idc) >> ADCLPF);

  if (ADC.adc_res_idc > PMSM_FOC_INPUT.idc_over_current_limit)
  {
    /* Next Go to error state */
      PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_OVER_CURRENT;
      MotorVar.error_status |= PMSM_FOC_EID_OVER_CURRENT;
  }
#endif  // End of #if(USER_OVERCURRENT_PROTECTION == ENABLED)

  /***************** Torque Limit protection *****************************************/
#if(USER_TORQUE_LIMITER == ENABLED)
  if (PMSM_FOC_CTRL.transition_status == PMSM_FOC_MOTOR_STATUS_STABLE)
  {
    /* Check if iq limit exceeded even after blanking time. If yes then go to error state */
    if (PMSM_FOC_OUTPUT.park_transform.torque_iq >= PMSM_FOC_INPUT.limit_max_iq)
    {
      if (PMSM_FOC_CTRL.iq_limit_blanking_counter >= USER_IQ_LIMIT_BLANKING_TIME)
      {
        /* Next Go to error state */
          PMSM_FOC_CTRL.error_status |= PMSM_FOC_EID_TORQUE_LIMIT_EXCEED;
          MotorVar.error_status |= PMSM_FOC_EID_TORQUE_LIMIT_EXCEED;
        /* Reset associated variables */
        PMSM_FOC_CTRL.iq_limit_blanking_counter = 0;
        PMSM_FOC_OUTPUT.park_transform.torque_iq = 0;
      }
      else
      {
        PMSM_FOC_CTRL.iq_limit_blanking_counter++;
      }
    }
    else
    {
      PMSM_FOC_CTRL.iq_limit_blanking_counter = 0;
    }
  }
#endif  // End of #if(USER_TORQUE_LIMITER == ENABLED)

  /***************** Over temperature Protection ***************************************
  * Read analog input from MCP9700 temperature sensor
  *     0°C = 500mV
  *     sensitivity = 10mV/°C
  * For DVDD=3.3V, 10mV = 12.41212 (ADC cts)
  * For DVDD=5.0V, 10mV = 8.192 (ADC cts)
  * t_sensor is °C in Q4, conversion from ADC counts to t_sensor:
  *     DVDD=3.3V, 16/12.41212 = 1.289 ~= 660 >> 9
  *     DVDD=5.0V, 16/8.192 = 1.953125 = 125 >> 6
  ****************************************************/
  /* Over temperature protection */

  adc_res_tempSense_raw = XMC_VADC_GROUP_GetResult(VADC_TEMP_GROUP, VADC_TEMP_CHANNEL);
  if (MotorParam.Vadc_Ref_Voltage == 33)
  {
    /* DVDD = 3.3V */
      adc_res_tempSense_degreeC = ((adc_res_tempSense_raw - 620) * 660) >> 9;  /* 660/512=1.289063 */
  }
  else
  {
    /* DVDD = 5.0V */
      adc_res_tempSense_degreeC = ((adc_res_tempSense_raw - 409) * 125) >> 6;  /* 125/64=1.953125 */
  }
  MotorVar.t_sensor.sum += (adc_res_tempSense_degreeC - MotorVar.t_sensor.output) << 10;    /* simple low pass filter, time constant = 64 tics */

  /************************************************************************************/
  /* Check error status against enable fault */
  MotorVar.MaskedFault.Value = MotorVar.error_status & MotorParam.EnableFault.Value;
  if (MotorVar.MaskedFault.Value != PMSM_FOC_EID_NO_ERROR)
  {
    /* Enable low side braking in 6EDL7141 gate driver through nBRAKE pin and go to error state */
    XMC_GPIO_SetOutputLow(CYBSP_GD_NBRAKE);
    PMSM_FOC_CTRL.msm_state = (1 << 5);//PMSM_FOC_MSM_ERROR;
  }
}

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_INTERFACE_H_ */


