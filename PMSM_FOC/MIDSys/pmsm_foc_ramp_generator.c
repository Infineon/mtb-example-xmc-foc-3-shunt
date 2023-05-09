/**
 * @file pmsm_foc_ramp_generator.c
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
#include "pmsm_foc_ramp_generator.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_state_machine.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
void PMSM_FOC_LinearRampGenerator(int32_t set_val, int32_t rampup_step, int32_t rampdown_step,
                                         int32_t slew_rate, int32_t *reference_val)
{
  if (*reference_val != set_val)
  {
		PMSM_FOC_CTRL.ramp_counter ++;
		if(PMSM_FOC_CTRL.ramp_counter >= slew_rate)
		{
			PMSM_FOC_CTRL.ramp_counter = 0;
			if(*reference_val < set_val)
			{
				*reference_val += rampup_step;
				if(*reference_val > set_val)
				{
					*reference_val = set_val;
				}
			}
			else
			{
				if((*reference_val > rampdown_step) && (ADC.adc_res_vdc < MotorParam.BRAKING_VDC_MAX_LIMIT))
				{
					*reference_val -= rampdown_step;
					if(*reference_val < set_val)
					{
						*reference_val = set_val;
					}
				}
			}
		}
  }
}
