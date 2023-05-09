/**
 * @file pmsm_foc_error_handling.c
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
/**********************************************************************************************************************
 * HEADER FILES
 *********************************************************************************************************************/
#include "pmsm_foc_error_handling.h"
#include "pmsm_foc_interface.h"
#include "pmsm_foc_state_machine.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../MCUInit/6EDL_spi.h"
#include "../IPLib/pmsm_foc_ip.h"

/**********************************************************************************************************************
 * MACRO's
 *********************************************************************************************************************/
#define PMSM_FOC_MAX_CYCLE_TIME              (7500)

/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/

/**********************************************************************************************************************
 * API IMPLEMENTATION
 *********************************************************************************************************************/
/**
 * @brief   Check if any errors are cleared and change the control accordingly
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_ErrorHandling(void)
{
  static uint32_t cycle_counter = 0; /* Static variable and init. */

  if (MotorVar.fault_clear == 1U)
  {
    MotorVar.fault_clear = 0;
	  /******************************************************************************************************************
     * Error handling for CTRAP
     ******************************************************************************************************************/
    if ((MotorVar.error_status & PMSM_FOC_EID_CTRAP) && MotorVar.MaskedFault.Trap == 1)
    {
      /* If CCU8 TRAP has occurred, and fault_clear flag is set */
      cycle_counter++;

      if (cycle_counter > PMSM_FOC_MAX_CYCLE_TIME)
      {
        XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
        XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
        XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_IRQ_ID_EVENT2);

        /* Clear the error when fault_clear flag is set */
        NVIC_ClearPendingIRQ(TRAP_IRQn);
        MotorVar.error_status &= (~PMSM_FOC_EID_CTRAP);
        cycle_counter = 0;
      }
    }
    else
    {
      cycle_counter = 0; /* Clear counter. */
    }

    /******************************************************************************************************************
     * Error handling for Over Current Protection
     ******************************************************************************************************************/
    if ((MotorVar.error_status & PMSM_FOC_EID_OVER_CURRENT) && MotorVar.MaskedFault.OverCurrent == 1)
    {
      /* Reset the over current related variables & filters. */
      PMSM_FOC_PLL_ESTIMATOR.current_i_mag = 0;
      PMSM_FOC_PLL_ESTIMATOR.current_i_mag_filtered = 0;
      /* Clear the error when fault_clear flag is set */
      MotorVar.error_status &= (~PMSM_FOC_EID_OVER_CURRENT);
    }

    /******************************************************************************************************************
     * Error handling for DC bus Under/Over Voltage
     ******************************************************************************************************************/
    if ((MotorVar.error_status & PMSM_FOC_EID_OVER_VOLT) && (MotorVar.MaskedFault.OverVoltage == 1))
    {
      /* Clear the error when fault_clear flag is set */
      MotorVar.error_status &= (~PMSM_FOC_EID_OVER_VOLT);
    }
    if ((MotorVar.error_status & PMSM_FOC_EID_UNDER_VOLT) && (MotorVar.MaskedFault.UnderVoltage == 1))
    {
      /* Clear the error when fault_clear flag is set */
      MotorVar.error_status &= (~PMSM_FOC_EID_UNDER_VOLT);
    }

    /******************************************************************************************************************
     * Error handling for Torque limit exceed
     *****************************************************************************************************************/
    if (MotorVar.error_status & PMSM_FOC_EID_TORQUE_LIMIT_EXCEED)
    {
	    /* Clear the error when fault_clear flag is set */
      MotorVar.error_status &= (~PMSM_FOC_EID_TORQUE_LIMIT_EXCEED);
    }

    /******************************************************************************************************************
     * Error handling for 6EDL7141 Fault
     *****************************************************************************************************************/
    if ((MotorVar.error_status & PMSM_FOC_EID_NFAULT_FAULT)&& MotorVar.MaskedFault.Nfault == 1)
    {
      /* Clear all 6EDL7141 Faults including latched faults */
      write_word_16b(0x10, 0x03);   /* Write to 6EDL7141 */
      /* Clear the error when fault_clear flag is set */
      MotorVar.error_status &= (~PMSM_FOC_EID_NFAULT_FAULT);
    }
  }

  /* If all errors are cleared then go to STOP state */
  MotorVar.MaskedFault.Value = MotorVar.error_status & MotorParam.EnableFault.Value;
  if (MotorVar.MaskedFault.Value == 0U)
  {
	/* Disable low side braking in 6EDL7141 gate driver through nBRAKE pin */
	XMC_GPIO_SetOutputHigh(CYBSP_GD_NBRAKE);
	PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_STOP_MOTOR;
	PMSM_FOC_CTRL.msm_state = (1 << 6);//PMSM_FOC_MSM_IDLE;
  }

  PMSM_FOC_SVPWM_Update(0, 0);

} /* End of pmsm_foc_error_handling () */
