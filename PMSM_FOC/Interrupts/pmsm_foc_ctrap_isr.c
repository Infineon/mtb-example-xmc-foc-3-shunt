/**
 * @file pmsm_foc_ctrap_isr.c
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
#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../ControlModules/pmsm_foc_state_machine.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Interrupts
 * @{
 */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 * @brief Trap protection interrupt, e.g.: for over-current protection, triggered by CCU8x Event2.
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_CTRAP_ISR(void)
{
  #ifdef INVERTER_EN_PIN
  XMC_GPIO_SetOutputLevel(INVERTER_EN_PIN, USER_INVERTER_DISABLE_LEVEL); /* Disable gate driver. */
  #endif

  MotorVar.error_status |= PMSM_FOC_EID_CTRAP;

  /* Check error status against enable fault */
  MotorVar.MaskedFault.Value = MotorVar.error_status & MotorParam.EnableFault.Value;
  if (MotorVar.MaskedFault.Value != PMSM_FOC_EID_NO_ERROR)
  {
  /* Next go to ERROR state & wait until it get cleared */
  PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_ERROR;
  }

} /* End of CCU80_1_IRQHandler () */

/**
 * @}
 */

/**
 * @}
 */

