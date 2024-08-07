/**
 * @file pmsm_foc_fcl_pwm_isr.c
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
#include "../ControlModules/pmsm_foc_speed_current_ctrl.h"
#include "../ControlModules/pmsm_foc_state_machine.h"
#include "../Configuration/pmsm_foc_common.h"
#include "../MIDSys/pmsm_foc_debug.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Interrupts
 * @{
 */

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API DEFINATION
 ********************************************************************************************************************/
/**
 * @brief Fast Control Loop(FCL)  - Interrupt occurs every PWM period
 *
 * @param None
 *
 * @retval None
 */
volatile int32_t iu,iv,iw;
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_FCL_ISR(void)
{
  /* Motor control state machine */
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif
  PMSM_FOC_MSM();
  iu = PMSM_FOC_INPUT.i_u;
  iv = PMSM_FOC_INPUT.i_v;
  iw = PMSM_FOC_INPUT.i_w;

  /* Debug purpose only */
  #if ((DEBUG_PWM_0_ENABLE == 1U) || (DEBUG_PWM_1_ENABLE == 1U))
  PMSM_FOC_CCU4_Debug3output(PMSM_FOC_OUTPUT.rotor_angle_q31, 1, 10, PMSM_FOC_OUTPUT.rotor_angle_q31, 1, 16);
  #endif

  /* ucProbe sampling function for ucProbe oscilloscope feature */
  #if (USER_UCPROBE_OSCILLOSCOPE == ENABLED)
  ProbeScope_Sampling();
  #endif

  PMSM_FOC_MiscWorks();

  /* System monitoring to detect configured errors */
  PMSM_FOC_SysMonitoring();
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

