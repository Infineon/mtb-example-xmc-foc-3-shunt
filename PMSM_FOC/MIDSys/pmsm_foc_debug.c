/**
 * @file pmsm_foc_debug.c
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
#include "../MIDSys/pmsm_foc_debug.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
#if((DEBUG_PWM_0_ENABLE == 1U) || (DEBUG_PWM_1_ENABLE == 1U))
/* API to use CCU4 Debug with 2 Outputs, P0.4 and P1.0 */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_CCU4_Debug3output(int32_t In04, uint16_t In04_Flag, uint16_t In04_N, int32_t In10, uint16_t In10_Flag,
                       uint16_t In10_N)
{
  int32_t Tmp_CRS;                               /* Tmp for CCU4 debug. */
  #if(DEBUG_PWM_0_ENABLE == 1U)
  /* Update CCU40.OUT0 (P1.0) duty-cycle for debug. */
  if (In10_Flag == 0U)
  {
    Tmp_CRS = (int32_t)(In10 >> In10_N); /* In10 is a positive integer, In10 < 2^In10_N. */
  }
  else
  {
    /* In10 is a positive or negative integer, -2^In10_N < In10 < 2^In10_N. */
    Tmp_CRS = ((In10 + (1U << In10_N))) >> (In10_N + 1U);
  }

  if (Tmp_CRS < 0)
  {
    Tmp_CRS = REVERSE_CRS_OR_0;
  }

  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_0_SLICE, Tmp_CRS);     /* Update CCU41 Shadow Compare Register. */
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t)DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk);
  #endif /* (DEBUG_PWM_0_ENABLE == 1) */

  #if   (DEBUG_PWM_1_ENABLE == 1U)
  /* Update CCU40.OUT1 (P0.4) duty-cycle for debug. */
  if (In04_Flag == 0U)
  {
    Tmp_CRS = (In04 * DEBUG_PWM_PERIOD_CNTS) >> In04_N; /* In04 is a positive integer. */
  }
  else
  {
    /* In04 is a positive or negative integer. */
    Tmp_CRS = ((In04 + (1U << In04_N)) * DEBUG_PWM_PERIOD_CNTS) >> (In04_N + 1U);
  }

  if (Tmp_CRS < 0)
  {
    Tmp_CRS = REVERSE_CRS_OR_0;
  }
  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_1_SLICE, Tmp_CRS);     /* Update CCU41 Shadow Compare Register. */

  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t)DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk);
#endif /* (DEBUG_PWM_1_ENABLE == 1) */

}   /* End of pmsm_foc_ccu4_debug3output () */

#endif /* #if((DEBUG_PWM_0_ENABLE == 1U) || (DEBUG_PWM_1_ENABLE == 1U)) */


