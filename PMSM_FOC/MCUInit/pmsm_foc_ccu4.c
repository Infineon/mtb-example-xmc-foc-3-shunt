/**
 * @file pmsm_foc_ccu4.c
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
#include <PMSM_FOC/MCUInit/pmsm_foc_ccu4.h>

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

#if (( DEBUG_PWM_0_ENABLE) | (DEBUG_PWM_1_ENABLE))
/**
 *  Data Structure initialization - CCU4 Slice Configuration.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t Debug_CCU4_SliceConfig =
{
  .timer_mode = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot = XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear = 0U,
  .dither_timer_period = 0U,
  .dither_duty_cycle = 0U,
  .prescaler_mode = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable = 0U,
  .prescaler_initval = 0U, /* divide by 1--> 64MHz (as fast as possible) */
  .float_limit = 0U,
  .dither_limit = 0U,
  .passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH,
  .timer_concatenation = false
};
#endif /* ((DEBUG_PWM_0_ENABLE) | (DEBUG_PWM_1_ENABLE)) */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize CCU4 module to outputs debug information. Outputs P1. 0, P0.4, P1.2, P1.3 */
void PMSM_FOC_CCU4_Init(void)
{
  /* Init CCU40 */
  XMC_CCU4_Init(DEBUG_PWM_CCU4_MODULE, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  XMC_CCU4_SetModuleClock(DEBUG_PWM_CCU4_MODULE, XMC_CCU4_CLOCK_SCU);

  #if(DEBUG_PWM_0_ENABLE == 1U)
  /* Init Debug PWM Slice */
  /* Get the slice out of idle mode */
  XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_0_SLICE_NUM);
  /* Initialize the Slice */
  XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_0_SLICE, &Debug_CCU4_SliceConfig);

  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
  XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_0_SLICE, DEBUG_PWM_PERIOD_CNTS);
  XMC_CCU4_SLICE_SetTimerValue( DEBUG_PWM_0_SLICE, 0U);
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk);

  /* Setup the I/O Pin */
  XMC_GPIO_SetMode(DEBUG_PWM_0_PORT, DEBUG_PWM_0_PIN, DEBUG_PWM_0_ALT_OUT);
  XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_0_SLICE);
  #endif /* (DEBUG_PWM_0_ENABLE == 1) */

  #if(DEBUG_PWM_1_ENABLE == 1U)
  /* Init Debug PWM Slice */
  /* Get the slice out of idle mode */
  XMC_CCU4_EnableClock(DEBUG_PWM_CCU4_MODULE, DEBUG_PWM_1_SLICE_NUM);
  /* Initialize the Slice */
  XMC_CCU4_SLICE_CompareInit(DEBUG_PWM_1_SLICE, &Debug_CCU4_SliceConfig);

  XMC_CCU4_SLICE_SetTimerCompareMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_50_PERCENT_DC_CNTS);
  XMC_CCU4_SLICE_SetTimerPeriodMatch( DEBUG_PWM_1_SLICE, DEBUG_PWM_PERIOD_CNTS);
  XMC_CCU4_SLICE_SetTimerValue(DEBUG_PWM_1_SLICE, 0U);
  XMC_CCU4_EnableShadowTransfer(DEBUG_PWM_CCU4_MODULE, (uint32_t) DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk);

  /* Setup the I/O Pin */
  XMC_GPIO_SetMode(DEBUG_PWM_1_PORT, DEBUG_PWM_1_PIN, DEBUG_PWM_1_ALT_OUT);

  XMC_CCU4_SLICE_StartTimer(DEBUG_PWM_1_SLICE);
  #endif /* (DEBUG_PWM_1_ENABLE == 1) */

  XMC_CCU4_StartPrescaler(DEBUG_PWM_CCU4_MODULE);
}
