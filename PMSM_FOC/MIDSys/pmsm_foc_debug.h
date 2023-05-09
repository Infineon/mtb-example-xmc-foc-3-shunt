/**
 * @file pmsm_foc_debug.h
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

#ifndef PMSM_FOC_DEBUG_H_
#define PMSM_FOC_DEBUG_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/


/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MIDSys
 * @{
 */

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param In04      variable to be debug on P0.4     \n
 * @param In04_Flag 0: 0 < In04 < 2^In04_N           \n
 *                  1: -2^In04_N < In04 < 2^In04_N   \n
 * @param In04_N    resolution of the debug variable \n
 * @param In10      variable to be debug on P1.0     \n
 * @param In10_Flag 0: 0 < In10 < 2^In10_N           \n
 *                  1: -2^In10_N < In04 < 2^In10_N   \n
 * @param In10_N    resolution of the debug variable \n
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * To use CCU4 Debug with 2 Outputs, P0.4 and P1.0 <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
#if((DEBUG_PWM_0_ENABLE == 1U) || (DEBUG_PWM_1_ENABLE == 1U))
void PMSM_FOC_CCU4_Debug3output(int32_t In04, uint16_t In04_Flag, uint16_t In04_N, int32_t In10, uint16_t In10_Flag,
                       uint16_t In10_N);
#endif
void PMSM_FOC_UART_SetPOTADC(void);

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_MIDSYS_PMSM_FOC_DEBUG_H_ */

