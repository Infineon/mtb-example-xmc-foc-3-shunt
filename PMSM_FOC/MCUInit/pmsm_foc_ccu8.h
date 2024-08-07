/**
 * @file pmsm_foc_ccu8.h
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
#ifndef PMSM_FOC_CCU8_H_
#define PMSM_FOC_CCU8_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../ToolInterface/Register.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MCUInit
 * @{
 */
/*********************************************************************************************************************
 * MACRO
 ********************************************************************************************************************/
#define XMC_CCU8_GIDLC_CLOCK_MASK (15U)
#define XMC_CCU8_TC_TRAPSE_MASK   (15U)

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the CCU8 module for 3 phase pwm generation to turn the motor. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_CCU8_Init(void);


/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Synchronous start of CAPCOM modules <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
/* API to enable synchronous start of CAPCOM modules. */
__STATIC_INLINE void PMSM_FOC_CCUx_SyncStart(void)
{
  /* Enable Global Start Control CCU80 */
  XMC_SCU_SetCcuTriggerHigh(SCU_GENERAL_CCUCON_GSC80_Msk);
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_MCUINIT_PMSM_FOC_CCU8_H_ */

