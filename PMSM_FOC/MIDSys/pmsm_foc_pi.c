/**
 * @file pmsm_foc_pi.c
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
#include "../IPLib/pmsm_foc_ip.h"
#include "pmsm_foc_pi.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
extern PI_CONFIG_t PI_CONFIG;

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/

PMSM_FOC_PI_t PMSM_FOC_SPEED_PI;   /* Speed PI controller. */
PMSM_FOC_PI_t PMSM_FOC_TORQUE_PI;  /* Torque / Iq PI controller. */
PMSM_FOC_PI_t PMSM_FOC_FLUX_PI;    /* Flux /Id PI controller. */
PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI; /* PLL Observer PI Controller */

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/* API to initialize PI Controller parameters */

void PMSM_FOC_PI_Init(void)
{
  /************** Speed PI controller  *********************************************/
  PMSM_FOC_SPEED_PI.kp = PI_CONFIG.PI_SPEED_KP;
  PMSM_FOC_SPEED_PI.ki = PI_CONFIG.PI_SPEED_KI;
  PMSM_FOC_SPEED_PI.scale_kp_ki = PI_CONFIG.PI_SPEED_SCALE;

  PMSM_FOC_SPEED_PI.ik_limit_min = PI_CONFIG.PI_SPEED_IK_LIMIT_MIN;
  PMSM_FOC_SPEED_PI.ik_limit_max = PI_CONFIG.PI_SPEED_IK_LIMIT_MAX;
  PMSM_FOC_SPEED_PI.uk_limit_min = PI_CONFIG.PI_SPEED_UK_LIMIT_MIN;
  PMSM_FOC_SPEED_PI.uk_limit_max = PI_CONFIG.PI_SPEED_UK_LIMIT_MAX;

  PMSM_FOC_SPEED_PI.uk = 0;
  PMSM_FOC_SPEED_PI.ik = 0;
  PMSM_FOC_SPEED_PI.error = 0;
  PMSM_FOC_SPEED_PI.sat_status = 0;

  /*************** Torque(Iq) PI controller  ****************************************/
  PMSM_FOC_TORQUE_PI.kp = PI_CONFIG.PI_TORQUE_KP;
  PMSM_FOC_TORQUE_PI.ki = PI_CONFIG.PI_TORQUE_KI;
  PMSM_FOC_TORQUE_PI.scale_kp_ki = PI_CONFIG.PI_TORQUE_SCALE;

  PMSM_FOC_TORQUE_PI.ik_limit_min = PI_CONFIG.PI_TORQUE_IK_LIMIT_MIN;
  PMSM_FOC_TORQUE_PI.ik_limit_max = PI_CONFIG.PI_TORQUE_IK_LIMIT_MAX;
  PMSM_FOC_TORQUE_PI.uk_limit_min = PI_CONFIG.PI_TORQUE_UK_LIMIT_MIN;
  PMSM_FOC_TORQUE_PI.uk_limit_max = PI_CONFIG.PI_TORQUE_UK_LIMIT_MAX;

  PMSM_FOC_TORQUE_PI.uk = 0;
  PMSM_FOC_TORQUE_PI.ik = 0;
  PMSM_FOC_TORQUE_PI.error = 0;
  PMSM_FOC_TORQUE_PI.sat_status = 0;

  /**************** Flux(Id) PI controller  ******************************************/
  PMSM_FOC_FLUX_PI.kp = PI_CONFIG.PI_FLUX_KP;
  PMSM_FOC_FLUX_PI.ki = PI_CONFIG.PI_FLUX_KI;
  PMSM_FOC_FLUX_PI.scale_kp_ki = PI_CONFIG.PI_FLUX_SCALE;

  PMSM_FOC_FLUX_PI.ik_limit_min = PI_CONFIG.PI_FLUX_IK_LIMIT_MIN;
  PMSM_FOC_FLUX_PI.ik_limit_max = PI_CONFIG.PI_FLUX_IK_LIMIT_MAX;
  PMSM_FOC_FLUX_PI.uk_limit_min = PI_CONFIG.PI_FLUX_UK_LIMIT_MIN;
  PMSM_FOC_FLUX_PI.uk_limit_max = PI_CONFIG.PI_FLUX_UK_LIMIT_MAX;

  PMSM_FOC_FLUX_PI.uk = 0;
  PMSM_FOC_FLUX_PI.ik = 0;
  PMSM_FOC_FLUX_PI.error = 0;
  PMSM_FOC_FLUX_PI.sat_status = 0;

  /***************** PLL Observer PI controller  **************************************/
  PMSM_FOC_PLL_PI.kp = PI_CONFIG.PI_PLL_KP;
  PMSM_FOC_PLL_PI.ki = PI_CONFIG.PI_PLL_KI;
  PMSM_FOC_PLL_PI.scale_kp_ki = PI_CONFIG.PI_PLL_SCALE;

  PMSM_FOC_PLL_PI.ik_limit_min = (int32_t)PI_CONFIG.PI_PLL_IK_LIMIT_MIN;
  PMSM_FOC_PLL_PI.ik_limit_max = (int32_t)PI_CONFIG.PI_PLL_IK_LIMIT_MAX;
  PMSM_FOC_PLL_PI.uk_limit_min = (int32_t)PI_CONFIG.PI_PLL_UK_LIMIT_MIN;
  PMSM_FOC_PLL_PI.uk_limit_max = (int32_t)PI_CONFIG.PI_PLL_UK_LIMIT_MAX;

  PMSM_FOC_PLL_PI.uk = 0;
  PMSM_FOC_PLL_PI.ik = 0;
  PMSM_FOC_PLL_PI.error = 0;

}	/* End of pmsm_foc_pi_controller_init () */
