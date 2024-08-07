/**
 * File Name: main.c
 *
 * Description: main file of the FOC IMD700A 3 shunt application
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
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
 *******************************************************************************/

/***********************************************************************************************************************
 * Hardware setup for sensorless FOC test
 **********************************************************************************************************************/
/* Microcontroller:          XMC1404-Q064x0128, IMD701A-Q064X128
 * Power boards:             EVAL_6EDL7141_FOC_3SH, EVAL_IMD700A_FOC_3SH
 *
 * DB42S03 motor connection: Yellow     -> U of inverter
 *                           Red        -> V
 *                           Black      -> W
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include <xmc_common.h>
#include "PMSM_FOC/ControlModules/pmsm_foc_state_machine.h"
#include "cybsp.h"
#include "cy_utils.h"
#include "PMSM_FOC/Configuration/pmsm_foc_mcuhw_params.h"
#include "PMSM_FOC/Configuration/pmsm_foc_user_input_config.h"
#include "PMSM_FOC/ControlModules/pmsm_foc_functions.h"
#include "PMSM_FOC/MCUInit/6EDL_gateway.h"
#include "PMSM_FOC/MIDSys/pmsm_foc_debug.h"
#include "PMSM_FOC/ToolInterface/Register.h"



/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
/* User Parameter Stored on the Flash */

uint8_t idle_state_delay_counter;    /* have to add extra 3ms delay due to 6EDL7141 watchdog clock */

extern void PMSM_FOC_SYSTICK_TimerInit(void);
extern void EDL7141_Config_init(void);

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 @brief
 This example project controls 3-phase BLDC motor with sensorless field oriented control algorithm.
 This is configured for Infineon EVAL_IMD700A_FOC_3SH board with Nanotec motor part number DB42S03

 * Change of configurations:
 * Control algorithm configurations are in the Configuration/pmsm_foc_user_input_config.h file.
 * MCU HW resources configurations are in the Configuration/pmsm_foc_mcuhw_params.h file.
 */
int main(void)
{
  /*************************************************
   * Initialize the device and bsp configurations
   * **********************************************/
  cy_rslt_t result = cybsp_init();
  if (result != CY_RSLT_SUCCESS)
  {
      CY_ASSERT(0);
  }
  /* Reset status bits */

  SystemVar.Status = 0;

  /* Read chip ID, XMC1400: 0x00014xxx, IMD700A: 0x2700100A */
  PMSM_FOC_VADC_ModuleInit();
  PMSM_FOC_VADC_KitIDInit();
  MC_Info_init();
  while (((MC_INFO.chip_id >> 12) != 0x14) && (MC_INFO.chip_id != 0x2700100A)&& (MC_INFO.chip_id != 0x2701100A)) 
  {
    /* code stop here if not running on expected platform */
  }

  /* load parameter from flash, set ParamConfigured to 1 if load was succeed */
  FLASH_Parameter_load();

  /* load 6EDL7141 parameter from flash, set Edl7141Configured to 1 if load was succeed */
  EDL7141_FLASH_parameter_load();

  /* Initialize SPI interface with 6EDL7141, and 6EDL7141 related IO */
  EDL7141_Config_init();

  /* Put motor control state machine to IDLE state */
  idle_state_delay_counter = 0;
  PMSM_FOC_CTRL.msm_state = PMSM_FOC_MSM_IDLE;

  /* Initializes systick at 1ms - this will start the state machine interrupt */
  SystemVar.GlobalTimer = 0; /* reset timer */
  PMSM_FOC_SYSTICK_TimerInit();



  /* Placeholder for user application code. The while loop below can be replaced with user application code. */
  while (1)
  {
    /* Placeholder for user application code. */
  }
}
/* End of main () */
