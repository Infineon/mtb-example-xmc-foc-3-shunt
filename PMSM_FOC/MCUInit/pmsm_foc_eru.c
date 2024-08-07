/**
 * @file pmsm_foc_eru.c
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
#include <PMSM_FOC/MCUInit/pmsm_foc_eru.h>

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/*
 * Data Structure initialization - ERU Configuration.
 */

XMC_ERU_ETL_CONFIG_t XMC_ERU_ETL_CONFIG =
  {
    .input_a = (uint32_t)XMC_ERU_ETL_INPUT_A3, /* Event input selection for A(0-3) */
    .input_b = (uint32_t)XMC_ERU_ETL_INPUT_B0, /* Event input selection for B(0-3) */
    .enable_output_trigger = (uint32_t)1,
    .edge_detection = XMC_ERU_ETL_EDGE_DETECTION_FALLING, /* Select the edge to convert as event */
    .output_trigger_channel = XMC_ERU_ETL_OUTPUT_TRIGGER_CHANNEL3, /* Select the source for event */
    .source = XMC_ERU_ETL_SOURCE_A
  };
/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize the Fault pin interrupt with ERU */
void PMSM_FOC_PIN_INTERRUPT_Init(void)
{
  /* ERU Event Trigger Logic Hardware initialization */
  XMC_ERU_ETL_Init(XMC_ERU1, 2U, &XMC_ERU_ETL_CONFIG);

  /* OGU is configured to generate event on configured trigger edge */
  XMC_ERU_OGU_SetServiceRequestMode(XMC_ERU1, 3U, XMC_ERU_OGU_SERVICE_REQUEST_ON_TRIGGER);

  NVIC_SetPriority (FAULT_PIN_ERU_IRQn, PMSM_FOC_FAULT_NVIC_PRIO);
  XMC_SCU_SetInterruptControl(FAULT_PIN_ERU_IRQn, XMC_SCU_IRQCTRL_ERU1_SR3_IRQ6);
  NVIC_EnableIRQ(FAULT_PIN_ERU_IRQn);
}

