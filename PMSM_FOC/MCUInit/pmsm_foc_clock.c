/**
 * @file pmsm_foc_clock.c
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
#include "../MCUInit/pmsm_foc_clock.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - Clock Configuration.
 */
XMC_SCU_CLOCK_CONFIG_t Clock_Config =
{
    #if(UC_SERIES == XMC14)
        .fdiv       = 0U,  /**< 8/10 Bit Fractional divider */
        .idiv       = 1U,  /**< 8 Bit integer divider */

        .dclk_src   = XMC_SCU_CLOCK_DCLKSRC_DCO1,
        .oschp_mode = XMC_SCU_CLOCK_OSCHP_MODE_DISABLED,
        .osclp_mode = XMC_SCU_CLOCK_OSCLP_MODE_DISABLED,
        .pclk_src   = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK,
        .rtc_src    = XMC_SCU_CLOCK_RTCCLKSRC_DCO2
    #else
        .idiv       = 0x01U,
        .pclk_src   = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK,
        .rtc_src    = XMC_SCU_CLOCK_RTCCLKSRC_DCO2
    #endif
};

/***********************************************************************************************************************
 * GLOBAL DATA
***********************************************************************************************************************/
/* Global variable, MCU Reset Status Information, reason of last reset */
uint32_t MCUResetStatus;    // Global variable. MCU Reset Status Information, reason of last reset.

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize clock module and read reset status */
void PMSM_FOC_Clock_Init(void)
{
    uint32_t reset_status;

    /* Reset status, get reason of last reset */
    reset_status = XMC_SCU_RESET_GetDeviceResetReason();

    /* Record MCU Reset Status Information by a global variable */
    MCUResetStatus = reset_status;

    /* Clear reset status, to ensure a clear indication of cause of next reset */
    XMC_SCU_RESET_ClearDeviceResetReason();

    /* Enable reset triggered by critical events: Flash ECC error, loss of clock, 16kbytes SRAM parity error
     * (comment out SRAM parity error to prevent system reset trigger by GUI reading MC_INFO variables) */
    XMC_SCU_RESET_EnableResetRequest((uint32_t)XMC_SCU_RESET_REQUEST_FLASH_ECC_ERROR | (uint32_t)XMC_SCU_RESET_REQUEST_CLOCK_LOSS );

    /* 32MHz MCLK, PCLK = 2 x MCLK = 64MHz, RTC clock is standby clock, Counter Adjustment = 1024 clock cycles */
    XMC_SCU_CLOCK_Init(&Clock_Config);

}
