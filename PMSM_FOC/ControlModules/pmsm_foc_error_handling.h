/**
 * @file pmsm_foc_error_handling.h
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

#ifndef PMSM_FOC_ERROR_HANDLING_H_
#define PMSM_FOC_ERROR_HANDLING_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup ControlModules
 * @{
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * MACRO's
 ********************************************************************************************************************/
#define PMSM_FOC_EID_CTRAP_POS               (0U)
#define PMSM_FOC_EID_OVER_CURRENT_POS        (1U)
#define PMSM_FOC_EID_TORQUE_LIMIT_EXCEED_POS (3U)
#define PMSM_FOC_EID_INVERTER_OVER_TEMP_POS  (4U)
#define PMSM_FOC_EID_OVER_VOLT_POS           (6U)
#define PMSM_FOC_EID_UNDER_VOLT_POS          (10U)
#define PMSM_FOC_EID_SPI_FAULT_POS           (11U)
#define PMSM_FOC_EID_NFAULT_FAULT_POS        (12U)

/**********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * @brief This enumerates the error codes of the control which can occur during run-time.
 */
typedef enum PMSM_FOC_EID
{
  PMSM_FOC_EID_NO_ERROR            = 0,                                             /*!< Error ID 000 - NO ERROR */
  PMSM_FOC_EID_CTRAP               = (1U << PMSM_FOC_EID_CTRAP_POS),                /*!< Error ID 001 - CTRAP */
  PMSM_FOC_EID_OVER_CURRENT        = (1U << PMSM_FOC_EID_OVER_CURRENT_POS),         /*!< Error ID 002 - OVER CURRENT */
  PMSM_FOC_EID_TORQUE_LIMIT_EXCEED = (1U << PMSM_FOC_EID_TORQUE_LIMIT_EXCEED_POS),  /*!< Error ID 008 - TORQUE LIMIT EXCEED */
  PMSM_FOC_EID_INVERTER_OVER_TEMP  = (1U << PMSM_FOC_EID_INVERTER_OVER_TEMP_POS),   /*!< Error ID 016 - INVERTER OVER TEMPERATURE */
  PMSM_FOC_EID_OVER_VOLT           = (1U << PMSM_FOC_EID_OVER_VOLT_POS),            /*!< Error ID 064 - DC BUS OVER VOLTAGE */
  PMSM_FOC_EID_UNDER_VOLT          = (1U << PMSM_FOC_EID_UNDER_VOLT_POS),           /*!< Error ID 1024 - DC BUS UNDER VOLTAGE */
  PMSM_FOC_EID_SPI_FAULT           = (1U << PMSM_FOC_EID_SPI_FAULT_POS),            /*!< SPI communication with 6EDL7141 fault, error status = 2048 */
  PMSM_FOC_EID_NFAULT_FAULT        = (1U << PMSM_FOC_EID_NFAULT_FAULT_POS)          /*!< nFAULT from 6EDL7141 trigger, error status = 4096 */

} PMSM_FOC_EID_t;

/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/**
 * @brief	Check if any errors are cleared and change the control accordingly.
 *
 * @param	None
 *
 * @retval	None
 */
void PMSM_FOC_ErrorHandling(void);

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_INTERRUPTS_PMSM_FOC_ERROR_HANDLING_H_ */

