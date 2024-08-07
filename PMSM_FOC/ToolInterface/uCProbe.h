/**
 * @file pmsm_foc_ucProbe.h
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
#ifndef UCPROBE_UCPROBE_H_
#define UCPROBE_UCPROBE_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../ToolInterface/ProbeScope/probe_scope.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../ToolInterface/Register.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup uCProbe
 * @{
 */
/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
//typedef union
//{
//  uint32_t user_config_array[50];
//  struct
//  {
//    uint32_t CONFIG_VALID_ID; /*!< Valid Configuration ID 0 */
//    uint32_t PI_SPEED_KP; /*!< USER_PI_SPEED_KP 1*/
//    uint32_t PI_SPEED_KI; /*!< USER_PI_SPEED_KI 2*/
//    uint32_t PI_SPEED_SCALE; /*!< USER_PI_SPEED_SCALE_KPKI 3*/
//    int32_t PI_SPEED_IK_LIMIT_MIN; /*!< USER_PI_SPEED_IK_LIMIT_MIN 4*/
//    int32_t PI_SPEED_IK_LIMIT_MAX; /*!< USER_PI_SPEED_IK_LIMIT_MAX 5*/
//    int32_t PI_SPEED_UK_LIMIT_MIN; /*!< USER_PI_SPEED_UK_LIMIT_MIN 6*/
//    int32_t PI_SPEED_UK_LIMIT_MAX; /*!< USER_PI_SPEED_UK_LIMIT_MAX 7*/
//
//    uint32_t PI_TORQUE_KP; /*!< USER_PI_TORQUE_KP 8*/
//    uint32_t PI_TORQUE_KI; /*!< USER_PI_TORQUE_KI 9*/
//    uint32_t PI_TORQUE_SCALE; /*!< USER_PI_TORQUE_SCALE_KPKI 10*/
//    int32_t PI_TORQUE_IK_LIMIT_MIN; /*!< USER_PI_TORQUE_IK_LIMIT_MIN 11*/
//    int32_t PI_TORQUE_IK_LIMIT_MAX; /*!< USER_PI_TORQUE_IK_LIMIT_MAX 12*/
//    int32_t PI_TORQUE_UK_LIMIT_MIN; /*!< USER_PI_TORQUE_UK_LIMIT_MIN 13*/
//    int32_t PI_TORQUE_UK_LIMIT_MAX; /*!< USER_PI_TORQUE_UK_LIMIT_MAX 14*/
//
//    uint32_t PI_FLUX_KP; /*!< USER_PI_FLUX_KP 15*/
//    uint32_t PI_FLUX_KI; /*!< USER_PI_FLUX_KI 16*/
//    uint32_t PI_FLUX_SCALE; /*!< USER_PI_FLUX_SCALE_KPKI 17*/
//    int32_t PI_FLUX_IK_LIMIT_MIN; /*!< USER_PI_FLUX_IK_LIMIT_MIN 18*/
//    int32_t PI_FLUX_IK_LIMIT_MAX; /*!< USER_PI_FLUX_IK_LIMIT_MAX 19*/
//    int32_t PI_FLUX_UK_LIMIT_MIN; /*!< USER_PI_FLUX_UK_LIMIT_MIN 20*/
//    int32_t PI_FLUX_UK_LIMIT_MAX; /*!< USER_PI_FLUX_UK_LIMIT_MAX 21*/
//
//    uint32_t PI_PLL_KP; /*!< USER_PI_PLL_KP 22*/
//    uint32_t PI_PLL_KI; /*!< USER_PI_PLL_KI 23*/
//    uint32_t PI_PLL_SCALE; /*!< USER_PI_PLL_SCALE_KPKI 24*/
//    int32_t PI_PLL_IK_LIMIT_MIN; /*!< USER_PI_PLL_IK_LIMIT_MIN 25*/
//    int32_t PI_PLL_IK_LIMIT_MAX; /*!< USER_PI_PLL_IK_LIMIT_MAX 26*/
//    int32_t PI_PLL_UK_LIMIT_MIN; /*!< USER_PI_PLL_UK_LIMIT_MIN 27*/
//    int32_t PI_PLL_UK_LIMIT_MAX; /*!< USER_PI_PLL_UK_LIMIT_MAX 28*/
//
//  };
//
//} USER_CONFIG_t;

/***********************************************************************************************************************
 * EXTERN
 **********************************************************************************************************************/
extern volatile uint32_t         ucProbe_cmd;
//extern USER_CONFIG_t USER_CONFIG;
//extern edl7141_register_t Edl7141Reg;

/***********************************************************************************************************************
 * API's PROTOTYPE
 **********************************************************************************************************************/

/**
 *
 * @brief Handling flash variable, if flash contain any valid data, write to actual variable from flash
 * @retval None
 */
void PMSM_FOC_uCProbe_Init(void);
void PMSM_FOC_ucProbe_ReadFlash(void);
void PMSM_FOC_ucProbe_WriteFlash(void);

/**
 *
 * @brief This function handle command from UI
 * @retval None
 */
void PMSM_FOC_ucProbe_CmdProcessing(void);

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define PMSM_SL_FOC_PI_CONFIG_ADDR                  (uint32_t *)0x10010200       /* Start address of control loop PI parameters storage on flash */
#define PMSM_SL_FOC_UCPROBE_MAX_PARAMETER                (40U)                        /* Maximum number of parameter to be saved in Flash*/
#define PMSM_SL_FOC_UCPROBE_FLASH_VALID_ID               (0x1235)                     /* 16 bit Validation ID to check whether flash has any valid data*/

/* Command used to communicate with GUI */
#define UCPROBE_CMD_CLEAR_ERROR                          (3U)
#define UCPROBE_CMD_WRITE_USERPARAM_FLASH                (4U)
#define UCPROBE_CMD_ERASE_USERPARAM_FLASH                (5U)
#define UCPROBE_CMD_LOAD_DFLT_PARAM                      (8U)
#define UCPROBE_CMD_READ_USERPARAM_FLASH                 (10U)
#define UCPROBE_CMD_WRITE_DFLT_PIPARAM_FLASH             (13U)
#define UCPROBE_CMD_READ_DFLT_PIPARAM_FLASH              (14U)
#define UCPROBE_CMD_RESET                                (15U)
#define UCPROBE_CMD_MOTOR_STOP                           (16U)

/**
 * @}
 */

/**
 * @}
 */

#endif /* UCPROBE_UCPROBE_H_ */
