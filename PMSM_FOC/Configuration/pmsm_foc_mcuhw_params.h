/**
 * @file pmsm_foc_mcuhw_params.h
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

#ifndef PMSM_FOC_MCUCARD_PARAMETERS_H_
#define PMSM_FOC_MCUCARD_PARAMETERS_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "pmsm_foc_user_input_config.h"
#include <xmc_vadc.h>
#include <xmc_ccu4.h>
#include <xmc_ccu8.h>
#include <xmc_scu.h>
#include <xmc_gpio.h>
#include <xmc_math.h>
#include <xmc_wdt.h>
#include "xmc1_gpio_map.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Configuration
 * @{
 */
/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * GPIO Resources Configuration
 ********************************************************************************************************************/
//#define TRAP_PIN               P0_12 /* Active High */
#define nFAULT_PIN             P3_4     /* Active Low */
#define CYBSP_GD_NBRAKE        P1_3     /* Active Low */
#define INVERTER_EN_PIN        P0_7     /* Active High to enable gate driver*/
#define BRAKE_EN_PIN           P1_3     /* Active Low for motor braking*/
#define AUTO_ZERO_PIN          P1_2     /* Input pin to control Auto-Zero function */
#define DRV_CLK_EN_PIN         P2_13    /* Watchdog clock on EN_DRV pin */
#define OFFSTATE_DIAG_EN_PIN   P4_9     /* Active High to provide bias to SHx node */

#define MOTOR_DIR_INPUT_PIN    P4_10

#define PHASE_U_HS_PIN         P3_3
#define PHASE_U_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_U_LS_PIN         P3_2
#define PHASE_U_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_HS_PIN         P3_1
#define PHASE_V_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_V_LS_PIN         P3_0
#define PHASE_V_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_HS_PIN         P1_0
#define PHASE_W_HS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define PHASE_W_LS_PIN         P1_1
#define PHASE_W_LS_ALT_SELECT  XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5

#define TEST_PIN1               P4_7
#define TEST_PIN2               P4_8

/*********************************************************************************************************************
 * CCU8 Resources Configuration
 ********************************************************************************************************************/
#define CCU8_MODULE             CCU80
#define CCU8_MODULE_NUM         (0U)

#define CCU8_SLICE_PHASE_U      CCU80_CC81
#define CCU8_SLICE_PHASE_U_NUM  (1U)

#define CCU8_SLICE_PHASE_V      CCU80_CC82
#define CCU8_SLICE_PHASE_V_NUM  (2U)

#define CCU8_SLICE_PHASE_W      CCU80_CC80
#define CCU8_SLICE_PHASE_W_NUM  (0U)

#define CCU8_SLICE_ADC_TR       CCU80_CC83
#define CCU8_SLICE_ADC_TR_NUM   (3U)

/*********************************************************************************************************************
 * VADC Resources Configuration
 ********************************************************************************************************************/
/* For simultaneous sampling */
#define VADC_I1_GROUP         VADC_G1
#define VADC_I1_CHANNEL       (0U)
#define VADC_I1_RESULT_REG    (0U)

#define VADC_I2_GROUP         VADC_G0
#define VADC_I2_CHANNEL       (0U)
#define VADC_I2_RESULT_REG    (0U)

#define VADC_I3_GROUP         VADC_G1
#define VADC_I3_CHANNEL       (1U)
#define VADC_I3_RESULT_REG    (1U)

#define VADC_I4_GROUP         VADC_G0
#define VADC_I4_CHANNEL       (1U)
#define VADC_I4_RESULT_REG    (1U)

/* Motor Phase U VADC define */
#define VADC_IU_G1_CHANNEL    (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_G0_CHANNEL    (4U)        /* P2.11, VADC group0 channel 4 */

#define VADC_IU_GROUP         VADC_G1
#define VADC_IU_GROUP_NO      (1U)
#define VADC_IU_CHANNEL       (3U)        /* P2.11, VADC group1 channel 3 */
#define VADC_IU_RESULT_REG    (3U)

/* Motor Phase V VADC define */
#define VADC_IV_G1_CHANNEL    (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_G0_CHANNEL    (3U)       /* P2.10, VADC group0 channel 3 */

#define VADC_IV_GROUP         VADC_G1
#define VADC_IV_GROUP_NO      (1U)
#define VADC_IV_CHANNEL       (2U)       /* P2.10, VADC group1 channel 2 */
#define VADC_IV_RESULT_REG    (2U)

/* Motor Phase W VADC define */
#define VADC_IW_G1_CHANNEL    (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_G0_CHANNEL    (2U)       /* P2.9, VADC group0 channel 2 */

#define VADC_IW_GROUP         VADC_G1
#define VADC_IW_GROUP_NO      (1U)
#define VADC_IW_CHANNEL       (4U)       /* P2.9, VADC group1 channel 4 */
#define VADC_IW_RESULT_REG    (4U)

/* DC link voltage VADC define */
#define VADC_VDC_GROUP        VADC_G1
#define VADC_VDC_GROUP_NO     (1U)
#define VADC_VDC_CHANNEL      (5U)      /* P2.3 VADC group1 channel 5 */
#define VADC_VDC_RESULT_REG   (5U)

#if((USER_OVERCURRENT_PROTECTION == ENABLED) && (USER_OVER_CURRENT_DETECTION_SOURCE == DC_LINK_CURRENT_SENSE))
/* DC link current VADC define */
#define VADC_IDC_GROUP        VADC_G1
#define VADC_IDC_GROUP_NO     (1U)
#define VADC_IDC_CHANNEL      (6U)       /* P2.4 VADC group1 channel 6 */
#define VADC_IDC_RESULT_REG   (6U)
#endif

/* T_Sense Temp sensor VADC define*/
#define VADC_TEMP_GROUP       VADC_G1
#define VADC_TEMP_GROUP_NO    (1U)
#define VADC_TEMP_CHANNEL     (6U)      /* P2.4 VADC group1 channel 6 */
#define VADC_TEMP_RESULT_REG  (6U)

/* Potentiometer VADC define*/
#define VADC_POT_GROUP        VADC_G1
#define VADC_POT_GROUP_NO     (1U)
#define VADC_POT_CHANNEL      (7U)      /* P2.5 VADC group1 channel 7 */
#define VADC_POT_RESULT_REG   (7U)

/** Configuration Macros for reference kit select Channel */
#define KIT_SELECT_GRP        (VADC_G0)
#define KIT_SELECT_GRP_NO     (0U)
#define KIT_SELECT_CH_NUM     (0U)      /* P2.6, G0CH0 */
#define KIT_SELEC_RESULT_REG  (0U)

/* Sectors A. ADC sequences - Iv -> Iw -> Iu */
/* VADC Group 0 Alias channel 0 and channel 1 */
#define VADC_G0_CHANNEL_ALIAS0  VADC_IW_G0_CHANNEL
#define VADC_G0_CHANNEL_ALIAS1  0

/* VADC Group 1 Alias channel 0 and channel 1 */
#define VADC_G1_CHANNEL_ALIAS0  VADC_IV_G1_CHANNEL
#define VADC_G1_CHANNEL_ALIAS1  VADC_IU_G1_CHANNEL

#define INTERNAL_OP_GAIN          DISABLED                       /*1. ENABLED       2. DISABLED (Please configure OP-Gain manually) */
#if(INTERNAL_OP_GAIN == ENABLED)
#define OP_GAIN_FACTOR            (3U)                           /* Different HW Board has different OP Gain factor, XMC13/XMC14 built-in Gain Factor available 1, 3, 6 and 12 only*/
#elif(INTERNAL_OP_GAIN == DISABLED)
#define OP_GAIN_FACTOR            (12U)                          /* External opamp gain factor */
#endif


/* ********************************************************************************************************************/
/* NVIC Interrupt Resources Configuration */
/* ********************************************************************************************************************/
#define CCU80_0_IRQn                                    IRQ25_IRQn          /*!< CCU80 SR0 Interrupt  */
#define TRAP_IRQn                                       IRQ26_IRQn          /*!< CCU80 SR1 Interrupt  */
#define FAULT_PIN_ERU_IRQn                              IRQ6_IRQn           /*!< ERU */

#define VADC0_G0_1_IRQn                                 IRQ18_IRQn          /*!< VADC0.G0SR1          */
#define VADC0_G1_1_IRQn                                 IRQ20_IRQn          /*!< VADC0.G1SR1          */

/* NVIC ISR handler mapping */
#define PMSM_FOC_FCL_ISR                                IRQ25_Handler       /*!< Fast Control Loop(FCL) - PWM Period Match*/
#define PMSM_FOC_SCL_ISR                                SysTick_Handler     /*!< Slow Control Loop(SCL) - Systick */
#define PMSM_FOC_CTRAP_ISR                              IRQ26_Handler       /*!< CTRAP - CCU8 - PWM */
#define PMSM_FOC_DRIVER_nFAULT_ISR                      IRQ6_Handler        /*!< ERU - 6EDL7141 nFault pin interrupt handler */


/* ********************************************************************************************************************/
/* NVIC Interrupt Priority Configuration */
/* ********************************************************************************************************************/
/* Interrupt priority configurations - 0 is the highest priority and 3 is the lowest */
#define PMSM_FOC_FCL_NVIC_PRIO                          (2U)  /*!< FAST Control loop - Executed every PWM period */
#define PMSM_FOC_SCL_NVIC_PRIO                          (3U)  /*!< Slow Control loop  - SysTick */
#define PMSM_FOC_CTRAP_NVIC_PRIO                        (0U)  /*!< CTRAP   */
#define PMSM_FOC_FAULT_NVIC_PRIO                        (1U)  /*!< nFault from 6EDL7141 */


/*********************************************************************************************************************
 * DEBUGGING ENABLE Resources Configuration
 ********************************************************************************************************************/
#define DEBUG_PWM_0_ENABLE                              (0U)        /* 1 = Enable Debug PWM, 0 = Disable Debug PWM */
#define DEBUG_PWM_1_ENABLE                              (0U)        /* 1 = Enable Debug PWM, 0 = Disable Debug PWM */

#define DEBUG_PWM_CCU4_MODULE                           (CCU40)

/* Debug Period Value controls the resolution of the PWM.
 * This is the value that goes into the PWM period register.
 */
#define DEBUG_PWM_PERIOD_CNTS                           (4800U)

/* Initial Duty Cycle of Debug PWM Channels */
#define DEBUG_PWM_50_PERCENT_DC_CNTS                    ((uint16_t)(DEBUG_PWM_PERIOD_CNTS >> 1))

/* P4.8 */
#if (DEBUG_PWM_0_ENABLE == 1U)
#define DEBUG_PWM_0_SLICE                              (CCU40_CC40)
#define DEBUG_PWM_0_SLICE_NUM                          (0U)
#define DEBUG_PWM_0_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_0)
#define DEBUG_PWM_0_PORT                               (XMC_GPIO_PORT4)
#define DEBUG_PWM_0_PIN                                (8U)
#define DEBUG_PWM_0_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6)
#endif  /*DEBUG_PWM_0_ENABLE == 1*/

/* P0.4 */
#if (DEBUG_PWM_1_ENABLE == 1U)
#define DEBUG_PWM_1_SLICE                              (CCU40_CC43)
#define DEBUG_PWM_1_SLICE_NUM                          (3U)
#define DEBUG_PWM_1_SLICE_SHADOW_TRANS_ENABLE_Msk      (XMC_CCU4_SHADOW_TRANSFER_SLICE_3)
#define DEBUG_PWM_1_PORT                               (XMC_GPIO_PORT4)
#define DEBUG_PWM_1_PIN                                (3U)
#define DEBUG_PWM_1_ALT_OUT                            (XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT5)
#endif  /*DEBUG_PWM_1_ENABLE == 1 */

/* Tmp_CRS = 0 or (- Tmp_CRS) if Tmp_CRS < 0. */
#define REVERSE_CRS_OR_0  (- Tmp_CRS)

/**
 * @}
 */

/**
 * @}
 */
#endif /* PMSM_FOC_MCUCARD_PARAMETERS_H_ */
