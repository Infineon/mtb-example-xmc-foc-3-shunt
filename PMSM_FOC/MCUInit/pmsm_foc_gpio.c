/**
 * @file pmsm_foc_gpio.c
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

#include "../MCUInit/pmsm_foc_gpio.h"
#include <xmc_ccu8.h>

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - GPIO Configuration for Gate Driver enable pin .
 */
XMC_GPIO_CONFIG_t Inverter_Enable_Pin_Config  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
  .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)USER_INVERTER_DISABLE_LEVEL
};

/**
 *  Data Structure initialization - GPIO Configuration for Brake enable pin .
 */
XMC_GPIO_CONFIG_t Brake_Enable_Pin_Config  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
  .output_level    = (XMC_GPIO_OUTPUT_LEVEL_t)USER_BRAKE_DISABLE_LEVEL
};


XMC_GPIO_CONFIG_t AUTO_ZERO_Pin_Config  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_INPUT_PULL_UP,
  .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};

XMC_GPIO_CONFIG_t nFault_Pin_Config  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_INPUT_TRISTATE,
  .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};

/**
 *  Data Structure initialization - GPIO Configuration for motor Direction pin .
 */
XMC_GPIO_CONFIG_t Motor_Dir_Pin_Config  =
{
  .mode            = (XMC_GPIO_MODE_t)XMC_GPIO_MODE_INPUT_PULL_DOWN,
  .input_hysteresis= XMC_GPIO_INPUT_HYSTERESIS_STANDARD,
};

/** GPIO Init handle for Phase U High Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhU_High_Config =
{
  .mode             = PHASE_U_HS_ALT_SELECT,
  #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
  #else
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
  #endif
  .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase U Low Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhU_Low_Config =
{
  .mode             = PHASE_U_LS_ALT_SELECT,
  #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
  #else
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
  #endif
  .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase V High Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhV_High_Config =
{
  .mode             = PHASE_V_HS_ALT_SELECT,
  #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
  #else
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
  #endif
 .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase V Low Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhV_Low_Config =
{
  .mode             = PHASE_V_LS_ALT_SELECT,
  #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
  #else
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
  #endif
  .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase W High Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhW_High_Config =
{
  .mode             = PHASE_W_HS_ALT_SELECT,
  #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
  #else
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
  #endif
  .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/** GPIO Init handle for Phase W Low Pin */
const XMC_GPIO_CONFIG_t PMSM_FOC_GPIO_PhW_Low_Config =
{
  .mode             = PHASE_W_LS_ALT_SELECT,
  #if(USER_CCU8_PASSIVE_LEVEL_OUT0 == CCU8_PASSIVE_LOW)
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_LOW,
  #else
  .output_level     = XMC_GPIO_OUTPUT_LEVEL_HIGH,
  #endif
  .input_hysteresis = (XMC_GPIO_INPUT_HYSTERESIS_t)0
};

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize GPIO pins used */
void PMSM_FOC_GPIO_Init(void)
{
	/* GPIO defination for INVERTER_EN_PIN, BRAKE_EN_PIN, AUTO_ZERO, DRV_CLK_EN_PIN are declared at EDL7141_Config_init() */
//  #ifdef INVERTER_EN_PIN
//  /* Gate driver enable/disable pin */
//  XMC_GPIO_Init(INVERTER_EN_PIN, &Inverter_Enable_Pin_Config);
//  #endif
//
  #ifdef CYBSP_GD_NBRAKE
  /* Brake enable/disable pin */
  XMC_GPIO_Init(CYBSP_GD_NBRAKE, &Brake_Enable_Pin_Config);
  #endif

  #ifdef AUTO_ZERO
  /* over current indicator pin, input to XMC  */
  XMC_GPIO_Init(AUTO_ZERO, &AUTO_ZERO_Pin_Config);
  #endif

  #ifdef nFAULT_PIN
  /* 6EDL7141 nFault pin  */
  XMC_GPIO_Init(nFAULT_PIN, &nFault_Pin_Config);
  #endif

  #ifdef MOTOR_DIR_INPUT_PIN
  /* motor direction control PIN */
  XMC_GPIO_Init(MOTOR_DIR_INPUT_PIN, &Motor_Dir_Pin_Config);
  #endif

  /* Phase U High Side PWM Output */
  XMC_GPIO_Init(PHASE_U_HS_PIN, &PMSM_FOC_GPIO_PhU_High_Config);
  /* Phase U Low Side PWM Output */
  XMC_GPIO_Init(PHASE_U_LS_PIN, &PMSM_FOC_GPIO_PhU_Low_Config);

  /* Phase V High Side PWM Output */
  XMC_GPIO_Init(PHASE_V_HS_PIN, &PMSM_FOC_GPIO_PhV_High_Config);

  /* Phase V Low Side PWM Output */
  XMC_GPIO_Init(PHASE_V_LS_PIN, &PMSM_FOC_GPIO_PhV_Low_Config);

  /* Phase W High Side PWM Output */
  XMC_GPIO_Init(PHASE_W_HS_PIN, &PMSM_FOC_GPIO_PhW_High_Config);

  /* Phase W Low Side PWM Output *//* P0.9 ALT5 CCU80.OUT21 */
  XMC_GPIO_Init(PHASE_W_LS_PIN, &PMSM_FOC_GPIO_PhW_Low_Config);

  /* Debugging only - ADC trigger */
  //XMC_GPIO_SetMode(PHASE_ADC_TRIG_PIN, PHASE_ADC_TRIG_PIN_ALT_SELECT);

  #ifdef TRAP_PIN
  #if(USER_CCU8_INPUT_TRAP_LEVEL == XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW)
  /* Trap input, internal pull-down */
  XMC_GPIO_SetMode(TRAP_PIN, XMC_GPIO_MODE_INPUT_PULL_UP);
  #else
  /* Trap input, internal pull-up */
  XMC_GPIO_SetMode(TRAP_PIN, XMC_GPIO_MODE_INPUT_PULL_DOWN);
  #endif
  #endif

  /* Test 1 PIN */
  #ifdef TEST_PIN1
  XMC_GPIO_SetMode (TEST_PIN1,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
  XMC_GPIO_SetOutputHigh(TEST_PIN1);
  #endif

  /* Test 2 PIN */
  #ifdef TEST_PIN2
  XMC_GPIO_SetMode (TEST_PIN2,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
  XMC_GPIO_SetOutputHigh(TEST_PIN2);
  #endif
}
