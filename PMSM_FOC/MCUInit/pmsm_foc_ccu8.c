/**
 * @file pmsm_foc_ccu8.c
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
#include "../MCUInit/pmsm_foc_ccu8.h"
#include "../ToolInterface/uCProbe.h"
#include "../ToolInterface/Register.h"
#include "xmc_common.h"

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - CC8 Slices configuration for all three pwm phase.
 */
XMC_CCU8_SLICE_COMPARE_CONFIG_t CCU8_Phase_Init =
{
  .timer_mode           = (uint32_t)XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot             = (uint32_t)XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear    = 1U,     /* CLST = 1b, Enable a shadow transfer whenever a clearing action by SW or external event */
  .dither_timer_period  = 0U,
  .dither_duty_cycle    = 0U,
  .prescaler_mode       = (uint8_t)XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_ch1_enable       = 0U,
  .mcm_ch2_enable       = 0U,
  .slice_status         = (uint8_t)XMC_CCU8_SLICE_STATUS_CHANNEL_1,
  .passive_level_out0   = USER_CCU8_PASSIVE_LEVEL_OUT0,
  .passive_level_out1   = USER_CCU8_PASSIVE_LEVEL_OUT1,
  .asymmetric_pwm       = (uint8_t)true,
  #if(UC_SERIES == XMC14)
  .selector_out0         = XMC_CCU8_SOURCE_OUT0_INV_ST1,
  .selector_out1         = XMC_CCU8_SOURCE_OUT1_ST1,
  .selector_out2         = DISABLED,
  .selector_out3         = DISABLED,
  #else
  .invert_out0          = 1U,
  .invert_out1          = 0U,
  #endif
  .prescaler_initval    = (uint8_t)0,
  .float_limit          = 0U,
  .dither_limit         = 0U,
  .timer_concatenation  = 0U

};

/**
 *  Data Structure initialization - CC8 Slice configuration to trigger ADC conversion start.
 */
XMC_CCU8_SLICE_COMPARE_CONFIG_t CCU8_VADC_Trigger_Init =
{
  .timer_mode           = (uint32_t)XMC_CCU8_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot             = (uint32_t)XMC_CCU8_SLICE_TIMER_REPEAT_MODE_REPEAT,
  .shadow_xfer_clear    = 1U,
  .dither_timer_period  = 0U,
  .dither_duty_cycle    = 0U,
  .prescaler_mode       = (uint8_t)XMC_CCU8_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_ch1_enable       = 0U,
  .mcm_ch2_enable       = 0U,
  .slice_status         = (uint8_t)XMC_CCU8_SLICE_STATUS_CHANNEL_1,
  .passive_level_out0   = 0,
  .passive_level_out1   = 0,
  .asymmetric_pwm       = (uint8_t)true,
  #if(UC_SERIES == XMC14)
  .selector_out0         = XMC_CCU8_SOURCE_OUT1_INV_ST1,
  .selector_out1         = XMC_CCU8_SOURCE_OUT1_ST1,
  .selector_out2         = DISABLED,
  .selector_out3         = DISABLED,
  #else
  .invert_out0          = 0U,
  .invert_out1          = 1U,
  #endif
  .prescaler_initval    = (uint8_t)0,
  .float_limit          = 0U,
  .dither_limit         = 0U,
  .timer_concatenation  = 0U

};

/**
 *  Data Structure initialization - CC8 Slice dead time configuration.
 */

XMC_CCU8_SLICE_DEAD_TIME_CONFIG_t CCU8_Deadtime_Config =
{
    {
        .enable_dead_time_channel1 = 1U, /**< Enable dead time for Compare Channel-1 */
        .enable_dead_time_channel2 = 1U, /**< Enable dead time for Compare Channel-2 */
        .channel1_st_path = 1U, /**< Should dead time be applied to ST output of Compare Channel-1? */
        .channel1_inv_st_path = 1U, /**< Should dead time be applied to inverse ST output of
         Compare Channel-1? */
        .channel2_st_path = 1U, /**< Should dead time be applied to ST output of Compare Channel-2? */
        .channel2_inv_st_path = 1U, /**< Should dead time be applied to inverse ST output of
         Compare Channel-2? */
        .div = 0U, /**< Dead time prescaler divider value.*/
    },
};

/**
 *  Data Structure initialization - CC8 Slice event 0 set to sync start.
 */
XMC_CCU8_SLICE_EVENT_CONFIG_t CCU8_Input_Event0_Config =
{
  #if(UC_SERIES == XMC14)
  .mapped_input = XMC_CCU8_SLICE_INPUT_AH,
  #else
  .mapped_input = XMC_CCU8_SLICE_INPUT_H,
  #endif
  .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_RISING_EDGE
};


#ifdef TRAP_PIN
/**
 *  Data Structure initialization - CC8 Slice Trap event configuration.
 */
XMC_CCU8_SLICE_EVENT_CONFIG_t CCU8_Input_Trap_Config =
{
  #if(UC_SERIES == XMC14)
  .mapped_input = XMC_CCU8_SLICE_INPUT_AA,
  #else
  .mapped_input = XMC_CCU8_SLICE_INPUT_A,
  #endif
  .level = USER_CCU8_INPUT_TRAP_LEVEL,
  .edge = XMC_CCU8_SLICE_EVENT_EDGE_SENSITIVITY_NONE,
  .duration = XMC_CCU8_SLICE_EVENT_FILTER_7_CYCLES
};
#endif

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/

/* API to initialize CCU8 module for 3 phase pwm generation. Trap functionality enable */
void PMSM_FOC_CCU8_Init(void)
{
  CCU8_Deadtime_Config.channel1_st_rising_edge_counter = MotorParam.CCU8_DEADTIME_RISE_T, /**< CH1 dead time rising edge */
  CCU8_Deadtime_Config.channel1_st_falling_edge_counter = MotorParam.CCU8_DEADTIME_FALL_T, /**< CH1 dead time falling edge */
  CCU8_Deadtime_Config.channel2_st_rising_edge_counter = MotorParam.CCU8_DEADTIME_RISE_T, /**< CH2 dead time rising edge */
  CCU8_Deadtime_Config.channel2_st_falling_edge_counter = MotorParam.CCU8_DEADTIME_FALL_T, /**< CH2 dead time falling edge */

  XMC_CCU8_Init(CCU8_MODULE, XMC_CCU8_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  /* Set Prescaler run bit */
  XMC_CCU8_StartPrescaler(CCU8_MODULE);

  XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_PHASE_U, &CCU8_Phase_Init);
  XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_PHASE_V, &CCU8_Phase_Init);
  XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_PHASE_W, &CCU8_Phase_Init);
  XMC_CCU8_SLICE_CompareInit(CCU8_SLICE_ADC_TR, &CCU8_VADC_Trigger_Init);

  /* Timer Period Shadow Value */
  XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_PHASE_U, MotorParam.CCU8_PERIOD_REG_T);
  XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_PHASE_V, MotorParam.CCU8_PERIOD_REG_T);
  XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_PHASE_W, MotorParam.CCU8_PERIOD_REG_T);
  XMC_CCU8_SLICE_SetTimerPeriodMatch(CCU8_SLICE_ADC_TR, MotorParam.CCU8_PERIOD_REG_T);

  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_U, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_U, MotorParam.CCU8_PERIOD_REG_T+1);

  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_V, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_V, MotorParam.CCU8_PERIOD_REG_T+1);

  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_PHASE_W, 0);
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_PHASE_W, MotorParam.CCU8_PERIOD_REG_T+1);

  /* For ADC trigger  - Pulse created with 3 counts */
  /* DRIVERIC_DELAY = Driver(Ton +Trise) + Dead time in terms of PWM resolution */
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel1(CCU8_SLICE_ADC_TR, ((MotorParam.CCU8_PERIOD_REG_T >> 1U) + MotorParam.DRIVERIC_DELAY));
  XMC_CCU8_SLICE_SetTimerCompareMatchChannel2(CCU8_SLICE_ADC_TR, ((MotorParam.CCU8_PERIOD_REG_T >> 1U) + MotorParam.DRIVERIC_DELAY +20U));

  /* Dead time enabled, No prescaler for the dead time counters */
  /* Dead time could cause motor phase current distortion, especially at low motor speed */
  XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_PHASE_U, &CCU8_Deadtime_Config);
  XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_PHASE_V, &CCU8_Deadtime_Config);
  XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_PHASE_W, &CCU8_Deadtime_Config);
  XMC_CCU8_SLICE_DeadTimeInit(CCU8_SLICE_ADC_TR, &CCU8_Deadtime_Config);


  /* Enable shadow transfer for slice 0,1,2,3 for CCU80 Kernel */
  XMC_CCU8_EnableShadowTransfer(CCU8_MODULE, ((uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_0 |
					      (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_1 |
					      (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_2 |
					      (uint32_t)XMC_CCU8_SHADOW_TRANSFER_SLICE_3));

  /*  Enable interrupt for CCU80 Period Match Enable */
  XMC_CCU8_SLICE_EnableEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
  XMC_CCU8_SLICE_SetInterruptNode(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU8_SLICE_SR_ID_0);

  /* CCU80_0_IRQn = 25. CCU80 SR0 Interrupt has lower priority than ADC IRQ. CCU80_0_IRQHandler */
  NVIC_ClearPendingIRQ(CCU80_0_IRQn);
  NVIC_SetPriority (CCU80_0_IRQn, PMSM_FOC_FCL_NVIC_PRIO);
  XMC_SCU_SetInterruptControl(CCU80_0_IRQn, XMC_SCU_IRQCTRL_CCU80_SR0_IRQ25);
  NVIC_EnableIRQ(CCU80_0_IRQn);

  /* Configuring CCU80 CC8xINS - Input Selector Configuration to SCU.GSC80 */
  /* Event0 -> INyH (SCU.GSC80, Global Start Control CCU80) for EV0IS */
  /* INyH for EV0IS */
  XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);
  XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);
  XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);
  XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_ADC_TR, XMC_CCU8_SLICE_EVENT_0, &CCU8_Input_Event0_Config);

  /* Setup Event0 for external start trigger */
  XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
  XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
  XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);
  XMC_CCU8_SLICE_StartConfig(CCU8_SLICE_ADC_TR, XMC_CCU8_SLICE_EVENT_0, XMC_CCU8_SLICE_START_MODE_TIMER_START_CLEAR);

  /* Disable Global Start Control CCU80 */
  XMC_SCU_SetCcuTriggerLow(SCU_GENERAL_CCUCON_GSC80_Msk);

  /* Clear idle bit slice 0,1,2,3 for CCU80 Kernel */
  XMC_CCU8_EnableMultipleClocks(CCU8_MODULE, XMC_CCU8_GIDLC_CLOCK_MASK);

  #ifdef TRAP_PIN
  /* Trap functionality enable */
  /* LPF2M = 11b, Event2 LPF -> 7 clock cycles of fCCU8 */
  /* EV2LM = 1b, Event2 (active LOW) -> TRAP -> CCU8x.INyA -> P0.12. Note Trap function is level active */
  XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_Trap_Config);
  XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_Trap_Config);
  XMC_CCU8_SLICE_ConfigureEvent(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_EVENT_2, &CCU8_Input_Trap_Config);

  /* CCU8 TRAP functionality enabled so PWM outputs react on state of an input pin. For over-current protection */
  /* TS = 1, TRAP function enabled and connected to CCU80-CC8x Event2 */
  /* TRPSW = TRPSE = 1b, TRAP state only exited by SW, synch with PWM */
  /* TRAPE3/2/1/0 = 1b, TRAP affects CCU8x.OUTy0/1/2/3 */
  XMC_CCU8_SLICE_TrapConfig(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
  XMC_CCU8_SLICE_EnableTrap(CCU8_SLICE_PHASE_U, XMC_CCU8_TC_TRAPSE_MASK);

  XMC_CCU8_SLICE_TrapConfig(CCU8_SLICE_PHASE_V, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
  XMC_CCU8_SLICE_EnableTrap(CCU8_SLICE_PHASE_V, XMC_CCU8_TC_TRAPSE_MASK);

  XMC_CCU8_SLICE_TrapConfig(CCU8_SLICE_PHASE_W, XMC_CCU8_SLICE_TRAP_EXIT_MODE_SW, true);
  XMC_CCU8_SLICE_EnableTrap(CCU8_SLICE_PHASE_W, XMC_CCU8_TC_TRAPSE_MASK);

  /* Enable interrupt for CCU80-CC80 Event2 */
  XMC_CCU8_SLICE_EnableEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);
  XMC_CCU8_SLICE_ClearEvent(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2);

  /* Event2 interrupt forward to CC8ySR1 */
  XMC_CCU8_SLICE_SetInterruptNode(CCU8_SLICE_PHASE_U, XMC_CCU8_SLICE_IRQ_ID_EVENT2, XMC_CCU8_SLICE_SR_ID_1);

  /* CCU80_1_IRQn = 26. Trap protection interrupt. CCU80_1_IRQHandler Trap protection interrupt has highest priority */
  NVIC_SetPriority (TRAP_IRQn, PMSM_FOC_CTRAP_NVIC_PRIO);
  XMC_SCU_SetInterruptControl(TRAP_IRQn, XMC_SCU_IRQCTRL_CCU80_SR1_IRQ26);
  NVIC_EnableIRQ(TRAP_IRQn);
  #endif

  /* Interrupt Status Clear, for interrupts of Period Match, Trap Flag, and Event2 */
  CCU8_SLICE_PHASE_U->SWR |= 0x00000C01U;
}
