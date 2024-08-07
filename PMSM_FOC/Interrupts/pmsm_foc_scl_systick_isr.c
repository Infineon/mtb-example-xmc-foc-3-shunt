/**
 * @file pmsm_foc_scl_systick_isr.c
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
#include "../ControlModules/pmsm_foc_error_handling.h"
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ControlModules/pmsm_foc_interface.h"
#include "../ToolInterface/uCProbe.h"
#include "../MCUInit/6EDL_gateway.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Interrupts
 * @{
 */

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
extern uint8_t idle_state_delay_counter;

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
/**
 * @brief Systick event handler for Slow Control Loop(SCL) \n
 * This is the lowest priority interrupt which performs less time critical tasks like ramp, potentiometer reading,etc.\n
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_SCL_ISR(void)
{
  /* Call Slow Control Loop task handler */
  SystemVar.GlobalTimer++;   /* this timer reaches 0xFFFFFFFF after 49.7 days */
  /* Configure/status monitoring of 6EDL7141 through GUI*/
  EDL7141_Update();
  switch (PMSM_FOC_CTRL.msm_state)
  {
  case PMSM_FOC_MSM_IDLE:
      if ((SystemVar.ParamConfigured == 1) && (SystemVar.Edl7141Configured == 1))
    {
      if (idle_state_delay_counter <= 3)
      {
        idle_state_delay_counter++;  /* have to add extra 3ms delay due to 6EDL7141 watchdog clock, otherwise offset calibration will be off or current sensing doesn't working properly */
      }
      else if (idle_state_delay_counter > 3)
      {
        /* Initialize MCU and motor control peripherals */
        EdlIo.en_drv_level = 1;
        PMSM_FOC_Init();
        /* Start the motor */
        PMSM_FOC_MotorStart();
        idle_state_delay_counter = 0;
      }
    }
    else
    {

    }
      break;

  default:
    /* Users may choose to start off with Voltage control first then speed inner current control. See example code below*/

      if (PMSM_FOC_INPUT.user_ctrl_scheme == SPEED_INNER_CURRENT_CTRL_SCHEME)
      {
          if ((PMSM_FOC_CTRL.msm_state == PMSM_FOC_MSM_CLOSED_LOOP)
            && (PMSM_FOC_CTRL.transition_status == PMSM_FOC_MOTOR_STATUS_TRANSITION))
          {
            PMSM_FOC_SPEED_PI.uk_limit_max = PMSM_FOC_INPUT.limit_max_iq;

            // During startup Vq voltage control is used and then it will switch to the speed current control
            if (PMSM_FOC_PLL_ESTIMATOR.rotor_speed > (2 * MotorParam.ELECTRICAL_SPEED_LOW_LIMIT_TS))
            {
              PMSM_FOC_CTRL.transition_status = PMSM_FOC_MOTOR_STATUS_STABLE;
              PMSM_FOC_SPEED_PI.ik = PMSM_FOC_OUTPUT.park_transform.torque_iq << PMSM_FOC_SPEED_PI.scale_kp_ki;
              PMSM_FOC_TORQUE_PI.ik = PMSM_FOC_OUTPUT.torque_vq << PMSM_FOC_TORQUE_PI.scale_kp_ki;
              PMSM_FOC_CTRL.ctrl_scheme_fun_ptr = &PMSM_FOC_SpeedCurrentCtrl;
            }
          }
      }
      else if(PMSM_FOC_INPUT.user_ctrl_scheme == VQ_VOLTAGE_CTRL_SCHEME)
      {
        if(PMSM_FOC_PLL_ESTIMATOR.rotor_speed > MotorParam.ELECTRICAL_SPEED_LOW_LIMIT_TS)
        PMSM_FOC_CTRL.transition_status = PMSM_FOC_MOTOR_STATUS_STABLE;
      }

      /***************************************************************************************************/
      /* If ucProbe/Microinspector GUI is enabled then this function process the cmd received from GUI */
    #if(USER_UCPROBE_GUI == ENABLED)
      PMSM_FOC_ucProbe_CmdProcessing();
    #endif
      break;
      }
}

/**
 * @}
 */
/**
 * @}
 */
