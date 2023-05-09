/**
 * @file pmsm_foc_functions.h
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

#ifndef PMSM_FOC_FUNCTIONS_H
#define PMSM_FOC_FUNCTIONS_H

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
#include <PMSM_FOC/Configuration/pmsm_foc_common.h>
#include "../MCUInit/pmsm_foc_cordic.h"
#include "../MIDSys/pmsm_foc_svpwm.h"
#include "../MCUInit/pmsm_foc_clock.h"
#include "../MCUInit/pmsm_foc_adc.h"
#include "../MCUInit/pmsm_foc_wdt.h"
#include "../MCUInit/pmsm_foc_ccu8.h"
#include "../MIDSys/pmsm_foc_pi.h"

#include "../IPLib/pmsm_foc_ip.h"

#include "../ToolInterface/uCProbe.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define IS_CORDIC_BUSY                       (MATH->STATC & 0x01)          /*!< Returns 1 if CORDIC is busy */
#define IS_MATH_DIV_BUSY                     (MATH->DIVST & 0x01)          /*!< Returns 1 if DIVIDER is busy */
#define SWAP(x,y)                            int32_t t;t=x;x=y;y=t;        /*!< Swap two numbers */
#define ADCLPF                               (3U)
#define SYSTEM_BE_IDLE                       ((ADC.adc_res_pot < USER_TH_POT_ADC) || (PMSM_FOC_CTRL.motor_start_flag == 0))    /*!< Reference is too low then go to IDLE state */

/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/**
 * @brief This enumerates the motor control scheme options.
 */
typedef enum {
  SPEED_INNER_CURRENT_CTRL_SCHEME, /*!< 0000 - Speed Inner Current Control Loop */
  VQ_VOLTAGE_CTRL_SCHEME, /*!< 0001 - Vq Voltage Control  */
  VF_OPEN_LOOP_CTRL_SCHEME /*!< 0002 - V/F Open Loop Control  */

} PMSM_FOC_CTRL_SCHEME_t;

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**  @brief Clarke Transform structure */
typedef struct {
  int32_t i_alpha_1q31; /*!<  Iα (1Q31), Alpha value of current space vector */
  int32_t i_beta_1q31; /*!<  Iβ (1Q31) */

} PMSM_FOC_CLARKE_TRANSFORM_t;

/**  @brief Park Transform structure */
typedef struct {
  int32_t flux_id; /*!< Flux current component Id */
  int32_t torque_iq; /*!< Torque current component Iq */

} PMSM_FOC_PARK_TRANSFORM_t;

/**  @brief Cartesian to polar transform structure */
typedef struct {
  int32_t vref_angle_q31; /*!<  Reference voltage vector angle for SVPWM generation */
  uint32_t vref32; /*!<  Reference voltage vector magnitude for SVPWM generation */

} PMSM_FOC_CAR2POL_t;

typedef void
(*PMSM_FOC_CTRL_SCHEME_FUNC_t)(void);

/**  @brief FOC Input structure */
typedef struct {
  PMSM_FOC_CTRL_SCHEME_t user_ctrl_scheme; /*!< User configured control scheme */

  int32_t i_u; /*!<  Motor phase current - Iu */
  int32_t i_v; /*!<  Motor phase current - Iv */
  int32_t i_w; /*!<  Motor phase current - Iw */

  int32_t ref_speed; /*!<  Motor reference speed in speed inner current control loop */
  int32_t ref_vq; /*!<  Reference voltage in vq voltage control loop */

  int32_t ref_id; /*!<  combined(MTPA+FW) id reference used for Flux PI control */
  int32_t ref_iq; /*!<  iq reference used for Torque PI control,generated from speed PI control */

  int16_t limit_max_iq; /*!<  In Torque Limit mode it is set by user */

  uint16_t brake_duty_val; /*!< While braking lower PWM duty cycle compare value */

  int16_t idc_over_current_limit; /*!< DC bus estimated/actual over current limit */

} PMSM_FOC_INPUT_t;

/**  @brief FOC Output Structure */
typedef struct {
  PMSM_FOC_CLARKE_TRANSFORM_t clarke_tansform; /*!<  Clarke transform output - i_alpha,i_beta */
  PMSM_FOC_PARK_TRANSFORM_t park_transform; /*!<  Park transform output - flux_id,flux_iq */

  int32_t rotor_speed; /*!<  Estimated rotor speed by PLL estimator. */
  int32_t rotor_angle_q31; /*!<  Estimated rotor angle by PLL estimator. */

  int32_t flux_vd; /*!<  Flux PI controller output vd */
  int32_t torque_vq; /*!<  Torque PI controller output vq */

  PMSM_FOC_CAR2POL_t car2pol; /*!<  Cartesian to polar conversion results */

  int16_t decoupling_id; /*!<  decoupling flux component - Flux PI control feed forward term */
  int16_t decoupling_iq; /*!<  decoupling torque component - Torque PI control feed forward term */

  uint16_t svm_vref_16; /*!<  |Vref|, Magnitude (1Q15) of reference vector (for SVM) */
  uint16_t svm_angle_16; /*!<  Angle θ (16-bit) of reference vector. 0 ~ 2^16 represent electrical angle 0° ~ 360° */

} PMSM_FOC_OUTPUT_t;

/**  @brief Motor control structure */
typedef struct {
  PMSM_FOC_CTRL_SCHEME_FUNC_t ctrl_scheme_fun_ptr;/*!< FOC Control scheme function pointer */
  int32_t set_val_pot; /*!<  Target Motor speed set by POT ADC, or PWM duty cycle */

  int32_t ramp_up_rate; /*!<  motor speed ramp up rate */
  int32_t ramp_dn_rate; /*!<  motor speed ramp down rate */
  uint32_t ramp_counter; /*!<  General purpose counter, or counter for Motor speed ramp up/down. */

  int32_t msm_state; /*!<  motor control state machine active state PMSM_FOC_MSM_t */
  uint32_t adjust_para_flag; /*!<  Flag to indicate parameter scheduling status */

  uint32_t alignment_counter; /*!<  Counter for rotor initial positioning / alignment in V/f */
  uint32_t braking_counter; /*!<  General purpose counter */
  uint32_t iq_limit_blanking_counter; /*!<  iq limiter counter during startup torque limitation is blanked */

  uint32_t error_status; /*!<  Error status to identify which error has occurred */
  union {
    struct {
      uint32_t Trap :1;
      uint32_t OverCurrent :1;
      uint32_t ShortCircuit :1;
      uint32_t WrongHall :1;
      uint32_t HallLearning :1;
      uint32_t Stall :1;
      uint32_t OverVoltage :1;
      uint32_t FreeRunning :1;
      uint32_t FreeRevRunning :1;
      uint32_t StartupFailure :1;
      uint32_t UnderVoltage :1;
      uint32_t SpiError :1;
      uint32_t Nfault :1;
    };
    uint32_t Value;
  } MaskedFault;

  uint16_t transition_status; /*!<  Flag to indicate if motor is in transition (MOTOR_TRANSITION) or stable (MOTOR_STABLE) */
  uint16_t rotation_dir; /*!<  Rotation direction of motor (rotor angle increasing, or decreasing) */
  uint16_t motor_start_flag; /*!<  motor start/stop indication flag */

} PMSM_FOC_CTRL_t;

/**  @brief V/F open loop */
typedef struct {
  uint16_t vf_offset; /*!<  V/F open loop offset */
  uint16_t vf_constant; /*!<  V/F constant */
  uint16_t vf_speed_ramp_up_rate; /*!<  V/F speed ramp up rate */
  uint16_t vf_speed_ramp_counter; /*!<  ramp counter to track ramp up rate */
  uint16_t vf_transition_speed; /*!<  V/F transition to close loop speed */
  uint16_t vf_motor_speed; /*!<  V/F open loop motor speed */
  uint16_t vref_mag; /*!<  V/F open loop voltage amplitude for SVPWM */
  uint16_t vref_angle; /*!<  V/F open loop voltage angle for SVPWM */
  uint16_t stablization_count; /*!< Time before jump to close loop */
  uint16_t stablization_counter; /*!< Counter to track time before jump to close loop */
  uint8_t dont_exit_open_loop_flag; /*!< If flag set to 1 ctrl will not go to close loop - Useful for debugging only */

} PMSM_FOC_VF_OPEN_LOOP_t;

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/
extern PMSM_FOC_INPUT_t PMSM_FOC_INPUT;
extern PMSM_FOC_OUTPUT_t PMSM_FOC_OUTPUT;
extern PMSM_FOC_CTRL_t PMSM_FOC_CTRL;
extern PMSM_FOC_VF_OPEN_LOOP_t PMSM_FOC_VF_OPEN_LOOP_CTRL;

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
/**
 * @brief  API to initialize MCU and peripherals for motor control
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_Init(void);

/**
 * @brief To init PI controllers' integral terms (Ik) for first FOC PWM cycle after V/F open loop
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_PI_InitIks(void);

/**
 * @brief  Variables initialization before motor start
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VariablesInit(void);

/**
 * @brief Initialize V/F open loop parameters
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VF_OpenLoopInit(void);

/**
 * @brief Miscellaneous works in FOC, such as ramp up, speed adjustment, stop Motor, etc
 * Do NOT add any CORDIC calculations in this function.
 *
 * @param None
 *
 * @retval None
 */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_MiscWorksOfFOC(void);

/*********************************************************************************************************************
 * API DEFINATION
 ********************************************************************************************************************/
/**
 * @brief	3-Shunt 3-Phase Current Reconstruction, ADC values are from last PWM cycle
 * 		ADCs of 2or3-Shunt are triggered by CCU83 CR1S
 *
 * @param vadc_res_iu
 * @param vadc_res_iv
 * @param vadc_res_iw
 *
 *@retval 	*PMSM_FOC_INPUT_t
 */

__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_CurrentReconstruction(int32_t vadc_res_iu, int32_t vadc_res_iv, int32_t vadc_res_iw, PMSM_FOC_INPUT_t * const handle_ptr)
{
  /* Motor phase current, Iu, Iv, Iw*/
  handle_ptr->i_u = (int16_t) ((ADC.adc_bias_iu - vadc_res_iu) << 4);
  handle_ptr->i_v = (int16_t) ((ADC.adc_bias_iv - vadc_res_iv) << 4);
  handle_ptr->i_w = (int16_t) ((ADC.adc_bias_iw - vadc_res_iw) << 4);

}

/**
 * @brief To get current I_Alpha / I_Beta of last PWM cycle
 * I_Alpha = I_U
 * I_Beta = (I_U + 2 * I_V)/√3 = (I_V - I_W)/√3
 * Above transform scales down I_Mag (i.e.: |I|) by 2/3. Need scale up by 3/2.
 * Alternatively, can scale up inductance L in ωL|I| by 3/2 (legacy scaling).
 *
 * @param ph_iu
 * @param ph_iv
 * @param ph_iw
 *
 *@retval *PMSM_FOC_CLARKE_TRANSFORM_t
 */

__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_ClarkeTransform(int32_t ph_iu, int32_t ph_iv, int32_t ph_iw, PMSM_FOC_CLARKE_TRANSFORM_t* const handle_ptr)
{
  if (SVPWM.flag_3or2_adc == USE_ALL_ADC) {
    /* Use all three phase current measurements to calculate alpha,beta */
    /* I_Alpha = (2 * I_U - (I_V + I_W))/3 */
    handle_ptr->i_alpha_1q31 = (int32_t) (((ph_iu << 1) - (ph_iv + ph_iw)) * (DIV_3_Q14 << (CORDIC_SHIFT - 14)));
    /*  I_Beta = (I_V - I_W)/√3 in 1Q31 */
    handle_ptr->i_beta_1q31 = (int32_t) ((ph_iv - ph_iw) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT - 14U)));
  }
  else {
    /* Use only two measurable phase current measurements in particular SVPWM sector to calculate alpha,beta */
    switch (SVPWM.previous_sector_num)
    {
    case 0:
    case 5:
      handle_ptr->i_alpha_1q31 = -(int32_t) ((ph_iv + ph_iw) << CORDIC_SHIFT);
      handle_ptr->i_beta_1q31 = (int32_t) ((ph_iv - ph_iw) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT - 14U)));
      break;

    case 1:
    case 2:
      handle_ptr->i_alpha_1q31 = (int32_t) (ph_iu << CORDIC_SHIFT);
      handle_ptr->i_beta_1q31 = -(int32_t) ((ph_iu + (ph_iw << 1)) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT - 14U)));
      break;

    default:
      handle_ptr->i_alpha_1q31 = (int32_t) (ph_iu << CORDIC_SHIFT);
      handle_ptr->i_beta_1q31 = ((int32_t) (ph_iu + (ph_iv << 1)) * (DIV_SQRT3_Q14 << (CORDIC_SHIFT - 14U)));
      break;
    }
  }
}

/**
 * @brief CORDIC #1 - Park Transform
 * Iq = K[I_Beta cos(φ)-I_Alpha sin(φ)]/MPS   * Iq = Xfinal = K[X cos(Z) - Y sin(Z)] / MPS, where K = 1.646760258121.
 * Id = K[I_Alpha cos(φ)+I_Beta sin(φ)]/MPS   * Id = Yfinal = K[Y cos(Z) + X sin(Z)] / MPS      (Zfinal = 0).
 *
 * @param i_alpha_1q31
 * @param i_beta_1q31
 * @param rotor_angle_q31
 *
 *@retval None
 *
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_ParkTransform(PMSM_FOC_CLARKE_TRANSFORM_t * const handle_ptr, int32_t rotor_angle_q31)
{
  /* General control of CORDIC Control Register */
  MATH->CON = CORDIC_CIRCULAR_ROTATION_MODE;
  /* Z = φ, Hall rotor angle, or estimated rotor angle of last PWM cycle from PLL */
  MATH->CORDZ = rotor_angle_q31;
  /* Y = I_Alpha */
  MATH->CORDY = handle_ptr->i_alpha_1q31;
  /* X = I_Beta. Input CORDX data, and auto start of CORDIC calculation (~62 kernel clock cycles) */
  MATH->CORDX = handle_ptr->i_beta_1q31;
}

/**
 * @brief Get CORDIC Result from Park Transform
 *
 * @param *PMSM_FOC_PARK_TRANSFORM_t
 *
 * @retval *PMSM_FOC_PARK_TRANSFORM_t
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_ParkTransform_GetResult(PMSM_FOC_PARK_TRANSFORM_t* const handle_ptr)
{
  /* Wait if CORDIC is still running calculation */
  while (IS_CORDIC_BUSY);

  /* Read CORDIC results Iq and Id - 32-bit. CORDIC Result Register [7:0] are 0x00 */
  handle_ptr->torque_iq = MATH->CORRX;
  handle_ptr->flux_id = MATH->CORRY;

  /*CPU computes the following simultaneously when CORDIC #2 is computing */
  handle_ptr->torque_iq >>= CORDIC_SHIFT;

  /* Shift to get real results */
  handle_ptr->flux_id >>= CORDIC_SHIFT;
  handle_ptr->torque_iq = (handle_ptr->torque_iq * CORDIC_CIRCULAR_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE; // x MPS/K.;
  handle_ptr->flux_id = (handle_ptr->flux_id * CORDIC_CIRCULAR_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE; // x MPS/K.;
}

/**
 * @brief Cartesian to Polar + Angle Addition, optimized FOC of Infineon
 *      Vref = K/MPS * sqrt(V_q^2+V_d^2)    * Xfinal = K/MPS * sqrt(X^2+Y^2), where K = 1.646760258121.
 *      Θ = atan(V_q/V_d) + Phi         * Zfinal = Z + atan(Y/X)          (Yfinal = 0).
 *
 * @param torque_vq
 * @param flux_vd
 * @param rotor_angle_q31
 *
 * @retval None
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_Cart2Polar(int32_t torque_vq, int32_t flux_vd, int32_t rotor_angle_q31)
{
  /* General control of CORDIC Control Register */
  MATH->CON = CORDIC_CIRCULAR_VECTORING_MODE;
  /* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
  MATH->CORDZ = rotor_angle_q31;
  /* Y = Vq = PI_Torque.Uk */
  MATH->CORDY = (uint32_t) (torque_vq << CORDIC_SHIFT);
  /* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */
  MATH->CORDX = (uint32_t) (flux_vd << CORDIC_SHIFT);
}

/**
 * @brief Cartesian to Polar + Angle Addition, optimized FOC of Infineon
 *
 * @param *PMSM_FOC_CAR2POL_t
 *
 * @retval *PMSM_FOC_CAR2POL_t
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_Cart2Polar_GetResult(PMSM_FOC_CAR2POL_t * const handle_ptr)
{
  while (IS_CORDIC_BUSY);
  /* Read CORDIC result |Vref| - 32-bit unsigned  and scale down to get real value */
  handle_ptr->vref32 = MATH->CORRX;
  /* Angle addition by CORDIC directly, where Θ = atan(Vq/Vd), φ is rotor angle */
  handle_ptr->vref_angle_q31 = (int32_t) MATH->CORRZ;

  /* Get real values by scaling down */
  handle_ptr->vref32 = (handle_ptr->vref32 >> CORDIC_SHIFT);
  handle_ptr->vref32 = (uint32_t) ((handle_ptr->vref32 * CORDIC_CIRCULAR_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.
  handle_ptr->vref32 = (uint32_t) (handle_ptr->vref32 << CORDIC_SHIFT);
}

/**
 * @brief Calculates magnitude of resultant vector in hyperbolic mode.
 * @param x  Input X co-ordinate value
 * @param y  Input Y co-ordinate value
 * @return   None <BR>
 *
 * \par<b>Description</b><br>
 * This function calculates the magnitude of resultant vector in hyperbolic mode.\n
 * Magnitude  = SQRT(x*x-y*y); where k = 0.828159360960 \n
 *
 * \par<b>Note</b><br>
 * On XMC14 device this API will use HW CORDIC module for computation of trigonometric, hyperbolic and linear functions. \n
 * x should be greater than y.
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_HyperMag(int16_t x, int16_t y)
{
  /* General control of CORDIC Control Register */
  MATH->CON = CORDIC_HYPERBOLIC_VECTORING_MODE;
  /* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
  MATH->CORDZ = 0;
  /* Y = Vq = PI_Torque.Uk */
  MATH->CORDY = (uint32_t) (y << CORDIC_SHIFT);
  /* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */
  MATH->CORDX = (uint32_t) (x << CORDIC_SHIFT);
}

/**
 * @brief Reads the results of magnitude of resultant vector in hyperbolic mode.
 *
 * @param None
 *
 * @retval None
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE int16_t PMSM_FOC_HyperMag_GetResult(void)
{
  uint32_t resultant_magnitude;
  while (IS_CORDIC_BUSY);
  /* Read CORDIC result - 32-bit unsigned  and scale down to get real value */
  resultant_magnitude = MATH->CORRX;

  /* Get real values by scaling down */
  resultant_magnitude = (resultant_magnitude >> CORDIC_SHIFT);
  resultant_magnitude = (uint32_t) ((resultant_magnitude * CORDIC_HYPERBOLIC_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.
  return ((int16_t) resultant_magnitude);
}

/**
 * @brief Calculates magnitude of resultant vector in circular mode.
 * @param x  Input X co-ordinate value
 * @param y  Input Y co-ordinate value
 * @return   None <BR>
 *
 * \par<b>Description</b><br>
 * This function calculates the magnitude of resultant vector in hyperbolic mode.\n
 * Magnitude  = SQRT(x*x-y*y); where k = 0.828159360960 \n
 *
 * \par<b>Note</b><br>
 * On XMC14 device this API will use HW CORDIC module for computation of trigonometric, hyperbolic and linear functions. \n
 * x should be greater than y.
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_CircularMag(int16_t x, int16_t y)
{
  /* General control of CORDIC Control Register */
  MATH->CON = CORDIC_CIRCULAR_VECTORING_MODE;
  /* Z = φ. Θ = atan(Vq/Vd) + rotor angle φ, equivalent to Inv. Park Transform */
  MATH->CORDZ = 0;
  /* Y = Vq = PI_Torque.Uk */
  MATH->CORDY = (uint32_t) (y << CORDIC_SHIFT);
  /* X = Vd = PMSM_FOC_FLUX_PI.Uk. Input CORDX data, and auto start of CORDIC calculation */
  MATH->CORDX = (uint32_t) (x << CORDIC_SHIFT);
}

/**
 * @brief Reads the results of magnitude of resultant vector in hyperbolic mode.
 *
 * @param None
 *
 *@retval None
 */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE uint32_t PMSM_FOC_CircularMag_GetResult(void)
{
  uint32_t resultant_magnitude;
  while (IS_CORDIC_BUSY);
  /* Read CORDIC result - 32-bit unsigned  and scale down to get real value */
  resultant_magnitude = MATH->CORRX;

  /* Get real values by scaling down */
  resultant_magnitude = (resultant_magnitude >> CORDIC_SHIFT);
  resultant_magnitude = (uint32_t) ((resultant_magnitude * CORDIC_CIRCULAR_MPS_BY_K_SCALED) >> CORDIC_MPS_BY_K_SCALE); // x MPS/K.
  return (resultant_magnitude);
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_FUNCTIONS_H */

