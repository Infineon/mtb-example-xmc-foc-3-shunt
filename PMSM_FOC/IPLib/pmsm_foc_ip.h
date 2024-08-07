/**
 * @file pmsm_foc_ip.h
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

#ifndef PMSM_FOC_IPLIB_H_
#define PMSM_FOC_IPLIB_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup IPLib
 * @brief  Infineon Motor control protected modules <br>
 * @{
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include <xmc_common.h>

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define PLL_LIB_RAM_ATTRIBUTE  __RAM_FUNC     /*!< If defined PLL observer library routines will be executed from RAM */
//#define FOC_PLL_RAM_ATTRIBUTE

/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/
/**
 *  @brief PI PLL Control parameters
 */
typedef struct PMSM_FOC_PLL_PI
{
  int32_t error;                              /*!< PI error signal (reference value � feedback value), error[k] */
  int32_t uk;                                 /*!< PI output U[k] */
  int32_t ik;                                 /*!< Integral result I[k] */
  int32_t ik_limit_min;                       /*!< Integral buffer limit  - minimum */
  int32_t ik_limit_max;                       /*!< Integral buffer limit  - maximum */
  int32_t uk_limit_min;                       /*!< PI output limit - minimum */
  int32_t uk_limit_max;                       /*!< PI output limit - maximum */
  int32_t uk_limit_max_scaled;                /*!< Internal variable - PI output limit scaled - maximum */
  int32_t uk_limit_min_scaled;                /*!< Internal variable - PI output limit scaled - minimum */
  uint16_t kp;                                /*!< Proportional gain Kp */
  uint16_t ki;                                /*!< Integral gain Ki */
  int16_t scale_kp_ki;                        /*!< Scale-up Kp and Ki by 2^Scale_KpKi */

} PMSM_FOC_PLL_PI_t;

/**
 *  @brief PLL Estimator parameters
 */
typedef struct PMSM_FOC_PLL_ESTIMATOR
{
  int32_t phase_inductance_Ls;                /*!< Motor stator phase inductance scaled */
  int32_t bemf_1;                             /*!< PLL observer internal signal */
  int32_t bemf_2;                             /*!< PLL observer internal signal */
  int32_t current_i_mag;                      /*!< stator current magnitude */
  int32_t current_i_mag_filtered;             /*!< */
  int32_t delta_vi_angle_q31;                 /*!< */
  int32_t vref_x_sin_delta;                   /*!< */
  int32_t vref_x_cos_delta;                   /*!< */
  int32_t rotor_angle_q31;                    /*!< */
  int32_t rotor_speed;                        /*!< */
  int16_t phase_inductance_scale;             /*!< */
  int16_t lpf_n_bemf;                         /*!< Low pass filter coefficient for PLL internal signals */
  int32_t lpf_n_speed;                        /*!< Low pass filter coefficient for estimated speed */
  int16_t speed_angle_conversion_factor;      /*!< Rotor speed to angle conversion factor */
  int16_t speed_angle_conversion_factor_scale; /*!< Rotor speed to angle conversion factor scale */

}PMSM_FOC_PLL_ESTIMATOR_t;


/**********************************************************************************************************************
* EXTERN
**********************************************************************************************************************/
extern PMSM_FOC_PLL_PI_t PMSM_FOC_PLL_PI;
extern PMSM_FOC_PLL_ESTIMATOR_t PMSM_FOC_PLL_ESTIMATOR;

/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/**
 * This function calculates magnitude of stator current from alpha,beta current component.
 * It uses the CORDIC to calculate magnitude. Function loads alpha beta to CORDIC register
 * and initiates the calculations.
 * @param Vref_AngleQ31
 * @param I_Alpha_1q31
 * @param I_Beta_1q31
 * @return None
 */
PLL_LIB_RAM_ATTRIBUTE void PMSM_FOC_PLL_Imag(int32_t Vref_AngleQ31, int32_t i_alpha_1q31, int32_t i_beta_1q31);
/**
 * This function reads the stator current magnitude result from CORDIC result register.
 * @param PLL Estimator structure pointer
 * @return None
 */
PLL_LIB_RAM_ATTRIBUTE void PMSM_FOC_PLL_ImagGetResult(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr);
/**
 * This function calculates the PLL observer internal signal value required for the estimator.
 * @param vref_32 Stator voltage magnitude
 * @param PMSM_FOC_PLL_ESTIMATOR_t handle pointer
 * @return None
 */
PLL_LIB_RAM_ATTRIBUTE void PMSM_FOC_PLL_Vref (uint32_t vref_32, PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr, PMSM_FOC_PLL_PI_t* const pi_pll_handle_ptr);
/**
 * This function reads the calculated PLL observer internal signal value required for the estimator.
 * @param PLL Estimator structure pointer
 * @param PLL PI controller structure pointer
 * @return None
 */
PLL_LIB_RAM_ATTRIBUTE void PMSM_FOC_PLL_VrefGetResult(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr);
/**
 * This function estimates the motor speed and rotor angle.
 * @param PLL Estimator structure pointer
 * @param PLL PI controller structure pointer
 * @return None
 */
PLL_LIB_RAM_ATTRIBUTE void PMSM_FOC_PLL_GetPosSpeed(PMSM_FOC_PLL_ESTIMATOR_t* const handle_ptr,PMSM_FOC_PLL_PI_t* const pi_pll_handle_ptr);

/**
 * @}
 */

/**
 * @}
 */

#endif

/* --- End of File ------------------------------------------------ */
