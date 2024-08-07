/**
 * @file pmsm_foc_pi.h
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
#ifndef PMSM_FOC_PI_H_
#define PMSM_FOC_PI_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MIDSys
 * @{
 */


/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../ToolInterface/uCProbe.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#ifndef MIN_MAX_LIMIT                                    /*!< Limits the input as per limits */
#define MIN_MAX_LIMIT(Buffer,LimitH,LimitL) ((Buffer) > (LimitH)) ? (LimitH) : (((Buffer) < (LimitL))? (LimitL): (Buffer))
#endif

#ifndef MIN
#define MIN(a, b)          (((a) < (b)) ? (a) : (b))   /*!< macro returning smallest input */
#endif

#ifndef MAX
#define MAX(a, b)          (((a) > (b)) ? (a) : (b))   /*!< macro returning biggest input */
#endif

#ifndef ROUND
#define ROUND(x)           ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#endif

#ifndef ROUNDUP
//#define ROUNDUP(x)       ((x - (int)x)==0 ? (int)x : (int)x+1)
#define ROUNDUP(x)         ((x)>=0?(((x - (int)x)==0 ? (int)x : (int)x+1)):(((x - (int)x)==0 ? (int)x : (int)x-1)))
#endif

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct PMSM_FOC_PI
{
  int32_t error;                              /*!< PI error signal (reference value � feedback value), error[k] */
  int32_t uk;                                 /*!< PI output U[k] */
  int32_t ik;                                 /*!< Integral result I[k] */
  int32_t ik_limit_min;                       /*!< Integral buffer limit  - minimum */
  int32_t ik_limit_max;                       /*!< Integral buffer limit  - maximum */
  int32_t uk_limit_min;                       /*!< PI output limit  - minimum */
  int32_t uk_limit_max;                       /*!< PI output limit  - maximum */
  int32_t uk_limit_max_scaled;                /*!< Internal variable - PI output limit scaled - maximum */
  int32_t uk_limit_min_scaled;                /*!< Internal variable - PI output limit scaled - minimum */
  uint16_t kp;                                /*!< Proportional gain Kp */
  uint16_t ki;                                /*!< Integral gain Ki */
  int16_t scale_kp_ki;                        /*!< Scale-up Kp and Ki by 2^Scale_KpKi */
} PMSM_FOC_PI_t;

/**********************************************************************************************************************
* EXTERN
**********************************************************************************************************************/
extern PMSM_FOC_PI_t PMSM_FOC_SPEED_PI;
extern PMSM_FOC_PI_t PMSM_FOC_TORQUE_PI;
extern PMSM_FOC_PI_t PMSM_FOC_FLUX_PI;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/

/* Updates the PI control output limits */
/**
 * @brief Updates the PI control output limits
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_PI_Init(void);

/*********************************************************************************************************************
 * API IMPLEMENTATIONS
 ********************************************************************************************************************/
/**
 * @brief PI controller
 *      U(t)=Kp x e(t) + (Ki/Ts) x ∫e(t)dt, where Ts is sampling period, e.g.: Ts = 50us.
 *      I[k] = I[k-1] + Ki * error[k]
 *      U[k] = Kp * error[k] + I[k]
 *
 * @param reference
 * @param feedback
 * @param *PI controller
 *
 * @retval *PI controller
 */
__STATIC_INLINE void PMSM_FOC_PI_Controller(int32_t reference, int32_t feedback, PMSM_FOC_PI_t *pi_handle_ptr)
{
  int32_t temp_ik_uk;

  pi_handle_ptr->error = reference - feedback;

  /* Integral output I[k] = I[k-1] + Ki * error[k] */
  temp_ik_uk = ((int32_t)pi_handle_ptr->ki * pi_handle_ptr->error) + pi_handle_ptr->ik;

  /* Check I[k] limit */
  pi_handle_ptr->ik = MIN_MAX_LIMIT(temp_ik_uk, pi_handle_ptr->ik_limit_max, pi_handle_ptr->ik_limit_min);

  /* PI output U[k] = Kp * error[k] + I[k] */
  temp_ik_uk = (int32_t)((((int32_t)pi_handle_ptr->kp * pi_handle_ptr->error) + pi_handle_ptr->ik) >> pi_handle_ptr->scale_kp_ki);

  /* Check U[k] output limit */
  pi_handle_ptr->uk = MIN_MAX_LIMIT(temp_ik_uk, pi_handle_ptr->uk_limit_max, pi_handle_ptr->uk_limit_min);
}


/**
 * @brief PI controller with anti-windup and dynamic clampling and feed forward.
 *      U(t)=Kp x e(t) + (Ki/Ts) x ∫e(t)dt, where Ts is sampling period, e.g.: Ts = 50us.
 *      I[k] = I[k-1] + Ki * error[k]
 *      U[k] = Kp * error[k] + I[k]
 *
 * @param reference
 * @param feedback
 * @param feedforward
 * @param *PI controller
 *
 * @retval *PI controller
 */
__STATIC_INLINE void PMSM_FOC_PI_AntiWindup(int32_t reference, int32_t feedback, int16_t feedforward, \
                                                                             PMSM_FOC_PI_t *pi_handle_ptr)
{
    int32_t temp_uk;
    int32_t temp_kp_ff_error;

    pi_handle_ptr->error = MIN_MAX_LIMIT((reference - feedback), 32767, -32767);

    temp_kp_ff_error = (int32_t)(pi_handle_ptr->kp * pi_handle_ptr->error);
    temp_kp_ff_error += (int32_t)(feedforward << pi_handle_ptr->scale_kp_ki);


    /* Integral output I[k] = I[k-1] + Ki * error[k] */
    pi_handle_ptr->ik += (int32_t)(pi_handle_ptr->ki * pi_handle_ptr->error);

    /* Dynamic error integral limit */
    pi_handle_ptr->ik_limit_max = (MAX((pi_handle_ptr->uk_limit_max_scaled - temp_kp_ff_error), 0));
    pi_handle_ptr->ik_limit_min = (MIN((pi_handle_ptr->uk_limit_min_scaled - temp_kp_ff_error), 0));


    /* Limit the integral buffer as per dynamic error integral limit  */
    pi_handle_ptr->ik = MIN_MAX_LIMIT(pi_handle_ptr->ik, pi_handle_ptr->ik_limit_max, pi_handle_ptr->ik_limit_min);


    /* PI output U[k] = Kp * error[k] + FF + I[k] */
    temp_uk = (int32_t)((temp_kp_ff_error + pi_handle_ptr->ik) >> pi_handle_ptr->scale_kp_ki);

    /* Check U[k] output limit */
    pi_handle_ptr->uk = MIN_MAX_LIMIT(temp_uk, pi_handle_ptr->uk_limit_max, pi_handle_ptr->uk_limit_min);


}

/* Updates the PI control output limits */
/**
 * @brief Updates the PI control output limits
 *
 * @param PI Output limit - min
 * @param PI Output limit - max
 * @param *PI controller
 *
 * @retval *PI controller
 */
__STATIC_INLINE void PMSM_FOC_PI_SetMinMax(int32_t uk_limit_min, int32_t uk_limit_max, PMSM_FOC_PI_t *pi_handle_ptr)
{
    pi_handle_ptr->uk_limit_min = uk_limit_min;
    pi_handle_ptr->uk_limit_max = uk_limit_max;
}

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_CONTROLMODULES_PMSM_FOC_PI_H_ */

