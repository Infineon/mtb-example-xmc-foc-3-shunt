/**
 * @file pmsm_foc_common.h
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
 *@endcond
 ***********************************************************************************************************************/

#ifndef PMSM_FOC_COMMON_H_
#define PMSM_FOC_COMMON_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Configuration
 * @{
 */
#include <xmc_common.h>
/**********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
/* If defined then FOC time critical API's will be executed from RAM. */
#define PMSM_FOC_RAM_ATTRIBUTE  __RAM_FUNC

#define XMC14_STANDALONE                            (0U)
#define IMD700A                                     (1U)

/*******************************************************************************************************************
* Evaluation board configuration
*******************************************************************************************************************/
#define EVAL_6EDL7141_FOC_3SH                 (1U)
#define EVAL_IMD700A_FOC_3SH                  (2U)

/***********************************************************************************************************************
 * Device Related
 **********************************************************************************************************************/
#define   AA_STEP                                   (1U)
#define   AB_STEP                                   (2U)

#define  STANDARD_SVM_7_SEGMENT                     (1U)
#define  STANDARD_SVM_5_SEGMENT                     (2U)

/* Current Sensing Feedback Scheme */
#define  THREE_SHUNT_ASYNC_CONV                     (1U)
#define  THREE_SHUNT_SYNC_CONV                      (2U)

/*-----------------------------------Control Scheme  ------------------------------------------------------------- */
#define  SPEED_INNER_CURRENT_CTRL                   (0U)
#define  VQ_VOLTAGE_CTRL                            (1U)
#define  VF_OPEN_LOOP_CTRL                          (2U)
/*      -----------------------------Motor Startup Configurations  -------------------------------------------------*/
#define ROTOR_PRE_ALIGN_NONE                        (0U)   /*!< Motor starts without pre-alignment */
#define ROTOR_PRE_ALIGNMENT                         (1U)   /*!< Motor starts with pre-alignment first */

#define MOTOR_STARTUP_DIRECT_FOC                    (0U)   /*!< IFX proprietary Direct FOC Close loop startup */
#define MOTOR_STARTUP_VF_OPEN_LOOP                  (1U)   /*!< Conventional V/F open loop startup */

#define PMSM_FOC_MOTOR_STATUS_STABLE                (0U)   /*!< Motor status stable */
#define PMSM_FOC_MOTOR_STATUS_TRANSITION            (1U)   /*!< Motor is in transition state */
/*      -----------------------------Over current / Over Voltage Protection Scheme  -------------------------------- */
#define ENABLED                                      (1U)
#define DISABLED                                     (0U)

#define MOTOR_PHASE_CURRENT_SENSE                    (0U)
#define DC_LINK_CURRENT_SENSE                        (1U)

/*      -----------------------------Gate driver Input logic definition  --------------------------------------------*/
#define PASSIVE_HIGH                                 (0U)
#define PASSIVE_LOW                                  (1U)
/*      -----------------------------CCU8 Passive LEVEL definition  ------------------------------------------------ */
#define CCU8_PASSIVE_LOW                             (0U)
#define CCU8_PASSIVE_HIGH                            (1U)
/*      -----------------------------UART USIC Channel Configuration  ---------------------------------------------- */
#define USIC_DISABLED_ALL                            (0U)
#define USIC0_CH0_P1_4_P1_5                          (1U)
#define USIC0_CH1_P1_2_P1_3                          (2U)

#define FALSE                                        (0U)
#define TRUE                                         (1U)
/*      ----------------------------- Reference Speed Adjustment Method  --------------------------------------------*/
#define DISABLED                                     (0U)
#define ENABLED                                      (1U)
/*--------------------------------------------------- SVM with Pseudo Zero Vectors ----------------------------------*/
#define USER_INVERSE_SVM_LAMDA                      (float)(20.0)
/*      --------------------------------------------------- MCU Parameters ----------------------------------------- */
#define USER_MCLK_FREQ_MHz                          (48U)       /* CPU Clock in Mhz*/
#define USER_PCLK_FREQ_MHz                          (96U)       /* Peripheral CLK frequency = double of CPU Clock */
#define USER_CCU8_PRESCALER                         (1U)
#define USER_CORDIC_MPS                             (2.0f)      /* CORDIC module MPS Setting value */
#define CORDIC_K                                    (1.646760258f)                  /* CORDIC SCALE (Inherent Gain Factor) */
#define CORDIC_SHIFT                                (14)        /* 8 ~ 16. Shift for CORDIC input / output registers, whose [7:0] are 0x00. Normally no need change.*/

/* --------------------------------------------------- Parameters for Startup Lock / Stall Detection -------------- */
#define USER_MAX_RETRY_MOTORSTARTUP_STALL           (3U)        /* Max retry of Motor startup if stall  */
/*      --------------------------------------------------- MISC Constant ----------------------------------------- */
#define USER_SQRT_3_CONSTANT                        (1.7320508075f)
#define USER_CCU4_DEBUG_KHZ                         (160U)
#define PI                                          (3.1415926536f)
#define PI_Q16                                      (205887.416f)
#define SHIFT_BIAS_LPF                              (2)                     /* Shift times for unity gain LPF: Y[n] = Y[n-1] + (X[n]-Y[n-1])>>SHIFT_BIAS_LPF. */

#define SQRT3                                       (1.732050807569f)       /* √3 */
#define DIV_SQRT3                                   (591)                   /* ((int16_t)((1/SQRT3) * (1<<SCALE_SQRT3))) */
#define SCALE_SQRT3                                 (10U)
#define DIV_SQRT3_Q14                               (9459U)                 /* (1/√3)*2^14 */
#define SCALE_DIV_3                                 (14U)                   /* For 1/3 scaling. */
#define DIV_3_Q14                                   (5461U)                 /* ((int16_t)((1/3) * (1<<SCALE_DIV_3))) */
#define DIV_SQRT2_Q14                               (11585U)                /* ((1/SQRT2) * (1<<14)) */

#define PMSM_FOC_ANGLE_000_DEGREE_Q31               (0)                     /* 0° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_030_DEGREE_Q31               (357913941)             /* 30° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_060_DEGREE_Q31               (715827883)             /* 60° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_090_DEGREE_Q31               (1073741824)            /* 90° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_120_DEGREE_Q31               (1431655765)            /* 120° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_150_DEGREE_Q31               (1789569707)            /* 150° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_180_DEGREE_Q31               (2147483648)            /* 180° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_210_DEGREE_Q31               (-1789569707)           /* 210° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_240_DEGREE_Q31               (-1431655765)           /* 240° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_270_DEGREE_Q31               (-1073741824)           /* 270° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_300_DEGREE_Q31               (-715827883)            /* 300° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_330_DEGREE_Q31               (-357913941)            /* 330° angle (0 ~ 2^31 represent electrical angle 0° ~ 180° in CORDIC) */
#define PMSM_FOC_ANGLE_INVALID                      (555)

#define CORDIC_CIRCULAR_VECTORING_MODE              (0x62)                  /* CORDIC: Circular Vectoring Mode (default). MPS: Divide by 2 (default).*/
#define CORDIC_CIRCULAR_ROTATION_MODE               (0x6A)                  /* CORDIC: Circular Rotation Mode. MPS: Divide by 2 (default).*/
#define CORDIC_HYPERBOLIC_VECTORING_MODE            (0x66)                  /* CORDIC: Hyperbolic Vectoring Mode. MPS: Divide by 2 (default).*/

#define CORDIC_CIRCULAR_MPS_BY_K_SCALED             (311)                   /* CORDIC MPS/K ->(2/1.64676)* 2^CORDIC_MPS_BY_K_SCALE */
#define CORDIC_HYPERBOLIC_MPS_BY_K_SCALED           (618)                   /* CORDIC MPS/K ->(2/0.828159360960)* 2^CORDIC_MPS_BY_K_SCALE */
#define CORDIC_MPS_BY_K_SCALE                       (8)                     /* CORDIC MPS/K scaling factor */

#define DIRECTION_INC                               (0U)                    /* Motor rotation direction - rotor angle increasing */
#define DIRECTION_DEC                               (1U)                    /* Motor rotation direction - rotor angle increasing */
#define ADJUST_DONE                                 (1U)                    /* Parameter adjustment has been done */

#define IS_GPIO_HIGH                                (1U)
#define IS_GPIO_LOW                                 (0U)

#define CFR_PHASE_U                                 (0U)
#define CFR_PHASE_V                                 (1U)
#define CFR_PHASE_W                                 (2U)

#define YES_CATCH_FREE                              (1U)
#define NO_CATCH_FREE                               (0U)

#define MAX_U_Q15                                   (32767U)

/**********************************************************************************************************
 * Scaling parameters for speed to angle conversion
 */
#define N_ESPEED_T                                  ((1U<<15)-1)
#define N_EROTORANGLE_RAD                           (float)PI
#define N_EROTORANGLE_T                             ((1U<<31)-1)

/*****************************************************************************************************************************************
 * Torque estimation constants
 *****************************************************************************************************************************************/
#define THREE_BY_TWO_Q14                           (24576)

/* Miscellaneous Constants */
#define DISABLED                              (0U)
#define ENABLED                               (1U)

#define UART_MODE                             (1U)
#define SWD_MODE                              (2U)

/*********************************************************************************************************************
 *                                      Configuration of 6EDL7141
 ********************************************************************************************************************/
#define PARAMETER_VERSION (1U)                /* set to 0xFFFF for developer, always program and load default value */
#define FIRMWARE_VERSION  (0x0001)


#if defined ( __GNUC__ )
#define RAM_ATTRIBUTE                         __RAM_FUNC
//#define RAM_ATTRIBUTE
#else
#define RAM_ATTRIBUTE
#endif

typedef struct
{
    union
    {
        struct
        {
            uint16_t precision;
            int16_t  output;
        };
        int32_t sum;
    };
} FILTER_TYPE_t;

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_CONST_MARCOS_H_ */
