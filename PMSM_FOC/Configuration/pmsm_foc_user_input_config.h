

/**
 * @file user_input_config.h
 * @brief Control algorithm parameters which user need to configure.
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
 *@endcond
 ***********************************************************************************************************************/

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Configuration
 * @{
 */

#ifndef USER_INPUT_CONFIG_H_
#define USER_INPUT_CONFIG_H_

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include <xmc_common.h>
#include "pmsm_foc_common.h"
#include "math.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/


/*********************************************************************************************************************
 * Motor and power board selection
 ********************************************************************************************************************/
/* BLDC motor
 * Options -  CUSTOM_FOC_MOTOR */
#define MOTOR0_PMSM_FOC_MOTOR                       (CUSTOM_FOC_MOTOR)


/*********************************************************************************************************************
 *                                    Default value of PMSM FOC motor control
 ********************************************************************************************************************/
/****************************
 * Hardware group
 ***************************/
#define USER_CURRENT_SENSING                        (THREE_SHUNT_SYNC_CONV)         /*!<1. THREE_SHUNT_ASYNC_CONV 2. THREE_SHUNT_SYNC_CONV***fixed**/
#define WATCHDOG_CLOCK                              (DISABLED)                      /* 6EDL7141 watchdog clock on EN_DRV pin */

/****************************
 * System group
 ***************************/
#define USER_SLOW_CTRL_LOOP_PERIOD_uS               (1000.0F)                       /*!< Slow Control Loop Scheduler interrupt period. Range: 1000 to 10000 us*/
#define USER_IDC_ADC_BIAS                           (2048)                          /*!< DC link current bias */
#define USER_IU_ADC_BIAS                            (2048)                          /*!< Motor phase U current bias */
#define USER_IV_ADC_BIAS                            (2048)                          /*!< Motor phase V current bias */
#define USER_IW_ADC_BIAS                            (2048)                          /*!< Motor phase W current bias */
#define VOLTAGE_DIVIDER_R_HIGH                      (75.0f)                         /*!< DC link voltage sense resistor divider high side value ***fixed for IMD700A 3S eval board**/
#define VOLTAGE_DIVIDER_R_LOW                       (7.87f)                         /*!< DC link voltage sense resistor divider low side value ***fixed for IMD700A 3S eval board**/
#define USER_VDC_LINK_DIVIDER_RATIO                 (VOLTAGE_DIVIDER_R_LOW/(VOLTAGE_DIVIDER_R_HIGH+VOLTAGE_DIVIDER_R_LOW))   /*!< R2/(R2+R1) ratio for DC link voltage divider */
#define USER_DRIVERIC_DELAY_US                      (0.2f)                          /*!< Driver IC propagation delay in uSec. */
#define USER_BOOTSTRAP_PRECHARGE_TIME_MS            (100U)                          /*!< Initial Bootstrap pre-charging time in ms */

#define USER_INVERTER_ENABLE_LEVEL                  XMC_GPIO_OUTPUT_LEVEL_HIGH      /*!< Inverter Enable - required pin level for PWM output***fixed**/
#define USER_INVERTER_DISABLE_LEVEL                 XMC_GPIO_OUTPUT_LEVEL_LOW       /*!< Inverter Disable - required pin level for PWM output ***fixed**/

#define USER_BRAKE_ENABLE_LEVEL                     XMC_GPIO_OUTPUT_LEVEL_LOW       /*!< Brake Enable - required pin level to enable braking */
#define USER_BRAKE_DISABLE_LEVEL                    XMC_GPIO_OUTPUT_LEVEL_HIGH      /*!< Brake Disable - required pin level to enable braking*/

#define MAX_VREF_AMPLITUDE                          (32767.0f)                      /* maximum Vdc link amplitude Q15 */

#define USER_ROTOR_PRE_ALIGN_METHOD                 (ROTOR_PRE_ALIGN_NONE)                /*!< 1. ROTOR_PRE_ALIGNMENT 2. ROTOR_PRE_ALIGN_NONE */
#define USER_MOTOR_STARTUP_METHOD                   (MOTOR_STARTUP_DIRECT_FOC)      /*!< 1. MOTOR_STARTUP_DIRECT_FOC 2. MOTOR_STARTUP_VF_OPEN_LOOP */

#define USER_ROTOR_PRE_ALIGNMENT_V_RAMP_RATE        (100.0F)         /*!< in volt/sec. Voltage is ramped to USER_ROTOR_PRE_ALIGNMENT_VOLTAGE_V and then maintained for USER_ROTOR_PRE_ALIGNMENT_TIME_MS*/
#define USER_ROTOR_PRE_ALIGNMENT_VOLTAGE_V          (0.8F)           /*!< in volts  should be less than PMSM_FOC_SL_DC_LINK_VOLT */
#define USER_ROTOR_PRE_ALIGNMENT_TIME_MS            (100.0F)         /*!< Rotor startup alignment time in mSec. Min range: 1/PWM_Freuency */

#define USER_VQ_INITIAL_VALUE_V                     (0.8F)                          /*!<  Direct FOC Startup - Vq value initial value based upon load and max current */

#define USER_FOC_CTRL_SCHEME                        (SPEED_INNER_CURRENT_CTRL)      /*!<1. SPEED_INNER_CURRENT_CTRL 2. VQ_VOLTAGE_CTRL 3. VF_OPEN_LOOP_CTRL - Debugging purpose only*/
#define USER_MOTOR_BI_DIRECTION_CTRL                (ENABLED)                       /*!<1. ENABLED       2. DISABLED*/
#define USER_REF_SETTING                            (ENABLED)                       /*!< POTENTIOMETER 1. DISABLED 2. ENABLED */
#define USER_TH_POT_ADC                             (50U)                           /*!< Threshold POT ADC that Motor can enter or exit Motor idle state. ***fixed 50U**/
#define USER_POT_ADC_LPF                            (3U)                            /*!< POT adc filter configuration. ***fixed**/


/****************************
 * Protection group
 ***************************/
/*----------------------------Under/Over Voltage Protection ----------------------------------------------------------------*/
#define USER_VDC_UV_OV_PROTECTION                   (ENABLED)           /*!< DC bus under/over voltage protection. 1. ENABLED       2. DISABLED*/

#define USER_OVERCURRENT_PROTECTION                 (DISABLED)          /*!<1. ENABLED       2. DISABLED*/

#define USER_WATCH_DOG_TIMER                        (DISABLED)          /*!<1. ENABLED       2. DISABLED*/

#define USER_VDC_OVER_LIMIT                         ((uint16_t)(((USER_VDC_LINK_V * USER_VDC_LINK_DIVIDER_RATIO)/USER_MAX_ADC_VDD_V) * (1<<12) * 140.0 / 100.0))
#define USER_VDC_MIN_LIMIT                          ((uint16_t)(((USER_VDC_LINK_V * USER_VDC_LINK_DIVIDER_RATIO)/USER_MAX_ADC_VDD_V) * (1<<12) * 60.0 / 100.0))

/*----------------------------Over Current Protection ----------------------------------------------------------------------*/
#define USER_IDC_OVER_CURRENT_LIMIT_A               (USER_CURRENT_TRIP_THRESHOLD_A)    /*!<   Over current limit in Amps */
#define USER_OVER_CURRENT_DETECTION_SOURCE          (MOTOR_PHASE_CURRENT_SENSE)        /*!< 1. DC_LINK_CURRENT_SENSE       2.MOTOR_PHASE_CURRENT_SENSE */
#define USER_OVER_CURRENT_DETECTION_LPF             (4)                                /*!<0 - Filter Disabled, >0 - Filter enabled(Time const = PWM_PERIOD * 2^USER_PLL_LPF) */

/****************************
 * Motor group
 ***************************/
#if (MOTOR0_PMSM_FOC_MOTOR == CUSTOM_FOC_MOTOR)
#define  USER_MOTOR_R_PER_PHASE_OHM                 (0.36f)       /*!< Motor Resistance per phase in Ohm for PMSM Motor */
#define  USER_MOTOR_LS_PER_PHASE_uH                 (600.0f)      /*!< Motor Inductance Ls per phase in uH  */
#define  USER_MOTOR_POLE_PAIR                       (4.0f)        /*!< Motor Pole Pairs */
#define  USER_SPEED_HIGH_LIMIT_RPM                  (4000U)       /*!< No load Speed of User Motor*/
#define  USER_SPEED_LOW_LIMIT_RPM                   (200U)        /*!< Min Speed of User Motor, original 200U*/
#endif

/****************************
 * PWM group
 ***************************/
#define USER_CCU8_PWM_FREQ_HZ                       (20000U)                        /*!< CCU8 PWM Switching Frequency in Hz*/
#define USER_DEAD_TIME_US                           (0.75f)                         /*!< dead time, rise and fall values in us, Max 2.65uS for PCLK -96MHz,
                                                                                        if not sufficient then modify dead time prescalar  */
#define USER_CCU8_PASSIVE_LEVEL_OUT0                CCU8_PASSIVE_LOW                /*!< PWM output passive level required for driver IC for high side */
#define USER_CCU8_PASSIVE_LEVEL_OUT1                CCU8_PASSIVE_LOW                /*!< PWM output passive level required for driver IC for low side */

#define USER_CCU8_INPUT_TRAP_LEVEL                  XMC_CCU8_SLICE_EVENT_LEVEL_SENSITIVITY_ACTIVE_LOW /*!< Trap signal input level selection for ctrap to occur */

#define CCU8_PERIOD_REG                             ((uint32_t)((USER_PCLK_FREQ_MHz*1000000)/USER_CCU8_PWM_FREQ_HZ) - 1)

/****************************
 * Control Loop group
 ***************************/
/*----------------------------------------	V/F Start Up Parameters ----------------------------------------------- */
#define USER_VF_OFFSET_V                            (0.5f)		   /*!< V/F open loop - offset in V */
#define USER_VF_V_PER_HZ                            (0.08f) 	   /*!< V/F open loop - (V/F) constant */
#define USER_VF_TRANSITION_SPEED_RPM                (500)		   /*!< threshold Speed to transit from Open loop to closed loop */
#define USER_VF_SPEED_RAMPUP_RATE_RPM_PER_S         (100U)		   /*!< V/F open loop - speed ramp up rate */

/* ---------------------------------------------- Vq Voltgae Control Scheme Configuration ---------------------------------------- */
#define USER_VQ_REF_HIGH_LIMIT_V                   ((USER_VDC_LINK_V / USER_SQRT_3_CONSTANT) * 1.15)   /*!< 0 < USER_VQ_REF_HIGH_LIMIT_V <= VREF_MAX_V   VREF_MAX_V =  (USER_VDC_LINK_V / USER_SQRT_3_CONSTANT), multiply by 1.15 to achieve overmodulation in VQ control */
#define USER_VQ_REF_LOW_LIMIT_V                    (0.2f)                                           /*!< minimum Vq reference voltage required for motor to operate in close loop */

#define USER_VQ_RAMPUP_STEP                        (1U)                                             /*!< Vq voltage increment step in target count*/
#define USER_VQ_RAMPDOWN_STEP                      (1U)                                             /*!< Vq voltage decrement step in target count*/
#define USER_VQ_RAMP_SLEWRATE                      (3U)                                             /*!< USER_VQ_RAMP_SLEWRATE x PWM period, every cycle increase USER_VQ_RAMPUP or USER_VQ_RAMPDOWN */

/* ---------------------------------------------- Speed inner current Control Scheme Configuration -------------------------------- */
#define USER_SPEED_REF_HIGH_LIMIT_RPM              (USER_SPEED_HIGH_LIMIT_RPM)          /*!< User speed reference upper limit */
#define USER_SPEED_REF_LOW_LIMIT_RPM               (USER_SPEED_LOW_LIMIT_RPM)           /*!< User speed reference lower limit */
#define USER_SPEED_RAMPUP_RPM_PER_S                (50U)
#define USER_SPEED_RAMPDOWN_RPM_PER_S              (50U)
#define USER_SPEED_RAMP_SLEWRATE                   (2U)                                 /*!< USER_SPEED_RAMP_SLEWRATE x PWM period */
#define USER_RATIO_S                               (0U)                                 /*!< Minimum ramp up and down ratio for S-curve profile */

/* --------------------------------------------------- Torque Limiter --------------------------------------------------------------- */
#define USER_TORQUE_LIMITER                         DISABLED                            /*!< 1. ENABLED       2. DISABLED */
#define USER_IQ_LIMIT_Q15                           (2000)                              /*!< Torque Current Component Iq limit in Q14 */
#define USER_IQ_LIMIT_BLANKING_TIME                 (5000)                              /*!< Blanking time = x * pwm period */


/* --------------------------------------------------- SVM Switching Sequences ---------------------------------------------- */
#define USER_SVM_SWITCHING_SCHEME                   STANDARD_SVM_5_SEGMENT              /*!<1. STANDARD_SVM_7_SEGMENT          2. STANDARD_SVM_5_SEGMENT  */
#define USER_SVM_SINE_LUT_SIZE                      (1024U)                             /*!<1. Look-Up Table (LUT) array size 256  2.LUT array size 1024 */
#define USER_SCALE_UP_SINE_LUT                      (10U)                                /*!<  Scale up LUT by 10 to increase resolution */

#define USER_VDC_VOLT_COMPENSATION                  ENABLED                             /*!< DC bus voltage compensation. 1.ENABLED       2. DISABLED*/

/* ---------------------------------------- ADC & Motor Phase Offset Calibration ------------------------------------------------------------------ */
#define USER_ADC_CALIBRATION                        ENABLED                             /*!< ADC startup calibration. 1. ENABLED       2. DISABLED*/
#define USER_MOTOR_PH_OFFSET_CALIBRATION            ENABLED                             /*!< Motor phase current offset calibration. 1. ENABLED       2. DISABLED*/

/* -----------------------------------------Add d-q voltage decoupling components ----------------------------------------------------- */
#define USER_DQ_DECOUPLING                          DISABLED                            /*!< DQ decoupling 1. ENABLED       2. DISABLED*/

/* **************************************** PLL Observer Setting *****************************************************************/
#define USER_PLL_LPF                                (1)                                 /*!<0 - Filter Disabled, >0 - Filter enabled(Time const = PWM_PERIOD * 2^USER_PLL_LPF) */
#define USER_PLL_SPEED_LPF                          (2)                                 /*!<0 - Filter Disabled, >0 - Filter enabled(Time const = PWM_PERIOD * 2^USER_PLL_LPF) */


/* --------------------------------------------------- BRAKING CONFIG  -------------------------------------------------- */
#define USER_MOTOR_BRAKE_DUTY                       (90U)                                /*!< Brake duty percentage. Ex: 100 - Strong brake, 10 - Weak Brake  */
#define USER_BRAKING_VDC_MAX_LIMIT                  (115U)                               /*!< 115% of ideal DC link voltage = Vdc maximum limit, voltage clamping during brake */

/* -------------------------------------------- MicroInspector Enable/Disable  ---------------------------------------- */
#define USER_UCPROBE_GUI                            ENABLED                              /*!< 1. ENABLED       2. DISABLED */
#define USER_UCPROBE_OSCILLOSCOPE                   ENABLED                              /*!< 1. ENABLED       2. DISABLED */

/*********************************************************************************************************************
 *                                      Configuration of 6EDL7141
 ********************************************************************************************************************/
#define PWM_MODE_6EDL7141           PWM_MODE_6
#define PWM_MODE_1_HALL_ALT_RECIRC  DISABLED        /* 0: DISABLED, 1: ENABLED */
#define CSAMP_GAIN_6EDL7141         0               /* 0 to disable all amplifiers, or CS_GAIN 4, 8, 12, 16, 24, 32, 64, 96 */

/* Configuration of GUI 6EDL_SPI_LINK code */
#define GUI_6EDL7141_INTEGRATION    SWD_MODE        /* 0: DISABLED, 2: SWD_MODE */
#define WRITE_6EDL7141_REG_AT_INIT  ENABLED         /* 0: DISABLED, 1: ENABLED */

/************************ Power board parameters **************************************************************/
/* Power Inverter parameters */
#define USER_VDC_LINK_V                             (24.0f) 					    /*!< Inverter DC link voltage in Volts  */
#define USER_CURRENT_TRIP_THRESHOLD_A               (15.0f)						    /*!< threshold current for trip detection in Ampere */
#define USER_DC_SHUNT_OHM                           (0.050f)					    /*!< DC link shunt current resistor in ohm, Not used in this board */
#define USER_CURRENT_AMPLIFIER_GAIN                 (12.0f) 					    /*!< R_FEEDBACK (of equivalent amplifier) kohm using 6EDL7141 CS gain of 4/8/12/16*/
#define USER_MAX_ADC_VDD_V                          (5.0f)						    /*!< VDD5, maximum voltage at ADC */

#if  (MOTOR0_PMSM_FOC_BOARD == EVAL_IMD700A_FOC_3SH)
#define USER_R_SHUNT_OHM                            (0.010f)					    /*!< Phase shunt resistor in ohm, eval board 0.010 */
/*end of if  (MOTOR0_BLDC_SCALAR_BOARD == EVAL_IMD700A_FOC_3SH) */
#elif (MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7141_FOC_3SH)
#define USER_R_SHUNT_OHM                            (0.005f)					    /*!< Phase shunt resistor in ohm, eval board 0.005 */
#endif



/*----------------------------------------- PI Controller Parameters ---------------------------------------------- */

#define USER_PI_SPEED_KPP                           (2000)                          /*!<  SPEED PI CTRL - Proportional gain Kp, uint16_t. */
#define USER_PI_SPEED_KII                           (2)                             /*!<  SPEED PI CTRL - Integral gain Ki, uint16_t. */
#define USER_PI_SPEED_SCALE_KPKII                   (8+ USER_RES_INC)               /*!<  SPEED PI CTRL - kp,ki scale, uint16_t. */

#define USER_PI_PLL_KP                              (20)							/*!<  PLL PI CTRL - Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_KI                              (1) 							/*!<  PLL PI CTRL - Proportional gain Kp, uint16_t. */
#define USER_PI_PLL_SCALE_KPKI                      (9) 							/*!<  PLL PI CTRL - Proportional gain Kp, uint16_t. */

/*********************************************************************************************************************
 * Below Parameters usually not required any change
 ********************************************************************************************************************/
/*--------------------------------------------------- Speed PI Controller Parameters ---------------------------------------- */
#define USER_PI_SPEED_UK_LIMIT_MIN                       (-32768)                                   /* (-32767), 16. U[k] output limit LOW. */
#define USER_PI_SPEED_UK_LIMIT_MAX                       (32767)                                    /* MAX_I_REF. U[k] output limit HIGH. Normally no need change. */
#define USER_PI_SPEED_IK_LIMIT_MIN                       (-(1<<(30 - USER_PI_SPEED_SCALE_KPKI)))    /* (-(1<<30 - scale)). I[k] output limit LOW. */
#define USER_PI_SPEED_IK_LIMIT_MAX                       (1<<(30 - USER_PI_SPEED_SCALE_KPKI))       /* (1<<30 - scale). I[k] output limit HIGH. */

/* --------------------------------------------------- Torque/Iq PI Controller Parameters ------------------------------- */
/* Kp and Ki calculated from Motor parameter L and R. Normally no need change. */
#define USER_PI_TORQUE_UK_LIMIT_MIN                      (0)                                        /* U[k] output limit LOW. Normally no need change, originally -32768*/
#define USER_PI_TORQUE_UK_LIMIT_MAX                      (38000)                                    /* Set slightly more than 1.1547*32768 for overmodulation. Normally no need change. */
#define USER_PI_TORQUE_IK_LIMIT_MIN                      (-(1<<(30 - USER_PI_TORQUE_SCALE_KPKI)))   /* (-(1<<30 - scale)). I[k] output limit LOW. Normally no need change. */
#define USER_PI_TORQUE_IK_LIMIT_MAX                      (1<<(30 - USER_PI_TORQUE_SCALE_KPKI))      /* (1<<30 - scale). I[k] output limit HIGH. Normally no need change. */

/* --------------------------------------------------- Flux/Id PI Controller Parameters ---------------------------------------- */
/* Kp and Ki calculated from Motor parameter L and R. Normally no need change. */
#define USER_PI_FLUX_UK_LIMIT_MIN                       (-32768)                                    /* U[k] output limit LOW. Normally no need change. */
#define USER_PI_FLUX_UK_LIMIT_MAX                       (32767)                                     /* U[k] output limit HIGH. Normally no need change. */
#define USER_PI_FLUX_IK_LIMIT_MIN                       (-(1<<(30 - USER_PI_FLUX_SCALE_KPKI)))      /* (-(1<<30 - scale)). I[k] output limit LOW. Normally no need change. */
#define USER_PI_FLUX_IK_LIMIT_MAX                       (1<<(30 - USER_PI_FLUX_SCALE_KPKI))         /* (1<<30 - scale). I[k] output limit HIGH. Normally no need change. */

/* -----------------------------------------PLL Estimator PI Controller Parameters ---------------------------------------- */
#define USER_PI_PLL_IK_LIMIT_MIN                        (-(int32_t)(1U << (30U-(USER_PI_PLL_SCALE_KPKI - USER_RES_INC))))  //(-(1<<(30))) /* I[k] output limit LOW. */
#define USER_PI_PLL_IK_LIMIT_MAX                        ((1U << (30U-(USER_PI_PLL_SCALE_KPKI - USER_RES_INC))))//           /* I[k] output limit HIGH. */


/**************** pmsm_foc_variables_scaling ****************************************************/

/* *********************************************  Increase Angle (and Speed Resolution) ***********************************************************************/
#define USER_RES_INC                                (3U)

/* *********************************************  CORDIC SCALING *********************************************************************************************/
#define SCALEUP_MPS_K                               (8U)

/* ********************************************* MISC Integer Speed in SW to RPM speed of real world  ********************************************************/
#define SPEED_TO_RPM_SCALE                          (11U)

/**
 * @}
 */

/**
 * @}
 */
/**
 * @}
 */
#endif /* USER_INPUT_CONFIG_H_ */

