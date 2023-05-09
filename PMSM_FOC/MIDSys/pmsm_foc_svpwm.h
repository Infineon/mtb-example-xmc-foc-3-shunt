/**
 * @file pmsm_foc_svpwm.h
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

#ifndef PMSM_FOC_PWMSVM_H_
#define PMSM_FOC_PWMSVM_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../PMSM_FOC/MCUInit/pmsm_foc_adc.h"
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../ToolInterface/Register.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MIDSys
 * @{
 */

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define SVM_USE_PZV          (0)        /* To indicate using SVM with Pseudo Zero Vectors (PZV) */
#define SVM_USE_STANDARD     (0xAD)     /* To indicate using standard SVM (3 or 4-segment) */

#if(USER_SVM_SINE_LUT_SIZE == 256U)     /* Look-Up Table (LUT) array size 256 */
#define ANGLETEMP_SHIFT   (8U)       /* For calculation of Angle_Temp */
#define SECTOR_ANGLE_AND  (0x00FFU)  /* For calculation of Sector_Angle */
#define SECTOR_NO_SHIFT   (8U)       /* For calculation of sector number */
#define MAX_LUT_INDEX     (255U)     /* Maximum angle index in LUT */
#else                                  /* Look-Up Table (LUT) array size 1024 */
#define ANGLETEMP_SHIFT   (6U)       /* For calculation of Angle_Temp */
#define SECTOR_ANGLE_AND  (0x03FFU)  /* For calculation of Sector_Angle */
#define SECTOR_NO_SHIFT   (10U)      /* For calculation of sector number */
#define MAX_LUT_INDEX     (1023U)    /* Maximum angle index in LUT */
#define SCALE_UP_SINE_LUT  (uint16_t)(15-(log10(CCU8_PERIOD_REG * 1.0 / 32767.0 * 32767.0)/log10(2)))
#define SVM_LUT_SCALE      (uint32_t)((CCU8_PERIOD_REG * 1.0 / 32767.0 * 32767.0)*(1 << SCALE_UP_SINE_LUT))
#endif

/*
 * T0 threshold time, to tell if PWM T0 has enough time for three valid ADC samplings, for 3-shunt phase current
 * sensing
 */
#define T0_THRESHOLD       (USER_PCLK_FREQ_MHz * 14)

/* To use all (e.g.: three) ADC samplings for current reconstruction, for 3-shunt phase current sensing */
#define USE_ALL_ADC       (0U)
#define USE_2_ADC         (1U)     /* To use two of three ADC samplings for current reconstruction */

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef void (*SVPWM_MODULATION_t)(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t1nt2);

typedef struct SVPWMType
{
  CCU8_CC8_TypeDef *ccu8_phu_module_ptr;      /*!< CCU8 module pointer for Phase U */
  CCU8_CC8_TypeDef *ccu8_phv_module_ptr;      /*!< CCU8 module pointer for Phase V */
  CCU8_CC8_TypeDef *ccu8_phw_module_ptr;      /*!< CCU8 module pointer for Phase W */

  SVPWM_MODULATION_t modulation_func_ptr;     /*!< Function pointer for the svpwm modulation - 7 Segment, 5 Segment */

  uint16_t pwm_period_reg_val;                /*!< PWM period register value */
  uint16_t vadc_trigger_point;                /*!< VADC conversion trigger point */
  uint16_t t_min;                             /*!< Minimum SVPWM time vector duration in which current sensing of 3 phase is possible */
  uint16_t t_max;                             /*!< Maximum SVPWM time vector duration */
  uint16_t current_sector_num;                /*!< Current new sector number: 0 ~ 5 (represent Sector A ~ F) in SVM space vector hexagon */
  uint16_t previous_sector_num;               /*!< SVM sector number of last PWM cycle, for 3-phase current reconstruction */
  uint16_t flag_3or2_adc;                     /*!< used for dynamic switching between current sampling */
  uint16_t invalid_current_sample_flag;       /*!< indicates invalid current sample */

} SVPWM_t;

extern SVPWM_t SVPWM;
/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param Amplitude   \n
 * @param Angle Rotor Angle \n
 * @return None<BR>
 *
 * \par<b>Description:</b><br>
 * To update SVPWM CCU8 duty cycles, based upon the configuration <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_SVPWM_Update(uint16_t amplitude, uint16_t angle);

/* This function calculates the duty cycle for each phase as per 7 Segment(Continuous) SVPWM modulation */
void PMSM_FOC_SVPWM_7Seg(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t1nt2);

/* This function calculates the duty cycle for each phase as per 5 Segment(Discontinuous) SVPWM modulation */
void PMSM_FOC_SVPWM_5Seg(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t1nt2);

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_MIDSYS_PMSM_FOC_PWMSVM_H_ */
