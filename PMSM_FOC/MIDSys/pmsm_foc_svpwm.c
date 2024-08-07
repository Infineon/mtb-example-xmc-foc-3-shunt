/**
 * @file pmsm_foc_svpwm.c
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
#include "PMSM_FOC/MIDSys/pmsm_foc_svpwm.h"
#include <xmc_common.h>              /* SFR declarations of the selected device */
#include "../ControlModules/pmsm_foc_functions.h"
#include "../MIDSys/pmsm_foc_pi.h"
#include "../IPLib/pmsm_foc_ip.h"
#include "../ToolInterface/uCProbe.h"
#include "../ToolInterface/Register.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define DEGREE_60                  (10922U)                   /* 60Â° angle (0 ~ 2^16 represent electrical angle 0Â° ~ 360Â°). */
#define RATIO_T0_111               (2U)                       /* = 2 for standard SVM. */
#define RATIO_T0_111               (2U)                       /* = 2 for standard SVM.*/
#define SHIFT_OVERMODULATION       (6U)                       /* Overmodulation resolution increase for calculations */

/*********************************************************************************************************************
 * GLOBAL DATA
*********************************************************************************************************************/
extern const uint16_t SIN60_TAB[];  /* Sine LUT used for SVM calculations, array size 256 or 1024. */

/*********************************************************************************************************************
 * LOCAL DATA
*********************************************************************************************************************/
uint16_t sampling_critical_vector;
SVPWM_t SVPWM;

/*********************************************************************************************************************
 * LOCAL API DECLARATION
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
volatile int32_t duty1,duty2,duty3;
/* API to update SVPWM CCU8 duty cycles based upon modulation scheme configured  */
PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_SVPWM_Update(uint16_t amplitude, uint16_t angle)
{
  /* SVM time T1, T2. */
  uint32_t t1;
  uint32_t t2;
  uint16_t t1nt2; /* Time (T1+T2). */
  uint16_t angle_temp;
  uint16_t sector_angle;
  uint32_t t0;

  SVPWM.previous_sector_num = SVPWM.current_sector_num; /* Record sector information of last PWM cycle. */

  /* Angle: 0 ~ 2^16 represent electrical angle 0° ~ 360° */
  angle_temp = (angle * 6U) >> ANGLETEMP_SHIFT;
  sector_angle = angle_temp & SECTOR_ANGLE_AND; /* Relative angle θrel in each sector. */
  SVPWM.current_sector_num = angle_temp >> SECTOR_NO_SHIFT; /* Update new SVM sector number */

  /* Calculate T1 / T2 by LUT. */
#if defined (__ICCARM__)
#pragma diag_suppress=Ta023
#endif

#if defined (__ICCARM__)
#pragma diag_suppress=Ta022
#endif
  t1 = (uint16_t)(((uint32_t)((amplitude * SIN60_TAB[MAX_LUT_INDEX - sector_angle]) >> 15) * SVM_LUT_SCALE) >> (15 + SCALE_UP_SINE_LUT));
  t2 = (uint16_t)(((uint32_t)((amplitude * SIN60_TAB[sector_angle]) >> 15) * SVM_LUT_SCALE) >> (15 + SCALE_UP_SINE_LUT));

#if defined (__ICCARM__)
#pragma diag_default=Ta022
#endif

#if defined (__ICCARM__)
#pragma diag_default=Ta023
#endif

  t1nt2 = (uint16_t)(t1 + t2); /* Temp variable for (T1+T2) <= CCU8_PERIOD_REG. */

  if (t1nt2 > SVPWM.pwm_period_reg_val)
  {
    MATH->DIVCON = (0x00008004 | (SHIFT_OVERMODULATION << 16UL) | (SHIFT_OVERMODULATION << 8UL));
    MATH->DVD = t1 * SVPWM.pwm_period_reg_val;
    MATH->DVS = t1nt2;
    t1nt2 = SVPWM.pwm_period_reg_val;

    /* CPU wait */
    while (MATH->DIVST);

    t1 = MATH->QUOT;
    t2 = SVPWM.pwm_period_reg_val - t1;
  }

  t0 = SVPWM.pwm_period_reg_val - t1nt2;

  if (t0 > T0_THRESHOLD)
  {
    SVPWM.flag_3or2_adc = USE_ALL_ADC; /* To use all (e.g.: three) ADC samplings for current reconstruction. */
  }
  else
  {
    SVPWM.flag_3or2_adc = USE_2_ADC; /* To use two ADC samplings for current reconstruction. */
  }

  /* Calls modulator function according to the user configuration */
  SVPWM.modulation_func_ptr(t0, t1, t2, t1nt2);

  if (sampling_critical_vector > SVPWM.t_max)
  {
    SVPWM.invalid_current_sample_flag = 1U;
  }
  else
  {
    SVPWM.invalid_current_sample_flag = 0U;
  }

  duty1 = SVPWM.ccu8_phu_module_ptr->CR1S;
  duty2 = SVPWM.ccu8_phv_module_ptr->CR1S;
  duty3 = SVPWM.ccu8_phw_module_ptr->CR1S;


  /* Enable shadow transfer for slice 0,1,2 for CCU80 Kernel. */
  CCU8_MODULE->GCSS |= (uint32_t) (XMC_CCU8_SHADOW_TRANSFER_SLICE_0 | XMC_CCU8_SHADOW_TRANSFER_SLICE_1
      | XMC_CCU8_SHADOW_TRANSFER_SLICE_2);
}

/* This function calculates the duty cycle for each phase as per 7 Segment(Continuous) SVPWM modulation */
void PMSM_FOC_SVPWM_7Seg(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t1nt2)
{
    /* 7-segment SVM, T0, T0_111 for first [111], T0_111 + T1/2, T0_111 + T2/2, T0_111 + (T1+T2)/2. */
    uint16_t t0_111;
    uint16_t t0nhalft1;
    uint16_t t0nhalft2;
    uint16_t t0nhalft1nt2;
    uint16_t pwm_period;

    pwm_period = SVPWM.pwm_period_reg_val;
    t0_111 = t0 >> RATIO_T0_111;                                                            /* T0_111, time of first [111]. */
    t0nhalft1 = (t0 + (uint16_t)(t1 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;               /* T0_111 + T1/2. */
    t0nhalft2 = (t0 + (uint16_t)(t2 << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;               /* T0_111 + T2/2. */
    t0nhalft1nt2 = (t0 + (uint16_t)((t1 + t2) << (RATIO_T0_111 - 1U))) >> RATIO_T0_111;     /* T0_111 + (T1+T2)/2. */

    /* Standard 7-segment symmetric PWM: */
    switch (SVPWM.current_sector_num)
    {
        case 0:                                                     /* Sector A */

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft2;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft2);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) t0_111;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);
        sampling_critical_vector = t2;
        break;

        case 1:                                                     /* Sector B */

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft1;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) t0_111;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);
        sampling_critical_vector = t1;
        break;

        case 2:                                                 /* Sector C */

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) t0_111;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft2;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft2);
        sampling_critical_vector = t2;
        break;

        case 3:                                                     /* Sector D */

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) t0_111;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) t0nhalft1;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);
        sampling_critical_vector = t1;
        break;

        case 4:                                                     /* Sector E */

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft2;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft2);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) t0_111;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);
        sampling_critical_vector = t2;
        break;

        case 5:                                                     /* Sector F */

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) t0nhalft1nt2;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1nt2);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) t0_111;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - t0_111);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) t0nhalft1;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - t0nhalft1);
        sampling_critical_vector = t1;
        break;

        default:
        // Control should not come here.
        break;
    }
}

/* This function calculates the duty cycle for each phase as per 5 Segment(Discontinuous) SVPWM modulation */
void PMSM_FOC_SVPWM_5Seg(uint16_t t0, uint16_t t1, uint16_t t2, uint16_t t1nt2)
{
    /* 5-segment SVM, (T1+T2)/2, T2/2, T1/2 */
    uint16_t half_t1nt2;
    uint16_t half_t2;
    uint16_t half_t1;
    uint16_t pwm_period;

    pwm_period = SVPWM.pwm_period_reg_val;

    half_t2 = t2 >> 1U;                    // T2/2.
    half_t1 = t1 >> 1U;                    // T1/2.
    half_t1nt2 = t1nt2>> 1U;              // (T1+T2)/2.

    /* Standard 5-segment symmetric PWM: */
    switch (SVPWM.current_sector_num)
    {
        case 0:                        // Sector A

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) half_t1nt2;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) half_t2;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t2);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) 0;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period + 1);

        sampling_critical_vector = t2;
        break;

        case 1:                       // Sector B

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) half_t1;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) half_t1nt2;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) 0;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period + 1);

        sampling_critical_vector = t1;
        break;

        case 2:                       // Sector C

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) 0;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period + 1);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) half_t1nt2;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) half_t2;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t2);

        sampling_critical_vector = t2;
        break;

        case 3:                       // Sector D

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) 0;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period + 1);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) half_t1;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) half_t1nt2;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        sampling_critical_vector = t1;
        break;

        case 4:                       // Sector E

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) half_t2;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t2);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) 0;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period + 1);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) half_t1nt2;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        sampling_critical_vector = t2;
        break;

        case 5:                       // Sector F

        SVPWM.ccu8_phu_module_ptr->CR1S = (uint32_t) half_t1nt2;
        SVPWM.ccu8_phu_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1nt2);

        SVPWM.ccu8_phv_module_ptr->CR1S = (uint32_t) 0;
        SVPWM.ccu8_phv_module_ptr->CR2S = (uint32_t) (pwm_period + 1);

        SVPWM.ccu8_phw_module_ptr->CR1S = (uint32_t) half_t1;
        SVPWM.ccu8_phw_module_ptr->CR2S = (uint32_t) (pwm_period - half_t1);

        sampling_critical_vector = t1;
        break;

        default:
        // Control should not come here.
        break;
    }
}
