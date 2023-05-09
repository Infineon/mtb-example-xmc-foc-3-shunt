/**
 * @file pmsm_foc_adc.h
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
#ifndef PMSM_FOC_ADC_H_
#define PMSM_FOC_ADC_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../ToolInterface/Register.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MCUInit
 * @{
 */
/*********************************************************************************************************************
 * MACRO
 ********************************************************************************************************************/
#define SHS0_CALOC0 ((uint32_t *)0x480340E0)
#define SHS0_CALOC1 ((uint32_t *)0x480340E4)
#define SHS0_CALCTR ((uint32_t *)0x480340BC)

#define SHS_CALLOC0_CLEAR_OFFSET (0x8000U)
#define REG_RESET (0x00U)
#define GLOBCFG_CLEAR (0x80030000U)
#define CLEAR_OFFSET_CALIB_VALUES         *SHS0_CALOC0 = SHS_CALLOC0_CLEAR_OFFSET;\
                                          *SHS0_CALOC1 = SHS_CALLOC0_CLEAR_OFFSET
/** Delay cycles to complete startup calibration */
#define VADC_CALIBRATION_CYCLE_DELAY  (20U)
/** Trigger dummy conversion for 9* 2000 times.*/
#define VADC_DUMMY_CONVERSION_COUNTER (18000U)


/*********************************************************************************************************************
 * ENUMS
 ********************************************************************************************************************/
/* Enumeration for MCU with internal opamp gain selection */
typedef enum SHS_GAIN_FACTOR
{
  SHS_GAIN_FACTOR_1 = 0,   /**< Select gain factor 1 */
  SHS_GAIN_FACTOR_3,       /**< Select gain factor 3 */
  SHS_GAIN_FACTOR_6,       /**< Select gain factor 6 */
  SHS_GAIN_FACTOR_12       /**< Select gain factor 12 */
}SHS_GAIN_FACTOR_t;


/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
typedef struct ADC_TYPE_t
{
  int32_t adc_bias_iu;			/* Bias of ADC Iu. */
  int32_t adc_bias_iv;			/* Bias of ADC Iv. */
  int32_t adc_bias_iw;			/* Bias of ADC Iw. */
  int32_t adc_bias_idc;		  /* Bias of dc-link current amplifier, or on-chip gain */

  int32_t adc_res_pot;			/* ADC Value of potentiometer (POT) */
  int32_t adc_res_vdc;			/* ADC Value of inverter DC link voltage Vdc */
  int32_t adc_res_idc;          /* ADC Value of inverter DC link current Idc*/
  int32_t adc_res_temp;         /* ADC Value of temperature sensor */

  uint16_t adc_res_iu;		    /* ADC result Motor Ph-U */
  uint16_t adc_res_iv;          /* ADC result Motor Ph-V */
  uint16_t adc_res_iw;          /* ADC result Motor Ph-W */

}ADC_t;


/*********************************************************************************************************************
 * EXTETRN
 ********************************************************************************************************************/
extern ADC_t ADC;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC module for scan configuration. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_PhCurrentInit(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC module with the associated configuration structure for 3-shunt phase current sensing. <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_ModuleInit(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for potentiometer voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_PotInit(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for temperature sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_TEMPInit(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for DC link voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_VDCInit(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for KitID sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_KitIDInit(void);

/**
 * @param None
 * @return
 *    Kit ID 0 - 31
 *
 * \par<b>Description:</b><br>
 * read the Kit ID number based on voltage setting at the adc port  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
uint16_t read_kit_id(void);

/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Initializes the VADC channel for DC Link voltage sensing.  <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_VADC_Init(void);

/* This API is used for VADC gain calibration. */
/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Calibrates VADC by triggering dummy conversion for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void PMSM_FOC_VADC_GainCalib(void);

/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Turn on Group 0 converter to calibrates VADC by triggering dummy conversions.
 * This startup calibration is required only for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void PMSM_FOC_VADC_StartupCalib(void);


/**
 * @}
 */

/**
 * @}
 */

#endif /* MCUINIT_ADC_H_ */
