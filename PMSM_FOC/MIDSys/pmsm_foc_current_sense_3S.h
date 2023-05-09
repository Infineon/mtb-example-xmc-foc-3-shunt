/**
 * @file pmsm_foc_current_sense_3S.h
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
#ifndef PMSM_FOC_CURRENT_SENSE_3S_H_
#define PMSM_FOC_CURRENT_SENSE_3S_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../ControlModules/pmsm_foc_functions.h"
#include "../ToolInterface/Register.h"

#include "../Configuration/pmsm_foc_mcuhw_params.h"

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

/*********************************************************************************************************************
 * EXTERN
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @param None
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * To initialize value of 12-bit ADC current bias <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
void PMSM_FOC_GetCurrentBias(void);

/**
 * @param Previous_SVM_SectorNo previous SVM sector number \n
 * @param New_SVM_SectorNo      next SVM sector number \n
 * @param  HandlePtr pointer to an object of ADC Current.\n
 * @return
 *    None<BR>
 *
 * \par<b>Description:</b><br>
 * Read ADC result of 3-phase motor current <BR>\n
 *
 * \par<b>Related APIs:</b><br>
 * None.
 */
#if (USER_CURRENT_SENSING ==  THREE_SHUNT_SYNC_CONV)
/* API to read ADC result of the 3 shunt current */
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_VADC_GetPhasecurrent(uint16_t previous_svm_sector_num, uint16_t new_svm_sector_num,
                                       ADC_t* const handle_ptr)
{
  uint16_t I1 = 0;
  uint16_t I2 = 0;
  uint16_t I3 = 0;

  /* Read current ADC (ADC synchronous conversion) */

  I1 = VADC_I1_GROUP->RES[VADC_I1_RESULT_REG];
  I2 = VADC_I2_GROUP->RES[VADC_I2_RESULT_REG];
  I3 = VADC_I3_GROUP->RES[VADC_I3_RESULT_REG];

  #if((USER_ADC_CALIBRATION == ENABLED) && (UC_SERIES == XMC13))
  /* Clear offset calibration values*/
  CLEAR_OFFSET_CALIB_VALUES;
  #endif

  /* 3-phase current reconstruction */
  switch (previous_svm_sector_num)
  {
    case 0:
    case 5:
	/* Sectors A and F. ADC sequences - Iw -> Iv -> Iu */
	handle_ptr->adc_res_iu = I3;
	handle_ptr->adc_res_iv = I1;
	handle_ptr->adc_res_iw = I2;
    break;

    case 1:
    case 2:
	/* Sectors B and C. ADC sequences - Iw -> Iu -> Iv */
	handle_ptr->adc_res_iu = I1;
	handle_ptr->adc_res_iv = I3;
	handle_ptr->adc_res_iw = I2;

	if (MotorParam.BiDirectional == ENABLED )
	{
		/* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
		if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
		{
		  handle_ptr->adc_res_iv = I2;
		  handle_ptr->adc_res_iw = I3;
		}
	}
	break;

    default:
	/* Process for all other cases, Sectors D and E. ADC sequences - Iu -> Iv -> Iw */
	handle_ptr->adc_res_iu = I1;
	handle_ptr->adc_res_iv = I2;
	handle_ptr->adc_res_iw = I3;

	if (MotorParam.BiDirectional == ENABLED )
	{
		/* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
		if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
		{
		  handle_ptr->adc_res_iv = I3;
		  handle_ptr->adc_res_iw = I2;
		}
	}
	break;
  }

  /* If SVM sector changed */
  if (new_svm_sector_num != previous_svm_sector_num)
  {
    /* Rotating ADC alias */
    switch (new_svm_sector_num)
    {
      case 0:
      case 5:
      /* Sectors A and F. ADC sequences - Iw -> Iv -> Iu */
      VADC_G1->ALIAS = (((uint32_t)VADC_IU_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IV_G1_CHANNEL);
      VADC_G0->ALIAS = ((uint32_t)VADC_IW_G0_CHANNEL);
      break;

      case 1:
      case 2:
      /*  Sectors B and C. ADC sequences - Iw -> Iu -> Iv */
      VADC_G1->ALIAS = (((uint32_t)VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
      VADC_G0->ALIAS = ((uint32_t)VADC_IW_G0_CHANNEL);

	  if (MotorParam.BiDirectional == ENABLED )
	  {
		  /* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
		  if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
		  {
			/* ADC sequences - Iu -> Iv -> Iw */
			VADC_G1->ALIAS = (((uint32_t)VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
			VADC_G0->ALIAS = ((uint32_t)VADC_IV_G0_CHANNEL);
		  }
      }
      break;

      default:
      /* Process for all other cases, Sectors D and E. ADC sequences - Iu -> Iv -> Iw */
      VADC_G1->ALIAS = (((uint32_t)VADC_IW_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
      VADC_G0->ALIAS = ((uint32_t) VADC_IV_G0_CHANNEL);

	  if (MotorParam.BiDirectional == ENABLED )
	  {
		  /* May need swap SVPWM CCU8 duty cycles of phase V and W, so that motor reverses */
		  if (PMSM_FOC_CTRL.rotation_dir == DIRECTION_DEC)
		  {
			/* ADC sequences - Iu -> Iw -> Iv */
			VADC_G1->ALIAS = (((uint32_t)VADC_IV_G1_CHANNEL << VADC_G_ALIAS_ALIAS1_Pos) | VADC_IU_G1_CHANNEL);
			VADC_G0->ALIAS = ((uint32_t)VADC_IW_G0_CHANNEL);
		  }
	  }
      break;
    }
  }
}
#else
__STATIC_INLINE PMSM_FOC_RAM_ATTRIBUTE void PMSM_FOC_VADC_GetPhasecurrent(uint16_t previous_svm_sector_num, uint16_t new_svm_sector_num, ADC_t* const handle_ptr)
{
  handle_ptr->adc_res_iu = XMC_VADC_GROUP_GetResult(VADC_IU_GROUP,VADC_IU_RESULT_REG);
  handle_ptr->adc_res_iv = XMC_VADC_GROUP_GetResult(VADC_IV_GROUP,VADC_IV_RESULT_REG);
  handle_ptr->adc_res_iw = XMC_VADC_GROUP_GetResult(VADC_IW_GROUP,VADC_IW_RESULT_REG);
}
#endif


/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_CURRENT_SENSE_3S_H_ */
