/**
 * @file pmsm_foc_uart.h
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
#ifndef PMSM_FOC_UART_H_
#define PMSM_FOC_UART_H_

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../Configuration/pmsm_foc_user_input_config.h"

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MCUInit
 * @{
 */


/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#if(UART_ENABLE == USIC0_CH1_P1_2_P1_3)
#define USIC0_CHX_IN0	(USIC0_CH1->IN[0])        /* UART uses USIC0_CH1 */
#define USIC0_CHX_TRBSR	(USIC0_CH1->TRBSR)
#define USIC0_CHX_OUTR	(USIC0_CH1->OUTR)
#elif(UART_ENABLE == USIC0_CH0_P1_4_P1_5)
#define USIC0_CHX_IN0 (USIC0_CH0->IN[0])        // UART uses USIC0_CH0
#define USIC0_CHX_TRBSR (USIC0_CH0->TRBSR)
#define USIC0_CHX_OUTR  (USIC0_CH0->OUTR)
#endif

#if(UART_ENABLE != USIC_DISABLED_ALL)
#define UART_FIFO_BUFFER  USIC0_CHX_IN0           /* USIC Transmit FIFO Buffer. */
#endif

#define DIFF_09_TO_ASCII  (0x30)                /* Difference between ASCII of an integer 0~9 and the integer. */
#define DIFF_AF_TO_ASCII  (0x37)                /* Difference between ASCII of an integer 0xA ~ 0xF and the integer. */
#define STRING_MAX_NO   (32)                /* Max length of a string (text). */
#define UART_FIFO_SIZE    (0x05UL)    /* FIFO buffer size 32, for UART Transmit/Receive Buffer */

#define RMD_FROM_DIV    1               /*
                                         * Read remainder from DIV MATH->RMD (0.2us faster code).
                                         * Comment out to calculate remainder by CPU.
                                         */

/* BAUD_RATE_xxxxxx = MCLK/( (BRG_PDIV+1) x (BRG_DCTQ+1) x (BRG_PCTQ+1) ) * (FDR_STEP/1024). */

#define FDR_STEP  590UL       /* UART baud rate constants for 460.8kbps @ MCLK=32MHz */
#define BRG_PDIV  3UL
#define BRG_DCTQ  9UL
#define BRG_PCTQ  0UL

#define UART_RATE   (3U)            /* To determine UART communication frequency. */
#define UART_SPEED_UPDATE_RATE    (UART_RATE * 30U)	/* To determine frequency of UART speed update (for debug only). */

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/

/**
 * @brief Initialize USIC Module
 *
 * @param  None
 * @retval None
 */
void pmsm_foc_uart_init (void);

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_MCUINIT_PMSM_FOC_UART_H_ */

