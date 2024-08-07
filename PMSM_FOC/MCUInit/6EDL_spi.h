/**
 * @file 6EDL_spi.h
 * @brief SPI interface for middleware and application code
 *
 **********************************************************************************************************************
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

#ifndef MCUINIT_6EDL_SPI_H_
#define MCUINIT_6EDL_SPI_H_

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include <xmc_common.h>
#include "xmc_spi.h"
#include "xmc_usic.h"
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../Configuration/pmsm_foc_gatedrv_config.h"
#include "../ToolInterface/Register.h"

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
/* SPI configuration */
#define SPI_MAS_CH       XMC_SPI1_CH1
#define SPI_BAUD_RATE    8000000
#define SPI_WORD_LENGTH  (8U)
#define SPI_FRAME_LENGTH (24U)

/* GUI_6EDL7141_INTEGRATION SPI pin assignment */
#define MOSI_PIN         P0_0 /* USIC1_CH1.DOUT0, ALT9 */
#define MISO_PIN         P0_1 /* USIC1_CH1.DX0B */
#define SCLK_PIN         P0_3 /* USIC1_CH1.SCLKOUT, ALT8 */
#define SELO_PIN         P0_4 /* USIC1_CH1.SELO0, ALT8 */

#define IRQ9_IRQn        USIC0_0_IRQn

/* Register write and read definition */
#define REG_WRITE        0x80
#define REG_READ         0x7F


#define BIT_RIGHT_SHIFT  (8)
#define WORD_AND_MASK    (0x0FF)

/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
/**
 * @brief API to initialize USIC SPI peripherals
 *
 * @param None
 *
 * @retval None
 */
void SPI_master_init(XMC_USIC_CH_t * const channel);

/**
 * @brief To send the read command to the SPI channel.
 * The RegAddr is the address of register to read.
 * Data parameter is a pointer to store the two bytes of data read from the register
 *
 * @param None
 *
 * @retval 16 bit unsigned integer
 */
uint16_t read_word_16b(uint8_t RegAddr);

/**
 * @brief To write data to register via SPI channel.
 * The RegAddr is the address of register to write to.
 * Data parameter is a pointer which stored the two bytes of data written to the register
 *
 * @param None
 *
 * @retval None
 */
void write_word_16b(uint8_t RegAddr, uint16_t data);

/**
 * @}
 */

/**
 * @}
 */

#endif /* MCUINIT_6EDL_SPI_H_ */
