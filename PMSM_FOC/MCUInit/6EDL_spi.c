/**
 * @file 6EDL_SPI.c
 * @brief SPI interface for middleware and application code
 *
 **********************************************************************************************************************
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

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "6EDL_spi.h"
#include "XMC1400.h"
#include "pmsm_foc_gpio.h"

/*********************************************************************************************************************
 * DATA STRUCTURE
 ********************************************************************************************************************/
uint8_t error_SPI;
extern SYSTEM_VAR_t SystemVar;

/* SPI configuration structure */
XMC_SPI_CH_CONFIG_t SPI_config =
{
  .baudrate = SPI_BAUD_RATE,
  .bus_mode = XMC_SPI_CH_BUS_MODE_MASTER,
  .selo_inversion = XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS,  //set Slave select active low
  .parity_mode = XMC_USIC_CH_PARITY_MODE_NONE
};

/* GPIO SPI TX pin configuration */
XMC_GPIO_CONFIG_t tx_pin_config =
{
  .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT9
};

/* GPIO SPI DX pin configuration */
XMC_GPIO_CONFIG_t dx_pin_config =
{
  .mode = XMC_GPIO_MODE_INPUT_TRISTATE
};

/* GPIO SPI SELO pin configuration */
XMC_GPIO_CONFIG_t selo_pin_config =
{
  .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8,
};

/* GPIO SPI SCLKOUT pin configuration */
XMC_GPIO_CONFIG_t clk_pin_config =
{
  .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT8,
};

/*********************************************************************************************************************
 * LOCAL ROUTINE
 ********************************************************************************************************************/


/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 * @brief API to initialize USIC SPI peripherals
 *
 * @param None
 *
 * @retval None
 */
void SPI_master_init(XMC_USIC_CH_t * const channel)
{
  /* Initialize SPI master*/
  XMC_SPI_CH_Init(channel, &SPI_config);
  XMC_SPI_CH_SetWordLength(channel, SPI_WORD_LENGTH);
  XMC_SPI_CH_SetFrameLength(channel, SPI_FRAME_LENGTH);
  XMC_SPI_CH_SetBitOrderMsbFirst(channel);
  XMC_SPI_CH_EnableSlaveSelect(channel, XMC_SPI_CH_SLAVE_SELECT_0);
  XMC_SPI_CH_ConfigureShiftClockOutput(channel, XMC_SPI_CH_BRG_SHIFT_CLOCK_PASSIVE_LEVEL_0_DELAY_DISABLED,
      XMC_SPI_CH_BRG_SHIFT_CLOCK_OUTPUT_SCLK); //Falling sclk sampling and no polarity inversion
  XMC_SPI_CH_SetSlaveSelectPolarity(channel, XMC_SPI_CH_SLAVE_SEL_INV_TO_MSLS); //Slave select is active low to communicate with 6EDL7141

  /* Initialize FIFO */
  XMC_USIC_CH_RXFIFO_Configure(channel, 40, XMC_USIC_CH_FIFO_SIZE_8WORDS, 2); //receive from SPI slave: limit is 1, when FIFO is full with 2 word and a 3nd is received, the event happens.
  //this is used in main to while until 3 words are received
  XMC_USIC_CH_TXFIFO_Configure(channel, 32, XMC_USIC_CH_FIFO_SIZE_8WORDS, 0);

  /* Configure input multiplexor */
  XMC_SPI_CH_SetInputSource(channel, XMC_SPI_CH_INPUT_DIN0, USIC1_C1_DX0_P0_1); //P0.1 DX0B
  /* Start operation. */
  XMC_SPI_CH_Start(channel);

  /* Initialize GPIO */
  XMC_GPIO_Init(MOSI_PIN, &tx_pin_config);
  XMC_GPIO_Init(MISO_PIN, &dx_pin_config);
  XMC_GPIO_Init(SELO_PIN, &selo_pin_config);
  XMC_GPIO_Init(SCLK_PIN, &clk_pin_config);

  error_SPI = 0;
}

/**
 * @brief To send the read command to the SPI channel.
 * The RegAddr is the address of register to read.
 * Data parameter is a pointer to store the two bytes of data read from the register
 *
 * @param None
 *
 * @retval 16 bit unsigned integer
 */
uint16_t read_word_16b(uint8_t RegAddr)
{
  uint16_t read_data = 0;
  uint8_t data_high, data_low;
  volatile uint8_t addr = RegAddr;
  XMC_USIC_CH_RXFIFO_Flush(SPI_MAS_CH);
  SPI_MAS_CH->IN[0] = RegAddr & REG_READ;
  SPI_MAS_CH->IN[0] = 0x00;
  SPI_MAS_CH->IN[0] = 0x00;
  while ((XMC_USIC_CH_RXFIFO_GetEvent(SPI_MAS_CH) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD) == 0) {}; /*wait for received data /2 words*/
  XMC_USIC_CH_RXFIFO_ClearEvent(SPI_MAS_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
  SPI_MAS_CH->OUTR; //data read of invalid data
  data_high = SPI_MAS_CH->OUTR; //data read of MSB
  data_low = SPI_MAS_CH->OUTR;  //data read of LSB
  read_data = ((data_high << 8) + data_low);
  if ((addr == 0x07) && (read_data == 0x00))
  {
	  error_SPI = 1;
  }
  return read_data;
}

/**
 * @brief To write data to register via SPI channel.
 * The RegAddr is the address of register to write to.
 * Data parameter is a pointer which stored the two bytes of data written to the register
 *
 * @param None
 *
 * @retval None
 */
void write_word_16b(uint8_t RegAddr, uint16_t data)
{
  XMC_USIC_CH_RXFIFO_Flush(SPI_MAS_CH);
  SPI_MAS_CH->IN[0] = RegAddr | REG_WRITE;
  SPI_MAS_CH->IN[0] = (uint8_t)(data >> 8);
  SPI_MAS_CH->IN[0] = (uint8_t)data;
  while ((XMC_USIC_CH_RXFIFO_GetEvent(SPI_MAS_CH) & XMC_USIC_CH_RXFIFO_EVENT_STANDARD) == 0) {};  //wait for received data /2 words
  XMC_USIC_CH_RXFIFO_ClearEvent(SPI_MAS_CH, XMC_USIC_CH_RXFIFO_EVENT_STANDARD);
  SPI_MAS_CH->OUTR; //data read to clear buffer
  SPI_MAS_CH->OUTR; //data read of clear buffer
  SPI_MAS_CH->OUTR; //data read of clear buffer
}

