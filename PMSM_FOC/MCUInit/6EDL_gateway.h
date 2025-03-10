/**
 * @file 6EDL_gateway.h
 * @brief SPI communication API with 6EDL7141 registers
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

#ifndef MCUINIT_6EDL_GATEWAY_H_
#define MCUINIT_6EDL_GATEWAY_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup MCUInit
 * @{
 */

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "6EDL_spi.h"
#include "xmc_usic.h"
#include "xmc_gpio.h"
#include "xmc_uart.h"

#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../Configuration/pmsm_foc_gatedrv_config.h"
#include "../ToolInterface/Register.h"
#else
#include "../Configuration/pmsm_foc_user_input_config.h"
#include "../Configuration/pmsm_foc_gatedrv_config.h"
#include "../ToolInterface/Register.h"
#endif

/*********************************************************************************************************************
 * MACROS
 ********************************************************************************************************************/
#define GPIO_nBRAKE              P1_3
#define GPIO_AUTO_ZERO           P1_2
#define GPIO_EN_DRV              P0_7   /* Inverter enable pin in general purpose output mode */
#define GPIO_EN_DRV_CLK          P2_13  /* Watchdog clock on EN_DRV pin */
#define GPIO_EN_OFFSTATE_DIAG    P4_9   /* Enable bias on HSx pin for off-state diagnostic */
#define Edl7141ParameterBlock_Addr (uint32_t*)0x10010000
#define Edl7151ParameterBlock_Addr (uint32_t*)0x10010000

/* For 6EDL7141 evaluation */
#define FW_VER                    0x02   /* for FW version storage */

#define UART_BAUD_RATE            (115200)
#define XMC_UART_CH_OVERSAMPLING  (16UL)

/* GUI_6EDL7141_INTEGRATION UART pin assignment */
#define UART_PIN_TX_TO_PC         P4_4 /* USIC1_CH0.DOUT0, ALT6 */
#define UART_PIN_RX_FROM_PC       P4_5 /* USIC1_CH0.DX0D */
#define UART_COM_CH               XMC_UART1_CH0

#define LED_TO_BLINK              XMC_GPIO_PORT0, 11

#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
extern uint8_t GUIwrReg_6EDL7151_addr;
extern uint8_t WriteReg_6EDL7151_addr;
extern uint16_t GUIwrReg_6EDL7151_data;
extern uint16_t WriteReg_6EDL7151_data;
extern uint8_t GuiMonitor_6EDL7151_addr;
extern uint16_t GuiMonitor_6EDL7151_value;
#else
extern uint8_t GUIwrReg_6EDL7141_addr;
extern uint8_t WriteReg_6EDL7141_addr;
extern uint16_t GUIwrReg_6EDL7141_data;
extern uint16_t WriteReg_6EDL7141_data;
extern uint8_t GuiMonitor_6EDL7141_addr;
extern uint16_t GuiMonitor_6EDL7141_value;
#endif

typedef struct
{
    uint8_t en_drv_level:1;
    uint8_t nbrake_level:1;
    uint8_t en_off_state_diagnostic:1;
} EDL_IO_CONTROL_t;
extern EDL_IO_CONTROL_t EdlIo;


/*********************************************************************************************************************
 * API PROTOTYPES
 ********************************************************************************************************************/
/**
 * @brief If macro PROGRAM_DEFAULT_PARAM is 1U, it copy default 6EDL7141 register values to FLASH,
 * else it read the chip version from Flash address, if match, it will upload the 6EDL7141 register value from FLASH to RAM,
 * if chip version read back does not match, it will stay in IDLE state.
 *
 * @param None
 *
 * @retval None
 */
#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
void EDL7151_FLASH_parameter_load(void);
#else
void EDL7141_FLASH_parameter_load(void);
#endif

/**
 * @brief To communicate with PC GUI 6EDL Configurator
 *
 * @param None
 *
 * @retval None
 */
#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
void EDL7151_Update(void);
#else
void EDL7141_Update(void);
#endif

/**
 * @brief Initialize SPI for 6EDL communication, write configure register value into device, read back all register values
 *
 * @param None
 *
 * @retval None
 */
#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
void EDL7151_Config_init(void);
#else
void EDL7141_Config_init(void);
#endif

/**
 * @brief Default 6EDL7141 register setting for testing purpose if macro PROGRAM_DEFAULT_PARAM set to 1U
 *
 * @param None
 *
 * @retval None
 */
#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
void EDL7151_MOTOR_PARAM_set_default(void);
#else
void EDL7141_MOTOR_PARAM_set_default(void);
#endif

/**
 * @brief To write the 6EDL7141 configuration registers to default values
 *
 * @param None
 *
 * @retval None
 */
#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
void SPI_write_6EDL7151_registers(void);
#else
void SPI_write_6EDL7141_registers(void);
#endif

/**
 * @brief To read all the 6EDL7141 registers when startup
 *
 * @param None
 *
 * @retval None
 */
#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
void SPI_read_6EDL7151_registers(void);
#else
void SPI_read_6EDL7141_registers(void);
#endif

/**
 * @brief To setup for OFF state Diagnostic
 *
 * @param None
 *
 * @retval None
 */
#if(MOTOR0_PMSM_FOC_BOARD == EVAL_6EDL7151_FOC_3SH)
void EDL7151_OFFstate_diagnose(void);
#endif

/**
 * @}
 */

/**
 * @}
 */

#endif /* MCUINIT_6EDL_GATEWAY_H_ */
