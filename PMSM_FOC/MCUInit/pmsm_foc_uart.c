/**
 * @file pmsm_foc_uart.c
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
#include "../MCUInit/pmsm_foc_uart.h"
#include "../ControlModules/pmsm_foc_functions.h"
/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/

extern PMSM_FOC_CTRL_t PMSM_FOC_CTRL; /* motor control information */

/**
  * @brief  Initialize USIC Module
  *
  * @param  None
  * @retval None
  */
#define XMC_SCU_GCU_PASSWD_PROT_DISABLE (0x000000C0UL)  /*
                                                         * Password for disabling protection.
                                                         * Access to protected bits allowed.
                                                         */
#define XMC_SCU_GCU_PASSWD_PROT_ENABLE  (0x000000C3UL)  /* Password for enabling protection. */

#if(UART_ENABLE == USIC0_CH1_P1_2_P1_3)
/**
 * @brief   Initialize USIC Module
 *
 * @param  None
 * @retval None
 */
void pmsm_foc_uart_init (void)
{
  /* Disable clock gating to USIC0: */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_DISABLE;

  /* Stop gating USIC0 */
  SCU_CLK->CGATCLR0 = 0x00000008UL;

  /* Wait if VDDC is too low, for VDDC to stabilise */
  while (SCU_CLK->CLKCR & 0x40000000UL)
  {
    continue;
  }

  /* Enable bit protection */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_ENABLE;

  /* Enable the module kernel clock and the module functionality: */
  USIC0_CH1->KSCFG |= USIC_CH_KSCFG_MODEN_Msk | USIC_CH_KSCFG_BPMODEN_Msk;

  /* fFD = fPB. */
  /* FDR.DM = 02b (Fractional divider mode). */
  USIC0_CH1->FDR &= ~(USIC_CH_FDR_DM_Msk | USIC_CH_FDR_STEP_Msk);
  USIC0_CH1->FDR |= (0x02UL << USIC_CH_FDR_DM_Pos) | (FDR_STEP << USIC_CH_FDR_STEP_Pos);

  /* Configure baud rate generator: */
  /* BAUDRATE = fCTQIN/(BRG.PCTQ x BRG.DCTQ). */
  /* CLKSEL = 0 (fPIN = fFD), CTQSEL = 00b (fCTQIN = fPDIV), PPPEN = 0 (fPPP=fPIN). */
  USIC0_CH1->BRG &= ~(USIC_CH_BRG_PCTQ_Msk | USIC_CH_BRG_DCTQ_Msk | USIC_CH_BRG_PDIV_Msk | USIC_CH_BRG_CLKSEL_Msk |
                    USIC_CH_BRG_PPPEN_Msk);
  USIC0_CH1->BRG |= (BRG_PCTQ << USIC_CH_BRG_PCTQ_Pos) | (BRG_DCTQ << USIC_CH_BRG_DCTQ_Pos) |
                    (BRG_PDIV << USIC_CH_BRG_PDIV_Pos);

  /* Configuration of USIC Shift Control: */
  /* SCTR.FLE = 8 (Frame Length). */
  /* SCTR.WLE = 8 (Word Length). */
  /* SCTR.TRM = 1 (Transmission Mode). */
  /*
   * SCTR.PDL = 1 (This bit defines the output level at the shift data output signal when no data is available
   * for transmission).
   */
  USIC0_CH1->SCTR &= ~(USIC_CH_SCTR_TRM_Msk | USIC_CH_SCTR_FLE_Msk | USIC_CH_SCTR_WLE_Msk);
  USIC0_CH1->SCTR |= USIC_CH_SCTR_PDL_Msk | (0x01UL << USIC_CH_SCTR_TRM_Pos) | (0x07UL << USIC_CH_SCTR_FLE_Pos) |
                     (0x07UL << USIC_CH_SCTR_WLE_Pos);

  /* Configuration of USIC Transmit Control/Status Register: */
  /* TBUF.TDEN = 1 (TBUF Data Enable: A transmission of the data word in TBUF can be started if TDV = 1. */
  /*
   * TBUF.TDSSM = 1 (Data Single Shot Mode: allow word-by-word data transmission which avoid sending the same data
   * several times.
   */
  USIC0_CH1->TCSR &= ~(USIC_CH_TCSR_TDEN_Msk);
  USIC0_CH1->TCSR |= USIC_CH_TCSR_TDSSM_Msk | (0x01UL << USIC_CH_TCSR_TDEN_Pos);

  /* Configuration of Protocol Control Register: */
  /* PCR.SMD = 1 (Sample Mode based on majority). */
  /* PCR.STPB = 0 (1x Stop bit). */
  /* PCR.SP = 5 (Sample Point). */
  /* PCR.PL = 0 (Pulse Length is equal to the bit length). */
  USIC0_CH1->PCR &= ~(USIC_CH_PCR_ASCMode_STPB_Msk | USIC_CH_PCR_ASCMode_SP_Msk | USIC_CH_PCR_ASCMode_PL_Msk);
  USIC0_CH1->PCR |= USIC_CH_PCR_ASCMode_SMD_Msk | (9 << USIC_CH_PCR_ASCMode_SP_Pos);

  /* Configure Transmit Buffer: */
  /* Standard transmit buffer event is enabled. */
  /* Define start entry of Transmit Data FIFO buffer DPTR = 0. */
  USIC0_CH1->TBCTR &= ~(USIC_CH_TBCTR_SIZE_Msk | USIC_CH_TBCTR_DPTR_Msk);

  /* Set Transmit Data Buffer size and set data pointer to position 0. */
  USIC0_CH1->TBCTR |= ((UART_FIFO_SIZE << USIC_CH_TBCTR_SIZE_Pos) | (0x00 << USIC_CH_TBCTR_DPTR_Pos));

  /* Init UART_RX (P1.3 --> DX0A, or P2.11 --> DX0E): */
  XMC_GPIO_SetMode(P1_3, XMC_GPIO_MODE_INPUT_TRISTATE);
  USIC0_CH1->DX0CR = 0x00000000; /* USIC0_CH1.DX0A <-- P1.3. */

  /* Configure Receive Buffer: */
  /* Standard Receive buffer event is enabled. */
  /* Define start entry of Receive Data FIFO buffer DPTR. */
  USIC0_CH1->RBCTR &= ~(USIC_CH_RBCTR_SIZE_Msk | USIC_CH_RBCTR_DPTR_Msk);

  /* Set Receive Data Buffer Size and set data pointer to position max. */
  USIC0_CH1->RBCTR |= ((UART_FIFO_SIZE << USIC_CH_RBCTR_SIZE_Pos) | ((1 << UART_FIFO_SIZE) << USIC_CH_RBCTR_DPTR_Pos));

  /* Init UART_TX (P1.2 --> DOUT): */
  XMC_GPIO_SetMode(P1_2, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT7);

  /* Configuration of Channel Control Register: */
  /* CCR.PM = 00 ( Disable parity generation). */
  /* CCR.MODE = 2 (ASC mode enabled). */
  USIC0_CH1->CCR &= ~(USIC_CH_CCR_PM_Msk | USIC_CH_CCR_MODE_Msk);
  USIC0_CH1->CCR |= 0x02UL << USIC_CH_CCR_MODE_Pos;

}

#elif(UART_ENABLE == USIC0_CH0_P1_4_P1_5)
void pmsm_foc_uart_init (void)
{
  /* Disable clock gating to USIC0: */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_DISABLE;

  /* Stop gating USIC0 */
  SCU_CLK->CGATCLR0 = 0x00000008UL;

  /* Wait if VDDC is too low, for VDDC to stabilise */
  while (SCU_CLK->CLKCR & 0x40000000UL) continue;

  /* Enable bit protection */
  SCU_GENERAL->PASSWD = XMC_SCU_GCU_PASSWD_PROT_ENABLE;

  /* Enable the module kernel clock and the module functionality: */
  USIC0_CH0->KSCFG |= USIC_CH_KSCFG_MODEN_Msk | USIC_CH_KSCFG_BPMODEN_Msk;

  // fFD = fPB.
  // FDR.DM = 02b (Fractional divider mode).
  USIC0_CH0->FDR &= ~(USIC_CH_FDR_DM_Msk | USIC_CH_FDR_STEP_Msk);
  USIC0_CH0->FDR |= (0x02UL << USIC_CH_FDR_DM_Pos) | (FDR_STEP << USIC_CH_FDR_STEP_Pos);

  // Configure baud rate generator:
  // BAUDRATE = fCTQIN/(BRG.PCTQ x BRG.DCTQ).
  // CLKSEL = 0 (fPIN = fFD), CTQSEL = 00b (fCTQIN = fPDIV), PPPEN = 0 (fPPP=fPIN).
  USIC0_CH0->BRG &= ~(USIC_CH_BRG_PCTQ_Msk | USIC_CH_BRG_DCTQ_Msk | USIC_CH_BRG_PDIV_Msk | USIC_CH_BRG_CLKSEL_Msk | USIC_CH_BRG_PPPEN_Msk);
  USIC0_CH0->BRG |= (BRG_PCTQ << USIC_CH_BRG_PCTQ_Pos) | (BRG_DCTQ << USIC_CH_BRG_DCTQ_Pos) | (BRG_PDIV << USIC_CH_BRG_PDIV_Pos);

  // Configuration of USIC Shift Control:
  // SCTR.FLE = 8 (Frame Length).
  // SCTR.WLE = 8 (Word Length).
  // SCTR.TRM = 1 (Transmission Mode).
  // SCTR.PDL = 1 (This bit defines the output level at the shift data output signal when no data is available for transmission).
  USIC0_CH0->SCTR &= ~(USIC_CH_SCTR_TRM_Msk | USIC_CH_SCTR_FLE_Msk | USIC_CH_SCTR_WLE_Msk);
  USIC0_CH0->SCTR |= USIC_CH_SCTR_PDL_Msk | (0x01UL << USIC_CH_SCTR_TRM_Pos) | (0x07UL << USIC_CH_SCTR_FLE_Pos) | (0x07UL << USIC_CH_SCTR_WLE_Pos);

  // Configuration of USIC Transmit Control/Status Register:
  // TBUF.TDEN = 1 (TBUF Data Enable: A transmission of the data word in TBUF can be started if TDV = 1.
  // TBUF.TDSSM = 1 (Data Single Shot Mode: allow word-by-word data transmission which avoid sending the same data several times.
  USIC0_CH0->TCSR &= ~(USIC_CH_TCSR_TDEN_Msk);
  USIC0_CH0->TCSR |= USIC_CH_TCSR_TDSSM_Msk | (0x01UL << USIC_CH_TCSR_TDEN_Pos);

  // Configuration of Protocol Control Register:
  // PCR.SMD = 1 (Sample Mode based on majority).
  // PCR.STPB = 0 (1x Stop bit).
  // PCR.SP = 5 (Sample Point).
  // PCR.PL = 0 (Pulse Length is equal to the bit length).
  USIC0_CH0->PCR &= ~(USIC_CH_PCR_ASCMode_STPB_Msk | USIC_CH_PCR_ASCMode_SP_Msk | USIC_CH_PCR_ASCMode_PL_Msk);
  USIC0_CH0->PCR |= USIC_CH_PCR_ASCMode_SMD_Msk | (9 << USIC_CH_PCR_ASCMode_SP_Pos);

  // Configure Transmit Buffer:
  // Standard transmit buffer event is enabled.
  // Define start entry of Transmit Data FIFO buffer DPTR = 0.
  USIC0_CH0->TBCTR &= ~(USIC_CH_TBCTR_SIZE_Msk | USIC_CH_TBCTR_DPTR_Msk);

  // Set Transmit Data Buffer size and set data pointer to position 0.
  USIC0_CH0->TBCTR |= ((UART_FIFO_SIZE << USIC_CH_TBCTR_SIZE_Pos)|(0x00 << USIC_CH_TBCTR_DPTR_Pos));

  // Init UART_RX (P1.4 --> DX5E, or P1.5 --> DOUT0):
  XMC_GPIO_SetMode(P1_4, XMC_GPIO_MODE_INPUT_TRISTATE);
  USIC0_CH0->DX0CR |= 0x00000016;      // USIC0_CH0.DX0E <-- P1.4.
  USIC0_CH0->DX3CR |= 0x00000015;      // USIC0_CH0.DX3E <-- P1.4.
  USIC0_CH0->DX5CR |= 0x00000014;      // USIC0_CH0.DX5E <-- P1.4.

  // Configure Receive Buffer:
  // Standard Receive buffer event is enabled.
  // Define start entry of Receive Data FIFO buffer DPTR.
  USIC0_CH0->RBCTR &= ~(USIC_CH_RBCTR_SIZE_Msk | USIC_CH_RBCTR_DPTR_Msk);

  // Set Receive Data Buffer Size and set data pointer to position max.
  USIC0_CH0->RBCTR |= ((UART_FIFO_SIZE << USIC_CH_RBCTR_SIZE_Pos)|((1<<UART_FIFO_SIZE) << USIC_CH_RBCTR_DPTR_Pos));

  // Init UART_TX (P1.2 --> DOUT):
  XMC_GPIO_SetMode(P1_5, XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT2);

  // Configuration of Channel Control Register:
  // CCR.PM = 00 ( Disable parity generation).
  // CCR.MODE = 2 (ASC mode enabled).
  USIC0_CH0->CCR &= ~(USIC_CH_CCR_PM_Msk | USIC_CH_CCR_MODE_Msk);
  USIC0_CH0->CCR |= 0x02UL << USIC_CH_CCR_MODE_Pos;

}
#endif

