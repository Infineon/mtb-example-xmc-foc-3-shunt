/**
 * @file pmsm_foc_adc.c
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

/*********************************************************************************************************************
 * HEADER FILES
 ********************************************************************************************************************/
#include "../Configuration/pmsm_foc_mcuhw_params.h"
#include "../MCUInit/pmsm_foc_adc.h"

/*********************************************************************************************************************
 * GLOBAL DATA
 ********************************************************************************************************************/
ADC_t ADC;

/*********************************************************************************************************************
 * DATA STRUCTURES
 ********************************************************************************************************************/
/**
 *  Data Structure initialization - VADC global Configuration.
 */
XMC_VADC_GLOBAL_CONFIG_t VADC_GlobalConfig =
    {
        .module_disable = 0U,
        .disable_sleep_mode_control = 1U,
        .clock_config =
        {
            .analog_clock_divider = 0U, /*Divider Factor for the Analog Internal Clock*/
            .arbiter_clock_divider = 0U,
            .msb_conversion_clock = 0U,
        },
        .data_reduction_control = 0U, /* Data Reduction disabled*/
        .wait_for_read_mode = 0U, /* GLOBRES Register will not be overwrite until the previous value is read*/
        .event_gen_enable = 0U, /* Result Event from GLOBRES is disabled*/

        .boundary0 = 0U, /* Lower boundary value for Normal comparison mode*/
        .boundary1 = 0U  /* Upper boundary value for Normal comparison mode*/
    };

/**
 *  Data Structure initialization - VADC group 0 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_Group0_Init =
    { .emux_config =
        { .stce_usage = 0U, /*Use STCE when the setting changes*/
        .emux_mode = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
        .emux_coding = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
        .starting_external_channel = (uint32_t) 0U, /* Channel starts at 0 for EMUX*/
        .connected_channel = (uint32_t) 0U /* Channel connected to EMUX*/
        },
      .class0 =
        { .sample_time_std_conv = 0U, /* Using Short Sample Time */
        .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/
        .sampling_phase_emux_channel = (uint32_t) 1U, /*The Sample time is (2*tadci)*/
        .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT /* 12bit conversion Selected*/
        }, /* !<ICLASS-0 */
      .class1 =
        { .sample_time_std_conv = 0U, /* Using Short Sample Time */
        .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/
        .sampling_phase_emux_channel = (uint32_t) 1U, /*The Sample time is (2*tadci)*/
        .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT /* 12bit conversion Selected*/
        }, /* !< ICLASS-1 */
      .boundary0 = 0U, /* Lower boundary value for Normal comparison mode*/
      .boundary1 = 0U, /* Upper boundary value for Normal comparison mode*/
      .arbitration_round_length = 0U, /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
      .arbiter_mode = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS, /*Determines when the arbiter should run.*/
    };

/**
 *  Data Structure initialization - VADC group 1 Configuration.
 */
XMC_VADC_GROUP_CONFIG_t VADC_Group1_Init =
    { .emux_config =
        { .stce_usage = 0U, /*Use STCE when the setting changes*/
        .emux_mode = XMC_VADC_GROUP_EMUXMODE_SWCTRL, /* Mode for Emux conversion*/
        .emux_coding = XMC_VADC_GROUP_EMUXCODE_BINARY, /*Channel progression - binary format*/
        .starting_external_channel = (uint32_t) 0U, /* Channel starts at 0 for EMUX*/
        .connected_channel = (uint32_t) 0U /* Channel connected to EMUX*/
        },
      .class0 =
        { .sample_time_std_conv = 0U, /* Using Short Sample Time */
        .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/
        .sampling_phase_emux_channel = (uint32_t) 1U, /*The Sample time is (2*tadci)*/
        .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT /* 12bit conversion Selected*/
        }, /* !<ICLASS-0 */
      .class1 =
        { .sample_time_std_conv = 0U, /* Using Short Sample Time */
        .conversion_mode_standard = XMC_VADC_CONVMODE_12BIT, /* 12bit conversion Selected*/
        .sampling_phase_emux_channel = (uint32_t) 1U, /*The Sample time is (2*tadci)*/
        .conversion_mode_emux = XMC_VADC_CONVMODE_12BIT /* 12bit conversion Selected*/
        }, /* !< ICLASS-1 */
      .boundary0 = 0U, /* Lower boundary value for Normal comparison mode*/
      .boundary1 = 0U, /* Upper boundary value for Normal comparison mode*/
      .arbitration_round_length = (uint32_t) 0U, /* 4 arbitration slots per round selected (tarb = 4*tadcd) */
      .arbiter_mode = (uint32_t) XMC_VADC_GROUP_ARBMODE_ALWAYS, /*Determines when the arbiter should run.*/
    };

/**
 *  Data Structure initialization - VADC group scan request source.
 */

XMC_VADC_BACKGROUND_CONFIG_t VADC_Group_ScanConfig =
    { .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_WFS,
      .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_0,
      .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P, /*If trigger needed the signal input*/
      .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_NONE, /*Trigger edge needed if trigger enabled*/
      .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_P, /*If gating needed the signal input*/
      .timer_mode       = (uint32_t) 0, /* Disabled equidistant sampling*/
      .external_trigger = (uint32_t) 0,
      .load_mode        = (uint32_t) XMC_VADC_SCAN_LOAD_COMBINE, /*Response from SCAN when a Load event occours.*/
    };

XMC_VADC_BACKGROUND_CONFIG_t VADC_Group0_ScanConfig =
    { .conv_start_mode  = (uint8_t) XMC_VADC_STARTMODE_WFS,
      .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_0,
      .trigger_signal   = (uint8_t) XMC_VADC_REQ_TR_P, /*If trigger needed the signal input*/
      .trigger_edge     = (uint8_t) XMC_VADC_TRIGGER_EDGE_NONE, /*Trigger edge needed if trigger enabled*/
      .gate_signal      = (uint32_t) XMC_VADC_REQ_GT_P, /*If gating needed the signal input*/
      .timer_mode       = (uint32_t) 0, /* Disabled equidistant sampling*/
      .external_trigger = (uint32_t) 0,
      .load_mode        = (uint32_t) XMC_VADC_SCAN_LOAD_COMBINE, /*The old set of channels is discarded in favor of the new set awaiting conversion.*/
    };

/* Potentiometer ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Pot_Init =
    { .alias_channel = -1,
      .result_reg_number = VADC_POT_RESULT_REG,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = false
    };

/* DC voltage ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_VDC_Init =
    { .alias_channel = -1,
      .result_reg_number = VADC_VDC_RESULT_REG,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = false,
    };

/* Temparature sensor ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_TEMP_Init =
    { .alias_channel = -1,
      .result_reg_number = VADC_TEMP_RESULT_REG,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = false,
    };

/* Kit ID sensor data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_KIT_SELECT_Init =
    { .alias_channel = -1,
      .result_reg_number = KIT_SELEC_RESULT_REG,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = false,
    };

#if (USER_CURRENT_SENSING ==  THREE_SHUNT_SYNC_CONV)
/**
 *  Data Structure initialization - VADC group queue request source.
 */
XMC_VADC_QUEUE_CONFIG_t VADC_Group_QueueConfig =
    { .conv_start_mode = (uint8_t) XMC_VADC_STARTMODE_CIR, /* Conversion start mode WFS/CIR/CNR*/
      .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_3, /*The queue request source priority */
      .trigger_signal = (uint8_t) XMC_VADC_REQ_TR_P, /* gate input as a trigger */
      .trigger_edge = (uint8_t) XMC_VADC_TRIGGER_EDGE_RISING, /*Trigger edge needed if trigger enabled*/
      .gate_signal = (uint32_t) XMC_VADC_REQ_GT_F, /*If gating needed the signal input - CCU80.ST3*/
      .timer_mode = (uint32_t) 0, /* Disabled equidistant sampling*/
      .external_trigger = (uint32_t) 1 /*External trigger Enabled/Disabled*/
    };

/**
 *  Data Structure initialization - VADC group queue entries.
 */
XMC_VADC_QUEUE_ENTRY_t VADC_Group1_QueueEntry_AliasCH0 =
    { .channel_num = VADC_I1_CHANNEL,
      .external_trigger = true,
      .generate_interrupt = false,
      .refill_needed = true
    };

XMC_VADC_QUEUE_ENTRY_t VADC_Group1_QueueEntry_AliasCH1 =
    { .channel_num = VADC_I3_CHANNEL,
      .external_trigger = false,
      .generate_interrupt = false,
      .refill_needed = true
    };

#if(USER_OVERCURRENT_PROTECTION == ENABLED)
XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_IDC =
{
    .channel_num = VADC_IDC_CHANNEL,
    .external_trigger = false,
    .generate_interrupt = false,
    .refill_needed = true
};
#endif

XMC_VADC_QUEUE_ENTRY_t VADC_Group_QueueEntry_VDC =
    { .channel_num = VADC_VDC_CHANNEL,
      .external_trigger = false,
      .generate_interrupt = false,
      .refill_needed = true
    };

XMC_VADC_QUEUE_ENTRY_t VADC_Group_QueueEntry_POT =
    { .channel_num = VADC_POT_CHANNEL,
      .external_trigger = false,
      .generate_interrupt = false,
      .refill_needed = true
    };

XMC_VADC_QUEUE_ENTRY_t VADC_Group_QueueEntry_TEMP =
    { .channel_num = VADC_TEMP_CHANNEL,
      .external_trigger = false,
      .generate_interrupt = false,
      .refill_needed = true
    };

/**
 *  Data Structure initialization - VADC group channels.
 */
XMC_VADC_CHANNEL_CONFIG_t VADC_G1_CH0_Init =
    { .alias_channel = (int8_t) VADC_G1_CHANNEL_ALIAS0,
      .result_reg_number = 0,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = true };

XMC_VADC_CHANNEL_CONFIG_t VADC_G1_CH1_Init =
    { .alias_channel = (int8_t) VADC_G1_CHANNEL_ALIAS1,
      .result_reg_number = 1,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = true
    };

XMC_VADC_CHANNEL_CONFIG_t VADC_G0_CH0_Init =
    { .alias_channel = (int8_t) VADC_G0_CHANNEL_ALIAS0,
      .result_reg_number = 0,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = false
    };

XMC_VADC_CHANNEL_CONFIG_t VADC_G0_CH1_Init =
    { .alias_channel = (int8_t) VADC_G0_CHANNEL_ALIAS1,
      .result_reg_number = 1,
      .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
      .channel_priority = 0,
      .sync_conversion = false
    };

#if(USER_OVERCURRENT_PROTECTION == ENABLED)
/* DC current ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_IDC_Init =
{
    .alias_channel = -1,
    .result_reg_number = VADC_IDC_RESULT_REG,
    .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
    .channel_priority = 0,
    .sync_conversion = false
};
#endif

/*********************************************************************************************************************
 * API IMPLEMENTATION
 ********************************************************************************************************************/
/**
 * @brief API to initializes the VADC module with the associated configuration structure for 3-shunt phase current sensing
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VADC_PhCurrentInit(void)
{
  /* Configuration of VADC_G1 - Q source */
  /* External trigger 1,  refill 1 */
  XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_Group_QueueConfig);

  XMC_VADC_GROUP_QueueFlushEntries(VADC_G1);

  /* Configure the gating mode for queue*/
  XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);

  /* Request the LLD to insert the channel */
  XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_Group1_QueueEntry_AliasCH0);

  XMC_VADC_GROUP_QueueInsertChannel(VADC_G1, VADC_Group1_QueueEntry_AliasCH1);

  /* Master group - G1 for Synchronous ADC */
  /* I1, Result Register RES0 */
  XMC_VADC_GROUP_ChannelInit(VADC_I1_GROUP, VADC_I1_CHANNEL, &VADC_G1_CH0_Init);

  /* I3, Result Register RES1 */
  XMC_VADC_GROUP_ChannelInit(VADC_I3_GROUP, VADC_I3_CHANNEL, &VADC_G1_CH1_Init);

  /* I2, Result Register RES0 */
  XMC_VADC_GROUP_ChannelInit(VADC_I2_GROUP, VADC_I2_CHANNEL, &VADC_G0_CH0_Init);

  /* Idc, Result Register RES1 */
  XMC_VADC_GROUP_ChannelInit(VADC_I4_GROUP, VADC_I4_CHANNEL, &VADC_G0_CH1_Init);

#if(INTERNAL_OP_GAIN == ENABLED)
  /* Channel Gain factor setting for Iu, Iv, Iw and DC link current */
#if(OP_GAIN_FACTOR == 1U)
  gain_factor = SHS_GAIN_FACTOR_1;
#elif(OP_GAIN_FACTOR == 3U)
  gain_factor = SHS_GAIN_FACTOR_3;
#elif(OP_GAIN_FACTOR == 6U)
  gain_factor = SHS_GAIN_FACTOR_6;
#elif(OP_GAIN_FACTOR == 12U)
  gain_factor = SHS_GAIN_FACTOR_12;
#endif

  if(VADC_IU_G0_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_0, VADC_IU_G0_CHANNEL);
  }
  if(VADC_IU_G1_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_1, VADC_IU_G1_CHANNEL);
  }
  if(VADC_IV_G0_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_0, VADC_IV_G0_CHANNEL);
  }
  if(VADC_IV_G1_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_1, VADC_IV_G1_CHANNEL);
  }
  if(VADC_IW_G0_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_0, VADC_IW_G0_CHANNEL);
  }
  if(VADC_IW_G1_CHANNEL != 0x0F)
  {
    XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,XMC_VADC_GROUP_INDEX_1, VADC_IW_G1_CHANNEL);
  }
#endif

  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_OFF);
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_OFF);

  /* G0: synchronization slave */
  XMC_VADC_GROUP_SetSyncSlave(VADC_G0, 1U, 0U);

  /* Ready input R1 is considered */
  XMC_VADC_GROUP_CheckSlaveReadiness(VADC_G0, 0U);

  /* G1: synchronization master */
  XMC_VADC_GROUP_SetSyncMaster(VADC_G1);

  /* ANONS = 11B: Normal Operation */
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

#if(USER_OVERCURRENT_PROTECTION == ENABLED)
  XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_CH_IDC_Init);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, SHS_GAIN_FACTOR_1,VADC_IDC_GROUP_NO, VADC_IDC_CHANNEL);
  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_IDC_GROUP,VADC_IDC_CHANNEL);
#endif
}

#elif (USER_CURRENT_SENSING ==  THREE_SHUNT_ASYNC_CONV)
/**
 *  Data Structure initialization - VADC group queue request source.
 */
XMC_VADC_QUEUE_CONFIG_t VADC_Group_QueueConfig =
{
  .conv_start_mode = (uint8_t) XMC_VADC_STARTMODE_WFS, /* Conversion start mode WFS/CIR/CNR*/
  .req_src_priority = (uint8_t) XMC_VADC_GROUP_RS_PRIORITY_1, /*The queue request source priority */
  .trigger_signal = (uint8_t) XMC_VADC_REQ_TR_P, /*If trigger needed the signal input*/
  .trigger_edge = (uint8_t) XMC_VADC_TRIGGER_EDGE_RISING, /*Trigger edge needed if trigger enabled*/
  .gate_signal = (uint32_t) XMC_VADC_REQ_GT_F, /*If gating needed the signal input*/
  .timer_mode = (uint32_t) 0, /* Disabled equidistant sampling*/
  .external_trigger = (uint32_t) 1 /*External trigger Enabled/Disabled*/
};

#if(USER_OVERCURRENT_PROTECTION == ENABLED)
XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_IDC =
{
  .channel_num = VADC_IDC_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};
#endif

XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_Iu =
{
  .channel_num = VADC_IU_CHANNEL,
  .external_trigger = true,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_Iv =
{
  .channel_num = VADC_IV_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_Iw =
{
  .channel_num = VADC_IW_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_VDC =
{
  .channel_num = VADC_VDC_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

XMC_VADC_QUEUE_ENTRY_t VADC_QueueEntry_POT =
{
  .channel_num = VADC_POT_CHANNEL,
  .external_trigger = false,
  .generate_interrupt = false,
  .refill_needed = true
};

/**
 *  Data Structure initialization - motor Phases VADC channels.
 */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Iu_Init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IU_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Iv_Init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IV_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

XMC_VADC_CHANNEL_CONFIG_t VADC_CH_Iw_Init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IW_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};

#if(USER_OVERCURRENT_PROTECTION == ENABLED)
/* DC current ADC channel data configuration */
XMC_VADC_CHANNEL_CONFIG_t VADC_CH_IDC_Init =
{
  .alias_channel = -1,
  .result_reg_number = VADC_IDC_RESULT_REG,
  .alternate_reference = XMC_VADC_CHANNEL_REF_INTREF,
  .channel_priority = 0,
  .sync_conversion = false
};
#endif

void PMSM_FOC_VADC_PhCurrentInit(void)
{
  /* Configuration of VADC_G1 - Q source */
  /* External trigger 1,  refill 1 */
  XMC_VADC_GROUP_QueueInit(VADC_G0, &VADC_Group_QueueConfig);
  XMC_VADC_GROUP_QueueInit(VADC_G1, &VADC_Group_QueueConfig);
  XMC_VADC_GROUP_QueueFlushEntries(VADC_G0);
  XMC_VADC_GROUP_QueueFlushEntries(VADC_G1);

  /* Configure the gating mode for queue*/
  XMC_VADC_GROUP_QueueSetGatingMode(VADC_G0, XMC_VADC_GATEMODE_IGNORE);
  XMC_VADC_GROUP_QueueSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);

  /* Channel initialization - Result Register / Gain factor setting  */
  XMC_VADC_GROUP_ChannelInit(VADC_IU_GROUP, VADC_IU_CHANNEL, &VADC_CH_Iu_Init);
  XMC_VADC_GROUP_ChannelInit(VADC_IV_GROUP, VADC_IV_CHANNEL, &VADC_CH_Iv_Init);
  XMC_VADC_GROUP_ChannelInit(VADC_IW_GROUP, VADC_IW_CHANNEL, &VADC_CH_Iw_Init);

  /* Request the LLD to insert the channel */
  XMC_VADC_GROUP_QueueInsertChannel(VADC_IU_GROUP, VADC_QueueEntry_Iu);
  XMC_VADC_GROUP_QueueInsertChannel(VADC_IV_GROUP, VADC_QueueEntry_Iv);
  XMC_VADC_GROUP_QueueInsertChannel(VADC_IW_GROUP, VADC_QueueEntry_Iw);

#if(INTERNAL_OP_GAIN == ENABLED)
  /* Channel Gain factor setting for Iu, Iv, Iw and DC link current */
#if(OP_GAIN_FACTOR == 1U)
  gain_factor = SHS_GAIN_FACTOR_1;
#elif(OP_GAIN_FACTOR == 3U)
  gain_factor = SHS_GAIN_FACTOR_3;
#elif(OP_GAIN_FACTOR == 6U)
  gain_factor = SHS_GAIN_FACTOR_6;
#elif(OP_GAIN_FACTOR == 12U)
  gain_factor = SHS_GAIN_FACTOR_12;
#endif
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,VADC_IU_GROUP_NO, VADC_IU_CHANNEL);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,VADC_IV_GROUP_NO, VADC_IV_CHANNEL);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,VADC_IW_GROUP_NO, VADC_IW_CHANNEL);
#if(USER_OVERCURRENT_PROTECTION == ENABLED)
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0, gain_factor,VADC_IDC_GROUP_NO, VADC_IDC_CHANNEL);
#endif
#endif

#if(USER_OVERCURRENT_PROTECTION == ENABLED)
  /* Idc, Result Register RES1 */
  XMC_VADC_GROUP_ChannelInit(VADC_IDC_GROUP, VADC_IDC_CHANNEL, &VADC_CH_IDC_Init);
  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_IDC_GROUP,VADC_IDC_CHANNEL);
#endif
}
#endif

/**
 * @brief Initializes the VADC channel for scan configuration
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VADC_ModuleInit(void)
{
#if(INTERNAL_OP_GAIN == ENABLED)
  uint32_t gain_factor;
#endif

  XMC_VADC_GLOBAL_Init(VADC, &VADC_GlobalConfig);

  /* Short sample time(SST) set to 1, sample time = SST * tadc (Ex: fADC = 48 MHz, i.e. tADC = 20.84 ns, sample time = 20.84 ns */
  XMC_VADC_GLOBAL_SHS_SetShortSampleTime(SHS0,XMC_VADC_GROUP_INDEX_0,1);
  XMC_VADC_GLOBAL_SHS_SetShortSampleTime(SHS0,XMC_VADC_GROUP_INDEX_1,1);

  XMC_VADC_GROUP_Init(VADC_G0, &VADC_Group0_Init);
  XMC_VADC_GROUP_Init(VADC_G1, &VADC_Group1_Init);

  /* Configuration of VADC_G1/0,  Turn on ADC modules */
  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
  XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

#if(USER_ADC_CALIBRATION == ENABLED)
#if (UC_SERIES == XMC13)
  /* XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.*/
  PMSM_FOC_VADC_StartupCalib();
#else
  /* Trigger Start-up Calibration - Takes around 40uSec */
  /*  */
  XMC_VADC_GLOBAL_StartupCalibration(VADC);
  /* Enable automatic post calibration */
  XMC_VADC_GLOBAL_EnablePostCalibration(VADC, XMC_VADC_GROUP_INDEX_0);
  XMC_VADC_GLOBAL_EnablePostCalibration(VADC, XMC_VADC_GROUP_INDEX_1);
#endif
#endif

  XMC_VADC_GROUP_ScanInit(VADC_G0, &VADC_Group0_ScanConfig);
  /* Configure the gating mode for scan*/
  XMC_VADC_GROUP_ScanSetGatingMode(VADC_G0, XMC_VADC_GATEMODE_IGNORE);

  XMC_VADC_GROUP_ScanInit(VADC_G1, &VADC_Group_ScanConfig);
  /* Configure the gating mode for scan*/
  XMC_VADC_GROUP_ScanSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);
}

/**
 * @brief API to Initializes the VADC channel for potentiometer voltage sensing
 *
 * @param None
 *
 * @retval Identification number of the evaluation board
 */
void PMSM_FOC_VADC_PotInit(void)
{
    /* Initializes the POT VADC channel for conversion */
    XMC_VADC_GROUP_ChannelInit (VADC_POT_GROUP, VADC_POT_CHANNEL,
                                &VADC_CH_Pot_Init);
    XMC_VADC_GROUP_ScanAddChannelToSequence (VADC_POT_GROUP, VADC_POT_CHANNEL);
}

/**
 * @brief API to initialize VADC channel for DC link voltage sensing
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VADC_VDCInit(void)
{
  /* Initializes the DC Link VADC channel for conversion */
  XMC_VADC_GROUP_ChannelInit(VADC_VDC_GROUP, VADC_VDC_CHANNEL, &VADC_CH_VDC_Init);
  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_VDC_GROUP, VADC_VDC_CHANNEL);
}

/**
 * @brief API to initialize the VADC channel for temperature sensing
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VADC_TEMPInit(void)
{
  /* Initializes the DC Link VADC channel for conversion */
  XMC_VADC_GROUP_ChannelInit(VADC_TEMP_GROUP, VADC_TEMP_CHANNEL, &VADC_CH_TEMP_Init);
  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_VDC_GROUP, VADC_TEMP_CHANNEL);
}

/**
 * @brief API to initialize the VADC channel for KitID sensing
 *
 * @param None
 *
 * @retval None
 */
void PMSM_FOC_VADC_KitIDInit(void)
{
  /* Initializes the DC Link VADC channel for conversion */
  XMC_VADC_GROUP_ChannelInit(KIT_SELECT_GRP, KIT_SELECT_CH_NUM, &VADC_CH_KIT_SELECT_Init);
  XMC_VADC_GROUP_ScanAddChannelToSequence(VADC_G0, KIT_SELECT_CH_NUM);
}

/**
 * @brief read the Kit ID number based on voltage setting at the adc port
 *
 * @param None
 *
 * @retval Kit ID 0 - 31
 */
uint16_t read_kit_id(void)
{
  /* Add kit select channel into queue, no refill and trigger only once */
  uint32_t kit_select_result;
  VADC_G0->ASMR |= (uint32_t) VADC_G_ASMR_LDEV_Msk;
  while (((kit_select_result = KIT_SELECT_GRP->RES[KIT_SELECT_CH_NUM]) & VADC_G_RES_VF_Msk) == 0)
  {
  };
  kit_select_result = XMC_VADC_GROUP_GetResult(KIT_SELECT_GRP,
  KIT_SELECT_CH_NUM);
  kit_select_result = ((uint16_t) kit_select_result + 0x40) >> 7; /* Convert to kit id 0-31. 0~63->0; 64~191->1;... 3968~4095->31 */
  return (uint16_t) kit_select_result;
}

/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Turn on Group 0 converter to calibrates VADC by triggering dummy conversions.
 * This startup calibration is required only for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void PMSM_FOC_VADC_StartupCalib(void)
{
  volatile uint32_t dealy_counter; /* Delay counter for the startup calibration to complete. */
  *SHS0_CALOC0 = REG_RESET;
  *SHS0_CALOC1 = REG_RESET;

#if(VADC_ENABLE_GROUP_QUEUE_0 != 1U)
  /* Enable Group 0 for Calibration */
  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
#endif

  /* Enable the StartUp calibration in the VADC */
  VADC->GLOBCFG |= (((uint32_t) 1 << (uint32_t) VADC_GLOBCFG_SUCAL_Pos) & (uint32_t) VADC_GLOBCFG_SUCAL_Msk)
      | (((uint32_t) 1 << (uint32_t) VADC_GLOBCFG_DPCAL0_Pos) & (uint32_t) VADC_GLOBCFG_DPCAL0_Msk);

  /* Wait for 1920cycles or 60us for the startup calibration to complete. */
  dealy_counter = VADC_CALIBRATION_CYCLE_DELAY;

  while (dealy_counter > 0U)
  {
    dealy_counter--;
    /* Clear offset calibration values */
    CLEAR_OFFSET_CALIB_VALUES;
  }
  PMSM_FOC_VADC_GainCalib();

  /* Switch off Group 0 converter if it is not used for any conversions */
#if(VADC_ENABLE_GROUP_QUEUE_0 != 1U)
  XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_OFF);
#endif
}

/* This API is used for VADC gain calibration. */
/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Calibrates VADC by triggering dummy conversion for XMC13 AA STEP micro-controllers as per ADC_AI.004 errata.
 */
void PMSM_FOC_VADC_GainCalib(void)
{
  volatile uint16_t dummy_conv_counter = VADC_DUMMY_CONVERSION_COUNTER; /* Used for keeping track for dummy conversions */
  volatile uint32_t adc_result_aux; /* Used for reading ADC values after dummy conversions */

  /* ADC_AI.004 errata*/
  *SHS0_CALCTR = 0X3F100400U;

  /* add a channel in group-0 for dummy conversion*/
  VADC->BRSSEL[0] = VADC_BRSSEL_CHSELG0_Msk;

  /*Clear the DPCAL0, DPCAL1 and SUCAL bits*/
  VADC->GLOBCFG &= ~((uint32_t) VADC_GLOBCFG_DPCAL0_Msk | (uint32_t) VADC_GLOBCFG_DPCAL1_Msk | (uint32_t) VADC_GLOBCFG_SUCAL_Msk);

  /* Clear offset calibration values*/
  CLEAR_OFFSET_CALIB_VALUES;

  VADC->BRSMR = ((uint32_t) 1 << (uint32_t) VADC_BRSMR_ENGT_Pos);
#if UC_SERIES != XMC11
  VADC_G0->ARBPR = (VADC_G_ARBPR_ASEN2_Msk);
#endif
  /*Trigger dummy conversion for 9* 2000 times*/

  while (dummy_conv_counter > 0U)
  {
    /*load event */
    VADC->BRSMR |= (uint32_t) VADC_BRSMR_LDEV_Msk;
#if (UC_SERIES != XMC11)
    /*Wait until a new result is available*/
    while (VADC_G0->VFR == 0U)
    {

    }

    /*dummy read of result */
    adc_result_aux = VADC_G0->RES[0];
#else
    /*Wait untill a new result is available*/
    while ((VADC->GLOBRES & VADC_GLOBRES_VF_Msk) == 0)
    {

    }

    /*dummy read of result */
    adc_result_aux = VADC->GLOBRES;
#endif

    /* Clear offset calibration values*/
    CLEAR_OFFSET_CALIB_VALUES
    ;
    dummy_conv_counter--;
  }

  /* To avoid a warning*/
  adc_result_aux &= 0U;

  /* Wait until last gain calibration step is finished */
  while ((SHS0->SHSCFG & (uint32_t) SHS_SHSCFG_STATE_Msk) != 0U)
  {
    /* Clear offset calibration values*/
    CLEAR_OFFSET_CALIB_VALUES
    ;
  }
  /* Re enable SUCAL DPCAL */
  VADC->GLOBCFG |= ((uint32_t) VADC_GLOBCFG_DPCAL0_Msk | (uint32_t) VADC_GLOBCFG_DPCAL1_Msk);
  VADC->BRSMR = (uint32_t) 0x00;
  VADC->BRSSEL[0] = (uint32_t) 0x00;
#if (UC_SERIES != XMC11)
  VADC_G0->REFCLR = 1U;
  VADC_G0->ARBPR &= ~((uint32_t) VADC_G_ARBPR_ASEN2_Msk);
#endif
}
