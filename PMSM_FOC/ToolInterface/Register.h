/**
 * @file Register.h
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
 *@endcond
 ***********************************************************************************************************************/

#ifndef PMSM_FOC_TOOLINTERFACE_REGISTER_H_
#define PMSM_FOC_TOOLINTERFACE_REGISTER_H_

/**
 * @addtogroup PMSM_FOC
 * @{
 */

/**
 * @addtogroup Configuration
 * @{
 */
#include  <stdint.h>
#include <PMSM_FOC/Configuration/pmsm_foc_common.h>

/* Motor Parameter block address in flash memory */
#define ParameterBlock_Addr    (uint32_t *)0x10010100
/* Control loop PI Parameter block address in flash memory */
#define motor_PI_config_addr   PMSM_SL_FOC_PI_CONFIG_ADDR

typedef struct
{
	uint32_t  chip_id;				    /* MCU device ID */
	uint16_t  parameter_version;		/* Parameter version */
	uint16_t  firmware_version;			/* Firmware version */
	uint8_t   kit_id;					/* Hardware kit ID: 0: Unidentified, 1: 1S 6EDL7141 v2 */
} MC_INFO_t;
extern MC_INFO_t MC_INFO;

typedef struct
{
	union
	{
		struct
		{
			uint32_t ParamConfigured:1;	    /* This bit indicates parameters are configured either from flash or configured by GUI */
			uint32_t Edl7141Configured:1;   /* This bit indicates 6EDL7141 configure registers are configured either from flash or by GUI */
			uint32_t OffsetCalibrated:1;	/* This bit indicates current sense offset is calibrated */
			uint32_t ReInitParam:1;		    /* This bit can be set by GUI or other code re-init peripheral configuration */
		};
		uint32_t Status;
	};
    void (*ControlLoop)(void);
	uint32_t GlobalTimer;					/* Free running timer counts up every systick */
} SYSTEM_VAR_t;
extern SYSTEM_VAR_t SystemVar;

/*******************************************************************************
 * MotorVar is the structure for motor related running variables.
 ******************************************************************************/
typedef struct
{
	uint8_t Command;
//	uint8_t SupressStallCheck;
//    uint8_t hall_learning_flag;             /*!< hall pattern detection done*/
//    uint8_t hall_pattern;                   /*!< to store previous captured hall pattern - used in wrong hall event to identify hall failure */
//    uint8_t hall_sector;
//	int16_t TargetValue;
//	int16_t current_ref;
//    int16_t voltage_output;
    int8_t fault_clear;
    uint16_t error_status;
	union
	{
		struct
		{
			uint16_t Trap:1;
			uint16_t OverCurrent:1;
			uint16_t ShortCircuit:1;
			uint16_t TorqueOverLimit:1;
			uint16_t TempOverLimit:1;
			uint16_t Stall:1;
			uint16_t OverVoltage:1;
			uint16_t :1;
			uint16_t :1;
			uint16_t :1;
			uint16_t UnderVoltage:1;
			uint16_t SpiError:1;
			uint16_t Nfault:1;
		};
		uint16_t Value;
	} MaskedFault;
	uint16_t CCU8_PERIOD;                   /* CCU8 period register value */
	uint16_t direct_dc_amplifier_offset;	/*!< external amplifier offset for direct DC link current*/
	uint16_t avg_dc_amplifier_offset;    	/*!< external amplifier offset for average current*/
	int16_t MotorCurrent;                   /* Motor current raw value in Q14 */
	FILTER_TYPE_t t_sensor;    	                /*!< temperature sensor, degree C in Q4*/
} MOTOR_VAR_t;
extern MOTOR_VAR_t MotorVar;

/*******************************************************************************
 * MotorParam is the structure for motor related running parameters. These parameters
 * are derived from user input parameters.
 ******************************************************************************/
#define PARAMETER_SIZE	    (sizeof(MotorParam))
#define PI_CONFIG_SIZE	    (sizeof(PI_CONFIG_t))
typedef struct
{
	uint16_t ParamVersion;
	// HwConfig
	uint8_t InternalGain;		            /* 0: gain=1; 1: gain=3; 2: gain=6; 3: gain=12 */
	uint8_t EnablePin;                      /* 0: Inverter enable pin not in use, 1: Inverter enable pin is in use */
	uint8_t EnablePinPolarity;              /* Active level of inverter enable pin, 0: no enable pin, 1: active high, 2: active low */
	uint8_t WatchdogClock;                  /* 0: Watchdog clock on EN_DRV pin is disabled, 1: Watchdog clock on EN_DRV pin is enabled */
    // SysConfig;
	uint8_t AnalogControl;		            /* 0: Register; 1: Potentiometer */
	uint8_t ControlScheme;                  /* 0: Voltage; 1: Speed; 2: Current; 3: Speed current */
	uint8_t BiDirectional;		            /* 1: BiDirectional control when using potentiometer */
	uint8_t SvmScheme;				        /* 1. STANDARD_SVM_7_SEGMENT          2. STANDARD_SVM_5_SEGMENT */
	uint8_t SwapDirection;		            /* 1: Swap motor position direction */
	uint8_t PwmModulationType;              /* 0: PWM_HIGHSIDE_SYNCREC, 1: PWM_HIGHSIDE, 2: PWM_LOWSIDE */
	uint8_t DcBusCompensation;              /* 0: Disable DC bus compensation; 1: Enable DC bus compensation */
	uint8_t StartupMethod;                  /*!< 0: MOTOR_STARTUP_DIRECT_FOC; 1: MOTOR_STARTUP_VF_OPEN_LOOP */
	uint8_t InitPosDetect;                  /*!< 1. ROTOR_PRE_ALIGNMENT 2. ROTOR_PRE_ALIGN_NONE */
	uint8_t Vadc_Ref_Voltage;               /* VADC reference voltage in 0.1V */
	union
	{
		struct
		{
			uint16_t Trap:1;
			uint16_t OverCurrent:1;
			uint16_t ShortCircuit:1;
			uint16_t TorqueOverLimit:1;
			uint16_t TempOverLimit:1;
			uint16_t Stall:1;
			uint16_t OverVoltage:1;
			uint16_t FreeRunning:1;
			uint16_t FreeRevRunning:1;
			uint16_t StartupFailure:1;
			uint16_t UnderVoltage:1;
			uint16_t SpiError:1;
			uint16_t Nfault:1;
		};
		uint16_t Value;
	} EnableFault;
	// old definition
	uint16_t G_OPAMP_PER_PHASECURRENT;
	uint32_t I_MAX_A;
	uint32_t I_MIN_A;
	uint32_t CCU8_PERIOD_REG_T;
	uint16_t CCU4_PERIOD_REG;
	uint8_t  DRIVERIC_DELAY;
	uint32_t SPEED_REF_HIGH_LIMIT_RPM;
	uint32_t SPEED_REF_LOW_LIMIT_RPM;
	uint32_t SPEED_REF_HIGH_LIMIT_TS;
	uint32_t SPEED_REF_LOW_LIMIT_TS;
	uint32_t CCU8_DEADTIME_RISE_T;
	uint32_t CCU8_DEADTIME_FALL_T;
	uint16_t PMSM_FOC_SYSTICK_COUNT_T;
	uint32_t IDC_OVER_CURRENT_LIMIT;
	uint32_t MOTOR_BRAKE_DUTY_VAL;
	uint32_t BOOTSTRAP_BRAKE_TIME;
	uint32_t STARTUP_VF_TRANSITION_SPEED;
	uint32_t STARTUP_VF_OFFSET;
	uint32_t STARTUP_VF_V_PER_HZ_CONST;
	uint32_t STARTUP_VF_SPEED_RAMP_UP_RATE;
	uint32_t STARTUP_VF_STABILIZATION_COUNT;
	uint32_t VQ_INITIAL_VALUE;
	uint32_t ROTOR_PRE_ALIGNMENT_VREF;
	uint32_t ROTOR_PRE_ALIGNMENT_COUNT;
	uint32_t ROTOR_PRE_ALIGNMENT_RAMP_RATE;
	uint32_t BRAKING_VDC_MAX_LIMIT;
	uint32_t ELECTRICAL_SPEED_LOW_LIMIT_TS;
	uint32_t ELECTRICAL_SPEED_HIGH_LIMIT_TS;
	uint32_t ELECTRICAL_SPEED_FREQ_HZ;
	uint32_t BASE_VOLTAGE;
	uint32_t VADC_DCLINK_T;
	uint32_t VQ_REF_HIGH_LIMIT;
	uint32_t VQ_REF_LOW_LIMIT;
	uint32_t RAMP_UP_SPEED;
	uint32_t RAMP_DOWN_SPEED;
	uint32_t USER_CURRENTCTRL_CUTOFF_FREQ_HZ;
	uint32_t BEMF_MAX_V;
	uint32_t N_I_UVW_A;
	uint32_t N_VREF_SVM_V;
	uint32_t N_I_ALPHABETA_A;
	uint32_t N_VREF_ALPHABETA_V;
	uint32_t N_I_DQ_A;
	uint32_t N_V_DQ_V;
  uint32_t BEMF_MAG_SCALING;
  uint32_t CORDIC_MPS_PER_K;
  uint32_t DEFAULT_SCALE_OF_L;
  uint32_t DEFAULT_LS_SCALEDUP;
  uint32_t CONVERT_SPEED_TO_RPM;
  uint32_t SVM_LUTTABLE_SCALE ;
  uint32_t PWM_PERIOD_TS_US;
  uint32_t SVM_LAMDA;
  uint32_t KS_SCALE_SVM;

  uint32_t PMSM_FOC_CCU8_SYNC_START;
  uint32_t OVERVOLTAGE_THRESHOLD;
  uint32_t UNDERVOLTAGE_THRESHOLD;
  uint32_t DQ_DECOUPLING;
} MOTOR_PAR_t;
extern MOTOR_PAR_t MotorParam;

typedef struct
{
	uint32_t CONFIG_VALID_ID; /*!< Valid Configuration ID */
	uint32_t PI_SPEED_KP; /*!< USER_PI_SPEED_KP */
	uint32_t PI_SPEED_KI; /*!< USER_PI_SPEED_KI */
	uint32_t PI_SPEED_SCALE; /*!< USER_PI_SPEED_SCALE_KPKI */
	int32_t PI_SPEED_IK_LIMIT_MIN; /*!< USER_PI_SPEED_IK_LIMIT_MIN */
	int32_t PI_SPEED_IK_LIMIT_MAX; /*!< USER_PI_SPEED_IK_LIMIT_MAX */
	int32_t PI_SPEED_UK_LIMIT_MIN; /*!< USER_PI_SPEED_UK_LIMIT_MIN */
	int32_t PI_SPEED_UK_LIMIT_MAX; /*!< USER_PI_SPEED_UK_LIMIT_MAX */

	uint32_t PI_TORQUE_KP; /*!< USER_PI_TORQUE_KP */
	uint32_t PI_TORQUE_KI; /*!< USER_PI_TORQUE_KI */
	uint32_t PI_TORQUE_SCALE; /*!< USER_PI_TORQUE_SCALE_KPKI */
	int32_t PI_TORQUE_IK_LIMIT_MIN; /*!< USER_PI_TORQUE_IK_LIMIT_MIN */
	int32_t PI_TORQUE_IK_LIMIT_MAX; /*!< USER_PI_TORQUE_IK_LIMIT_MAX */
	int32_t PI_TORQUE_UK_LIMIT_MIN; /*!< USER_PI_TORQUE_UK_LIMIT_MIN */
	int32_t PI_TORQUE_UK_LIMIT_MAX; /*!< USER_PI_TORQUE_UK_LIMIT_MAX */

	uint32_t PI_FLUX_KP; /*!< USER_PI_FLUX_KP */
	uint32_t PI_FLUX_KI; /*!< USER_PI_FLUX_KI */
	uint32_t PI_FLUX_SCALE; /*!< USER_PI_FLUX_SCALE_KPKI */
	int32_t PI_FLUX_IK_LIMIT_MIN; /*!< USER_PI_FLUX_IK_LIMIT_MIN */
	int32_t PI_FLUX_IK_LIMIT_MAX; /*!< USER_PI_FLUX_IK_LIMIT_MAX */
	int32_t PI_FLUX_UK_LIMIT_MIN; /*!< USER_PI_FLUX_UK_LIMIT_MIN */
	int32_t PI_FLUX_UK_LIMIT_MAX; /*!< USER_PI_FLUX_UK_LIMIT_MAX */

	uint32_t PI_PLL_KP; /*!< USER_PI_PLL_KP */
	uint32_t PI_PLL_KI; /*!< USER_PI_PLL_KI */
	uint32_t PI_PLL_SCALE; /*!< USER_PI_PLL_SCALE_KPKI */
	int32_t PI_PLL_IK_LIMIT_MIN; /*!< USER_PI_PLL_IK_LIMIT_MIN */
	int32_t PI_PLL_IK_LIMIT_MAX; /*!< USER_PI_PLL_IK_LIMIT_MAX */
	int32_t PI_PLL_UK_LIMIT_MIN; /*!< USER_PI_PLL_UK_LIMIT_MIN */
	int32_t PI_PLL_UK_LIMIT_MAX; /*!< USER_PI_PLL_UK_LIMIT_MAX */
} PI_CONFIG_t;

/*********************************************************************************************************************
 * API Prototypes
 ********************************************************************************************************************/
/**
 * @brief Updates the chip_id, parameter_version, firmware_version and kit_id of the board
 *
 * @param None
 *
 * @retval None
 */
void MC_Info_init(void);

/**
 * @brief MC_INFO structure contains expected parameter version for the firmware. Head of parameter block should contain same value, otherwise parameter block won't be loaded
 *
 * @param None
 *
 * @retval None
 */
void FLASH_Parameter_load(void);

/**
 * @brief Update MotorParam structure with calculated parameters based on user configuration
 *
 * @param None
 *
 * @retval None
 */
void MOTOR_PARAM_set_default(void);

/**
 * @brief Updates the default Kp,Ki,Scale value for the four control loop
 *
 * @param None
 *
 * @retval None
 */
void PI_set_default(void);

#define REGISTER_MAP_SIZE 74
extern void* const RegisterAddrMap[];
typedef struct
{
    uint8_t DataType:3;
    uint8_t BitEn:1;
    uint8_t BitSel:5;
} REGISTER_TYPE_MAP_t;
extern REGISTER_TYPE_MAP_t const RegisterTypeMap[];

/**
 * @}
 */

/**
 * @}
 */

#endif /* PMSM_FOC_TOOLINTERFACE_REGISTER_H_ */
