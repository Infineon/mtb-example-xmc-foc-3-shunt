# XMC&trade; MCU: 6EDL7141 and MOTIX&trade; IMD700A sensorless FOC 3-shunt

This code example works with both the MOTIX™ 6EDL7141 3-shunt FOC sensorless and MOTIX™ IMD700A 3-shunt FOC sensorless projects in the Battery Powered Application (BPA) Motor Control GUI to illustrate sensorless FOC motor control on [EVAL_6EDL7141_FOC_3SH](https://www.infineon.com/dgdl/Infineon-Evaluation_Board_EVAL_6EDL7141_FOC_3SH-UserManual-v01_00-EN.pdf?fileId=8ac78c8c82ce566401830db3784605f9) and [EVAL_IMD700A_FOC_3SH](https://www.infineon.com/dgdl/Infineon-Evaluation_board_EVAL_IMD700A_FOC_3SH-ApplicationNotes-v01_00-EN.pdf?fileId=8ac78c8c7fb5929e017fdf3691800b9d) reference design/evaluation board for FOC motor drive based on the MOTIX™ 6EDL7141 smart 3-phase driver and MOTIX™ IMD701A motor controller IC.

## Requirements

- [ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0)
- [SEGGER J-Link](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack) software
- [BPA Motor Control](https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ifxmotorsolutions) software
- Programming language: C
- Associated parts: All [MOTIX&trade; 6EDL7141](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/6edl7141/) parts; all [MOTIX&trade; IMD701A](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd701a-q064x128-aa/)

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`

## Supported boards (make variable 'TARGET')

- [EVAL_IMD700A_FOC_3SH](https://www.infineon.com/dgdl/Infineon-Evaluation_board_EVAL_IMD700A_FOC_3SH-ApplicationNotes-v01_00-EN.pdf?fileId=8ac78c8c7fb5929e017fdf3691800b9d) reference design/evaluation board - sensorless FOC motor drive based on the MOTIX™ IMD701A motor controller
- [EVAL_6EDL7141_FOC_3SH](https://www.infineon.com/cms/en/product/evaluation-boards/eval_6edl7141_foc_3sh/#!documents) reference design/evaluation board - sensorless FOC motor drive based on the MOTIX™ 6EDL7141 smart 3-phase driver

## Hardware setup

This example uses the board's default configuration. See the board user guide to ensure that the board is configured correctly.

## Software setup

This example is compatible with the [BPA Motor Control](https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ifxmotorsolutions) software. Before you can install this tool, you need to install the [Infineon Developer Center software launcher](https://www.infineon.com/cms/en/design-support/tools/utilities/infineon-developer-center-idc-launcher/) first. To interact with BPA Motor Control GUI, see the *[*Design and implementation](#design-and-implementation) section.


## Using the code example

Create the project and open it using one of the following:

<details open><summary><b>In Eclipse IDE for ModusToolbox&trade; software</b></summary>

1. Click the **New Application** link in the **Quick Panel** (or, use **File** > **New** > **ModusToolbox&trade; Application**). This launches the [Project Creator](https://www.infineon.com/ModusToolboxProjectCreator) tool.

2. Pick EVAL 6EDL7141 FOC sensorless 3 Shunt Board or EVAL IMD700A FOC sensorless 3 Shunt Board from the list shown in the **Project Creator - Choose Board Support Package (BSP)** dialog, depending on the evaluation board you are using.

   When you select a supported kit, the example is reconfigured automatically to work with the kit. To work with a different supported kit later, use the [Library Manager](https://www.infineon.com/ModusToolboxLibraryManager) to choose the BSP for the supported kit. You can use the Library Manager to select or update the BSP and firmware libraries used in this application. To access the Library Manager, click the link from the **Quick Panel**.

   You can also just start the application creation process again and select a different kit.

   If you want to use the application for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. In the **Project Creator - Select Application** dialog, choose the example by enabling the checkbox.

4. (Optional) Change the suggested **New Application Name**.

5. The **Application(s) Root Path** defaults to the Eclipse workspace which is usually the desired location for the application. If you want to store the application in a different location, you can change the *Application(s) Root Path* value. Applications that share libraries should be in the same root path.

6. Click **Create** to complete the application creation process.

For more details, see the [Eclipse IDE for ModusToolbox&trade; software user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>

<details open><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox&trade; software provides the Project Creator as both a GUI tool and the command-line tool, "project-creator-cli". The CLI tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; software install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the "project-creator-cli" tool. On Windows, use the command line "modus-shell" program provided in the ModusToolbox&trade; software installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; software tools. You can access it by typing `modus-shell` in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The "project-creator-cli" tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the `<id>` field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the `<id>` field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br>

The following example will clone the [foc sensorless 6EDL7141 motor control](https://github.com/Infineon/mtb-example-xmc-foc-3-shunt) application with the desired name "sensorless_foc_6EDL7141_motor_control" configured for the *EVAL_6EDL7141_SLFOC_3SH* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id EVAL_6EDL7141_SLFOC_3SH --app-id mtb-example-xmc-foc-3-shunt --user-app-name foc_sensorless_6edl7141_motor_control --target-dir "C:/mtb_projects"
   ```

The following example will clone the [foc sensorless IMD700A motor control](https://github.com/Infineon/mtb-example-xmc-foc-3-shunt) application with the desired name "sensorless_foc_imd700a_motor_control" configured for the *EVAL_IMD700A_SLFOC_3SH* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id EVAL_IMD700A_SLFOC_3SH --app-id mtb-example-xmc-foc-3-shunt --user-app-name foc_sensorless_imd700a_motor_control --target-dir "C:/mtb_projects"
   ```

**Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project Creator tools" section of the [ModusToolbox&trade; software user guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>

<details open><summary><b>In third-party IDEs</b></summary>

**Note:** Only VS Code is supported.

1. Follow the instructions from the **In command-line interface (CLI)** section to create the application, and then import the libraries using the `make getlibs` command.

2. Export the application to a supported IDE using the `make <ide>` command.

3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox&trade; software user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; software install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Connect the board to your PC using a micro-USB cable through the debug USB connector.

2. Program the board using Eclipse IDE for ModusToolbox&trade; software:

   1. Select the application project in the Project Explorer.

   2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (JLink)**.

   The hex code should be downloaded without error.


## Debugging

You can debug the example to step through the code. In the IDE, use the **\<Application Name> Debug (JLink)** configuration in the **Quick Panel**. For more details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).


## Design and implementation

This example runs a BLDC motor using sensorless FOC motor control. Then, the hex code generated by this code example can be downloaded to the EVAL_6EDL7141_FOC_3SH board or EVAL_IMD700A_FOC_3SH via the BPA Motor Control GUI.

To customize the code example for your application, see the *Application Notes* folder in the **Documents** section, [AN_2110_PL88_2202_070459 - PMSM FOC motor control software using MOTIX&trade; IMD700A](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents).


With the BPA Motor Control GUI, you can configure the following options:

- Select the control mode as one of the following:
   - Speed control mode
   - Voltage control mode
- Select motor stop/run and set the target speed using one of the following:
   - Onboard potentiometer
   - Direct variable write
- Protections:
   - DC link overvoltage protection
   - DC link undervoltage protection


### MOTIX&trade; 6EDL7141 smart gate driver support

- Low-level read/write MOTIX&trade; 6EDL7141 registers through the SPI interface
- High-level 6EDL gateway provides an interface to the application code


### BPA Motor Control GUI's oscilloscope support

- 4-channel scope
- Trace signal of each channel can be changed on-the-fly


## Supported motor

This board and application support 3-phase BLDC motors (rated voltage 24 VDC)


### Correct motor connection

This board supports 3-phase BLDC/PMSM motors.
Connect the motor's phase U, V, W cables to the board as follows:

Motor    | Board
---------| --------
Phase U  | U (J3)
Phase V  | V (J4)
Phase W  | W (J5)

<br>

## Basic operations

### Motor control operation

- Change the control scheme, define the `USER_FOC_CTRL_SCHEME` macro to either (SPEED_INNER_CURRENT_CTRL) or (VQ_VOLTAGE_CTRL) in *user_input_config.h*. For example :
  ```c
  #define USER_FOC_CTRL_SCHEME   SPEED_INNER_CURRENT_CTRL // Set to speed control mode
  ```
  or

  ```c
  #define USER_FOC_CTRL_SCHEME   VQ_VOLTAGE_CTRL      // Set to voltage control mode
  ```


- Enable onboard potentiometer control for motor start/stop and target setting. Motor direction is controlled by the direction switch on the board (SW1).

- To enable the motor control with the potentiometer, define the `USER_REF_SETTING` macro as `(ENABLED)` in *user_input_config.h*. For example:

  ```c
  #define USER_REF_SETTING         (ENABLED) // Enable on-board potentiometer control
  ```

- To disable motor control with the potentiometer, define the `USER_REF_SETTING` macro as `(DISABLED)` in *user_input_config.h*. For example:

  ```c
  #define USER_REF_SETTING         (DISABLED) // Disable on-board potentiometer control
  ```
  When potentiometer control is disabled, motor start/stop and target setting change are controlled through a variable write:

  - To start the motor:
    ```c
    PMSM_FOC_MotorStart();                   // set motor_start flag
    ADC.adc_res_pot = 2048;                  // set adc_res_pot to 2048 = 50% nominal speed (speed mode), or 50% duty cycle (voltage mode)
    ```
  - To stop the motor:
    ```c
    ucProbe_cmd = 16;  // set motor stop command
    ```
  Scaling of `ADC.adc_res_pot` is Q12(4096=1PU), which means that `ADC.adc_res_pot` at the max value is 4096.

  - In voltage control mode, the torque Vq `PMSM_FOC_INPUT.ref_vq` controls the output duty cycle (32768 = 100% output duty cycle).

  - In speed control mode, the reference speed `PMSM_FOC_INPUT.ref_speed`  controls the motor speed (32768 = motor maximum speed); the actual motor speed can be read from the `PMSM_FOC_OUTPUT.rotor_speed` variable.

- The fault status can be read from `MotorVar.error_status` (unmasked faults) or `MotorVar.MaskedFault` (masked faults). Bitfields set in `MotorVar.error_status` mean that faults are present. Control will enter the FAULT state only if the fault is enabled and the bitfields are set in the `MotorVar.MaskedFault` masked fault variable. In other words, only an enabled fault can trigger the state machine to enter the FAULT status. In firmware, the masked fault is calculated as follows:

   ```c
   MotorVar.MaskedFault.Value = MotorVar.error_status & MotorParam.EnableFault
   ```
- A fault can be enabled/disabled by setting/clearing the corresponding bit in the `MotorParam.EnableFault` parameter. Its value is calculated by the Motor Control GUI.

- To clear the fault, do one of the following:

   - **Option 1:** Write `ucProbe_cmd` to '3'. After this command is received, firmware will write `MotorVar.fault_clear` to '1'.

    ```c
    ucProbe_cmd = 3;  // Control word 3: clear fault. Firmware will then set MotorVar.fault_clear to 1 after receiving this control word command
    ```
  - **Option 2** (Preferred): Directly write `MotorVar.fault_clear` to '1'.
    ```c
    MotorVar.fault_clear = 1;                     // Directly set clear fault flag
    ```
- The detected faults will be stored in different bit position of the  'MotorVar.error_status' variable. E.g., a '1' at bit 12 position indicates nFault from 6EDL7141 detected, a '1' at bit 10 position indicates DC link undervoltage detected, a '1' at bit 4 position indicates DC link overvoltage detected.
- Board temperature from MCP9700 is sampled, filtered, and converted to degree celsius. To read the board temperature:
  ```c
   int degreeC_x16 = MotorVar.t_sensor;  // Degree C temperature in Q4, 16 counts = 1°C* e.g. 0=0°C, 160=10°C, 320=20°C
  ```

### Managing the parameters

There are two types of parameters:

- **User input:** User input parameters are usually presented in engineering units (volt, amps, RPM, etc.) which you can configure based on the application need.

- **Internal parameters:** Internal parameters are actual parameters that are being used by firmware. Most internal parameters use different scaling to ensure efficient code execution, and so may not be easy to be understood.

One key feature of the Motor Control GUI is the ability to calculate the internal parameter from the user input parameter. A conversion function is available in firmware.

In the firmware, user inputs are `#define` in the *Configuration/pmsm_foc_user_input_config.h* header file; the conversion function is in the C source file, *ToolInterface/Register.c*.

All internal parameters related to motor control are grouped in the `MotorParam` data structure; members of this structure must be configured correctly before the motor could run. Use the BPA Motor Control GUI to enter the user input and let the tool calculate the internal parameter value, and then click **Write Parameter** to write all internal parameters into the MCU's RAM. After completing this step, the motor should be ready to run.

You can save the parameters into the MCU so that the next time when the board powers up, the motor is ready to run immediately. A location inside the MCU's flash memory is designated for storing the internal parameters. Click **Write to Flash** in the Motor Control GUI to store the internal parameters into the flash memory.

All 6EDL7141 config registers are handled in a similar way.

To write both `MotorParam` and `Edl7141Reg` into the flash, set the control word to '4':

```c
ucProbe_cmd = 4;    // Control word 4: program parameter RAM value into flash memory
```

### Using the oscilloscope

Scope traces are no longer selected in the oscilloscope page. The register map defined in *Register.c* contains the TraceId, variable, data type, and bit field (if in use). Change `ProbeScope_Chx.TraceId` (x=1, 2, 3, or 4) to select a different trace signal for each scope channel so that the Motor Control GUI can select the scope signal in Test Bench view.


### Controlling MOTIX&trade; 6EDL7141 smart gate driver and MOTIX&trade; IMD700A motor controller

The EN_DRV pin output level (1 or 0) from XMC1400 MCU is controlled by `EdlIo.en_drv_level`. Note that the motor state machine controls the EN_DRV pin in some state changes; the EN_DRV pin control is valid only if there is no motor control state change.

The nBrake pin output level (1 or 0) from XMC1400 MCU is controlled by `EdlIo.nbrake_level`. Note that the nBrake pin control is independent of motor control.

>**IMPORTANT:** EN_DRV and nBrake control are only for testing the MOTIX&trade; 6EDL7141 function. If nBrake is set to low level manually while the motor is running, it may cause faults such as stall error. In such cases, you should set nBrake to a high level, clear the fault, and verify that the motor is in ‘STOP’ state before running the motor again. Do not set EN_DRV to low level while the motor is running.

DC calibration control can be turned on or off by writing '1' or '0' to the MOTIX&trade; 6EDL7141 configure register bitfield `CSAMP_CFG[11]`.

ADC input selection (Idigital/DVDD/VDDB) is written to the MOTIX&trade; 6EDL7141 configure register bitfield `ADC_CFG[2:1]`.

An ADC request is initiated by writing '1' to the MOTIX&trade; 6EDL7141 configure register bitfield `ADC_CFG[0]`.


## Firmware code overview

This firmware code example can integrate with the BPA Motor Control GUI to provide an easy user configuration for the motor control system.

The code has stored most of the configurations in the `MotorParam` structure. Those parameters in this structure  have internal scaling. The BPA Motor Control GUI is designed to take the user input (user input parameter) and convert it into internal parameter values in `MotorParam`. This means that these parameters can be changed without recompiling or reflashing the code.

If you choose not to use the BPA Motor Control GUI for parameter configuration, this process can be simulated by taking the user input into *user_input_config.h*, and then converting it into internal parameter values by using the `parameter_set_default()` function in *parameter_bldc.c*. This function will be called once during code initialization; it initializes the parameters in the `MotorParam` structure into default values. These values can be changed by modifying the member values in the `MotorParam` structure directly.

A different control scheme can be switched at the motor stop state by changing the value in `MotorParam.ControlScheme`.

The fault bit in `error_status` will not cause a motor fault unless the corresponding bit in `MotorParam.EnableFault` is set. In other words, `error_status` is masked by `EnableFault`; if a fault is not enabled by the `EnableFault` bit, the fault will not put the motor control into the ERROR state.

### Resources and settings

The project uses the default *design.modus* file.

## Known limitations

This section lists the current known limitations of the system.

### Supported operating modes between the Motor Control GUI and code example

Currently, the following two modes of operations are supported in using the BPA Motor Control GUI and the code example itself in source code form:

- **Using the BPA Motor Control GUI**

   You can use the BPA Motor Control GUI for evaluation of the code example performance and tuning the parameters by flashing the board using the programming features of the BPA Motor Control GUI. For this, you must supply both the HEX and ELF artifacts of the build to the tool following the on-screen prompts. Using this method, the tool will always overwrite the motor control parameters that are defined in the firmware source code. Therefore, editing of parameters is possible only through the BPA Motor Control GUI.

   You can use all BPA Motor Control GUI features to modify the parameters, control the motor, and observe the application performance. This option is intended for early evaluation of the related kit and firmware or for fine-tuning the motor control parameters in a graphical way.

- **Using the code example in source code form with ModusToolbox&trade;**

   If you use the code example in source code format and modify parameters of the application, motor control, or MCU, compatibility with the BPA Motor Control GUI cannot be guaranteed and might lead to unexpected behavior. This applies especially to modifying motor control parameters that are automatically overwritten in the tool as mentioned above. Use this option if you want to edit the firmware source code or tune the parameters in the source code.


### Oscillation in motor RPM against the set target

In normal operation with a running motor in speed control mode, the observable motor RPM can be slightly different from the specified target speed. This variation from the target speed is dependant on the PI tuning of the control loop and whether the motor is loaded or running free.

This code example is not fine-tuned for any specific motor. Fine tuning of the motor control parameters can be done either in firmware or by using the BPA Motor Control GUI.


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes | [AN_2112_PL88_2112_011208 - EVAL_IMD700A_FOC_3SH 18 V brushless DC motor drive board](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents) <br> [AN_2201_PL88_2202_025343 - Sensorless FOC tuning guide for BPA motor control GUI](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents)
Code examples  | [Using ModusToolbox&trade; software](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [MOTIX&trade; 6EDL7141 datasheets](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/6edl7141/#!documents); [MOTIX&trade; IMD700A datasheets](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents)
Development kits | [MOTIX&trade; 6EDL7141 eval boards](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/6edl7141/#!boards); [MOTIX&trade; IMD700A eval boards](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!boards)
Tools  | [Eclipse IDE for ModusToolbox&trade; software](https://www.cypress.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use software and tools enabling rapid development with Infineon MCUs, covering applications from embedded sense and control to wireless and cloud-connected systems using AIROC&trade; Wi-Fi and Bluetooth® connectivity devices.


## Other resources

Infineon provides a wealth of data at www.infineon.com to help you select the right device, and quickly and effectively integrate it into your design.

For XMC&trade; MCU devices, see [32-bit XMC™ industrial microcontroller based on Arm® Cortex®-M](https://www.infineon.com/cms/en/product/microcontroller/32-bit-industrial-microcontroller-based-on-arm-cortex-m/).


## Document history

Document title: *CE237314* – *XMC&trade; MCU: 6EDL7141 and MOTIX&trade; IMD700A sensorless FOC 3-shunt* 

| Version | Description of change |
| ------- | --------------------- |
| 0.5.0   | Initial version       |

------

All other trademarks or registered trademarks referenced herein are the property of their respective owners.

--------------
© 2023 Infineon Technologies AG

All Rights Reserved.

### Legal Disclaimer

The information given in this document shall in no event be regarded as a guarantee of conditions or characteristics. With respect to any examples or hints given herein, any typical values stated herein and/or any information regarding the application of the device, Infineon Technologies hereby disclaims any and all warranties and liabilities of any kind, including without limitation, warranties of non-infringement of intellectual property rights of any third party.

### Information

For further information on technology, delivery terms and conditions and prices, please contact the nearest Infineon Technologies Office (www.infineon.com).

### Warnings

Due to technical requirements, components may contain dangerous substances. For information on the types in question, please contact the nearest Infineon Technologies Office.

Infineon Technologies components may be used in life-support devices or systems only with the express written approval of Infineon Technologies, if a failure of such components can reasonably be expected to cause the failure of that life-support device or system or to affect the safety or effectiveness of that device or system. Life support devices or systems are intended to be implanted in the human body or to support and/or maintain and sustain and/or protect human life. If they fail, it is reasonable to assume that the health of the user or other persons may be endangered.

--------------------------------------------------------------
