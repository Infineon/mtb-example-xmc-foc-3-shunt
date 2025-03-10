# XMC&trade; MCU:MOTIX&trade; 6EDL71x1 and IMD700A Sensorless FOC 3-shunt

This code example works with MOTIX&trade; 6EDL7151 3-shunt field-oriented control (FOC) sensorless 3-shunt FOC sensorless projects in the ModusToolbox&trade; Motor Suite GUI to illustrate sensorless FOC motor control on [EVAL_6EDL7151_FOC_3SH](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/#!documents) evaluation boards for FOC motor drive based on the MOTIX&trade; 6EDL7151 smart 3-phase driver.

This code example also works with both the MOTIX&trade; 6EDL7141 3-shunt FOC sensorless and MOTIX&trade; IMD700A 3-shunt FOC sensorless projects in the ModusToolbox&trade; Motor Suite GUI to illustrate sensorless FOC motor control on [EVAL_6EDL7141_FOC_3SH](https://www.infineon.com/dgdl/Infineon-Evaluation_Board_EVAL_6EDL7141_FOC_3SH-UserManual-v01_00-EN.pdf?fileId=8ac78c8c82ce566401830db3784605f9) and [EVAL_IMD700A_FOC_3SH](https://www.infineon.com/dgdl/Infineon-Evaluation_board_EVAL_IMD700A_FOC_3SH-ApplicationNotes-v01_00-EN.pdf?fileId=8ac78c8c7fb5929e017fdf3691800b9d) evaluation boards for FOC motor drive based on the MOTIX&trade; 6EDL7141 smart 3-phase driver and MOTIX&trade; IMD701A motor controller IC. However, this will involve changes to the `TARGET` definition in the Makefile of the project.


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.1)
- [SEGGER J-Link](https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack) software
- [ModusToolbox&trade; Motor Suite GUI](https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ifxmotorsolutions) software (formerly MOTIX&trade; BPA Motor Control Workbench)
- Programming language: C
- Associated parts: All [MOTIX&trade; 6EDL7141](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/6edl7141/) parts, all [MOTIX&trade; IMD701A](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd701a-q064x128-aa/) parts


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`

## Supported boards (make variable 'TARGET')

- [EVAL_IMD700A_FOC_3SH Evaluation Board - sensorless FOC motor drive based on the MOTIX&trade; IMD701A motor controller](https://www.infineon.com/cms/en/product/evaluation-boards/eval_imd700a_foc_3sh/) (`EVAL_IMD700A_FOC_3SH`)
- [EVAL_6EDL7141_FOC_3SH Evaluation Board - sensorless FOC motor drive based on the MOTIX&trade; 6EDL7141 smart 3-phase driver](https://www.infineon.com/cms/en/product/evaluation-boards/eval_6edl7141_foc_3sh/) (`EVAL_6EDL7141_FOC_3SH`)
- [EVAL_6EDL7151_FOC_3SH Evaluation Board - sensorless FOC motor drive based on the MOTIX&trade; 6EDL7151 smart 3-phase driver](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/#!designsupport) (`EVAL_6EDL7151_36V_1kW`) – Default value of `TARGET`


## Hardware setup

This example uses the board's default configuration. See the board user guide to ensure that the board is configured correctly.


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

This example is compatible with the [ModusToolbox&trade; Motor Suite GUI](https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ifxmotorsolutions) software. Before you can install this tool, install the [Infineon Developer Center Launcher](https://www.infineon.com/cms/en/design-support/tools/utilities/infineon-developer-center-idc-launcher/) application. To interact with the ModusToolbox&trade; Motor Suite GUI, see the [Design and implementation](#design-and-implementation) section. This code example can be compiled in Debug mode to ensure the full compatibility features with the ModusToolbox&trade; Motor Suite GUI.


## Using the code example


### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*).

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-boards-make-variable-target).

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work.

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you.

   b. Select this code example from the list by enabling its check box.

   > **Note:** You can narrow the list of displayed examples by typing in the filter box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[mtb-example-xmc-foc-3-shunt](https://github.com/Infineon/mtb-example-xmc-foc-3-shunt)" application with the desired name "Sensorless_FOC_3_Shunt" configured for the *EVAL_6EDL7151_36V_1kW* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id EVAL_6EDL7151_36V_1kW --app-id mtb-example-xmc-foc-3-shunt --user-app-name Sensorless_FOC_3_Shunt --target-dir "C:/mtb_projects"
   ```


The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br>

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>


### Open the project

After the project has been created, you can open it in your preferred development environment.

<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Connect the board to your PC using a Micro-USB cable through the debug USB connector.

2. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (JLink)**.
   </details>

   <details><summary><b>In other IDEs</b></summary>

   Follow the instructions in your preferred IDE.
   </details>


   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

Download the hex code without error.


## Debugging

You can debug the example to step through the code.


<details><summary><b>In Eclipse IDE</b></summary>

Use the **\<Application Name> Debug (JLink)** configuration in the **Quick Panel**. For more details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

</details>


<details><summary><b>In other IDEs</b></summary>

Follow the instructions in your preferred IDE.

</details>


## Design and implementation

This example runs a brushless DC (BLDC) motor using sensorless FOC motor control. Then, the hex code generated by this code example can be downloaded to the EVAL_6EDL7141_FOC_3SH, or EVAL_IMD700A_FOC_3SH, or EVAL_6EDL7151_FOC_3SH board via the ModusToolbox&trade; Motor Suite GUI. However, before generating the hex code, ensure that the `TARGET` definition in the Makefile is set to the desired evaluation board.

To customize the code example for your application, see the appropriate application note which can be found by expanding the *Application Notes* folder in the **Documents** section, [AN070459 - PMSM FOC motor control software using MOTIX&trade; 6EDL7141 and MOTIX&trade; IMD700A](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents) or [AN043426 - PMSM FOC motor control software using MOTIX&trade; 6EDL7151](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/6edl7151/#!documents).

With the ModusToolbox&trade; Motor Suite GUI, you can configure the following options:

- Select the control mode as one of the following:
   - Speed control mode
   - Voltage control mode

- Select motor stop/run and set the target speed using one of the following:
   - Onboard potentiometer
   - Direct variable write

- Protections:
   - DC link overvoltage protection
   - DC link undervoltage protection


### MOTIX&trade; 6EDL7141 and MOTIX&trade; 6EDL7151 smart gate driver support

- Low-level read/write MOTIX&trade; 6EDL7141 or MOTIX&trade; 6ELD7151 registers through the SPI interface
- High-level 6EDL gateway provides an interface to the application code


### ModusToolbox&trade; Motor Suite GUI's oscilloscope support

- 4-channel scope
- Trace signal of each channel can be changed on-the-fly


## Supported motor

This board and application support 3-phase BLDC motors (rated voltage 24 V DC).


### Correct motor connection

This board supports 3-phase BLDC/PMSM motors.
Connect the motor's phase U, V, and W cables to the board as follows:

**Table 1. Motor's phase connection**

Motor    | Board
---------| --------
Phase U  | U (J4)
Phase V  | V (J5)
Phase W  | W (J6)

<br>


## Basic operations

### Motor control operation

- Change the control scheme, define the `USER_FOC_CTRL_SCHEME` macro to either (SPEED_INNER_CURRENT_CTRL) or (VQ_VOLTAGE_CTRL) in the *user_input_config.h* file. For example:
  
   ```
   #define USER_FOC_CTRL_SCHEME   SPEED_INNER_CURRENT_CTRL // Sets to speed control mode
   ```
   or

   ```
   #define USER_FOC_CTRL_SCHEME   VQ_VOLTAGE_CTRL      // Sets to voltage control mode
   ```

- Enable onboard potentiometer control for motor start/stop and target setting. Motor direction is controlled by the direction switch on the board (SW1).

- To enable the motor control with the potentiometer, define the `USER_REF_SETTING` macro as `(ENABLED)` in *user_input_config.h*. For example:

   ```
   #define USER_REF_SETTING         (ENABLED) // Enables onboard potentiometer control
   ```

- To disable motor control with the potentiometer, define the `USER_REF_SETTING` macro as `(DISABLED)` in the *user_input_config.h* file. For example:

   ```
   #define USER_REF_SETTING         (DISABLED) // Disables onboard potentiometer control
   ```
  When potentiometer control is disabled, motor start/stop and target setting change are controlled through a variable write:

  - To start the motor:
    
    ```
    PMSM_FOC_MotorStart();                   // Sets motor_start flag
    ADC.adc_res_pot = 2048;                  // Sets adc_res_pot to 2048 = 50% nominal speed (speed mode) or 50% duty cycle (voltage mode)
    ```
  - To stop the motor:
    
    ```
    ucProbe_cmd = 16;  // Sets motor stop command
    ```
    
  Scaling of `ADC.adc_res_pot` is Q12(4096=1PU), which means that `ADC.adc_res_pot` at the max value is 4096.

  - In voltage control mode, the torque Vq `PMSM_FOC_INPUT.ref_vq` controls the output duty cycle (32768 = 100% output duty cycle).

  - In speed control mode, the reference speed `PMSM_FOC_INPUT.ref_speed`  controls the motor speed (32768 = motor maximum speed); the actual motor speed can be read from the `PMSM_FOC_OUTPUT.rotor_speed` variable.

- The fault status can be read from `MotorVar.error_status` (unmasked faults) or `MotorVar.MaskedFault` (masked faults). Bitfields set in `MotorVar.error_status` mean that faults are present. Control will enter the FAULT state only if the fault is enabled and the bitfields are set in the `MotorVar.MaskedFault` masked fault variable. In other words, only an enabled fault can trigger the state machine to enter the FAULT status. In firmware, the masked fault is calculated as follows:

   ```
   MotorVar.MaskedFault.Value = MotorVar.error_status & MotorParam.EnableFault
   ```
- A fault can be enabled/disabled by setting/clearing the corresponding bit in the `MotorParam.EnableFault` parameter. Its value is calculated by the ModusToolbox&trade; Motor Suite GUI.

- To clear the fault, do one of the following:

   - **Option 1:** Write `ucProbe_cmd` to '3'. After this command is received, the firmware will write `MotorVar.fault_clear` to '1'.

     ```
     ucProbe_cmd = 3;  // Control word 3: clear fault. Firmware will then set `MotorVar.fault_clear` to '1' after receiving this control word command
     ```

  - **Option 2** (Preferred): Directly write `MotorVar.fault_clear` to '1'.
  
    ```
    MotorVar.fault_clear = 1;                     // Directly sets the clear fault flag
    ```
- The detected faults will be stored in a different bit position of the 'MotorVar.error_status' variable. E.g., a '1' at bit 12 position indicates nFault from MOTIX&trade; 6EDL7141 or MOTIX&trade; 6EDL7151 detected, a '1' at bit 10 position indicates DC link undervoltage detected, a '1' at bit 4 position indicates DC link overvoltage detected.
- Board temperature from MCP9700 is sampled, filtered, and converted to degree celsius. To read the board temperature:
  
   ```
   int degreeC_x16 = MotorVar.t_sensor;  // Degree celsius temperature in Q4, 16 counts = 1°C e.g., 0 = 0°C, 160 = 10°C, 320 = 20°C
   ```

### Managing the parameters

There are two types of parameters:

- **User input:** User input parameters are usually presented in engineering units (volt, amps, RPM, etc.) which can be configured based on the application requirements.

- **Internal parameters:** Internal parameters are actual parameters that are being used by the firmware. Most internal parameters use different scaling to ensure efficient code execution and cannot be easy to be understood.

One key feature of the Motor Control GUI is the ability to calculate the internal parameter from the user input parameter. A conversion function is available in the firmware.

In the firmware, user inputs are `#define` in the *Configuration/pmsm_foc_user_input_config.h* header file; the conversion function is in the C source file, *ToolInterface/Register.c*.

All internal parameters related to motor control are grouped in the `MotorParam` data structure; configure this structure correctly before the motor could run. Use the ModusToolbox&trade; Motor Suite GUI to enter the user input and let the tool calculate the internal parameter value, and then click **Write Parameter** to write all internal parameters into the MCU's RAM. After completing this step, the motor should be ready to run.

You can save the parameters into the MCU so that the next time when the board powers up, the motor is ready to run immediately. A location inside the MCU's flash memory is designated for storing the internal parameters. Click **Write to Flash** in the ModusToolbox&trade; Motor Suite GUI to store the internal parameters into the flash memory.

All MOTIX&trade; 6EDL7141 or MOTIX&trade; 6EDL7151 config registers are handled in a similar way.

To write both `MotorParam` and `Edl7141Reg` into the flash, set the control word to '4':

```
ucProbe_cmd = 4;    // Control word 4: program parameter RAM value into flash memory
```


### Using the oscilloscope

Scope traces are selected in the oscilloscope tool of the ModusToolbox&trade; Motor Suite GUI.


### Controlling MOTIX&trade; 6EDL7141, MOTIX&trade; 6EDL7151 smart gate driver, and MOTIX&trade; IMD700A motor controller

The EN_DRV pin output level ('1' or '0') from XMC1400 MCU is controlled by `EdlIo.en_drv_level`. Note that the motor state machine controls the EN_DRV pin in some state changes; the EN_DRV pin control is valid only if there is no motor control state change.

The nBrake pin output level ('1' or '0') from XMC1400 MCU is controlled by `EdlIo.nbrake_level`. Note that the nBrake pin control is independent of motor control.

> **Note:** EN_DRV and nBrake control are only for testing the MOTIX&trade; 6EDL7141 and MOTIX&trade; 6EDL7151 smart gate driver function. If nBrake is set to a low level manually while the motor is running, it can cause faults such as stall error. In such cases, set nBrake to a high level, clear the fault, and verify that the motor is in ‘STOP’ state before running the motor again. Do not set EN_DRV to a low level while the motor is running.

DC calibration control can be turned on or off by writing '1' or '0' to the MOTIX&trade; 6EDL7141 or MOTIX&trade; 6ELD7151 configure register bitfield `CSAMP_CFG[11]`.

ADC input selection (Idigital/DVDD/VDDB) is written to the MOTIX&trade; 6EDL7141 or MOTIX&trade; 6ELD7151 configure register bitfield `ADC_CFG[2:1]`.

An ADC request is initiated by writing '1' to the MOTIX&trade; 6EDL7141 or MOTIX&trade; 6ELD7151 configure register bitfield `ADC_CFG[0]`.


## Firmware code overview

This firmware code example can integrate with the ModusToolbox&trade; Motor Suite GUI to provide an easy user configuration for the motor control system.

The code has stored most of the configurations in the `MotorParam` structure. Those parameters in this structure have internal scaling. The ModusToolbox&trade; Motor Suite GUI is designed to take the user input (user input parameter) and convert it into internal parameter values in `MotorParam`. This means that these parameters can be changed without recompiling or reflashing the code.

If you choose not to use the ModusToolbox&trade; Motor Suite GUI for parameter configuration, this process can be simulated by taking the user input into *user_input_config.h* file, and then converting it into internal parameter values by using the `PROGRAM_DEFAULT_PARAM = 1` macro in *Makefile* file of the project. This macro will be used during code initialization; it initializes the parameters in the `MotorParam` structure into default values. These values can be changed by modifying the member values in the `MotorParam` structure directly.

A different control scheme can be switched at the motor stop state by changing the value in `MotorParam.ControlScheme`.

The fault bit in `error_status` will not cause a motor fault unless the corresponding bit in `MotorParam.EnableFault` is set. In other words, `error_status` is masked by `EnableFault`; if a fault is not enabled by the `EnableFault` bit, the fault will not put the motor control into the ERROR state.


### Resources and settings

The project uses the default *design.modus* file.


## Known limitations

This section lists the current known limitations of the system.


### Supported operating modes between the ModusToolbox&trade; Motor Suite GUI and code example

Currently, the following two modes of operations are supported in using the ModusToolbox&trade; Motor Suite GUI and the code example itself in source code form:

- **Using the ModusToolbox&trade; Motor Suite GUI**

   Use the ModusToolbox&trade; Motor Suite GUI for evaluation of the code example performance and tuning the parameters by flashing the board using the programming features of the ModusToolbox&trade; Motor Suite GUI. For this, supply both the HEX and ELF artifacts of the build to the tool following the on-screen prompts. Using this method, the tool will always overwrite the motor control parameters that are defined in the firmware source code. Therefore, editing of parameters is possible only through the ModusToolbox&trade; Motor Suite GUI.

   Use all ModusToolbox&trade; Motor Suite GUI features to modify the parameters, control the motor, and observe the application performance. This option is intended for early evaluation of the related kit and firmware or for fine-tuning the motor control parameters in a graphical way.

- **Using the code example in source code form with ModusToolbox&trade;**

   If you use the code example in source code format and modify the parameters of the application, motor control, or MCU, compatibility with the ModusToolbox&trade; Motor Suite GUI cannot be guaranteed and can lead to unexpected behavior. This applies especially to modifying motor control parameters that are automatically overwritten in the tool as mentioned above. Use this option if you want to edit the firmware source code or tune the parameters in the source code.


### Oscillation in motor RPM against the set target

In normal operation with a running motor in speed control mode, the observable motor RPM can be slightly different from the specified target speed. This variation from the target speed is dependent on the PI tuning of the control loop and whether the motor is loaded or running free.

This code example is not fine-tuned for any specific motor. Fine-tuning of the motor control parameters can be done either in firmware or by using the ModusToolbox&trade; Motor Suite GUI.


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes | [AN_2112_PL88_2112_011208 - EVAL_IMD700A_FOC_3SH 18 V brushless DC motor drive board](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents) <br> [AN_2201_PL88_2202_025343 - Sensorless FOC tuning guide for BPA motor control GUI](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents) <br> [AN043426 - PMSM FOC motor control software using MOTIX&trade; 6EDL7151](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/6edl7151/#!documents)
Code examples  | [Using ModusToolbox&trade; software](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [MOTIX&trade; 6EDL7141 datasheets](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/6edl7141/#!documents); [MOTIX&trade; IMD700A datasheets](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!documents); [MOTIX&trade; 6EDL7151 datasheets](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/#!documents)
Development kits | [MOTIX&trade; 6EDL7141 eval boards](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/6edl7141/#!boards); [MOTIX&trade; IMD700A eval boards](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-driver-ics/battery-supplied-bldc-motor-controller-ics/imd700a-q064x128-aa/#!boards); [MOTIX&trade; 6EDL7151 eval boards](https://www.infineon.com/cms/en/product/power/motor-control-ics/bldc-motor-control-ics/battery-supplied-bldc-motor-controller-ics/#!documents)
Libraries on GitHub | [mtb-xmclib-cat3](https://github.com/Infineon/mtb-xmclib-cat3) – XMC&trade; Peripheral Driver Library (XMCLib)
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development.

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.

For XMC&trade; MCU devices, see [32-bit XMC&trade; industrial microcontroller based on Arm&reg; Cortex&reg;-M](https://www.infineon.com/cms/en/product/microcontroller/32-bit-industrial-microcontroller-based-on-arm-cortex-m/).


## Document history

Document title: *CE237314* – *XMC&trade; MCU: MOTIX&trade; 6EDL7141, 6EDL7151, and IMD700A sensorless FOC 3-shunt* 

 Version | Description of change
 ------- | ---------------------
 0.5.0   | Initial version
 0.5.1   | Bug fixes and Probe Scope update supporting arbitrary variables access from ModusToolbox&trade; Motor Suite GUI. This enables users to observe any variables in Oscilloscope and Custom GUI Builder features
 0.5.2   | Adding code to support features available in MOTIX&trade; 6EDL7151 device
<br>


All other trademarks or registered trademarks referenced herein are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

© 2024 Infineon Technologies AG

All Rights Reserved.

### Legal disclaimer

The information given in this document shall in no event be regarded as a guarantee of conditions or characteristics. With respect to any examples or hints given herein, any typical values stated herein and/or any information regarding the application of the device, Infineon Technologies hereby disclaims any and all warranties and liabilities of any kind, including without limitation, warranties of non-infringement of intellectual property rights of any third party.

### Information

For further information on technology, delivery terms and conditions and prices, please contact the nearest Infineon Technologies Office (www.infineon.com).

### Warnings

Due to technical requirements, components may contain dangerous substances. For information on the types in question, please contact the nearest Infineon Technologies Office.

Infineon Technologies components may be used in life-support devices or systems only with the express written approval of Infineon Technologies, if a failure of such components can reasonably be expected to cause the failure of that life-support device or system or to affect the safety or effectiveness of that device or system. Life support devices or systems are intended to be implanted in the human body or to support and/or maintain and sustain and/or protect human life. If they fail, it is reasonable to assume that the health of the user or other persons may be endangered.

----------------------------------------------------------------------------