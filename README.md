# NXP Application Code Hub
[<img src="https://mcuxpresso.nxp.com/static/icon/nxp-logo-color.svg" width="100"/>](https://www.nxp.com)

## LPC553x dual-servo motor control
<div>This application note describes the dual servo demo with the NXP LPC55S36&nbsp;<span style="background-color: rgb(255 255 255/var(--tw-bg-opacity)); color: var(--tw-prose-body); font-size: 1rem;">processor. It also can be used as a reference for motor control application&nbsp;</span><span style="background-color: rgb(255 255 255/var(--tw-bg-opacity)); color: var(--tw-prose-body); font-size: 1rem;">developing based on other products.</span></div>

This demo includes LPCXpresso55S36, two FRDM-MC-PMSM driver boards, and two three-phase servo motors. The LPC553x/ LPC55S3x processor samples the currents and voltages of the motor through the ADC. The ENC module receives the encoder signal to obtain the rotor position and speed, and generates PWM based on the FOC algorithm to drive the motor. At the same time, UART can be used to communicate with the FreeMASTER to achieve command sending, variable observation, and other functions that are convenient for users to debug. Finally, precise position control and smooth speed regulation can be achieved.

#### Boards: LPCXpresso55S36

#### Categories: Motor control
#### Peripherals: ADC, GPIO, CLOCKS, PWM, UART
#### Toolchains: IAR

## Table of Contents
1. [System Structure](#step1)
2. [Software](#step2)
3. [Hardware](#step3)
4. [Setup](#step4)
5. [Results](#step5)
6. [Support](#step6)
7. [Release Notes](#step7)

## 1. System Structure

This figure presents the system structure block diagram of this dual servo demo.

•  LPCXpresso55S36 designed by NXP contains the LPC553x/LPC55S3x chip and peripheral interfaces.

•  FRDM-MC-LVPMSM designed by NXP is a motor driver board that contains driver bridges, analog sampling circuits, and an encoder interface.

•  M1 and M2 are the servo motors that include 1000 lines encoder.

•  eFlexPWM, ENC, ADC are on-chip peripherals, used for motor control, encoder signal acquisition, and analog acquisition respectively.

•  InputMUX is an input multiplexing module that can provide different signal path options for the internal peripherals of the chip. In this demo, it is responsible for providing signal connections for PWM synchronization, ADC hardware triggering, and fault protection.

•  The application software is running on LPC553x/LPC55S3x which includes the FOC algorithm, CM33_RTCESL_4.6.2 (Real-Time Control Embedded Software Motor Control and Power Conversion Libraries), and SDK 2.10.

•  Flexcomm provides various peripheral function options that can be configured into USART, SPI, I2C, I2S functions through software. Here we configure the USART function to realize the communication between the FreeMASTER debugging tool and LPC553x/LPC55S3x to demonstrate the user's operation.

![](picture/system_structure.png)

## 2. Software

- CM33_RTCESL_4.6.2

- SDK: 2.10

- FreeMASTER 3.1.2

- IAR Workbench IDE 8.50.9

## 2. Hardware

-  LPC55S36-EVK REV B

-  Two FRDM-MC-LVPMSM boards

-  Two 24 V servo motors

-  Micro USB cable

## 3. Setup<a name="step3"></a>

1. Plug the LPCXpresso55S36 and FRDM-MC-LVPMSM board together via Arduino interface, connect motor wires and encoder interface.

   ![](picture/front.png)

2. Power on 24 V adapter to power on the FRDM-MC-LVPMSM board.

3. Connect LPCXpresso55S36 and PC via USB interface.

4. Open FM_DualServo.pmp in the software package. (FREEMATER version must be not lower than 3.1.2)

5. Click the GO! button to enable the communication between PC and LPC553x/LPC55S3x.

   ![](picture/FreeMASTER_control_page.png)

6. Click the DualServo page.

7. Click the Start button to enable demo.

8. Operate the demo by clicking other buttons on the control page.

## 4. Results<a name="step4"></a>

All the following experimental results are tested when the motor is loaded with a light plastic ring. And all the figures come from the FreeMASTER.

Below figure shows the speed and current waveforms when the motor startup is at 2500 RPM. The red line is speed requirement, the green line is actual speed, and blue line is torque current. We can see that it can accelerate to 2500 RPM within 0.13 s, and the overshoot is very small.

![](picture/speed_and_current_response.png)

As shown in [Figure 18](#_bookmark22), the top waveforms show the speed response and the bottom waveforms show the position response. The red line is the requirement, the green line is the actual value, and the blue line shows the error between them. After setting the 180o position requirement, it takes about 0.1 seconds to reach the desired position. We can see that the error of dynamic response is small and the static response is very stable.

![](picture/Position_and_speed_response.png)

Please refer to the application note for the details: https://www.nxp.com.cn/docs/en/application-note/AN13569.pdf

## 5. Support<a name="step5"></a>
#### Project Metadata
<!----- Boards ----->
[![Board badge](https://img.shields.io/badge/Board-LPCXPRESSO55S36-blue)](https://github.com/search?q=org%3Anxp-appcodehub+LPCXpresso55S36+in%3Areadme&type=Repositories)

<!----- Categories ----->
[![Category badge](https://img.shields.io/badge/Category-MOTOR%20CONTROL-yellowgreen)](https://github.com/search?q=org%3Anxp-appcodehub+motor_control+in%3Areadme&type=Repositories)

<!----- Peripherals ----->
[![Peripheral badge](https://img.shields.io/badge/Peripheral-ADC-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+adc+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-GPIO-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+gpio+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-CLOCKS-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+clocks+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-PWM-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+pwm+in%3Areadme&type=Repositories) [![Peripheral badge](https://img.shields.io/badge/Peripheral-UART-yellow)](https://github.com/search?q=org%3Anxp-appcodehub+uart+in%3Areadme&type=Repositories)

<!----- Toolchains ----->
[![Toolchain badge](https://img.shields.io/badge/Toolchain-IAR-orange)](https://github.com/search?q=org%3Anxp-appcodehub+iar+in%3Areadme&type=Repositories)

Questions regarding the content/correctness of this example can be entered as Issues within this GitHub repository.

>**Warning**: For more general technical questions regarding NXP Microcontrollers and the difference in expected funcionality, enter your questions on the [NXP Community Forum](https://community.nxp.com/)

[![Follow us on Youtube](https://img.shields.io/badge/Youtube-Follow%20us%20on%20Youtube-red.svg)](https://www.youtube.com/@NXP_Semiconductors)
[![Follow us on LinkedIn](https://img.shields.io/badge/LinkedIn-Follow%20us%20on%20LinkedIn-blue.svg)](https://www.linkedin.com/company/nxp-semiconductors)
[![Follow us on Facebook](https://img.shields.io/badge/Facebook-Follow%20us%20on%20Facebook-blue.svg)](https://www.facebook.com/nxpsemi/)
[![Follow us on Twitter](https://img.shields.io/badge/Twitter-Follow%20us%20on%20Twitter-white.svg)](https://twitter.com/NXP)

## 6. Release Notes<a name="step6"></a>
| Version | Description / Update                           | Date                        |
|:-------:|------------------------------------------------|----------------------------:|
| 1.0     | Initial release on Application Code HUb        | June 5<sup>nd</sup> 2023 |

