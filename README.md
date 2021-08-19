<!-- TODO 
- Add image backgrounds
- Check the alternate text on images
- Add paypal link
 -->

# Overview
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/OverviewDiagram.png" alt="Overview of the Pneumatic Soft Robotics Controller (PneuSoRD), including the electronics driver board and National Instruments myRIO controller, which is connected to a single pump and air receiver, up to 5 proportional valves, and 26 on-off valves. Soft robotic glove reproduced with permission from [35]. Copyright 2019, Mary Ann Liebert, Inc." width="600">
</p>

The Pneumatic Soft Robotics Driver (PneuSoRD) is an open-source power electronics design for controlling pneumatic soft robotic actuators with a large number of simultaneous actuated chambers. This system is designed around the two major categories of valve types. On/Off valves with single or multiple solenoids which each take a binary input and proportional valves which can be driven from a variable DC voltage. Therefore, the PneuSoRD design encompasses a modular solution for each drive system: The "Proportional Drive" and the "On/Off Drive". Each module has been designed to be as generic as possible to allow control of a large range of valves with an available control strategy for each. These two modules can be driven by either a myRIO 1950 embedded controller through a real-time visual user interface with LabVIEW or an Arduino DUE.

A link to the paper can be found [here]: <!-- http://www.reddit.com  TODO -->

PneuSoRD is available to be purchased from: **Temporarily unavailable**
<!-- <form action="https://www.paypal.com/cgi-bin/webscr" method="post" target="_top">
<input type="hidden" name="cmd" value="_s-xclick">
<input type="hidden" name="hosted_button_id" value="HB4DUWE5M24YY">
<input type="image" src="https://www.paypalobjects.com/en_AU/i/btn/btn_buynowCC_LG.gif" border="0" name="submit" alt="PayPal – The safer, easier way to pay online!">
<img alt="" border="0" src="https://www.paypalobjects.com/en_AU/i/scr/pixel.gif" width="1" height="1">
</form> -->


**NOTE:** rev. 0.2 boards (unmarked revision) require the primary voltage input to be 7-12V. The secondary input can be used 7-25V.

## PneuSoRD Driver and Shield
The PneuSoRD can be replicated via the PCB_PneuSoRD files for use with the myRIO. An optional Arduino Due shield can also be made to adapt the PneuSoRD to an Arduino Due mirco controller.

<!-- TODO Add more information regarding build - brief summary of costs
Cost per unit for 20 units \~$160 -->

<!-- TODO Check these docs -->
https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/PCB_PneuSoRD/PCB_IC/PneuSoRD-BOM.xlsx
https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/PCB_DueAdapter/PCB_DueAdapterL2/Due-BOM.xlsx


# Quickstart Guide

The following guide is for the example [3 Way 2 Position](LabVIEW/3Way_2Position/OnOffControl_32Valve.vi) as seen in [On/Off 3/2](##On/Off-3/2).

### Step 1 - Pneumatic Connection
Connect the components as seen in Fig [On/Off 3/2](##On/Off-3/2). The specific fittings and components will vary depending on the valves and sensors used in your setup. 

### Step 2 - Pump/Valve Connections
Connect the two leads of the DC pump/motor to the Motor +- screw terminals.

For the 3/2 example connect the two leads of the valve to the +- of the desired channel, the example code utilises SO1, however this can be changed.

Connect the wall wart or power supply to the V2 power input of the driver via the DC barrel jack or the screw terminals. This supply should be selected based on the operating voltage of the motor/valves used. If these are the same, only one supply is needed, if they are different a second power supply can be connected to the V1 DC barrel jack or screw terminals. 

Using a pin jumper connect each devices controller to the appropriate power supply for that device. The right pin is used for V1 and the left for V2.

**NOTE:** rev. 0.2 boards (unmarked revision) require the primary voltage input to be 7-12V. The secondary input can be used 7-25V.

### Step 3 - Sensor Input
Connect the analogue sensor connectors to the headers marked AIX on the PCB.
For the example 3/2 on/off AI1 Pump, AI2 Actuator.

### Step 4 - LabVIEW Configuration
For each device configure the correct myRIO port.

Double clicking on the IO block for each device allows the user to configure the pin which the block reads/writes to and from. Set the pump, valve and sensors to use the myRIO pins which collorate to the driver pins. See tables [MXP A](#MXP-A) and [MXP B](#MXP-B) for the correct pin.


<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control32OnOffPump.png" alt="LabView Control32OnOffPump interface" width="400">
</p>

<p align="center">3/2 on-off pump configuration</p>

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control32OnOffValve.png" alt="LabView Control32OnOffValve interface" width="400">
</p>

<p align="center">3/2 on-off valve configuration</p>

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control32OnOffSensor.png" alt="LabView Control32OnOffSensor interface" width="400">
</p>

<p align="center">3/2 on-off sensor</p>

### Step 5 - Running the LabVIEW vi
Once the configuration is complete, the system is ready to test. Use the *run* and *Abort Execution* buttons to control the device. 

The "Setpoint Receiver" slider allows the resivour pressure (kPa) to be adjusted. The limit for this pressure is dependent on the resivour and pump used.

The second slider "Setpoint Actuator" adjusts the desired actuator pressure. 

For each slider there is an associated "Performance" guage. The *red* needle displays the setpoint direct from the slider and the *black* needle displays the measured pressure.

<!-- TODO Talk about the control view -->
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control32OnOff.png" alt="3/2 on-off valve system with bang-bang controller" width="600">
</p>

<!-- TODO Talk about the interface view -->
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Interface32OnOff.png" alt="LabView control interface 3/2 On/Off" width="600">
</p>


Table of contents
=================
<!-- TODO Update TOC -->
<!--ts-->
   * [Overview](#Overview)
   * [Quickstart Guide](#Quickstart-Guide)
   * [PneuSoRD Driver](#PneuSoRD-Driver)
   * [Pnuematic Overview](#Pnuematic-Overview)
	 * [Pump/Compressor ](#Pump/Compressor )
	 * [3/2 Valve Configuration](#3/2-Valve-Configuration)
	 * [3/3 Valve Configuration](#3/3-Valve-Configuration)
	 * [2x 2/2 Valve Configuration](#2x-2/2-Valve-Configuration)
   * [LabVIEW Overview](#LabVIEW-Overview)
	 * [Main Loop](#Main-Loop)
	 * [Analog Input](#Analog-Input)
	 * [PWM](#PWM)
	 * [Simulated Squarewave](#Simulated-Squarewave)
	 * [Digital Write](#Digital-Write)
	 * [PID Block](#PID-Block)
	 * [Relay Block](#Relay-Block)
	 * [Rate Limiter Block](#Rate-Limiter-Block)
   * [Control Implementations](#Control-Implementations)
	 * [Pump Control](#Pump-Control)
	 * [On/Off 3/2](#On/Off-3/2)
		 * [Bang-Bang](#Bang-Bang)
		 * [PID](#PID)
	 * [On/Off 2x 2/2](##On/Off-2x-2/2)
		 * [Bang-Bang](#Bang-Bang)
		 * [PID](#PID)
	 * [Proportional 3/2](#Proportional-3/2)
	 * [Proportional 2x 2/2](#Proportional-2x-2/2)
   * [Hardware Overview](#Hardware-Overview)
	 * [Electrical Design (PneuSoRD)](#Electrical-Design-(PneuSoRD))
		 * [Motor/Proportional Drive](#Motor/Proportional-Drive)
		 * [On/Off Drive](#On/Off-Drive)
		 * [Sensor Input](#Sensor-Input)
	 *  [Arduino Sheild](#Arduino-Shield)
	 * [Pin Mapping](#Pin-Mapping)
		 * [MXP A](#MXP-A)
		 * [MXP B](#MXP-B)
<!--te-->

# Pnuematic Overview
An overview of pneumatic configurations.

## Pump/Compressor 
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-pump.png" alt="Air compressor with reservoir and pressure sensor" width="300">
</p>

The main components of a compressed air system are the pump or source, an optional accumulator and an inline pressure sensor. The addition of an air receiver (storage reservoir or gas tank) to the pneumatic system smooths pulsating flow and prevents excessively temporary pressure drop during sudden short-term demand. Moreover, the air receiver allows for reduced energy consumption and fast pressurization of soft actuators.

## 3/2 Valve Configuration
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-32.png" alt="Pneumatic system with a 3/2 valve coniguration" width="300">
</p>

For the 3/2 valve system as shown above, the inlet port is connected to the compressor or the receiver, the outlet port is connected to the soft actuator and the exhaust port is open to atmosphere. The 3/2 valve system implies constantly inflating and deflating each pneumatic chamber, which causes the pressure in the actuator to continuously oscillate around its target and potentially reduces the lifetime of the valves.

## 3/3 Valve Configuration
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-33.png" alt="Pneumatic system with a 3/3, 4/3 or 5/3 valve coniguration." width="300">
</p>

For the 3/3, 4/3 or 5/3 valve system as shown above, the inlet port is connected to the compressor or the receiver, the outlet port is connected to the soft actuator and the exhaust port is open to atmosphere. These valve systems have both inflating, deflating and holding modes.

## 2x 2/2 Valve Configuration
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-22.png" alt="Pneumatic system with 2x 2/2 valve coniguration" width="300">
</p>

For the system with two 2/2 valves, the first valve is responsible for the charging process and has its inlet port connected to the compressor or the receiver and outlet port connected to the soft actuator. The second valve is responsible for the discharging process and has its inlet port connected to the soft actuator and outlet port open to atmosphere. Proportional valves outperform on-off solenoid valves in regards to tracking precision and steady-state accuracy but are 3-4 times more expensive.

# LabVIEW Overview
LabVIEW offers a graphical programming approach that helps you visualize every aspect of your application, including hardware configuration, measurement data, and debugging. This visualization makes it simple to design and develop custom engineering user interfaces.

Below is an overview of some of the major LabVIEW blocks used for a the outlined control configurations for the PneuSoRD.

## Installed packages

Download the 'LabVIEW 2019 myRIO Software Bundle' https://www.ni.com/en-au/support/downloads/software-products/download.labview-myrio-software-bundle.html and install these optional dependencies
-	JKI VI Package Manager
-	LabVIEW
-	LabVIEW myRIO Toolkit
-	LabVIEW Real-Time Module
-	LabVIEW Control Design and Simulation Module
-	LabVIEW MathScript Module
-	LabVIEW Robotics Module for LabRIO

## Main Loop
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-TimedLoop.PNG" alt="LabView Timed Loop Block" width="600">
</p>

The Timed Loop block  forms the foundation of the control system developed. All LabVIEW blocks are placed within this loop which determines the speed at which the control system runs. Whilst it is possible to replace this block with a While Loop, a fixed period is preferred. This block contains a parameter for setting the loop frequency or period.

## Analog Input
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-AnalogInput.png" alt="LabView Analog Input Block" width="600">
</p>

This block reads the analog value of a single user predefined pin. This block contains no input functionality for use in the LabVIEW environment. The block outputs a 12bit analog value.

## PWM
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-PWM.png" alt="LabView PWM Block" width="600">
</p>

This block generates a hardware PWM signal to a single user predefined pin. The inputs to this block are the PWM duty cycle and PWM frequency. This block contains no output functionality for use in the LabVIEW environment.


## Simulated Squarewave
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-SimulatedSquarewave.png" alt="LabView Simulated Squarewave Block" width="600">
</p>

This block generates a wave function defined by the user. For this application a squarewave has been selected to mimic a PWM signal. The inputs to this block are the amplitude of the signal (for PWM generation this is 1), PWM duty cycle and PWM frequency. This block outputs the state of the Simulated wave at each loop itteration. 

## Digital Write
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-DigitalWrite.png" alt="LabView Digital Write Block" width="600">
</p>

This block allows the user to set the state of a single user predefined pin. The input to this block is the binary pin state. This block contains no output functionality for use in the LabVIEW environment. When used in conjunction with the Simulated Squarewave block this can be used to generate a software PWM signal.

## PID Block
<!-- TODO Add this image -->
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-PIDBlock.png" alt="LabView PID Block" width="600">
</p>

This block implements a standard PID controller with P gain, I gain and D gain controls. The input to this block is the driving error signal. The output for this block is the driving control signal.

## Relay Block
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-RelayBlock.png" alt="LabView Relay Block" width="600">
</p>

This block can be used to create a hysteresis window with two states. The inputs to this block are the maximum and minimum crossing point for the window and the driving error signal. The output to this block is the binary state of the signal.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-Relay3Block.png" alt="LabView 3 State Relay Block" width="600">
</p>

When combined with a second relay block, a three state hysteresis window can be implemented. The block inputs are the maximum, minimum and middle crossing point for the windows and the driving error signal. The output to this block is a pair of binary states for each window.

## Rate Limiter Block
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-RateLimitBlock.png" alt="LabView Rate Limiter Block" width="600">
</p>

This block limits the rate of change over time of the input signal. The input to this block is the signal to be rate limited. The output to this block is the rate limited signal.

# Control Implementations
The following section decribes a control implementation for each valve type. Each of these control implementations are available in the repository above.

## Pump Control

The accumulator pressure requires closed-loop control to provide a constant source pressure. This is achieved by controlling the motor voltage by regulating the duty cycle using a PID controller.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-pump.png" alt="Motor driver control scheme" width="400"> 
</p>


## On/Off 3/2
### Bang-Bang

For the system with a 3/2 on-off valve, the bang-bang controller is implemented with two states, which are:

<p align="center">
P < P_ref - h  
P > P_ref + h 
</p>

where **P** is the measured pressure, **P_ref** is the reference pressure and **h** is half the size of the hysteresis band. Lower values of hysteresis result in less steady-state error but also higher switching frequency. These conditions are implemented by using a relay block in Simulink or a relay function in a subsystem in LabVIEW with "switch on point" equal to **h** and "switch off point" equal to **-h**. Note that hysteresis is introduced for the bang-bang controllers to prevent excessive switching, which is particularly beneficial for the systems with 2/2 valves.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32hys.png" alt="3/2 on-off valve system with bang-bang controller" width="400"> 
</p>

### PID

For the system with a 3/2 on-off valve, the PID controller is implemented by regulating the duty cycle of the PWM wave into the valve between zero and 100%, which is used to switch the valve continuously between two states at a fixed frequency, e.g. 40Hz.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32pwm.png" alt="3/2 on-off valve system with PID controller" width="400"> 
</p>

## On/Off 2x 2/2
For the system with two 2/2 on-off valves, the first valve (control **u_1**) is used for charging by allowing air flow from the receiver into the actuator. The second valve (control **u_2**) is used for discharging by allowing flow from the actuator into atmosphere.
### Bang-Bang

The bang-bang controller, three states can be used. In the first state, valve 1 is used for charging the actuator while valve 2 is blocked, i.e. **u_1 = 1** and **u_2 = 0**. In the second state, both valves are blocked and no flow is allowed from the receiver or into the atmosphere, i.e. **u_1 = 0** and **u_2 = 0**. Finally, in the third state, valve 2 is used to discharge the actuator into the atmosphere while valve 1 is blocked, i.e. **u_1 = 0** and **u_2 = 1**. These conditions can be implemented using a case structure (state machine) with three states or by considering the logic below with two relays.

Valve 1

* Switch on: P_ref - P > h

* Switch off: P_ref - P = 0

Valve 2

* Switch on: P_ref - P < -h

* Switch off: P_ref - P = 0

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22hys.png" alt="2x 2/2 on-off valve system with bang-bang controller" width="400"> 
</p>


### PID

The PID controller is implemented by regulating the duty cycle of complementary PWM signals at a fixed frequency.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22pwm.png" alt="2x 2/2 on-off valve system with PID controller" width="400"> 
</p>

## Proportional 2x 2/2

For 2/2 proportional valves, the mapping from input voltage to output flow is approximately linear over the operating range. A small deadband can be implemented for each valve around the zero error point. Outside of this region a linear mapping between error and duty cycle can be produced to regulate the pressure in the system. Alternatively, the use of a bang-bang controller would not be recommended, as this would effectively be treating the proportional valve as a two state on/off valve.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22prop.png" alt="Proportional PWM driver with PID controller" width="400"> 
</p>

# Hardware Overview
The driver allows for the control of up to 31 valves for use in a multitude of applications. The number of sensor inputs is limited according the maximum number of analog inputs of myRIO. Consequently, 8 sensors input headers can be used to perform independent feedback control on 8 chambers. Note that more chambers can be actuated if they are allowed to share the same pressure values. 
An electrical PWM is used to drivethe proportional valves through a buck converter, which produces the required DC voltage. Where as a pneumatic PWM is produces by switching on and off the supply to the On/Off valves to produce the required driving signal. The PneuSoRD has two power inputs each of which can supply an operating voltage 7-25V. Two inputs were chosen to maximise the range or valves that could be driven. The primary input is used to power the 5V regulator and is required for the board's protection functionality. Whilst the secondary input is only required if a second voltage level is needed.

**NOTE:** rev. 0.2 boards (unmarked revision) require the primary voltage input to be 7-12V. The secondary input can be used 7-25V.

As seen in the image below, the driver interfaces with the myRIO via the two IDC connectors on the left hand side of the myRIO. 
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/MyRIOSideView.png" alt="MyRIO Side View" width="600">
</p>

## Electrical Design (PneuSoRD)
Each module has been designed to maximise the variety of devices which can be driven. The following shows the implementation of the three drive designs and the feedback sensor electronics.  
<!-- <div class="text-red mb-2"> -->
Red - 26 On/Off Drives  
<!-- </div> -->
<!-- <div class="text-yellow mb-2"> -->
Yellow - 5 Proportional Drives
<!-- </div>   -->
<!-- <div class="text-blue mb-2"> -->
Blue - 1 Motor Drive  
<!-- </div> -->
<!-- <div class="text-green mb-2"> -->
Green - Power Supply  
<!-- </div> -->
<!-- <div class="text-orange mb-2"> -->
Orange - Sensor Inputs  
<!-- </div> -->
<!-- <div class="text-pink mb-2"> -->
Pink - Expansion Headers  
<!-- </div> -->

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/MyRIOTopViewBoxes.png" alt="MyRIO Top View" width="600">
</p>

### Motor/Proportional Drive
The PneuSoRD has 6 Proportional Drive modules. The motor driver is sized for a peak current up to 1.4A. This driver is designed for DC motor pumps that typically require a current of 500mA to 1A. This output could also be used to drive a logic signal for a larger pump. The 5 proportional drivers are sized for a peak current capacity up to 700mA. This driver is designed for driving proportional valves that typically require 200mA to 500mA.

The motor and proportional valve drivers are based off a synchronous rectifier buck converter. The switch is driven by a 40-200kHz PWM signal with a variable duty cycle, which can be generated by the myRIO or Arduino Due controllers. The switch (DRV88703.6A) is a half-bridge motor driver and was selected for the comprehensive overload protection, which is described in Section .

Each circuit channel can be supplied by either of two power supply inputs via a jumper on each circuit channel. The maximum output current is set by the **Rsen** resistor, which provides over-current and short-circuit protection. In the default configuration, the motor driver channel can supply 1.4A continuously and 2.5A peak, which is adjustable up to 3.6A. The remaining 5 proportional driver channels can supply 500mA continuously and 

### On/Off Drive
Since on-off valves can be driven by a digital signal, the PneuSoRD uses 26 digital pins on the myRIO and Arduino Due. The TPS1H000 is a fully protected single channel high-side power switch with an integrated power transistor. Each drive contains an adjustable current limit that can be adjusted via the **Rsen** resistor. This protection feature limits the inrush or overload current. The trip delay capacitor has been selected to minimise the trip time if a fault event occurs. A light emitting diode is connected to the fault pin, which provides user feedback during a fault condition.

Each drive channel can be supplied by either of the power supply inputs via a jumper on each channel. The on-off channels can supply 500mA continuously, with a peak of 500mA, which is adjustable up to 1A.

### Sensor Input
A total of 8 sensor input headers are included on the PneuSoRD, with an additional 4 included on the Arduino Due shield. A linear regulator provides each sensor with 5V, this reduces sensor noises caused by noise on the power rail due to the switching of the other components. Primarily, this input is designed for 0-5V sensors that can be read via the ADC input of the microcontroller. An optional resistor can be added to suit 4mA to 20mA style sensors.

## Arduino Shield
<!-- TODO add diagram of shield and overview -->
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/DueAdapter.png" alt="Arduino Shield Render" width="600">
</p>

## Pin Mapping
The pin mapping for the PneuSoRD integrates the two MXP connectors of the MyRIO. The table below presents the pin mapping as seen in the orientation of the PneuSoRD driver. This is also the orientation of the pins of the breakout header.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/PneuSoRDConnector.png" alt="MyRIO MXP Connector" width="600">
</p>

### MXP A
| MyRIO Pin | MyRio Function | PneuSoRD Pin | MyRIO Pin | MyRio Function | PneuSoRD Pin | 
| ------ | ------ | ------ | ------ | ------ | ------ |
| MXP A Pin 34 | DIO15/I2C.SDA | SO26 | MXP A Pin 33 | +3.3V         | 3V3  |
| MXP A Pin 32 | DIO14/I2C.SCL | SO25 | MXP A Pin 31 | DIO10/PWM2    | P5   |
| MXP A Pin 30 | DGND          | GND  | MXP A Pin 29 | DIO9/PWM1     | P4   |
| MXP A Pin 28 | DGND          | GND  | MXP A Pin 27 | DIO8/PWM0     | P3   |
| MXP A Pin 26 | DIO13         | SO24 | MXP A Pin 25 | DIO7/SPI.MOSI | SO23 |
| MXP A Pin 24 | DGND          | GND  | MXP A Pin 23 | DIO6/SPI.MISO | SO22 |
| MXP A Pin 22 | DIO12/ENC.B   | SO21 | MXP A Pin 21 | DIO5/SPI.CLK  | SO20 |
| MXP A Pin 20 | DGND          | GND  | MXP A Pin 19 | DIO4          | SO19 |
| MXP A Pin 18 | DIO11/ENC.A   | SO18 | MXP A Pin 17 | DIO3          | SO17 |
| MXP A Pin 16 | DGND          | GND  | MXP A Pin 15 | DIO2          | SO16 |
| MXP A Pin 14 | UART.TX       | NC   | MXP A Pin 13 | DIO1          | SO15 |
| MXP A Pin 12 | DGND          | GND  | MXP A Pin 11 | DIO0          | SO14 |
| MXP A Pin 10 | UART.RX       | NC   | MXP A Pin 9  | AI3           | AI8  |
| MXP A Pin 8  | DGND          | GND  | MXP A Pin 7  | AI2           | AI7  |
| MXP A Pin 6  | AGND          | NC   | MXP A Pin 5  | AI1           | AI6  |
| MXP A Pin 4  | AO1           | NC   | MXP A Pin 3  | AI0           | AI5  |
| MXP A Pin 2  | AO0           | NC   | MXP A Pin 1  | +5V           | 5V   |

### MXP B
| MyRIO Pin | MyRio Function | PneuSoRD Pin | MyRIO Pin | MyRio Function | PneuSoRD Pin | 
| ------ | ------ | ------ | ------ | ------ | ------ |
| MXP B Pin 34 | DIO15/I2C.SDA | SO13 | MXP B Pin 33 | +3.3V         | 3V3  |
| MXP B Pin 32 | DIO14/I2C.SCL | SO12 | MXP B Pin 31 | DIO10/PWM2    | P2   |
| MXP B Pin 30 | DGND          | GND  | MXP B Pin 29 | DIO9/PWM1     | P1   |
| MXP B Pin 28 | DGND          | GND  | MXP B Pin 27 | DIO8/PWM0     | MTR  |
| MXP B Pin 26 | DIO13         | SO11 | MXP B Pin 25 | DIO7/SPI.MOSI | SO10 |
| MXP B Pin 24 | DGND          | GND  | MXP B Pin 23 | DIO6/SPI.MISO | SO09 |
| MXP B Pin 22 | DIO12/ENC.B   | SO08 | MXP B Pin 21 | DIO5/SPI.CLK  | SO07 |
| MXP B Pin 20 | DGND          | GND  | MXP B Pin 19 | DIO4          | SO06 |
| MXP B Pin 18 | DIO11/ENC.A   | SO05 | MXP B Pin 17 | DIO3          | SO04 |
| MXP B Pin 16 | DGND          | GND  | MXP B Pin 15 | DIO2          | SO03 |
| MXP B Pin 14 | UART.TX       | NC   | MXP B Pin 13 | DIO1          | SO02 |
| MXP B Pin 12 | DGND          | GND  | MXP B Pin 11 | DIO0          | SO01 |
| MXP B Pin 10 | UART.RX       | NC   | MXP B Pin 9  | AI3           | AI4  |
| MXP B Pin 8  | DGND          | GND  | MXP B Pin 7  | AI2           | AI3  |
| MXP B Pin 6  | AGND          | NC   | MXP B Pin 5  | AI1           | AI2  |
| MXP B Pin 4  | AO1           | NC   | MXP B Pin 3  | AI0           | AI1  |
| MXP B Pin 2  | AO0           | NC   | MXP B Pin 1  | +5V           | 5V   |

### Arduino Due Shield
| MCU Pin | Arduino Pin | PneuSoRD Pin |
| ------ | ------ | ------ |
| PA16 | A0  | A1    |
| PA24 | A1  | A2    |
| PA23 | A2  | A3    |
| PA22 | A3  | A4    |
| PA6  | A4  | A5    |
| PA4  | A5  | A6    |
| PA3  | A6  | A7    |
| PA2  | A7  | A8    |
|      |     |       |
| PB17 | A8  | \*A9  |
| PB18 | A9  | \*A10 |
| PB19 | A10 | \*A11 |
| PB20 | A11 | \*A12 |
| PB15 |     | NC    |
| PB16 |     | NC    |
| PA1  |     | NC    |
| PA0  |     | NC    |

\*Items on adapter PCB

| MCU Pin | Arduino Pin | PneuSoRD Pin | MCU Pin | Arduino Pin | PneuSoRD Pin |
| ------ | ------ | ------ | ------ | ------ | ------ |
| +5V  | +5V | NC   | +5V  | +5V | NC   |
| PB26 | D22 | SO1  | PA14 | D23 | SO2  |
| PA15 | D24 | SO3  | PD0  | D25 | SO4  |
| PD1  | D26 | SO5  | PD2  | D27 | SO6  |
| PD3  | D28 | SO7  | PD6  | D29 | SO8  |
| PD9  | D30 | SO9  | PA7  | D31 | SO10 |
| PD10 | D32 | SO11 | PC1  | D33 | SO12 |
| PC2  | D34 | SO13 | PC3  | D35 | P1   |
| PC4  | D36 | SO14 | PC5  | D37 | P2   |
| PC6  | D38 | SO15 | PC7  | D39 | P3   |
| PC8  | D40 | SO16 | PC9  | D41 | P4   |
| PA19 | D42 | P5   | PA20 | D43 | SO17 |
| PC19 | D44 | SO18 | PC18 | D45 | MTR  |
| PC17 | D46 | SO19 | PC16 | D47 | SO20 |
| PC15 | D48 | SO21 | PC14 | D49 | SO22 |
| PC13 | D50 | SO23 | PC12 | D51 | SO24 |
| PB21 | D52 | SO25 | PB14 | D53 | SO26 |
| GND  | GND | GND  | GND  | GND | GND  |

# Useful Links
Arduino PID Library - https://playground.arduino.cc/Code/PIDLibrary/
