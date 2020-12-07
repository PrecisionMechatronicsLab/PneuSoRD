# Pneumatic Overview
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/OverviewDiagram.png" alt="Overview of the Pneumatic Soft Robotics Controller (PneuSoRD), including the electronics driver board and National Instruments myRIO controller, which is connected to a single pump and air receiver, up to 5 proportional valves, and 26 on-off valves. Soft robotic glove reproduced with permission from [35]. Copyright 2019, Mary Ann Liebert, Inc." width="600">
</p>



The Pneumatic Soft Robotics Driver (PneuSoRD) as shown in Fig. 2 is an open-source power electronics design for controlling pneumatic soft robotic actuators with a large number of simultaneous actuated chambers. This system is designed around the two major categories of valve types. On/Off valves Fig. 1a-c with single or multiple solenoids which each take a binary input and proportional valves Fig. 1d which can be driven from a variable DC voltage. Therefore, the PneuSoRD design encompasses a modular solution for each drive system: The "Proportional Drive" and the "On/Off Drive". Each module has been designed to be as generic as possible to allow control of a large range of valves with an available control strategy for each. These two modules are driven by a myRIO 1950 embedded controller through a real-time visual user interface with LabVIEW.

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
A solution to this problem includes the use of a single but expensive 3/3 (3-way, 3-position) or 5/3 (5-way, 3-position) solenoid valve shown in the figure above. 

## 2x 2/2 Valve Configuration
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-22.png" alt="Pneumatic system with 2x 2/2 valve coniguration" width="300">
</p>
Alternatively, a system with two 2/2 (2-way, 2-position) valves can be used. In this case, the first valve is responsible for the charging process and has its inlet port connected to the compressor or the receiver and outlet port connected to the soft actuator. The second valve is responsible for the discharging process and has its inlet port connected to the soft actuator and outlet port open to atmosphere. Proportional valves outperform on-off solenoid valves in regards to tracking precision and steady-state accuracy but are 3-4 times more expensive.

## Design Overview
The table below provides a comparison of pneumatic systems for soft robotic applications. Accuracy refers to steady-state tracking and control refers to the difficulty level in the controller design and hardware requirements. Systems with 3/2 valves Fig. 1a are the most widely used due to their low price and easy implementation, especially considering their use in the fluidic control board. Systems with 2/2 valves Fig. 1c offer the advantages of reduced energy consumption and oscillation, which increases the lifetime of the valves. However, these systems are more difficult to control and are more expensive since two valves are required. Proportional valves allow for tracking precision at the expense of size and cost.

| Setup    | Cost   | Lifetime  | Accuracy | Control | 
| ------ | ------ | ------ | ------ | ------ |
| 3/2 on-off valve | Low | Low | Medium | Low |
| 2/2 on-off valves | Medium | Medium | Medium | High |
| 3/2 prop. valve | High | High | High | Low |
| 2/2 prop. valves | High | High | High | High |

# LabVIEW Overview
LabVIEW offers a graphical programming approach that helps you visualize every aspect of your application, including hardware configuration, measurement data, and debugging. This visualization makes it simple to design and develop custom engineering user interfaces.

Below is an overview of some of the major LabVIEW blocks used for a the outlined control configurations for the PneuSoRD.

### Main Loop
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-TimedLoop.PNG" alt="LabView Timed Loop Block" width="600">
</p>
The Timed Loop block  forms the foundation of the control system developed. All LabVIEW blocks are placed within this loop which determines the speed at which the control system runs. Whilst it is possible to replace this block with a While Loop, a fixed period is preferred. This block contains a parameter for setting the loop frequency or period.

### Analog Input
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-AnalogInput.png" alt="LabView Analog Input Block" width="600">
</p>
This block reads the analog value of a single user predefined pin. This block contains no input functionality for use in the LabVIEW environment. The block outputs a 12bit analog value.

### PWM
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-PWM.png" alt="LabView PWM Block" width="600">
</p>
This block generates a hardware PWM signal to a single user predefined pin. The inputs to this block are the PWM duty cycle and PWM frequency. This block contains no output functionality for use in the LabVIEW environment.


### Simulated Squarewave
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-SimulatedSquarewave.png" alt="LabView Simulated Squarewave Block" width="600">
</p>
This block generates a wave function defined by the user. For this application a squarewave has been selected to mimic a PWM signal. The inputs to this block are the amplitude of the signal (for PWM generation this is 1), PWM duty cycle and PWM frequency. This block outputs the state of the Simulated wave at each loop itteration. 

### Digital Write
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-DigitalWrite.png" alt="LabView Digital Write Block" width="600">
</p>
This block allows the user to set the state of a single user predefined pin. The input to this block is the binary pin state. This block contains no output functionality for use in the LabVIEW environment. When used in conjunction with the Simulated Squarewave block this can be used to generate a software PWM signal.

### PID Block
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-PIDBlock.png" alt="LabView PID Block" width="600">
</p>
This block implements a standard PID controller with P gain, I gain and D gain controls. The input to this block is the driving error signal. The output for this block is the driving control signal.

### Relay Block
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-RelayBlock.png" alt="LabView Relay Block" width="600">
</p>
This block can be used to create a hysteresis window with two states. The inputs to this block are the maximum and minimum crossing point for the window and the driving error signal. The output to this block is the binary state of the signal.

<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-Relay3Block.png" alt="LabView 3 State Relay Block" width="600">
</p>
When combined with a second relay block, a three state hysteresis window can be implemented. The block inputs are the maximum, minimum and middle crossing point for the windows and the driving error signal. The output to this block is a pair of binary states for each window.

### Rate Limiter Block
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-RateLimiterBlock.png" alt="LabView Rate Limiter Block" width="600">
</p>
This block limits the rate of change over time of the input signal. The input to this block is the signal to be rate limited. The output to this block is the rate limited signal.

# Control Implementations
The following section decribes a control implementation for each valve type. Each of these control implementations are available in the repository above.

## Pump Control
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-pump.png" alt="Motor driver control scheme" width="400"> 
</p>
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/ControlPump.png" alt="Motor driver control scheme" width="600">
</p>
TODO Add side by side version of a LabVIEW file

TODO Talk about the control interface
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/InterfacePump.png" alt="LabView control interface Pump" width="600">
</p>

## On/Off 3/2
### Bang-Bang
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32hys.png" alt="3/2 on-off valve system with bang-bang controller" width="400"> 
</p>
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control32OnOff.png" alt="3/2 on-off valve system with bang-bang controller" width="600">
</p>
TODO Add side by side version of a LabVIEW file

TODO Talk about the control interface
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Interface32OnOff.png" alt="LabView control interface 3/2 On/Off" width="600">
</p>

### PID
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32pwm.png" alt="3/2 on-off valve system with PID controller" width="400"> 
</p>
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control32PID.png" alt="3/2 on-off valve system with PID controller" width="600">
</p>
TODO Add side by side version of a LabVIEW file

TODO Talk about the control interface
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Interface32PID.png" alt="LabView control interface" width="600">
</p>

## On/Off 2x 2/2
### Bang-Bang
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22hys.png" alt="2x 2/2 on-off valve system with bang-bang controller" width="400"> 
</p>
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control22OnOff.png" alt="2x 2/2 on-off valve system with bang-bang controller" width="600">
</p>
TODO Add side by side version of a LabVIEW file

TODO Talk about the control interface
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Interface22OnOff.png" alt="LabView control interface 2/2 On/Off" width="600">
</p>

### PID
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22pwm.png" alt="2x 2/2 on-off valve system with PID controller" width="400"> 
</p>
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control22PID.png" alt="2x 2/2 on-off valve system with PID controller" width="600">
</p>
TODO Add side by side version of a LabVIEW file

TODO Talk about the control interface
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Interface22PID.png" alt="LabView control interface 2/2 PID" width="600">
</p>

## Proportional 3/2
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32prop.png" alt="Proportional PWM driver with hysteresis controller" width="400"> 
</p>
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control32Prop.png" alt="Proportional PWM driver with hysteresis controller" width="600">
</p>
TODO Add side by side version of a LabVIEW file

TODO Talk about the control interface
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Interface32Prop.png" alt="LabView control interface 3/2 Proportional" width="600">
</p>

## Proportional 2x 2/2
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22prop.png" alt="Proportional PWM driver with PID controller" width="400"> 
</p>
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Control22Prop.png" alt="Proportional PWM driver with PID controller" width="600">
</p>
TODO Add side by side version of a LabVIEW file

TODO Talk about the control interface
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabVIEW/Interface22Prop.png" alt="LabView control interface 2/2 Proportional" width="600">
</p>

# Hardware Overview
The driver allows for the control of up to 31 valves for use in a multitude of applications. The number of sensor inputs is limited according the maximum number of analog inputs of myRIO. Consequently, 8 sensors input headers can be used to perform independent feedback control on 8 chambers. Note that more chambers can be actuated if they are allowed to share the same pressure values. 
An electrical PWM is used to drivethe proportional valves through a buck converter, which produces the required DC voltage. Where as a pneumatic PWM is produces by switching on and off the supply to the On/Off valves to produce the required driving signal. The PneuSoRD has two power inputs each of which can supply an operating voltage 7-25V. Two inputs were chosen to maximise the range or valves that could be driven. The primary input is used to power the 5V regulator and is required for the board's protection functionality. Whilst the secondary input is only required if a second voltage level is needed.

As seen in the image below, the driver interfaces with the myRIO via the two IDC connectors on the left hand side of the myRIO. 
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/MyRIOSideView.png" alt="MyRIO Side View" width="600">
</p>

## Electrical Design
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
The PneuSoRD has 6 Proportional Drive modules. The motor driver is sized for a peak current capacity up to 1.4A. This driver is designed for driving dc motor pumps which typically have a current range of 500mA to 1A. The 5 proportional drivers are sized for a peak current capacity up to 700mA. This driver is designed for driving proportional valves which typically have a current range of 200 to 500mA. The motor and proportional valve drivers are based off a synchronous rectifier buck converters. The buck converters switch is driven by a 40-200kHz PWM signal with a variable duty cycle direct from the myRIO. The switch is a DRV88703.6A, a half-bridge motor driver and was selected due to its variety of protection capabilities as outlined in Section III-D. Each drive can be supplied by either of the two onboard power inputs via a selectable voltage header on each module. Current regulation is achieved based on the analog input VREF and the voltage on the ISEN pin, which is proportional to motor current through an external sensing resistor. This is adjustable via the Rsen resistor and can provide a range of current limits depending on the application. The motor drive can supply a continuous 1.4A with an adjustable peak rating of 2.5A (adjustable up to 3.6 A). The remaining proportional drives can supply a continuous 500mA with an adjustable peak rating of 700mA
(adjustable up to 700mA).
TODO Add close up image of section showing wiring
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/MyRIOCloseUpProp.png" alt="MyRIO Proportional Drive" width="600">
</p>

### On/Off Drive
Since On/Off valves can be driven via a simple digital signal, the PneuSoRD takes advantage of the 26 Digital Input/Output (DIO) pins on the myRIO. The TPS1H000 is a fully protected single channel high-side power switch with an integrated power Field-Effect Transistor (FET). An adjustable current limit via the Rsen resistor improves system reliability by limiting the inrush or overload current. The high accuracy of the current limit improves overload protection. The trip delay capacitor has been selected to minimise the trip time if a fault event occurs. A light emitting diode is connected to the common collector fault pin, this provides user feedback if a fault event were to occur.
Each drive can be supplied by either of the two onboard power inputs via a selectable voltage header on each module. The On/Off drives can supply a continuous 500mA.
TODO Add close up image of section showing wiring
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/MyRIOCloseUpOnOff.png" alt="MyRIO OnOff Drive" width="600">
</p>

### Sensor Input
A total of 8 sensor input headers are included on the PneuSoRD. These are designed to provide each sensor with 5V. Primarily, this input is designed for 0-5V sensors that can be read via the ADC input of the myRIO. An optional resistor can be added to allow 4-20mA style sensors to be used.
TODO Add close up image of section showing wiring
<p align="center">
	<img src="https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/MyRIOCloseUpSensor.png" alt="MyRIO Sensors" width="600">
</p>

## Pin Mapping
The pin mapping for the PneuSoRD integrates the two MXP connectors of the MyRIO. The table below presents the pin mapping as seen in the orientation of the PneuSoRD driver. This is also the orientation of the pins of the breakout header.

### MXP A
| MyRIO Pin | PneuSoRD Pin | MyRIO Pin | PneuSoRD Pin | 
| ------ | ------ | ------ | ------ |
| MXP A Pin 34 | SO26 | MXP A Pin 33 | 3V3  |
| MXP A Pin 32 | SO25 | MXP A Pin 31 | P5   |
| MXP A Pin 30 | GND  | MXP A Pin 29 | P4   |
| MXP A Pin 28 | GND  | MXP A Pin 27 | P3   |
| MXP A Pin 26 | SO24 | MXP A Pin 25 | SO23 |
| MXP A Pin 24 | GND  | MXP A Pin 23 | SO22 |
| MXP A Pin 22 | SO21 | MXP A Pin 21 | SO20 |
| MXP A Pin 20 | GND  | MXP A Pin 19 | SO19 |
| MXP A Pin 18 | SO18 | MXP A Pin 17 | SO17 |
| MXP A Pin 16 | GND  | MXP A Pin 15 | SO16 |
| MXP A Pin 14 | GND  | MXP A Pin 13 | SO15 |
| MXP A Pin 12 | GND  | MXP A Pin 11 | SO14 |
| MXP A Pin 10 | NC   | MXP A Pin 9  | AI8  |
| MXP A Pin 8  | GND  | MXP A Pin 7  | AI7  |
| MXP A Pin 6  | NC   | MXP A Pin 5  | AI6  |
| MXP A Pin 4  | NC   | MXP A Pin 3  | AI5  |
| MXP A Pin 2  | NC   | MXP A Pin 1  | 5V   |

### MXP B
| MyRIO Pin | PneuSoRD Pin | MyRIO Pin | PneuSoRD Pin | 
| ------ | ------ | ------ | ------ |
| MXP B Pin 34 | SO13 | MXP B Pin 33 | 3V3  |
| MXP B Pin 32 | SO12 | MXP B Pin 31 | P2   |
| MXP B Pin 30 | GND  | MXP B Pin 29 | P1   |
| MXP B Pin 28 | GND  | MXP B Pin 27 | MTR  |
| MXP B Pin 26 | SO11 | MXP B Pin 25 | SO10 |
| MXP B Pin 24 | GND  | MXP B Pin 23 | SO09 |
| MXP B Pin 22 | SO08 | MXP B Pin 21 | SO07 |
| MXP B Pin 20 | GND  | MXP B Pin 19 | SO06 |
| MXP B Pin 18 | SO05 | MXP B Pin 17 | SO04 |
| MXP B Pin 16 | GND  | MXP B Pin 15 | SO03 |
| MXP B Pin 14 | GND  | MXP B Pin 13 | SO02 |
| MXP B Pin 12 | GND  | MXP B Pin 11 | SO01 |
| MXP B Pin 10 | NC   | MXP B Pin 9  | AI4  |
| MXP B Pin 8  | GND  | MXP B Pin 7  | AI3  |
| MXP B Pin 6  | NC   | MXP B Pin 5  | AI2  |
| MXP B Pin 4  | NC   | MXP B Pin 3  | AI1  |
| MXP B Pin 2  | NC   | MXP B Pin 1  | 5V   |