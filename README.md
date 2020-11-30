# Pneumatic Overview
![Overview of the Pneumatic Soft Robotics Controller (PneuSoRD), including the electronics driver board and National Instruments myRIO controller, which is connected to a single pump and air receiver, up to 5 proportional valves, and 26 on-off valves. Soft robotic glove reproduced with permission from [35]. Copyright 2019, Mary Ann Liebert, Inc.](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/OverviewDiagram.png | width=100)

![Overview of the Pneumatic Soft Robotics Controller (PneuSoRD), including the electronics driver board and National Instruments myRIO controller, which is connected to a single pump and air receiver, up to 5 proportional valves, and 26 on-off valves. Soft robotic glove reproduced with permission from [35]. Copyright 2019, Mary Ann Liebert, Inc.](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/OverviewDiagram.png =250x250)

![Overview of the Pneumatic Soft Robotics Controller (PneuSoRD), including the electronics driver board and National Instruments myRIO controller, which is connected to a single pump and air receiver, up to 5 proportional valves, and 26 on-off valves. Soft robotic glove reproduced with permission from [35]. Copyright 2019, Mary Ann Liebert, Inc.](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/OverviewDiagram.png)

The Pneumatic Soft Robotics Driver (PneuSoRD) as shown in Fig. 2 is an open-source power electronics design for controlling pneumatic soft robotic actuators with a large number of simultaneous actuated chambers. This system is designed around the two major categories of valve types. On/Off valves Fig. 1a-c with single or multiple solenoids which each take a binary input and proportional valves Fig. 1d which can be driven from a variable DC voltage. Therefore, the PneuSoRD design encompasses a modular solution for each drive system: The "Proportional Drive" and the "On/Off Drive". Each module has been designed to be as generic as possible to allow control of a large range of valves with an available control strategy for each. These two modules are driven by a myRIO 1950 embedded controller through a real-time visual user interface with LabVIEW.

## Pump/Compressor
TODO Add figure with compressor, reservoir and pressure sensor.
The main components of a compressed air system are the pump or source, an optional accumulator and an inline pressure sensor. The addition of an air receiver (storage reservoir or gas tank) to the pneumatic system smooths pulsating flow and prevents excessively temporary pressure drop during sudden short-term demand. Moreover, the air receiver allows for reduced energy consumption and fast pressurization of soft actuators.

## 3/2 Valve Configuration
![Pneumatic system with a 3/2 valve coniguration](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-32.png)

## 3/3 Valve Configuration
![Pneumatic system with a 3/3, 4/3 or 5/3 valve coniguration.](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-33.png)

## 2x 2/2 Valve Configuration
![Pneumatic system with 2x 2/2 valve coniguration](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/pneumaticdiagrams-22.png)

## Design Overview
The table below provides a comparison of pneumatic systems for soft robotic applications. Accuracy refers to steady-state tracking and control refers to the difficulty level in the controller design and hardware requirements. Systems with 3/2 valves Fig. 1a are the most widely used due to their low price and easy implementation, especially considering their use in the fluidic control board. Systems with 2/2 valves Fig. 1c offer the advantages of reduced energy consumption and oscillation, which increases the lifetime of the valves. However, these systems are more difficult to control and are more expensive since two valves are required. Proportional valves allow for tracking precision at the expense of size and cost.

| Setup    | Cost   | Lifetime  | Accuracy | Control | 
| ------ | ------ | ------ | ------ | ------ |
| 3/2 on-off valve | Low | Low | Medium | Low |
| 2/2 on-off valves | Medium | Medium | Medium | High |
| 3/2 prop. valve | High | High | High | Low |
| 2/2 prop. valves | High | High | High | High |

# LabView Overview
-	Different blocks functions

### PWM
TODO
![LabView PWM Block](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-PWM.png)

### Digital Write
TODO
![LabView Digital Write Block](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-DigitalWrite.png)

### Simulated Squarewave
TODO
![LabView Simulated Squarewave Block](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-SimulatedSquarewave.png)

### Analog Input
TODO
![LabView Analog Input Block](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-AnalogInput.png)

### PID Block
TODO
![LabView PID Block](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-PIDBlock.png)

### Relay Block
TODO
![LabView Relay Block](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/LabViewBlockExample-RelayBlock.png)

# Pump Control
![Motor driver control scheme](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-pump.png)

# On/Off 3/2
![3/2 on-off valve system with bang-bang controller](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32hys.png)
![3/2 on-off valve system with PID controller](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32pwm.png)

# On/Off 2x 2/2
![2x 2/2 on-off valve system with bang-bang controller](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22hys.png)
![2x 2/2 on-off valve system with PID controller](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22pwm.png)

# Proportional 3/2
![Proportional PWM driver with hysteresis controller](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-32prop.png)

# Proportional 2x 2/2
![Proportional PWM driver with PID controller](https://github.com/PrecisionMechatronicsLab/PneuSoRD/blob/main/figures/ControlSchemes-22prop.png)

