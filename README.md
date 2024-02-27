# Embedded-system-Walking-machine

Code developed for "A. Marino, - Programming and construction of a scale walking machine. ".  
Master's student "Calabria University".  
For any questions or suggestions write to alexismarino0109@gmail.com


# Sumary.
The project entails developing a scale system designed to emulate the operations of an electric walking machine, which is fully based on Embedded systems programming. Its core functionality revolves around a conveyor belt capable of adjusting velocities per user specifications, a stepper motor for configuring the conveyor belt's inclination, and strategically positioned input/output interfaces that facilitate model control.

At the heart of this system is the microcontroller STM32F411RET6, featuring an ARM Cortex-M architecture meticulously suited for embedded devices characterized by low power consumption. This choice ensures optimal performance and efficiency in line with the project's embedded systems focus.

the system is completely programmed using REGISTERS.

# Objectives   
The main objectives are:
-	Generate controllers that allow us to configure the velocity.
-	Generate an analog input to set up the inclination of the conveyor belt.
-	Configurate a 16x2 LCD to show instructions and values.
-	Configure the DC gear motor and servo motor for the conveyor belt and inclination respectively.  
-	Control the velocity of the conveyor belt with a controller. 
-	Generate a code capable of following the logic presented in the figure.
![image](https://github.com/fercho-0109/Embedded-system-Walking-machine/assets/40362695/ebde29f8-2838-4d8d-a130-a5d584483453)

## Detailed list of the materials used
STM32 Nucleo-STM32F411RET  
Mini-USB <-> USB cable  
Breadboard  
Cables  
Resistors  
Capacitors  
Buttons  
LED  
Optical encoder   
Rotary Potentiometer  
L293D h-bridge  
DC motor   
Step motor
## Software and Development Environment
[[STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html#tools-software)]
## References
[[Nucleo 64 - User Manual](https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo-64-boards-mb1136-stmicroelectronics.pdf)]  
[[STM32F411RET6 Datasheet](https://www.alldatasheet.com/datasheet-pdf/pdf/1009260/STMICROELECTRONICS/STM32F411RET6.html)]
[[STM32F411RET6 reference manual](https://www.micro-semiconductor.com/datasheet/02-STM32F411RET6.pdf)]

## Conections 
The main connections are shown in the figure below while the connections of the other elements are detailed in the report.


# File description
The repository contains three files
1. **MAIN**: This Matlab file contains the configuration parameters of the program and shows the poles of the plant after control.
2. **CD_MOTORControl2**: This simulink file contains the complete simulation of the CNC machine with its reference generator.
3. **Report**: This contains the explanation of the complete process, the mathematical formulations, and the control configuration.


# Example to run the experiment  
**"CNC cutting machine"**
### Matlab/Simulink simulation 
1. Download the files. 
2. Run the Matlab file "**MAIN**".
3. Open and run the Simulink file "**CD_MOTORControl2**"
4. The XY Graph block should start to show the results  
![image](https://github.com/fercho-0109/CNC-2D-cutting-machine-Control/assets/40362695/fdf0f90a-b552-409e-ade6-690a95bdbeeb)



