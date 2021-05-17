# FOC2
SmartBrushlessServo_priv


# Hardware (commercial off the shelf)

![alt text](https://github.com/pat92fr/FOC2/blob/main/00-Doc/00-Hardware/pf267025_m.jpg?raw=true)

Product page : https://www.st.com/en/evaluation-tools/b-g431b-esc1.html

Cost: approx. 18$ per unit.

# Wiring

As mentioned in the user manual, the B-G431B-ESC1 Discovery kit is equipped with a USB connector and different pads for communication, such as:
* U4 USB port for programming and debugging
* J1 for CAN port
* J2 for SWD-STM32F103 (reserved)
* J3 for PWM/UART/BECout input/output signal
* J4 for SWD-STM32G431 debug/programming port (without daughterboard)
* J8 for motor sensor (Hall or encoder)

![alt text](https://github.com/pat92fr/FOC2/blob/main/00-Doc/01-Wiring/CaptureSTmanual2.PNG?raw=true)

The USB interface is provided on the daughterboard and it allows to program and debug the main board. It provides also the supply voltage to the STM32G431CB MCU in case of no voltage on the bus (J5 and J6 not connected to the LiPo battery).

Power supply, brushless motor, position sensor and CAN bus has to be connected for proper installation.

The firmware accept two types of position sensor :
* AS5600 Position Sensor connected through I2C port (https://ams.com/as5600)
* AS5048A High-Resolution Position Sensor connected through PWM interface (https://ams.com/as5048a)

![alt text](https://github.com/pat92fr/FOC2/blob/main/00-Doc/01-Wiring/CaptureSTmanual.PNG?raw=true)
![alt text](https://github.com/pat92fr/FOC2/blob/main/00-Doc/01-Wiring/ESCwiring%20v0.01.png?raw=true)

Current firmware do not use the J3 port (PWM/UART).

