# FOC2 Development repository for Smart Brushless Servo 

PID/FOC Custom Firmware for B-G431B-ESC1 board.

Follow us on Hackaday.io : https://hackaday.io/project/177578-b-g431b-esc-brushless-servo-controller

## Characteristics

- [x] 16KHz motor PWM 
- [x] Open-loop and closed-loop FOC algorithm (16KHz)
- [x] FOC with CSVPWM algorithm 
- [x] Torque and Flux control (P only), with current limiter
- [x] Position control with min/max position limiter (4KHz)
- [x] Velocity control with max velocity limiter (4KHz) 
- [x] Torque and Flux feed-forward
- [x] Auto-calibration of electrical angle and motor rotation 
- [x] USB control interface : (dynamixel like protocol) allowing full-access to control (RAM) and configuration (EERPOM) registers through a GUI tool.
- [x] CAN control interface : variable-size control frames (RX) and feedback frames (TX), with fail-safe on bus time-out
- [x] Power supply voltage monitoring, with min/max operating voltage (user configurable)
- [x] Position sensor monitoring, with fail-safe on system error and bus time-out
- [x] Temperature monitoring, with fail-safe on maximum temprature (user configurable) 
- [ ] Field-Weakening (work in progress)

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
* J5/J6 for Lipo battery (30V or 6S max)
* J7 for motor
* J8 for motor sensor (Hall or encoder)

![alt text](https://github.com/pat92fr/FOC2/blob/main/00-Doc/01-Wiring/CaptureSTmanual2.PNG?raw=true)

The USB interface is provided on the daughterboard and it allows to program and debug the main board. It provides also the supply voltage to the STM32G431CB MCU in case of no voltage on the bus (J5 and J6 not connected to the LiPo battery).

**For proper operation of the FOC firmware, the J1, J5, J6, J7, an J8 solder pads should be connected.**
The J2, J3, J4 solder pads may be left unconnected.

For J8, the firmware accept two types of position sensor :
* The AS5600 Position Sensor connected through I2C port (https://ams.com/as5600)
* The AS5048A High-Resolution Position Sensor connected through PWM interface (https://ams.com/as5048a)

![alt text](https://github.com/pat92fr/FOC2/blob/main/00-Doc/01-Wiring/CaptureSTmanual.PNG?raw=true)
![alt text](https://github.com/pat92fr/FOC2/blob/main/00-Doc/01-Wiring/ESCwiring%20v0.01.png?raw=true)

# Uploading firmware

A full erase of the Chip is strongly recommended, to insure proper initialisation of the default configuration (the last Flash page stores PID/FOC configuration).

The release configuration of the PID/FOC firmware should be built with CubeIDE, and then uploaded using the USB port of the board.

**A hard-coded setting must be adjusted by user, according its hardware configuration :**

At lines 40+ of **main.c**, 
```
// Position sensor type :
//    "AS5600_I2C"
//    "AS5048A_PWM"
#define SENSOR_TYPE AS5048A_PWM
```






