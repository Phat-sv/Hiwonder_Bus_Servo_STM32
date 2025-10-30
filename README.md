**This driver provides STM32 HAL-based communication and control for Hiwonder Bus Servos, which operate using an UART serial protocol (115200 baud).**

Unlike traditional RC servos that only accept position commands, Hiwonder bus servos ***support half duplex communication, allowing both control and feedback.***

Each servo can be configged with an ID, and the communication between microcontroller and servos bases on this ID.

*I used these bus servo for my robot arm, [see here](https://www.youtube.com/watch?v=gQMT6rC0Xig)*

Servo can also be configged many parameters:
* Servo mode or motor mode
* Speed
* Range of angle (0 - 240 degrees)
* Voltage and Temperature
* Led signal in case of overload or overtemperature,...

**Check these files for more information:** 
* [Protocol]() and [Frame format]()
* [User manual]()





