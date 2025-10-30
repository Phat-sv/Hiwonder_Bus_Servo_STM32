**This driver provides STM32 HAL-based communication and control for Hiwonder Bus Servos, which operate using an UART serial protocol (115200 baud).**

Unlike traditional RC servos that only accept position commands, Hiwonder bus servos ***support half duplex communication, allowing both control and feedback.***

Each servo can be configged with an ID, and the communication between microcontroller and servos bases on this ID.

*I used these bus servo for **My robot arm**, [see here](https://www.youtube.com/watch?v=gQMT6rC0Xig)*

Servo can also be configged many parameters:
* Servo mode or motor mode
* Speed
* Range of angle (0 - 240 degrees)
* Voltage and Temperature
* Led signal in case of overload or overtemperature,...

**Check these files for more information:** 
* [Protocol](https://github.com/Phat-sv/Hiwonder_Bus_Servo_STM32/blob/main/Documents/Hiwonder%20Bus%20Servo%20Communication%20Protocol.pdf) and [Frame format](https://github.com/Phat-sv/Hiwonder_Bus_Servo_STM32/blob/main/Documents/Frame%20format.png)
* [User manual](https://github.com/Phat-sv/Hiwonder_Bus_Servo_STM32/blob/main/Documents/LX-15D%20Bus%20Servo%20User%20Manual.pdf)





