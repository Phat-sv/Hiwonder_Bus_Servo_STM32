This driver using HAL library of STM32 to communicate and control Hiwonder bus servo motor.

Hiwonder bus servo motor is controlled by ***serial commands with a baud rate of 115200 through a protocol***.

**Microcontrollers can control not only position of servo but also can read feedback of their current positions, making them diffrence from RC servo type**.

Each servo can be configged with an ID, and the communication between microcontroller and servos bases on this ID.

Servo can also be configged many parameters:
* Servo mode or motor mode
* Speed
* Range of angle (0 - 240 degrees)
* Voltage and Temperature
* Led signal in case of overload or overtemperature,...

**Check these files for more information:** 
* [Protocol]()
* [User manual]()





