# BeanControl - work in progress

A basic temperature control system for the Gaggia Classic.  This is never intended to be a polished, ready-to-deploy piece of software (although you are welcome to use it as you wish!), but rather a reference for others who are rolling their own controller.  I'm aiming for simplicity, both in terms of code and hardware, and the ability to fiddle around with different control algorithms.  

Hardware:
* Arduino Leonardo
* Fotek SSR
* 128x64 OLED
* PT100 temperature sensor, with offboard amplifier circuitry
* rotary encoder for setpoint adjustment

Control algorithm will be a simple PI.  Possibly in future will look into the shortest time LQ problem for the heat equation, to get shortest heat-up time.  Steam switch is connected directly to the Arduino and simply changes the setpoint.  Also includes a button-actuated shot timer.

Other projects of interest include [Gagguino](https://github.com/Zer0-bit/gaggiuino) and [Espressuino](https://github.com/basahun/Espressuino).
