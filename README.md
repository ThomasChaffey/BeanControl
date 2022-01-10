# BeanControl - work in progress

A basic temperature control system for the Gaggia Classic.

Hardware:
* Arduino Leonardo
* Fotek SSR
* 128x64 OLED
* PT100 temperature sensor, with offboard amplifier circuitry
* rotary encoder for setpoint adjustment

Control algorithm will be a simple PI.  Possibly in future will look into the shortest time LQ problem for the heat equation, to get shortest heat-up time.  Steam switch is connected directly to the Arduino and simply changes the setpoint.  Also includes a button-actuated shot timer.
