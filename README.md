# SERVO42C-ESP32WROOM32-UART-Library
A small library to control the makerbase servo42c via UART from an AZdelivery ESP32 Wroom32 NodeMCU. Written in cpp for vstudio with platform.io. For other ESP32 boards the platform.ini file needs to be changed to fit the board.

</br></br>
# Warning
This library does not support the v1.0 version and is written for the v1.1 version of the 42C board. It looks like all the raw driver PCBs that are sold without stepper motor are the v1.0 version while the ones with stepper motor are more likely to be the v1.1.

</br></br>
# Notes
This library uses UART2 mapped to GPIO16 as RX and GPIO0 as TX. Using UART1 on this board does not work without issues.


Flash it with vstudio code and platform.io (extension for vstudio code) 

ESP RX (GPIO16) to MKS42C TX ESP TX (GPIO0) to MKS42C RX MKS42C UART GND to the 12v GND. MKS42C UART 3.3v is not needed in this configuration.
If the ESP already has a shared gnd with the 12v PSU it is not needed to connect the GND from the UART terminal.

12v +- power input of the MKS42C are on the right of the 6 Pin connector.
