# SERVO42C-ESP32WROOM32-UART-Library
A small library to control the makerbase servo42c via UART from an AZdelivery ESP32 Wroom32 NodeMCU. Written in cpp for vstudio with platform.io. For other ESP32 boards the platform.ini file needs to be changed to fit the board.


# Notes
This library uses UART2 mapped to GPIO17 as RX and GPIO0 as TX.Using UART1 on this board does not work without issues.


Flash it with vstudio code and platform.io (extension for vstudio code) 

ESP RX (GPIO17) to MKS42C TX ESP TX (GPIO0) to MKS42C RX MKS42C UART GND to the 12v GND. MKS42C UART 3.3v is not needed in this configuration.

12v +- to the pwer input of the MKS42C ( there are on the right of the 6 Pin connector.



For different ESP32 boards it would require some changes. platform.ini.
