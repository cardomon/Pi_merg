# Pi_merg
Raspberry Pi and Arduino applications for model railway application

Application 1 - simple DCC accessory decoder for Raspberry Pi Pico microcontroller

This Micropython program allows a Pico to decode DCC signals and detect those addressed to an accessory decoder.
The code has been tested connected to a Lenz LHZ100 system and uses the address space interpretation used by that system.
The decoder looks for switch instructions send to one of four addresses (e.g. 37-40 inclusive) and interprets those instructions
by setting one of four GPIO pins high or low.

To interface to DCC a few components are needed including an optoisolator.
