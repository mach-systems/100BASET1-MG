# 100BASET1-MG
100BASE-T1 Media Gateway C language SDK - An STM32CubeIDE project

100BASE-T1 Media Gateway is a configurable Automotive Ethernet switch and communication bridge. The device can be used as frame sniffer (Active TAP), employing the packet forwarding function, as well as Ethernet – CAN(/FD) gateway, allowing data to be bi-directionally passed between any of the Ethernet ports and the CAN or CAN FD network.
The gateway features three 100BASE-T1 ports, one Gigabit Ethernet (1000BASE-T or 10/100BASE-TX) port, two CAN(/FD) channels, LIN bus, and a USB port. All Ethernet ports are internally connected to a switch circuit. A microSD card slot, two digital outputs, and two analogue inputs are also available.

This repository contains the C language SDK - an STM32CubeIDE example project that shows the usage of all peripherals. Detailed description can be found in main.c.

Product Page: https://www.machsystems.cz/en/products/embedded-networking/gateways-and-bus-converters/100base-t1-media-gateway

Product Number: 100BASET1-MG

### Device Features:
- 3x 100BASE-T1 port
- 1x 1000BASE-T port (gigabit Ethernet)
- All Ethernet ports are „switched“ (including the MCU)
- 2x CAN(/FD)
- LIN bus
- USB 2.0
- MicroSD card slot
- 2x Digital output (1x high-side 5V/0.5A, 1x low-side 40V/1A)
- 2x Analogue input (0-30 V)
- Embedded web server for configuration and status information
- User-programmable firmware
- Externally or USB-powered
- Table-top use or DIN-rail mount
