# STM32-HAL-Libcanard

[Libcanard](https://github.com/UAVCAN/libcanard) is a compact implementation of the UAVCAN/CAN protocol stack in C99/C11 for high-integrity real-time embedded systems.

[UAVCAN](https://uavcan.org) is an open lightweight data bus standard designed for reliable intravehicular communication in aerospace and robotic applications via CANb bus, Ethernet, and other robust transports.

This repo provides the bare minimum example of Libcanard 2.0 usage with STM32F series MCU, both messages transmition and receiving. It does not use the advised memory allocator [O1heap](https://github.com/pavel-kirienko/o1heap) for the sake of simplicity. Communication with the CAN bus is done via standard STM32HAL driver. The example is tested on STM32F446RExx MCU, but should be portable on any chip of STM32F family.

Example application generates a heartbeat message with 1s period and pushes it to the CAN bus in polling mode. Also application subscribes for the Real64-array message and processes it in interrupt triggered by each new CAN frame. 

## Building the example
Open .ioc file in the [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) and generate source files for your preffered IDE. Then just add **`canard.c `** path into the sources and  **`/include `** into the search directories. 
