# embedded_uwb

This is all the embedded code for the SKITH UWB boards.

# Hardware

The modules used are the SKITH boards from the chair 8 of the university of Würzburg. The skith modules consist of a stm32f4 microcontroller and a Decawave DWM1000 UWB transceiver. The board can be expanded modularly with an UART module for serial communication. To flash code an ST-Link has to be used. This has been tested with an ST nucleo board.

## Prerequisites

```bash
sudo apt install -y apt-utils
sudo apt install -y clang clang-format clang-tools gdb
sudo apt install -y gcc-multilib g++-multilib
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
sudo apt install -y cmake
```

## How to build

```bash
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../rodos/cmake/port/skith.cmake ..
make all
```

## How to connect

See http://galileo2.informatik.uni-wuerzburg.de/wiki/index.php/SKITH if you are connected in the university network, to know how to connect the pins. The Nucleo-F446RE board needs to be connected to the SKITH UWB board by SWD. See https://www.radioshuttle.de/turtle/nucleo-st-link-interface/ to get whitch pin needs to be opened. The SKITH UWB board needs to be power for example by the SKITH B main board. With the STM32cubeProgrammer it is possible to connect with the ST-Link configurations and to flash it. The reset mode needs to be set to Software reset and the port to SWD. The IDD connection needs to be connected so that the protection is not enabled.

# Software

The [decadriver](decadriver/) and [wrapper](wrapper/) folders include the decawawe api for the DWM1000 module. The [rodos](rodos/) and [ros_lib](ros_lib/) folders include the base RODOS code and the RODOS-ROS-Bridge used in this project. In the [include](include/) and [common](src/common/) folders there are files relevant to all UWB tasks. Currently, there are four different code options, that can be flashed onto the UWB board. [anchor](src/anchor/) and [tag](src/tag/) are very similar and make the board function either as an UWB anchor or tag respectively. [id](src/id/) is a simple program that only publishes the hardware id of the chip. [monitor](src/monitor/) reads out all sent UWB messages and publishes - currently only for a few of them - a ROS message via the RODOS-ROS-Brige. Therefore, if only makes sense to run this module while it is connected to a PC with the SKITH B main board. Then it can be used to monitor UWB messages between nodes not physically connected to a PC.