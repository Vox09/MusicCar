HKUST ELEC3300 Project

We developed a 3-wheel camera car that can move in all directions, take photo and save to SD card. It has a LCD screen to display its current state or the image from camera (in a very low speed).It can be controlled via a bluetooth App. PID control is implemented for the three wheels.

The control board is stm32f103. The project is generated using CubeMX. Third libraray includes FreeRTOS and FatFS.

The toolchain for master branch is `gcc-none-eabi`  
The toolchain for keil brach is `arm-cc` for Keil.
