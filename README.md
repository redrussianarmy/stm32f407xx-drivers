# 1 - Introduction

This repository contains GPIO, SPI, I2C and USART drivers for STM32F407xx microcontrollers. Drivers and examples were written in C programming language.

This repository contains two types of folders, identifiable using the following naming convention:

- folder that contains the *GPIO, SPI, I2C and USART drivers*, named  `drivers`,
- folder that contains the *examples*, named  `src`,

Another folder, named  `Startup`, contains startup file of *STM32f407vgtx* that is created automatically by STM32CubeIDE.

In order to `clone` the complete content of this folder use the command:

```git
git clone git@github.com:redrussianarmy/stm32f407xx-drivers.git
```
or
```git
git clone https://github.com/redrussianarmy/stm32f407xx-drivers.git
```

### 1.a - GPIO, SPI, I2C and USART drivers folder structure

*drivers* folder contains:

- *inc* folder that contains device-specific header file and specified driver header files
- *src* folder that contains specified driver source files

### 1.b - Examples folder structure

*src* folder contains some examples of related drivers.

------

# 2 - Integration details
You only need to include the device-specific header file ```"stm32f407xx.h"``` in your main.c file

### 2.a Required properties

> * A standard C language compiler for the target MCU
> * Optional: STM32CubeIDE

------

# 3 - Running Examples

STM32CubeIDE was used to develop this project.

### 3.a Using a STMicroelectronics STM32F407xx Discovery Board

In case of using the supported STMicroelectronics evaluation boards the examples file(.c) can run without applying any modifications (as is).

In order to do that, please follow the following steps:

1. Simply open the project with STM32CubeIDE
2. Open one of the example in *src* folder.
3. Make sure that other examples are excluded from build.
4. Build the project.
5. You may run in the Target MCU or debug to examine working steps in detail
6. Enjoy :-)
