<h1 align=center>Getting Started with STM32CubeIDE</h1>
This repository contains sample drivers developed for GPIO, SPI, I2C and UART Peripherals of NUCLEO-F401RE development board and it also contains some testing applications.

<h3>Creating a New Project</h3>
File > New > STM32 Project<br>
Board Selector > NUCLEO-F401RE > Next > Project Name > C > Executable > Empty

<h3>Disabling Floating Point Unit</h3>
Right Click > Properties > C/C++ Build > Settings > Tool Settings > Floting Point Unit > None<br>
Floating Point ABI > Software Implementation

<h3>Semihosting for Debugging</h3>
<h5>Set Linker Parameters</h5>
Right Click > Properties > C/C++ Build > Settings >Tool settings > MCU GCC Linker ><br>
Miselaneous>other flags > add > -specs=rdimon.specs -lc -lrdimon

<h5>Exclude syscalls.c from build</h5>
Right Click on syscalls.c > Properties > Exclude Source from Build

<h5>Set Debug Configuration</h5>
Right Click on Project > Debug as > STM32 Cortex-M C/C++ Application >Debug Configurations > <br>
Debugger > ST-Link(OpenOCD) > Startup > Run Commands > monitor arm semihosting enable

<h5>Add the Code</h5>
extern  void initialise_monitor_handles(void);
initialise_monitor_handles();

<h3>Including Path for new Folders</h3>
Properties / C/C++ Build / Settings / MCU GCC Compiler / include paths / Add
