# TestAccel
A small example for reading 16 bit register via I2C by ESP32 ULP coprocessor using macros.

Why this? Trying to implement a simple activity monitoring by an accelerometer I encountered several issues. 
1st was intagrating assembly code for the ULP-core into a arduino project. After spending some days reading the whole internet ;-) trying to find a solution I capitulate. A sense making support for this you only can gain if you switch to cmake, which is the only supported platform by espressif since some time. Switching a complex arduiono-project to another make process is no fun. By the way, I'm not using the Arduino-IDE + environment but Eclipse with Arduiono-CDT. It seems there is a solution for using assembly code within the Arduino environment, but I do not intend to switch a project because of some assembly code lines...  So I decided to use macros.
That leads me straight to the next issues. The I2C hardware interface of the ESP32 UPL core is not able to read 16 bit registers. So I looked for a possibility to deal with 8 bit access, and yes, for my purposes there was one. Implementing the procedure using according register setups of I2C interface was bringing up the next issue - the I2C hardware interface of the ESP32 UPL core seems to be buggy and thus useless. 
Heavenly God! Okay, next step. Implementing a I2C software interface is not a big problem, so I did this, encountering next issue: the UPL code size is limited to 128 instructions. To bypass this, there is a possibility to increase the maximum code size in configuration files and avoid a failing code length check by using a own copy of the initialization procedure with disabled length check - obviously the standard initialization procedure don't care about setting int the config files.

Now here you can find the result of all my investigations. Please consider some points:

* this example just reads one acceloration value out of a ADXL345 accelerometer and writes it into RTC_SLOW_MEMORY, you may easily change device adress and register address to test your own I2C device
* the I2C bus clocking is nearly 400kHz (somewhat below), if you intend to use a I2C device only supporting 100kHz you need to insert delays. Have a look into TestAccel_ULP.h, I wrote down some instruction timings, so maybe you can use the instructions istself to reach the needed delay
* if you define the RTC_SLOW_MEMORY space for writing values, consider the code length
* if you enable I2C_DEBUG you can trace the written and the read values including the single bits (have a look on the main loop)
* if you fill the _DEBUG_SLOW define with a big amount of delays, you may observe the bus status by driving LEDs (if no oscilloscope is available)
* both debug modi require a increased code space - read the comments in the header file regarding this
* you may save 12 instructions in this code, if you carefully count instructions and substitute all macro branches M_B*() by the matching immediate instruction I_B*() with absolute destination addresses (simply the number of instruction where to jump to) because the macro branches will compile to 2 instructions
* have fun
