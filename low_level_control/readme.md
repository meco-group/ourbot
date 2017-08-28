LOW LEVEL CONTROL
=================

This folder contains the software and toolchain to flash the low level control software to the Teensy. The toolchain is meant for the teensy3.1/teensy3.2 only, not for other variants, although they are normally not used within the ourbot project.

To get started, install the udev rules as described in the file '49-teensy.rules'. This is needed so that your system gets to know the teensy. Now you should be able to access the teensy.

Folder structure
================

The ./avr-libs contains some libraries such as (an older version of...) MicroOS, components, communication layers, ... You should not bother to much about these.

The ./mavlink folders contains (an older version of...) the mavlink protocol toolchain. This is needed to compile a set of messages transmitted to the pc. If you want to add messages to the protocol, you should change the ./mavlink/message\_definitions/v1.0/ourbot\_messages.xml and recompile everthing using the mavgenerate.

The ./ourbot folder contains the toolchain needed to flash the teensy as well as the ourbot-specific source code. The makefile supports two important commands:
* make: compiles the code for the teensy (same as make all)
* make upload: compiles the code and flash it to the teensy
You can also do a make clean to clean the directory.

Uploading software
==================

The software is uploaded to the teensy by navigating to ./ourbot in a terminal window and doing a 'make upload'. This should launch the teensy uploader, which should tell you the software has been uploaded

Changing the software
=====================

The software relies on an older version of the MicroOS. This schedules tasks, in the ourbots case communication, heartbeat, control and spi IMU readings. The execution frequencies are:
* heartbeat: 1Hz
* IMU: 200Hz
* communication: 1000Hz
* control: 1000Hz
Note: although the communication runs at a rate of 1000Hz, messages are sent at a rate of +/- 100Hz.

If you want to add some functionality, the easiest approach would be to add it to the main.cpp file as was done for the IMUs. You can assign this function to a thread in the microOS, as was done for the 200Hz_loop().
