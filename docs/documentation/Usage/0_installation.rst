Installation
============

Prerequisites
-------------
Download and install SoCEDS 17.1 standard edition from https://dl.altera.com/soceds/17.1/?edition=standard&platform=linux&download_manager=dlm3 .

Download and install Quartus 17.1 lite edition from http://dl.altera.com/?edition=lite

Clone roboy_plexus recursively into your catkin workspace:
::
    git clone --recursive https://github.com/Roboy/roboy_plexus

The package also depends on the following packages which need to be cloned into your workspace aswell:
::
    git clone https://github.com/Roboy/common_utilities
    git clone https://github.com/Roboy/roboy_communication

FPGA
----
If you have a fresh de10-nano soc fpga, download the sd card image DE10_nano_lxce_4.1.33-ltsi-altera.img from https://roboy.org/dists/stable/main/binary/ .
Plug the sdcard into you reader. In a terminal, find out the name of your sd card device:
::
    sudo fdisk -l

Then flash the image onto the sd card using the following command, replacing /dev/sdX with your sdcard device name:
::
    sudo dd if=DE10_nano_lxce_4.1.33-ltsi-altera.img of=/dev/sdX bs=1M status=progress
.. warning::
    The dd command overwrites any device you give it with 'of'. MAKE SURE YOU USE THE CORRECT DEVICE.

The flashing procedure takes up to 10 minutes. While your waiting, you can flash the fpga. We will flash it permanently,
so make sure the dip switches are set to On-Off-On-On-Off-Off, ie configured to flash the fpga on startup from eeprom via jic
( read the de10 nano user guide if your don't know what that means ).

Start the jtag server
::
    sudo ~/intelFPGA/17.1/quartus/bin/jtagd

Connect the fpga flash port with your computer using a USB cable, then verify teh connection using:
::
    sudo ~/intelFPGA/17.1/quartus/bin/jtagconfig

Start quartus and open the project DE10_NANO_SoC_GHRD.qpf in roboy_de10_nano_soc. Open the programmer.
Under Hardware Setup choose your fpga. Click auto-detect and verify the model.
Choose soc_system.jic and flash the fpga.

Build
-----
Before building the package make sure you are in an embedded command shell, then build with catkin:
::
    ~/intelFPGA/17.1/embedded/embedded_command_shell.sh
    catkin_make