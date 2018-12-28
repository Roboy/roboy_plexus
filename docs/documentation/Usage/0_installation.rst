Installation
============

Prerequisites
-------------
`Download <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_  ROS

`Download <https://dl.altera.com/soceds/17.1/?edition=standard&platform=linux&download_manager=dlm3>`_  and install **SoCEDS 17 (or greater)** standard edition. (! must be installed with sudo)

`Download <http://dl.altera.com/?edition=lite>`_ and install **Quartus 17 (or greater)** lite edition. (Quartus is only needed if you want to change the FPGA core)

Get The GNU C/C++ compilers for armhf architecture
::
    sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf 

Clone roboy_plexus recursively into your catkin workspace:
::
    git clone --recursive https://github.com/Roboy/roboy_plexus

The package also depends on the following packages which need to be cloned into your workspace aswell:
::
    git clone https://github.com/Roboy/common_utilities
    git clone https://github.com/Roboy/roboy_communication

FPGA
----
you can download the sd-card image with Ubuntu 16.04, xfce, kernel linux 4.9.78-ltsi from our servers:
::
    wget -nv http://bot.roboy.org:8081/~roboy/DE10_nano_lxce_4.9.78-ltsi-altera.md5sum
    wget -nv http://bot.roboy.org:8081/~roboy/DE10_nano_lxce_4.9.78-ltsi-altera.img
Plug the sdcard into you reader. In a terminal, find out the name of your sd card device:
::
    sudo fdisk -l
flash the image to a min 8GB sd-card: (sdX has to be replaced with the correct device)
::
    sudo dd if=DE10_nano_lxce_4.9.78-ltsi-altera.img of=/dev/sdX bs=1M status=progress
.. warning::
    The dd command overwrites any device you give it with 'of'. MAKE SURE YOU USE THE CORRECT DEVICE.
    
FPGA (flashing the old way --obsoleted)
----
The flashing procedure takes up to 10 minutes. While your waiting, you can flash the fpga. We will flash it permanently,
so make sure the dip switches are set to On-Off-On-On-Off-Off, ie configured to flash the fpga on startup from eeprom via jic
( read the de10 nano user guide if your don't know what that means ).

Start the jtag server
::
    sudo ~/intelFPGA/17.1/quartus/bin/jtagd

Connect the fpga flash port with your computer using a USB cable, then verify teh connection using:
::
    sudo ~/intelFPGA/17.1/quartus/bin/jtagconfig

Start quartus and open the project ``DE10_NANO_SoC_GHRD.qpf`` in ``roboy_de10_nano_soc``. Open the programmer.
Under Hardware Setup choose your fpga. Click auto-detect and verify the model.
Choose soc_system.jic and flash the fpga.

Build
-----
Before building the package make sure you are in an embedded command shell, then build with catkin:
::
    ~/intelFPGA/17.1/embedded/embedded_command_shell.sh
    catkin_make
    

