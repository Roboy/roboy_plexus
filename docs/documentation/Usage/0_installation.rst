Installation
============

Prerequisites
-------------
`Download <http://wiki.ros.org/kinetic/Installation/Ubuntu>`_  ROS

`Download <https://dl.altera.com/soceds/17.1/?edition=standard&platform=linux&download_manager=dlm3>`_  and install **SoCEDS 17 (or greater)** standard edition (! must be installed with sudo).

`Download <http://dl.altera.com/?edition=lite>`_ and install **Quartus 17 (or greater)** lite edition. (Quartus is only needed if you want to change the FPGA core)

Get The GNU C/C++ compilers for armhf architecture
::
    sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf 

Clone roboy_plexus recursively into your catkin workspace:
::
    git clone --recursive https://github.com/Roboy/roboy_plexus

The package also depends on the following packages which need to be cloned into your workspace as well:
::
    git clone https://github.com/Roboy/common_utilities
    git clone https://github.com/Roboy/roboy_communication

FPGA
----
Find the SD card image with Ubuntu 16.04, xfce, kernel linux 4.9.78-ltsi on our servers:
::
    wget -nv http://bot.roboy.org:8081/~roboy/DE10_nano_lxce_4.9.78-ltsi-altera.md5sum
    wget -nv http://bot.roboy.org:8081/~roboy/DE10_nano_lxce_4.9.78-ltsi-altera.img
Plug the SD card into your reader and get the name of your SD card device with the following command:
::
    sudo fdisk -l
Flash the image to a min 8GB SD card: (sdX has to be replaced with the correct device)
::
    sudo dd if=DE10_nano_lxce_4.9.78-ltsi-altera.img of=/dev/sdX bs=1M status=progress
.. warning::
    The dd command overwrites any device you give it with 'of'. MAKE SURE YOU USE THE CORRECT DEVICE.
    
FPGA (flashing the old way --obsoleted)
----
Notice: The flashing procedure takes up to 10 minutes. Meanwhile the flashing of the FPGA can be done. This has to be done permanently. Make sure the dip switches are set to On-Off-On-Off-Off, i.e. configure to flash on the FPGA on startup from eeprom via jic (for further information read the de10 nano user guide: `Download <http://www.terasic.com.tw/cgi-bin/page/archive_download.pl?Language=China&No=1046&FID=1c19d1d50e0ee9b21678e881004f6d81>`_).


Start the jtag server with the following command:
::
    sudo ~/intelFPGA/17.1/quartus/bin/jtagd

Connect the FPGA flash port with your computer by using an USB cable, then verify the connection using:
::
    sudo ~/intelFPGA/17.1/quartus/bin/jtagconfig

Start the software quartus and open the project ``DE10_NANO_SoC_GHRD.qpf`` in the folder ``roboy_de10_nano_soc``. Open the programmer.
Under Hardware Setup choose your FPGA. Click auto-detect and verify the model.
Choose soc_system.jic and flash the FPGA.

Build
-----
Before building the package make sure you are in an embedded command shell, then build with catkin:
::
    ~/intelFPGA/17.1/embedded/embedded_command_shell.sh
    catkin_make
    

