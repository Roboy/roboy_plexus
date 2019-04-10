Getting started
===============

Connect to FPGA
===============

Connect via USB port (not recommended, only if not found in the network)
----
Power-up your FPGA with the flashed SD card in it. Connect a USB cable to the UART port of the FPGA, after that use the following command:
::
    sudo screen /dev/ttyUSB0 115200

You should see either the FPGA boot screen or already the login screen (if not press enter).
Login in data: 
user: 'root'
password: 'Roboy2016'

To find out what IP the FPGA is using:
::
    ifconfig

Connect via ssh 
----
Connect either an ethernet cable to the FPGA and your LAN or connect it directly to your PC 
(in ubuntu-> settings-> network-> wired-> options-> ipv4-> shared_to_others)

The FPGA can be found via NMAP (search for Gandalf)
::
    sudo nmap 192.168.0.0/24

If you cannot find this net, use Roboy's class B net:
::
    sudo nmap 192.168.0.0/16

In the ~/.bashrc of the FPGA edit the ROS_MASTER_URI variable so it is set to the IP where ROS core was started.
Source the ~/.bashrc and make sure ROS_IP and ROS_MASTER_URI are set correctly:
::
    echo $ROS_IP
    echo $ROS_MASTER_URI

copy data to FPGA  
===============
Copy plexus to FPGA
----
Copy the roboy_plexus binary to the FPGA:
::
    scp <YOUR CATKIN WORKSPACE>/devel/lib/roboy_plexus/roboy_plexus root@10.42.0.1:~

Copy HDL to FPGA 
----
The copy is only needed if the RBF file was changed.

FireUp the system 
===============
Start the ROS core on your host pc.

::    
    roscore

Run roboy_plexus on the FPGA:
::
    ./roboy_plexus
      
If you haven't done this before execute the following commands on your computer:
::
    echo "export ROS_IP=$(hostname -I|head -n1 | awk '{print $1;'})" >> ~/.bashrc
    echo "export ROS_MASTER_URI=http://$ROS_IP:11311" >> ~/.bashrc

Troubleshooting
===============
Troubleshooting plexus on PC
----
Echo $ROS_MASTER_URI shows the wrong IP even bashrc was edited 

First solution
:: 
    source ~/.bashrc   
Second solution (IP address has to be changed to your current IP)
::
    export ROS_MASTER_URI=http://192.168.0.231:11311

Troubleshooting plexus on FPGA
----
If the LED slides shows two "running" LED's, plexus is started more than once. In that case use the following comment:
::
    killall roboy_plexus
    
It might happen that the ssh interface doesn't open the terminal. This might occur if the plexus code is broken - to prevent it from autostart. In that case connect the board to a PC via USB and comment out the last line of ~/.bashrc that starts Plexus.

Visualize data
===============
For visualizing your data use RQT

Go into your catkin WORKSPACE and into SRC and clone RQT
::
    git clone https://github.com/Roboy/roboy_rqt_plugins.git
    
After catkin_make is executed the new plugins have to be source:
::
    source <YOUR CATKIN WORKSPACE>/devel/setup.bash
