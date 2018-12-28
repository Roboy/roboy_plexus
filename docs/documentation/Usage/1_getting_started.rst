Getting started
===============

Connect to FPGA
===============

Connect via USB (not recomended, only if not found in network)
----
Power-up your fpga with the flashed sd-card in it. Connect a USB cable to the uart port of the fpga, then:
::
    sudo screen /dev/ttyUSB0 115200

You should see the fpga boot screen, or already login screen (if not press enter).
Login in ( user: root, password: Roboy2016 ).

To find out what ip the fpga has:
::
    ifconfig

Connect via ssh 
----
Connect an ethernet cable to the fpga and you lan. You can also connect it directly to your pc ( in ubuntu->settings->network->wired->options->ipv4->shared_to_others )

The FPGA can be found via nmap (look for Gandalf)
::
    sudo nmap 192.168.0.0/24
(roboy usally has a class B net so if you can't find it try /16)

In the ~/.bashrc of the fpga edit the ROS_MASTER_URI variable so it is set to the IP where the roscore was started.
Source the ~/.bashrc and make sure ROS_IP and ROS_MASTER_URI are set correctly:
::
    echo $ROS_IP
    echo $ROS_MASTER_URI

copy data to FPGA  
===============
copy plexus to FPGA
----
Copy the roboy_plexus binary to the fpga:
::
    scp <YOUR CATKIN WORKSPACE>/devel/lib/roboy_plexus/roboy_plexus root@10.42.0.1:~

copy HDL to FPGA 
----
(only needed if the rbf file was changed)

FireUp the system 
===============
Start the roscore on your host pc.

::    
    roscore

Run roboy_plexus on the fpga:
::
    ./roboy_plexus
      
if you haven't done this befor add execute the following commands on your computer:
::
    echo "export ROS_IP=$(hostname -I|head -n1 | awk '{print $1;'})" >> ~/.bashrc
    echo "export ROS_MASTER_URI=http://$ROS_IP:11311" >> ~/.bashrc

troubleshooting
===============
troubleshooting plexus on PC
----
echo $ROS_MASTER_URI shows the wrong IP even bashrc was eddited 
first solution
:: 
    source ~/.bashrc   
second solution (Ip address has to be changed to your current Ip)
::
    export ROS_MASTER_URI=http://192.168.0.231:11311

troubleshooting plexus on FPGA
----
If the led slides shows 2 "running" LED's, plexus ist started more than once. This means it has to be killed. 
::
    killall roboy_plexus
    
Sometimes the ssh interface dosn't open the terminal, this sometimes happens if the plexus code is broken. To prevent it from autostart. Connect the board to a pc via USB and comment out the last line of ~/.bashrc that starts plexus.

Visualize data
===============
To visualize data, there is something called rqt

go into your catkin WORKSPACE and into src and clone rqt
::
    git clone https://github.com/Roboy/roboy_rqt_plugins.git
    
after catkin_make is executed the new plugins have to be source 
::
    source <YOUR CATKIN WORKSPACE>/devel/setup.bash
