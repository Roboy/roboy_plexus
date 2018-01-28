Getting started
===============

Power-up your fpga with the flashed sd-card in it. Connect a USB cable to the uart port of the fpga, then:
::
    sudo screen /dev/ttyUSB0 115200

You should see the fpga boot screen, or already login screen (if not press enter).
Login in ( user: root, password: Roboy2016 ).

Connect an ethernet cable to the fpga and you lan. You can also connect it directly to your pc ( in ubuntu->settings->network->wired->options->ipv4->shared_to_others )

Make sure the fpga has an IP using:
::
    ifconfig

In the ~/.bashrc of the fpga edit the ROS_MASTER_URI variable so it is set to the IP where the roscore was started.
Source the ~/.bashrc and make sure ROS_IP and ROS_MASTER_URI are set correctly:
::
    echo $ROS_IP
    echo $ROS_MASTER_URI

Copy the roboy_plexus binary to the fpga:
::
    scp devel/lib/roboy_plexus/roboy_plexus root@10.42.0.1:~

Start the roscore on your host pc.
Run roboy_plexus on the fpga:
::
    ./roboy_plexus
