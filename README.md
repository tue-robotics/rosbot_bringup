# rosbot_bringup
makes use of adapted version of rosbot_ekf
https://github.com/KdVos/rosbot_ekf

## Prerequisites
Make sure the following repos are present in the husarion_ws of the robot

https://github.com/KdVos/rosbot_rosserial_python

https://github.com/KdVos/rosbot_ekf_multi

### IP-adresses of the Rosbots
On the rosbot network the rosbots have the following IPs

| Robot  | Type | IP |
| ------------- | ------------- |  ------------- |
| __Pyro__  | ROSBOT 2.0 (Red and White)  | 192.168.1.3
| __Bobo__ | ROSBOT 2.0 PRO (Black and Purple) | 192.168.1.4
| __Coco__ | ROSBOT 2.0 PRO (Black and Purple) | 192.168.1.5

Logging into the robots is easily done through the command line
```
ssh husarion@<IP of Robot>
```

# Installation instructions

After installation of the default ROSBOT software:

## Single Robot
### Setting up the ROS network

install the following packages for namespace resolution. Reboot afterwards.

`sudo apt-get install libnss-mdns avahi-daemon avahi-dnsconfd`

libnss-mdns # Allows for searching .local dns names
avahi-daemon # Allows for exposing .local dns name
avahi-dnsconfd # Listens on the network for announced DNS servers and passes them to resolvconf


install the following packages for namespace resolution. Reboot afterwards.

`sudo apt-get install libnss-mdns avahi-daemon avahi-dnsconfd`

libnss-mdns # Allows for searching .local dns names
avahi-daemon # Allows for exposing .local dns name
avahi-dnsconfd # Listens on the network for announced DNS servers and passes them to resolvconf


#TODO: setup the ROS network with proper namespace resolution.

**ROBOT**

In the bashrc of the robot add the line:

  `export ROS_IP=<ip of the robot>`

Disable the firewall on the robot, or whitelist the remote PC on the robot. To do the latter, on the robot run:

`sudo ufw allow from <ip of the remote PC>`

Furthermore, make sure that the following enviroment variable is set

 `export ROSBOT_VER=ROSBOT_2.0` 
 or
  `export ROSBOT_VER=ROSBOT_2.0_PRO`
 
 depending on the exact model you're using.

**Remote PC**

In the bashrc of the remote PC add:
 
  `export ROS_HOSTNAME=<ip of the remote PC>`
  
Disable the firewall on your remote PC or whitelist the robot on the remote PC. To do the latter, on the robot run:

`sudo ufw allow from <ip of the robot>`

### Launch the Hardware
On the Robot run the following command on the robot

`roslaunch rosbot_bringup start.launch`

To test the correct functioning of the robot you can drive around using teleop

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

To also launch the camera node, run instead

`roslaunch rosbot_bringup start.launch camera:=true`

To test the working of the (rgb)-camera run (on the remote laptop)

`rosrun image_view image_view image:=/camera/rgb/image_raw/ compressed`

## Multi-Robot

Using ROS, there has to be **exactly one** master, which runs the *roscore*. 

Depending on your specific application, and/or preferences, you might want to run the roscore on either the remote pc or one of the robots.

In this readme we will assume that the roscore is running on one of the robots.

### Setting up the ROS network
Access the bashrc file on each robot/laptop using `nano ~/.bashrc`, close and save using `ctrl + x`.

Make sure that for each, the following variables are set

**Remote PC**

  `export ROS_MASTER_URI=http://<IP of the Master Robot>:11311`
  
  `export ROS_HOSTNAME=<ip of the remote PC>`
  
**Master Robot**

  `export ROS_MASTER_URI=http://master:11311`
  
  `export ROS_IP=<ip of the Master Robot>`
  
**Other Robots**

 `export ROS_MASTER_URI=http://<IP of the Master Robot>:11311`
  
  `export ROS_HOSTNAME=<ip of this Robot>`
  
  ### Launch the Hardware
  
  In order to prevent a clash between nodes that have the same name, we have to namespace all nodes that we want to launch. 
  
On each Robot run the following command on the robot

`roslaunch rosbot_bringup start_ns.launch ns:=<Name of Robot>`

To test the correct functioning of the robot you can drive around using teleop

`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:= /<Name of Robot>/cmd_vel`

To also launch the camera node, run instead

`roslaunch rosbot_bringup start_ns.launch ns:=<Name of Robot> camera:=true`

To test the working of the (rgb)-camera run (on the remote laptop)

`rosrun image_view image_view image:=/<Name of Robot>/camera/rgb/image_raw/ compressed`
  





