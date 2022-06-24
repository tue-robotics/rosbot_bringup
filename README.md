# rosbot_bringup

# installation instructions

After installation of the default ROSBOT software:

## setting up the ROS network

install the following packages for namespace resolution. Reboot afterwards.

`sudo apt-get install libnss-mdns avahi-daemon avahi-dnsconfd`

libnss-mdns # Allows for searching .local dns names
avahi-daemon # Allows for exposing .local dns name
avahi-dnsconfd # Listens on the network for announced DNS servers and passes them to resolvconf


#TODO: setup the ROS network with proper namespace resolution.

### ROBOT
In the bashrc of the robot add the line:

  `export ROS_IP=<ip of the robot>`

Disable the firewall on the robot, or whitelist the remote PC on the robot. To do the latter, on the robot run:

`sudo ufw allow from <ip of the remote PC>`

### Remote PC
In the bashrc of the remote PC add:
 
  `export ROS_HOSTNAME=<ip of the remote PC>`
  
Disable the firewall on your remote PC or whitelist the robot on the remote PC. To do the latter, on the robot run:

`sudo ufw allow from <ip of the remote PC>`

  

