# rosbot_bringup

### IP-adresses of the Rosbots
On the atHome network the rosbots have the following IPs

| Robot  | Type | IP |
| ------------- | ------------- |  ------------- |
| __Pyro__  | ROSBOT 2.0 (Red and White)  | 192.168.44.122
| *TBD 1* | ROSBOT 2.0 PRO (Black and Purple) | 192.168.44.141
| *TBD 2* | ROSBOT 2.0 PRO (Black and Purple) | 192.168.44.147

Logging into the robots is easily done through the command line
```
ssh husarion@<IP of Robot>
```

# installation instructions

After installation of the default ROSBOT software:

## Single Robot
### setting up the ROS network

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

`sudo ufw allow from <ip of the remote PC>`

## Multi-Robot



