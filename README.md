# rosbot_bringup

# installation instructions

After installation of the default ROSBOT software:

## setting up the ROS network

#TODO: setup the ROS network with proper namespace resolution.

In the bashrc of the robot add the line:

  `export ROS_IP=<ip of the robot>`
  
In the bashrc of the remote PC add:
 
  `export ROS_HOSTNAME=<ip of the remote PC>
  
Disable the firewall on the robot, or whitelist the remote PC on the robot. To do the latter, on the robot run:

`sudo ufw allow from <ip of the remote PC>`
