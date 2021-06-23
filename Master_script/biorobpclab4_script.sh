#!/usr/bin/expect -f


send -- "export ROS_MASTER_URI=http://$1:11311\r"
send -- "export ROS_IP=biorobpclab4\r"


interact

