#!/usr/bin/expect -f

set 1 [lindex $argv 0]



set timeout 1
spawn ssh biorob@$1
expect {
        -re "The.*(yes/no)?"      {send "yes\r"; exp_continue}
        -re "biorob@$1's password:" {send "BioRob_2012\r" ;exp_continue}
        eof
    }

send -- "ssh unitree@192.168.123.12\r"
expect {
        -re "The.*(yes/no)?"      {send "yes\r"; exp_continue}
        -re "unitree@192.168.123.12's password:" {send "BioRob_2012\r" ;exp_continue}
        eof
    }

send -- "export ROS_MASTER_URI=http://192.168.123.42:11311\r"
send -- "export ROS_IP=192.168.123.12\r"
send -- "roslaunch camera_pkg camera_launcher.launch\r"

interact



