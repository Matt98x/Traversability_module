#!/usr/bin/expect -f

set 1 [lindex $argv 0]
set 2 [lindex $argv 1]
set 3 [lindex $argv 2]
set 4 [lindex $argv 3]

set timeout 1
spawn ssh $1@$2
expect {
        -re "The.*(yes/no)?"      {send "yes\r"; exp_continue}
        -re "$1\@$2's password:" {send "$4\r";exp_continue}
        eof
    }

send -- "export ROS_MASTER_URI=http://$3:11311"
send -- "export ROS_HOSTNAME=$2"

interact



