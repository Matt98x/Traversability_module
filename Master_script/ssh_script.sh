#!/usr/bin/expect

set 1 [lindex $argv 0]
set 2 [lindex $argv 1]
set 3 [lindex $argv 2]


spawn ssh $1@$2
expect {
        -re "The.*(yes/no)?"      {send "yes\r"; exp_continue}
        -re "$1\@$2's password:" {send "$3\r"; exp_continue}
        eof
    }


