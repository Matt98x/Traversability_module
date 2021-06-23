#!/bin/bash

# parameter order: EXTTX2 wifi address - EXTTX2 and A1 password - optitrack computer username - optitrack computer address - optitrack computer password

#128.179.182.188 EXTTX2
#128.178.148.174 biorobpclab4

addr1=m48b02d0f67b1.dyn.epfl.ch
gnome-terminal -- ./EXTTX2_script1.sh $addr1


gnome-terminal -- ./A1_script.sh $addr1


gnome-terminal -- ./biorobpclab4_script.sh $addr1
