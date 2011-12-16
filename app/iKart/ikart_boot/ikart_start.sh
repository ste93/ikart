#!/bin/sh

sudo -u icub -i joystickCheck
ret1=$?

if [ $ret1 -eq 101 ]; then
    echo "joystick boot requested" 
    sudo -u icub -i yarp server &
    sleep 2
    sudo -u icub -i iCubInterface --context iKart --config conf/iKart.ini &
    sudo -u icub -i iKartCtrl --no_compass &
    sudo -u icub -i joystickCtrl --context joystickCtrl --from conf/xbox_linux.ini --silent &
    sleep 1
    sudo -u icub -i yarp connect /joystickCtrl:o /ikart/joystick:i
elif [ $ret1 -eq 0 ]; then
    echo "No joystick boot requested" 
else
    echo "Unknown error" 
fi