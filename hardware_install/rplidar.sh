#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' >/etc/udev/rules.d/ydlidar.rules

service udev reload
sleep 2
service udev restart

