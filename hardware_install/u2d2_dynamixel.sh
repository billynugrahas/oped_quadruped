#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE:="0666", GROUP:="dialout",  SYMLINK+="U2D2"' >/etc/udev/rules.d/U2D2_dynamixel.rules

service udev reload
sleep 2
service udev restart

