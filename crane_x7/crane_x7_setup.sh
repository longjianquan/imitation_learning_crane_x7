sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
sudo echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
sudo echo 1 > /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
