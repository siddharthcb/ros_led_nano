# ros_led_nano

## Jetson Nano
```
sudo /opt/nvidia/jetson-io/jetson-io.py
- Configure 40-pin expansion header
- tick spi1 for pin 19 and spi2 for pin 13
- back 
- reboot


pip install spidev

$ git clone https://github.com/siddharthcb/ros_led_nano.git
cd ~/src/
python led_strip_ros.py

Example script for publishing the topic is located in ~/examples
```

## Raspberry Pi 4 (Ubuntu 20.04)

Install raspi-config from [here](https://qiita.com/penguinprogrammer/items/a252676a3ce6bd1410da).

`sudo raspi-config` and enable spi device, then reboot

`sudo pip3 install spidev`

make sure `/etc/modules` has `spi-dev` at the end of the file

make sure there is `spi` group exist by `group` command, if it shows `spi`, then skip to add `ubuntu` to the group

if there is no `spi` as group, then create `group`

	sudo groupadd spi
	sudo chmod g+rw /dev/spidev0.0

add `ubuntu` to `spi` group

	sudo usermod -a -G spi ubuntu

make udev-rules to change owner of spi when boot up

	sudo touch /etc/udev/rules.d/99-spi.rules

	sudo gedit /etc/udev/rules.d/99-spi.rules

add this to the file

	SUBSYSTEM=="spidev", MODE="0666"

`sudo udevadm control --reload`

`reboot`

clone the repo and run the node similar to Jetson Nano
