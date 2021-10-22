# akig_2022
TUW akig course W2022

# Use xsens driver on Ubuntu 20.04
´´´bash
$ git clone https://github.com/ethz-asl/ethzasl_xsens_driver.git
$ sudo chmod o+rw /dev/ttyUSB0
$ catkin_make
$ source devel/setup.bash
$ sudo apt-get install python2.7-minimal
$ curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
$ sudo python2 get-pip.py
$ pip2 install serial
$ pip2 install pyserial
$ cd ~/workspace_AKIG/src/ethzasl_xsens_driver/nodes
* mtdevice.py  
* mtnode.py
 - #!/usr/bin/env python (zu)
 - #!/usr/bin/env python2.7 (editieren)
$ rosrun xsens_driver mtdevice.py
$ rosrun xsens_driver mtnode.py _device:=/dev/ttyUSB0 _baudrate:=115200
´´´
asdf
