sensors-pkg
===========

Modules for interfacing common robotics sensors or devices

Requisites:

  * Module `PhidgetsEncoders`
    * libphidgets: [Linux install](http://www.phidgets.com/docs/OS_-_Linux)

  * Module `IMU_XSens`
    * (Linux only) To run without `sudo`, copy MRPT's [`scripts/52-xsens.rules`](https://github.com/jlblancoc/mrpt/blob/master/scripts/52-xsens.rules) udev rule:
    
            sudo cp [MRPT_DIR]/scripts/52-xsens.rules /etc/udev/rules.d/

