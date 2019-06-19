# driver_venus_638flpx

## Overview

This package includes driver software for the SkyTraq Venus [638FLPx] GPS Receiver.

**Keywords:** skytraq venus 638flpx gps driver raspberry_pi

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_venus_638flpx package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs) (ROS sensor_msgs)
- [pigpio](http://abyz.me.uk/rpi/pigpio/) (Raspberry PI I/O)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-driver_venus_638flpx.git driver_venus_638flpx
        cd ../
        catkin_make

## Usage

Run any of the driver nodes with (where xxx is the driver type):

        rosrun driver_venus_638flpx xxx_node

For example, to run the node using a driver for a Raspberry Pi:

        rosrun driver_venus_638flpx rpi_node

## Nodes

### rpi_node

A Raspberry Pi driver for [638FLPx].  Ensure that the pigpio daemon is running before starting this node.


#### Published Topics
* **`gps/position`** ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

        The GPS position measured by the sensor.

* **`gps/time`** ([sensor_msgs/TimeReference](http://docs.ros.org/api/sensor_msgs/html/msg/TimeReference.html))

        The current time of day in UTC time.


#### Parameters

* **`~/serial_port`** (string, default: /dev/ttyAMA0)

        The serial port connected to the sensor.

* **`~/scan_rate`** (double, default: 50)

        The rate at which to scan the serial port for new NMEA messages.

* **`~/uere`** (double, default: 6.74)

        The User Equivalent Range Error (UERE) representing the total pseudorange error budget.  This is typically 6.74 for C/A, and 6.0 for P(Y).


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_venus_638flpx/issues).


[ROS]: http://www.ros.org
[638FLPx]: https://cdn.sparkfun.com/datasheets/Sensors/GPS/Venus/638/doc/Venus638FLPx_DS_v07.pdf
