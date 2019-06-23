# driver_venus_638flpx

## Overview

This package includes driver software for the SkyTraq Venus [638FLPx] GPS Receiver.

**Keywords:** skytraq venus 638flpx gps driver

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_venus_638flpx package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [serial](http://wiki.ros.org/serial) (ROS serial package)
- [sensor_msgs](http://wiki.ros.org/sensor_msgs) (ROS sensor_msgs)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-driver_venus_638flpx.git driver_venus_638flpx
        cd ../
        catkin_make

## Usage

Run the driver with the following command:

        rosrun driver_venus_638flpx node

## Nodes

### node

A driver for interacting with the Venus [638FLPx] GPS.  Enables configuration and reading of NMEA data.


#### Published Topics
* **`gps/position`** ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

        The GPS position measured by the sensor.

* **`gps/time`** ([sensor_msgs/TimeReference](http://docs.ros.org/api/sensor_msgs/html/msg/TimeReference.html))

        The current time of day in UTC time.


#### Runtime Parameters

* **`~/serial_port`** (string, default: /dev/ttyAMA0)

        The serial port connected to the sensor.

* **`~/baud_rate`** (uint32, default: 115200)

        The baud rate to use for serial communication with the sensor.

* **`~/read_rate`** (double, default: 50)

        The rate at which to scan the serial port for new NMEA messages.

* **`~/uere`** (double, default: 6.74)

        The User Equivalent Range Error (UERE) representing the total pseudorange error budget.  This is typically 6.74 for C/A, and 6.0 for P(Y).

#### Configuration Parameters

These parameters should only be changed when needed, and not set prior to every run.  These parameters will be updated in the sensor's flash memory and will be preserved over power cycles.

* **`~/update_baud`** (int, default: -1)

        Changes the baud rate of the Venus GPS using an enumeration.  Possible values are:
        0 = 4800 bps
        1 = 9600 bps
        3 = 38400 bps
        5 = 115200 bps
        The default value of -1 instructs the node to ignore updating the baud rate.

* **`~/update_rate`** (int, default: -1)

        Changes the position update rate of the Venus GPS using an enumeration.  Possible values are:
        1 = 1Hz
        2 = 2Hz
        4 = 4Hz
        5 = 5Hz
        8 = 8Hz
        10 = 10Hz
        20 = 20Hz
        The default value of -1 instructs the node to ignore updating the update rate.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_venus_638flpx/issues).


[ROS]: http://www.ros.org
[638FLPx]: https://cdn.sparkfun.com/datasheets/Sensors/GPS/Venus/638/doc/Venus638FLPx_DS_v07.pdf
