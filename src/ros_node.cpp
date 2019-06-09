#include "ros_node.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>

ros_node::ros_node(driver *driver, int argc, char **argv)
{
    // Create a new driver.
    ros_node::m_driver = driver;

    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mpu9250");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle();

    // Read parameters.
    ros::NodeHandle private_node("~");
    std::string param_serial_port;
    private_node.param<std::string>("serial_port", param_serial_port, "/dev/ttyAMA0");
    double param_scan_rate;
    private_node.param<double>("scan_rate", param_scan_rate, 50.0);

    // Set up publishers.
    ros_node::m_nav_publisher = ros_node::m_node->advertise<sensor_msgs::NavSatFix>("gps/position", 1);
    ros_node::m_time_publisher = ros_node::m_node->advertise<sensor_msgs::TimeReference>("gps/time", 1);

    // Initialize ros node members.
    ros_node::m_scan_rate = new ros::Rate(param_scan_rate);

    // Initialize the driver and set parameters.
    try
    {
        // Attach the callback to the driver.
        ros_node::m_driver->set_data_callback(std::bind(&ros_node::data_callback, this, std::placeholders::_1));

        // Initialize driver.
        ros_node::m_driver->initialize(param_serial_port);

        ROS_INFO_STREAM("Venus 638FLPX driver successfully initialized on port " << param_serial_port << ".");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
        // Deinitialize driver.
        ros_node::deinitialize_driver();
        // Quit the node.
        ros::shutdown();
    }
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_scan_rate;
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

void ros_node::spin()
{
    while(ros::ok())
    {
        // Scan for NMEA strings.
        ros_node::m_driver->spin();

        // Spin the ros node once.
        ros::spinOnce();

        // Loop.
        ros_node::m_scan_rate->sleep();
    }

    // Deinitialize driver.
    ros_node::deinitialize_driver();
}

void ros_node::deinitialize_driver()
{
    try
    {
        ros_node::m_driver->deinitialize();
        ROS_INFO_STREAM("Driver successfully deinitialized.");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
    }
}

void ros_node::data_callback(driver::data data)
{
    // Populate nav sat message.
    sensor_msgs::NavSatFix nav_message;
    nav_message.header.stamp = ros::Time::now();
    nav_message.header.frame_id = "driver_mpu9250";
    nav_message.status.service = nav_message.status.SERVICE_GPS;
    switch(data.fix_type)
    {
    case 1: // No fix.
    {
        nav_message.status.status = nav_message.status.STATUS_NO_FIX;
        nav_message.latitude = std::numeric_limits<double>::quiet_NaN();
        nav_message.longitude = std::numeric_limits<double>::quiet_NaN();;
        nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
        break;
    }
    case 2: // 2D Fix
    {
        nav_message.status.status = nav_message.status.STATUS_FIX;
        nav_message.latitude = data.latitude;
        nav_message.longitude = data.longitude;
        nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
        break;
    }
    case 3: // 3D Fix
    {
        nav_message.status.status = nav_message.status.STATUS_FIX;
        nav_message.latitude = data.latitude;
        nav_message.longitude = data.longitude;
        nav_message.altitude = data.altitude;
        break;
    }
    }
    // Publish time message.
    ros_node::m_nav_publisher.publish(nav_message);

    // Populate time message.
    sensor_msgs::TimeReference time_message;
    time_message.header.stamp = ros::Time::now();
    time_message.source = "MPU9250 GPS";
    time_message.time_ref = ros::Time(data.utc_time_of_day);
    // Publish time message.
    ros_node::m_time_publisher.publish(time_message);
}
