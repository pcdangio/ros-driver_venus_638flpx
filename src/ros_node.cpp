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
    sensor_msgs::NavSatFix message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "driver_mpu9250";
    message.
}
