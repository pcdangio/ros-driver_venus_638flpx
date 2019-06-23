#include "ros_node.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>

// CONSTRUCTORS
ros_node::ros_node(int argc, char **argv)
{
    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mpu9250");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle();

    // Read standard parameters.
    ros::NodeHandle private_node("~");
    std::string param_serial_port;
    private_node.param<std::string>("serial_port", param_serial_port, "/dev/ttyAMA0");
    int param_baud_rate;
    private_node.param<int>("baud_rate", param_baud_rate, 115200);
    double param_read_rate;
    private_node.param<double>("read_rate", param_read_rate, 50.0);
    private_node.param<double>("uere", ros_node::m_uere, 6.74);

    // Read configure parameters.
    int param_update_baud;
    private_node.param<int>("update_baud", param_update_baud, -1);
    int param_update_rate;
    private_node.param<int>("update_rate", param_update_rate, -1);

    // Set up publishers.
    ros_node::m_nav_publisher = ros_node::m_node->advertise<sensor_msgs::NavSatFix>("gps/position", 1);
    ros_node::m_time_publisher = ros_node::m_node->advertise<sensor_msgs::TimeReference>("gps/time", 1);

    // Initialize ros node members.
    ros_node::m_read_rate = new ros::Rate(param_read_rate);

    // Initialize driver.
    ros_node::m_driver = new driver(param_serial_port, static_cast<unsigned int>(param_baud_rate));
    ROS_INFO_STREAM("Connecting to Venus GPS on " << param_serial_port << " at " << param_baud_rate << "bps.");

    // Check if baud rate is getting updated.
    if(param_update_baud != -1)
    {
        // Update baud rate.
        // Try to convert parameter to baud rate type.
        driver::baud_rates new_baud;
        try
        {
            new_baud = static_cast<driver::baud_rates>(param_update_baud);
        }
        catch(...)
        {
            ROS_FATAL_STREAM("Cannot update baud rate: invalid baud rate enumeration supplied (" << param_update_baud << ").");
            ros::shutdown();
        }

        // Update the baud rate.
        if(ros_node::m_driver->set_baud(new_baud))
        {
            // ACKed by Venus GPS.
            ROS_INFO_STREAM("Baud rate successfully changed to enumeration: " << param_update_baud << ".");
        }
        else
        {
            // NAKed or no response.
            ROS_FATAL_STREAM("Could not change baud rate.");
        }

        // Exit the node since either the baud was changed or the operation failed.
        ros::shutdown();
    }

    // Check if update rate is getting updated.
    if(param_update_rate != -1)
    {
        // Try to convert parameter to update rate type.
        driver::update_rates new_rate;
        try
        {
            new_rate = static_cast<driver::update_rates>(param_update_rate);
        }
        catch(...)
        {
            ROS_FATAL_STREAM("Cannot set positioning rate: invalid rate enumeration supplied (" << param_update_rate << ").");
            ros::shutdown();
        }

        // Update the rate.
        if(ros_node::m_driver->set_update_rate(new_rate))
        {
            // ACKed by Venus GPS.
            ROS_INFO_STREAM("Position update rate successfully changed to enumeration: " << param_update_rate << ".");
        }
        else
        {
            // NAKed or no response.
            ROS_FATAL_STREAM("Could not change position update rate.");
            ros::shutdown();
        }
    }

    // Ensure that the GPS is in normal power mode.
    if(ros_node::m_driver->set_power(true) == false)
    {
        ROS_FATAL_STREAM("Could not power on GPS.");
        ros::shutdown();
    }

    // Attach callbacks.
    ros_node::m_driver->set_data_callback(std::bind(&ros_node::data_callback, this, std::placeholders::_1));
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_read_rate;
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

// METHODS
void ros_node::spin()
{
    while(ros::ok())
    {
        // Scan for NMEA strings.
        ros_node::m_driver->spin();

        // Spin the ros node once.
        ros::spinOnce();

        // Loop.
        ros_node::m_read_rate->sleep();
    }
}

// CALLBACKS
void ros_node::data_callback(driver::data data)
{
    // Populate nav sat message.
    sensor_msgs::NavSatFix nav_message;
    nav_message.header.stamp = ros::Time::now();
    nav_message.header.frame_id = "driver_mpu9250";
    nav_message.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    if(data.fix_type == 1)
    {
        // No fix.
        nav_message.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        nav_message.latitude = std::numeric_limits<double>::quiet_NaN();
        nav_message.longitude = std::numeric_limits<double>::quiet_NaN();;
        nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
        nav_message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        nav_message.position_covariance.fill(std::numeric_limits<double>::quiet_NaN());
    }
    else
    {
        // 2D or 3D fix.
        nav_message.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        nav_message.latitude = data.latitude;
        nav_message.longitude = data.longitude;
        nav_message.altitude = std::numeric_limits<double>::quiet_NaN();
        // Set covariance matrix.
        nav_message.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        // Calculate cov_xy = (uere*hdop)^2 / 2, cov_z = (uere*vdop)^2
        double cov_xy = std::pow(ros_node::m_uere * static_cast<double>(data.hdop), 2.0) / 2.0;
        double cov_z = std::pow(ros_node::m_uere * static_cast<double>(data.vdop), 2.0);
        nav_message.position_covariance = {cov_xy, 0.0, 0.0,
                                           0.0, cov_xy, 0.0,
                                           0.0, 0.0, cov_z};
    }
    if(data.fix_type == 3)
    {
        // 3D fix.
        nav_message.altitude = data.altitude;
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
