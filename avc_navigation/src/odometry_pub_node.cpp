//subscribe to various sensor messages
//perform sensor fusion, estimate current odometry values
//output values (Odometry message)

//ROS includes
#include <ros/ros.h>
#include <tf/tf.h>//for tf transforms receiving and giving data
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Included messages
#include <sensor_msgs/Range.h> //some other sensor_msg
#include <nav_msgs/Odometry.h>  //create Odom estimation file
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h> //may not need
//IMU values - velocity, acceleration, heading


//GPS location and velocity


//Hall Effect - wheel speed


//Subscribe to sensor node for value collection
//GPS Callback function
int GPS_pos = 0;
GPS_latitude=msg-->GPS_latitude
interim_pos=0;

//functions for each sensor

//messages
geometry_msg::Twist pos header.frame_id
geometry_msg.linear.x=0
geometry_msg.linear.y=0
geometry_msg.linear.z=0
geometry_msg.angular.x=0
geometry_msg.angular.y=0
geometry_msg.angular.z=0

//publishing odometry messages without latching
ros::Publisher odometry_pub = node_private.advertise<geometry_msgs::Twist>("odom", 10, false)
ros::Subscriber

//refresh rate specification
ros::Rate loop_rate(refresh_rate)

//loop to update GPS_pos and auto reset interim with each reading
while (ros:ok())
{

}
