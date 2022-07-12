#include "ros/ros.h"
#include <ros/callback_queue.h>

#include "nav_msgs/Odometry.h"
#include "custom_msgs/position.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

std::string odometry_frame_id, robot_frame_id;
long count;

tf2_ros::Buffer* buffer = nullptr;
tf2_ros::TransformListener* listener = nullptr;

ros::Subscriber sub_odom;
ros::Publisher pub_xyzR;

float  x = 0,  y = 0,  z = 0,  R = 0,  P = 0,  Y = 0;
float dx = 0, dy = 0, dz = 0, dR = 0, dP = 0, dY = 0;

double roll, pitch, yaw;
double tfDelay;

void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

	tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Matrix3x3 (q).getRPY(roll, pitch, yaw);

    if (std::abs(roll)<3.1415) R = roll;
    if (std::abs(pitch)<3.1415) P = pitch;
    if (std::abs(yaw)<3.1415) Y = yaw;

    custom_msgs::position positionMsg;

    positionMsg.header.seq = count;
    positionMsg.header.stamp = ros::Time::now();
    positionMsg.header.frame_id = odometry_frame_id;

    positionMsg.x = x + dx;
    positionMsg.y = y + dy;
    positionMsg.z = z + dz;
    positionMsg.R = R + dR;
    positionMsg.P = P + dP;
    positionMsg.Y = Y + dY;

    pub_xyzR.publish(positionMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Position Tracker");
    ros::NodeHandle node;

    node.param<std::string>("map/frame",            odometry_frame_id,  "odom");
    node.param<std::string>("robot/frame",          robot_frame_id,     "base_link");
    node.param(             "robot/tfUpdateDelay",  tfDelay,            0.0001);

    sub_odom = node.subscribe("positionIn", 1, callback);
    pub_xyzR = node.advertise<custom_msgs::position>("positionOut",   1, true);

    buffer = new tf2_ros::Buffer();
    listener = new tf2_ros::TransformListener(*buffer);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (ros::ok())
    {
	    geometry_msgs::TransformStamped transform;
	    bool tf_set = false;
        double rollT, pitchT, yawT;

        while (!tf_set)
        {
            try{
                transform = buffer->lookupTransform(odometry_frame_id, robot_frame_id, ros::Time::now());
                tf_set = true;
            } catch (tf2::TransformException ex){
                //ROS_ERROR("%s", ex.what());
                tf_set = false;
                ros::Duration(tfDelay).sleep();
            }
        }

        tf2::Quaternion q (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
	    tf2::Matrix3x3 (q).getRPY(rollT, pitchT, yawT);

        dx = transform.transform.translation.x - x;
        dy = transform.transform.translation.y - y;
        dz = transform.transform.translation.z - z;
        dR = rollT  - R;
        dP = pitchT - P;
        dY = yawT   - Y;
    }

    return 0;
}