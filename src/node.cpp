#include <cmath>
#include <mutex>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/BumperEvent.h>

#include <custom_msgs/baseDrive.h>
#include <custom_msgs/baseRotate.h>
#include <custom_msgs/areaReset.h>
#include <custom_msgs/position.h>

double linear_vel_max;
double vel_constant;
double angular_vel_max;
double ang_constant;
double rotateVelocity;
double reverseDistance;
double aligningThreshold;
double movingThreshold;
double controlDelay;

double x_negative, temp_x_negative;
double y_negative, temp_y_negative;

octomap::point3d currentPosition;
octomap::point3d goalPosition;
octomap::point3d markedPosition;
geometry_msgs::Twist cmd;
kobuki_msgs::MotorPower power_cmd;

bool power_status = false;
bool connected = false;
bool bumper = true;

int count = 0;

ros::Publisher velocity_pub;
ros::Publisher motpower_pub;

double angle;
double angleDiff;
double angularV;
double velocity;
double distance;
double currentYaw;

void updateArea()
{
	x_negative = temp_x_negative;
	y_negative = temp_y_negative;
}

void positionCallback(const custom_msgs::position::ConstPtr &msg){
	currentPosition = octomap::point3d(msg->x+x_negative, msg->y+y_negative, msg->z);
	currentYaw = msg->Y;
}

void bumperCallback(const kobuki_msgs::BumperEventConstPtr &msg)
{
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED){
    	ROS_INFO_STREAM("Bumper pressed. backing up");
		bumper = false;
    }
}

bool resetCallback(custom_msgs::areaReset::Request &request, custom_msgs::areaReset::Response &response)
{
	ROS_DEBUG("control_qbot_node : reset request received");

    temp_x_negative = request.x_negative;
    temp_y_negative = request.y_negative;

	response.success = true;

	ROS_DEBUG("control_qbot_node : succesfully resetted");

	return true;
}

bool driveCallback(custom_msgs::baseDrive::Request &request, custom_msgs::baseDrive::Response &response)
{ 
	updateArea();

	ROS_DEBUG("control_qbot_node : drive request received");

    goalPosition.x() = request.x;
	goalPosition.y() = request.y;
	goalPosition.z() = request.z;

	double travelledDistance;
	octomap::point3d startPosition = currentPosition;

	if (!power_status){
		ROS_DEBUG("control_qbot_node : motor was powered down. turning on now....");
		power_cmd.state = kobuki_msgs::MotorPower::ON;
		motpower_pub.publish(power_cmd);
		ROS_DEBUG("control_qbot_node : successfully powered up.");
		power_status = true;
	}
	
	double desiredYaw = atan2(goalPosition.y()-currentPosition.y(), goalPosition.x()-currentPosition.x());
	double initDistance = goalPosition.distanceXY(currentPosition);

	do{
		angleDiff = desiredYaw - currentYaw;

		if (std::abs(angleDiff)>6.28) angleDiff = 0;
		
		if (angleDiff >  3.14) angleDiff = angleDiff - 6.28;
		if (angleDiff < -3.14) angleDiff = angleDiff + 6.28;

		angularV = ang_constant*angleDiff;
		
		if (angleDiff>0) if (angularV > angular_vel_max) angularV = angular_vel_max; 
		if (angleDiff<0) if ((-1*angularV) > angular_vel_max) angularV = -1*angular_vel_max;

		cmd.angular.z = angularV;
	    velocity_pub.publish(cmd);
	    ros::Duration(controlDelay).sleep();
	} while ((std::abs(desiredYaw - currentYaw)>=aligningThreshold) && bumper);

	cmd.angular.z = 0;
	velocity_pub.publish(cmd);
	ros::Duration(controlDelay).sleep();
	ROS_DEBUG("control_qbot_node : successfully rotated");

	response.success = true;

	do{
		travelledDistance = startPosition.distanceXY(currentPosition);
		distance = initDistance-travelledDistance;
		velocity = distance*vel_constant;
		if (velocity >= linear_vel_max) velocity = linear_vel_max;
		cmd.linear.x = velocity;
		velocity_pub.publish(cmd);
		ros::Duration(controlDelay).sleep();
	} while (((initDistance-startPosition.distanceXY(currentPosition))>=movingThreshold) && bumper);

	if (!bumper){
		response.success = false;
		ROS_INFO("control_qbot_node : interrupted due to too close obstacle");
	} else {
		response.success = true;
		ROS_DEBUG("control_qbot_node : successfully moved");
	}

	cmd.linear.x = 0;
	velocity_pub.publish(cmd);
	ros::Duration(controlDelay).sleep();
	ROS_DEBUG("control_qbot_node : successfully moved");

	ROS_DEBUG("control_qbot_node : reached goal. turning motors off...");
	power_cmd.state = kobuki_msgs::MotorPower::OFF;
	motpower_pub.publish(power_cmd);
	ros::Duration(controlDelay).sleep();
	ROS_DEBUG("control_qbot_node : successfully powered down.");
	power_status = false;

	ROS_DEBUG("control_qbot_node : response sent");

	return true;
}

bool reverseCallback(custom_msgs::baseDrive::Request &request, custom_msgs::baseDrive::Response &response)
{
	updateArea();

    goalPosition.x() = request.x;
	goalPosition.y() = request.y;
	goalPosition.z() = request.z;

	octomap::point3d startPosition = currentPosition;

	ROS_DEBUG("control_qbot_node : reverse request received");

	double initDistance = goalPosition.distanceXY(currentPosition);

	if (!power_status){
		ROS_DEBUG("control_qbot_node : motor was powered down. turning on now....");
		power_cmd.state = kobuki_msgs::MotorPower::ON;
		motpower_pub.publish(power_cmd);
		ros::Duration(controlDelay).sleep();
		ROS_DEBUG("control_qbot_node : successfully powered up.");
		power_status = true;
	}

	do{
		distance = initDistance - startPosition.distanceXY(currentPosition);
		velocity = -1*distance*vel_constant;
		if (velocity <= -1*linear_vel_max) velocity = -1*linear_vel_max;
		cmd.linear.x = velocity;
		velocity_pub.publish(cmd);
		ros::Duration(controlDelay).sleep();
	} while ((initDistance - startPosition.distanceXY(currentPosition))>=movingThreshold);

	cmd.linear.x = 0;
	velocity_pub.publish(cmd);
	ros::Duration(controlDelay).sleep();
	ROS_DEBUG("control_qbot_node : successfully moved");

	response.success = true;

	ROS_DEBUG("control_qbot_node : reached goal. turning motors off...");
	power_cmd.state = kobuki_msgs::MotorPower::OFF;
	motpower_pub.publish(power_cmd);
	ros::Duration(controlDelay).sleep();
	ROS_DEBUG("control_qbot_node : successfully powered down.");
	power_status = false;

	ROS_DEBUG("control_qbot_node : response sent");

	return true;
}

bool rotateCallback(custom_msgs::baseRotate::Request &request, custom_msgs::baseRotate::Response &response)
{ 
	updateArea();

	ROS_DEBUG("control_qbot_node : rotate request received");

	if (!power_status){
		ROS_DEBUG("control_qbot_node : motor was powered down. turning on now....");
		power_cmd.state = kobuki_msgs::MotorPower::ON;
		motpower_pub.publish(power_cmd);
		ros::Duration(controlDelay).sleep();
		ROS_DEBUG("control_qbot_node : successfully powered up.");
		power_status = true;
	}

	double previousYaw = currentYaw;
	double angleDifference = 0;

	do{
		cmd.angular.z = rotateVelocity;
		velocity_pub.publish(cmd);
		ros::Duration(controlDelay).sleep();
		angleDifference += (currentYaw - previousYaw);
		if (currentYaw<0 && previousYaw>0) angleDifference += 6.28;
		previousYaw = currentYaw;
	} while (request.angle>=angleDifference);

	cmd.angular.z = 0;
	velocity_pub.publish(cmd);
	ros::Duration(controlDelay).sleep();
	ROS_DEBUG("control_qbot_node : successfully rotated");

	ROS_DEBUG("control_qbot_node : reach goal. turning motors off...");
	power_cmd.state = kobuki_msgs::MotorPower::OFF;
	motpower_pub.publish(power_cmd);
	ros::Duration(controlDelay).sleep();
	ROS_DEBUG("control_qbot_node : successfully powered down.");
	power_status = false;

	response.success = true;

	ROS_DEBUG("control_qbot_node : response sent");
	
	return true;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "Velocity_Controller_QBot");
	ros::NodeHandle node;
	
	ROS_INFO("Initialized the control_qbot_node");

	node.param("area/back", 						x_negative, 		0.0);
    node.param("area/right", 						y_negative, 		0.0);

	node.param("robot/velocity/max", 				linear_vel_max, 	0.5);
    node.param("robot/velocity/constant", 			vel_constant, 		2.0);
    node.param("robot/angularVelocity/max", 		angular_vel_max, 	0.3);
    node.param("robot/angularVelocity/constant", 	ang_constant, 		1.0);
    node.param("robot/angularVelocity/fixed", 		rotateVelocity, 	0.3);
    node.param("robot/movement/reverseDistance", 	reverseDistance, 	0.1);
	node.param("robot/control/aligningThreshold", 	aligningThreshold, 	0.2);
	node.param("robot/control/movingThreshold", 	movingThreshold, 	0.0);
	node.param("robot/control/delay", 				controlDelay, 		0.005);

    temp_x_negative = x_negative;
    temp_y_negative = y_negative;

	ROS_INFO("control_qbot_node : loaded parameters");

	ros::Subscriber bump_sub = node.subscribe("bumper", 1, bumperCallback);
	ros::Subscriber posi_sub = node.subscribe("position", 1, positionCallback);
	
	ROS_INFO("control_qbot_node : created subscribers");

	ros::ServiceServer serviceDrive = node.advertiseService<custom_msgs::baseDriveRequest, custom_msgs::baseDriveResponse>("baseForword", driveCallback);
	ros::ServiceServer serviceReverse = node.advertiseService<custom_msgs::baseDriveRequest, custom_msgs::baseDriveResponse>("baseReverse", reverseCallback);
	ros::ServiceServer serviceRotate = node.advertiseService<custom_msgs::baseRotateRequest, custom_msgs::baseRotateResponse>("baseRotate", rotateCallback);
	ros::ServiceServer serviceReset = node.advertiseService<custom_msgs::areaResetRequest, custom_msgs::areaResetResponse>("areaResetControl", resetCallback);

	ROS_INFO("control_qbot_node : created service");

	velocity_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	motpower_pub = node.advertise<kobuki_msgs::MotorPower>("motor_power", 1);

	ROS_INFO("control_qbot_node : created publishers");

	cmd.linear.x = 0.0;
  	cmd.linear.y = 0.0;
  	cmd.linear.z = 0.0;
  	cmd.angular.x = 0.0;
  	cmd.angular.y = 0.0;
  	cmd.angular.z = 0.0;

	ROS_DEBUG("control_qbot_node : velocity message initialized");
	
	while (!connected)
	{
		if (motpower_pub.getNumSubscribers() > 0){
			connected = true;
			ROS_WARN_STREAM("control_qbot_node : connected with base");
			break;
		}
		
		if (count == 6){
			connected = false;
			break;
		} else {
			ROS_WARN_STREAM("control_qbot_node : could not connect with base, trying again after 500ms...");
			ros::Duration(0.5).sleep(); // sleep for half a second
			++count;
		}
	}

	if (!connected){
		ROS_ERROR("control_qbot_node : could not connect with base");
		ROS_ERROR("control_qbot_node : check remappings for enable/disable topics).");
	}

	ros::AsyncSpinner spinner (7);
	spinner.start();
	
	ros::waitForShutdown();

	return 0;
}