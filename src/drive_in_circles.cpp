#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "circle_driver");

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/desState", 50);

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.5;
	double vy = 0.0;
	double vth = -0.5;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(50.0);
	while (n.ok()) {
		current_time = ros::Time::now();

		//integration step
		double dt = (current_time - last_time).toSec();
		double delta_x = vx * cos(th) * dt;
		double delta_y = vx * sin(th) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		double th2 = 0.5 * th;
		geometry_msgs::Quaternion odom_quat;
		odom_quat.x = 0.0;
		odom_quat.y = 0.0;
		odom_quat.z = sin(th2);
		odom_quat.w = cos(th2);

		nav_msgs::Odometry odom;

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		//odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		odom_pub.publish(odom);

		//tick
		last_time = current_time;
		r.sleep();
	}
}
