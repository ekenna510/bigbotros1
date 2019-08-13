#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/LinearMath/Quaternion.h"
#include  <tf/transform_datatypes.h>

nav_msgs::Odometry odommsg;
geometry_msgs::Pose2D bot_pose;




void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        bot_pose.x=msg->pose.pose.position.x;
        bot_pose.y=msg->pose.pose.position.y;

        tf::Quaternion q(
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w
                    );
        bot_pose.theta= tf::getYaw(q);
odommsg.pose.pose.position.x = msg->pose.pose.position.x;
odommsg.pose.pose.position.y = msg->pose.pose.position.y;
odommsg.twist.twist.linear.x = msg->twist.twist.linear.x;
odommsg.twist.twist.linear.y = msg->twist.twist.linear.y;
odommsg.twist.twist.angular.z = msg->twist.twist.angular.z;
odommsg.twist.twist.linear.z = sqrt(pow(odommsg.twist.twist.linear.y,2 ) + pow(odommsg.twist.twist.linear.x,2));


}





int main(int argc, char **argv)
{

  ros::init(argc, argv, "dosquare");


  ros::NodeHandle n;

  ros::Publisher velocity_pub;


  velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  ros::Subscriber pose_subscriber = n.subscribe("odom", 10, &odomCallback);


  double speed, angular_speed;
  double distance, angle;
  bool isForward, clockwise;

  ros::Rate r(10);



      ROS_INFO("\n\n\n******START TESTING************\n");
      geometry_msgs::Pose2D  pose;
      pose.x=5;
      pose.y=0;
      pose.theta=0;

      geometry_msgs::Twist vel_msg;
      //set a random linear velocity in the x-axis
      vel_msg.linear.x =.1;
      vel_msg.linear.y =0;
      vel_msg.linear.z =0;
      //set a random angular velocity in the y-axis
      vel_msg.angular.x = 0;
      vel_msg.angular.y = 0;
      vel_msg.angular.z =0;
      velocity_pub.publish(vel_msg);

      for (int i = 0;i < 20;i++)
      {
         ros::spinOnce();
         ROS_INFO("%d\n",i);

         r.sleep();
      }

      vel_msg.linear.x =0;
      velocity_pub.publish(vel_msg);

      ROS_INFO("\n\n\n******End TESTING************\n");

      /** test your code here **/




  return 0;
}

