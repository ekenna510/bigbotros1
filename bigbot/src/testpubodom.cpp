#include <string>
#include <ros/ros.h>
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "testpubodom");
    ros::NodeHandle n;
    ros::Rate loop_rate(30);
    ros::Publisher* ptr_pub;

    ros::NodeHandle rosNode;

    //Setup to publish ROS messages
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 100);

    nav_msgs::Odometry msg;    
    
    
    geometry_msgs::Pose2D bot_pose; // hold current position of bot.
    

    int dt;
    double  deltaLeft, deltaRight, Velth, deltaX, deltaY,deltath,linear, angular;


 //        long lastleftclick=0,lastrightclick=0, deltaLeft, deltaRight;

    bot_pose.x = 0.0;
    bot_pose.y = 0.0;
    bot_pose.theta = 0.0;

    dt = 100;

    // how many click did wee see.
    deltaLeft =  .05;
    deltaRight = .05;

    while (ros::ok()) {


    bot_pose.x += deltaLeft;
    bot_pose.y += deltaRight;
    bot_pose.theta = 0;

    msg.pose.pose.position.x= bot_pose.x;
    msg.pose.pose.position.y= bot_pose.y;
    msg.pose.pose.position.z= 0;


    //---------------------------------------------------------------------------------------
    /*
    http://answers.ros.org/question/11973/gathering-wheel-encoder-data-and-publishing-on-the-odom-topic/
    */
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(bot_pose.theta);

    //first, we'll publish the transform over tf
    //geometry_msgs::TransformStamped odom_trans;
    //odom_trans.header.stamp = ros::Time::now();
    //odom_trans.header.frame_id = "odom";
    //odom_trans.child_frame_id = base_frame;

    //odom_trans.transform.translation.x = bot_pose.x;
    //odom_trans.transform.translation.y = bot_pose.y;
    //odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation = odom_quat;

    //send the transform
//    ptr_broadcaster->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    //  nav_msgs::Odometry odom;
    msg.header.stamp = ros::Time::now();//CurrentEncoderTime;
    msg.header.frame_id = "odom";

    //set the position
    msg.pose.pose.position.x = bot_pose.x;
    msg.pose.pose.position.y = bot_pose.y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = odom_quat;

    //set the velocity
    msg.child_frame_id = "base_link";
    msg.twist.twist.linear.x = deltaX; //vx;//vx
    msg.twist.twist.linear.y = deltaY;//vy
    msg.twist.twist.angular.z = Velth;

    pub.publish(msg);
    
    // This will adjust as needed per iteration
    loop_rate.sleep();
    
    
    }
    
    
    
    
    
        
}
