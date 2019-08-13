
// ROS includes
#include "ros/ros.h"
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>


#include <vector>
#include <iostream>
#include <string>
#include <signal.h>




sensor_msgs::LaserScan CDLaser_msg;

//
int counts;
double f_angle_min;
double f_angle_max;
double f_angle_increment;
double f_time_increment;
double f_scan_time;
double f_range_min;
double f_range_max;

int publisher_timer;



int main(int argc, char * argv[]) {
    ros::init(argc, argv, "fake_scan");

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);


    counts = 100;

    CDLaser_msg.ranges.resize(counts);
    CDLaser_msg.intensities.resize(counts);
    CDLaser_msg.header.frame_id = "laser_frame";
    CDLaser_msg.angle_min = -3.14;
    CDLaser_msg.angle_max = 3.14;
    CDLaser_msg.angle_increment = (CDLaser_msg.angle_max - CDLaser_msg.angle_min) / (double)counts;
    CDLaser_msg.range_min = 0.08;
    CDLaser_msg.range_max = 16.0;
    CDLaser_msg.time_increment =.0005; // seconds between measurements
    CDLaser_msg.scan_time = .1; // seconds between scans

double maxd = 14;
int midpoint = counts/2;
double rangeinterval = maxd/midpoint;
    ros::Rate rate(20);
    while (ros::ok()) {
        try{


        CDLaser_msg.header.stamp = ros::Time::now();

        for (int i=0;i < counts; i++)
        {
        CDLaser_msg.ranges[i] = maxd - (fabs(midpoint - i )*rangeinterval) ;
        CDLaser_msg.intensities[i] = .5;
        }

    scan_pub.publish(CDLaser_msg);
    rate.sleep();
    ros::spinOnce();

    }catch(std::exception &e){//
              ROS_ERROR_STREAM("Unhandled Exception: " << e.what() );
              break;
    }catch(...){//anthor exception
              ROS_ERROR("Unhandled Exception:Unknown ");
              break;
    }
    }


}
