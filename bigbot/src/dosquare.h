#ifndef DOSQUARE_H
#define DOSQUARE_H
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/LinearMath/Quaternion.h"
#include  <tf/transform_datatypes.h>
class dosquare
{
private:
nav_msgs::Odometry odommsg;
geometry_msgs::Pose2D bot_pose;
double frontRange;
bool gotRange=false;

bool isInitialPoseSet = false;



const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;
const double PI = 3.14159265359;

double degrees2radians(double angle_in_degrees);
double getDistance(double x1, double y1, double x2, double y2);
double AvgDiff();
double Trend();

public:

ros::Publisher *velocity_publisher;

ros::Subscriber pose_subscriber;
ros::Subscriber range_subscriber;

dosquare();
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void rangeCallback( const sensor_msgs::Range::ConstPtr& msg);
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
void moveGoal(geometry_msgs::Pose2D  goal_pose, double distance_tolerance,double speed);
void setDesiredOrientation (double desired_angle_radians);
void gridClean();
void spiralClean();
void SetSubscriber();

};

#endif // DOSQUARE_H
