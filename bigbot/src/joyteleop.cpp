#include <ros/ros.h>
//#include <turtlesim/Velocity.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19


class joyteleop
{
public:

    joyteleop();
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    // configure  axis index
    int linear_axes, angular_axes;
   
    // config action button index
    int linearlock,angularlock,linearunlock,angularunlock;
    int deadman_button;
    
    // orientation
    bool invert_linear=false,invert_angular=false;
   
    
    double maxlinear=0,maxangular=0;
    double l_scale_, a_scale_;
    int countsticky;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    bool linearsticky = false;
    bool angularsticky = false;
    bool deadman_on = false;

    bool linearset = false;
    bool angularset = false;
};

joyteleop::joyteleop():
   linear_axes(PS3_AXIS_STICK_LEFT_UPWARDS),
   angular_axes(PS3_AXIS_STICK_RIGHT_LEFTWARDS),
   deadman_button(6),
   linearlock(5),
   angularlock(7),
   linearunlock(2),
   angularunlock(3)
{
nh_.param("linear_axes", linear_axes, linear_axes);
nh_.param("angular_axes", angular_axes, angular_axes);
nh_.param("deadman_button", deadman_button, deadman_button);
nh_.param("linearlock", linearlock, linearlock);
nh_.param("angularlock", angularlock, angularlock);
nh_.param("linearunlock", linearunlock, linearunlock);
nh_.param("angularunlock", angularunlock, angularunlock);

nh_.param("invert_linear", invert_linear, invert_linear);
nh_.param("invert_angular", invert_angular, invert_angular);

nh_.param("scale_angular", a_scale_, a_scale_);
nh_.param("scale_linear", l_scale_, l_scale_);

vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &joyteleop::joyCallback, this);
int a=0,b=0;
if (invert_linear)
	{
	a =1;
	}
if (invert_angular)
	{
	b=1;
	} 
  ROS_INFO("linear_axes %d angular_axes %d deadman_button %d linearlock %d angularlock %d linearunlock %d angularunlock %d invert_linear %d invert_angular %d scale_angular %f  scale_linear %f ",linear_axes,angular_axes,deadman_button,linearlock,angularlock,linearunlock,angularunlock,a,b,a_scale_,l_scale_);

}
// require dead button
// allow sticky values for stick values


void joyteleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  double newlinear,newanglar;
  //make sure deadman is pressed
  deadman_on = joy->buttons[deadman_button];
  // calculate new values
  newlinear = l_scale_ * joy->axes[linear_axes];
  newanglar = a_scale_ * joy->axes[angular_axes];

  // inver sign if needed to match forward and left position.
  if (invert_linear)
   {
	newlinear *= -1; 
   }

  if (invert_angular)
   {
   newanglar *= -1;
   }

  // ignore anything too small because joystick does not center well
  if (fabs(newanglar) < .05)
  {
     newanglar = 0;
  }
  if (fabs(newlinear) < .05)
  {
     newlinear = 0;
  }

  // adjust minimums based on both values
  if (fabs(newanglar) > 0 && fabs(newanglar) < 0.2  && fabs(newlinear) < 0.1)
    {
        if (newanglar > 0)
        {
        newanglar = .2;
        }
        else
        {
        newanglar = -.2;
        }
    }


  if (fabs(newlinear) > 0 && fabs(newlinear) < 0.1 && fabs(newanglar) < .2 )
    {
        if (newlinear > 0)
        {
        newlinear = 0.1;
        }
        else
        {
            newlinear = -0.1;
        }
    }   

  if (joy->buttons[linearunlock]> 0)
   {
   ROS_INFO("\n****** linear clear sticky ************\n");
   linearsticky = false;
   linearset = false;
   }

  if (joy->buttons[angularunlock]> 0)
   {
   ROS_INFO("\n****** angular clear sticky ************\n");

   angularsticky = false;
   angularset = false;
   }


  if (joy->buttons[linearlock]> 0 && !linearsticky )
   {
   ROS_INFO("\n****** set linear sticky ************\n");
   linearsticky = true;
   linearset = false;
   countsticky++;
   }

  if (linearsticky)
   {
   if (!linearset)
     {
      maxlinear = newlinear;
      linearset = true;     
      ROS_INFO("\n****** set linearset %f ************\n",maxlinear);

     } 
   }

  //     vel.linear.x =maxlinear;
  if (joy->buttons[angularlock]> 0 && !angularsticky )
   {
   ROS_INFO("\n****** set angular sticky ************\n");

   angularsticky = true;
   angularset = false;
   countsticky++;
   }

  if (angularsticky)
   {
   if (!angularset)
     {
     maxangular = newanglar;
     angularset = true;
     ROS_INFO("\n****** set linearset %f ************\n",maxangular);

     }
   }

  if (deadman_on)
  {
    if (linearsticky)
    {
        vel.linear.x = maxlinear;
    }
    else
    {
        vel.linear.x = newlinear;
    }
    if (angularsticky)
    {
        vel.angular.z = maxangular;
    }
    else
    {
        vel.angular.z = newanglar;
    }
  }
  else
  {
      vel.linear.x = 0;
      vel.angular.z = 0;
  }
  if (vel.linear.x > 0 || vel.linear.x < 0 || vel.angular.z > 0 || vel.angular.z < 0 )
    {
    ROS_INFO("linear %f angular %f",vel.linear.x,vel.angular.z);
    }
  vel_pub_.publish(vel);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joyteleop");
  joyteleop joyteleop;
  ros::spin();
}
