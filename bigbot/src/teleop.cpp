#include <ros/ros.h>
#include "ros/console.h"
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_S 0x53
#define KEYCODE_s 0x73
#define KEYCODE_F 0x46
#define KEYCODE_f 0x66
#define KEYCODE_G 0x47
#define KEYCODE_g 0x67

#define KEYCODE_H 0x48
#define KEYCODE_h 0x68
#define KEYCODE_I 0x49
#define KEYCODE_i 0x69



class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(0.1),
  a_scale_(0.4)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("L left R Right U up D down S stop F lin faster by .1 G ang faster by .1");
  puts("H lin slower by .1 I ang slower by .1");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_INFO("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_INFO("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_INFO("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_INFO("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_INFO("Stop");
        linear_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_s:
        ROS_INFO("Stop");
        linear_ = 0.0;
        dirty = true;
        break;
      case KEYCODE_F:
		l_scale_ =l_scale_+ .01;
        ROS_INFO("Fast linear %0.4f",l_scale_);
        dirty = true;
        break;		
      case KEYCODE_f:
		l_scale_ =l_scale_+ .01;
        ROS_INFO("Fast linear %0.4f",l_scale_);
        dirty = true;
        break;		
      case KEYCODE_G:
		a_scale_ = a_scale_+ .01;		
        ROS_INFO("Fast angular %0.4f",a_scale_);
        dirty = true;
        break;		
      case KEYCODE_g:
		a_scale_ = a_scale_+ .01;		
        ROS_INFO("Fast angular %0.4f",a_scale_);
        dirty = true;		
        break;
      case KEYCODE_H:
		l_scale_ = l_scale_- .01;
        ROS_INFO("Slow linear %0.4f",l_scale_);
        dirty = true;
        break;		
      case KEYCODE_h:
		l_scale_ = l_scale_- .01;
        ROS_INFO("Slow linear %0.4f",l_scale_);
        dirty = true;
        break;		
      case KEYCODE_I:
		a_scale_ = a_scale_- .01;		
        ROS_INFO("Slow angular %0.4f",a_scale_);
        dirty = true;
        break;		
      case KEYCODE_i:
		a_scale_ = a_scale_- .01;		
        ROS_INFO("Slow angular %0.4f",a_scale_);
        dirty = true;		
        break;

    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}
