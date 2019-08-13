
#include "ros/ros.h"
#include "ros/console.h"
#include <geometry_msgs/Vector3.h>

int main(int argc, char **argv)
{
ros::init(argc, argv, "pubpwm");
ros::NodeHandle rosNode("~");

ROS_INFO("pubpwm starting");

 int delay = 10;
 int refreshrate = 5;
 int startLeft = 100;
 int startRight = 0;
 int endLeft = 250;
 int endRight = 0;
 int delaycounter;
 int incr = 1;
 rosNode.getParam("startLeft", startLeft );
if (argc >= 6)
  {
  startLeft  = atoi(argv[1]);
  startRight  = atoi(argv[2]);
  endLeft  = atoi(argv[3]);
  endRight  = atoi(argv[4]);
  delay = atoi(argv[5]);
  }

if (startLeft > endLeft || startRight > endRight)
{
  incr = -1;
}
// rosNode.param("startLeft",startLeft,startLeft);
// rosNode.getParam("endLeft", endLeft);
 geometry_msgs::Vector3 pwm;
// sp=rosNode.getParamNames();

ros::Publisher pub = rosNode.advertise<geometry_msgs::Vector3>("/pwm", 100);
pwm.x= startLeft;  // left pwm
pwm.y = startRight;  // right pwm
pwm.z = 0;   // sent reset encoder value
// delay = seconds refreshrate = how many time per seoncd to push
ros::Rate r(refreshrate);
delaycounter =delay* refreshrate;
while (ros::ok()) {

  ROS_INFO("pubpwm pub %lf %lf %lf",pwm.x,pwm.y,pwm.z);
  pub.publish(pwm);
  ros::spinOnce();
  r.sleep();
  pwm.z = 0;
  delaycounter--;
  if (delaycounter <= 0)
  {
    delaycounter =delay* refreshrate;
    //pwm.z = 1; // sent reset encoder value
    // increment left pwm if it did not start at 0
    if (pwm.x > 0)
    {
      pwm.x +=incr;
    }
    // increment right pwm if it did not start at 0
    if (pwm.y > 0)
    {
      pwm.y +=incr;
    }
    if (incr > 0)
    {
    if (pwm.x > endLeft || pwm.y > endRight )
    {
      pwm.x=0;
      pwm.y=0;
      pwm.z=0;
      pub.publish(pwm);
      break;
    }
    }
    else
    {
      if (pwm.x < endLeft || pwm.y < endRight )
      {
        break;
        pwm.x=0;
        pwm.y=0;
        pwm.z=0;
        pub.publish(pwm);
        break;

      }
    }
  }
}
}
