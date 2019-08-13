
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include "tf/transform_broadcaster.h"
//#include <tf/transform_broadcaster.h>

#include "SlaveParse.h"
#include "sensordata.h"
#include "serial.h"

//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <termios.h>
//#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// this is from r2serial
//#include <pthread.h>
//#include <sys/time.h>
#include <termios.h>

#include "botconfig.h"

#define BAUDRATE B57600   // Change as needed, keep B

/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyUSB0"

#define NUMOUTGOING 60

// 3.14159
#define WHEELRADIUS .09763 // in meters 7 11/16in /2
#define BASE 0.76708  //(30.2 inch)

// 0.613434717 meter =  1 rotation

// convert m/s to radians
// (v(m/s) * 2PI)/circumferance
// (.1ms * 2 * 3.14159) /.613434717 = 1.0242
// (.5   * 2 * 3.14159) /.613434717 = 5.1213
// (1    * 2 * 3.14159) /.613434717 = 10.242

// another website said
// v (mps)/wheel radius
// .1/.09763 =1.0242
// .5/.09763 = 5.1213
// 1./.09763 = 10.242

// default min and max PWM
#define MINLEFTDEFAULT 200
#define MAXLEFTDEFAULT 500
#define MINRIGHTDEFAULT 200
#define MAXRIGHTDEFAULT  570



// ros debug flag
#define LOG_INFO_DEBUG 1
/*
state 0 wait for build message
state 1 send echo and wait for response
state 2 send p wait for config
state 3 send sonar
state 4 send gain
state 5 send sensor state = 1
state 6 wait for sensor readingd      */


class MotorControl
{

protected:


int CommandID=0;  // for ids for commands 
int State =0; // need to use this to do init
char OutgoingBuffer[NUMOUTGOING];
int sensorEnd; // set based on config


BotConfig mconfig;
nav_msgs::Odometry msg;
sensor_msgs::Range  sonarmsg;

sensorData msensor;
Serial mComm;//= new Serial();

short MinLEFT;
short MaxLEFT;
short MinRIGHT;
short MaxRIGHT;
short BMinLEFT;
short BMaxLEFT;
short BMinRIGHT;
short BMaxRIGHT;



const double MaxSpeed =13.3;//4.7; // in radian
const double MinSpeed =0.1;
const double LeftSlope= 231; //(MaxLEFT-MinLEFT)/(MaxSpeed- MinSpeed) ;
const double RightSlope = 286;//(MaxRIGHT - MinRIGHT) / (MaxSpeed - MinSpeed);
const double LeftY = 154;//MaxLEFT - (LeftSlope * MaxSpeed);
const double RightY =143;// MaxRIGHT - (RightSlope * MaxSpeed);

const double RIGHTMETERPERCLICK = 0.0004786; // (.5 rotation plus 8cm/ 808 original values in feet 0.015408; // original 0.016; //  oover rode orig value
const double LEFTMETERPERCLICK =  0.000937; //(.5rotation (0.306713432) plus 4cm / 370 clicks)
//0.008863831;// o;ringinal values in feet 0.02908081; //0.030287; original
//.5 rotation plus 4cm

int lastleftclick=0,lastrightclick=0, deltaLeft, deltaRight,SlaveTimestamp = 0;



// prior requested speed values in feet per second
double mLastRequestedLeftSpeed = 0, mLastRequestedRightSpeed = 0;
// new requested speed
double mRequestedLeftSpeed = 0, mRequestedRightSpeed = 0;
// hold pwm values between set up request and get command call
short mLeftPWM = 0, mRightPWM = 0, mLastLeftPWM = 0, mLastRightPWM = 0;
// sent mLastDirection to invalid vaule to force sending direction for first call
char mDirection = (char)'F', mLastDirection = 'Z';


bool mNeedNewCommand = false;
bool mNeedDirection = false;
ros::Time mLastCommndSent = ros::Time::now();

geometry_msgs::Pose2D bot_pose; // hold current position of bot.

int angularVelocitytoPWM(int MotorMax, int MotorMin,double VelocityMax,double VelocityMin,double angularVelocity);

bool ReadConfig();
void CalcOdom();

void RequestConfig();
void SetSonar( int Index);
void SetGain();
bool StartSensor();
bool SetEcho(int echo);

public:
std::string base_frame =  "base_link";
void SetPose( geometry_msgs::Pose2D new_pose);
void CloseSerial();
bool InitSerial(char * port, int baud);
ros::Publisher* ptr_pub;
ros::Publisher* ptr_pubR;

tf::TransformBroadcaster* ptr_broadcaster;
void readLoop();
void TwistCallBack(const geometry_msgs::Twist::ConstPtr& msg);
void SetPWMMinMax(int minleft,int maxleft,int minright,int maxright,int bminleft,int bmaxleft,int bminright,int bmaxright);
};



void MotorControl::SetPWMMinMax(int minleft,int maxleft,int minright,int maxright,int bminleft,int bmaxleft,int bminright,int bmaxright)
{
MinLEFT = (short) minleft;
MaxLEFT =(short) maxleft;
MinRIGHT =(short) minright;
MaxRIGHT =(short) maxright;


BMinLEFT = (short)bminleft ;
BMaxLEFT = (short)bmaxleft;
BMinRIGHT = (short)bminright;
BMaxRIGHT = (short)bmaxright;


ROS_INFO("SetPWMMinMax %d %d %d %d  %d %d %d %d",MinLEFT,MaxLEFT,MinRIGHT,MaxRIGHT,BMinLEFT,BMaxLEFT,BMinRIGHT,BMaxRIGHT);

}
// need to figure these out for left and right wheel

int MotorControl::angularVelocitytoPWM(int MotorMax, int MotorMin,double VelocityMax,double VelocityMin,double angularVelocity)
{
int motorCmd;
double slope,yintercept;
if (angularVelocity == 0)
    {
    return 0;
    }
slope = (MotorMax - MotorMin) / (VelocityMax - VelocityMin );
yintercept = MotorMax - (slope * VelocityMax);



motorCmd = (int) (slope * fabs(angularVelocity) + yintercept);
if (motorCmd > MotorMax)
{
motorCmd = MotorMax;
}
if (motorCmd < MotorMin)
{
motorCmd = MotorMin;
}
//motorCmd = -motorCmd;

/*
lets just abs not matter what
if (angularVelocity > 0) // positive angular velocity
    {
    motorCmd = (int)(slope * angularVelocity + yintercept);
    if (motorCmd > MotorMax)
        {
        motorCmd = MotorMax;
        }
      if (motorCmd < MotorMin)
        {
        motorCmd = MotorMin;
        }
    }
else // negative angular velocity
    {
    motorCmd = (int) (slope * fabs(angularVelocity) + yintercept);
    if (motorCmd > MotorMax)
        {
        motorCmd = MotorMax;
        }
    if (motorCmd < MotorMin)
        {
        motorCmd = MotorMin;
        }
    motorCmd = -motorCmd;
    }
*/
return motorCmd;

}


//----------------------------------------------------------------------------


//Process ROS command message, send to uController
void MotorControl::TwistCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
    int nbrBytes;
    //only execute code if slave is ready
    if (State == 6)
    {
    double TargetLinearVelocity,targetAngularVelocity,leftVelocity, rightVelocity,leftAngular;
    bool sendspeed = false,sentdirection=false;

    TargetLinearVelocity= msg->linear.x;
    targetAngularVelocity=msg->angular.z;
    //ROS_INFO("TargetLinearVelocity %f  targetAngularVelocity %f", TargetLinearVelocity,targetAngularVelocity );

    //calculate tangential velocity
    leftVelocity = ((2 * TargetLinearVelocity) - (targetAngularVelocity * BASE))/2.0;
    rightVelocity = ((2 * TargetLinearVelocity) + (targetAngularVelocity * BASE))/2.0;

    ros::Time tnow =ros::Time::now();
    double diff = (tnow - mLastCommndSent).toSec();


    mLastRequestedLeftSpeed = mRequestedLeftSpeed;
    mLastRequestedRightSpeed = mRequestedRightSpeed;

    // calculate angular for left and right wheel
    //leftAngular = leftVelocity/WHEELRADIUS;
    //rightAngular = rightVelocity/WHEELRADIUS;
    mRequestedLeftSpeed =  leftVelocity/WHEELRADIUS;
    mRequestedRightSpeed = rightVelocity/WHEELRADIUS;

    if (mRequestedLeftSpeed != mLastRequestedLeftSpeed)
    {
    mLastLeftPWM= mLeftPWM;
    // calculate new pwm
    //    mleftpwm = angularVelocitytoPWM(MaxLEFT, MinLEFT,leftVelocityMaxRads,leftVelocityMinRads,mRequestedLeftSpeed);
    if (mRequestedLeftSpeed < 0)
	{
    	mLeftPWM = angularVelocitytoPWM(BMaxLEFT, BMinLEFT,MaxSpeed,MinSpeed,mRequestedLeftSpeed);
	}
    else
	{
    	mLeftPWM = angularVelocitytoPWM(MaxLEFT, MinLEFT,MaxSpeed,MinSpeed,mRequestedLeftSpeed);
	}
    sendspeed = true;
    }
    else
    {
        if (diff > 1)
        {
                sendspeed = true;
        }
    }

    if (mRequestedRightSpeed != mLastRequestedRightSpeed)
    {
    // save old values
    mLastRightPWM = mRightPWM;
    //    mrightpwm = angularVelocitytoPWM(MaxRIGHT, MinRIGHT,rightVelocityMaxRads,rightVelocityMinRads,mRequestedRightSpeed);
    if (mRequestedRightSpeed < 0)
	{
	mRightPWM = angularVelocitytoPWM(BMaxRIGHT, BMinRIGHT,MaxSpeed,MinSpeed,mRequestedRightSpeed);
	}
    else
	{
	mRightPWM = angularVelocitytoPWM(MaxRIGHT, MinRIGHT,MaxSpeed,MinSpeed,mRequestedRightSpeed);
	}
    }
    else
    {
        if (diff > 1)
        {
                sendspeed = true;
        }
    }




    if (mRequestedLeftSpeed  < 0 ||  mRequestedRightSpeed < 0)
        {
        if (mRequestedLeftSpeed <  0)
            {
            if (mRequestedRightSpeed < 0)
                {
                mDirection = 'B';
                }
            else
                {
                mDirection ='L';
                }
            }
        else
            {
            mDirection ='R';
            }
        }
    else
        {
        mDirection ='F';
        }

    if (mDirection != mLastDirection)
        {
        nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"D %d %c\n",CommandID,mDirection);
        mComm.SerialWrite(OutgoingBuffer,nbrBytes);
        }


    if (sendspeed)
    {
        nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"M %d %d %d\n",CommandID,mLeftPWM,mRightPWM);
        mComm.SerialWrite(OutgoingBuffer,nbrBytes);
    }

  ROS_INFO("TwistCallBack: lin %f ang %f lv %f rv %f rls %f rrs %f L %d R %d Dir %c %c", TargetLinearVelocity , targetAngularVelocity,leftVelocity, rightVelocity,
           mRequestedLeftSpeed, mRequestedRightSpeed, mLeftPWM,mRightPWM,mDirection,mLastDirection);
    mLastDirection = mDirection;
    }
} //ucCommandCallback

void MotorControl::CalcOdom()
{
   int localleftclick,localrightclick,dt;
   double  VelLeft, VelRight, Velth, deltaX, deltaY,deltath,linear, angular;


//        long lastleftclick=0,lastrightclick=0, deltaLeft, deltaRight;

   //get time period for tick difference
   dt = msensor.getTimeStamp()-SlaveTimestamp ;
   // first time through  SlaveTimestamp=0 default to 100 ms
   if (dt > 200)
   {
       dt = 100;
   }
   SlaveTimestamp = msensor.getTimeStamp();

   localleftclick = msensor.getLeftClick();
   localrightclick = msensor.getRightClick();
   //ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"CalcOdom localleftclick %d lastleftclick %d localrightclick %d lastrightclick %d",localleftclick,lastleftclick,localrightclick,lastrightclick );

   // how many click did wee see.
   deltaLeft =  (localleftclick -lastleftclick);
   deltaRight = (localrightclick - lastrightclick);
   // save current click count for next run
   lastleftclick = localleftclick;
   lastrightclick = localrightclick;


   // need to figureout how to get lape time correctly
   //dt = (e.current_real -e.last_real).toSec();
   // use clicks to calculate distance moved
   VelLeft = (deltaLeft * LEFTMETERPERCLICK ); //0.008863831
   VelRight = (deltaRight * RIGHTMETERPERCLICK ); //0.004696358


   linear = (VelLeft + VelRight)/2.0;
   angular = (VelRight - VelLeft ) /BASE; // divide by wheel base 0.12005

   //ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"CalcOdom linear %f angular %f ", linear, angular);


   double dtSeconds = dt/1000.0;
   VelLeft = VelLeft/dtSeconds;
   VelRight = VelRight/dtSeconds;
   Velth = angular/dtSeconds;

   //ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"CalcOdom Vel l %f r %f th %f ",VelLeft,VelRight, Velth);



   if (fabs(angular) < 1e-6)
   {
   const double direction = bot_pose.theta + angular * 0.5;

   /// Runge-Kutta 2nd order integration:
   deltaX =linear * cos(direction);
   deltaY =linear * sin(direction);
   bot_pose.x += deltaX;
   bot_pose.y += deltaY;
   bot_pose.theta += angular *dt;

   }
   else
   {
   /// Exact integration (should solve problems when angular is zero):
   const double oldth = bot_pose.theta;
   const double r = linear/angular;
   bot_pose.theta += angular;
     deltaX= r * (sin(bot_pose.theta) - sin(oldth));
     deltaY = -r * (cos(bot_pose.theta) - cos(oldth));
   bot_pose.x  +=  deltaX;
   bot_pose.y  += deltaY;
   }

   ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftclick > 0 || localrightclick > 0) ,"locallc %d lastlc %d lrc %d lastrc %d linear %f angular %f Vel l %f r %f th %f | delta x %f y %f new X %f y %f pwm L %d R %d",localleftclick,lastleftclick,localrightclick,lastrightclick,linear, angular,VelLeft,VelRight, Velth, 		deltaX,deltaY, bot_pose.x, bot_pose.y,msensor.getLeftMotor(),msensor.getRightMotor());




   /*
   https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/diff_drive_controller/src/diff_drive_controller.cpp

   */


   /**
    * This is a message object. You stuff it with data, and then publish it.
    */
//x,y,th, VelLeft, VelRight, Velth, deltaLeft, deltaRight
   msg.pose.pose.position.x= bot_pose.x;
   msg.pose.pose.position.y= bot_pose.y;
   msg.pose.pose.position.z= 0;


   //---------------------------------------------------------------------------------------
   /*
   http://answers.ros.org/question/11973/gathering-wheel-encoder-data-and-publishing-on-the-odom-topic/
   */
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(bot_pose.theta);

   //first, we'll publish the transform over tf
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = ros::Time::now();
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = base_frame;

   odom_trans.transform.translation.x = bot_pose.x;
   odom_trans.transform.translation.y = bot_pose.y;
   odom_trans.transform.translation.z = 0.0;
   odom_trans.transform.rotation = odom_quat;

   //send the transform
   ptr_broadcaster->sendTransform(odom_trans);

   //next, we'll publish the odometry message over ROS
   //  nav_msgs::Odometry odom;
   msg.header.stamp = odom_trans.header.stamp;//CurrentEncoderTime;
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


       //publish the message

   //---------------------------------------------------------

           ptr_pub->publish(msg);
   //----------------------------------------------------------------------

   // send sonar
   sonarmsg.range = msensor.GetFrontRange();
   ptr_pubR->publish(sonarmsg);

}


void MotorControl::SetPose( geometry_msgs::Pose2D new_pose)
{
    bot_pose.x = new_pose.x;
    bot_pose.y = new_pose.y;
    bot_pose.theta = new_pose.theta;

}

bool MotorControl::SetEcho(int echo)
{
    int nbrBytes;
CommandID++;
if (echo == 1)
    {
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"E %d 1\n",CommandID);
    }
else
{
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"E %d 0\n",CommandID);
}
mComm.SerialWrite(OutgoingBuffer,nbrBytes);


return true;
}


bool MotorControl::StartSensor()
{
    int nbrBytes;
    ROS_INFO("Sending H B 1 Reset encoder counter and turn on sensor");
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"H B 1\n");
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);


    return true;
}

void MotorControl::RequestConfig()
{
    int nbrBytes;
    ROS_INFO("Sending P");
    nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"P\n");
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);
}
void MotorControl::SetSonar( int Index)
{
int nbrBytes;
    ROS_INFO("Sending A S %d %d", Index,mconfig.SonarAddress(Index));
    nbrBytes=snprintf(OutgoingBuffer, NUMOUTGOING,"A S %d %d\n", Index+1,mconfig.SonarAddress(Index));
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);

}
void MotorControl::SetGain(){
    int nbrBytes;
    ROS_INFO("Sending G %d 80 10\n",CommandID);
    nbrBytes=snprintf(OutgoingBuffer, NUMOUTGOING,"G %d 80 10\n", CommandID);
    mComm.SerialWrite(OutgoingBuffer,nbrBytes);
}

void MotorControl::readLoop()
{
    char *ptr;
    int strLen,BeginCommand;

    if (mComm.SerialRead())
    {

     if ( mComm.SerialLineAvailable())
     {
       ptr = mComm.SerialGetData();
       strLen = strlen(ptr);
       for (BeginCommand = 0;BeginCommand <=strLen && !(ptr[BeginCommand] == '$' || ptr[BeginCommand] == 'E' || ptr[BeginCommand] == '1' ) ; BeginCommand++)
       {
           ;
       }
       if (ptr[BeginCommand]=='$' && ptr[BeginCommand+1]=='0' )
       {
        if (ptr[BeginCommand+2] == '3')
        {
            // sensor
            if (State == 6 && ptr[(BeginCommand+strLen)-2] =='Z')
            {
 //                sensorData msensor;

                 // 0123456789112345678921234567893123456
                 // $03  2ff5   572   a9710d11dR ad3 688Z
                 //we have a sensor reading
               //  ROS_INFO("sensor reading");
                 msensor.getSensorData(&ptr[BeginCommand],mconfig);
                 // call odom I think
                 CalcOdom();
            }
            else
            {
                ROS_INFO("ELSE sensor reading");
            }



        }
        else if (ptr[BeginCommand+2] == '1')
        {
            // annoucement state 1
            ROS_INFO("Robot Build %s",&ptr[BeginCommand+13]);
            State=1;
            MotorControl::SetEcho(0);
        }
        else if (ptr[BeginCommand+2] == '2')
        {
            // error
            // error
        int slavestate = SlaveParse::ParseHex(&ptr[BeginCommand], 9, 2);
        int result = SlaveParse::ParseInt(&ptr[BeginCommand], 11, 3);
        ROS_ERROR("Slave State %d Error %d",slavestate,result);

        }
        else if (ptr[BeginCommand+2] == '7')
        {
            // config

            if (ptr[BeginCommand+24] == 'Z')
            {
                // we have the whole config pass it to the config object to parse
              mconfig =  BotConfig(&ptr[BeginCommand]);
              if (!mconfig.IsConfigValid() )
              {
                  // This can happen if slave code was updated the eeprom config gets cleared
                  // so we need to reset config and reread config
                  mconfig.ResetConfig();
                  for (int Index =0; Index < mconfig.NumberSonars();Index++ )
                  {
                   MotorControl::SetSonar(  Index);
                   State =3;
                  }
    
              }
              else
              {
                   ROS_INFO("motorControl BotConfig valid. %d sonars",mconfig.NumberSonars());
              }
              // set this for later use in message $03
              sensorEnd = (mconfig.HasCompass()? 4 :0) + (mconfig.NumberSonars() * 4) + 28;
              // now need to get range and gain every time.
              CommandID++;
              MotorControl::SetGain();
              State = 4;
              }
        }
        else if (ptr[BeginCommand+2] == '8')
            {
                // port value
                int slavestate = SlaveParse::ParseHex(&ptr[BeginCommand], 3, 2);
                ROS_INFO("Slave portvalue %d",slavestate);
            }
            else
            {
                     ROS_ERROR("unknown command %c",ptr[BeginCommand+2] );
            }


       }
       else if( ptr[BeginCommand] == 'E' )
       {
        State=2;
        MotorControl:RequestConfig();
       }
       else if( ptr[BeginCommand] == '1' )
       {
        State = 6;
        MotorControl::StartSensor();
       }
       else
       {
           ROS_ERROR("unknown command %s",ptr );

       }
    }
   }
}

void MotorControl::CloseSerial()
{
   mComm.SerialClose();
}

bool MotorControl::InitSerial(char * port, int baud)
{
    // init sonrarmsg
    sonarmsg.min_range = .043;
    sonarmsg.max_range = 3.0;  // reduced to 3 meters
    sonarmsg.radiation_type = sonarmsg.ULTRASOUND; //sensor_msgs::Range::ULTRASOUND
    sonarmsg.field_of_view= 1; // 1 radian =approx 60degrees

    return mComm.serialInit(port,baud);
}
int main(int argc, char **argv)
{
  char port[20];    //port name
  int baud;     //baud rate
  std::string serial_port (MODEMDEVICE);
  //"/dev/atmegaslave"
  
  //Initialize ROS
  ros::init(argc, argv, "motorControl");
  ros::NodeHandle rosNode;

  ROS_INFO("motorControl starting");

  MotorControl mc;
  geometry_msgs::Pose2D startPose;
  startPose.x=0.0;
  startPose.y=0.0;
  startPose.theta=0.0;
  mc.SetPose(startPose);
  //strcpy(port, MODEMDEVICE);
  //if (argc > 1)
  //{
  //   strcpy(port, argv[1]);
  //}
  baud = BAUDRATE;
  //if (argc > 2) {
  //  if(sscanf(argv[2],"%d", &baud)!=1) {
  //    ROS_ERROR("motorControl baud rate parameter invalid");
  //    return 1;
  //  }
  //}



  // read port from config
  rosNode.param<std::string>("/motorControl/port",serial_port,serial_port);

  ROS_INFO("connection initializing (%s) at %d baud", serial_port.c_str(), baud);

 // baud = 57600;

  //ROS_INFO("override connection initializing (%s) at %d baud", serial_port, baud);


  if (!mc.InitSerial((char *)serial_port.c_str(), baud) ) 
  {
    ROS_ERROR("unable to create a new serial port %s baud %d",port, baud);
    return 1;
  }
  ROS_INFO("serial connection successful");


  int minleft,maxleft,minright,maxright,LocalDefault,bminleft,bmaxleft,bminright,bmaxright;
  LocalDefault = MINLEFTDEFAULT;
  rosNode.param("/motorControl/pwm/forward/left/min",minleft,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/left/min",bminleft,LocalDefault);
  LocalDefault =MAXLEFTDEFAULT;
  rosNode.param("/motorControl/pwm/forward/left/max",maxleft,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/left/max",bmaxleft,LocalDefault);
  LocalDefault =MINRIGHTDEFAULT;
  rosNode.param("/motorControl/pwm/forward/right/min",minright,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/right/min",bminright,LocalDefault);
  LocalDefault =MAXRIGHTDEFAULT;
  rosNode.param("/motorControl/pwm/forward/right/max",maxright,LocalDefault);
  rosNode.param("/motorControl/pwm/backward/right/max",bmaxright,LocalDefault);

  mc.SetPWMMinMax(minleft, maxleft, minright, maxright,bminleft, bmaxleft, bminright, bmaxright);


  rosNode.param<std::string>("/motorControl/base_frame",mc.base_frame,mc.base_frame);


  //Subscribe to ROS messages

  ros::Subscriber sub = rosNode.subscribe("cmd_vel", 1000, &MotorControl::TwistCallBack, &mc);

  //Setup to publish ROS messages
  ros::Publisher pub = rosNode.advertise<nav_msgs::Odometry>("odom", 100);
  mc.ptr_pub = &pub;

  //Setup to publish ROS messages
  ros::Publisher pubR = rosNode.advertise<sensor_msgs::Range>("sensor", 100);
  mc.ptr_pubR = &pubR;
//  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


  // this is untested transform
  tf::TransformBroadcaster odom_broadcaster;

  mc.ptr_broadcaster =  &odom_broadcaster;
  // end of transform


// need to rip this out of class
  ROS_INFO("rcvThread: receive thread running");

  ros::Rate r(200);
  while (ros::ok()) {
     // process incoming
      mc.readLoop();
      ros::spinOnce();

  }
  //Create receive thread

//  pthread_t rcvThrID;   //receive thread ID
//  int err;
//  err = pthread_create(&rcvThrID, NULL, &MotorControl::rcvThread, &mc);
//  if (err != 0) {
//    ROS_ERROR("unable to create receive thread");
//    return 1;
//  }



  mc.CloseSerial();
  ROS_INFO("r2Serial stopping");
  return 0;
}
