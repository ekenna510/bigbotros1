// common
#define WHEELRADIUS .041125 // in meters
#define TICKSPERROTATION 24
#define METERPERTICK (WHEELRADIUS * 2 *3.14)/TICKSPERROTATION // .01046 meter
#define BASE 0.12005  //(120.05 mm)

// box in inches x =10 y = 8 y 10 (x= forward y = left z = up
// 
//encoder part

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
// for turning sonar to laser
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

extern "C" {
#include "bcm2835.h"
}
#include <sstream>

// Set internal debug to info set to 1 
#define LOG_INFO_DEBUG 1 
#define  PWM_DIRCHANNEL0_A RPI_V2_GPIO_P1_22
#define  PWM_DIRCHANNEL0_B RPI_V2_GPIO_P1_24
#define  PWM_DIRCHANNEL1_A RPI_V2_GPIO_P1_26
#define  PWM_DIRCHANNEL1_B RPI_V2_GPIO_P1_23
#define  LEFTENCODER  RPI_V2_GPIO_P1_11
#define  RIGHTENCODER RPI_V2_GPIO_P1_15

// encoder global variables 

unsigned char LeftBit,RightBit,LastLeft,LastRight;
// wheel encoder 
long leftwheel,rightwheel,lastleftwheel,lastrightwheel, deltaLeft, deltaRight;
boost::mutex mtx; 
double x, y, th, VelLeft, VelRight, Velth, deltaX, deltaY,deltath,linear, angular;

ros::Publisher* ptr_pub; 
tf::TransformBroadcaster* ptr_odom_broadcaster;

ros::Time CurrentEncoderTime,LastEncoderTime;	




//motorpart

#include "geometry_msgs/Twist.h"

// define pwm pins
#define PWM0 RPI_V2_GPIO_P1_12
#define PWM1 RPI_V2_GPIO_P1_35  
#define PWM_CHANNEL0 0
#define PWM_CHANNEL1 1
#define PWMRANGE 1024


#define  PWM_DIRCHANNEL0_A RPI_V2_GPIO_P1_22
#define  PWM_DIRCHANNEL0_B RPI_V2_GPIO_P1_24
#define  PWM_DIRCHANNEL1_A RPI_V2_GPIO_P1_26
#define  PWM_DIRCHANNEL1_B RPI_V2_GPIO_P1_23


int leftpwm,Lastleftpwm,rightpwm,Lastrightpwm;
int stateLoopStartDelay;

// max count per pwm test 850 = 84 count / 24 clicks per rotation 
//        = 3.5 rotation * 2pi = 21.98 / 5 seconds = 4.398 radians per second
// min count per pwm test 300 = 25 count /24 clicks per rotoation 
// = ~1 rotation *2pi = 6.283/5seconds = 1.2566 radians per seconds
int leftMotorCmdMin =250;
int leftMotorCmdMax =850;
int rightMotorCmdMin =230;
int rightMotorCmdMax =850;
double leftVelocityMinRads =1.256;
double leftVelocityMaxRads =4.398;
double rightVelocityMinRads =1.256;
double rightVelocityMaxRads =4.398;

// direction
char Direction='S';
char LastDirection='S';

// sonar
#include "sensor_msgs/Range.h"
#include <ros/console.h>

#define FRONT 				0x76 /*!<sonar 4 address 1 long 6 short flashes*/
#define LEFTFRONT 			0x70 /*!<sonar 1 address 1 long flashes */
#define LEFTBACK 			0x71 /*!<sonar 2 address 1 long 1 short flashes*/
#define RIGHT 				0x75 /*!<sonar 3 address 1 long 5 short flashes */

#define RANGE_IN 0x50
#define RANGE_CM 0x51
#define RANGE_US 0x52

#define MAX_SONAR_CM 2540
#define MAX_SONAR_IN 100
uint8_t GlobalDebug;

//ros::Publisher* ptr_sonarpub; 

//sensor_msgs::Range FrontMsg,LeftFMsg,LeftBMsg,RightMsg;
uint8_t Measurement;

// convert sonar to laser variables
// assume sonar is 28deg beam left front right is assumed at base_link 
// left and right are centered on 90 degrees so 
// we are going from 104 to -104 degrees with 0 degrees out front
// need to add offsets to get to base_link

#define NUM_READINGS 180
#define SIDE_OFFSET .0635
#define FRONT_OFFSET .05010
double ranges[NUM_READINGS];
double intensities[NUM_READINGS];

//Scan message

sensor_msgs::LaserScan scan;

// publisher
ros::Publisher* ptr_scan_pub;

//-- Motor routines -------------------------------
int SetDirection(char direction)
{
	switch (direction)
	{
		case 'F':
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, LOW);  // left wheel forward
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, HIGH);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, HIGH);  // right wheel Backward
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, LOW);		
		ROS_DEBUG("for");
		break;
		case 'M':
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, LOW);  // left wheel stop
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, LOW);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, LOW);  // right wheel forward
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, HIGH);		
		ROS_DEBUG("bearl");
		break;
		case 'L':
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, HIGH); // left wheel backward
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, LOW);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, HIGH); // right wheel Backward
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, LOW);			
		ROS_DEBUG("spinl");
		break;		
		case 'Q':
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, LOW);  // left wheel forward
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, HIGH);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, LOW);  // right wheel stop
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, LOW);		
		ROS_DEBUG("bearr");
		break;
		case 'R':
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, LOW);  // left wheel forward
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, HIGH);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, LOW);  // right wheel forward
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, HIGH);		
		ROS_DEBUG("spinr");
		break;
		
		
		case 'B':
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, HIGH);  // left wheel backward
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, LOW);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, LOW);  // right wheel forward
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, HIGH);	
		ROS_DEBUG("bac");
		break;
		
		case 'S':
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, LOW); //Left brake
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, LOW);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, LOW); //right brake
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, LOW);		
		ROS_DEBUG("stop");
		break;
		
		default:  // if unknown brake
		bcm2835_gpio_write(PWM_DIRCHANNEL0_A, LOW);
		bcm2835_gpio_write(PWM_DIRCHANNEL0_B, LOW);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_A, LOW);
		bcm2835_gpio_write(PWM_DIRCHANNEL1_B, LOW);				
		ROS_DEBUG("unknown stop");

	}
	return 0;
}

/**
 * moorerobot.com/blog/post/4
 */

// need to figure these out for left and right wheel

int angularVelocitytoPWM(int MotorMax, int MotorMin,double VelocityMax,double VelocityMin,double angularVelocity)
{
int motorCmd;
double slope,yintercept;
if (angularVelocity == 0)
	{
	return 0;
	}
slope = (MotorMax - MotorMin) / (VelocityMax - VelocityMin );
yintercept = MotorMax - (slope * VelocityMax);

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
	motorCmd = (int) (slope * abs(angularVelocity) + yintercept);
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
return motorCmd;
	
}




void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
double TargetLinearVelocity,targetAngularVelocity,leftVelocity, rightVelocity,leftAngular,rightAngular,slope,yintercept;
unsigned long int pwm;

TargetLinearVelocity= msg->linear.x;
targetAngularVelocity=msg->angular.z;
//ROS_INFO("TargetLinearVelocity %f  targetAngularVelocity %f", TargetLinearVelocity,targetAngularVelocity );

//calculate tangential velocity
leftVelocity = ((2 * TargetLinearVelocity) - (targetAngularVelocity * BASE))/2.0;
rightVelocity = ((2 * TargetLinearVelocity) + (targetAngularVelocity * BASE))/2.0;

// calculate angular for left and right wheel
leftAngular = leftVelocity/WHEELRADIUS;
rightAngular = rightVelocity/WHEELRADIUS;

// save old values
Lastleftpwm= leftpwm;
Lastrightpwm = rightpwm;
LastDirection = Direction;

// calculate new pwm
leftpwm = angularVelocitytoPWM(leftMotorCmdMax, leftMotorCmdMin,leftVelocityMaxRads,leftVelocityMinRads,leftAngular);
rightpwm = angularVelocitytoPWM(rightMotorCmdMax, rightMotorCmdMin,rightVelocityMaxRads,rightVelocityMinRads,rightAngular);

if (leftAngular  < 0 ||  rightAngular < 0)
	{
	if (leftAngular <  0)
		{
		if (rightAngular < 0)
			{
			Direction = 'B';
			}
		else
			{
			Direction ='L';
			if (leftpwm < 375)
				{
				leftpwm = 375;				
				}
			if (rightpwm < 375)
				{
				rightpwm = 375;				
				}
			}
		}
	else
		{
		Direction ='R';
			if (leftpwm < 375)
				{
				leftpwm = 375;				
				}
			if (rightpwm < 375)
				{
				rightpwm = 375;				
				}		
		}
	}
else
	{
	Direction ='F';
	}

if (Direction != LastDirection)
	{
	SetDirection(Direction);
	}

ROS_INFO("TargetLinearVelocity %f  targetAngularVelocity %f Leftpwm %d rightpwm %d direction %c", TargetLinearVelocity,targetAngularVelocity,leftpwm, rightpwm,Direction);

if (Lastleftpwm != leftpwm)
	{
	pwm = (unsigned long int) abs(leftpwm);			
	bcm2835_pwm_set_data(PWM_CHANNEL0, pwm); 
	}
if (Lastrightpwm != rightpwm)
	{
	pwm = (unsigned long int) abs(rightpwm);			
	bcm2835_pwm_set_data(PWM_CHANNEL1, pwm);				
	}	
}


//-- encoder functions  --------------------------
	
	void CheckEncoder(const ros::TimerEvent& e)
	{
		//ROS_INFO("CheckEncoder triggered");
	
		// grab encoder pin status
		LeftBit = bcm2835_gpio_lev(LEFTENCODER);
		RightBit = bcm2835_gpio_lev(RIGHTENCODER);
		// since we only have 1 encoder we need to track direction
		int LeftDirectionA,LeftDirectionB,RightDirectionA,RightDirectionB;
		// read current direction pins
		// see driver code for more details.
		LeftDirectionA =bcm2835_gpio_lev(PWM_DIRCHANNEL0_A);
		LeftDirectionB =bcm2835_gpio_lev(PWM_DIRCHANNEL0_B);
		RightDirectionA =bcm2835_gpio_lev(PWM_DIRCHANNEL1_A);
		RightDirectionB =bcm2835_gpio_lev(PWM_DIRCHANNEL1_B);
		
    	// if changed then need to record a tick
		if (LeftBit != LastLeft || RightBit != LastRight)
			{
			// lock so we can update variables
			mtx.lock();
			// update left and right
			if (LeftBit != LastLeft)
				{
				LastLeft = LeftBit;
				if (LeftDirectionA == LOW && LeftDirectionB == HIGH)
					{
					leftwheel++;
					}
					else
					{
					leftwheel--;
					}
				}
				
			if (RightBit != LastRight)
				{
				LastRight = RightBit;
				if (RightDirectionA == HIGH && RightDirectionB == LOW)
					{
					rightwheel++;
					}
					else
					{
					rightwheel--;
					}
				}
		    ROS_DEBUG("CheckEncoder %d %d",leftwheel,rightwheel);

			// unlock
			mtx.unlock();
			}
	}

int init()
{
	if (!bcm2835_init())
		{
		ROS_ERROR("bcm2835_init failed. Are you running as root??");
		return 1;
		}
	else
		{
		ROS_INFO("lib loaded");
		}
	GlobalDebug=1;
	leftpwm=0;
	Lastleftpwm=0;
	rightpwm=0;
	Lastrightpwm= 0;

	bcm2835_gpio_fsel(PWM_DIRCHANNEL0_A, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(PWM_DIRCHANNEL0_B, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(PWM_DIRCHANNEL1_A, BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(PWM_DIRCHANNEL1_B, BCM2835_GPIO_FSEL_OUTP);

	// stop
	SetDirection(Direction);

	ROS_INFO("Turn on Motor switch");
	bcm2835_delay(5000);	


	// Clock divider is set to 16.
	// With a divider of 16 and a RANGE of 1024, in MARKSPACE mode,
	// the pulse repetition frequency will be
	// 1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM

	bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16);   // Set maximum PWM clock Freq.

	// enable pwm0 
	bcm2835_gpio_fsel(PWM0, BCM2835_GPIO_FSEL_ALT5);  // Enable PWM on pin 12
	bcm2835_pwm_set_mode(PWM_CHANNEL0, 1, 1);    // Configure Channel 0
	bcm2835_pwm_set_range(PWM_CHANNEL0, 1024);        // Set Channel 0 Range
	bcm2835_pwm_set_data(PWM_CHANNEL0, 0);     // Set Channel 0 Data	

	// enable pwm1
	bcm2835_gpio_fsel(PWM1, BCM2835_GPIO_FSEL_ALT5);  // Enable PWM on pin 35
	bcm2835_pwm_set_mode(PWM_CHANNEL1, 1, 1);     // Configure Channel 1
	bcm2835_pwm_set_range(PWM_CHANNEL1, 1024);   // Set Channel 1 Range
	bcm2835_pwm_set_data(PWM_CHANNEL1, 0);      


return 0;
}

void close()
{
	Direction = 'S';
	SetDirection(Direction);	
    // reset pwm to output and set clear it
	bcm2835_gpio_fsel(PWM0, BCM2835_GPIO_FSEL_ALT0); 
	bcm2835_gpio_fsel(PWM1, BCM2835_GPIO_FSEL_ALT0); 
	bcm2835_gpio_fsel(PWM0, BCM2835_GPIO_FSEL_OUTP); 
	bcm2835_gpio_fsel(PWM1, BCM2835_GPIO_FSEL_OUTP); 
	bcm2835_gpio_clr(PWM0);
	bcm2835_gpio_clr(PWM1);	

  //turnoff i2c from sonar
	bcm2835_i2c_end();

	bcm2835_close();
		
	ROS_INFO("... done!");
}

//--- sonar routines

/*
 * srf08_ping - initiate a ping to the SRF08 module.  Returns
 * immidiately, the caller must then wait the appropriate amount of
 * time before reading the echo results.  Use 'srf08_range()' to read
 * the echo results.  'device' is the 7 bit I2C address of the SRF08
 * module.  'cmd' is the command to execute.
 *
 * Returns 0 on success, or -1 if an I2C error occured.  */
uint8_t srf08_ping(uint8_t device, uint8_t cmd, uint8_t Debug)
{

uint8_t returnvalue;
char buf[3];										// Buffer for data being read/ written on the i2c bus

	buf[0] = 0;													// Commands for performing a ranging on the SRF08
	buf[1] = cmd;
	returnvalue=0;

	if (Debug)
	{
	ROS_DEBUG("srf08_ping: ping i2c slave %x",device);
	}

	bcm2835_i2c_setSlaveAddress(device);
	returnvalue = bcm2835_i2c_write(buf, 2);	
	if (returnvalue != BCM2835_I2C_REASON_OK && Debug) {								// Write commands to the i2c port
		ROS_ERROR("srf08_ping: Error writing ping command to i2c slave  %x RESULT %d",device ,returnvalue);
	}

  return returnvalue;
}


/*
 * srf08_range - Read echo results from the SRF08 module.
 *
 * 'device' should be the 7 bit I2C address of the unit.  'echo'
 * should contain which echo to read, 0=first echo, 1=second echo,
 * etc.  'range' should point to a 2 byte location to hold the 16 bit
 * echo result.
 *
 * Returns 0 on success, or 1 if an I2C error occured.
 */
uint8_t srf08_range(uint8_t device, uint8_t echo, uint16_t * range, uint8_t Debug)
{
  uint8_t vh, vl;

	uint8_t returnvalue;
	char  buf[3];
	char regaddr;

	returnvalue=0;
	if (Debug)
	{
	ROS_DEBUG("srf08_range: get range for i2c slave %x echo %d",device, echo );
	}


	bcm2835_i2c_setSlaveAddress(device);

	regaddr = echo*2 + 2;


	returnvalue = bcm2835_i2c_read_register_rs(&regaddr, buf, 2);	
	if (returnvalue != BCM2835_I2C_REASON_OK && Debug) {								// Write commands to the i2c port
	ROS_ERROR("srf08_range: Error reading register address %d echo %d to i2c slave %x RESULT %d",regaddr,echo,device,returnvalue);
	}
	if (returnvalue == BCM2835_I2C_REASON_OK)
	{
	vh= buf[0];
	vl= buf[1];

	*range = (vh << 8) | vl;
	}
	return returnvalue;
}

/*
 * srf08_Version - Read software version from the SRF08 module.
 *
 * 'device' should be the 7 bit I2C address of the unit.  
 * 'Version' should point to a 1 byte location to hold the 8 bit
 * result.
 *
 * Returns 0 on success, or 1 if an I2C error occured.
 */
int8_t srf08_Version(uint8_t device, uint8_t  * Version, uint8_t Debug)
{

	uint8_t returnvalue;
	char  buf[3];
	char regaddr;

	regaddr = 0;

	returnvalue=0;
	if (Debug)
	{
	ROS_DEBUG("srf08_Version: get version for i2c slave %x",device);
	}

	bcm2835_i2c_setSlaveAddress(device);



	returnvalue = bcm2835_i2c_read_register_rs(&regaddr, buf, 1);	
	if (returnvalue != BCM2835_I2C_REASON_OK && Debug) {								// Write commands to the i2c port
	ROS_ERROR("srf08_Version: Error reading register address %d to i2c slave %x RESULT %d",regaddr,device,returnvalue);
	}
	if (returnvalue == BCM2835_I2C_REASON_OK)
	{
	*Version =buf[0];
	}
	return returnvalue;

}
/*
 * srf08_Light - Read light reading from the SRF08 module.
 *
 * 'device' should be the 7 bit I2C address of the unit.  
 * 'Light' should point to a 1 byte location to hold the 8 bit
 * result.
 *
 * Returns 0 on success, or 1 if an I2C error occured.
 */

int8_t srf08_Light(uint8_t device, uint8_t  * Light, uint8_t Debug)
{

	uint8_t returnvalue;
	char  buf[3];
	char regaddr;

	regaddr = 1;

	returnvalue=0;
	if (Debug)
	{
	ROS_DEBUG("srf08_Light: get version for i2c slave %x",device);
	}

	bcm2835_i2c_setSlaveAddress(device);



	returnvalue = bcm2835_i2c_read_register_rs(&regaddr, buf, 1);	
	if (returnvalue != BCM2835_I2C_REASON_OK && Debug) {								// Write commands to the i2c port
	ROS_ERROR("srf08_Light: Error reading register address %d to i2c slave %x RESULT %d",regaddr,device,returnvalue);
	}
	if (returnvalue == BCM2835_I2C_REASON_OK)
	{
	*Light =buf[0];
	}
	return returnvalue;

}


//-----------------------------------
/*
 * srf08_ChangeAddress - Changes SRF08 module address from 1 device to another.
 *
 * 'device' should be the 7 bit I2C address of the unit.  
 * 'Newdevice' should be the new 7 bit I2C address of the unit.  
 * We need to write 4 byte sequence 
 * A0 AA A5 newdevice
 * 
 * after change the board should flash 1 long and 0 or more short flashes.
 * Returns 0 on success, or -1 if an I2C error occured.
 * This is the one to use. The address needs to be E0 based not 70 based.
 */

uint8_t srf08_ChangeAddress(uint8_t device, uint8_t Newdevice, uint8_t Debug)
{

	uint8_t returnvalue;
	char  buf[7];	
	uint8_t addr;
	returnvalue=0;
	bcm2835_i2c_setSlaveAddress(device);

	// write to the command register
	addr = 0;
	buf[0]=addr;
	buf[1]=0xA0;
	buf[2]=0xAA;
	buf[3]=0xA5;
	buf[4]=Newdevice;
	if (Debug)
	{
	ROS_DEBUG("srf08_ChangeAddress: change device %x to %x",device,Newdevice);
	}

	returnvalue = bcm2835_i2c_write(buf, 5);	
	if (returnvalue != BCM2835_I2C_REASON_OK && Debug) {								// Write commands to the i2c port
	ROS_ERROR("srf08_ChangeAddress: Error writing command register address to i2c slave  %x RESULT %d",device ,returnvalue);
	}
	return returnvalue;

}

//-----------------------------------

uint8_t srf08_SetMaxrange(uint8_t device, uint8_t range, uint8_t  gain, uint8_t Debug)
{

	uint8_t returnvalue;
	char  buf[7];	
	returnvalue = 0;
	if (gain > 31)
	gain = 31;
	  
	if (Debug)
	{
	ROS_DEBUG("srf08_SetMaxrange: set device %x max range to %d gain to %d",device,range,gain);
	}

	bcm2835_i2c_setSlaveAddress(device);



	// range register address is 2 
	buf[0] = 2;
	buf[1] = range;


	returnvalue = bcm2835_i2c_write(buf, 2);	
	if (returnvalue != BCM2835_I2C_REASON_OK && Debug) {								// Write commands to the i2c port
	ROS_ERROR("srf08_SetMaxrange: Error setting max range for i2c slave  %x RESULT %d",device ,returnvalue);
	}



	// gain register address is 1 
	buf[0] = 1;
	buf[1] = gain;

	returnvalue = bcm2835_i2c_write(buf, 2);	
	if (returnvalue != BCM2835_I2C_REASON_OK && Debug) {								// Write commands to the i2c port
	ROS_ERROR("srf08_SetMaxrange: Error setting gain for i2c slave  %x RESULT %d",device ,returnvalue);
	}

	return returnvalue;

}
uint8_t SetupSonar(uint8_t Debug)
{
	uint8_t Result;    

	// I2C begin if specified    
	if (!bcm2835_i2c_begin())
	  {
		ROS_ERROR("bcm2835_i2c_begin failed. Are you running as root??");
		return 1;
	  }
		  

	// per my code from nav robot 100 khz should be good
	bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500 ); // /*!< 2500 = 10us = 100 kHz */




	Result =srf08_SetMaxrange(FRONT, 25,0,  Debug);
	if (Debug && Result )
	{
		ROS_ERROR("SetuoSonar: Front Error %d",Result);
	}
	else
	{
		Result =srf08_SetMaxrange(LEFTFRONT, 25,0,  Debug);
		if (Debug && Result )
		{
			ROS_ERROR("SetuoSonar: LeftFront Error %d",Result);
		}
		else
		{
		Result =srf08_SetMaxrange(LEFTBACK, 25,0,  Debug);
		if (Debug && Result )
		{
			ROS_ERROR("SetuoSonar: LeftBack Error %d",Result);
		}
		else
		{
		Result =srf08_SetMaxrange(RIGHT, 25,0,  Debug);
		if (Debug && Result )
		{
			ROS_ERROR("SetuoSonar: Right Error %d",Result);
		}
		}
		}
	}
	return Result ;
}

void ping( uint8_t  Measurement)
{
	uint8_t  address = 0;			// Address of the 0 is broadcast	
	uint8_t Result; 
	address = 0;
	// send out ping 
	Result =  srf08_ping(address, Measurement, GlobalDebug);	
	if (Result  != BCM2835_I2C_REASON_OK)
	{
		GlobalDebug = 1;
	} 
	else
	{
		GlobalDebug = 0;
		
	}
	
}
void range(uint16_t *front,uint16_t *leftf, uint16_t *leftb, uint16_t *right,uint8_t  Measurement)
{

	uint8_t  address = 0;
	uint8_t Result; 
	char const* Meas;
	address = LEFTFRONT;
	Result= srf08_range(address, 0, leftf, GlobalDebug);
	if (*leftf ==0)
	{
		*leftf = ( Measurement ==RANGE_US ? 14810 : (Measurement ==RANGE_CM  ? MAX_SONAR_CM : MAX_SONAR_IN));
	}
	if (Result  != BCM2835_I2C_REASON_OK)
	{
		GlobalDebug = 1;
	} 

	address = LEFTBACK;
	Result= srf08_range(address, 0, leftb, GlobalDebug);
	if (*leftb ==0)
	{
		*leftb = ( Measurement ==RANGE_US ? 14810 : (Measurement ==RANGE_CM ? MAX_SONAR_CM : MAX_SONAR_IN));
	}
	if (Result  != BCM2835_I2C_REASON_OK)
	{
		GlobalDebug = 1;
	} 

	address = RIGHT;
	Result=  srf08_range(address, 0, right, GlobalDebug);
	if (*right ==0)
	{
		*right = ( Measurement ==RANGE_US ? 14810 : (Measurement ==RANGE_CM ? MAX_SONAR_CM : MAX_SONAR_IN));
	}
	if (Result  != BCM2835_I2C_REASON_OK)
	{
		GlobalDebug = 1;
	} 

	address = FRONT;
	Result=  srf08_range(address, 0, front, GlobalDebug);
	if (*front ==0)
	{
		*front = ( Measurement ==RANGE_US ? 14810 : (Measurement ==RANGE_CM ? MAX_SONAR_CM : MAX_SONAR_IN));
	}
	if (Result  != BCM2835_I2C_REASON_OK)
	{
		GlobalDebug = 1;
	}
	if (Measurement == RANGE_US)
		{
		Meas = "us";
	}
	else
	{
	if (Measurement == RANGE_CM)
		{
		Meas = "cm";
		}
	else
		{
		if (Measurement == RANGE_IN)
			{
			Meas = "in";
			}
		}
	}
	 

	ROS_DEBUG("R %s F %d, LF %d LB %d R %d res %d",Meas,*front,*leftf, *leftb,*right,Result);

	
}

	void CalcOdom(const ros::TimerEvent e)
	{
	long localleftwheel,localrightwheel;
	double dt,oldth;

	CurrentEncoderTime = ros::Time::now();

	dt = (LastEncoderTime-CurrentEncoderTime).toSec();
	
	nav_msgs::Odometry msg;

	//ROS_INFO("CalcOdom Triggered");
	//ROS_INFO("TimerEvent %f ", dt);

	// lock
	mtx.lock();
	// grab ticks to local variable
	localleftwheel = leftwheel;
	localrightwheel = rightwheel;
	// unlock
	mtx.unlock();
	
	// calculate odom here	
	ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftwheel > 0 || localrightwheel > 0) ,"CalcOdom localleftwheel %ld lastleftwheel %ld localrightwheel %ld lastrighttwheel %ld",localleftwheel,lastleftwheel,localrightwheel,lastrightwheel );

	// how many click did wee see.
	deltaLeft =  (localleftwheel -lastleftwheel);
	deltaRight = (localrightwheel - lastrightwheel);
	// save current click count for next run
	lastleftwheel = localleftwheel;
	lastrightwheel = localrightwheel;
	// need to figureout how to get lape time correctly
	//dt = (e.current_real -e.last_real).toSec();
	// use clicks to calculate distance moved
	VelLeft = (deltaLeft * METERPERTICK ); //.01046
	VelRight = (deltaRight * METERPERTICK );

	ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftwheel > 0 || localrightwheel > 0) ,"CalcOdom Delta l %ld r %ld Vel l %f r %f",deltaLeft, deltaRight,VelLeft,VelRight);

	linear = (VelLeft + VelRight)/2.0;
	angular = (VelRight - VelLeft ) /BASE; // divide by wheel base 0.12005

	ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftwheel > 0 || localrightwheel > 0) ,"CalcOdom linear %f angular %f ", linear, angular);
	
	VelLeft = VelLeft/dt;
	VelRight = VelRight/dt;
	Velth = angular/dt;
	
	ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftwheel > 0 || localrightwheel > 0) ,"CalcOdom Vel l %f r %f th %f ",VelLeft,VelRight, Velth);

    if (fabs(angular) < 1e-6)
	{
    const double direction = th + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
	deltaX =linear * cos(direction);
	deltaY =linear * sin(direction);
    x += deltaX;
    y += deltaY;
    th += angular *dt;

	}
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double oldth = th;
      const double r = linear/angular;
      th += angular;
	  deltaX= r * (sin(th) - sin(oldth));
	  deltaY = -r * (cos(th) - cos(oldth));
      x  +=  deltaX;
      y  += deltaY;
    }

	ROS_INFO_COND(LOG_INFO_DEBUG == 1 && (localleftwheel > 0 || localrightwheel > 0) ,"CalcOdom  delta x %f y %f new X %f y %f",		deltaX,deltaY, x, y);
	
	/*
https://github.com/ros-controls/ros_controllers/blob/kinetic-devel/diff_drive_controller/src/diff_drive_controller.cpp
   
   */

	
		/**
		 * This is a message object. You stuff it with data, and then publish it.
		 */
//x,y,th, VelLeft, VelRight, Velth, deltaLeft, deltaRight
		msg.child_frame_id = "odom";
		msg.pose.pose.position.x= x;
		msg.pose.pose.position.y= y;
		msg.pose.pose.position.z= 0;
	/*
		ROS_INFO("%s", msg.data.c_str());
	*/


//---------------------------------------------------------------------------------------
/*
http://answers.ros.org/question/11973/gathering-wheel-encoder-data-and-publishing-on-the-odom-topic/
*/
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = CurrentEncoderTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    ptr_odom_broadcaster->sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
  //  nav_msgs::Odometry odom;
    msg.header.stamp = CurrentEncoderTime;
    msg.header.frame_id = "odom";

    //set the position
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = odom_quat;

    //set the velocity
    msg.child_frame_id = "base_link";
    msg.twist.twist.linear.x = deltaX; //vx;//vx
    msg.twist.twist.linear.y = deltaY;//vy
    msg.twist.twist.angular.z = Velth;


	    //publish the message
   // odom_pub.publish(odom);



//---------------------------------------------------------

		ptr_pub->publish(msg);

		// set last for next run
		lastleftwheel = localleftwheel;
		lastrightwheel = lastrightwheel;
		LastEncoderTime = CurrentEncoderTime;
//	    ROS_DEBUG("CalcOdom encoder l %d r %d Diff l %ld r %ld Vel l %f r %f th %f linear %f angular %f diff x %f y %f"
//		,localleftwheel,localrightwheel,deltaLeft, deltaRight,VelLeft,VelRight, Velth, linear, angular,deltaX,deltaY);


		uint16_t Front,LeftF,LeftB,Right;

		range(&Front,&LeftF, &LeftB, &Right,Measurement);	

//		FrontMsg.header.stamp = ros::Time::now();
//		FrontMsg.range = (float)Front/100.0;
//		ptr_sonarpub->publish(FrontMsg);

//		LeftFMsg.header.stamp = FrontMsg.header.stamp;
//		LeftFMsg.range = (float)LeftF/100.0;
//		ptr_sonarpub->publish(LeftFMsg);

//		LeftBMsg.header.stamp = FrontMsg.header.stamp;
//		LeftBMsg.range = (float)LeftB/100.0;
//		ptr_sonarpub->publish(LeftBMsg);

//		RightMsg.header.stamp = FrontMsg.header.stamp;
//		RightMsg.range = (float)Right/100.0;
//		ptr_sonarpub->publish(RightMsg);

// now convert to laser
  ROS_INFO("begin scan conversion");
  // 
  scan.header.stamp = ros::Time::now();
  
  
  scan.ranges[0] = 2450;
  scan.intensities[0] = .5;
  scan.ranges[9] = 150;
  scan.intensities[9] = .5;
  scan.ranges[19] = 300;
  scan.intensities[19] = .5;
  scan.ranges[29] = 150;
  scan.intensities[29] = .5;
  scan.ranges[39] = 300;
  scan.intensities[39] = .5;
  scan.ranges[49] = 600;
  scan.intensities[49] = .5;
  scan.ranges[59] = 1200;
  scan.intensities[59] = .5;
  scan.ranges[69] = 600;
  scan.intensities[69] = .5;
  scan.ranges[79] = 1200;
  scan.intensities[79] = .5;
  scan.ranges[89] = 400;
  scan.intensities[89] = .5;
  scan.ranges[99] = 400;
  scan.intensities[99] = .5;
  scan.ranges[109] = 800;
  scan.intensities[109] = .5;
  scan.ranges[119] = 800;
  scan.intensities[119] = .5;
  scan.ranges[129] = 300;
  scan.intensities[129] = .5;
  scan.ranges[139] = 300;
  scan.intensities[139] = .5;
  scan.ranges[149] = 900;
  scan.intensities[149] = .5;
  scan.ranges[159] = 900;
  scan.intensities[159] = .5;
  scan.ranges[169] = 1500;
  scan.intensities[169] = .5;
  scan.ranges[179] = 1500;
  scan.intensities[179] = .5;  
  
  /*
  // overwrite the no signal with our sonar for the 14 readings within our beam
  for(unsigned int i = NUM_READINGS-14; i < NUM_READINGS+1; ++i){
    scan.ranges[i] = (float)Right/100.0 + SIDE_OFFSET;
    scan.intensities[i] = 0.5;
  }

  for(unsigned int i =(NUM_READINGS/2)-7; i < (NUM_READINGS/2)+7; ++i){
    scan.ranges[i] = (float)Front/100.0 + FRONT_OFFSET;
    scan.intensities[i] = 0.5;
  }
  
  for(unsigned int i = 0; i < 14; ++i){
    scan.ranges[i] = (float)(LeftF/100.0) + SIDE_OFFSET;
    scan.intensities[i] = 0.5;
  }
 */ 
  ptr_scan_pub->publish(scan);
  ROS_INFO("end scan conversion");
  
		ping(Measurement);

// 



	}


//--------
int main(int argc, char **argv)
{

	if (init()==1)
	{
	  ROS_ERROR("init failed. Are you running as root??");
	  return 1;
	}

	if (SetupSonar(GlobalDebug)==1)
	{
	  ROS_ERROR("Sonar init failed. Are you running as root??");
	  return 1;
	}
	
	// encoder variable init
	leftwheel=0;
	rightwheel=0;
	x = 0;
	y = 0;
	th = 0;
	LeftBit = bcm2835_gpio_lev(LEFTENCODER);
	RightBit = bcm2835_gpio_lev(RIGHTENCODER);
	LastLeft = LeftBit;
	LastRight = RightBit;
	lastleftwheel=leftwheel;
	lastrightwheel = rightwheel;
	linear=0.0;
	angular=0.0;

	// sonar init variables

	ros::init(argc, argv, "MotorandSensor");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, TwistCallback);
	ROS_INFO("Motor started");

	ros::Publisher pubOdom_pub;
	ptr_pub = &pubOdom_pub;
	pubOdom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
	LastEncoderTime = ros::Time::now();

	Measurement = RANGE_CM;

  ros::Publisher scan_pub;
  ptr_scan_pub = &scan_pub;
  
  // init the laser ranges arrays to no reading
//for(unsigned int i = 0; i < NUM_READINGS; ++i){
//    ranges[i] = (MAX_SONAR_CM + .05)/100.0;
//    intensities[i] = 0;
//  }
  

  scan.header.frame_id = "/base_link";
  scan.angle_min = 3.14159;//DEG_TO_RAD(MIN_ANGLE); 1.81514 = 105deg 1.5708 = 90 deg
  scan.angle_max = -3.14159;//DEG_TO_RAD(MAX_ANGLE);
  scan.angle_increment = 0.034906585;//DEG_TO_RAD(DEG_INCREMENT);
  scan.time_increment = (1.0 / 100) / (NUM_READINGS);
  scan.range_min = 0.0;
  scan.range_max = MAX_SONAR_CM;

  scan.ranges.resize(NUM_READINGS);
  scan.intensities.resize(NUM_READINGS);
  for(unsigned int i = 0; i < NUM_READINGS; ++i){
    scan.ranges[i] = (double)(MAX_SONAR_CM + .15)/100.0;
    scan.intensities[i] = 0;
  }



  // Publish generated laserscan data with a queue size of 50
  scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);


	ping(Measurement);

	ros::Timer encoderTimer = n.createTimer(ros::Duration(0.010),CheckEncoder);
	ros::Timer odomTimer = n.createTimer(ros::Duration(0.1), CalcOdom);

	// this is untested transform
	tf::TransformBroadcaster odom_broadcaster;
	ptr_odom_broadcaster = &odom_broadcaster;
	// end of transform

	ROS_INFO("Timers started");

	ros::spin();

	return 0;
}
