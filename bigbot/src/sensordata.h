#ifndef SENSORDATA_H
#define SENSORDATA_H
#include "botconfig.h"
#include <sensor_msgs/Range.h>
#include <string>


class sensorData
{
//$03   e17     0     0  0  0F aba 688Z
private:
    int mtimestamp=0;
    int mleftClicks=0;
    int mrightClicks=0;
    unsigned short mfrontSonar;
    unsigned short mleftSonar;
    unsigned short mrightSonar;
    unsigned short mleftMotor;
    unsigned short mrightMotor;
    unsigned short mcompass;
    char mDirection;
    char description[200];
    const double SonarMPS =340.0;


public:

    int getTimeStamp();
    int getLeftClick();
    int getRightClick();
    unsigned short getFrontSonar();
    unsigned short getLeftSonar();
    unsigned short getRightSonar();
    unsigned short getLeftMotor();
    unsigned short getRightMotor();
    double GetFrontRange();
    double GetLeftRange();
    double GetRightRange();

    char getDirection();

    sensorData();
    void getSensorData(char* ptr,BotConfig config);
	void setSensorTestData(int leftClicks, int rightClicks,unsigned short leftMotor, unsigned short rightMotor, char direction);
    char* ToString();
};

#endif // SENSORDATA_H
