#include "sensordata.h"
#include "SlaveParse.h"


sensorData::sensorData()
{
mDirection='Z';
}

int sensorData::getTimeStamp(){
    return mtimestamp;
}

int sensorData::getLeftClick(){
return mleftClicks;
}

int sensorData::getRightClick(){
return mrightClicks;
}

unsigned short sensorData::getFrontSonar(){
return mfrontSonar;
}

unsigned short sensorData::getRightSonar(){
return mrightSonar;
}

unsigned short sensorData::getLeftSonar(){
return mleftSonar;
}

unsigned short sensorData::getLeftMotor(){
return mleftMotor;
}

unsigned short sensorData::getRightMotor(){
return mrightMotor;
}


double sensorData::GetFrontRange()
{
   // sonars are in micro seconds divide by 1000 to get millseconds and again to get seconds
   // divide by 2 for out and back flight time
   return (((double)mfrontSonar/1000000) * SonarMPS)/2.0;
}
double sensorData::GetLeftRange()
{
   return (((double)mfrontSonar/1000000) * SonarMPS)/2.0;
}
double sensorData::GetRightRange()
{
   return (((double)mfrontSonar/1000000) * SonarMPS)/2.0;
}


char sensorData::getDirection(){
return mDirection;
}

char* sensorData::ToString()
{

    snprintf(description,199, "Click L %d R %d Motor L %d R %d %c Sonar F %d R %d",mleftClicks,mrightClicks,mleftMotor ,mrightMotor, mDirection,mfrontSonar,mrightSonar);
    return description;
}
void sensorData::setSensorTestData(int leftClicks, int rightClicks,unsigned short leftMotor, unsigned short rightMotor, char direction)
{
mleftClicks=leftClicks;
mrightClicks=rightClicks;
mleftMotor=leftMotor;
mrightMotor=rightMotor;
mDirection=direction;
mtimestamp=mtimestamp + 100; // simulate 100 ms cycle 
}
void sensorData::getSensorData(char *ptr,BotConfig config)
{
// old
// 0123456789112345678921234567893123456
// $03  2ff5   572   a9710d11dR ad3 688Z
// new
// 0123456789112345678921234567893123456789412
// $0300002ff50000057200000a9710d11dR ad3 688Z
    int Index,maxlen = 37;

    char* Data = ptr;
    if (config.HasCompass() )
    {
        maxlen += 4;
    }
    if (config.NumberSonars() > 0)
    {
        maxlen += (config.NumberSonars() * 4);
    }

    if (Data[1] == 48 && Data[2] == 51)
        {
            mtimestamp =  SlaveParse::ParseSignedHex(Data, 3, 8);
            mleftClicks = (int) SlaveParse::ParseSignedHex(Data, 11, 8);
            mrightClicks = (int) SlaveParse::ParseSignedHex(Data, 19, 8);
            mleftMotor = (ushort) SlaveParse::ParseHex(Data, 27, 3);
            mrightMotor = (ushort) SlaveParse::ParseHex(Data, 30, 3);
            mDirection = (char)Data[33];
            Index = 34;

            if (config.HasCompass())
                {
                mcompass = (ushort) SlaveParse::ParseHex(Data, Index, 4);
                Index += 4;
                }
            if (config.NumberSonars() > 0)
                {
                    mfrontSonar = (ushort) SlaveParse::ParseHex(Data, Index, 4);
                    Index += 4;
                    if (config.NumberSonars() > 1)
                        {
                            if (config.NumberSonars() > 2)
                            {
                                mleftSonar= (ushort) SlaveParse::ParseHex(Data, Index, 4);
                            }
                            else
                            {
                                mrightSonar = (ushort) SlaveParse::ParseHex(Data, Index, 4);
                            }
                        Index += 4;
                        if (config.NumberSonars() > 2)
                            {
                                mrightSonar = (ushort) SlaveParse::ParseHex(Data, Index, 4);
                            }
                        }
                }



    }

}
