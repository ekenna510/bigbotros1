#include "botconfig.h"
#include "SlaveParse.h"
#include <stdio.h>

BotConfig::BotConfig()
{
}
// 0123456790123456789012345
// $07C  0 S 72 70  0  0  0Z
BotConfig::BotConfig(char* ptr)
    {
//        char Data[0];
//        Data = ptr;
//ptr.length == 26 &&
    try
    {
        if (ptr[0] == '$' && ptr[1] == '0' && ptr[2] == '7' and ptr[24] == 'Z')
            {
            mCompassAddress = SlaveParse::ParseHex(ptr, 4, 3);
            mSonarAddresses[0] = SlaveParse::ParseHex(ptr, 10, 3);
            mSonarAddresses[1] = SlaveParse::ParseHex(ptr, 13, 3);
            mSonarAddresses[2] = SlaveParse::ParseHex(ptr, 16, 3);
            mSonarAddresses[3] = SlaveParse::ParseHex(ptr, 19, 3);
            mSonarAddresses[4] = SlaveParse::ParseHex(ptr, 22, 3);

            if (mCompassAddress > 0)
            {
                mHasCompass = true;
            }
            mNumberSonar=0;
            for (int Index=0;Index <mMaxSonars;Index++)
            {
                if (mSonarAddresses[Index] > 0)
                {
                mNumberSonar = Index+ 1;
                }
            }
            if (mNumberSonar ==2 || mHasCompass)
                {
                mValidConfig = true;
                }
        }

        }
    catch (...)
    {
       printf("Error BotConfig");
       mValidConfig =false;
    }
    }


//    unsigned int mCompassAddresses;
//    unsigned int[] mSonarAddresses;
//    bool[] mRangeSet;
//    bool mHasCompass = false;
//    int  mNumberSonar = 2;
//    int  mMaxSonars = 5;
    void BotConfig::ResetConfig(){

     mSonarAddresses[0] = 114;
     mSonarAddresses[1] = 112;
     mNumberSonar = 2;
     mRangeSet = false;
     mHasCompass = false;
     mValidConfig = true;
    }

    bool BotConfig::IsConfigValid()
    {
    return mValidConfig;
    }

    unsigned int BotConfig::SonarAddress(int Index){
        unsigned int retval = 0;
        if (mValidConfig)
        {
            if (Index <= mNumberSonar)
            {
                retval= mSonarAddresses[Index];
            }
        }
        return retval;

    }

    bool BotConfig::IsRangeSet(){
            return mRangeSet;
    }

    void BotConfig::SetRange()
    {
        if (mValidConfig)
        {
            mRangeSet = true;
        }
    }

    unsigned int BotConfig::CompassAddress(){
        unsigned int retval = 0;
        if (mValidConfig)
            {

            retval= mCompassAddress;

            }
        return retval;

    }
    bool BotConfig::HasCompass()
    {
        return mHasCompass;
    }
    int BotConfig::NumberSonars(){
        if (mValidConfig)
        {
            return mNumberSonar;
        }
    }
    void BotConfig::SetNumberSonars(int NumberSonars)
    {
        mNumberSonar = NumberSonars;
    }



