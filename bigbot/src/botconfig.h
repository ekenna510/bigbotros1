#ifndef BOTCONFIG_H
#define BOTCONFIG_H



class BotConfig
{


private:

    unsigned int mCompassAddress;
    unsigned int mSonarAddresses[5];
    bool mRangeSet=false;
    bool mHasCompass = false;
    int  mNumberSonar = 2;
    int  mMaxSonars = 5;
    bool mValidConfig = false;


public:
    BotConfig();
    BotConfig(char* ptr);
    unsigned int CompassAddress();
    unsigned int SonarAddress(int Index);
    bool HasCompass();
    bool IsRangeSet();
    void SetRange();
    bool IsConfigValid();
    int NumberSonars();
    void SetNumberSonars(int NumberSonars);
    void ResetConfig();

};
#endif // BOTCONFIG_H
