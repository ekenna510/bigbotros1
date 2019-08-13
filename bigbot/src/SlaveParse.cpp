#include "SlaveParse.h"

unsigned int SlaveParse::ParseHex(char Data[], int start, int length)
    {
        int Index = 0;
        unsigned int result = 0;

        for (Index = start; Index - start < length; Index++)
        {

            if (Data[Index] > 47 && Data[Index] < 58)
            {
                // handle 0-9
                result = (result << 4) + Data[Index] - (unsigned int)48;
            }
            else
            {
                // handle a-f
                if (Data[Index] > 96 && Data[Index] < 103)
                {
                    result = (result << 4) + (Data[Index] - (unsigned int)87);
                }
                else
                {
                    // handle A-F
                    if (Data[Index] > 64 && Data[Index] < 71)
                    {
                        result = (result << 4) + (Data[Index] - (unsigned int)55);
                    }
                    else
                    {
                    }

                }
            }
        }
        return result;
    }

int SlaveParse::ParseSignedHex(char Data[], int start, int length)
    {
        int Index = 0,i=0;
        int result = 0;

        char tempbuffer[10];


        for (Index = start; Index - start < length; Index++,i++)
        {
            tempbuffer[i]= Data[Index];
        }
        tempbuffer[i]=0;
        result = strtol(tempbuffer, (char **)NULL, 16);
        return result;
    }

 int SlaveParse::ParseInt(char Data[], int start, int length)
{
    int Index = 0;
    int result = 0;
    bool isNegative = false;

    for (Index = start; Index - start < length; Index++)
    {

        if (Data[Index] > 47 && Data[Index] < 58)
        {
            // handle 0-9
            result = (result * 10);
            result = result + (int)(Data[Index] - (int)48);
        }
        else
        {
            // - minus
            if (Data[Index] == 45)
            {
                isNegative = true;
            }
            else
            {
            }
        }
    }
    return (isNegative ? result * -1 : result);
}


