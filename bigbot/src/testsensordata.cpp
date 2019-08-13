#include "sensordata.h"
#include "botconfig.h"
#include <stdio.h>
#include <stdlib.h>     /* strtol */

int main(int argc, char **argv)
{
sensorData msensor;


static const int rcvBufSize = 200;
char ucResponse[rcvBufSize];   //response string from uController
char *bufPos;

char buf0[]="$07C  0 S 72 70  0  0  0Z";
bufPos = &buf0[0];
BotConfig mconfig = BotConfig(bufPos);

char TempBuffer[80];

// 0123456790123456789012345
// $07C  0 S 72 70  0  0  0Z
char buf1[] = "$0300002ff500000572fffffa9710d11dR ad3 688Z";
char buf2[] = "$03  2ff5   572   a971";

// time,leftdistance,rightdistance,leftmotor,rightmotor,Direction
const char s_output[] = "$03%08lx%08lx%08lx%3x%3x%c";
long unsigned int TimeCounter = 2147483640;
long unsigned int TempLeft = -1; //2,147,483,647
long unsigned int TempRight = 2147483647;
int TempMotorLeft = 500;
int TempMotorRight = 512;
char Direction  ='F';
snprintf(TempBuffer,79,s_output,TimeCounter, TempLeft,TempRight,TempMotorLeft,TempMotorRight,Direction);

printf("%s\n",TempBuffer);
char hextext[]="FFFFFFFF";
long unsigned int hextesta= strtol(hextext, (char **)NULL, 16);
printf("%ld\n",hextesta );

bufPos = &buf1[0];

          msensor.getSensorData(bufPos,mconfig);
          printf("after create %d %d %d %d %d %d %d %d %c\n",msensor.getTimeStamp(),
                 msensor.getLeftClick(),
                 msensor.getRightClick(),
                 msensor.getFrontSonar(),
                 msensor.getLeftSonar(),
                 msensor.getRightSonar(),
                 msensor.getLeftMotor(),
                 msensor.getRightMotor(),
                 msensor.getDirection()
                 );
         printf("string %s\n",msensor.ToString());

bufPos = &buf2[0];
    msensor.getSensorData(bufPos,mconfig);
    printf("after create %d %d %d %d %d %d %d %d %c\n",msensor.getTimeStamp(),
           msensor.getLeftClick(),
           msensor.getRightClick(),
           msensor.getFrontSonar(),
           msensor.getLeftSonar(),
           msensor.getRightSonar(),
           msensor.getLeftMotor(),
           msensor.getRightMotor(),
           msensor.getDirection()
           );
    printf("string %s\n",msensor.ToString());


}

