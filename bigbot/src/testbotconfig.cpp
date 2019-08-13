#include "botconfig.h"
#include <stdio.h>

int main(int argc, char **argv)
{
BotConfig mconfig;

static const int rcvBufSize = 200;
char ucResponse[rcvBufSize];   //response string from uController
char *bufPos;

// 0123456790123456789012345
// $07C  0 S 72 70  0  0  0Z
char buf1[] = "$07C  0 S 72 70  0  0  0Z";
char buf2[] = "$07C  0 S 00 00  0  0  0Z";
char buf3[] = "$07C  0 S 72 70  0  0";

char* Data;

bufPos = &buf1[0];

          mconfig =  BotConfig(bufPos);
          printf("after create");
          if (!mconfig.IsConfigValid() )
          {
              // This can happen if slave code was updated the eeprom config gets cleared
              // so we need to reset config and reread config
              mconfig.ResetConfig();
              for (int Index =0; Index < mconfig.NumberSonars();Index++ )
              {

                printf("S %d %d\n", Index,mconfig.SonarAddress(Index));
              }

          }
          else
          {
              printf("valid config");
          }

bufPos = &buf2[0];
          mconfig =  BotConfig(bufPos);
          printf("after create");
          if (!mconfig.IsConfigValid() )
          {
              // This can happen if slave code was updated the eeprom config gets cleared
              // so we need to reset config and reread config
              mconfig.ResetConfig();
              for (int Index =0; Index < mconfig.NumberSonars();Index++ )
              {

                printf("S %d %d\n", Index,mconfig.SonarAddress(Index));
              }

          }
          else
          {
              printf("valid config");
          }

bufPos = &buf3[0];
          mconfig =  BotConfig(bufPos);
          printf("after create");
          if (!mconfig.IsConfigValid() )
          {
              // This can happen if slave code was updated the eeprom config gets cleared
              // so we need to reset config and reread config
              mconfig.ResetConfig();
              for (int Index =0; Index < mconfig.NumberSonars();Index++ )
              {

                printf("S %d %d\n", Index,mconfig.SonarAddress(Index));
              }

          }
          else
          {
              printf("valid config");
          }


}
