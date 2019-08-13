#include "serial.h"
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
//----

#include <sstream>





#define BAUDRATE B57600   // Change as needed, keep B


/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyUSB0"

int main(int argc, char **argv)
{
Serial mComm;
char port[20];    //port name
int baud;     //baud rate
char *ptr;
strcpy(port, MODEMDEVICE);
baud = BAUDRATE;
char OutgoingBuffer[200];

if (!mComm.serialInit(port,baud))
{
          printf("unable to create a new serial port %s baud %d",port, baud);
          return 1;
}
int CommandID=0;
int State = 0;
int BeginCommand;
int strLen;
int sleepu=1000;
int nbrBytes;
int loopCounter=0;
while(State < 7 || (State==7 && loopCounter < 1000))
{
   if (mComm.SerialRead())
   {

    if ( mComm.SerialLineAvailable())
    {
        ptr = mComm.SerialGetData();
        strLen = strlen(ptr);
        for (BeginCommand = 0;BeginCommand <=strLen && !(ptr[BeginCommand] == '$' || ptr[BeginCommand] == 'E' || ptr[BeginCommand] == '1' ) ; BeginCommand++)
        {
            printf("%c",ptr[BeginCommand]);
        }
        if (ptr[BeginCommand]=='$' && ptr[BeginCommand+1]=='0' )
        {
            if (ptr[BeginCommand+2] == '3')
            {
                printf("%s",&ptr[BeginCommand]);
                State=7;
                loopCounter++;
            }
            else if (ptr[BeginCommand+2] == '1')
            {
                State++;
                printf("%s",&ptr[BeginCommand]);
                CommandID++;
                nbrBytes=snprintf(OutgoingBuffer,10,"E %d 0\n",CommandID);
                mComm.SerialWrite(OutgoingBuffer,nbrBytes);

            }
            else if (ptr[BeginCommand+2] == '2')
            {
                printf("%s",&ptr[BeginCommand]);

            }
            else if (ptr[BeginCommand+2] == '7')
            {
                printf("%s",&ptr[BeginCommand]);
                // check we have config
                if (ptr[BeginCommand+11] == '0')
                {
                    printf("Sending Sonar\n");
                    nbrBytes=snprintf(OutgoingBuffer, 30,"A S %d %d\nA S %d %d\n", 1,114,2,112);
                    mComm.SerialWrite(OutgoingBuffer,nbrBytes);
                    // send sonar
                }
                CommandID++;
                //send gain
                nbrBytes=snprintf(OutgoingBuffer, 20,"G %d 10 80\n",CommandID);
                mComm.SerialWrite(OutgoingBuffer,nbrBytes);
                State = 4;
                printf("sent gain\n");

            }
            else if (ptr[BeginCommand+2] == '8')
            {
                printf("%s",&ptr[BeginCommand]);

            }
            else
            {
                     printf("unknown command %s\n",&ptr[BeginCommand] );
            }




        }
        else if( ptr[BeginCommand] == 'E' )
        {
            State++;
            nbrBytes=snprintf(OutgoingBuffer,3,"P\n");
            mComm.SerialWrite(OutgoingBuffer,nbrBytes);
            printf("echo\n");
        }
        else if( ptr[BeginCommand] == '1' )
               {
                printf("%s",&ptr[BeginCommand]);

                State = 6;
                nbrBytes=snprintf(OutgoingBuffer,5,"B 1\n");
                mComm.SerialWrite(OutgoingBuffer,nbrBytes);
                loopCounter=0;
               }
    }
   }
   else
   {

       if (State == 4 )
       {
           loopCounter++;
        if( loopCounter > 20000  )
        {
        printf("state 6 counter\n");
         State = 6;
         nbrBytes=snprintf(OutgoingBuffer,5,"B 1\n");
         mComm.SerialWrite(OutgoingBuffer,nbrBytes);

        }

       }
   }

usleep(sleepu);

}

mComm.SerialClose();
}

