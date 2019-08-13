#ifndef SERIAL_H
#define SERIAL_H

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <sstream>

#include <sys/stat.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define MYSERIALNUMBUFFERS 5

class Serial
{
private:
    //Global data
    //FILE *fpSerial = NULL;   //serial port file pointer
    int fd = -1;
    static const int rcvBufSize = 60;
    char incomingBufferArray[MYSERIALNUMBUFFERS][rcvBufSize];   //buffer to read into for each serail read
    char consolidatedBuffer[200];
    bool inUse[MYSERIALNUMBUFFERS];  // indicator if we can reuse the buffe
    int endLine[MYSERIALNUMBUFFERS]; // last character read for this buffer
    int bufferIndex; // index to which buffer can be written to next
    char *bufPos;
    int StartofLine=0;
    bool IsStreamOpen;
    bool IsLineAvailable;


public:
    Serial();
    bool SerialRead();
    bool SerialLineAvailable();
    //bool SerialReadLine();
    char *SerialGetData();
    void SerialClose();
    bool serialInit(char * port, int baud);
    bool SerialIsOpen();
    void SerialWrite(char * data,int nbrbytes);


};

#endif // SERIAL_H
