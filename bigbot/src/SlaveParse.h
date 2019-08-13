#ifndef SLAVEPARSE_H
#define SLAVEPARSE_H
#include <stdlib.h>     /* strtol */

class SlaveParse
{

private:

public:



static unsigned int ParseHex(char Data[], int start, int length);
static int ParseInt(char Data[], int start, int length);
static int ParseSignedHex(char Data[], int start, int length);

};

#endif // SLAVEPARSE_H

