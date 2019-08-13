/*
main.c Master loop 

This version acts a a slave to the laptop 
It is responsible for handling the compass, sonar and pwm
It updates the laptop once every 100 ms

It communicates to the laptop via serial coms
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/signal.h>
//#include <avr/wdt.h>
#include <compat/twi.h>
#include <stdlib.h> 
#include <avr/sfr_defs.h>

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "ringbuf.h"
#include "uart.h"
#include "sonar.h"
#include "compass.h"
#include "i2c.h"
#include "motor.h"

#define SETB(REG,POS)   REG|=_BV(POS)
#define CLRB(REG,POS)   REG&=(~_BV(POS))

// define some constants
	#define MS_OCRMATCH 125
	#define MS_HALFSECOND 500
	#define MS_TENTHSECOND 200
	#define MS_PINGDELAY 50

// define sonar constants
#define FRONTDEVICE 0x70   
#define LEFTDEVICE  0x76   
#define RIGHTDEVICE 0x72   
#define ALLDEVICES 0

const char s_bksp[]              PROGMEM = "\b \b";
const char s_FormatNbr[] PROGMEM = "%u\n";
const char s_FormatNbrSpace[] PROGMEM = "%u ";


// LED flag variables 
volatile uint8_t lnObstacle,lnLeftFlag,lnRightFlag,lnFrontFlag,TurnDirection,EnableLEDS,lnTurnDirection; 
volatile uint16_t heartbeatCycle,heartbeatCycleSize,heartbeatOnTime,Heartbeat2On,Heartbeat2Off;


// motor pwm values 
volatile uint16_t MotorLeft, MotorRight; // current PWMvalues
volatile uint8_t MotorState,ErrorState,ErrorDelay;
volatile uint16_t MotorDelay;
volatile uint8_t State,SonarDebug;
volatile uint16_t SongDelay;
volatile int16_t CommandID ;

volatile char Direction;
// encoder variables 
volatile uint8_t SawLeftTick, SawRightTick,SawLeftTick2,SawRightTick2;
volatile uint32_t LeftDistance, RightDistance,  LeftDistance2, RightDistance2;

// sonar values
volatile uint16_t Sonar1, Sonar2, Sonar3, Sonar4, Sonar5;   
volatile uint8_t SonarAddress1,SonarAddress2,SonarAddress3,SonarAddress4,SonarAddress5,CompassAddress;


// compass
volatile int16_t CurrentBearing;

// here is the time interrupt
volatile uint16_t ms_count;
volatile uint32_t TimeCounter;
volatile uint16_t Watchdog;
volatile uint32_t PowerWatch;
//
volatile uint8_t EchoInput,Gain,Range;

// MESSAGES TO LAPTOP
const char s_annc[]              PROGMEM = "$01EFK%6lx Build %s\n";
const char s_error[]             PROGMEM = "$02%6lx%2x%3i\n"; //time, state, result
//// time,front,left,right,compass,leftdistance,rightdistance,leftmotor,rightmotor,Direction
//const char s_output[]            PROGMEM = "$03%6lx%4x%4x%4x%3x%6lx%6lx%3x%3x%cZ\n"; 
// time,leftdistance,rightdistance,leftmotor,rightmotor,Direction
const char s_output[]            PROGMEM = "$03%6lx%6lx%6lx%3x%3x%c"; 
const char s_AckWatchdog[]            PROGMEM = "$04%6lx\n"; 
const char s_AckMotor[]            PROGMEM = "$05%6lx%4x\n"; 
const char s_Calibrate[]            PROGMEM = "$06%6lx%4i\n"; 
const char s_HexFour[]            PROGMEM = "%4x"; 
const char s_DisplayConfig[]            PROGMEM = "$07C %2x S %2x %2x %2x %2x %2xZ\n"; 
const char s_DisplayPort[]            PROGMEM = "$08%2xZ\n"; 
const char s_Endoutput[]            PROGMEM = "Z\n";
const char s_BuildTime[]  = __DATE__ __TIME__;
#include "fsm.c"
#include "EEConfig.c"

/*
 * timer0 compare match interrupt handler
 */
ISR(SIG_OUTPUT_COMPARE0,ISR_NOBLOCK )
{
ms_count++;
if (MotorState > 0)
	if (MotorDelay > 0)
		MotorDelay--;
//if (TickDelay > 0)
//	TickDelay--;


TimeCounter++;	
if (Watchdog > 0 )
	{
	Watchdog--;
	}
if (PowerWatch > 0 )
	{
	PowerWatch--;
	}
if (SongDelay > 0 )
	{
	SongDelay--;
	}
}

/*
 * Initialize timers - init timer 0  Use this timer as a 1 ms thread
 * scheduling timer.  
 */
void init_timers(void)
{

  /*
   * Initialize timer0   Enable output compare interrupt and set
   * the output compare register to 125 which will cause an interrupt
   * to be generated every 1 milliseconds 
   */
  TCNT0  = 0;
  OCR0   = MS_OCRMATCH;     /* non debug use 32 match in 0.9765625 ms */
  TCCR0  = _BV(WGM01) |_BV(CS02)|_BV(CS00); /* CTC, prescale = 128  */
  TIFR  |= _BV(OCIE0);
  TIMSK |= _BV(OCIE0);    /* enable output compare interrupt */
  
}

void delay(uint16_t pnDistance)
{

uint16_t Counter;
// This uses the timer0
cli();
ms_count = 0;
sei();
while (ms_count< pnDistance)
	;
Counter = ms_count;	
//printf_P(s_FormatNbrSpace,Counter);
}

void printbad(uint16_t ID)
{
snprintf_P(UartPrepBuffer,79,"\nZ%uZ\n",ID);
srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
}

SIGNAL(__vector_default)
{
// user code here
  printbad(0);
}
SIGNAL(SIG_INTERRUPT0)
{
// user code here
  printbad(1);
}
SIGNAL(SIG_INTERRUPT1)
{
// user code here
  printbad(2);
}
SIGNAL(SIG_INTERRUPT2 )
{
// user code here
  printbad(3);
}
SIGNAL(SIG_INTERRUPT3 )
{
// user code here
  printbad(4);
}
SIGNAL(SIG_INTERRUPT4 )
{

SawLeftTick = (PINE & 0x30) >> 4; // pins 4 and 5
if (Direction == 'F'|| Direction == 'R' )
{
	LeftDistance++;	 //CW increase
}
else
{
	LeftDistance--;	 //CW increase
}
 //if ( Direction == 'L')
 //	{
 //	LeftDistance--;	 //CW increase
 //	}
 //if ( Direction == 'R')
 //	{
 //	LeftDistance++;	 //CW increase
 //	}
 //if ( Direction == 'B')
 //	{
 //   LeftDistance--;	 //CW increase
 //	}
 //



}
SIGNAL(SIG_INTERRUPT5 )
{
SawLeftTick = (PINE & 0x30) >> 4; // pins 4 and 5
if (Direction == 'F'|| Direction == 'R')
{
	LeftDistance++;	 //CW increase
}
else
{
	LeftDistance--;	 //CW increase
}

//
//if ( Direction == 'L')
//	{
//	LeftDistance--;	 //CW increase
//	}
//if ( )
//	{
//	LeftDistance++;	 //CW increase
//	}
//if ( Direction == 'B')
//	{
//	LeftDistance--;	 //CW increase
//	}
//
//SawLeftTick2 = SawLeftTick;
}
SIGNAL(SIG_INTERRUPT6 )
{
// user code here
//  printbad(7);

SawRightTick= ((PINE & 0xC0) >> 6) && 3; // pins 6 and 7
if (Direction == 'F' || Direction == 'L')
{
	RightDistance++;	 //CW increase
}
else
	{
	RightDistance--;	 //CW increase
	}
//
//if ( Direction == 'L')
//	{
//	RightDistance++;	 //CW increase
//	}
//if ( Direction == 'R')
//	{
//	RightDistance--;	 //CW increase
//	}
//if ( Direction == 'B')
//	{
//	RightDistance--;	 //CW increase
//	}
//
//
//SawRightTick2 = SawRightTick;
//
}
SIGNAL(SIG_INTERRUPT7 )
{

SawRightTick= ((PINE & 0xC0) >> 6) && 3; // pins 6 and 7
if (Direction == 'F' || Direction == 'L')
{
	RightDistance++;	 //CW increase
}
else
	{
	RightDistance--;	 //CW increase
	}
//
//if ( Direction == 'L')
//	{
//	RightDistance++;	 //CW increase
//	}
//if ( Direction == 'R')
//	{
//	RightDistance--;	 //CW increase
//	}
//if ( Direction == 'B')
//	{
//	RightDistance--;	 //CW increase
//	}
//
//
//SawRightTick2 = SawRightTick;
//
}
SIGNAL(SIG_OUTPUT_COMPARE2)
{
// user code here
  printbad(9);
}
SIGNAL(SIG_OVERFLOW2)
{
// user code here
  printbad(10);
}
SIGNAL(SIG_INPUT_CAPTURE1 )
{
// user code here
  printbad(11);
}
SIGNAL(SIG_OUTPUT_COMPARE1A)
{
// user code here
  printbad(12);
}
SIGNAL(SIG_OUTPUT_COMPARE1B)
{
// user code here
  printbad(13);
}
SIGNAL(SIG_OVERFLOW3) // was 1
{
// user code here
  printbad(14);
}
// 15 SIG_OUTPUT_COMPARE0 main timer int in main.c

SIGNAL(SIG_OVERFLOW0)
{
// user code here
  printbad(16);
}
SIGNAL(SIG_SPI)
{
// user code here
  printbad(17);
}
//18 SIG_USART0_RECV inuart.c
//   SIG_UART0_RECV
SIGNAL(SIG_USART0_DATA)
{
// also SIG_UART0_DATA
// user code here
  printbad(19);
}
//20 SIG_UART0_TRANS in uart.c
//   SIG_USART0_TRANS dup

SIGNAL(SIG_ADC)
{
// user code here
  printbad(21);
}
SIGNAL(SIG_EEPROM_READY)
{
// user code here
  printbad(22);
}
SIGNAL(SIG_COMPARATOR)
{
// user code here
  printbad(23);
}
SIGNAL(SIG_OUTPUT_COMPARE1C)
{
// user code here
  printbad(24);
}
SIGNAL(SIG_INPUT_CAPTURE3 )
{
// user code here
  printbad(25);
}
SIGNAL(SIG_OUTPUT_COMPARE3A)
{
// user code here
  printbad(26);
}
SIGNAL(SIG_OUTPUT_COMPARE3B)
{
// user code here
  printbad(27);
}
SIGNAL(SIG_OUTPUT_COMPARE3C)
{
// user code here
  printbad(28);
}

// 29 SIG_OVERFLOW3 is in motor.c

//30 SIG_UART1_RECV in uart.c
//   SIG_USART1_RECV

SIGNAL(SIG_USART1_DATA)
{
// user code here
// SIG_UART1_DATA
  printbad(31);
}
// 32 SIG_USART1_TRANS in uart.c
//    SIG_UART1_TRANS
// user code here dup 


SIGNAL(SIG_2WIRE_SERIAL )
{
// user code here
  printbad(33);
}
SIGNAL(SIG_SPM_READY)
{
// user code here
  printbad(34);
}




char *readcmd(char * sCmd, char * cReturn)
{

char * sIndex;
sIndex = sCmd;
while (*sIndex && *sIndex == ' ')
	{
	sIndex++;
	}
*cReturn = *sIndex;
if ( *cReturn  != 0)
	{
	sIndex++; 	//advance ptr after command
	// skip until eos or non space
	while (*sIndex && *sIndex == ' ')
		{
		sIndex++;
		}
	}

return sIndex; 
}

char * readInt16(char * sIndex,int16_t * nResult)
{
char nbrstring[12];
uint8_t Index;

// in case there is a space skip it
while (*sIndex && *sIndex == ' ')
	{
	*sIndex++;
	}
Index = 0;
while (*sIndex && *sIndex != ' ')
	{
	nbrstring[Index] = *sIndex;
	sIndex++;
	Index++;
	}
// terminate the string	
nbrstring[Index] = 0;	
	
*nResult = atoi(nbrstring);

return sIndex;

}


char * readdouble(char * sIndex, double * nResult)
{
char nbrstring[12];
uint8_t Index;

// in case there is a space skip it
while (*sIndex && *sIndex == ' ')
	{
	sIndex++;
	}
Index = 0;
while (*sIndex && *sIndex != ' ')
	{
	nbrstring[Index] = *sIndex;
	sIndex++;
	Index++;
	}
// terminate the string	
nbrstring[Index] = 0;	

*nResult = strtod((char*) nbrstring,0);

return sIndex;
}


void do_cmd(char * sCmd)
{

char * sIndex,cCmd;
uint8_t Sregs;

int16_t nbr,nbr2;
sIndex = sCmd; 
sIndex = readcmd( sIndex,&cCmd);
while ( cCmd )
	{ 
	// put code to enter watchdog here
	if ( cCmd == 'w' || cCmd == 'W' )
		{
		Sregs =SREG; //save & clear ints
		cli();
		Watchdog = 2000;
		PowerWatch = 400000;
		SREG=Sregs; //save & clear ints
//		snprintf_P(UartPrepBuffer,79,s_AckWatchdog,TimeCounter); efk decides to remove acknowledgement because of watchdog problem i think coms is backig up
//		srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
		
		}
	else
		{
		if ( cCmd == 'm' || cCmd == 'M' )
			{
			sIndex = readInt16(sIndex,&nbr );
			CommandID = nbr;
			sIndex = readInt16(sIndex,&nbr );
			SetPWMLeft(nbr);
			sIndex = readInt16(sIndex,&nbr );
			SetPWMRight(nbr);
			//  let any motor control reset watch dog
			Sregs =SREG; //save & clear ints
			cli();
			Watchdog = 2000;
			PowerWatch = 400000;
			SREG=Sregs; //save & clear ints
//			snprintf_P(UartPrepBuffer,79,s_AckMotor,TimeCounter,CommandID); removed ack
//			srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
			
			}
		else
			{

			// calibrate compass
			if ( cCmd == 'c' || cCmd == 'C' )
				{
// 20150111 Set LED to blink really long basically on all the time
				heartbeatOnTime = 499;
				Heartbeat2On=0;
				Heartbeat2Off=0;
				sIndex = readInt16(sIndex,&nbr );
				CommandID = nbr;
				State = CALIBRATE;
//				snprintf_P(UartPrepBuffer,79,s_AckMotor,TimeCounter,CommandID);
//				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
				}
		
			// turn on or off echo command after prcessing them
			if ( cCmd == 'e' || cCmd == 'E' )
				{
				sIndex = readInt16(sIndex,&nbr );
				CommandID = nbr;
				sIndex = readInt16(sIndex,&nbr );
				if (nbr == 1)
					{
					EchoInput = 1;
					}
				else
					{
					EchoInput = 0;
					}
//				snprintf_P(UartPrepBuffer,79,s_AckMotor,TimeCounter,-200);
//				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
				}
			// set range aND gain
			if ( cCmd == 'g' || cCmd == 'G' )
				{
				sIndex = readInt16(sIndex,&nbr );
				CommandID = nbr;
				sIndex = readInt16(sIndex,&nbr );
				Range = (uint8_t) nbr;
				sIndex = readInt16(sIndex,&nbr );
				Gain = (uint8_t) nbr;
//				snprintf_P(UartPrepBuffer,79,s_AckMotor,TimeCounter,CommandID);
//				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
				MotorState = 5;
				}
			if ( cCmd == 'd' || cCmd == 'D' )
				{
				sIndex = readInt16(sIndex,&nbr );
				CommandID = nbr;
				sIndex = readcmd( sIndex,&cCmd);
				SetDirection(cCmd);
				Direction = cCmd;
				//  let any motor control reset watch dog
				Sregs =SREG; //save & clear ints
				cli();
				Watchdog = 2000;
				PowerWatch = 400000;
				SREG=Sregs; //save & clear ints
		
//				snprintf_P(UartPrepBuffer,79,s_AckMotor,TimeCounter,CommandID);
//				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
				}
		
			if ( cCmd == 'l' || cCmd == 'L' )
				{
				sIndex = readInt16(sIndex,&nbr );
				CommandID = nbr;
				sIndex = readInt16(sIndex,&nbr );
				if (nbr == 0)
					{
					EnableLEDS = 1;
					}
				else
					{
					EnableLEDS = 0;
					if (nbr > 0 && nbr < 9)
						{
						nbr -=1;
						nbr = 1<<nbr;
						if (PINC & nbr)
							{
							nbr = (~nbr);
							PORTC &= nbr;
							}
						else
							{
							//nbr = _BV(nbr);
							PORTC |= nbr;
							}
						}
					}
//				snprintf_P(UartPrepBuffer,79,s_AckMotor,TimeCounter,CommandID);
//				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
		
				}
		
			if ( cCmd == 's' || cCmd == 'S' )
				{
				SongDelay = 10000;
				sIndex = readInt16(sIndex,&nbr );
				CommandID = nbr;
//				snprintf_P(UartPrepBuffer,79,s_AckMotor,TimeCounter,CommandID);
//				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
				}
			if ( cCmd == 'a' || cCmd == 'A')
				{
				// allow Sonar and compass to be added dynamically
				sIndex = readcmd( sIndex,&cCmd);
				sIndex = readInt16(sIndex,&nbr );
				sIndex = readInt16(sIndex,&nbr2);
				
				 writeConfig(cCmd, (uint8_t) nbr,(uint8_t) nbr2);
				 readConfig();
				}
			if ( cCmd == 'r' || cCmd == 'R')
				{
				clearConfig();
				}
			if ( cCmd == 'p' || cCmd == 'P')
				{
				displayConfig();
				}		
			if (cCmd == 'q' || cCmd == 'Q')
				{
				sIndex = readInt16(sIndex,&nbr );
				SonarDebug = (uint8_t)  nbr;				
				
				}
			if ( cCmd == 'b' || cCmd == 'B')
				{
				sIndex = readInt16(sIndex,&nbr );
				// only allow certain state changes
				if (nbr == COMMAND)
					{
					State =  COMMAND;
					}
				else
					{
					if (nbr == WAITSTART)
						{
						handleWait();
						State =  WAITSTART;
						}
					}
				
				}	
			if (cCmd == 'f' || cCmd == 'F')
				{
				// Print contents of port
 				sIndex = readInt16(sIndex,&nbr );
				if (nbr == 1)
					{
					Gain = PORTA;
					}
				if (nbr == 2)
					{
					Gain = PORTB;
					}
				if (nbr == 3)
					{
					nbr = PORTC;
					}
				if (nbr == 4)
					{
					Gain = PORTD;
					}
				if (nbr == 5)
					{
					Gain = PORTE;
					}
				if (nbr == 6)
					{
					Gain = PORTF;
					}
				if (nbr == 7)
					{
					Gain = PORTG;
					}
				snprintf_P(UartPrepBuffer,79,s_DisplayPort,Gain);
				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
				}
			if (cCmd == 'h' || cCmd == 'H')
			{
			LeftDistance = 0;
			RightDistance = 0;
			}
		}
	// read either the next command or find out if we are at then end of the string
	sIndex = readcmd( sIndex,&cCmd);

	}
	
}
}


/*
 * place a function in the '.init1' section to enable external RAM so
 * that the C runtime startup can initialize it during C startup.  
 */
void ClearMotorPins (void)  
  __attribute__ ((naked)) __attribute__ ((section (".init1")));

void ClearMotorPins(void)
{
//20170715 changed to port B 5 and 6
// set these as output (pins 0 through 2 on port B)
// PORTB0 is led 
// PORTB1 PORTB2 Direction ports

DDRB |= _BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2) | _BV(PORTB5) | _BV(PORTB6); /* enable PWM outputs */
PORTE = 0;
LeftEnable = 0;
RightEnable = 0;

PORTB = 0;


}
void LigthLEDs(void)
{

// regular heartbeat
heartbeatCycle += 1;
	
if (heartbeatCycle > heartbeatCycleSize)
	{
	heartbeatCycle = 0;
	}

if (heartbeatCycle == 0)
	{
	SETB(PORTB,0) ;    // on LED 
	}
if (heartbeatCycle == heartbeatOnTime)
	{
	CLRB(PORTB,0);//  off LED 
	}
// allow for different flash patterns 
// set 	Heartbeat2On = 0 for only 1 flash 
// set 	Heartbeat2On >  heartbeatOnTime
//   and Heartbeat2Off > Heartbeat2On but < heartbeatCycleSize
//  for 2 flashes
if (Heartbeat2On > 0 && heartbeatCycle == Heartbeat2On)
		{ 
		SETB(PORTB,0) ;    // on LED 		
		}
if (Heartbeat2On > 0 && heartbeatCycle == Heartbeat2Off)
	{
	CLRB(PORTB,0);//  off LED 
	}

	
//if (EnableLEDS)
//	{
//	if (lnObstacle) 
//		{
//		SETB(PORTC,2);
//		}
//	else	
//		{
//		CLRB(PORTC,2); 
//		}
		 
//	if (lnLeftFlag) 
//		{
//		SETB(PORTC,2);
//		}
//	else	
//		{
//		CLRB(PORTC,2); 
//		}
		
//	if (lnRightFlag) 
//		{
//		SETB(PORTC,3);
//		}
//	else	
//		{
//		CLRB(PORTC,3); 
//		}
		
//	if (lnFrontFlag) 
//		{
//		SETB(PORTC,4);
//		}
//	else	
//		{
//		CLRB(PORTC,4); 
//		}
		
//	if (TurnDirection) 
//		{
//		SETB(PORTC,6);
//		}
//	else	
//		{
//		CLRB(PORTC,6); 
//		}
		
//	if (lnTurnDirection) 
//		{
//		SETB(PORTC,7);
//		}
//	else	
//		{
//		CLRB(PORTC,7); 
//		}
		
	
//	}
//else
//	{
//	}
}

int main(void)
{ 
uint8_t ch;
uint8_t Sregs;

// init all state and global variables
MotorState = MotorDelay= MotorLeft = MotorRight =  0;
LeftDistance = RightDistance = 0;

// heartbeat timers
//quick single flash initializing
heartbeatCycle = 0;
heartbeatCycleSize = 500;
heartbeatOnTime = 100;
Heartbeat2On = 0;
Heartbeat2Off = 0;

Sonar1 = Sonar2  = Sonar3 = Sonar4 = Sonar5 = 0;
CurrentBearing = 0;
Direction = 'F';
EchoInput = 1;
SonarDebug = 0; 
// set these as output on port C These are for leds
//DDRC =0xFF;
//DDRB =0x1; this already set in clear pins 20170715
// these are used for buttons
//DDRD =0xDC; 
PORTD =0xDC;  

//SETB(PORTC,0) ;
SETB(PORTB,0) ; // led on
// only sets variables
init_timers(); // setup ms timer
// need this pause before setting baud so programmer can work.
delay(MS_HALFSECOND); 
delay(MS_HALFSECOND); 
delay(MS_HALFSECOND); 

initmotor();  // set up PWM also sets PORTB0 as output (led)

// need this pause before setting baud so programmer can work.
delay(MS_HALFSECOND); 

i2c_init(); // init I2c  needed for compass and sonar

// need this pause before setting baud so programmer can work.
delay(MS_HALFSECOND); 

//SETB(PORTC,7);

sei(); // enable interupts here
EnableLEDS = 1;
delay(MS_HALFSECOND); 
//heartbeatExecute();	
delay(MS_HALFSECOND); 

// set uart0 to 56K uart1 to 4800 open device on uart1 for printf
if (setbaud())
	{
	exit(-1);  
	}

CLRB(PORTB,0); 

/* output startup announcement message */ 
snprintf_P(UartPrepBuffer,79,s_annc,TimeCounter, s_BuildTime);
srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));

//CLRB(PORTC,2) ;
// calls 1st bearing16, sets default heartbeat, and state=WAITGPSFIX

//SETB(PORTC,6);


// set up 1 second watchdog
Sregs =SREG; //save & clear ints
cli();
Watchdog = 1000;
PowerWatch = 400000;
SongDelay = 0;
SREG=Sregs; //save & clear ints
State = WAITSTART;



readConfig();

// longer single flash done initializing
heartbeatOnTime = 250;

// hardcoded this againt because the eeprom gets cleared

while (1) 
	{
	// 
	DoState();
	LigthLEDs();

	if (event.rx_int0 == 1)
		{
		while ((ch = ringbuf_get(&uartbuf0)) != 0) 
			{
			recv_input(ch);
			}
		Sregs =SREG; //save & clear ints
		cli();
		event.rx_int0 = 0;
		SREG=Sregs; //save & clear ints
		
		}
	// check watchdog to make sure we are still receiving commands from latop
	if (Watchdog == 0)
		{
		// 20150111 change led to continuous on
		if (heartbeatOnTime != 499)
			{
			heartbeatOnTime  = 499;
			}
		if (Heartbeat2On > 0)
			{
			Heartbeat2On  = 0;
			Heartbeat2Off = 0;
			}		
		
		// if watchdog timer counts down to 0 then we lost contact
		// shut motors down
		SetPWMLeft(0);
		SetPWMRight(0);
		}
	handleSong();

	if (PowerWatch == 0)
		{
			SETB(PORTC,0);
		}


	// flash led  based on state
	delay(1);
	}
}


