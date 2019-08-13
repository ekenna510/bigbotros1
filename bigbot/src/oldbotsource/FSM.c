
 
#include "fsm.H" 
// some protypes
void delay(uint16_t pnDistance);
void LigthLEDs(void);
//----
// 20150111 Commented this out because I do not think it is getting called
// I think LightLeds in main.c is used instead.
//void heartbeatExecute(void)
//{

//// regular heartbeat
//heartbeatCycle += 1;
//	
//if (heartbeatCycle > heartbeatCycleSize)
//	{
//	heartbeatCycle = 0;
//	}
//
//if (heartbeatCycle == 0)
//	{
//	SETB(PORTB,0) ;    // on LED 
//	}
//if (heartbeatCycle == heartbeatOnTime)
//	{
//	CLRB(PORTB,0);//  off LED 
//	}
	
////-----------------------------
//// error flag error state can be negative	
////if (ErrorState != 0)
////	{
////	ErrorDelay =100;
////	ErrorState=0;
////	}
//// 5 cycles on and clear the error state
////if ( ErrorDelay == 50)
////	{
////	SETB(PORTC,ERRORLED) ;    // on LED 
////	ErrorDelay--;
////	}
////else
////	{
////	if (ErrorDelay == 0)
////		{
////		CLRB(PORTC,ERRORLED);//  off LED 
////		}
////	else
////		{
////		ErrorDelay--;
////		}
////	}
////-----------------------------
//// obstacle flag 0, 1, 2  1=obstacle 2 = stay lite
////if (lnObstacle > 0 )
////	{
////	SETB(PORTC,OBSTACLELED) ;    // on LED 
////	}
////else
////	{
////	CLRB(PORTC,OBSTACLELED);//  off LED 
////	}


////-----------------------------
//// Left flag 0, 1, 2  1=need to turn left 2 = stay lite
////if (lnLeftFlag > 0 )
////	{
////	SETB(PORTC,LEFTLED) ;    // on LED 
////	}
////else
////	{
////	CLRB(PORTC,LEFTLED);//  off LED 
////	}


////-----------------------------
//// Front flag 0, 1, 2  1=need to turn left 2 = stay lite
////if (lnFrontFlag > 0 )
////	{
////		SETB(PORTC,FRONTLED) ;    // on LED 
////	}
////else
////	{
////	CLRB(PORTC,FRONTLED);//  off LED 
////	}

////-----------------------------
//// Right flag 0, 1, 2  1=need to turn left 2 = stay lite
////if (lnRightFlag > 0 )
////	{
////	SETB(PORTC,RIGHTLED) ;    // on LED 
////	}
////else
////	{
////	CLRB(PORTC,RIGHTLED);//  off LED 
////	}
////}




#define TOOCLOSE 3600
#define VISIBLE 6400

// define buttons A-E
#define BUTTON_A_PRESSED (PIND & (1<<PIND4))
#define BUTTON_B_PRESSED (PIND & (1<<PIND2))
#define BUTTON_C_PRESSED (PIND & (1<<PIND3))
#define BUTTON_D_PRESSED (PIND & (1<<PIND6))
#define BUTTON_E_PRESSED (PIND & (1<<PIND7))



void handleWaitingForButton(void)
{
	CLRB(PORTC,0) ; // turn off sound
	if (BUTTON_A_PRESSED == 0 || BUTTON_C_PRESSED == 0 || BUTTON_D_PRESSED == 0 ) 
		{
		// turn the led on so I know to release it
		SETB( PORTB ,0);
		if (BUTTON_A_PRESSED == 0)
			{
			}
		if (BUTTON_D_PRESSED == 0)
			{
			}
		if (BUTTON_C_PRESSED == 0)
			{
			}
						
		// wait until I release the switch
		while (BUTTON_A_PRESSED  == 0 || BUTTON_D_PRESSED  == 0 || BUTTON_C_PRESSED  == 0 )
			;
		CLRB( PORTB ,0);  // led off 
		
		delay(50); 
	}
}

/*
Calibrating Rev 7 Software - Recognized by revision number label on CMPS01 
CPU chip, or read revision number from register 0.
Also applies to Calibrating the CMPS03 Module (all revisions).

Note that pin 5 (CalDone) and register 14 (Calibration Done Flag) are 
not used with Rev 7 software or the CMPS03. Pin 5 should be left 
unconnected and register 14 ignored. When calibrating the compass, 
you must know exactly which direction is North, East, South and West. 
Don't guess at it. Get a magnetic needle compass and check it. 

I2C Method
To calibrate using the I2C bus, you only have to write 255 (0xff) to 
register 15 for each of the four major compass points 
North, East, South and West. 
The 255 is cleared internally automatically after each point is calibrated. 
The compass points can be set in any order, but all four points must be 
calibrated. 
For example 
1. Set the compass module flat, pointing North. Write 255 to register 15
2. Set the compass module flat, pointing East. Write 255 to register 15
3. Set the compass module flat, pointing South. Write 255 to register 15
4. Set the compass module flat, pointing West. Write 255 to register 15
That's it.
*/
void handleCalibrateCompass(void )
{
uint8_t flag;
int8_t result;
	if (BUTTON_A_PRESSED == 0 || BUTTON_C_PRESSED == 0  ) 
		{
		SETB( PORTB ,0);
		if (BUTTON_A_PRESSED == 0   ) 
			{
			flag = 1;
			}
		else
			{
			if (BUTTON_C_PRESSED == 0)
				{
				flag = 0;
				}
			}
		while (BUTTON_A_PRESSED  == 0  || BUTTON_C_PRESSED  == 0 )
			;
		CLRB( PORTB ,0);  // led off 
		if (flag == 1)
			{
			result = CalibrateCompass(CompassAddress);
			snprintf_P(UartPrepBuffer,79,s_Calibrate,TimeCounter,result);
			srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
			 
			}
		else
			{
			State = COMMAND;
			}
		}
		
return;
}    

void handleSong(void)
{


if (SongDelay > 0)
	{
	SETB(PORTC,0);
	}
else
	{
	CLRB(PORTC,0);
	}
}
extern volatile uint16_t Sonar1, Sonar2, Sonar3, Sonar4, Sonar5;   
extern volatile uint8_t SonarAddress1,SonarAddress2,SonarAddress3,SonarAddress4,SonarAddress5,CompassAddress;;

void runSensors(void)
{
// 20150111 change led to flash twice 
if (heartbeatOnTime != 100)
	{
	heartbeatOnTime  = 100;
	}
if (Heartbeat2On != 200)
	{
	Heartbeat2On  = 200;
	Heartbeat2Off = 300;
	}
// 
uint8_t debug;
int16_t tempvalue;
int8_t nResult;
uint16_t SonarRange;
uint32_t TempLeft,TempRight;
uint8_t Sregs;
uint16_t TempMotorLeft, TempMotorRight;
char TempBuffer[80];
char TempNumber[6];

debug = SonarDebug;

	if (MotorState == 0 )
		{
		MotorState = 1; 
		}
	if (MotorState == 1 )
		{
		if (SonarAddress1 > 0)
			{
			//debug = 0; // debug flag for sonar stuff
			nResult = srf08_ping(ALLDEVICES, RANGE_US,debug);      /* initiate a ping, distance in cm */
			if (nResult < 0)
				{
				snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,2,nResult);
				srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
				i2c_init(); // try to reinit I2C
				} 
			}
		MotorDelay = 99;
		MotorState = 2;
		}
	if (MotorState == 2 )
		{
		if (MotorDelay==0)
			{
			// get the tick counter in local variable for sending to laptop
			Sregs =SREG; //save & clear ints
			cli();
			TempLeft = LeftDistance;
			TempRight = RightDistance;
			TempMotorLeft = MotorLeft;
			TempMotorRight = MotorRight;
			
			SREG =Sregs; //restore ints	
			// time,leftdistance,rightdistance,leftmotor,rightmotor,Direction
			snprintf_P(TempBuffer,79,s_output,TimeCounter, TempLeft,TempRight,TempMotorLeft,TempMotorRight,Direction);

			debug = SonarDebug; // debug flag for sonar stuff

			if (CompassAddress > 0)
				{
				tempvalue = bearing16(CompassAddress,(uint8_t) 0);
				if ( tempvalue > -1) 
					{
					CurrentBearing = tempvalue;
					}
				else
					{
					CurrentBearing = 3601;
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,6,tempvalue);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					i2c_init(); // try to reinit I2C
					}
					snprintf_P(TempNumber,79,s_HexFour,CurrentBearing);
					strlcat(  TempBuffer,  TempNumber,(size_t) 79 );

					
				}
			if (SonarAddress1 > 0)
				{
				nResult = srf08_range(SonarAddress1, 0, &SonarRange,debug); 
				if (nResult == 0 )
					{
					Sonar1 = SonarRange;
					if (Sonar1 == 0) 
						{
						Sonar1 = MAXVALUE;
						}
					}
				else 
					{
					Sonar1 = 0;
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,3,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					i2c_init(); // try to reinit I2C
					}
					snprintf_P(TempNumber,79,s_HexFour,Sonar1);
					strlcat(  TempBuffer,  TempNumber,(size_t) 79 );					
				}
			if (SonarAddress2 > 0)
				{
				nResult = srf08_range(SonarAddress2, 0, &SonarRange,debug); 
				if (nResult == 0 )
					{
					Sonar2 = SonarRange;
					if (Sonar2 == 0) 
						{
						Sonar2 = MAXVALUE;
						}
					}
				else 
					{
					Sonar2 = 0;
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,4,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					i2c_init(); // try to reinit I2C
					}
					snprintf_P(TempNumber,79,s_HexFour,Sonar2);
					strlcat(  TempBuffer,  TempNumber,(size_t) 79 );					
					
				}
			if (SonarAddress3 > 0)
				{
				nResult = srf08_range(SonarAddress3, 0, &SonarRange,debug); 
				if (nResult == 0 )
					{
					Sonar3 = SonarRange;
					if (Sonar3 == 0) 
						{
						Sonar3 = MAXVALUE;
						}
					}
				else 
					{
					Sonar3 = 0;
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					i2c_init(); // try to reinit I2C
					}
					snprintf_P(TempNumber,79,s_HexFour,Sonar3);
					strlcat(  TempBuffer,  TempNumber,(size_t) 79 );					
					
				}
			if (SonarAddress4 > 0)
				{
				nResult = srf08_range(SonarAddress4, 0, &SonarRange,debug); 
				if (nResult == 0 )
					{
					Sonar4 = SonarRange;
					if (Sonar4 == 0) 
						{
						Sonar4 = MAXVALUE;
						}
					}
				else 
					{
					Sonar4 = 0;
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					i2c_init(); // try to reinit I2C
					}
					snprintf_P(TempNumber,79,s_HexFour,Sonar4);
					strlcat(  TempBuffer,  TempNumber,(size_t) 79 );					
					
				}
			if (SonarAddress5 > 0)
				{
				nResult = srf08_range(SonarAddress5, 0, &SonarRange,debug); 
				if (nResult == 0 )
					{
					Sonar5 = SonarRange;
					if (Sonar5 == 0) 
						{
						Sonar5 = MAXVALUE;
						}
					}
				else 
					{
					Sonar5 = 0;
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					i2c_init(); // try to reinit I2C
					}
					snprintf_P(TempNumber,79,s_HexFour,Sonar5);
					strlcat(  TempBuffer,  TempNumber,(size_t) 79 );					
					
				}
			// write out Z at end of line
			snprintf_P(TempNumber,79,s_Endoutput);
			strlcat(  TempBuffer,  TempNumber,(size_t) 79 );

			//compass,front,left,right,

			// Now send all the data to the laptop
			srl_write(0, TempBuffer, strlen(TempBuffer));
			
			
			MotorState = 1;
			}
			
		}
	if (MotorState == 5 )
	{
		if (MotorDelay==0)
			{
			if (SonarAddress1 > 0)
				{
				nResult = srf08_SetMaxrange(SonarAddress1, Range, Gain, debug);
				if (nResult != 0 )
					{
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					}
				}
			if (SonarAddress2 > 0)
				{
				nResult = srf08_SetMaxrange(SonarAddress2, Range, Gain, debug);
				if (nResult != 0 )
					{
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					}
				}
			if (SonarAddress3 > 0)
				{
				nResult = srf08_SetMaxrange(SonarAddress3, Range, Gain, debug);
				if (nResult != 0 )
					{
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					}
				}
			if (SonarAddress4 > 0)
				{
				nResult = srf08_SetMaxrange(SonarAddress4, Range, Gain, debug);
				if (nResult != 0 )
					{
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					}
				}
			if (SonarAddress5 > 0)
				{
				nResult = srf08_SetMaxrange(SonarAddress5, Range, Gain, debug);
				if (nResult != 0 )
					{
					snprintf_P(UartPrepBuffer,79,s_error,TimeCounter,5,nResult);
					srl_write(0, UartPrepBuffer, strlen(UartPrepBuffer));
					}
				}
				
			MotorState = 1;
		}
	}

}
void handleWait(void)
{
	SetPWMLeft(0);
	SetPWMRight(0);

}
void DoState(void)
{
//printf_P(s_DoState,State);

	switch (State) 
		{
		case COMMAND:
			runSensors();
			break;
		case CALIBRATE:
			handleCalibrateCompass();
			break;
		case WAITSTART:
			break;


	}
}


