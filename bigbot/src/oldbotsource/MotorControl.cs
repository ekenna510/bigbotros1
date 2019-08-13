using System;
using System.Collections.Generic;
using System.Text;


namespace master
{

class MotorControl
{
private const short MinLEFT =200;
private const short MaxLEFT =500;
private const short MinRIGHT =200;
private const short MaxRIGHT =570;


private const double MaxSpeed =1.5;
private const double MinSpeed =0.2;
private const double LeftSlope= 231; //(MaxLEFT-MinLEFT)/(MaxSpeed- MinSpeed) ;
private const double RightSlope = 286;//(MaxRIGHT - MinRIGHT) / (MaxSpeed - MinSpeed);
private const double LeftY = 154;//MaxLEFT - (LeftSlope * MaxSpeed);
private const double RightY =143;// MaxRIGHT - (RightSlope * MaxSpeed);


    // prior requested speed values in feet per second
    private double mLastRequestedLeftSpeed = 0, mLastRequestedRightSpeed = 0;
    // new requested speed
    private double mRequestedLeftSpeed = 0, mRequestedRightSpeed = 0;
    // hold pwm values between set up request and get command call
    private short mLeftPWM = 0, mRightPWM = 0, mLastLeftPWM = 0, mLastRightPWM = 0;
    private char mDirection = (char)'F', mLastDirection = (char)'F';
    // set up standard speeds for each vote point
    private double[,] aSpeeds = new double [,] {
        {0.5 ,0.5 },  // Spin left
        {  0 ,0.7 },  // hard left
        {0.6 ,1.0 },  // left
        {1.0 ,1.0 },  // straight
        {1.0 ,0.6 },  // right
        {0.7 ,0 },    // hard right 
        {0.5 ,0.5 }}; // Spin right

    // require directions for ech votes point
    private char[] aDirection = "LFFFFFR".ToCharArray();

    // prior vote
    private int mLastVote=-1;

    // current vote
    private int mVote=-1;

    private Boolean mNeedNewCommand = false;
    private Boolean mNeedDirection = false;
    private DateTime mLastCommndSent = DateTime.Now;
    public Boolean NeedDirection
    {
        get
        {
            return mNeedDirection;
        }

    }
    public Boolean NeedtoSend
    {
        get
        {
            return mNeedNewCommand;
        }

    }
    private CommandSent ThisCommand = new CommandSent(0, RobotCommandTypes.MotorControl);

    public MotorControl()
    {
        // do nothing for now.
    }
    public short SpeedToPWM(double Speed, char Side)
    {
        double mdPWM;
        short msPWM;
        
        //double slope, yintercept;
        if (Speed == 0)
        {
            return 0;
        }
        if (Side == 'L')
        {
            mdPWM =  LeftY + (LeftSlope * Speed);
            msPWM = (short)mdPWM;
            if (msPWM > MaxLEFT)
                {
                msPWM  = MaxLEFT;
                }
            if (msPWM  < MinLEFT)
                {
                msPWM  = MinLEFT;
                }

        }
        else
        {
            mdPWM =  RightY + (RightSlope * Speed);
            msPWM = (short)mdPWM;
            if (msPWM > MaxRIGHT)
                {
                msPWM  = MaxRIGHT;
                }
            if (msPWM  < MinRIGHT)
                {
                msPWM  = MinRIGHT;
                }

        }
        return msPWM;
    }
    // pass in the current vote and speed and stop
    public Boolean SetMotor(VoteObject theVote, double CurrentLeftSpeed, double CurrentRightSpeed, char CurrentDirection, Boolean Stop)
    {
        Boolean bNeedToSent = false;

        short mTempLeft, mTempRight;

        if (Stop)
            {
                if (CurrentLeftSpeed != 0 && CurrentRightSpeed != 0)
                {
                // see if we already requested to stop 
                // CommandID > 0 means we sent a command
                if (mLastRequestedLeftSpeed == 0 && mLastRequestedLeftSpeed == 0 && ThisCommand.CommandID > 0)
                    {
                    // check how long ago that was sent.
                    System.TimeSpan diff = DateTime.Now - mLastCommndSent;
                    if (diff.Seconds > 1)
                    {
                        mLeftPWM = 0;
                        mRightPWM = 0;
                        bNeedToSent = true;
                    }
                    }
                else
                    {
                    // we have not previously sent stop so sent it
                        mLeftPWM = 0;
                        mRightPWM = 0;
                        bNeedToSent = true;
                    }
                }
            }
        else
        {
            // here we need to check the vote to determine a speed
            // then compare to current speed 
            // if different calc new pwm 
            // if same check how long ago we sent command 
            //    and if over 2 seconds may need to bump pwm
            mVote = theVote.VoteNumber();

            // valid vote
            if (mVote >= 0 && mVote < 7)
            {
                // get standard speeds based on vote
                mRequestedLeftSpeed = aSpeeds[mVote,0];
                mRequestedRightSpeed = aSpeeds[mVote, 1];
                mDirection = aDirection[mVote];

                mTempLeft = SpeedToPWM(mRequestedLeftSpeed, 'L');
                mTempRight= SpeedToPWM(mRequestedRightSpeed, 'R');
                // is requested speed same as last speed sent and same direction
                if (mRequestedLeftSpeed == mLastRequestedLeftSpeed && mRequestedRightSpeed == mLastRequestedRightSpeed  && CurrentDirection == mDirection  )
                {
                    // check how long ago that was sent.
                    System.TimeSpan diff =  DateTime.Now - ThisCommand.TimeStamp;
                    if (diff.Seconds > 2)
                    {
                        // check are we not at the desired speed 
                        if (CurrentLeftSpeed > (mRequestedLeftSpeed + .05))
                        {
                            if (mLeftPWM - 3 >= mTempLeft * .9)
                            {
                                mTempLeft -= 3;
                                bNeedToSent = true;
                            }

                        }
                        else if (CurrentLeftSpeed < (mRequestedLeftSpeed - .05))
                        {
                            if (mLeftPWM + 3 <= mTempLeft * 1.1)
                            {
                                mTempLeft += 3;
                                bNeedToSent = true;
                            }

                        }

                        // check are we not at the desired speed 
                        if (CurrentRightSpeed > (mRequestedRightSpeed + .05))
                        {
                            if (mRightPWM - 3 >= mTempRight * .9)
                            {
                                mTempLeft -= 3;
                                bNeedToSent = true;
                            }

                        }
                        else if (CurrentRightSpeed < (mRequestedRightSpeed - .05))
                        {
                            if (mLeftPWM + 3 <= mTempRight * 1.1)
                            {
                                mRightPWM += 3;
                                bNeedToSent = true;
                            }

                        }
                        if (bNeedToSent = true)
                        {
                            mLeftPWM = mTempLeft;
                            mRightPWM = mTempRight;
                        }

                    }
                }
                else
                {
                    // speeds are different
                    if (CurrentDirection != mDirection)
                    {
                        if (CurrentLeftSpeed == 0 && CurrentRightSpeed == 0)
                        {
                            // need to change direction 
                            mNeedDirection = true;
                            bNeedToSent = true;
                        }
                        else
                        {
                            // need to stop
                            mLeftPWM = 0;
                            mRightPWM = 0;
                            bNeedToSent = true;
                        }

                    }
                    else
                    {
                        mLeftPWM = SpeedToPWM(mRequestedLeftSpeed, 'L');
                        mRightPWM = SpeedToPWM(mRequestedRightSpeed, 'R');
                        bNeedToSent = true;
                    }

                }
            
            }
        }
        // assume we will send the command
        if (bNeedToSent)
            {
            // reset last
                mLastRequestedLeftSpeed = mRequestedLeftSpeed;
                mLastRequestedRightSpeed = mRequestedRightSpeed;
                mLastDirection = mDirection;
            }
        mNeedNewCommand = bNeedToSent;
        return bNeedToSent;
    }


    
    public CommandSent MotorCommand(uint MessageID)
    {
        ThisCommand.TimeStamp = DateTime.Now;
        mLastCommndSent = DateTime.Now;
        if (mNeedDirection)
        {
            ThisCommand.CommandID = MessageID;
            ThisCommand.CommandType = RobotCommandTypes.Direction;
            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mDirection;
            mNeedDirection = false;
        }
        else
        {
            ThisCommand.CommandID = MessageID;
            ThisCommand.CommandType = RobotCommandTypes.MotorControl;
            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mLeftPWM.ToString() + " " + mRightPWM.ToString();
            //mLastDirection keep existing direction

        }
        mLastDirection = mDirection;
        mLastLeftPWM = mLeftPWM;
        mLastRightPWM = mRightPWM;

    return ThisCommand;
   

    }
}
}
