using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Text;
using System.Threading;
using System.Diagnostics;

namespace master
{
    /// <summary>
    /// type of object that is being transmitted over the net.
    /// </summary>
    public enum RobotOutputType : int
    {
        /// <summary>
        /// announcement
        /// </summary>
        Announcement = 1,
        /// <summary>
        /// error
        /// </summary>
        Error,
        /// <summary>
        /// sensor data output
        /// </summary>
        Sensor,
        /// <summary>
        /// acknowledge a watchdog command
        /// </summary>
        Acknowledge,
        /// <summary>
        /// acknowledge a Motor command
        /// </summary>
        MotorAcknowledge,
        /// <summary>
        /// Calibrate
        /// </summary>
        CompassCalibrate,
        //
        // configuration
        //
        Configuration

    }
    /// <summary>
    /// event argument for the event MessageReceivedHandler
    /// </summary>

    public class OutputArgs : EventArgs
    {
        public OutputArgs(string strMessage)
        {
            this.strReceivedMessage = strMessage;

        }
        public OutputArgs()
        {
        }
        public string ReceivedMessage
        {
            get
            {
                return this.strReceivedMessage;
            }
        }
        public ushort NumberSonar
        {
            get{return 2;}
        }
        public Boolean HasCompass
        {
            get { return false; }
        }
        string strReceivedMessage;
        private uint mTimeStamp;
        public uint TimeStamp
        {
            get { return mTimeStamp; }
            set { mTimeStamp = value; }
        }

        private ushort mFront=0;
        public ushort Front
        {
            get { return mFront; }
            set { mFront = value; }
        }

        ushort mLeft=0;
        public ushort Left
        {
            get { return mLeft; }
            set { mLeft = value; }
        }
        ushort mRight=0;
        public ushort Right
        {
            get { return mRight; }
            set { mRight = value; }
        }
        ushort mCompass;
        public ushort Compass
        {
            get { return mCompass; }
            set { mCompass = value; }
        }
        ushort mLeftMotor=0, mRightMotor=0;
        public ushort LeftMotor
        {
            get { return mLeftMotor; }
            set { mLeftMotor = value; }
        }
        public ushort RightMotor
        {
            get { return mRightMotor; }
            set { mRightMotor = value; }
        }
        int mLeftTicks = 0, mRightTicks = 0;
        public int LeftTicks
        {
            get { return mLeftTicks; }
            set { mLeftTicks = value; }
        }
        public int RightTicks
        {
            get { return mRightTicks; }
            set { mRightTicks = value; }
        }
        char mDirection;
        public char Direction
        {
            get { return mDirection; }
            set { mDirection = value; }
        }

    }

    public class AnnounceArgs : EventArgs
    {
        public AnnounceArgs()
        {


        }
    }

    public class ErrorArgs : EventArgs
    {

        private uint mTimeStamp;
        public uint TimeStamp
        {
            get { return mTimeStamp; }
            set { mTimeStamp = value; }
        }
        ushort mState;
        public ushort State
        {
            get { return mState; }
            set { mState = value; }
        }
        short mError;
        public short Error
        {
            get { return mError; }
            set { mError = value; }
        }


    }


    public class ACKArgs : EventArgs
    {

        private uint mTimeStamp;
        public uint TimeStamp
        {
            get { return mTimeStamp; }
            set { mTimeStamp = value; }
        }
        uint mRequestID;
        public uint RequestID
        {
            get { return mRequestID; }
            set { mRequestID = value; }
        }

    }
    public class ConfigArgs : EventArgs
    {
        private uint mCompassAddress;
        public uint CompassAddress
        {
            get { return mCompassAddress; }
            set { mCompassAddress = value; }
        }
        private uint mSonarAddress1;
        public uint SonarAddress1
        {
            get { return mSonarAddress1; }
            set { mSonarAddress1 = value; }
        }
        private uint mSonarAddress2;
        public uint SonarAddress2
        {
            get { return mSonarAddress2; }
            set { mSonarAddress2 = value; }
        }
        private uint mSonarAddress3;
        public uint SonarAddress3
        {
            get { return mSonarAddress3; }
            set { mSonarAddress3 = value; }
        }
        private uint mSonarAddress4;
        public uint SonarAddress4
        {
            get { return mSonarAddress4; }
            set { mSonarAddress4 = value; }
        }
        private uint mSonarAddress5;
        public uint SonarAddress5
        {
            get { return mSonarAddress5; }
            set { mSonarAddress5 = value; }
        }

    }

    class Com
    {

        #region event definitions
        public delegate void OutputReceivedHandler(object oSender, OutputArgs output);
        public event OutputReceivedHandler OutputReceived;
        public delegate void AnnounceReceivedHandler(object oSender, EventArgs args);
        public event AnnounceReceivedHandler AnnounceReceived;
        public delegate void ErrorReceivedHandler(object oSender, ErrorArgs args);
        public event ErrorReceivedHandler ErrorReceived;
        public delegate void ACKReceivedHandler(object oSender, ACKArgs args);
        public event ACKReceivedHandler ACKReceived;
        public delegate void ConfigReceivedHandler( object oSender, ConfigArgs args);
        public event ConfigReceivedHandler ConfigReceived; 
        #endregion event definition

        // Create the serial port with basic settings
        private SerialPort port = new SerialPort("COM1", 57600, Parity.None, 8, StopBits.One);
        private System.Threading.Thread timer;
        private System.Threading.Thread Polled;
        private System.Threading.Thread watchdog;
        private Encoding ascii = Encoding.ASCII;
        private String ContinuedPolled = "YES";
        System.IO.FileStream log;
        System.IO.FileStream purelog;
        System.IO.TextReader fs; 
        public  short UseTest = 0;
        
        public Com(String DirName)
        {
            //Console.WriteLine("Incoming Data:");
            // create a log file to echo all incoming data to for later analysis
            String filename = DirName + "\\CSensors.txt";
            log = new System.IO.FileStream(filename, System.IO.FileMode.CreateNew);
            filename = DirName + "\\PSensors.txt";
            purelog = new System.IO.FileStream(filename, System.IO.FileMode.CreateNew);
            // Attach a method to be called when there is data waiting in the port's buffer
            // uncimment for event driven comm
            // disabled because 2 coms were using all processing power
            //port.DataReceived += new SerialDataReceivedEventHandler(port_DataReceived);

            // Begin communications
            //      port.ReceivedBytesThreshold = 5;

            try
            {
                port.Open();
                StartPollingProcess();
            }
            catch (Exception ex)
            {
                UseTest = 1;
                Trace.WriteLine("Com Error" + ex.ToString());
                 byte[] ComError = { 67, 111,109, 101,114,114,10 };
                 log.Write(ComError, 0, 7);
            }

            watchdog = new Thread(new ThreadStart(this.WatchDog));
            watchdog.IsBackground = true;
            watchdog.Start();

            Trace.WriteLine("!started com");

        }
        ~Com()
        {
            Shutdown();
        }
        public void Shutdown()
        {
            if (watchdog.IsAlive)
            {
                watchdog.Abort();
            }
            if (port.IsOpen) 
            {
            port.DataReceived -= port_DataReceived;
            lock (ContinuedPolled)
            {
                ContinuedPolled = "NO";
            }
            Thread.Sleep(600);

            port.DiscardInBuffer();
            port.Close(); 
            }
            if (timer != null)
            {
                if (timer.IsAlive)
                {
                    timer.Abort();
                }
            }
        }
        private void WatchDog()
        {
            byte[] Command = { 87, 10 };
            while (1 == 1)
            {
                if (port.IsOpen)
                {
                    port.Write(Command, 0, 2);
                }
                Thread.Sleep(500);
            }
        }

        public void SendCommand(string Command)
        {
            try
            {

                byte[] Send = ascii.GetBytes(Command);
                port.Write(Send, 0, Send.Length);
                port.WriteLine("");
                log.Write(Send, 0, Send.Length); // efk added this not sure if it will work.
                log.Flush(); 
            }
            catch (Exception ex)
            {
                Trace.WriteLine("Send Command Error" + ex.ToString());
                byte[] ComError = { 67, 111, 109, 101, 114, 114, 10 };
                log.Write(ComError, 0, 7);

            }

        }

        public void StartDummy(String DirName)
        {
            fs = new System.IO.StreamReader(DirName + "\\PSensors.txt");
            timer = new Thread(new ThreadStart(this.dummydata));
            timer.IsBackground = true;
            timer.Start();

        }
        private void dummydata()
        {
            uint OldTimeStamp = 0, NewTimeStamp = 0,TimeDiff=0;
            byte[] data;
            TimeSpan t = new TimeSpan(5000);
            String line;
            int Front;
            // grab 1st line
            line = fs.ReadLine();
            while (line != null)
            {
                line += "\n"; 
                //Convert to bytes
                data = ascii.GetBytes(line);
                if (data.Length > 9)
                {
                    // find $
                    Front = Array.IndexOf(data, (byte)36);
                    // convert timestamp
                    NewTimeStamp = ParseHex(data, Front + 3, 6);
                    // get difference from last line
                    if (OldTimeStamp > NewTimeStamp)
                    {
                        TimeDiff = 1;
                    }
                    else
                    {
                        TimeDiff = NewTimeStamp - OldTimeStamp;
                    }
                    if (TimeDiff < 75)
                    {
                        TimeDiff = 75;
                    }
                    OldTimeStamp = NewTimeStamp;
                    // sleep for this time difference.
                    Thread.Sleep((int)TimeDiff);
                    // now send to regular handlers
                    PortBuffer.AddRange(data);
                    ProcessIncomingData(PortBuffer);
                    log.Write(data, 0, data.Length);
                }
                purelog.Write(data, 0, data.Length);
                // read another line
                line = fs.ReadLine();
            }
        }


        //The method's parameters are port.Read(byte[], int, int), so the first parameter is a byte array (byte[]). Try: byte[] buf = new byte[5]; 
        //But I don't recommend reading a fixed # of bytes if possible since it is very easy to get one or two bytes off in the incoming buffer and hence not receive the data when expected or get unexpected results. 
        //This is much preferred since it handles any amount of incoming data. Just process what ends up in the PortBuffer and remember to clean it out from time to time to prevent memory creep. 

        private List<byte> PortBuffer = new List<byte>();

        private void port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {

            SerialPort port = (SerialPort)sender;

            byte[] data = new byte[port.BytesToRead];

            port.Read(data, 0, data.Length);

            PortBuffer.AddRange(data);
            log.Write(data, 0, data.Length);
            log.Flush();

            byte[] logentry = Encoding.ASCII.GetBytes(System.Threading.Thread.CurrentThread.Name);
            lock (log)
            {
                log.Write(logentry, 0, logentry.Length);
                log.Flush();
            }

            purelog.Write(data, 0, data.Length);
            purelog.Flush();

            ProcessIncomingData(PortBuffer);
        }


        void StartPollingProcess()
        {
            ContinuedPolled = "YES";
            Polled = new Thread(new ThreadStart(this.PolledCom));
            Polled.IsBackground = true;
            Polled.Start();
            Thread.Sleep(200);
        }
        private void PolledCom()
        {
            while (ContinuedPolled == "YES")
            {
                if (port.BytesToRead > 0) 
                    {
                    byte[] data = new byte[port.BytesToRead];

                    port.Read(data, 0, data.Length);

                    PortBuffer.AddRange(data);
                    log.Write(data, 0, data.Length);
                    log.Flush();
                    purelog.Write(data, 0, data.Length);
                    purelog.Flush();

                    ProcessIncomingData(PortBuffer);
                    }
                Thread.Sleep(1);
            }
        }

        #region process incoming data



        private void ProcessIncomingData(List<byte> PortBuffer)
        {
            byte[] Data;

            int Front, Back = 0;
            if (PortBuffer.Count > 0)
            {
                Front = PortBuffer.IndexOf((byte)36);
                if (Front > -1)
                {
                    Back = PortBuffer.IndexOf((byte)10, Front);
                    if (Back > -1)
                    {
                        Data = new byte[Back - Front];
                        PortBuffer.CopyTo(Front, Data, 0, Back - Front);
                        Front = ((Data[1] - 48) * 10) + (Data[2] - 48);
                        switch (Front)
                        {
                            case (int)RobotOutputType.Announcement:
                                if (this.AnnounceReceived != null)
                                {
                                    AnnounceArgs oArgs = new AnnounceArgs();
                                    this.AnnounceReceived(this, oArgs);
                                }
                                break;
                            case (int)RobotOutputType.Error:

                                if (this.ErrorReceived != null)
                                {
                                    ErrorArgs oArgs = ParseError(Data);
                                    this.ErrorReceived(this, oArgs);
                                }

                                break;
                            case (int)RobotOutputType.Sensor:
                                if (this.OutputReceived != null)
                                {
                                    OutputArgs oArgs = ParseSensor(Data);
                                    this.OutputReceived(this, oArgs);
                                }

                                break;
                            case (int)RobotOutputType.Acknowledge:
                                if (this.ACKReceived != null)
                                {
                                    ACKArgs oArgs = new ACKArgs();
                                    oArgs.TimeStamp = ParseHex(Data, 3, 6);
                                    this.ACKReceived(this, oArgs);
                                }
                                break;
                            case (int) RobotOutputType.Configuration:
                                if (this.ConfigReceived != null)
                                {
                                    ConfigArgs oArgs = ParseConfig(Data);
                                    this.ConfigReceived(this, oArgs);
                                }
                                break;
                            default:
                                break;
                        }
                        PortBuffer.RemoveRange(0, Back);
                        // Search through the bytes to find the data you want 
                        // then remove the data from the list. It is good to 
                        // clean up unused bytes (usually everything before 
                        // what you're looking for)


                    }

                    //                    System.Windows.Forms.Message  M = new System.Windows.Forms.Message("a");
                    //                    MessageReceivedCallBack
                }
            }


        }
        /*
         * const char s_annc[]              PROGMEM = "$01EFK%6lx\n";
        *const char s_error[]             PROGMEM = "$02%6lx%2x%3i\n"; //time, state, result
        *    // time,leftdistance,rightdistance,leftmotor,rightmotor,Direction,compass,front,left,right
        *const char s_output[]            PROGMEM = "$03%6lx6x%6x%c%3x%3xZ\n"; 
        
        *const char s_AckWatchdog[]            PROGMEM = "$04%6lx\n"; 
        *const char s_AckMotor[]            PROGMEM = "$05%6lx%4x\n"; 
        *const char s_Calibrate[]            PROGMEM = "$06%6lx%4i\n"; 
$03   aed     0     0  0  0FZ
         * *
        */
        OutputArgs ParseSensor(byte[] Data)
        {
            int Index,maxlen = 29;
            OutputArgs oArgs = new OutputArgs();

            if (oArgs.HasCompass )
            {
                maxlen += 4;
            }
            if (oArgs.NumberSonar > 0)
            {
                maxlen += (oArgs.NumberSonar * 4);
            }
            if (Data.Length == maxlen)
            {
                if (Data[1] == 48 && Data[2] == 51)
                {
                    oArgs.TimeStamp = ParseHex(Data, 3, 6);
                    oArgs.LeftTicks = (int)ParseHex(Data, 9, 6);
                    oArgs.RightTicks = (int)ParseHex(Data, 15, 6);
                    oArgs.LeftMotor = (ushort)ParseHex(Data, 21, 3);
                    oArgs.RightMotor = (ushort)ParseHex(Data, 24, 3);
                    oArgs.Direction = (char)Data[27];
                    Index = 28;

                    if (oArgs.HasCompass)
                        {
                        oArgs.Compass = (ushort)ParseHex(Data, Index, 4);
                        Index += 4; 
                        }
                    if (oArgs.NumberSonar > 0)
                        {
                            oArgs.Front = (ushort)ParseHex(Data, Index, 4);
                            Index += 4;
                            if (oArgs.NumberSonar > 1)
                                {
                                    if (oArgs.NumberSonar > 2)
                                    {
                                        oArgs.Left = (ushort)ParseHex(Data, Index, 4);
                                    }
                                    else
                                    {
                                        oArgs.Right = (ushort)ParseHex(Data, Index, 4);
                                    }
                                Index += 4;
                                if (oArgs.NumberSonar > 2)
                                    {
                                        oArgs.Right = (ushort)ParseHex(Data, Index, 4);
                                    }
                                }
                        }

                }


            }
            return oArgs;

        }
        ErrorArgs ParseError(byte[] Data)
        {
            ErrorArgs oArgs = new ErrorArgs();
            if (Data.Length == 14)
            {
                if (Data[1] == 48 && Data[2] == 50)
                {
                    oArgs.TimeStamp = ParseHex(Data, 3, 6);
                    oArgs.State = (ushort)ParseHex(Data, 9, 2);
                    oArgs.Error = (short)ParseInt(Data, 11, 3);
                }
            }
            return oArgs;
        }


        ConfigArgs ParseConfig(byte[] Data)
        {   // 0123456790123456789012345
            // $07C 00 S 70 72 00 00 00Z\n
            

            ConfigArgs oArgs = new ConfigArgs(); 

            if (Data.Length == 25)
            {
                if (Data[1] == 48 && Data[2] == 55)
                {
                    oArgs.CompassAddress = ParseHex(Data, 4, 3);
                    oArgs.SonarAddress1 = ParseHex(Data, 10, 3);
                    oArgs.SonarAddress2 = ParseHex(Data, 13, 3);
                    oArgs.SonarAddress3 = ParseHex(Data, 16, 3);
                    oArgs.SonarAddress4 = ParseHex(Data, 19, 3);
                    oArgs.SonarAddress5 = ParseHex(Data, 22, 3);

                }


            }
            return oArgs;

        }



        #endregion

        #region parse number data

        uint ParseHex(byte[] Data, int start, int length)
        {
            int Index = 0;
            uint result = 0;

            for (Index = start; Index - start < length; Index++)
            {

                if (Data[Index] > 47 && Data[Index] < 58)
                {
                    // handle 0-9
                    result = (result << 4) + Data[Index] - (uint)48;
                }
                else
                {
                    // handle a-f
                    if (Data[Index] > 96 && Data[Index] < 103)
                    {
                        result = (result << 4) + (Data[Index] - (uint)87);
                    }
                    else
                    {
                        // handle A-F
                        if (Data[Index] > 64 && Data[Index] < 71)
                        {
                            result = (result << 4) + (Data[Index] - (uint)55);
                        }
                        else
                        {
                        }

                    }
                }
            }
            return result;
        }
        int ParseInt(byte[] Data, int start, int length)
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

        #endregion

    }
}

