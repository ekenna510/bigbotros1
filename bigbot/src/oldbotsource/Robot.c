using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using Camera;
using System.Diagnostics;
using master;


namespace master
{

    public enum CameraControlProperty
    {
        Pan = 1,
        Tilt,
        Roll,
        Zoom,
        Exposure,
        Iris,
        Focus
    }

    public enum RobotCommandTypes:byte
    {
        CalibrateCompass = (byte)'C',
        Watchdog = (byte)'W',
        MotorControl = (byte) 'M',
        Leds = (byte) 'L',
        Song = (byte) 'S',
        Direction = (byte) 'D',
        Echo = (byte) 'E',
        Gain = (Byte) 'G',
        StartSensor = (Byte) 'B',
        DisplayConfig = (Byte) 'P',
        AddSonar = (Byte) 'A',
        ClearConfig = (Byte) 'R',
        SonarDebug = (Byte) 'Q',
        QueryPort = (Byte) 'F',
        ResetEncoder = (Byte) 'H'


    }
    public enum CameraControlFlags
    {
        None = 0x0,
        Auto = 0x0001,
        Manual = 0x0002
    }



    #region RobotModeType

    public enum RobotModeType : short 
    {
        /// <summary>
        /// Robot operates under user control
        /// </summary>
        RemoteControl = 1,
        /// <summary>
        /// Robot is following a color
        /// </summary>
        ColorFollow,
        /// <summary>
        /// robot is navigating a path with waypoints
        /// </summary>
        Auto,
        /// <summary>
        /// Rotate until sonar is > 3600
        /// </summary>
        AvoidObstacle,
        /// <summary>
        /// Rotate until we find color
        /// </summary>
        FindColor,
        /// <summary>
        /// wait state to make sure no other mode is continuing
        /// </summary>
        WaitState,
        /// <summary>
        /// wait until sonar is > 3600
        /// </summary>
        WaitForPerson,
        /// <summary>
        /// wait until Slave is connected and intialized
        /// </summary>
        LookForSlave

    }
    #endregion

    public class CommandSent
    {
        public CommandSent(uint pCommandID, RobotCommandTypes pCommandType)
        {
            mCommandID = pCommandID;
            mCommandType = pCommandType;
            mTimeStamp = DateTime.Now;
        }
        private DateTime  mTimeStamp;
        public DateTime TimeStamp
        {
            get { return mTimeStamp; }
            set { mTimeStamp = value;}
         }
        private RobotCommandTypes mCommandType;
        public RobotCommandTypes CommandType
        {
            get { return mCommandType; }
            set { mCommandType = value; }
        }

        private string mCommandstring;
        public string Commandstring
        {
            get
            {
                return this.mCommandstring;
            }
            set
            {
                mCommandstring = value;
            }
        }
        private uint mCommandID;
        public uint CommandID
        {
            get
            {
                return this.mCommandID;
            }
            set
            {
                this.mCommandID = value;
            }
        }
        public void RecordIt(System.IO.FileStream plog)
        {
            DateTime dt  = DateTime.Now;
            string Logentry = System.Convert.ToString(this.mCommandID) + " " + dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " " + System.Convert.ToString(this.mCommandType) 
                 + " " + this.Commandstring + "\n";
            Byte[] bytes = Encoding.ASCII.GetBytes(Logentry);
            lock (plog)
            {
                plog.Write(bytes, 0, bytes.GetLength(0));
                plog.Flush();
            }
        }

    }


    public class Robot
    {

        public const int FollowTargetX = 160;
        public const int FollowTargetY = 120;

        RobotModeType mMode;
        RobotModeType mPriorMode= RobotModeType.RemoteControl;
        private static master.OutputArgs[] sensors = new master.OutputArgs[10];

        private static master.IMUArg ImuValues = new IMUArg();  
        private master.VoteObject ImageVote = new VoteObject();
        private static Int16 CurrentSensor = -1;
        private Int16 mRemoteLeft, mRemoteRight;
        private System.Threading.Thread MainThread;
        private System.Threading.Thread ImageThread;
        private String MainThreadKeepAlive = "Continue";
        private String ImageThreadKeepAlive = "Continue";

        Com com;
        public IMU Imu=null;
        private uint MessageID =1;
        private UInt64 ErrorCount=0;
        private List<CommandSent> CommandWaitingList = new List<CommandSent>();
        private bool SlaveStarted = false;
        private Camera.TheCamera myCamera;
        // variables for tracking object 
        private int[] FollowX = { -1, -1, -1, -1, -1,-1,-1,-1, -1, -1 };
        private int[] FollowY = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
        private int NavIndex = 0;
        private int FollowIndex = 0;
        private Form1 myparent;
        System.IO.FileStream log;
      //  System.IO.FileStream Cameralog;
        private bool LedsEnabled = true;
        private byte leds = 0;
        private int EchoCommand = 0;
        private long MsgCount = 0;
        private double gnCurrentX=0, gnCurrentY=0, gnWayDistance;
        private double gnBearing = 0;
        private double[,] gnWay = { { 25, 0 }, { 0, 0 } };
        // comment out the 2 lines above and uncomment the 2 below
        //real waypoints        
        //private double gnCurrentX = 312.5, gnCurrentY = 519.6, gnWayDistance;
        //private double[,] gnWay = { {  303.29, 369.2, 429.3, 489.37, 410.13, 358.74, 307.48, 344.97 }, { 628.83, 703.8, 646, 385.83, 367.73, 258.55, 252.54, 428.50  } };
        private double gnTargetBearing = 0;
        private int gnWayIndex = 0;
        private int PersonDelay = 0;
        private int PersonDelayWarning = 0;
        public double MyTargetBearing;
        public double myDistance = 10;
        string LastFiltered = "";
        String CaptureVideo = "NO";
        private String InputDirName = "";
        private ushort StartSonar;
        private master.FollowColor FollowColor = null;
        private master.SonarVote sonarVote = null;
        private master.BearingVote bearingVote = null;
        private master.BaseImgProcess ImageProcess = null;
        protected LogMessageArg logArg = new LogMessageArg();
        private master.IMUVote ImuVote = null;
        private Int16 MaxLeft = 350;
        private Int16 MaxRight = 370;
        private Int16 MinLeft = 220;
        private Int16 MinRight = 220;

        private MotorControl motorControl = new MotorControl();
        #region properties
        public double X
        {
            get 
            {
                return gnCurrentX;
            }
        }
        public double Y
        {
            get
            {
                return gnCurrentY;
            }
        }

        public RobotModeType Mode
        {
            get
            {
                return this.mMode;
            }
            set
            {
                if (value  == RobotModeType.ColorFollow)
                    {
                        // either we must set the matchcolor or be in test mode
                        if (myCamera.UseTest == 1 || FollowColor.IsMatchColorSet)
                        {
                            // grab prior mode 
                            mPriorMode = mMode;
                            //set mode to wait state so main loop can stop doing old mode
                            mMode = RobotModeType.WaitState;
                               

                            // play the sound
                            System.Media.SoundPlayer sndPing = new System.Media.SoundPlayer("C:\\Documents and Settings\\Admin\\My Documents\\master\\StartFollow.wav");
                            sndPing.Play();

                            // give main loop time to stop other mode
                            // 1 second should work,
                                Thread.Sleep(1000);


                            if (myCamera.UseTest == 1 || com.UseTest == 1)
                            {
                                System.Windows.Forms.FolderBrowserDialog folderBrowserDialog1 = new System.Windows.Forms.FolderBrowserDialog();
                                folderBrowserDialog1.Description = "select a sensor data drectory";
                                folderBrowserDialog1.ShowNewFolderButton = false;
                                folderBrowserDialog1.RootFolder = Environment.SpecialFolder.MyComputer;//MyDocuments;
                                System.Windows.Forms.DialogResult result = folderBrowserDialog1.ShowDialog();
                                if (result == System.Windows.Forms.DialogResult.OK)
                                {
                                    String Dirname = folderBrowserDialog1.SelectedPath;

                                    if (com.UseTest == 1)
                                    {
                                        com.StartDummy(Dirname);
                                    }
                                    if (myCamera.UseTest == 1)
                                    {
                                        System.IO.TextReader fs = new System.IO.StreamReader(Dirname + "\\Camera.txt");
                                        String Line = fs.ReadLine();
                                        char[] a = { ' ' };
                                        String[] MyString = Line.Split(a);
                                        FollowColor.SetMatchColor(System.Drawing.Color.FromArgb(System.Convert.ToInt32(MyString[2]), System.Convert.ToInt32(MyString[4]), System.Convert.ToInt32(MyString[6])));
                                        myCamera.TestInputDirName = Dirname;

                                    }
                                }
                            }
                            // Set to image processor to colorfollow
                            SetImageProcessor(99);
                            // finally set to new mode
                            mMode = RobotModeType.ColorFollow;

                        }
                        else
                        {
                            System.Windows.Forms.MessageBox.Show("Follow Color not set " , "Robot Mode", System.Windows.Forms.MessageBoxButtons.OK, System.Windows.Forms.MessageBoxIcon.Stop);
                        }

                }
                if (value == RobotModeType.Auto)
                {
                    mPriorMode = mMode;
                    //set mode to wait state so main loop can stop doing old mode
                    mMode = RobotModeType.WaitState;
                    RemoteDirection = 'F';
                    // give main loop time to stop other mode

                    System.Media.SoundPlayer sndPing = new System.Media.SoundPlayer("C:\\Documents and Settings\\Admin\\My Documents\\master\\Startnav.wav");
                    sndPing.Play();
                        
                    // 1 second should work,
                    Thread.Sleep(1000);

                    // if we need to set the matchcolor
                    if (!ImageProcess.IsMatchColorSet)
                    {
                        if (myCamera.UseTest == 0)
                        {
                            //Snap an image 
                            ImageProcess.Image = myCamera.button1_Click();
                            // set the matchcolor
                            ImageProcess.SetMatchColor();
                            //TempColor = ImageProcess.MatchColor;
                        }
                    }

                    if (ImageProcess is master.NavOnly)
                    {
                        // Assume start = 0,0 need to change this.
                        CalcHeadingDistanceXY();

                    }

                    // running simlator
                    if (myCamera.UseTest == 1 || com.UseTest == 1)
                    {
                        // only do this if comm is not already started
                        if (!SlaveStarted)
                        {
                            System.Windows.Forms.FolderBrowserDialog folderBrowserDialog1 = new System.Windows.Forms.FolderBrowserDialog();
                            folderBrowserDialog1.Description = "select a sensor data drectory";
                            folderBrowserDialog1.ShowNewFolderButton = false;
                            folderBrowserDialog1.RootFolder = Environment.SpecialFolder.MyComputer; //MyDocuments;
                            System.Windows.Forms.DialogResult result = folderBrowserDialog1.ShowDialog();
                            if (result == System.Windows.Forms.DialogResult.OK)
                            {
                                String Dirname = folderBrowserDialog1.SelectedPath;

                                if (myCamera.UseTest == 1 || com.UseTest == 1)
                                {
                                    com.StartDummy(Dirname);
                                }
                                if (myCamera.UseTest == 1)
                                {
                                    //System.IO.TextReader fs = new System.IO.StreamReader(Dirname + "\\Camera.txt");
                                    //String Line = fs.ReadLine();
                                    //char[] a = { ' ' };
                                    //String[] MyString = Line.Split(a);
                                    myCamera.TestInputDirName = Dirname;

                                    if (!ImageProcess.IsMatchColorSet)
                                    {
                                        ImageProcess.Image = myCamera.button1_Click();
                                        ImageProcess.SetMatchColor();
                                    }
                                }
                                ImageProcess.InputDirName = InputDirName;
                            }
                        }
                    }
                    mMode = value;
                    lock (logArg)
                    {
                        logArg.LogEntry = "Mode Start\n";
                        LogEntryReceivedCallBack(this, logArg);
                    }
                    

                }
                if (value == RobotModeType.AvoidObstacle || value == RobotModeType.FindColor || value == RobotModeType.LookForSlave || value == RobotModeType.RemoteControl || value == RobotModeType.WaitForPerson || value == RobotModeType.WaitState )
                {
                    mMode = value;
                }

                
            }
        }
        public Int16  RemoteLeft
        {
            get
            {
                return this.mRemoteLeft;
            }
            set 
            { 
                this.mRemoteLeft = value;
                MaxLeft = value;
        }
        }

        public Int16 RemoteRight
        {
            get
            {
                return this.mRemoteRight;
            }
            set 
            { 
                this.mRemoteRight = value;
                MaxRight = value;
            }
        }

        private char mRemoteDirection;

        public char RemoteDirection
        {
            get
            {
                return this.mRemoteDirection;
            }
            set
            {
                if (value == (byte)'F' || value == (byte)'B' || value == (byte)'L' || value == (byte)'R')
                {
                this.mRemoteDirection = value; 
                }
            }
        }

        public ushort LeftPWM
        {
            get {
            if (CurrentSensor > -1)
            {
                return sensors[CurrentSensor].LeftMotor ;
            }
            else{
                return 0;
            }
            }
        }
        public ushort RightPWM
        {
            get {
            if (CurrentSensor > -1)
            {
                return sensors[CurrentSensor].RightMotor  ;
            }
            else{
                return 0;
            }
            }
        }
        public int LeftTick
        {
            get
            {
            int Speed=0;
            int prior = CurrentSensor -1;
            if (prior < 0 )
            {
                prior = 9;
            }
            if (CurrentSensor > -1)
            {
                if (sensors[prior] == null)
                {
                    Speed = sensors[CurrentSensor].LeftTicks ;
                }
            }
            return Speed;

            }
        }
        public int RightTick
        {
            get
            {
                int Speed = 0;
                int prior = CurrentSensor - 1;
                if (prior < 0)
                {
                    prior = 9;
                }
                if (CurrentSensor > -1)
                {
                    if (sensors[prior] == null)
                    {
                        Speed = sensors[CurrentSensor].RightTicks; 
                    }
                }
                return Speed;

            }
        }
        public double LeftSpeed
        {
            get {
            int clicks;
            double Speed;
            int prior = CurrentSensor -1;
            if (prior < 0 )
            {
                prior = 9;
            }
            if (CurrentSensor > -1)
            {
                if (sensors[prior] == null)
                {
                    clicks = sensors[CurrentSensor].LeftTicks - 0;
                }
                else
                {
                    clicks = sensors[CurrentSensor].LeftTicks - sensors[prior].LeftTicks;
                }
                // left 0.036344410876132930513595166163142 inch per tick
                //0.0030287 feet per tick
        //   0.092326283987915407854984894259819 cm per inch
                // assume click could every 100ms 
                // time 10 to get speed per second
                //0.0030287 *10
                Speed = clicks * 0.02908081; //0.030287; original
                return Speed   ;
            }
            else{
                return 0;
            }
            }
        }
        public double RightSpeed
        {
            get {
            int clicks;
            double Speed;
            int prior = CurrentSensor -1;
            if (prior < 0 )
            {
                prior = 9;
            }
            if (CurrentSensor > -1)
            {
                if (sensors[prior] == null)
                {
                    clicks = sensors[CurrentSensor].RightTicks - 0;
                }
                else
                {
                    clicks = sensors[CurrentSensor].RightTicks - sensors[prior].RightTicks;
                }
                // right 0.017855268091488563929508811398575 in per tick
                // 0.045358080239970003749531308586427cm per tick

                // assume click could every 100ms 
                // time 10 to get speed per second
                // 0.01785527 *10
              //  Speed = clicks * 0.01785527;
                Speed = clicks * 0.015408; // original 0.016; //  oover rode orig value
                return  Speed  ;
            }
            else{
                return 0;
            }
            }
        }
        public ushort  Bearing
        {

            get {
                ushort m;
                
            if (CurrentSensor > -1)
            {
                if (sensors[CurrentSensor].Compass == 3601)
                {
                    m = 361;
                }
                else
                {
                    // Need to account for variaion
                    m = sensors[CurrentSensor].Compass;
                    m /= 10;

                }
                return m;
            }
            else
            {
                return 361;
            }
            }
        }

        public ushort  Front
        {
            get
            {
                if (CurrentSensor > -1)
                {
                    return sensors[CurrentSensor].Front;
                }
                else
                {
                    return 0;
                }
            }
        }
        public ushort Left
        {
            get
            {
                if (CurrentSensor > -1)
                {
                    return sensors[CurrentSensor].Left;
                }
                else
                {
                    return 0;
                }
            }
        }
        public ushort Right
        {
            get
            {
                if (CurrentSensor > -1)
                {
                    return sensors[CurrentSensor].Right;
                }
                else
                {
                    return 0;
                }
            }
        }

        public char Direction
        {
            get
            {
                if (CurrentSensor > -1)
                {
                    return sensors[CurrentSensor].Direction ;
                }
                else
                {
                    return ' ';
                }
            }
        }

        public uint Timestamp
        {
            get
            {
                if (CurrentSensor > -1)
                {
                    return sensors[CurrentSensor].TimeStamp ;
                }
                else
                {
                    return 0;
                }
            }
        }
        public string Message
        {
            get
            {
                if (CurrentSensor > -1)
                {
                    return sensors[CurrentSensor].ReceivedMessage  ;
                }
                else
                {
                    return "";
                }
            }
        }
        public byte Leds
        {
            get
            {
                return leds;
            }
            set
            {
                string command;
                byte pLed = value;
                if (pLed == 0 || LedsEnabled)
                    {
                    command = "L " + MessageID.ToString() + " 0";
                    com.SendCommand(command);
                    MessageID++;
                    if (pLed == 0 && !LedsEnabled)
                    {
                        LedsEnabled = true;
                    }
                    else
                    {
                        LedsEnabled = false;
                    }
                    }
                if (pLed > 0 && pLed < 9)
                    {
                        command = "L " + MessageID.ToString() + pLed.ToString();
                        com.SendCommand(command);
                        MessageID++;
                        LedsEnabled = false;
                    pLed--;
                    pLed = (byte)(1 << pLed);
                    if ((leds & pLed) > 0)
                    {
                        pLed = (byte)(~pLed);
                        leds  &= pLed;

                    }
                    else
                    {
                        leds  |= pLed;
                    }
                    }
            }
        }
/*        public System.Drawing.Color FollowColor
        {
            get
            {
                return this.myCamera.FollowColor;
            }
            set
            {
                this.myCamera.FollowColor = value;
            }
        }
        public System.Drawing.Color NavColor
        {
            get
            {
                return this.myCamera.NavColor;
            }
            set
            {
                this.myCamera.NavColor = value;
            }
        }
*/
        #endregion
         ~Robot()
        {
            Shutdown();
        }
        public void Shutdown()
        {
            this.Mode = RobotModeType.WaitState;
            MainThreadKeepAlive = "Shutdown";
            if (Imu != null)
                {
                Imu.Shutdown();
                }
            Thread.Sleep(1000);
            if (com != null)
            {
                com.Shutdown();
                com = null;
            }

            if (Imu != null)
            {
                Imu = null;  
            }
            if (myparent != null)
            {
                myparent = null;
            }
            if (log != null)
            {
                log.Close();
                log.Dispose();
            }
            if (myCamera != null)
            {
                myCamera = null;
            }
        }
        public Robot(object parent)
        {

            myparent = (Form1)parent;

            mMode = RobotModeType.LookForSlave;
            mRemoteLeft = 0;
            mRemoteRight = 0;
            mRemoteDirection = (char)'F';
            // reading waypoint file if exists
            String WayFile = AppDomain.CurrentDomain.BaseDirectory + "waypoints.txt";
            //WayFile = Environment.CurrentDirectory;
            if (System.IO.File.Exists(WayFile))
            {
                System.IO.StreamReader fs = new System.IO.StreamReader(WayFile);
                String line;
                char[] delimiters = {' ',',','\t'};
                String[] parts;
                int numWaypoints=-1;
                double tempX,tempY;
                line = fs.ReadLine();
                while (line != null)
                {
                    parts = line.Split(delimiters);
                    if (line.StartsWith("S"))
                    {
                        tempX = Convert.ToDouble(parts[1]);
                        tempY = Convert.ToDouble(parts[2]);
                        gnCurrentX = tempX;
                        gnCurrentY = tempY;
                    } 
                    else if (line.StartsWith("N"))
                    {
                        numWaypoints = Convert.ToInt16(parts[1]);
                        gnWay = new double [numWaypoints,2];
                        gnWayIndex = 0;
                    }
                    else if (line.StartsWith("W"))
                    {
                        tempX = Convert.ToDouble(parts[1]);
                        tempY = Convert.ToDouble(parts[2]);
                        gnWay[gnWayIndex, 0] = tempX;
                        gnWay[gnWayIndex, 1] = tempY;
                        gnWayIndex++;
                        if (gnWayIndex >= numWaypoints)
                        {
                            break;
                        }
                    }
                    line = fs.ReadLine();

                }

                gnWayIndex = 0;
            }
            
            // create a directory to hold all log and image data
            // create name based on time 
            String DirName = "C:\\users\\Dad\\My Documents\\" + DateTime.Now.ToString("yyyyMMddHHmmss") ;

            if (!System.IO.Directory.Exists(DirName))
            {
                System.IO.DirectoryInfo di = System.IO.Directory.CreateDirectory(DirName);
            }
 
            // set up command log
            String filename = DirName + "\\Motor.txt";
            log = new System.IO.FileStream(filename , System.IO.FileMode.CreateNew);    

            // create a comobject to hanlde talking to bot
            com = new Com(DirName);
            // read in any configuration settings like initial x,y waypoints,operation mode

            com.OutputReceived += new Com.OutputReceivedHandler(this.ComOutputReceivedCallBack);
            com.ErrorReceived += new Com.ErrorReceivedHandler(this.ComErrorReceivedCallBack);
            com.ACKReceived += new Com.ACKReceivedHandler(this.ComACKReceivedCallBack);
            com.AnnounceReceived += new Com.AnnounceReceivedHandler(this.ComAnnouceReceivedCallBack);
            com.ConfigReceived += new Com.ConfigReceivedHandler(this.ComConfigReceivedCallBack);

            Imu = new IMU();
            Imu.OutputReceived += new IMU.OutputReceivedHandler(IMUCallBack);

            myCamera = new Camera.TheCamera(DirName,myparent.livestream.Handle  );
            // have all logging done in robot have objects raise event to record it need to figure concurrecny
            //filename = DirName + "\\Camera.txt";
            //Cameralog = new System.IO.FileStream(filename, System.IO.FileMode.CreateNew);
            //ImageProcess = new master.NavEdge();
            //ImageProcess.LogReceived += new BaseImgProcess.CameraLogHandler(this.LogEntryReceivedCallBack);
            //ImageProcess.VoteReceived += new BaseImgProcess.VoteHandler(this.ImageVoteReceivedCallBack);
            SetImageProcessor(5);//NavChroma2

            //FollowColor = new master.FollowColor();
            //NavColor = new master.NavColor( );
            //NavColor.LogReceived += new BaseImgProcess.CameraLogHandler(this.LogEntryReceivedCallBack);
            sonarVote = new master.SonarVote();
            sonarVote.LogReceived += new SonarVote.CameraLogHandler(this.LogEntryReceivedCallBack);
            bearingVote = new master.BearingVote();
            bearingVote.LogReceived += new BearingVote.CameraLogHandler(this.LogEntryReceivedCallBack);
            //Navchroma = new master.NavChroma(Cameralog);
            //Navblue = new master.NavBlue(Cameralog);
           // ImageProcess = new master.NavEdge(Cameralog);
            //ImageProcess = new master.NavChroma(Cameralog);
            ImuVote = new IMUVote();

            ImuVote.LogReceived += new IMUVote.CameraLogHandler(this.LogEntryReceivedCallBack);


            InputDirName = DirName;

            MainThread = new Thread(new ThreadStart(this.MainThreadLoop));
            MainThread.IsBackground = true;
            MainThread.Start();

            ImageThread = new Thread(new ThreadStart(this.ImageThreadLoop));
            ImageThread.IsBackground = true;
            ImageThread.Start();

        }

        #region sensor callback

        void ComOutputReceivedCallBack(object oSender, OutputArgs output)
        {
            CurrentSensor++;
            if (CurrentSensor > 9)
            {
                CurrentSensor = 0;
            }
            sensors[CurrentSensor] = output;

            // process sensor data 
            MsgCount++;

            myparent.SetPacketCount(MsgCount.ToString());
            Map(); //20150116 Nolonger doing mapping
            // This is not needed this main thread is on a 100 ms sleep and should be aloowed to wake on its own.

        }

        void ComErrorReceivedCallBack(object oSender, ErrorArgs args)
        {
            ErrorCount++;
            myparent.SetErrorCount(ErrorCount.ToString());
        }
        void ComAnnouceReceivedCallBack(object oSender, EventArgs args)
        {
            // robot system is rebooting clear count.
            //MsgCount = ErrorCount = 0;
            //this.lblPacketCount.Text = MsgCount.ToString();
            //this.lblErrorCount.Text = ErrorCount.ToString();
            //System.Windows.Forms.FormCollection f = 


            // set echo on 
            CommandSent ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Echo);
            //byte[] 
            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Echo).ToString() + " " + ThisCommand.CommandID.ToString() + " " + EchoCommand.ToString();
            CommandWaitingList.Add(ThisCommand);
            com.SendCommand(ThisCommand.Commandstring);
            ThisCommand.RecordIt(log);
            MessageID++;

            // display config
            ThisCommand = new CommandSent(MessageID, RobotCommandTypes.DisplayConfig);
            //byte[]
            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.DisplayConfig).ToString() ;
            CommandWaitingList.Add(ThisCommand);
            com.SendCommand(ThisCommand.Commandstring);
            ThisCommand.RecordIt(log);
            MessageID++;


        }
        void ComConfigReceivedCallBack(object oSender, ConfigArgs args)
        {
            CommandSent ThisCommand;
            Boolean OkToStart;
            OkToStart = true ;
       
            if (args.SonarAddress1 == 0 || args.SonarAddress1 != 114)
            {
                OkToStart = false;
                // Need to add sonars
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.AddSonar);
                //byte[]
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.AddSonar).ToString() + " S 1 114";
                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;


            }

            if (args.SonarAddress2 == 0 || args.SonarAddress2 != 112)
            {
                OkToStart = false;
                // Need to add sonars
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.AddSonar);
                //byte[]
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.AddSonar).ToString() + " S 2 112";
                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;


            }

            if (OkToStart)
            {

                // Set a default gain and range 
                // valid gain =1 to 31 valid range = 1 to 255  
                // I have set this to 1/3 the gain and 1/3 range approx 11 feet
                this.SetGainRange(10, 80);

                // startSensor
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.StartSensor);
                //byte[]
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.StartSensor).ToString() + " 1 ";
                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;

                if (this.com.UseTest != 1)
                {
                    // Start listening for IMU
                    this.Imu.Connect();
                    String test = this.Imu.ToString();

                }
                SlaveStarted = true;
                myparent.SetRobotStatus("On line");
                // Set  direction to what is in robot
                // change direction to L 
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " L" ;

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;
                // then to what is in RemoteDirection. This is to fake out the slave.
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;

            }
            else
            {
                // display config
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.DisplayConfig);
                //byte[]
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.DisplayConfig).ToString();
                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;

            }



            
        }
        void ComACKReceivedCallBack(object oSender, ACKArgs args)
        {
            // a command has been acknowledged

            foreach (CommandSent iCommand in CommandWaitingList)
            {
               if (iCommand.CommandID == args.RequestID)
               {
                   CommandWaitingList.Remove (iCommand);
               }
            }


        }
        #endregion
        #region IMU callback
        void IMUCallBack(object oSender, master.IMUArg  args)
        {
            LogMessageArg entry = new LogMessageArg();

            lock (ImuValues)
            {
            ImuValues = args;
            entry.LogEntry = ImuValues.OutputString;
            }
            this.LogEntryReceivedCallBack(this, entry);
        }
        #endregion

        #region log callback
        void LogEntryReceivedCallBack(object oSender, LogMessageArg LogEntry)
        {

            Byte[] bytes = Encoding.ASCII.GetBytes(LogEntry.LogEntry );
            lock (log)
            {
                log.Write(bytes, 0, bytes.GetLength(0));
                log.Flush();
            }
            // This is not needed this main thread is on a 100 ms sleep and should be aloowed to wake on its own.

        }
        void ImageVoteReceivedCallBack(object oSender,VoteObject TheVote)
        {
            lock (ImageVote)
            {
                ImageVote = TheVote;
            }

        }
        #endregion

        #region VideoCallback
        //void CamFollowUpdateHandler(object oSender, CamFollowUpdateArgs args)
        //{
        //    try
        //    {
        //        // FollowIndex needs to point to the current value.
        //        // therefore we increment it each time.
        //        FollowIndex++;
        //        if (FollowIndex > 9)
        //        {
        //            FollowIndex = 0;
        //        }
        //        FollowX[FollowIndex] = args.FollowX;
        //        FollowY[FollowIndex] = args.FollowY;
        //        lock(myCamera.LastFilteredFile)
        //        {
        //        myparent.SetXY(FollowX[FollowIndex], FollowY[FollowIndex], myCamera.LastFilteredFile);
        //        }
        //    }
        //    catch (Exception ex)
        //    {
        //        System.Windows.Forms.MessageBox.Show("Error " + ex.ToString());

        //    }
           
        //}

        //void CamNavUpdateHandler(object oSender, CamNavUpdateArgs args)
        //{
        //    try
        //    {
        //        // FollowIndex needs to point to the current value.
        //        // therefore we increment it each time.
        //        NavIndex++;
        //        if (NavIndex > 2)
        //        {
        //            NavIndex = 0;
        //        }
        //        NavArgs[NavIndex] = args;
        //        lock (myCamera.LastFilteredFile)
        //        {
        //            myparent.SetXY((int)args.LeftRadius, (int)args.RightRadius, myCamera.LastFilteredFile);
        //        }
        //    }
        //    catch (Exception ex)
        //    {
        //        System.Windows.Forms.MessageBox.Show("Error " + ex.ToString());

        //    }

        //}
    
        #endregion

        #region setup colors
        public System.Drawing.Color setFollowColor()
        {
            System.Drawing.Color TempColor;
            // if camera is operating
            if (myCamera.UseTest == 0)
            {
                //Snap an image 
                FollowColor.Image = myCamera.button1_Click();
                // set the matchcolor
                FollowColor.SetMatchColor();
                TempColor = FollowColor.MatchColor;
            }
            else
            {
                TempColor = new System.Drawing.Color();
            }
            return TempColor;
            // if camera is not operating the color will be from test data
        }

        public System.Drawing.Color setNavigateColor()
        {
            System.Drawing.Color TempColor;
            // if camera is operating
            if (myCamera.UseTest == 0)
            {
                //Snap an image 
                ImageProcess.Image = myCamera.button1_Click();
                // set the matchcolor
                ImageProcess.SetMatchColor();
                TempColor = ImageProcess.MatchColor;
            }
            else
            {
                TempColor = new System.Drawing.Color() ;
            }
            // if camera is not operating the color will be from test data
            return TempColor;
        }
        public void SetVideoCapture()
        {
            if (CaptureVideo == "No")
            {
                CaptureVideo = "YES";
            }
            else
            {
                CaptureVideo = "No";
            }
        }
        public void SetMaxPWM(Int16 NewMaxLeft,Int16 NewMaxRight )
        {
            if (NewMaxLeft < 1001)
            {
                MaxLeft = NewMaxLeft;
            }
            else
            {
                MaxLeft = 1000;
            }
            if (NewMaxRight < 1001)
            {
                MaxRight = NewMaxRight;
            }
            else
            {
                MaxRight = 1000;
            }
                        
        }
        public void SetImageProcessor(int pProcessorID)
        {
            if (mMode== RobotModeType.Auto )
            {
                mMode = RobotModeType.WaitState; 
                Thread.Sleep(1000);
            }
            if (ImageProcess != null)
            {
                ImageProcess.LogReceived  -= this.LogEntryReceivedCallBack;
                ImageProcess.VoteReceived -= this.ImageVoteReceivedCallBack;
                ImageProcess = null;
            }
            switch (pProcessorID)
            {
                case 1:
                    ImageProcess = new master.NavBlue();
                    break;
                case 2:
                    ImageProcess = new master.NavChroma();
                    break;
                case 3:
                    ImageProcess = new master.NavColor();
                    break;
                case 4:
                    ImageProcess = new master.NavColor2();
                    break;
                case 5:
                    ImageProcess = new master.NavChroma2();
                    break;
                case 6:
                    ImageProcess = new master.NavGrass();
                    break;
                case 97:
                    ImageProcess = new master.NavSTOnly();
                    break;

                case 98:
                    ImageProcess = new master.NavOnly();
                    break;

                case 99:
                    ImageProcess = new master.FollowColor();
                    break;

                default:
                    break;
            }
            ImageProcess.LogReceived += new BaseImgProcess.CameraLogHandler(this.LogEntryReceivedCallBack);
            ImageProcess.VoteReceived += new BaseImgProcess.VoteHandler(this.ImageVoteReceivedCallBack);
            ImageProcess.Logit("Started " + ImageProcess.ToString());
            

        }
        public void SetThreshold(int pThreshold)
        {
         //   myCamera.threshold = pThreshold;
        }
        public void QueryPort(int port)
        {
            string command;


            command = "F " + port.ToString() ;
            com.SendCommand(command);

        }
        public void CalibrateCompass()
        {
            CommandSent ThisCommand = new CommandSent(MessageID, RobotCommandTypes.CalibrateCompass );
            //byte[] 
            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.CalibrateCompass).ToString() + " " + ThisCommand.CommandID.ToString() ;
            CommandWaitingList.Add(ThisCommand);
            com.SendCommand(ThisCommand.Commandstring);
            ThisCommand.RecordIt(log);
            MessageID++;

        }

        public void SetGainRange(int Gain, int Range)
        {
            // g ID range gain
            //checked  range < 255 and gain < 32
            if (Gain > -1 && Gain < 32 && Range > -1 && Range < 255)
                {
                CommandSent ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Gain);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Gain).ToString() + " " + ThisCommand.CommandID.ToString()
                + " " + Range.ToString() + " " + Gain.ToString();

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;
                }
        
        }

        public string SnapIt()
        {
            histographValues results = new histographValues();

            ImageProcess.Image = myCamera.button1_Click();
            string ImageFile = "C:\\users\\Dad\\My Documents\\HI" + DateTime.Now.ToString("yyyyMMddHHmmss") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
            System.Drawing.Bitmap histo = this.ImageProcess.CreateHistograph(ref results);
            histo.Save(ImageFile);
            histo = null;
            lock (LastFiltered)
            {
                LastFiltered = this.InputDirName + "\\NI" + DateTime.Now.ToString("HH_mm_ss_") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
                //NavColor.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
                ImageProcess.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
                myparent.SetImage(LastFiltered, ImageVote.ToString());
            }
            return ImageFile; // this.myCamera.Snapit();

        }
        #endregion

        #region main processor loops

        void MainThreadLoop()
        {


            TimeSpan ProcessingTime;
            DateTime StartTime;
           // int LoopCount = 0;
            // 
            while (MainThreadKeepAlive == "Continue")
            {
                try
                {
                    StartTime = DateTime.Now;
                    if (SlaveStarted)
                    {
                        // perform some action
                        if (mMode == RobotModeType.RemoteControl)
                        {
                            MoveRemoteControl();
                        }
                        if (mMode == RobotModeType.WaitState)
                        {
                            Wait();
                        }
                        if (CurrentSensor > -1)
                        {
                            if (mMode == RobotModeType.ColorFollow)
                            {
                                MoveFollowColor();
                            }
                            if (mMode == RobotModeType.FindColor)
                            {
                                MoveFindColor();
                            }
                            if (mMode == RobotModeType.Auto)
                            {
                                if (ImageProcess is master.NavOnly)
                                {
                                    MoveAutoNav3();

                                }
                                else if (ImageProcess is master.NavSTOnly)
                                {
                                    MoveAutoNav4();
                                }
                                else
                                {
                                    MoveAutoNav2();
                                }
                            }
                            if (mMode == RobotModeType.AvoidObstacle)
                            {
                                MoveAvoidObstacle();
                            }
                            if (mMode == RobotModeType.WaitForPerson)
                            {
                                WaitForPerson();
                            }
                        }
                        
                    }
                    else
                    {
                        // slave not started 
                        if (mMode != RobotModeType.LookForSlave && mMode != RobotModeType.WaitState)
                        {
                           lock (logArg)
                            {
                                logArg.LogEntry = "Slave not started by mode is  " + mMode.ToString() + " MLthread " + System.Threading.Thread.CurrentThread.Name + "\n";
                                LogEntryReceivedCallBack(this, logArg);
                            }
                        }
                        else
                        {
                            //handle looking for slave here

                        }
                    }
                    ProcessingTime = DateTime.Now - StartTime;
                    // go to sleep until sensor data returns
                    if (ProcessingTime.Milliseconds > 99)
                    {
                        lock (logArg)
                        {
                            logArg.LogEntry = "MTL PTime " + ProcessingTime.Milliseconds.ToString() + " MLthread " + System.Threading.Thread.CurrentThread.Name + "\n";
                            LogEntryReceivedCallBack(this, logArg);
                        }
                        Thread.Sleep(1);
                    }
                    else
                    {
                        Thread.Sleep(100 - ProcessingTime.Milliseconds);

                    }
                }
                catch (Exception ex)
                {
                    System.Windows.Forms.MessageBox.Show("MainThreadLoop()\r\n" + ex.ToString(), "Robot", System.Windows.Forms.MessageBoxButtons.OK, System.Windows.Forms.MessageBoxIcon.Stop);

                }

                }
        }

        void ImageThreadLoop()
        {

           TimeSpan ProcessingTime;
           DateTime StartTime;

           VoteObject n;
           //BaseImgProcess ImageProcess;
 
           lock (logArg)
           {
                DateTime dt = DateTime.Now;
                logArg.LogEntry = "Starting ImageThreadLoop " + dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " \n";
                LogEntryReceivedCallBack(this, logArg);
           }

          
            
            while (ImageThreadKeepAlive=="Continue") 
            {
                StartTime = DateTime.Now;

                if (Mode != RobotModeType.WaitState && Mode != RobotModeType.RemoteControl && Mode != RobotModeType.LookForSlave)
                {
                    try
                    {
                        if (ImageProcess is master.NavOnly || ImageProcess is master.NavSTOnly)
                        {
                            ;
                        }
                        else
                        {
                            ImageProcess.Image = myCamera.button1_Click();

                            n = ImageProcess.ProccessFrame();

                            lock (this.ImageVote)
                            {
                                this.ImageVote = n;
                            }
                            lock (LastFiltered)
                            {
                                LastFiltered = this.InputDirName + "\\FI" + DateTime.Now.ToString("HH_mm_ss_") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
                                ImageProcess.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
                                myparent.SetImage(LastFiltered, ImageVote.ToString());
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        System.Windows.Forms.MessageBox.Show("ImageThreadLoop()\r\n" + ex.ToString(), "Robot", System.Windows.Forms.MessageBoxButtons.OK, System.Windows.Forms.MessageBoxIcon.Stop);
                        lock (logArg)
                        {
                            DateTime dt = DateTime.Now;
                            logArg.LogEntry = "Error Robot ProcessNavLoop() " + ex.ToString() + dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " \n";
                            LogEntryReceivedCallBack(this, logArg);
                        }

                    }


                }       


                ProcessingTime = DateTime.Now - StartTime;
                // go to sleep until sensor data returns
                if (ProcessingTime.Milliseconds > 199)
                {
                    lock (logArg)
                    {
                        logArg.LogEntry = "ITL PTime " + ProcessingTime.Milliseconds.ToString() + "\n";
                        LogEntryReceivedCallBack(this, logArg);
                    }
                    Thread.Sleep(1);
                }
                else
                {
                    Thread.Sleep(200 - ProcessingTime.Milliseconds);

                }

            }
 
            lock (logArg)
            {
                logArg.LogEntry = "ITL Exit \n";
                LogEntryReceivedCallBack(this, logArg);
            }

        
        }



        void MoveRemoteControl()
        {
            TimeSpan ProcessingTime;
            String timings;
            DateTime StartTime;
            StartTime = DateTime.Now;

            CommandSent ThisCommand;
            if (this.RemoteDirection != this.Direction)
            {
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;

            }
            if (this.mRemoteLeft != LeftPWM || mRemoteRight != RightPWM)
            {
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + RemoteLeft.ToString() + " " + RemoteRight.ToString();

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;
            }        
            if (CaptureVideo == "YES")
            {
                myCamera.button1_Click();
                myparent.SetImage(myCamera.RawFile, ImageVote.ToString());
                
            }
            

            ProcessingTime = DateTime.Now - StartTime;
            timings = "Remote " + ProcessingTime.Milliseconds.ToString() + " " + StartTime.ToString("s") + ":" + StartTime.Millisecond.ToString("000") + "\n";
            lock (logArg)
            {
                logArg.LogEntry = timings;
                LogEntryReceivedCallBack(this, logArg);
            }

        }
        void MoveFollowColor()
        {
            CommandSent ThisCommand;
            VoteObject n; 
            lock (ImageVote)
            {
                n = ImageVote;
            }
            VoteObject s = sonarVote.Vote(sensors[CurrentSensor].Left, sensors[CurrentSensor].Front, sensors[CurrentSensor].Right);

            s.WeightFactor = .6;
            n.WeightFactor = .4;
            s.Factor();
            s.Accumulate(n);
            string TheVote = s.Vote();

            if (TheVote == "SL" || TheVote == "SR")
            {
                char[] mychar = TheVote.ToCharArray(1, 1);
                if (Direction != mychar[0])
                {
                    RemoteDirection = mychar[0];
                    // stop
                    mRemoteLeft = 0;
                    mRemoteRight = 0;
                    ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                    //byte[] 
                    ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                    // wait till we stop
                    Thread.Sleep(2000);

                    // change direction
                    ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                    //byte[] 
                    ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                }
                // otherwise we are going in the correct direction
                mRemoteLeft = MaxLeft;
                mRemoteRight = MaxRight;
            }
            else
            {
                if (Direction != 'F')
                    {
                    RemoteDirection = 'F';
                    // stop
                    mRemoteLeft = 0;
                    mRemoteRight = 0;
                    ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                    //byte[] 
                    ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                    // wait till we stop
                    Thread.Sleep(2000);

                    // change direction
                    ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                    //byte[] 
                    ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                    }
                if (TheVote == "HL")
                {
                    mRemoteLeft = 0;
                    mRemoteRight = MaxRight;
                }
                else
                {
                    if (TheVote == "L")
                    {
                        mRemoteLeft = 200;
                        mRemoteRight = MaxRight;
                    }
                    else
                    {
                        if (TheVote == "ST")
                        {
                            mRemoteLeft = MaxLeft;
                            mRemoteRight = MaxRight;
                        }
                        else
                        {
                            if (TheVote == "R")
                            {
                                mRemoteLeft = MaxLeft;
                                mRemoteRight = 200;
                            }
                            else
                            {
                                if (TheVote == "HR")
                                {
                                    mRemoteLeft = MaxLeft;
                                    mRemoteRight = 0;
                                }

                            }
                        }
                    }
                }
            }

            if (LeftPWM != mRemoteLeft || RightPWM != mRemoteRight)
            {
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;
            }

        }

        void MoveFindColor()
        {
            //put code To circle and find color here

        }
        void MoveAutoNav2()
        {
            CommandSent ThisCommand;
            String timings;
            int Index = NavIndex; // grab this locally so we do not have to worry about it changing
            TimeSpan ProcessingTime;
            DateTime StartTime;
            StartTime = DateTime.Now;
            VoteObject n,s;
            Double value;

            lock (this.ImageVote)
            {
                n = this.ImageVote;
            }


            while (CurrentSensor < 0)
            {
                Thread.Sleep(1);
            }
            //lock (this.ImuVote)
            //{
            //    i = this.ImuVote.Vote(this.Imu, );
            //}


            s = this.sonarVote.Vote(sensors, CurrentSensor);
            value = n.MaxValue();
            //if 
            //n.WeightFactor = .5;
            //i.WeightFactor = .3;
            //s.WeightFactor = .3;
            //n.Factor();
            //n.Accumulate(i);
            if (value > 3 )
                {
                    s.WeightFactor = 3;
                }
            else 
            {
                if (value > 2 )
                    {
                s.WeightFactor  = 2;
                    }
                else
                {
                s.WeightFactor  = 1;
                }
            }
            n.Accumulate(s); 
            string TheVote = n.Vote();
            // need ToString check ToString see that voted optin is > 0
            // if SL and its value is 0 then usually they are all 0 default it to straight
            if (TheVote == "SL")
            {
                if (n.SpinLeft==0)
                {
                    TheVote = "ST";
                }
            }
                 
            if (sensors[CurrentSensor].Front > 3600)
                {

//----------------
                    if (TheVote == "SL" || TheVote == "SR")
                        {
                        char[] mychar = TheVote.ToCharArray(1, 1);
                        if (Direction != mychar[0])
                        {
                            RemoteDirection = mychar[0];
                            // stop
                            mRemoteLeft = 0;
                            mRemoteRight = 0;
                            ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                            //byte[] 
                            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                            CommandWaitingList.Add(ThisCommand);
                            com.SendCommand(ThisCommand.Commandstring);
                            ThisCommand.RecordIt(log);
                            MessageID++;
                            // wait till we stop
                           // Thread.Sleep(2000);

                            // change direction
                            ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                            //byte[] 
                            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                            CommandWaitingList.Add(ThisCommand);
                            com.SendCommand(ThisCommand.Commandstring);
                            ThisCommand.RecordIt(log);
                            MessageID++;
                        }
                        // otherwise we are going in the correct direction
                        mRemoteLeft = MaxLeft;
                        mRemoteRight = MaxRight;
                        }
                    else
                        {



                        if (Direction != 'F')
                        {
                            RemoteDirection = 'F';
                            // stop
                            mRemoteLeft = 0;
                            mRemoteRight = 0;
                            ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                            //byte[] 
                            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                            CommandWaitingList.Add(ThisCommand);
                            com.SendCommand(ThisCommand.Commandstring);
                            ThisCommand.RecordIt(log);
                            MessageID++;
                            // wait till we stop
                            //Thread.Sleep(2000);

                            // change direction
                            ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                            //byte[] 
                            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                            CommandWaitingList.Add(ThisCommand);
                            com.SendCommand(ThisCommand.Commandstring);
                            ThisCommand.RecordIt(log);
                            MessageID++;
                        }
                        if (TheVote == "HL")
                        {
                            mRemoteLeft = 0;
                            mRemoteRight = MaxRight;
                        }
                        else
                        {
                            if (TheVote == "L")
                            {
                                mRemoteLeft = MinLeft;
                                mRemoteRight = MaxRight;
                            }
                            else
                            {
                                if (TheVote == "ST")
                                {
                                    mRemoteLeft = MaxLeft;
                                    mRemoteRight = MaxRight;
                                }
                                else
                                {
                                    if (TheVote == "R")
                                    {
                                        mRemoteLeft = MaxLeft;
                                        mRemoteRight = MinRight;
                                    }
                                    else
                                    {
                                        if (TheVote == "HR")
                                        {
                                            mRemoteLeft = MaxLeft;
                                            mRemoteRight = 0;
                                        }

                                    }
                                }
                            }
                        }
                    }

                    
                   // ---------
                    
                }
            else
            {
                mRemoteLeft = 0;
                mRemoteRight = 0;
                WaitForPerson();
            }

            if (LeftPWM != mRemoteLeft || RightPWM != mRemoteRight)
            {
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;
            }
            ProcessingTime = DateTime.Now - StartTime;
            timings = " MAN2 " + ProcessingTime.Milliseconds.ToString() + " " + StartTime.ToString("s") + ":" + StartTime.Millisecond.ToString("000") + n.ToString() + " thevote" + TheVote + "\n";
            lock (logArg)
            {
                logArg.LogEntry = timings;
                LogEntryReceivedCallBack(this, logArg);
            }




        }
        void MoveAutoNav3()
        {
            CommandSent ThisCommand;
            String timings;
            TimeSpan ProcessingTime;
            DateTime StartTime;
            StartTime = DateTime.Now;
            Boolean Stop = false;


            VoteObject b;
    //        VoteObject s;
            double Diff = BearingDifferenceXY(gnBearing, gnTargetBearing);
            b = bearingVote.VoteXY(Diff);
            b.WeightFactor = 1;
            b.Factor();
            while (CurrentSensor < 0)
            {
                Thread.Sleep(1);
            }
            string TheVote = b.Vote();
            //if (sensors[CurrentSensor].Front < 3600)
            //{
            //    Stop = true;
            //}
  //          s = this.sonarVote.Vote(sensors, CurrentSensor);

  //          s.WeightFactor = 0.5;
  //          b.Accumulate(s);
            Boolean SendCommand = motorControl.SetMotor(b, LeftSpeed, RightSpeed, Direction, Stop);
            if (motorControl.NeedDirection)
            {
                ThisCommand = motorControl.MotorCommand(MessageID);
                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;

            }
            else
            {
                if (SendCommand)
                {
                    ThisCommand = motorControl.MotorCommand(MessageID);
                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                }
            }
            if (Stop)
            {
                WaitForPerson();
            }
            ProcessingTime = DateTime.Now - StartTime;
            timings = " MAN3 " + ProcessingTime.Milliseconds.ToString() + " " + StartTime.ToString("s") + ":" + StartTime.Millisecond.ToString("000") + b.ToString() + " thevote " + TheVote + "\n";
            lock (logArg)
            {
                logArg.LogEntry = timings;
                LogEntryReceivedCallBack(this, logArg);
            }

        }
        void MoveAutoNav4()
        {
            CommandSent ThisCommand;
            String timings;
            TimeSpan ProcessingTime;
            DateTime StartTime;
            StartTime = DateTime.Now;
            Boolean Stop = false;
            //hardcode vote to straight

            VoteObject b = new VoteObject();
            b.HardLeft = b.HardRight = b.SoftLeft = b.SoftRight = b.SpinLeft = b.SpinRight = 0;
            b.Straight = 1;

            b.WeightFactor = 1;
            b.Factor();
            while (CurrentSensor < 0)
            {
                Thread.Sleep(1);
            }
            string TheVote = b.Vote();
            Boolean SendCommand = motorControl.SetMotor(b, LeftSpeed, RightSpeed, Direction, Stop);
            if (motorControl.NeedDirection)
            {
                ThisCommand = motorControl.MotorCommand(MessageID);
                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;

            }
            else
            {
                if (SendCommand)
                {
                    ThisCommand = motorControl.MotorCommand(MessageID);
                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                }
            }
            if (Stop)
            {
                WaitForPerson();
            }
            ProcessingTime = DateTime.Now - StartTime;
            timings = " MAN4 " + ProcessingTime.Milliseconds.ToString() + " " + StartTime.ToString("s") + ":" + StartTime.Millisecond.ToString("000") + b.ToString() + " thevote " + TheVote + "\n";
            lock (logArg)
            {
                logArg.LogEntry = timings;
                LogEntryReceivedCallBack(this, logArg);
            }

        }
        void MoveAutoNav()
        {
            CommandSent ThisCommand;
            String timings;  
            int Index = NavIndex; // grab this locally so we do not have to worry about it changing
            TimeSpan ProcessingTime;
            DateTime StartTime;
            StartTime = DateTime.Now;
            //VoteObject n = ImageProcess.ProccessFrame();
            VoteObject n;
            VoteObject i;
            // Imagevote set in ProcessNavLoop
            lock (this.ImageVote)
            {
                n = this.ImageVote; 
            }

            n.WeightFactor = .3;
            double Diff = BearingDifference(Bearing, gnTargetBearing);
            i = this.ImuVote.Vote(ImuValues, 504 - (int)Diff);
            i.WeightFactor  = .3;
            // wait until we have sonar and compass data
            while (CurrentSensor < 0)
            {
                Thread.Sleep(1);
            }
            //VoteObject s = sonarVote.Vote(sensors[CurrentSensor].Left, sensors[CurrentSensor].Front, sensors[CurrentSensor].Right);
            VoteObject s = sonarVote.Vote(sensors,CurrentSensor);   
            
            s.WeightFactor = .5;
            VoteObject b = bearingVote.VoteXY(Diff);
            b.WeightFactor = .3;

            s.Factor();
            s.Accumulate(n);
            s.Accumulate(b);
            s.Accumulate(i);

            string TheVote = s.Vote();
            DateTime dt = DateTime.Now;

            lock (logArg)
            {
                logArg.LogEntry = "Acc " + dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + s.ToString() + "\n"; 
                LogEntryReceivedCallBack(this, logArg);
            }

            ProcessingTime = DateTime.Now - StartTime;
            timings = " MAN1a " + ProcessingTime.Milliseconds.ToString() + " " + StartTime.ToString("s") + ":" + StartTime.Millisecond.ToString("000") + "\n";

            if (sensors[CurrentSensor].Front > 3600)
            {

                if (TheVote == "SL" || TheVote == "SR")
                {
                    char[] mychar = TheVote.ToCharArray(1, 1);
                    if (Direction != mychar[0])
                    {
                        RemoteDirection = mychar[0];
                        // stop
                        mRemoteLeft = 0;
                        mRemoteRight = 0;
                        ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                        //byte[] 
                        ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                        CommandWaitingList.Add(ThisCommand);
                        com.SendCommand(ThisCommand.Commandstring);
                        ThisCommand.RecordIt(log);
                        MessageID++;
                        // wait till we stop
                        Thread.Sleep(2000);

                        // change direction
                        ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                        //byte[] 
                        ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                        CommandWaitingList.Add(ThisCommand);
                        com.SendCommand(ThisCommand.Commandstring);
                        ThisCommand.RecordIt(log);
                        MessageID++;
                    }
                    // otherwise we are going in the correct direction
                    mRemoteLeft = MaxLeft;
                    mRemoteRight = MaxRight;
                }
                else
                {
                    if (Direction != 'F')
                    {
                        RemoteDirection = 'F';
                        // stop
                        mRemoteLeft = 0;
                        mRemoteRight = 0;
                        ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                        //byte[] 
                        ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                        CommandWaitingList.Add(ThisCommand);
                        com.SendCommand(ThisCommand.Commandstring);
                        ThisCommand.RecordIt(log);
                        MessageID++;
                        // wait till we stop
                        //Thread.Sleep(2000);

                        // change direction
                        ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                        //byte[] 
                        ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " " + this.RemoteDirection;

                        CommandWaitingList.Add(ThisCommand);
                        com.SendCommand(ThisCommand.Commandstring);
                        ThisCommand.RecordIt(log);
                        MessageID++;
                    }
                    if (TheVote == "HL")
                    {
                        mRemoteLeft = 0;
                        mRemoteRight = MaxRight;
                    }
                    else
                    {
                        if (TheVote == "L")
                        {
                            mRemoteLeft = 200;
                            mRemoteRight = MaxRight;
                        }
                        else
                        {
                            if (TheVote == "ST")
                            {
                                mRemoteLeft = MaxLeft;
                                mRemoteRight = MaxRight;
                            }
                            else
                            {
                                if (TheVote == "R")
                                {
                                    mRemoteLeft = MaxLeft;
                                    mRemoteRight = 200;
                                }
                                else
                                {
                                    if (TheVote == "HR")
                                    {
                                        mRemoteLeft = MaxLeft;
                                        mRemoteRight = 0;
                                    }

                                }
                            }
                        }
                    }
                }

            }
            else
            {
                mRemoteLeft = 0;
                mRemoteRight = 0;
                WaitForPerson();
            }
            if (LeftPWM != mRemoteLeft || RightPWM != mRemoteRight)
            {
                ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                //byte[] 
                ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                CommandWaitingList.Add(ThisCommand);
                com.SendCommand(ThisCommand.Commandstring);
                ThisCommand.RecordIt(log);
                MessageID++;
            }
            ProcessingTime = DateTime.Now - StartTime;
            timings += " MAN1b " + ProcessingTime.Milliseconds.ToString() + " " + StartTime.ToString("s") + ":" + StartTime.Millisecond.ToString("000") +"\n";
            lock (logArg)
            {
                logArg.LogEntry = timings;
                LogEntryReceivedCallBack(this, logArg);
            }




        }
// Mapping routine called after each output update

// bearing difference returns -180 to 180. 
// negative means turn to right
// positive mean turn left
        double BearingDifference(double pnCurrentBearing, double pntargetBearing)
        {
            double lnDifference;
         //   trueBearing = pnCurrentBearing -12;
            lnDifference = pnCurrentBearing - pntargetBearing;
            if (lnDifference < 0)
            {
                if (lnDifference < -180)
                {
                    lnDifference = ((pnCurrentBearing + 360) - pntargetBearing);
                }
            }
            else
            {
                if (lnDifference > 180)
                {
                    lnDifference = (pnCurrentBearing - (pntargetBearing + 360));
                }
            }
            return lnDifference;

        }

        double BearingDifferenceXY(double pnCurrentBearing, double pntargetBearing)
        {
            double lnDifference;
            lnDifference = pnCurrentBearing - pntargetBearing;
            return lnDifference;

        }
        public const double FEETPERTICK = .0382;
        // wheel base = 31 in 
        public const double WHEELBASE = 2.583; // in feet

        public const double TWOPI = 6.2831853070;
        public const double RADS = 57.2958;			// radians to degrees conversion 

        void Map()
        {
        double leftspeed = LeftSpeed/10.0;
        double rightspeed = RightSpeed/10.0;
        double angular;
        //Double bearing = Bearing;

           // mCurrentX += 0;
           // mCurrentY += 0;
        /*
        if (sensors[CurrentSensor].Direction == 'L')
        {
            leftspeed *= -1;
        }
        else
        {
            if (sensors[CurrentSensor].Direction == 'R')
            {
                rightspeed *= -1;
            }
            else
            {
                if (sensors[CurrentSensor].Direction == 'B')
                {
                    rightspeed *= -1;
                    leftspeed *= -1;
                }
            }
        }
        */
        double Distance = (double)(leftspeed + rightspeed) / 2;
        //myDistance -= Distance;
        angular = (rightspeed - leftspeed) / WHEELBASE;
        double deltaX, deltaY;
        // from ros
        if (Math.Abs(angular) < 0.000001)
            {
            double direction = gnBearing + (angular * .5);
            deltaX = Distance * Math.Cos(direction);
            deltaY = Distance * Math.Sin(direction);
            gnCurrentX += deltaX;
            gnCurrentY += deltaY;
            gnBearing += angular; 
            }
        else
            {
            double oldth = gnBearing;
            double r = Distance / angular;
            gnBearing += angular;
            deltaX = r * (Math.Sin(gnBearing) - Math.Sin(oldth));
            deltaY = -r * (Math.Cos(gnBearing) - Math.Cos(oldth));
            gnCurrentX += deltaX;
            gnCurrentY += deltaY;
            }

        //ConvertToRadians(Bearing); 
        /*
        Trig stuff
        right triangle Sides ABC where c= hypot a = adjacent b = opposite
        theta = angle from side a to side c
        sin of theta = B/C
        cos od theta = a/C
        tan of theta = b/a
        value of theta = atan(b/a)
        however atan does not give you the correct value depending on the quadrant of the y/x graph

          II  |  I
        ------|-----
         III  | IV
 
        Quad 1 result is angle measured from X axis (for bearing need to subtract from 90 to get bearing from north)
        Quad 2 angle needs Plus 180 ??
        Quad 3  angle needs Plus 180 ????
        Quad 4 angle plus 90 ???

        hyperphysics.phys-astr.gsu.edu/hbase/ttrig.html

        */
        // according to trig Y is opposite X is adjecent  if
        // if so then X uses cos and Y uses sin 

        // gnCurrentY += (Distance * (sin(gnOdomBearing))); 
        // gnCurrentX += (Distance * (cos(gnOdomBearing)));

        // I decided I need to swap sin and cos because it seems to fit
        // I ran test 0,90,180,270,360 and it seemed Y and X 
        //gnCurrentY += (Distance * (Math.Cos(lBearing)));
        //gnCurrentX += (Distance * (Math.Sin(lBearing)));


            DateTime dt = DateTime.Now;

            lock (logArg)
            {
                logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000")
                + " Map X " + gnCurrentX.ToString("0.000") + " Y " + gnCurrentY.ToString("0.000")
                + " gnBearing " + gnBearing.ToString("0.000") + " Distance " + Distance.ToString("0.00") 
                + " leftspeed " + leftspeed.ToString("0.000") + " rightspeed " + rightspeed.ToString("0.000") 
                + " angular " + angular.ToString("0.000")
                + " deltaX " + deltaX.ToString("0.000") + " deltaY " + deltaX.ToString("0.000")
                + "\n";
                LogEntryReceivedCallBack(this, logArg);
            }

         CalcHeadingDistanceXY();

    }
    void MoveAvoidObstacle()
    {
        CommandSent ThisCommand;
        ushort FrontDistance = sensors[CurrentSensor].Front;
        ushort LeftDistance = sensors[CurrentSensor].Left;
        ushort RightDistance = sensors[CurrentSensor].Right;
        if (FrontDistance > 4000)
        {
            // done way is clear ahead 

            // stop
            mRemoteLeft = 0;
            mRemoteRight = 0;
            ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
            //byte[] 
            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

            CommandWaitingList.Add(ThisCommand);
            com.SendCommand(ThisCommand.Commandstring);
            ThisCommand.RecordIt(log);
            MessageID++;

            ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
            ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " F";
            CommandWaitingList.Add(ThisCommand);
            com.SendCommand(ThisCommand.Commandstring);
            ThisCommand.RecordIt(log);
            MessageID++;

            DateTime dt = DateTime.Now;
            lock (logArg)
            {
                logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " F=" + FrontDistance.ToString() + " Exit MoveAvoidObstacle Returning to Mode " + mPriorMode.ToString() + "\n";
                LogEntryReceivedCallBack(this, logArg);
            }


            //Go back To prior mode

            Mode = mPriorMode; 
        }
        else
        {
            if (mRemoteLeft == 0 && mRemoteRight == 0)
            {
                DateTime dt  = DateTime.Now;
                lock (logArg)
                {
                    logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " Enter MoveAvoidObstacle \n";
                    LogEntryReceivedCallBack(this, logArg);
                }

                if (LeftPWM > 0 || RightPWM > 0)
                {
                    ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                    ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();
                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                    // wait 1 second to stop
                    Thread.Sleep(1000);
                }
                else
                {
                    if (LeftDistance > RightDistance)
                    {
                        ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                        ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " L";

                    }
                    else
                    {
                        ThisCommand = new CommandSent(MessageID, RobotCommandTypes.Direction);
                        ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.Direction).ToString() + " " + ThisCommand.CommandID.ToString() + " R";

                    }
                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;

                    mRemoteLeft = MaxLeft;
                    mRemoteRight = MaxRight;
                    ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                    ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();
                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                    // give it a 7 seconds to get moving
                    Thread.Sleep(7500);
                }
            }
        }
    }
    void Wait()
    {
        Thread.Sleep(1);
    }
    void WaitForPerson()
    {
        try
        {
            if (mMode != RobotModeType.WaitForPerson)
            {
                mPriorMode = mMode;
                mMode = RobotModeType.WaitForPerson;
 
                // we got here because we are less than 3000us to an obstacle
                // stop if we need to
                if (mRemoteLeft != 0 || mRemoteRight != 0 || LeftPWM != 0 || RightPWM != 0)
                {
                    mRemoteLeft = 0;
                    mRemoteRight = 0;
                    CommandSent ThisCommand = new CommandSent(MessageID, RobotCommandTypes.MotorControl);
                    //byte[] 
                    ThisCommand.Commandstring = System.Convert.ToChar(RobotCommandTypes.MotorControl).ToString() + " " + ThisCommand.CommandID.ToString() + " " + mRemoteLeft.ToString() + " " + mRemoteRight.ToString();

                    CommandWaitingList.Add(ThisCommand);
                    com.SendCommand(ThisCommand.Commandstring);
                    ThisCommand.RecordIt(log);
                    MessageID++;
                }
                // play the warning
                DateTime dt = DateTime.Now;
                System.Media.SoundPlayer sndPing = new System.Media.SoundPlayer("C:\\Documents and Settings\\Admin\\My Documents\\master\\Danger.wav");
                sndPing.Play();
                Thread.Sleep(10);
                PersonDelayWarning = 0;

                lock (logArg)
                {
                    logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " WaitForPerson Play Danger.wav" + mPriorMode.ToString() + "\n";
                    LogEntryReceivedCallBack(this, logArg);
                }
                PersonDelay = 60;
            }
            if (PersonDelay > 0)
            {
                PersonDelay--;
                DateTime dt = DateTime.Now;
                lock (logArg)
                {
                    logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " WaitForPerson dELAY" + PersonDelay.ToString() + "\n";
                    LogEntryReceivedCallBack(this, logArg);
                }

            }
            else
            {
                string mywarning = "C:\\Documents and Settings\\Admin\\My Documents\\master\\";
                if (PersonDelayWarning < 3 || PersonDelayWarning == 4 || PersonDelayWarning == 5)
                {
                    mywarning += "Danger.wav";
                }
                else
                {
                    if (PersonDelayWarning == 3)
                    {
                        mywarning += "PleaseMove.wav";
                    }
                    else
                    {
                        if (PersonDelayWarning == 6)
                        {
                            mywarning += "NotPerson.wav";
                        }
                        else
                        {
                            mywarning += "Danger.wav";
                        }
                    }
                }
                System.Media.SoundPlayer sndPing = new System.Media.SoundPlayer(mywarning);
                sndPing.Play();
                DateTime dt = DateTime.Now;

                lock (logArg)
                {
                    logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " WaitForPerson Play " + mywarning.ToString() + " PersonDelayWarning " + PersonDelayWarning.ToString() + "\n";
                    LogEntryReceivedCallBack(this, logArg);
                }
                PersonDelayWarning++;
                PersonDelay = 60;

            }
            if (PersonDelayWarning >= 8)
            {
                DateTime dt = DateTime.Now;

                lock (logArg)
                {
                    logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " WaitForPerson Set move to avoid \n";
                    LogEntryReceivedCallBack(this, logArg);
                }

                PersonDelayWarning = 0;
                Mode = RobotModeType.AvoidObstacle;
                mPriorMode = RobotModeType.Auto; 
            }
            if ( sensors[CurrentSensor].Front > 3600)
            {
                Mode = mPriorMode;
            }
        }
        catch (Exception ex)
        {
            DateTime dt = DateTime.Now;
            lock (logArg)
            {
                logArg.LogEntry = dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " WaitForPerson Error " + ex.ToString() + "\n";
                LogEntryReceivedCallBack(this, logArg);
            }


        }
    }

    public void CameraSettings(IntPtr winHandle)
        {

            // myCamera.SetCamera(winHandle);
        }
    public void DisplayCameraProperties(IntPtr winHandle)
        {
            myCamera.DisplayCameraProperties(winHandle);
        }
        public void CalibrateCamera()
        {
        }
    //    private static Camera.CameraSettings[] settings = new Camera.CameraSettings[17];
    //public void CalibrateCamera()
    //    {
    //    settings[0]=   new Camera.CameraSettings( 1);
    //    settings[0].PropertyType = 1;

    //    settings[1] = new Camera.CameraSettings(1); 
    //    settings[1].PropertyType = 2;
    //    settings[2] = new Camera.CameraSettings(1); 
    //    settings[2].PropertyType = 3;
    //    settings[3] = new Camera.CameraSettings(1); 
    //    settings[3].PropertyType = 4;
    //    settings[4] = new Camera.CameraSettings(1); 
    //    settings[4].PropertyType = 5;
    //    settings[5] = new Camera.CameraSettings(1); 
    //    settings[5].PropertyType = 6;
    //    settings[6] = new Camera.CameraSettings(1); 
    //    settings[6].PropertyType = 7;
    //    settings[7] = new Camera.CameraSettings(1); 
    //    settings[7].PropertyType = 8;
    //    settings[8] = new Camera.CameraSettings(1); 
    //    settings[8].PropertyType = 9;
    //    settings[9] = new Camera.CameraSettings(1); 
    //    settings[9].PropertyType = 10;
    //    settings[10] = new Camera.CameraSettings(1); 
    //    settings[10].PropertyType = 11;
    //    settings[11] = new Camera.CameraSettings(2);
    //    settings[11].PropertyType = (int)CameraControlProperty.Pan;
    //    settings[12] = new Camera.CameraSettings(2);
    //    settings[12].PropertyType = (int)CameraControlProperty.Tilt;
    //    settings[13] = new Camera.CameraSettings(2);
    //    settings[13].PropertyType = (int)CameraControlProperty.Roll;
    //    settings[14] = new Camera.CameraSettings(2);
    //    settings[14].PropertyType = (int)CameraControlProperty.Zoom;
    //    settings[15] = new Camera.CameraSettings(2);
    //    settings[15].PropertyType = (int)CameraControlProperty.Exposure;
    //    settings[16] = new Camera.CameraSettings(2);
    //    settings[16].PropertyType = (int)CameraControlProperty.Iris;

    //    lock (logArg)
    //    {
    //        DateTime dt = DateTime.Now;
    //        logArg.LogEntry = "Calibrating camera " + dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " \n";
    //        LogEntryReceivedCallBack(this, logArg);
    //    }

    //    myCamera.GetCamera(settings);
    //    // set camera to default values  auto 
    //    for (int i = 0; i < settings.Length; i++)
    //    {
    //        settings[i].Value = settings[i].Default;
    //        settings[i].Mode = (int)CameraControlFlags.Auto;  
    //    }

    //    myCamera.SetCamera(settings);
    //    // wait 10 seconds
    //    Thread.Sleep(1000);

    //    int lnExposure = this.GetExposure();
    //    int lnBrightness = this.GetBrightness();

    //    lock (logArg)
    //    {
    //        logArg.LogEntry = "Begin Calibrate Exposure = " + lnExposure.ToString() + " Brightness " + lnBrightness.ToString() + "\n";
    //        LogEntryReceivedCallBack(this, logArg);
    //    }
    //    VoteObject n;
    //    histographValues results = new histographValues();
    //    ImageProcess.Image = myCamera.button1_Click();
    //    System.Drawing.Bitmap histo = ImageProcess.CreateHistograph(ref results);
    //   // n = this.ImageProcess.ProccessFrame();
    //    lock (LastFiltered)
    //    {
    //        LastFiltered = this.InputDirName + "\\CI" + DateTime.Now.ToString("HH_mm_ss_") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
    //        //NavColor.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //        ImageProcess.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //        myparent.SetImage(LastFiltered, ImageVote.ToString());
    //        LastFiltered = this.InputDirName + "\\HI" + DateTime.Now.ToString("HH_mm_ss_") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
    //        histo.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //        myparent.Set2ndImage(LastFiltered);
    //    }
    //    Boolean bIsOk = false;
    //    int MinValue,MaxValue;
        
    //    while (bIsOk == false)
    //    {
    //        MinValue = results.MinBlue;
    //        if (results.MinGreen < MinValue)
    //        {
    //        MinValue = results.MinGreen;
    //        }
    //        if (results.MinRed  < MinValue)
    //        {
    //        MinValue = results.MinRed;
    //        }
    //        MaxValue = results.MaxBlue;
    //        if (results.MaxGreen > MaxValue)
    //        {
    //            MaxValue=results.MaxGreen;
    //        }
    //        if (results.MaxRed  > MaxValue)
    //        {
    //            MaxValue = results.MaxRed;
    //        }

    //        if (MinValue > 60)
    //        {
    //            // too bright
    //            lnExposure -= 5;
    //            this.SetExposure(lnExposure);
    //            if (lnExposure < 1)
    //            {
    //                if (lnBrightness > 1)
    //                {
    //                    lnBrightness -= 5;
    //                    this.SetBrightness(lnBrightness);
    //                }
    //                else
    //                {
    //                    // we have adjusted the best we can
    //                    bIsOk = true;
    //                }
    //            }
                
    //        }
    //        else
    //        {
    //            if (MaxValue < 190)
    //            {
    //                // too dark
    //                lnExposure += 5;
    //                this.SetExposure(lnExposure);

    //                if (lnExposure > 254)
    //                {
    //                    if (lnBrightness < 250)
    //                    {
    //                        lnBrightness += 5;
    //                        this.SetBrightness(lnBrightness);
    //                    }
    //                    else
    //                    {
    //                        // we have adjusted the best we can
    //                        bIsOk = true;
    //                    }
    //                }
    //            }
    //            else
    //            {
    //                // we have adjusted the best we can
    //                bIsOk = true;
    //            }
    //        }
    //        Thread.Sleep(100); // wait for camera to change

    //        try
    //        {
    //            ImageProcess.Image = myCamera.button1_Click();

    //            n = this.ImageProcess.ProccessFrame();

    //            lock (LastFiltered)
    //            {
    //                LastFiltered = this.InputDirName + "\\CI" + DateTime.Now.ToString("HH_mm_ss_") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
    //                //NavColor.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //                ImageProcess.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //                myparent.SetImage(LastFiltered, ImageVote.ToString());
    //            }

    //            lock (logArg)
    //            {
    //                logArg.LogEntry = "Step Calibrate Exposure = " + lnExposure.ToString() + " Brightness " + lnBrightness.ToString() + "\n";
    //                LogEntryReceivedCallBack(this, logArg);
    //            }

    //        }
    //        catch (Exception ex)
    //        {
    //            lock (logArg)
    //            {
    //                DateTime dt = DateTime.Now;
    //                logArg.LogEntry = "Error Robot CalibrateCamera() " + ex.ToString() + dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " \n";
    //                LogEntryReceivedCallBack(this, logArg);
    //            }

    //        }
    //    }



    //    //myCamera.GetCamera(settings);
    //    //settings[14].Mode = (int)CameraControlFlags.Manual;
    //    //myCamera.SetCamera(settings);
    //    //VoteObject n;
    //    //myCamera.GetCamera(settings);   
    //    //int Autovalue= settings[14].Value; 
        
    //    //ImageProcess.Image = myCamera.button1_Click();
    //    //n = this.ImageProcess.ProccessFrame();
    //    //lock (LastFiltered)
    //    //{
    //    //    LastFiltered = this.InputDirName + "\\CI" + DateTime.Now.ToString("HH_mm_ss_") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
    //    //    //NavColor.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //    //    ImageProcess.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //    //    myparent.SetImage(LastFiltered,ImageVote.ToString());
    //    //}
    //    //for (int step = 0; step <= 255; step += 5 )
    //    //    {
    //    //    settings[14].Value = step;  
    //    //    myCamera.SetCamera(settings);
    //    //    Thread.Sleep(1000); // wait for camera to change

    //    //        try
    //    //        {
    //    //            System.Windows.Forms.MessageBox.Show("Step = " + step.ToString(), "before click");
    //    //            ImageProcess.Image = myCamera.button1_Click();

    //    //            n = this.ImageProcess.ProccessFrame();

    //    //            lock (LastFiltered)
    //    //            {
    //    //                LastFiltered = this.InputDirName + "\\CI" + DateTime.Now.ToString("HH_mm_ss_") + DateTime.Now.Millisecond.ToString("000") + ".bmp";
    //    //                //NavColor.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //    //                ImageProcess.filteredimage.Save(LastFiltered, System.Drawing.Imaging.ImageFormat.Bmp);
    //    //                myparent.SetImage(LastFiltered,ImageVote.ToString());
    //    //            }
                    
    //    //            lock (logArg)
    //    //                {
    //    //                    logArg.LogEntry = "Calibrate step " + step.ToString() + LastFiltered + "\n";
    //    //                    LogEntryReceivedCallBack(this, logArg);
    //    //                }

    //    //        }
    //    //        catch (Exception ex)
    //    //        {
    //    //            lock (logArg)
    //    //            {
    //    //                DateTime dt = DateTime.Now;
    //    //                logArg.LogEntry = "Error Robot CalibrateCamera() " + ex.ToString() + dt.ToString("s") + ":" + dt.Millisecond.ToString("000") + " \n";
    //    //                LogEntryReceivedCallBack(this, logArg);
    //    //            }

    //    //        }
    //    //    }

    //    lock (logArg)
    //    {
    //        logArg.LogEntry = "Calibrate Camera Exit Exposure = " + lnExposure.ToString() + " Brightness " + lnBrightness.ToString() + "\n";
    //        LogEntryReceivedCallBack(this, logArg);
    //    }


    //    }

    public void SetExposure(int Exposure)
    {
        //Exposure += 1;
        this.myCamera.SetExposure( Exposure); 
    }
        int myexposure = 1;
    public int GetExposure()
    {
        return this.myCamera.GetExposure();
    }
    public void SetBrightness(int nBright)
    {
        //Exposure += 1;
        this.myCamera.SetBrightness(nBright);
    }

        public int GetBrightness()
    {
        return this.myCamera.GetBrightness();
    }


    void CalcHeadingDistanceMagnetic()
    {

        /*
        Trig stuff
        right triangle Sides ABC where c= hypot a = adjacent b = opposite
        theta = angle from side a to side c
        sin of theta = B/C
        cos od theta = a/C
        tan of theta = b/a
        value of theta = atan(b/a)
        however atan does not give you the correct value depending on the quadrant of the y/x graph

          II  |  I
        ------|-----
         III  | IV
          
          quad 1 0 < angle < (PI/2)
          quad 2 (PI/2) < angle < PI
          quad 3 -PI < angle < -(PI/2)
          quad 4 -(PI/2) < angle < 0)
 
        Quad 1 result is angle measured from X axis (for bearing need to subtract from 90 to get bearing from north)
        Quad 2 angle needs Plus 180 ??
        Quad 3  angle needs Plus 180 ????
        Quad 4 angle plus 90 ???

        hyperphysics.phys-astr.gsu.edu/hbase/ttrig.html
        // according to trig Y is opposite X is adjecent  

         * 
         atan2 help says 
        */

        //-------------------------------------------------------
        double x,y,TempBearing;

        x = gnWay[0,gnWayIndex] - gnCurrentX;
        y = gnWay[1,gnWayIndex] - gnCurrentY;

        gnWayDistance = Math.Sqrt((x*x)+(y*y));
        if (gnWayDistance < 4)
        {
            gnWayIndex++;
            if (gnWayIndex > gnWay.GetLength(1))
            {
                mRemoteLeft = 0;
                mRemoteRight = 0;
                mMode = RobotModeType.RemoteControl;
            }
            else
            {
                x = gnWay[0, gnWayIndex] - gnCurrentX;
                y = gnWay[1, gnWayIndex] - gnCurrentY;

                gnWayDistance = Math.Sqrt((x * x) + (y * y));
            }
        }
        if (gnWayDistance != 0 && (x != 0 || y != 0))
        {
            TempBearing = Math.Atan2(y, x); // returns radians from -3.14 to 3.14
        }
        else
        {
            TempBearing = 0;
        }

        TempBearing = ConvertToDegrees(TempBearing);
        // This value is relative to 90degrees (east) 
        //Need to convert it relative to 0 degrees (North)
        // above the X axis atan2 yields positive number
        // below x axis yields negative number (subtract a negative is adding)
        TempBearing = 90 - TempBearing;
        // subtract 12 to get true bearing
        TempBearing  -= 12; 
        // if the value is now negative then it is between 270 and 360
        // so add 360 to make it a positive number
        if (TempBearing < 0)
	        {
	        TempBearing += 360;
	        }
        // convert it to a targetbearing (4digit number ie like 3601)
        gnTargetBearing = (int)TempBearing;
        if (gnTargetBearing > 360)
        {
            gnTargetBearing -= 360;
        }
        else
        {
            if (gnTargetBearing < 0)
            {
                gnTargetBearing += 360;
            }

        }
        lock (logArg)
        {
            logArg.LogEntry = "CalcHead D " + gnWayDistance.ToString() + " H " + gnTargetBearing + "\n";
            LogEntryReceivedCallBack(this, logArg);
        }

}

void CalcHeadingDistanceXY()
{

            /*
            Trig stuff
            right triangle Sides ABC where c= hypot a = adjacent b = opposite
            theta = angle from side a to side c
            sin of theta = B/C
            cos od theta = a/C
            tan of theta = b/a
            value of theta = atan(b/a)
            however atan does not give you the correct value depending on the quadrant of the y/x graph

              II  |  I
            ------|-----
             III  | IV
          
              quad 1 0 < angle < (PI/2)
              quad 2 (PI/2) < angle < PI
              quad 3 -PI < angle < -(PI/2)
              quad 4 -(PI/2) < angle < 0)
 
            Quad 1 result is angle measured from X axis (for bearing need to subtract from 90 to get bearing from north)
            Quad 2 angle needs Plus 180 ??
            Quad 3  angle needs Plus 180 ????
            Quad 4 angle plus 90 ???

            hyperphysics.phys-astr.gsu.edu/hbase/ttrig.html
            // according to trig Y is opposite X is adjecent  

             * 
             atan2 help says 
            */

            //-------------------------------------------------------
            double x, y, TempBearing;
           
            x = gnWay[0, gnWayIndex] - gnCurrentX;
            y = gnWay[1, gnWayIndex] - gnCurrentY;

            gnWayDistance = Math.Sqrt((x * x) + (y * y));
            if (gnWayDistance < 1)
            {
                gnWayIndex++;
                if (gnWayIndex > gnWay.GetLength(1))
                {
                    mRemoteLeft = 0;
                    mRemoteRight = 0;
                    mMode = RobotModeType.RemoteControl;
                }
                else
                {
                    x = gnWay[0, gnWayIndex] - gnCurrentX;
                    y = gnWay[1, gnWayIndex] - gnCurrentY;

                    gnWayDistance = Math.Sqrt((x * x) + (y * y));
                }
            }
            if (gnWayDistance != 0 && (x != 0 || y != 0))
            {
                TempBearing = Math.Atan2(y, x); // returns radians from -3.14 to 3.14
            }
            else
            {
                TempBearing = 0;
            }

            // for MoveAutoNav3 keep in radians
            // we may need to reverse sign

            //TempBearing = ConvertToDegrees(TempBearing);
            // This value is relative to 90degrees (east) 
            //Need to convert it relative to 0 degrees (North)
            // above the X axis atan2 yields positive number
            // below x axis yields negative number (subtract a negative is adding)
            //TempBearing = 90 - TempBearing;

            gnTargetBearing = TempBearing;  
            lock (logArg)
            {
                logArg.LogEntry = "CalcHead D " + gnWayDistance.ToString() + " H " + gnTargetBearing.ToString() + " waypoint " + gnWayIndex.ToString() + "\n";
                LogEntryReceivedCallBack(this, logArg);
            }

}
const double RADIAN  = 57.2957795;
double ConvertToRadians(double pfDecimalDegrees)
{
    return (pfDecimalDegrees / RADIAN);
}

double ConvertToDegrees(double pfRadians)
{
    return (pfRadians * RADIAN);
} 
const double RADTOFEET = 20888089;

double ConvertToFeet(double pfRadians)
{
return pfRadians * RADTOFEET;
}
const double EQURADIUS = 6378137.0;
const double POLARRADIUS = 6356752.314;
const double FLATTENING = 0.003352811;
const double INVERSE = 298.2572236;
const double MEANRADIUS = 6367435.68;
const double SCALEFACTOR = 0.9996;
const double ECCENTRICITY = 0.081819191;
const double E2 = 0.006739497;
const double N = 0.00167922;


void ConvertToUTM(double pLatitude,double Longitude,double Northing,double easting)
{
    // incoming lat long are decimal degrees
    double lat, lon;
    lat = ConvertToRadians(pLatitude);
    lon = ConvertToRadians(Longitude);


}

        #endregion

        #region tcp control Server

        private static System.Net.Sockets.TcpListener MyTcpListner;
        private System.Threading.Thread TcpThread;
        private bool RunListener = false;

       public void StartListening()
        {
           RunListener = true;

            TcpThread = new Thread(new ThreadStart(this.TcpThreadLoop));
            TcpThread.IsBackground = true;
            TcpThread.Start(); 

        }

       public void StopListening()
        {
            RunListener = false;
        }

       void TcpThreadLoop()
       {

            try
            {
            // Get DNS host information.
            //IPHostEntry hostInfo = Dns.GetHostEntry("10.10.1.1");
            // Get the DNS IP addresses associated with the host.
            //IPAddress[] IPaddresses = hostInfo.AddressList;

            Byte[] bytes = new Byte[256];
           // IPAddress[] IPaddresses = Dns.GetHostEntry("localhost").AddressList;
            AskIP GetIp = new AskIP();
            GetIp.ShowDialog();
            IPAddress ipAddress = GetIp.IPAddress;
            GetIp.Close();
            
            //IPAddress ipAddress = Dns.GetHostEntry("localhost").AddressList[0];
            MyTcpListner = new TcpListener(ipAddress, 2222);
            MyTcpListner.Start(2);

            // Buffer for reading data
            String data = null;

            // Enter the listening loop.
            while (RunListener)
            {

                // Perform a blocking call to accept requests.
                // You could also user server.AcceptSocket() here.
                TcpClient client = MyTcpListner.AcceptTcpClient();

                data = null;

                // Get a stream object for reading and writing
                NetworkStream TcpStream = client.GetStream();

                int i;

                // Loop to receive all the data sent by the client.
                while ((i = TcpStream.Read(bytes, 0, bytes.Length)) != 0)
                    {
                        lock (log)
                        {
                            log.Write(bytes, 0, bytes.GetLength(0));
                            log.Flush();
                        }
                    // Translate data bytes to a ASCII string.
                    data = System.Text.Encoding.ASCII.GetString(bytes, 0, i);
                    data = data.Trim(new Char[] { '\0' }); // need to trim extra nulls off
                    if (data.StartsWith("M"))
                    {
                        // received motor command
                        String[] parts = data.Split(new Char[] { ' ' });
                        if (parts.GetLength(0) == 4)
                        {
                            // update internal values
                            // let robot.move handle it
                            mRemoteLeft = System.Convert.ToInt16(parts[2]);
                            mRemoteRight = System.Convert.ToInt16(parts[3]);
                            data = "OK";
                        }
                        else
                        {
                            data = "INVALID";
                        }
                    }
                    else
                    {
                        if (data.StartsWith("D"))
                        {
                            // received set direction command
                            // only allow change if we are stopped
                            if (mRemoteLeft == 0 && mRemoteRight == 0)
                            {
                                char[] chars = data.ToCharArray();
                                if (chars.GetLength(0) == 5)
                                {
                                    // only grab 5th charactor
                                    // set internal values
                                    // left protperty code do validation
                                    // robot.move will handle it
                                    RemoteDirection = (char)chars[4];
                                    data = "OK";
                                }
                                else
                                {
                                data = "INVALID";
                                }
                            }
                            else
                            {
                                data = "INVALID";
                            }
                        }
                        else
                        {
                            if (data.StartsWith("W"))
                                {
                                // received watchdog request to report pwm
                                    data = "L " + this.LeftPWM.ToString() + " R " + RightPWM.ToString() + " Z";                                 
                                }
                            else
                                if (data.StartsWith("SN"))
                                {
                                    // received request for the current picture
                                    //this.myCamera.Image
                                    //System.Drawing.Bitmap b = new System.Drawing.Bitmap( this.myCamera.Image);
                                    System.IO.MemoryStream ms = new System.IO.MemoryStream();

                                    System.Drawing.Bitmap b =myCamera.button1_Click();
                                    b.Save(ms, System.Drawing.Imaging.ImageFormat.Bmp);
                                    data = ms.Length.ToString();
                                    byte[] lengthMsg = System.Text.Encoding.ASCII.GetBytes(data);
                                    TcpStream.Write(lengthMsg, 0, lengthMsg.Length);

                                    // read the ready
                                    TcpStream.Read(bytes, 0, bytes.Length);

                                    //  now send the picture
                                    //byte[] imagemsg = new byte[ms.Length];
                                    //ms.Read(imagemsg, 0, ms.Length);
                                    //TcpStream.Write(imagemsg, 0, imagemsg.Length);
                                    TcpStream.Write(ms.GetBuffer(), 0, (int)ms.Length);
                                    data = "OK";
                                }
                                else
                                {
                                    if (data.StartsWith("F"))
                                    {
                                        // Start follow mode
                                        if (this.Mode != RobotModeType.ColorFollow)
                                        {
                                            this.Mode = RobotModeType.ColorFollow;
                                        }
                                    }
                                    else
                                    {
                                        if (data.StartsWith("V"))
                                        {
                                        // capture video to disk
                                            lock (CaptureVideo)
                                            {
                                                if (CaptureVideo == "YES")
                                                {
                                                    CaptureVideo = "NO";
                                                }
                                                else
                                                {
                                                    CaptureVideo = "YES";
                                                }
                                            }
                                        }
                                        else
                                        {
                                            // Process the data sent by the Pda.
                                            // should be L and S
                                            // direct pass on to slave
                                            com.SendCommand(data);
                                            data = "OK";
                                        }
                                    }
                                }
                        }
                    }
                    byte[] msg = System.Text.Encoding.ASCII.GetBytes(data);

                    // Send back a response.
                    TcpStream.Write(msg, 0, msg.Length);
                    bytes = new Byte[256];

                    // record what we received

                    }

                // Shutdown and end connection
                client.Close();
                }

           }
            catch (Exception e)
            {
            System.Windows.Forms.MessageBox.Show("Error" + e.Message);
            }

        }

        #endregion
    }
}
