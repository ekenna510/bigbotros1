using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.Threading;

namespace master
{
    public partial class Form1 : Form
    {
        delegate void SetErrorCallback(string text);
        delegate void SetPacketCallback(string text);
        delegate void SetStatusCallback(string text);
        delegate void SetXYCallback( string i,string vote);
        private static OutputArgs CurrentSensors = new OutputArgs(); 
       // Com com = new Com();
        master.Robot Myrobot;
        // these needed for mouse remote control
        System.Windows.Forms.Timer doubleClickTimer = new System.Windows.Forms.Timer();
        bool isFirstClick = true;
        int milliseconds = 0;
        bool isDoubleClick = false;
        MouseEventArgs LastMouse;
        bool OKToLeaveMouse = false; 

        private Int16 MaxLeft = 350;
        private Int16 MaxRight = 370;
        private Int16 MinLeft = 220;
        private Int16 MinRight = 220;

        public Form1()
        {
            InitializeComponent();
            Myrobot = new master.Robot(this);

            if (this.Myrobot.Mode == RobotModeType.Auto)
            {
                this.navigateModeToolStripMenuItem.Checked = true;
            }
            if (this.Myrobot.Mode == RobotModeType.RemoteControl)
            {
                this.remoteControlModeToolStripMenuItem.Checked = true;
            }
            if (this.Myrobot.Mode == RobotModeType.ColorFollow)
            {
                this.colorFollowModeToolStripMenuItem.Checked = true;
            }
            doubleClickTimer.Interval = 100;
            doubleClickTimer.Tick += new EventHandler(doubleClickTimer_Tick);


        }
 //       Form1_Load()
        #region form control update routines

        public  void SetRobotStatus(string text)
        {
            // InvokeRequired required compares the thread ID of the
            // calling thread to the thread ID of the creating thread.
            // If these threads are different, it returns true.
            if (this.lblRobotStatus.InvokeRequired)
            {
                SetStatusCallback d = new SetStatusCallback(SetRobotStatus);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                this.lblRobotStatus.Text = text;
            }
            

        }

        public void SetPacketCount(string text)
        {
            // InvokeRequired required compares the thread ID of the
            // calling thread to the thread ID of the creating thread.
            // If these threads are different, it returns true.
            if (this.lblPacketCount.InvokeRequired)
            {
                SetPacketCallback d = new SetPacketCallback(SetPacketCount);
                object[] oObject = new object[] { text };
                this.lblPacketCount.Invoke(d, oObject);
            }
            else
            {
                try
                {

                    if (this.Myrobot != null)
                    {
                        if (CurrentSensors.Front != this.Myrobot.Front || CurrentSensors.Left != this.Myrobot.Left || CurrentSensors.Right != this.Myrobot.Right)
                        {
                            CurrentSensors.Front = this.Myrobot.Front;
                            CurrentSensors.Left = this.Myrobot.Left;
                            CurrentSensors.Right = this.Myrobot.Right;
                            this.sonar1.update(CurrentSensors.Front, CurrentSensors.Left, CurrentSensors.Right);
                        }
                        if (CurrentSensors.Compass != this.Myrobot.Bearing)
                        {
                            CurrentSensors.Compass = this.Myrobot.Bearing;
                            this.compass1.update(CurrentSensors.Compass);
                        }

                        this.speed1.update(Myrobot.LeftSpeed, Myrobot.RightSpeed); 
                    }
                    this.lblPacketCount.Text = text;
                    this.lblLeftTicks.Text = this.Myrobot.LeftTick.ToString();
                    this.lblRightTicks.Text = this.Myrobot.RightTick.ToString();
                    this.lblX.Text = this.Myrobot.X.ToString("0.000");
                    this.lblY.Text = this.Myrobot.Y.ToString("0.000");
                }
                catch (Exception ex)
                {
                    System.Windows.Forms.MessageBox.Show("SetXY Error " + ex.ToString());

                }
            }
        }
        // Make Thread-Safe Calls to Windows Forms Controls 

        public void SetErrorCount(string text)
        {
            // InvokeRequired required compares the thread ID of the
            // calling thread to the thread ID of the creating thread.
            // If these threads are different, it returns true.
            if (this.lblErrorCount.InvokeRequired)
            {
                SetErrorCallback d = new SetErrorCallback(SetErrorCount);
                this.Invoke(d, new object[] { text });
            }
            else
            {
                this.lblErrorCount.Text = text;
            }
        }

        public void Set2ndImage(string i)
        {
            if (this.livestream.InvokeRequired)
            {
                SetStatusCallback d = new SetStatusCallback(Set2ndImage);
                if (!this.IsDisposed)
                {
                    this.Invoke(d, new object[] { i });
                }
            }
            else
            {
                try
                {
                    this.livestream.ImageLocation = i;
                }
                catch (Exception ex)
                {
                    System.Windows.Forms.MessageBox.Show("SetXY Error " + ex.ToString());
                }
            }

        }
        public void SetImage(string i,string vote)
        {
            if (this.pictureBox1.InvokeRequired)
            {
                SetXYCallback d = new SetXYCallback(SetImage);
                if (!this.IsDisposed)
                {
                    this.Invoke(d, new object[] { i, vote });
                }
            }
            else
            {
                  
                try
                {

               this.pictureBox1.ImageLocation = i;
                lblVote.Text = vote;
                }
                catch (Exception ex)
                {
                    System.Windows.Forms.MessageBox.Show("SetXY Error " + ex.ToString());
                }
            }

        }


        #endregion

        #region manual control functions
        private void SetPWM_Click(object sender, EventArgs e)
        {
            this.Myrobot.RemoteLeft = (short) System.Convert.ToInt16(txtLeftPWM.Text);
            this.Myrobot.RemoteRight = (short)System.Convert.ToInt16(this.txtRightPWM.Text);  
            char[] temp = this.txtDirection.Text.ToCharArray(0,1); 
            if (this.Myrobot.Direction != temp[0])
                {
                    this.Myrobot.RemoteDirection = (char)temp[0];
                }

        }

        private void Stop_Click(object sender, EventArgs e)
        {
            
            this.Myrobot.RemoteLeft = 0;
            this.Myrobot.RemoteRight = 0;
            this.txtLeftPWM.Text = this.Myrobot.RemoteLeft.ToString();
            this.txtRightPWM.Text = this.Myrobot.RemoteRight.ToString();

        }

        private void cmdLed_Click(object sender, EventArgs e)
        {
            byte led;
            led = System.Convert.ToByte(txtLed.Text);

            if (led >= 0 && led < 9)
            {
                Myrobot.Leds = led;
            }
        }

        private void barLeftRight_Scroll(object sender, EventArgs e)
        {
            int myleft, myright;
            if (barLeftRight.Value  > 0 )
            {
                myright = this.barSpeed.Value - ((this.barSpeed.Value / 10)* barLeftRight.Value);
                myleft = (short) this.barSpeed.Value;
            }
            else
            {
                // plus on this one because barLeftRight is negative
                myleft = this.barSpeed.Value + ((this.barSpeed.Value / 10) * barLeftRight.Value);
                myright = this.barSpeed.Value;
            }

            this.Myrobot.RemoteLeft = (short) myleft;
            this.Myrobot.RemoteRight = (short) myright;
            this.txtLeftPWM.Text  = myleft.ToString();
            this.txtRightPWM.Text  = myright.ToString();
        }

        private void barSpeed_Scroll(object sender, EventArgs e)
        {
            barLeftRight_Scroll(sender, e);
        }

        private void GoLeft_Click(object sender, EventArgs e)
        {

            this.Myrobot.RemoteLeft = (short)200;
            this.Myrobot.RemoteRight = (short)350;
            this.txtLeftPWM.Text = "200";
            this.txtRightPWM.Text = "350";

        }

        private void GoStraight_Click(object sender, EventArgs e)
        {
            this.Myrobot.RemoteLeft = (short)350;
            this.Myrobot.RemoteRight = (short)350;
            this.txtLeftPWM.Text = "350";
            this.txtRightPWM.Text = "350";

        }

        private void GoRight_Click(object sender, EventArgs e)
        {
            this.Myrobot.RemoteLeft = (short)350;
            this.Myrobot.RemoteRight = (short)200;
            this.txtLeftPWM.Text = "350";
            this.txtRightPWM.Text = "200";

        }


        private void txtDirection_TextChanged(object sender, EventArgs e)
        {
            
            if (this.txtDirection.Text != "")
            {
                char[] temp = this.txtDirection.Text.ToCharArray(0, 1);
                if (this.Myrobot.Direction != temp[0] || this.Myrobot.RemoteDirection != temp[0])
                {
                    this.Myrobot.RemoteDirection = (char)temp[0]; 
                }
            }
        }
        #endregion

        #region menu options
        private void menuStrip1_ItemClicked(object sender, ToolStripItemClickedEventArgs e)
        {
        }

        private void exitToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.Close();

           
           
        }

        private void dummyInputToolStripMenuItem_Click(object sender, EventArgs e)
        {

        }

        private void remoteControlToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (this.remoteControlToolStripMenuItem.Checked)
            {
                this.remoteControlToolStripMenuItem.Checked = false;
                // turn off the tcp listiner
            }
            else
            {
                this.Myrobot.StartListening();
                // turn on the tcp listiner
                this.remoteControlToolStripMenuItem.Checked = true;
            }
        }

        private void webcamToolStripMenuItem_Click(object sender, EventArgs e)
        {

        }

        private void remoteControlModeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            OKToLeaveMouse = true;
            remoteControlModeToolStripMenuItem.Checked = true;
            if (this.Myrobot.Mode != RobotModeType.RemoteControl)
            {
                this.Myrobot.Mode = RobotModeType.RemoteControl; 
            }
            if (this.colorFollowModeToolStripMenuItem.Checked)
            {
                this.colorFollowModeToolStripMenuItem.Checked = false;
            }
            if (this.navigateModeToolStripMenuItem.Checked)
            {
                this.navigateModeToolStripMenuItem.Checked = false;
            }


        }

        private void colorFollowModeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.colorFollowModeToolStripMenuItem.Checked = true;

            if (this.remoteControlModeToolStripMenuItem.Checked)
            {
                this.remoteControlModeToolStripMenuItem.Checked = false;
            }
            if (this.navigateModeToolStripMenuItem.Checked)
            {
                this.navigateModeToolStripMenuItem.Checked = false;
            }
            if (this.Myrobot.Mode != RobotModeType.ColorFollow)
            {
                this.Myrobot.Mode = RobotModeType.ColorFollow;
            }

        }

        private void navigateModeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            this.navigateModeToolStripMenuItem.Checked = true ;
            if (this.Myrobot.Mode != RobotModeType.Auto)
            {
                this.Myrobot.Mode = RobotModeType.Auto;
                
            }
            if (this.colorFollowModeToolStripMenuItem.Checked)
            {
                this.colorFollowModeToolStripMenuItem.Checked = false;
            }
            if (this.remoteControlModeToolStripMenuItem.Checked)
            {
                this.remoteControlModeToolStripMenuItem.Checked = false;
            }

        }

        private void setFollowColorToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (this.Myrobot.Mode != RobotModeType.ColorFollow)
            {

                this.panel1.BackColor = this.Myrobot.setFollowColor();
                
            }


        }

        //private void setNavigateColorToolStripMenuItem_Click(object sender, EventArgs e)
        //{
        //    if (this.Myrobot.Mode != RobotModeType.Auto)
        //    {
        //        this.panel2.BackColor = Myrobot.setNavigateColor();

        //    }

        //}

        #endregion

        private void pictureBox1_Click(object sender, EventArgs e)
        {

        }

        private void cmdSnapit_Click(object sender, EventArgs e)
        {
            string ImageFile;
            ImageFile = Myrobot.SnapIt();
            if (!ImageFile.Equals(""))
            {
            this.livestream.ImageLocation = ImageFile;
            }
        }

        private void setThresholdToolStripMenuItem_Click(object sender, EventArgs e)
        {
            int nThreshold;
            nThreshold = System.Convert.ToInt16(txtThreshold.Text);
            Myrobot.SetThreshold(nThreshold);
        }


        private void grabBearingToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.MyTargetBearing = Myrobot.Bearing;
        }

        private void calibrateCompassToolStripMenuItem1_Click(object sender, EventArgs e)
        {
            Myrobot.CalibrateCompass();
        }

        private void setGainRangeToolStripMenuItem_Click(object sender, EventArgs e)
        {
            AskGain  form = new AskGain();

            if (form.ShowDialog() == DialogResult.OK)
            {
                int Gain,Range;

                Gain = Convert.ToInt32(form.txtGain.Text);
                Range = Convert.ToInt32(form.txtRange.Text);
                Myrobot.SetGainRange(Gain, Range);
                
            }
        }

        private void imageHandlingToolStripMenuItem_Click(object sender, EventArgs e)
        {
            frmTestImage f = new frmTestImage();
            f.Show(); 
        }



        private void navblueToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(1);

        }

        private void navChromaToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(2);

        }

        //private void navEdgeToolStripMenuItem_Click(object sender, EventArgs e)
        //{
        //    Myrobot.SetImageProcessor(1);

        //}

        private void navcolorToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(3);

        }
        private void navcolor2ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(4);
        }

        private void navChroma2ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(5);
        }
        //private void navWaterSToolStripMenuItem_Click(object sender, EventArgs e)
        //{
        //    Myrobot.SetImageProcessor(7);

        //}


        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (Myrobot != null)
            {
                Myrobot.Shutdown();
                Myrobot = null;
            }
            Application.Exit();

        }

        private void setCameraToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.CalibrateCamera();
            //Myrobot.CameraSettings(this.Handle );
        }

        private void livestream_Click(object sender, EventArgs e)
        {

        }

        private void displayPropertyToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.DisplayCameraProperties(this.Handle );
        }
        int nExposure = 0;
        private void getExposureToolStripMenuItem_Click(object sender, EventArgs e)
        {
            nExposure = Myrobot.GetExposure();
            System.Windows.Forms.MessageBox.Show("Exposure " + nExposure.ToString());
        }

        private void setLighterToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (this.txtThreshold.Text.Length != 0)
            {
                nExposure = System.Convert.ToInt32(txtThreshold.Text);
                Myrobot.SetExposure(nExposure);
            }
            else
            {

                Myrobot.SetExposure(nExposure + 10);
            }
            nExposure = Myrobot.GetExposure();
            System.Windows.Forms.MessageBox.Show(nExposure.ToString());

        }

        private void setDarkerToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //nExposure = Myrobot.GetExposure();

            Myrobot.SetExposure(nExposure - 10);
            nExposure = Myrobot.GetExposure();
            System.Windows.Forms.MessageBox.Show(nExposure.ToString());


        }

        int nBright = 0;

        private void getBrightToolStripMenuItem_Click(object sender, EventArgs e)
        {
            nBright = Myrobot.GetBrightness();
        }

        private void setBrightLowerToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetBrightness(nBright -20);
            nBright = Myrobot.GetBrightness();
            System.Windows.Forms.MessageBox.Show(nBright.ToString());
        }

        private void setBrightHigherToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetBrightness(nBright +20);
            nBright = Myrobot.GetBrightness();
            System.Windows.Forms.MessageBox.Show(nBright.ToString());
        }

        private void mousecontrolToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //int a = SystemInformation.MouseWheelScrollDelta ;
            //int b = SystemInformation.MouseWheelScrollLines;
            
            //if(SystemInformation.NativeMouseWheelSupport)
            //{
            //}
            //if (SystemInformation.MouseWheelPresent)
            //{

            //}
            OKToLeaveMouse = false;
            Cursor.Current = Cursors.SizeAll;
            Point newlocation = new Point(this.Location.X + panel3.Location.X + (panel3.Width / 2), this.Location.Y + panel3.Location.Y + (panel3.Height / 2));
            Cursor.Position = newlocation;

            //Point newlocation = new Point(panel3.Location.X + (panel3.Width / 2), panel3.Location.Y + (panel3.Height / 2));
            //Cursor.Position =newlocation;
            ;

        }

        private void panel3_MouseLeave(object sender, EventArgs e)
        {
            if (Myrobot.Mode == RobotModeType.RemoteControl && OKToLeaveMouse == false )
            {
                //Cursor.Position.X = panel3.Left + (panel3.Width / 2);
                Point newlocation = new Point(this.Location.X + panel3.Location.X + (panel3.Width / 2), this.Location.Y + panel3.Location.Y + (panel3.Height / 2));
                Cursor.Position = newlocation;
            }
            //Point newlocation = new Point(panel3.Location.X + (panel3.Width / 2), panel3.Location.Y + (panel3.Height / 2));
            //Cursor.Position = newlocation;

        }

        private void panel3_MouseClick(object sender, MouseEventArgs e)
        {
            if(this.txtDirection.Text != "F")
            {
                this.txtDirection.Text = "F";
    //            this.Myrobot.RemoteDirection = (byte)'F';
            }
            if (e.Button == MouseButtons.Left )
            {

                this.Myrobot.RemoteLeft = (short)MinLeft;
                this.Myrobot.RemoteRight = (short)MaxRight;
                this.txtLeftPWM.Text = MinLeft.ToString();
                this.txtRightPWM.Text = MaxRight.ToString();
            }
            if (e.Button == MouseButtons.Middle )
            {
                if (this.Myrobot.RemoteLeft > 0 || this.Myrobot.RemoteRight > 0) 
                {
                    if (this.Myrobot.RemoteLeft == MaxLeft && this.Myrobot.RemoteRight == MaxRight)
                    {
                        this.Myrobot.RemoteLeft = (short)0;
                        this.Myrobot.RemoteRight = (short)0;
                        this.txtLeftPWM.Text = "0";
                        this.txtRightPWM.Text = "0";
                    }
                    else
                    {
                        this.Myrobot.RemoteLeft = (short)MaxLeft;
                        this.Myrobot.RemoteRight = (short)MaxRight;
                        this.txtLeftPWM.Text = MaxLeft.ToString();
                        this.txtRightPWM.Text = MaxRight.ToString();
                    }
                }
                else
                {
                this.Myrobot.RemoteLeft = (short)MaxLeft;
                this.Myrobot.RemoteRight = (short)MaxRight;
                this.txtLeftPWM.Text = MaxLeft.ToString();
                this.txtRightPWM.Text = MaxRight.ToString();
                }
            }
            if (e.Button == MouseButtons.Right )
            {
                this.Myrobot.RemoteLeft = (short)MaxLeft;
                this.Myrobot.RemoteRight = (short)MinRight; 
                this.txtLeftPWM.Text = MaxLeft.ToString();
                this.txtRightPWM.Text = MinRight.ToString();

            }


            //LastMouse = e;
            ////LastMouse.Clicks = e.Clicks;
            ////LastMouse.Delta = e.Delta;

            //// This is the first mouse click.
            //if (isFirstClick)
            //{
            //    isFirstClick = false;

            //    // Start the double click timer.
            //    doubleClickTimer.Start();
            //}
            //// This is the second mouse click.
            //else
            //{
            //    // Verify that the mouse click is within the double click
            //    // rectangle and is within the system-defined double 
            //    // click period. doubleClickRectangle.Contains(e.Location) &&
            //    if (milliseconds < SystemInformation.DoubleClickTime)
            //    {
            //        isDoubleClick = true;
            //    }
            //}

        }
        private void panel3_MouseDoubleClick(object sender, MouseEventArgs e)
        {

            if (e.Button == MouseButtons.Middle)
            {
                if (this.txtDirection.Text != "F")
                {
                    this.txtDirection.Text = "F";


                }

                this.Myrobot.RemoteLeft = (short)MaxLeft;
                this.Myrobot.RemoteRight = (short)MaxRight;
                this.txtLeftPWM.Text = MaxLeft.ToString();
                this.txtRightPWM.Text = MaxRight.ToString();
            }
            if (e.Button == MouseButtons.Left)
            {
                if (this.txtDirection.Text != "L")
                {
                    this.txtDirection.Text = "L";
                }

                this.Myrobot.RemoteLeft = (short)MaxLeft;
                this.Myrobot.RemoteRight = (short)MaxRight;
                this.txtLeftPWM.Text = MaxLeft.ToString();
                this.txtRightPWM.Text = MaxRight.ToString();
            }

            if (e.Button == MouseButtons.Right)
            {
                if (this.txtDirection.Text != "R")
                {
                    this.txtDirection.Text = "R";
                }

                this.Myrobot.RemoteLeft = (short)MaxLeft;
                this.Myrobot.RemoteRight = (short)MaxRight;
                this.txtLeftPWM.Text = MaxLeft.ToString();
                this.txtRightPWM.Text = MaxRight.ToString();
            }

        }
        void doubleClickTimer_Tick(object sender, EventArgs e)
        {
            milliseconds += 100;
           

            // The timer has reached the double click time limit.
            if (milliseconds >= SystemInformation.DoubleClickTime)
            {
                doubleClickTimer.Stop();

                if (isDoubleClick)
                {
                    
                }
                else
                {
                    if (LastMouse.Button == MouseButtons.Left)
                    {
                        MessageBox.Show("Single Click left");
                    }
                    if (LastMouse.Button == MouseButtons.Middle)
                    {
                        MessageBox.Show("Single Click Middle");
                    }
                    if (LastMouse.Button == MouseButtons.Right)
                    {
                        MessageBox.Show("Single Click Right");
                    }
                }

                // Allow the MouseDown event handler to process clicks again.
                isFirstClick = true;
                isDoubleClick = false;
                milliseconds = 0;
                
            }
        }

        //private void panel3_Click(object sender, EventArgs e)
        //{
        //    this.txtLeftPWM.Text = "0";
        //    this.txtRightPWM.Text = "0";

        //}

        //private void panel3_DoubleClick(object sender, EventArgs e)
        //{
        //    this.txtLeftPWM.Text = "350";
        //    this.txtRightPWM.Text = "350";

        //}

        private void optionsToolStripMenuItem_Click(object sender, EventArgs e)
        {
        }

        private void navSegToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(8);
        }

        private void btnPort_Click(object sender, EventArgs e)
        {

        }



        private void SetMax_Click(object sender, EventArgs e)
        {
            MaxLeft = (short)System.Convert.ToInt16(txtLeftPWM.Text);
            MaxRight = (short)System.Convert.ToInt16(this.txtRightPWM.Text);
            this.Myrobot.SetMaxPWM(MaxLeft, MaxRight);

        }

        private void SetCapture_Click(object sender, EventArgs e)
        {
            Myrobot.SetVideoCapture();
        }

        private void navGreenToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(6);
        }

        private void waypointsOnlyToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(98);
        }

        private void encoderToolStripMenuItem_Click(object sender, EventArgs e)
        {
            frmTestEncoder aform = new frmTestEncoder();
            aform.Show(); 
        }

        private void straightOnlyToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Myrobot.SetImageProcessor(97);
        }




     
 

      



       

       /* private override void pictureBox1_OnPaint(PaintEventArgs e)
        {
            if (this.pictureBox1.Image.
        e.Graphics.DrawImage( (Image) bitmap, 0, 0 );


        }
        */


    }
}