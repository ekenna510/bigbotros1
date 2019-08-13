#include "bno.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "serial.h"
#define BAUDRATE B115200   // Change as needed, keep B
#include <termios.h>
/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyUSB0"
#define NUMOUTGOING 60

class bno
{
protected:
    char OutgoingBuffer[NUMOUTGOING];
public:
    bno();
};

bno::bno()
{

Serial mComm;//= new Serial();
int nbrBytes;
ROS_INFO("Sending H B 1 Reset encoder counter and turn on sensor");
nbrBytes=snprintf(OutgoingBuffer,NUMOUTGOING,"H B 1\n");

mComm.SerialWrite(OutgoingBuffer,nbrBytes);



}
bool MotorControl::InitSerial(char * port, int baud)
{
    return mComm.serialInit(port,baud);
}


void bno::CloseSerial()
{
   mComm.SerialClose();
}


private void ConnectToBNO055()
  {
      GPIO.Write(ResetPin, true);
      Task.Delay(650).Wait();

      Port                = new SerialPort();
      Port.PortName       = PortName;
      Port.BaudRate       = 115200;
      Port.Parity         = Parity.None;
      Port.DataBits       = 8;
      Port.StopBits       = StopBits.One;
      Port.Handshake      = Handshake.None;
      Port.ReadTimeout    = 5000;
      Port.WriteTimeout   = 5000;

      try
      {
          Port.Open();

          while (!Port.IsOpen) { Task.Delay(1).Wait(); }

          SetMode(OperationMode.OPERATION_MODE_CONFIG);

          WriteRegister((byte)PageIDRegisterDefinition.BNO055_PAGE_ID_ADDR, 0, false);

          byte chipID = ReadRegister((byte)PageRegisterDefinitionStart.BNO055_CHIP_ID_ADDR);

          if ((byte)I2CAddress.BNO055_ID == chipID)
          {
              Reset();

              WriteRegister((byte)ModeRegisters.BNO055_PWR_MODE_ADDR, (byte)PowerMode.POWER_MODE_NORMAL);

              WriteRegister((byte)ModeRegisters.BNO055_SYS_TRIGGER_ADDR, 0x0);

              SetMode(OperationMode.OPERATION_MODE_NDOF);

              Connected = true;
              Console.WriteLine("Connected to IMU.");

              Console.WriteLine("Calibrating IMU. . .");

              BNO055.CalibrationStatus lastCal;
              BNO055.CalibrationStatus cal = GetCalibrationStatus();

              Console.WriteLine("System: " + cal.System.ToString() + ", Magnometer: " + cal.Mag.ToString() + ", Gyroscope: " + cal.Gyro.ToString() + ", Accelerometer: " + cal.Accel.ToString());
              while (cal.System != 3 || cal.Mag != 3 || cal.Gyro != 3 || cal.Accel != 3)
              {
                  Task.Delay(250).Wait();

                  lastCal = cal;
                  cal = GetCalibrationStatus();

                  if (cal.Accel != lastCal.Accel || cal.Gyro != lastCal.Gyro || cal.Mag != lastCal.Mag || cal.System != lastCal.System)
                  {
                      Console.WriteLine("System: " + cal.System.ToString() + ", Magnometer: " + cal.Mag.ToString() + ", Gyroscope: " + cal.Gyro.ToString() + ", Accelerometer: " + cal.Accel.ToString());
                  }
              }

              Calibrated = true;
              Console.WriteLine("IMU Calibrated.");
          }
          else
          {
              Console.WriteLine("Unable to connect to IMU.");
          }
      }
      catch (Exception)
      {
          Console.WriteLine("Unable to connect to IMU.");
      }
  }
private void WriteRegister(byte address, byte data, bool ack = true)
 {
     byte[] writeBuffer = new byte[] { 0xAA, 0x00, address, 1, data };
     byte[] response;
     try
     {
         response = WriteData(writeBuffer, ack);
         if (ack)
         {
             if (0xEE01 != (response[0] << 8 | response[1]))
                 throw new Exception(string.Format("WriteRegister returned 0x{0:x2},0x{1:x2}", response[0], response[1]));
         }
     }
     catch (Exception ex)
     {
         Console.WriteLine(string.Format("WriteRegister error: {0}", ex.Message));
     }
 }

 private byte ReadRegister(byte register)
 {
     byte[] commandBuffer = new byte[] { 0xAA, 0x01, register, 1 };
     byte[] readBuffer = new byte[1];
     try
     {
         readBuffer = ReadData(commandBuffer, 1);
     }
     catch (Exception ex)
     {
         Console.WriteLine(string.Format("ReadRegister error: {0}", ex.Message));
     }
     return readBuffer[0];
 }
