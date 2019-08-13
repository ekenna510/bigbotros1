#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H
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

#endif // MOTORCONTROL_H

