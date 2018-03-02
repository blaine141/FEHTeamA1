#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHIO.h>
#include <FEHmotor.h>
#include <FEHSD.h>
#include <math.h>



FEHMotor frontLeft(FEHMotor::Motor0,5.0);
FEHMotor frontRight(FEHMotor::Motor1,5.0);
FEHMotor backLeft(FEHMotor::Motor2,5.0);
FEHMotor backRight(FEHMotor::Motor3,5.0);
AnalogInputPin CdS_Cell(FEHIO::P0_0);
DigitalEncoder frontLeftEncoder(FEHIO::P2_0, FEHIO::EitherEdge);
DigitalEncoder frontRightEncoder(FEHIO::P2_2, FEHIO::EitherEdge);
DigitalEncoder backLeftEncoder(FEHIO::P2_4, FEHIO::EitherEdge);
DigitalEncoder backRightEncoder(FEHIO::P2_6, FEHIO::EitherEdge);
#define MOTOR_POWER 50.0
#define PI 3.1415926536

float min(float a, float b)
{
    if(a>b)
        return b;
    return a;
}

void buttonDecision(int direction);
void performanceTestOne(float light);
void setFrontLeftSpeed(float speed)
{
    if(speed>0)
        frontLeft.SetPercent((speed*4 + 9.4719) / 2.6);
    else if(speed < 0)
        frontLeft.SetPercent((speed*4 + 8.7879) / 2.6);
    else
        frontLeft.Stop();
}

void setFrontRightSpeed(float speed)
{
    if(speed>0)
        frontRight.SetPercent((speed*4 + 14.368) / 2.6);
    else if(speed < 0)
        frontRight.SetPercent((speed*4 + 13.645) / 2.6);
    else
        frontRight.Stop();
}

void setBackLeftSpeed(float speed)
{
    if(speed>0)
        backLeft.SetPercent((speed*4 + 14.113) / 2.65);
    else if(speed < 0)
        backLeft.SetPercent((speed*4 + 12.346) / 2.65);
    else
        backLeft.Stop();
}

void setBackRightSpeed(float speed)
{
    if(speed>0)
        backRight.SetPercent((speed*4 + 10.498) / 2.53);
    else if(speed < 0)
        backRight.SetPercent((speed*4 + 9.2121) / 2.53);
    else
        backRight.Stop();
}

void drivePolar(float angle, float distance, float percent);
void driveForwardTwo(int counts, float power);
void driveLeftTwo(int counts, float power);
void driveRightTwo(int counts, float power);
void driveBackwardTwo(int counts, float power);
void turnCounterClockwise(float sec);
void turnClockwise(float sec);
void driveForwardFour(int counts, float power);
void driveLeftFour(int counts, float power);
float driveLeftFourCdSCell(int counts, float power);
void driveRightFour(int counts, float power);
void driveBackwardFour(int counts, float power);
int main()
{
    float light = 3.3;
    float speed;
    int frontLeftClicks = 0, frontRightClicks = 0, backRightClicks = 0, backLeftClicks = 0;


    int direction = 1;

    float minCdSCellValue = 3.3;


   drivePolar(10,36,70);
   Sleep(.5);
   drivePolar(170,36,70);
   Sleep(.5);
   drivePolar(270,12.5,70);


    while(true)
    {
        frontLeftEncoder.ResetCounts();
        frontRightEncoder.ResetCounts();
        backLeftEncoder.ResetCounts();
        backRightEncoder.ResetCounts();

        Sleep(1000);

        LCD.WriteAt(frontLeftEncoder.Counts(),0,0);
        LCD.WriteAt(frontRightEncoder.Counts(),0,20);
        LCD.WriteAt(backLeftEncoder.Counts(),0,40);
        LCD.WriteAt(backRightEncoder.Counts(),0,60);

    }

    return 0;

    //Wait for light
    while(light > 2.7)
    {
        light = CdS_Cell.Value();
    }
    //Leave starting area
    driveForwardFour(34, MOTOR_POWER);
    Sleep(500);
    turnCounterClockwise((0.05));
    //Go towards button board, but go past and go to wall, reaading CdS cell values along the way
    minCdSCellValue = driveLeftFourCdSCell(70,MOTOR_POWER);

    LCD.WriteAt(minCdSCellValue,0,20);
    if(minCdSCellValue>=0.6)
    {
        direction=0;
    }
    Sleep(500);
    //Drive back to button board
    driveRightFour(10,MOTOR_POWER);
    driveBackwardFour(5,MOTOR_POWER);
    driveRightFour(10, MOTOR_POWER);
    //Choose a button and drive to it
    buttonDecision(direction);
    //Drive into wrench
    driveRightFour(90,MOTOR_POWER);
    //Drive towards starting/ending area
    driveLeftFour(50,MOTOR_POWER);
    //Drive into the ending button
    driveBackwardFour(50,MOTOR_POWER);
    return 0;
}
void drivePolar(float angle, float distance, float percent)
{
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();

    angle+=45;
    distance = distance * 500 / 81;
    percent = percent / 2;

    float XSpeed = cos(angle*PI/180)* percent;
    float YSpeed = sin(angle*PI/180)* percent;

    float XPos = 0;
    float YPos = 0;

    float XEnd = cos(angle*PI/180)* distance;
    float YEnd = sin(angle*PI/180)* distance;


    float p = 3;

    bool doneDriving = false;

    setFrontRightSpeed(-YSpeed);
    setBackLeftSpeed(-YSpeed);
    setFrontLeftSpeed(-XSpeed);
    setBackRightSpeed(-XSpeed);

    double start = TimeNow();

    while(abs(YPos) < abs(YEnd) || abs(XPos) < abs(XEnd))
    {
        float XPredicted = XSpeed*(TimeNow()-start);
        float YPredicted = YSpeed*(TimeNow()-start);

        if(frontRightEncoder.NewCount() || backLeftEncoder.NewCount())
        {
            if(YSpeed>=0)
            {
                YPos += .5;
            }
            else
            {
                YPos -= .5;
            }
        }

        if(abs(YPos) >= abs(YEnd))
        {
            frontRight.Stop();
            backLeft.Stop();
        }
        else
        {
            float YError = YPredicted - YPos;
            float SlowdownFactor = min(1,.2+.08*(abs(YEnd)-abs(YPos)));

            setFrontRightSpeed(-YSpeed * SlowdownFactor - p * YError);
            setBackLeftSpeed(-YSpeed * SlowdownFactor - p * YError);
        }

        if(frontLeftEncoder.NewCount() || backRightEncoder.NewCount())
        {
            if(XSpeed>=0)
            {
                XPos += .5;
            }
            else
            {
                XPos -= .5;
            }
        }

        if(abs(XPos) >= abs(XEnd))
        {
            frontLeft.Stop();
            backRight.Stop();
        }
        else
        {
            float XError = XPredicted - XPos;
            float SlowdownFactor = min(1,.2+.08*(abs(XEnd)-abs(XPos)));

            setFrontLeftSpeed(-XSpeed * SlowdownFactor - p * XError);
            setBackRightSpeed(-XSpeed * SlowdownFactor - p * XError);
        }
    }
    LCD.WriteAt(frontRightEncoder.Counts(),0,0);
    LCD.WriteAt(frontLeftEncoder.Counts(),0,40);
    LCD.WriteAt(backRightEncoder.Counts(),0,80);
    LCD.WriteAt(backLeftEncoder.Counts(),0,120);
}

void driveForwardTwo(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontRight.SetPercent(-1*power);
    backLeft.SetPercent(-1*power);
    while( sumClicks < 2*counts)
    {
        sumClicks = frontRightEncoder.Counts() + backLeftEncoder.Counts();
    }
    frontRight.Stop();
    backLeft.Stop();
}
void driveLeftTwo(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(power);
    backRight.SetPercent(power);
    while( sumClicks < 2*counts)
    {
        sumClicks = frontLeftEncoder.Counts() + backRightEncoder.Counts();
    }
    frontLeft.Stop();
    backRight.Stop();
}
void driveRightTwo(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(-1*power);
    backRight.SetPercent(-1*power);
    while( sumClicks < 2*counts)
    {
        sumClicks = frontLeftEncoder.Counts() + backRightEncoder.Counts();
    }
    frontLeft.Stop();
    backRight.Stop();
}
void driveBackwardTwo(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontRight.SetPercent(power);
    backLeft.SetPercent(power);
    while( sumClicks < 2*counts)
    {
        sumClicks = frontRightEncoder.Counts() + backLeftEncoder.Counts();
    }
    frontRight.Stop();
    backLeft.Stop();
}
void turnCounterClockwise(float sec)
{
    frontLeft.SetPercent(50.);
    frontRight.SetPercent(-50.);
    backRight.SetPercent(-50.);
    backLeft.SetPercent(50.);
    Sleep(sec);
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();

}
void turnClockwise(float sec)
{
    frontLeft.SetPercent(-50.);
    frontRight.SetPercent(50.);
    backRight.SetPercent(50.);
    backLeft.SetPercent(-50.);
    Sleep(sec);
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}
void driveBackwardFour(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(power);
    frontRight.SetPercent(power);
    backRight.SetPercent(power);
    backLeft.SetPercent(power);
    while( sumClicks < 4*counts)
    {
        sumClicks = frontLeftEncoder.Counts() +frontRightEncoder.Counts() + backRightEncoder.Counts() + backLeftEncoder.Counts();
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}
void driveForwardFour(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(-1*power);
    frontRight.SetPercent(-1*power);
    backRight.SetPercent(-1*power);
    backLeft.SetPercent(-1*power);
    while( sumClicks < 4*counts)
    {
        sumClicks = frontLeftEncoder.Counts() +frontRightEncoder.Counts() + backRightEncoder.Counts() + backLeftEncoder.Counts();
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}
void driveRightFour(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(-1*power);
    frontRight.SetPercent(power);
    backRight.SetPercent(-1*power);
    backLeft.SetPercent(power);
    while( sumClicks < 4*counts)
    {
        sumClicks = frontLeftEncoder.Counts() +frontRightEncoder.Counts() + backRightEncoder.Counts() + backLeftEncoder.Counts();
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}
void driveLeftFour(int counts, float power)
{
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(power);
    frontRight.SetPercent(-1*power);
    backRight.SetPercent(power);
    backLeft.SetPercent(-1*power);
    while( sumClicks < 4*counts)
    {
        sumClicks = frontLeftEncoder.Counts() +frontRightEncoder.Counts() + backRightEncoder.Counts() + backLeftEncoder.Counts();
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}
void performanceTestOne(float light)
{
    while(light > 2.7)
    {
        light = CdS_Cell.Value();
    }
    //Leave starting area
    driveForwardFour(30, MOTOR_POWER);
    Sleep(500);
    //Go owards button board
    driveLeftFour(30,MOTOR_POWER);
    Sleep(500);
    //Go into button board
    driveForwardFour(15,MOTOR_POWER);
    Sleep(500);
    //Leave backwards from button board
    driveBackwardFour(10,MOTOR_POWER);
    Sleep(500);
    //Drive towards the wrench
    driveRightFour(70,MOTOR_POWER);
    Sleep(500);
    //Drive towards wall
    driveForwardFour(75,MOTOR_POWER);
    Sleep(500);
    //Drive into lever
    driveLeftFour(8,MOTOR_POWER);
    Sleep(500);
    //Drive back from hitting lever
    driveRightFour(3,MOTOR_POWER);
    Sleep(500);
    //Drive back towards ramp
    driveBackwardFour(75,MOTOR_POWER);
    Sleep(500);
    //Turn my man
    //turnCounterClockwise(0.8);
    //Sleep(500);
    //Drive into wall
    driveRightFour(15,MOTOR_POWER);
    Sleep(500);
    //Drive up ramp
    driveBackwardFour(150,MOTOR_POWER);
    Sleep(500);
}
void buttonDecision(int direction)
{
    switch(direction)
    {
    case 0:
       driveLeftFour(8,MOTOR_POWER);
       driveForwardFour(15,MOTOR_POWER);
       driveBackwardFour(10,MOTOR_POWER);
       driveRightFour(8,MOTOR_POWER);
       LCD.WriteLine("BLUE");
    break;

    case 1:
       driveRightFour(8,MOTOR_POWER);
       driveForwardFour(15,MOTOR_POWER);
       driveBackwardFour(10,MOTOR_POWER);
       driveLeftFour(8,MOTOR_POWER);
       LCD.WriteLine("RED");
    break;

}
}
float driveLeftFourCdSCell(int counts, float power)
{
    float minCdSCellValue = 20;
    int sumClicks = 0;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(power);
    frontRight.SetPercent(-1*power);
    backRight.SetPercent(power);
    backLeft.SetPercent(-1*power);
    while( sumClicks < 4*counts)
    {
        sumClicks = frontLeftEncoder.Counts() +frontRightEncoder.Counts() + backRightEncoder.Counts() + backLeftEncoder.Counts();
        if (minCdSCellValue>CdS_Cell.Value())
        {
            minCdSCellValue = CdS_Cell.Value();
        }
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
    return minCdSCellValue;
}
