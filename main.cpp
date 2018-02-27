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
#define MOTOR_POWER 75.0
#define PI 3.1415926536

void setFrontLeftSpeed(float speed)
{
    if(speed>0)
        frontLeft.SetPercent((speed*4 + 9.4719) / 2.6932);
    else if(speed < 0)
        frontLeft.SetPercent((speed*4 + 8.7879) / 2.6062);
    else
        frontLeft.Stop();
}

void setFrontRightSpeed(float speed)
{
    if(speed>0)
        frontRight.SetPercent((speed*4 + 14.368) / 2.6083);
    else if(speed < 0)
        frontRight.SetPercent((speed*4 + 13.645) / 2.4834);
    else
        frontRight.Stop();
}

void setBackLeftSpeed(float speed)
{
    if(speed>0)
        backLeft.SetPercent((speed*4 + 14.113) / 2.6842);
    else if(speed < 0)
        backLeft.SetPercent((speed*4 + 12.346) / 2.6488);
    else
        backLeft.Stop();
}

void setBackRightSpeed(float speed)
{
    if(speed>0)
        backRight.SetPercent((speed*4 + 10.498) / 2.5681);
    else if(speed < 0)
        backRight.SetPercent((speed*4 + 9.2121) / 2.4795);
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
void driveRightFour(int counts, float power);
void driveBackwardFour(int counts, float power);
int main(void)
{
    float light = 3.3;
    float speed;
    int frontLeftClicks = 0, frontRightClicks = 0, backRightClicks = 0, backLeftClicks = 0;

drivePolar(20,98,50);
return 0;
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    SD.OpenLog();
        for(int i=-100;i<=100;i+=5)
        {
            LCD.WriteAt("    ",0,0);
            LCD.WriteAt(i,0,0);
            frontLeft.SetPercent(i);
            frontRight.SetPercent(i);
            backLeft.SetPercent(i);
            backRight.SetPercent(i);
            Sleep(2.0);
            frontLeftEncoder.ResetCounts();
            frontRightEncoder.ResetCounts();
            backLeftEncoder.ResetCounts();
            backRightEncoder.ResetCounts();
            Sleep(4.0);
            SD.Printf("%d,%d,%d,%d,%d\n",i,frontLeftEncoder.Counts(),frontRightEncoder.Counts(),backLeftEncoder.Counts(),backRightEncoder.Counts());
        }

FL   speed = 2.6932x - 9.4719       -2.6062x - 8.7879
BL   speed = 2.6842x - 14.113       -2.6488x - 12.346
BR   speed = 2.5681x - 10.498       -2.4795x - 9.2121
FR   speed = 2.6083x - 14.368       -2.4834x - 13.645


        SD.CloseLog();

    return 0;
    int direction = 1;
    if(CdS_Cell.Value()>=0.6)
    {
        direction=0;
    }
    switch(direction)
    {
       case 0:
           driveLeftFour(5,50);
           driveForwardFour(10,50);
           driveBackwardFour(5,50);
           driveRightFour(20,50);
        break;

       case 1:
           driveRightFour(5,50);
           driveForwardFour(10,50);
           driveBackwardFour(5,50);
        break;

       }


    return 0;
    //Wait for light
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
    driveForwardFour(5,MOTOR_POWER);
    Sleep(500);
    //Leave backwards from button board
    driveBackwardFour(10,MOTOR_POWER);
    Sleep(500);
    //Drive towards the wrench
    driveRightFour(70,MOTOR_POWER);
    Sleep(500);
    //Drive towards wall
    driveForwardFour(64,MOTOR_POWER);
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
    return 0;
}
void drivePolar(float angle, float distance, float percent)
{
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();

    float FL = cos(angle*PI/180) - sin(angle*PI/180);
    float FR = cos(angle*PI/180) + sin(angle*PI/180);
    float BL = cos(angle*PI/180) + sin(angle*PI/180);
    float BR = cos(angle*PI/180) - sin(angle*PI/180);

    float distanceFactor = distance/((abs(FL)+abs(FR))/2);
    bool doneDriving = false;

    while(!doneDriving)
    {
        LCD.WriteAt(frontLeftEncoder.Counts(),0,0);
        LCD.WriteAt(frontRightEncoder.Counts(),0,40);
        LCD.WriteAt(frontLeftEncoder.Counts(),0,0);
        LCD.WriteAt(frontRightEncoder.Counts(),0,40);
        doneDriving = true;
        if(frontRightEncoder.Counts() + backLeftEncoder.Counts() < abs(FR*distanceFactor*2))
        {
            frontRight.SetPercent(-percent* FR);
            backLeft.SetPercent(-percent* BL);
            doneDriving = false;
        }
        else
        {
            frontRight.Stop();
            backLeft.Stop();
        }
        if(frontLeftEncoder.Counts() + backRightEncoder.Counts() < abs(BR*distanceFactor*2))
        {
            frontLeft.SetPercent(-percent* FL);
            backRight.SetPercent(-percent* BR);
            doneDriving = false;
        }
        else
        {
            frontLeft.Stop();
            backRight.Stop();
        }
    }
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
