#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHIO.h>
#include <FEHmotor.h>
#include <FEHSD.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <math.h>



FEHMotor frontLeft(FEHMotor::Motor0,5.0);
FEHMotor frontRight(FEHMotor::Motor1,5.0);
FEHMotor backLeft(FEHMotor::Motor2,5.0);
FEHMotor backRight(FEHMotor::Motor3,5.0);
FEHServo bicep(FEHServo::Servo0);
AnalogInputPin CdS_Cell(FEHIO::P0_0);
DigitalEncoder frontLeftEncoder(FEHIO::P2_0, FEHIO::EitherEdge);
DigitalEncoder frontRightEncoder(FEHIO::P2_2, FEHIO::EitherEdge);
DigitalEncoder backLeftEncoder(FEHIO::P2_4, FEHIO::EitherEdge);
DigitalEncoder backRightEncoder(FEHIO::P2_6, FEHIO::EitherEdge);
#define MOTOR_SPEED 60.0
#define PI 3.1415926536

void performanceTestTwo();
float min(float a, float b)
{
    if(a>b)
        return b;
    return a;
}



void buttonDecision(int direction);
void performanceTestOne();
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
void turnCC(float degrees);
void turnC(float degrees);
float driveLeftFourCdSCell(int counts, float power);
void bicepStretch();
void bicepFlex();
void driveUpHill(float percent);

int main()
{
  /* drivePolar(10,36,70);
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

    return 0;*/
    bicep.SetMin(1211);
    bicep.SetMax(2340);

    float light = 3.3;
    float speed;
    int frontLeftClicks = 0, frontRightClicks = 0, backRightClicks = 0, backLeftClicks = 0;
    int direction = 1;
    float minCdSCellValue = 3.3;
    //Wait for light
    while(light > 2.7)
    {
        light = CdS_Cell.Value();
    }
    //Leave starting area
    drivePolar(90, 13.5, MOTOR_SPEED);
    Sleep(500);

    bicepFlex();

    drivePolar(0, 1.5, MOTOR_SPEED);
    Sleep(500);

    drivePolar(90, 5, MOTOR_SPEED);
    Sleep(500);

    drivePolar(270, 4.5, MOTOR_SPEED);
    Sleep(500);

    bicepStretch();
    drivePolar(0, 7, MOTOR_SPEED);
    Sleep(500);
    bicepFlex();
    drivePolar(180,1,MOTOR_SPEED);

    drivePolar(270,5.5,MOTOR_SPEED);
    Sleep(500);
    drivePolar(0,7,MOTOR_SPEED);
    Sleep(500);
    driveUpHill(MOTOR_SPEED);
    Sleep(500);
    turnC(45);
    Sleep(500);
    drivePolar(300,15,MOTOR_SPEED);
    Sleep(500);
    drivePolar(0,7,MOTOR_SPEED);
    bicepStretch();
    Sleep(1000);
    drivePolar(180,7,MOTOR_SPEED);
    Sleep(500);
    drivePolar(260.0,17,MOTOR_SPEED);
    return 0;
}

void bicepStretch()
{
    bicep.SetDegree(120);
}

void bicepFlex()
{
    bicep.SetDegree(0);
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

    float FLPos = 0;
    float FRPos = 0;
    float BLPos = 0;
    float BRPos = 0;

    float XEnd = cos(angle*PI/180)* distance;
    float YEnd = sin(angle*PI/180)* distance;

    float p = 3;

    bool doneDriving = false;

    setFrontRightSpeed(-YSpeed);
    setBackLeftSpeed(-YSpeed);
    setFrontLeftSpeed(-XSpeed);
    setBackRightSpeed(-XSpeed);

    double start = TimeNow();

    while(abs(BLPos) < abs(YEnd) || abs(BRPos) < abs(XEnd))
    {
        float XPredicted = XSpeed*(TimeNow()-start);
        float YPredicted = YSpeed*(TimeNow()-start);

        if(frontLeftEncoder.NewCount())
        {
            if(XSpeed > 0)
                FLPos++;
            else
                FLPos--;
        }

        if(abs(FLPos) >= abs(XEnd))
            frontLeft.Stop();
        else
        {
            float XError = XPredicted - FLPos;
            float SlowdownFactor = min(1,.2+.08*(abs(XEnd)-abs(FLPos)));

            setFrontLeftSpeed(-XSpeed * SlowdownFactor - p * XError);
        }

        if(frontRightEncoder.NewCount())
        {
            if(YSpeed > 0)
                FRPos++;
            else
                FRPos--;
        }

        if(abs(FRPos) >= abs(YEnd))
            frontRight.Stop();
        else
        {
            float YError = YPredicted - FRPos;
            float SlowdownFactor = min(1,.2+.08*(abs(YEnd)-abs(FRPos)));

            setFrontRightSpeed(-YSpeed * SlowdownFactor - p * YError);
        }

        if(backLeftEncoder.NewCount())
        {
            if(YSpeed > 0)
                BLPos++;
            else
                BLPos--;
        }

        if(abs(BLPos) >= abs(YEnd))
            backLeft.Stop();
        else
        {
            float YError = YPredicted - BLPos;
            float SlowdownFactor = min(1,.2+.08*(abs(YEnd)-abs(BLPos)));

            setBackLeftSpeed(-YSpeed * SlowdownFactor - p * YError);
        }

        if(backRightEncoder.NewCount())
        {
            if(XSpeed > 0)
                BRPos++;
            else
                BRPos--;
        }

        if(abs(BRPos) >= abs(XEnd))
            backRight.Stop();
        else
        {
            float XError = XPredicted - BRPos;
            float SlowdownFactor = min(1,.2+.08*(abs(XEnd)-abs(BRPos)));

            setBackRightSpeed(-XSpeed * SlowdownFactor - p * XError);
        }
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}


void driveUpHill(float percent)
{
    percent = percent / 2;
    float XSpeed = cos(315*PI/180)* percent;
    float YSpeed = sin(315*PI/180)* percent;

    setFrontRightSpeed(-YSpeed);
    setBackLeftSpeed(-YSpeed);
    setFrontLeftSpeed(-XSpeed);
    setBackRightSpeed(-XSpeed);

    bool touchedHill = false;
    while(!touchedHill || abs(Accel.X()) > .07)
    {
        if(abs(Accel.X()) >.1)
            touchedHill = true;
        setFrontRightSpeed(-YSpeed * (1+2*abs(Accel.X())));
        setBackLeftSpeed(-YSpeed * (1+2*abs(Accel.X())));
        setFrontLeftSpeed(-XSpeed * (1+2*abs(Accel.X())));
        setBackRightSpeed(-XSpeed * (1+2*abs(Accel.X())));
    }

}


void turnCC(float degrees)
{
    int counts = degrees / 2.025;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(50.);
    frontRight.SetPercent(-50.);
    backRight.SetPercent(-50.);
    backLeft.SetPercent(50.);
    while(frontLeftEncoder.Counts() + frontRightEncoder.Counts() + backLeftEncoder.Counts() + backRightEncoder.Counts() < counts*4) {}
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();

}
void turnC(float degrees)
{
    int counts = degrees / 2.025;
    frontRightEncoder.ResetCounts();
    frontLeftEncoder.ResetCounts();
    backLeftEncoder.ResetCounts();
    backRightEncoder.ResetCounts();
    frontLeft.SetPercent(-50.);
    frontRight.SetPercent(50.);
    backRight.SetPercent(50.);
    backLeft.SetPercent(-50.);
    while(frontLeftEncoder.Counts() + frontRightEncoder.Counts() + backLeftEncoder.Counts() + backRightEncoder.Counts() < counts*4) {}
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}
/*void performanceTestOne()
{
    float light = 3.3;
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
void performanceTestTwo()
{
    float light = 3.3;
    float speed;
    int frontLeftClicks = 0, frontRightClicks = 0, backRightClicks = 0, backLeftClicks = 0;


    int direction = 1;

    float minCdSCellValue = 3.3;



    //Wait for light
    while(light > 2.7)
    {
        light = CdS_Cell.Value();
    }
    //Leave starting area
    drivePolar(0, 8.0, MOTOR_POWER);
    Sleep(500);

    //Go towards button board, but go past and go to wall, reaading CdS cell values along the way

    minCdSCellValue = driveLeftFourCdSCell(70,MOTOR_POWER);

    LCD.WriteAt(minCdSCellValue,0,20);
    if(minCdSCellValue>=.6)
    {
        direction=0;
    }
    Sleep(500);
    //Drive back to button board
    driveRightFour(10,MOTOR_POWER);
    driveBackwardFour(5,MOTOR_POWER);
    driveRightFour(10, MOTOR_POWER);
    Sleep(500);
    //Choose a button and drive to it
    buttonDecision(direction);
    //Drive into wrench
    driveRightFour(90,MOTOR_POWER);
    //Drive towards starting/ending area
    driveLeftFour(50,MOTOR_POWER);
    //Drive into the ending button
    driveBackwardFour(50,MOTOR_POWER);
}*/
