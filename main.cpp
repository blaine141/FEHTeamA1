#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHIO.h>
#include <FEHmotor.h>
#include <FEHSD.h>
#include <FEHServo.h>
#include <FEHAccel.h>
#include <FEHBattery.h>
#include <math.h>
#include <FEHRPS.h>



FEHMotor frontLeft(FEHMotor::Motor0,5.0);
FEHMotor frontRight(FEHMotor::Motor1,5.0);
FEHMotor backLeft(FEHMotor::Motor2,5.0);
FEHMotor backRight(FEHMotor::Motor3,5.0);
FEHServo bicep(FEHServo::Servo0);
FEHServo spin(FEHServo::Servo1);
AnalogInputPin CdS_Cell(FEHIO::P0_0);
DigitalEncoder frontLeftEncoder(FEHIO::P2_0, FEHIO::EitherEdge);
DigitalEncoder frontRightEncoder(FEHIO::P2_1, FEHIO::EitherEdge);
DigitalEncoder backLeftEncoder(FEHIO::P2_2, FEHIO::EitherEdge);
DigitalEncoder backRightEncoder(FEHIO::P2_3, FEHIO::EitherEdge);
//I2C compass(FEHIO::P3_4,FEHIO::P3_4,0b0001101);
#define MOTOR_SPEED 60.0
#define PI 3.1415926536




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

int isPositive(int val)
{
    if (val > 0)
        return 1;
    return -1;
}

float min(float a, float b)
{
    if(a > b)
        return b;
    return a;
}

void drivePolar(float angle, float distance, float percent);
void turnCC(float degrees);
void turnC(float degrees);
void bicepStretch();
void bicepHalfFlex();
void bicepFlex();
void driveUpHill(float percent);

int main()
{

    RPS.InitializeTouchMenu();
    spin.SetMin(512);
    spin.SetMax(2389);
    float light = 3.3;
    int turnChoice = RPS.FuelType();
    if (turnChoice == 1){
        spin.SetDegree(0);
    }else{
        spin.SetDegree(180);
    }
    //Wait for light
    while(light > 2.7)
    {
        light = CdS_Cell.Value();
    }
    drivePolar(180,6,MOTOR_SPEED);
    drivePolar(270,20,MOTOR_SPEED);
    driveUpHill(MOTOR_SPEED);
    turnCC(45);
    drivePolar(0,18,MOTOR_SPEED);
    drivePolar(270,15,MOTOR_SPEED);
    if (turnChoice == 1){
       spin.SetDegree(180);
    }else{
        spin.SetDegree(0);
    }
    drivePolar(90,15,MOTOR_SPEED);
    drivePolar(180,18,MOTOR_SPEED);
    turnC(45);
    drivePolar(180,10,MOTOR_SPEED);
}


#define STRETCH_POSITION 110

void bicepStretch()
{
    bicep.SetDegree(STRETCH_POSITION);
}

void bicepHalfFlex()
{

    bicep.SetDegree(STRETCH_POSITION / 2);
}

void bicepFlex()
{
    bicep.SetDegree(0);
}


void drivePolar(float angle, float distance, float percent)
{
    angle+=45;
    distance = distance * 500 / 81;
    percent = percent / 2;

    const int allowableError = 2;

    float XSpeed = cos(angle*PI/180)* percent;
    float YSpeed = sin(angle*PI/180)* percent;

    float FLPos = 0;
    float FRPos = 0;
    float BLPos = 0;
    float BRPos = 0;

    float XEnd = cos(angle*PI/180)* distance;
    float YEnd = sin(angle*PI/180)* distance;

    float FLPredicted = 0;
    float FRPredicted = 0;
    float BLPredicted = 0;
    float BRPredicted = 0;

    float p = 5;

    double lastTime = TimeNow();

    while(abs((abs(BLPos) + abs(FRPos)) / 2 - YEnd) > allowableError || abs((abs(BRPos) + abs(FLPos)) / 2 - XEnd) > allowableError)
    {
        if(frontLeftEncoder.NewCount())
            FLPos += isPositive(XSpeed);
        if(frontRightEncoder.NewCount())
            FRPos += isPositive(YSpeed);
        if(backLeftEncoder.NewCount())
            BLPos += isPositive(YSpeed);
        if(backRightEncoder.NewCount())
            BRPos += isPositive(XSpeed);


        double currentTime = TimeNow();

        FLPredicted += XSpeed*(currentTime - lastTime);
        FRPredicted += YSpeed*(currentTime - lastTime);
        BLPredicted += YSpeed*(currentTime - lastTime);
        BRPredicted += XSpeed*(currentTime - lastTime);

        lastTime = currentTime;


        float slowdownFactorY = min(abs((abs(BLPos) + abs(FRPos)) / 2 - YEnd) / 12,1);
        float slowdownFactorX = min(abs((abs(BRPos) + abs(FLPos)) / 2 - XEnd) / 12,1);

        setFrontLeftSpeed(XSpeed * slowdownFactorX + p * (FLPredicted - FLPos));
        setFrontRightSpeed(YSpeed * slowdownFactorY + p * (FRPredicted - FRPos));
        setBackLeftSpeed(YSpeed * slowdownFactorY + p * (BLPredicted - BLPos));
        setBackRightSpeed(XSpeed * slowdownFactorX + p * (BRPredicted - BRPos));
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
}


void driveUpHill(float percent)
{
    percent = percent / 2;
    float XSpeed = cos(45*PI/180)* percent;
    float YSpeed = sin(45*PI/180)* percent;

    setFrontRightSpeed(-YSpeed);
    setBackLeftSpeed(-YSpeed);
    setFrontLeftSpeed(-XSpeed);
    setBackRightSpeed(-XSpeed);

    bool touchedHill = false;
    while(!touchedHill || abs(Accel.Y()) > .07)
    {
        if(abs(Accel.Y()) >.1)
            touchedHill = true;
        setFrontRightSpeed(-YSpeed * (1+2*abs(Accel.Y())));
        setBackLeftSpeed(-YSpeed * (1+2*abs(Accel.Y())));
        setFrontLeftSpeed(-XSpeed * (1+2*abs(Accel.Y())));
        setBackRightSpeed(-XSpeed * (1+2*abs(Accel.Y())));
    }

}


void turnCC(float degrees)
{
    int counts = degrees / 2.1;
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
    int counts = degrees / 2.1;
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
void driveLeftFour(int counts, float power)
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
    return
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
}

void performanceTestThree(){

    bicep.SetMin(1211);
    bicep.SetMax(2340);
    bicepFlex();

    float light = 3.3;
    float speed;
    int frontLeftClicks = 0, frontRightClicks = 0, backRightClicks = 0, backLeftClicks = 0;
    int direction = 1;
    float minCdSCellValue = 3.3;

    if(Battery.Voltage() < 11.0)
    {
        LCD.WriteAt("Charge Me!",0,0);
        LCD.WriteAt(Battery.Voltage(),0,40);
        return 0;
    }

    //Wait for light
    while(light > 2.7)
    {
        light = CdS_Cell.Value();
    }
    //Leave starting area
    drivePolar(90, 13.5, MOTOR_SPEED);
    Sleep(500);

    drivePolar(0, 9, MOTOR_SPEED);
    Sleep(500);

    drivePolar(90, 18, MOTOR_SPEED);
    Sleep(500);

    drivePolar(180, 5, MOTOR_SPEED);
    Sleep(500);

    drivePolar(270, 10.5, MOTOR_SPEED);
    Sleep(500);

    drivePolar(180, 5, MOTOR_SPEED);
    Sleep(500);

    bicepStretch();
    drivePolar(0, 5.5, MOTOR_SPEED);
    Sleep(200);
    bicepHalfFlex();
    Sleep(200);
    drivePolar(180,1,MOTOR_SPEED);

    drivePolar(270,5.5,MOTOR_SPEED);
    bicepFlex();
    Sleep(500);
    drivePolar(0,3,MOTOR_SPEED);
    Sleep(500);
    turnC(90);
    drivePolar(90,1,MOTOR_SPEED);
    Sleep(500);
    driveUpHill(75);
    Sleep(500);
    turnCC(45);
    Sleep(500);
    drivePolar(280,18,MOTOR_SPEED);
    Sleep(500);
    drivePolar(0,11,MOTOR_SPEED);
    bicepStretch();
    Sleep(1000);
    drivePolar(180,7,MOTOR_SPEED);
    Sleep(500);
    drivePolar(260.0,17,MOTOR_SPEED);
    return 0;
 }
*/
