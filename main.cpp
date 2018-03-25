#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHIO.h>
#include <FEHmotor.h>
#include <FEHSD.h>
#include <FEHRPS.h>
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

double curX = 0;
double curY = 0;
double curAngle = 0;




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
void drivePolarNew(float angle, float distance, float percent, float absoluteDirection);
bool driveToCoordinate(float x, float y, float percent);
bool driveToCoordinateNew(float x, float y, float percent);
bool turnToAngle(float angle);
void turnCC(float degrees);
void turnC(float degrees);
void bicepStretch();
void bicepHalfFlex();
void bicepFlex();
void driveUpHill(float percent);
void buttonDecision(int direction);
float driveLeftFourCdSCell(int counts, float power);

int main()
{
    RPS.InitializeTouchMenu();
    spin.SetMin(512);
    spin.SetMax(2600);
    bicep.SetMin(1211);
    bicep.SetMax(2340);

    float light = 3.3;
    int direction;



    if(Battery.Voltage() < 11.0)
    {
        LCD.WriteAt("Charge Me!",0,0);
        LCD.WriteAt(Battery.Voltage(),0,40);

	}


    Sleep(1000);


    curAngle = RPS.Heading();

    while(turnToAngle(90))
    {
        Sleep(1000);
        curAngle = RPS.Heading();
    }

	curX = RPS.X();
    curY = RPS.Y();





    
    while(driveToCoordinate(16.1,25.3,MOTOR_SPEED))
    {
        Sleep(1000);
        curX = RPS.X();
		curY = RPS.Y();
    }
    bicepFlex();
    //Wait for light
    while(light > 2.7)
    {
        light = CdS_Cell.Value();


        curX = RPS.X();
        curY = RPS.Y();

        LCD.WriteAt(curX,0,0);
        LCD.WriteAt(curY,0,40);
    }
    //Leave

    driveToCoordinateNew(curX, curY - 8, MOTOR_SPEED);
    driveToCoordinateNew(curX+8, curY , MOTOR_SPEED);
    driveToCoordinateNew(curX-8, curY , MOTOR_SPEED);
    driveToCoordinateNew(curX, curY + 8, MOTOR_SPEED);

    driveToCoordinateNew(curX, curY - 8, MOTOR_SPEED);
    Sleep(500);

    //Go towards button board, but go past and go to wall, reaading CdS cell values along the way

    //float minCdSCellValue = driveLeftFourCdSCell(70,MOTOR_SPEED);

    //LCD.WriteAt(minCdSCellValue,0,20);
    //if(minCdSCellValue>=.6)
    //{
        direction=0;
    //}
    Sleep(500);
    //Drive back to button board
    driveToCoordinateNew(curX - 10, curY, MOTOR_SPEED);

    driveToCoordinateNew(curX, curY - 10, MOTOR_SPEED);
    driveToCoordinateNew(curX +10, curY, MOTOR_SPEED);
    Sleep(500);
    //Choose a button and drive to it
    //buttonDecision(direction);
    //Drive into wrench
    driveToCoordinateNew(curX - 20, curY, MOTOR_SPEED);
    //Drive to wall
    driveToCoordinateNew(curX , curY+18, MOTOR_SPEED);
    Sleep(500);
    //Drive into car jack
    drivePolar(180, 5, MOTOR_SPEED);
    Sleep(500);
    //Drive back to wrench
    drivePolar(270, 10.5, MOTOR_SPEED);
    Sleep(500);
    //Back up
    drivePolar(180, 5, MOTOR_SPEED);
    Sleep(500);
    //Lower bicep, drive into wrench and pick it up
    bicepStretch();
    drivePolar(0, 5.5, MOTOR_SPEED);
    Sleep(200);
    bicepHalfFlex();
    Sleep(200);
    drivePolar(180,1,MOTOR_SPEED);
    //Drive towards ramp
    drivePolar(270,5.5,MOTOR_SPEED);
    bicepFlex();
    Sleep(500);
    drivePolar(0,3,MOTOR_SPEED);
    Sleep(500);
    //Drive up the hill
    driveUpHill(75);
    Sleep(500);
    //Turn to face garage
    turnCC(45);
    Sleep(500);
    //Drive to road leading up to garage
    drivePolar(280,18,MOTOR_SPEED);
    Sleep(500);
    //Drive to garage and deposit wrench
    drivePolar(0,11,MOTOR_SPEED);
    bicepStretch();
    Sleep(1000);
    //Drive to previous position on road
    drivePolar(180,11,MOTOR_SPEED);
    Sleep(500);
    //DRIVE TO AND SPIN THE BOY
    int turnChoice = RPS.FuelType();
    if (turnChoice == 1){
        spin.SetDegree(0);
    }else{
        spin.SetDegree(180);
    }

    curAngle = RPS.Heading();

    while(turnToAngle(45))
    {
        Sleep(1000);
        curAngle = RPS.Heading();
    }

    Sleep(1000);

    curX = RPS.X();
    curY = RPS.Y();

    while(driveToCoordinate(21.6,62.6,MOTOR_SPEED))
    {
       Sleep(1000);
       curX = RPS.X();
       curY = RPS.Y();
    }



    drivePolar(270,3,MOTOR_SPEED);
    Sleep(500);
    if (turnChoice == 1){
       spin.SetDegree(180);
    }else{
        spin.SetDegree(0);
    }
    Sleep(1000);
    //NO LONGER SPINNING THE BOY
    //Drive back to road
    drivePolar(90,15,MOTOR_SPEED);
    //Drive towards ramp
    drivePolar(180,18,MOTOR_SPEED);
    //Turn and go backwards down the ramp
    turnC(45);
    drivePolar(180,21,MOTOR_SPEED);
    //Drive to starting box
    drivePolar(90,7,MOTOR_SPEED);
    //End the run
    drivePolar(0,8,MOTOR_SPEED);
    return 0;


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

    while(abs((abs(BLPos) + abs(FRPos)) / 2 - abs(YEnd)) > allowableError || abs((abs(BRPos) + abs(FLPos)) / 2 - abs(XEnd)) > allowableError)
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


void drivePolarNew(float angle, float distance, float percent, float absoluteDirection)
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

    float lastX = RPS.X();
    float lastY = RPS.Y();
    bool correctionMade = false;

    double lastTime = TimeNow();

    while(abs((abs(BLPos) + abs(FRPos)) / 2 - abs(YEnd)) > allowableError || abs((abs(BRPos) + abs(FLPos)) / 2 - abs(XEnd)) > allowableError)
    {

        float slowdownFactorY = min(abs((abs(BLPos) + abs(FRPos)) / 2 - YEnd) / 12,1);
        float slowdownFactorX = min(abs((abs(BRPos) + abs(FLPos)) / 2 - XEnd) / 12,1);


        if(frontLeftEncoder.NewCount())
            FLPos += isPositive(XSpeed * slowdownFactorX + p * (FLPredicted - FLPos));
        if(frontRightEncoder.NewCount())
            FRPos += isPositive(YSpeed * slowdownFactorY + p * (FRPredicted - FRPos));
        if(backLeftEncoder.NewCount())
            BLPos += isPositive(YSpeed * slowdownFactorY + p * (BLPredicted - BLPos));
        if(backRightEncoder.NewCount())
            BRPos += isPositive(XSpeed * slowdownFactorX + p * (BRPredicted - BRPos));

        if(!correctionMade)
        {
            float RPSX = RPS.X();
            float RPSY = RPS.Y();
            if(RPSX != lastX || RPSY != lastY)
            {
                float movementDir = atan((RPSY - lastY)/(RPSX-lastX))*180/PI;
                if((RPSX-lastX)<0 )
                    movementDir -= 180;
                movementDir += 270;

                float angleError = movementDir - absoluteDirection;

                if(angleError <-180)
                    angleError += 360;
                if(angleError >180)
                    angleError -= 360;


                LCD.WriteAt(correctionMade,0,40);\
                LCD.WriteAt(movementDir,0,80);

                if(abs(angleError)<10)
                {
                    correctionMade = true;
                }
                lastX = RPSX;
                lastY = RPSY;
            }
        }

        LCD.WriteAt(correctionMade,0,0);

        double currentTime = TimeNow();

        FLPredicted += XSpeed*(currentTime - lastTime);
        FRPredicted += YSpeed*(currentTime - lastTime);
        BLPredicted += YSpeed*(currentTime - lastTime);
        BRPredicted += XSpeed*(currentTime - lastTime);

        lastTime = currentTime;




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


// 0 is by garage
// 90 is wrench
// 180 is car

bool driveToCoordinate(float x, float y, float percent)
{

    if(abs(x-curX) < .6 && abs(y-curY)<.6)
        return false;



    float angle = atan((y-curY)/(x-curX))*180/PI;
    if(x-curX < 0)
    {
        angle += 180;
    }

    angle = angle  - 90 -  curAngle;

    LCD.WriteAt(angle,0,80);

    float distance = sqrt((y-curY)*(y-curY)+(x-curX)*(x-curX));

    drivePolar(angle, distance, percent);

    curX = x;
    curY = y;


	return true;

}

bool driveToCoordinateNew(float x, float y, float percent)
{

    if(abs(x-curX) < .6 && abs(y-curY)<.6)
        return false;



    float angle = atan((y-curY)/(x-curX))*180/PI;
    if(x-curX < 0)
    {
        angle += 180;
    }

    angle -= 90;

    if(angle<0)
        angle += 360;

    float absoluteAngle = angle;

    angle = angle - curAngle;

    float distance = sqrt((y-curY)*(y-curY)+(x-curX)*(x-curX));

    drivePolarNew(angle, distance, percent, absoluteAngle);

    curX = x;
    curY = y;


    return true;

}

bool turnToAngle(float angle)
{
    float angleError = angle - curAngle;
    if(angleError > 180)
        angleError -= 360;
    if(angleError < -180)
        angleError += 360;
    curAngle = angle;
    LCD.WriteAt(angleError,0,0);
    LCD.WriteAt(curAngle,0,40);
    if(angleError > 2)
        turnCC(angleError);
    else if(angleError < -2)
        turnC(-angleError);
    else
        return false;
    return true;
}


void driveUpHill(float percent)
{
    percent = percent / 2;
    float XSpeed = cos(-45*PI/180)* percent;
    float YSpeed = sin(-45*PI/180)* percent;

    setFrontRightSpeed(YSpeed);
    setBackLeftSpeed(YSpeed);
    setFrontLeftSpeed(XSpeed);
    setBackRightSpeed(XSpeed);
    
    float accel = 0;
    
    bool touchedHill = false;
    while(!touchedHill || accel > .07)
    {
        accel = abs(Accel.X());
        if(accel >.15)
            touchedHill = true;
        setFrontRightSpeed(YSpeed * (1+2*accel));
        setBackLeftSpeed(YSpeed * (1+2*accel));
        setFrontLeftSpeed(XSpeed * (1+2*accel));
        setBackRightSpeed(XSpeed * (1+2*accel));
    }
    
    frontRight.Stop();
    frontLeft.Stop();
    backRight.Stop();
    backLeft.Stop();

}


void turnCC(float degrees)
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
void turnC(float degrees)
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

void buttonDecision(int direction)
{
    switch(direction)
    {
    case 0:
       drivePolar(180,2,MOTOR_SPEED);
       drivePolar(90,3,MOTOR_SPEED);
       while(RPS.IsDeadzoneActive() != 2)
       {
           if(RPS.IsDeadzoneActive() != 1)
           {
               setFrontLeftSpeed(-MOTOR_SPEED/2);
               setFrontRightSpeed(MOTOR_SPEED/2);
               setBackLeftSpeed(MOTOR_SPEED/2);
               setBackRightSpeed(-MOTOR_SPEED/2);
           }
           else
           {
               frontRight.Stop();
               frontLeft.Stop();
               backRight.Stop();
               backLeft.Stop();
           }

       }
       drivePolar(270,3,MOTOR_SPEED);
       drivePolar(0,2,MOTOR_SPEED);
       LCD.WriteLine("BLUE");
    break;

    case 1:
       drivePolar(0,2,MOTOR_SPEED);
       drivePolar(90,3,MOTOR_SPEED);
       while(RPS.IsDeadzoneActive() != 2)
       {
           if(RPS.IsDeadzoneActive() != 1)
           {
               setFrontLeftSpeed(-MOTOR_SPEED/2);
               setFrontRightSpeed(MOTOR_SPEED/2);
               setBackLeftSpeed(MOTOR_SPEED/2);
               setBackRightSpeed(-MOTOR_SPEED/2);
           }
           else
           {
               frontRight.Stop();
               frontLeft.Stop();
               backRight.Stop();
               backLeft.Stop();
           }

       }
       drivePolar(270,3,MOTOR_SPEED);
       drivePolar(180,2,MOTOR_SPEED);
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
    frontLeft.SetPercent(-power);
    frontRight.SetPercent(-power);
    backRight.SetPercent(-power);
    backLeft.SetPercent(-power);
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
 
 void performanceTestFour(){
    RPS.InitializeTouchMenu();
    spin.SetMin(512);
    spin.SetMax(2600);
    float light = 3.3;


    if(Battery.Voltage() < 11.0)
    {
        LCD.WriteAt("Charge Me!",0,0);
        LCD.WriteAt(Battery.Voltage(),0,40);

	}


    Sleep(1000);


    curAngle = RPS.Heading();

    while(turnToAngle(90))
    {
        Sleep(1000);
        curAngle = RPS.Heading();
    }

    LCD.WriteAt(curX,0,0);
    LCD.WriteAt(curY,0,40);

    curX = RPS.X();
    curY = RPS.Y();




    //Wait for light
    while(light > 2.7)
    {
        light = CdS_Cell.Value();


        curX = RPS.X();
        curY = RPS.Y();

        LCD.WriteAt(curX,0,0);
        LCD.WriteAt(curY,0,40);
    }
    //Leave

    drivePolar(90,7,MOTOR_SPEED);
    drivePolar(180,9,MOTOR_SPEED);
    drivePolar(90,5,MOTOR_SPEED);
    while(RPS.IsDeadzoneActive() != 2)
    {
        if(RPS.IsDeadzoneActive() != 1)
        {
            setFrontLeftSpeed(-MOTOR_SPEED);
            setFrontRightSpeed(MOTOR_SPEED);
            setBackLeftSpeed(MOTOR_SPEED);
            setBackRightSpeed(-MOTOR_SPEED);
        }
        else
        {
            frontRight.Stop();
            frontLeft.Stop();
            backRight.Stop();
            backLeft.Stop();
        }

    }
    drivePolar(270,5,MOTOR_SPEED);
    drivePolar(0,19,MOTOR_SPEED);
    driveUpHill(MOTOR_SPEED);

    turnC(45);
    drivePolar(0,3.3,MOTOR_SPEED);
    int turnChoice = RPS.FuelType();
    if (turnChoice == 1){
        spin.SetDegree(0);
    }else{
        spin.SetDegree(180);
    }

    drivePolar(270,25,MOTOR_SPEED);
    Sleep(1000);

    curAngle = RPS.Heading();

    while(turnToAngle(45))
    {
        Sleep(1000);
        curAngle = RPS.Heading();
    }

    Sleep(1000);

    curX = RPS.X();
    curY = RPS.Y();

    while(driveToCoordinate(21.6,62.6,MOTOR_SPEED))
    {
       Sleep(1000);
       curX = RPS.X();
       curY = RPS.Y();
    }



    drivePolar(270,3,MOTOR_SPEED);
    Sleep(500);
    if (turnChoice == 1){
       spin.SetDegree(180);
    }else{
        spin.SetDegree(0);
    }
    Sleep(1000);
    drivePolar(90,15,MOTOR_SPEED);
    drivePolar(180,18,MOTOR_SPEED);
    turnC(45);
    drivePolar(180,19,MOTOR_SPEED);
    }
*/
