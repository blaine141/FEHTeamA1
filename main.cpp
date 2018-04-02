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
#include <FEHBuzzer.h>


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
float courseOffsetX;
float courseOffsetY;

const double lightX = 25.0;
const double lightY = 17.7;
bool start = false;



#define MOTOR_SPEED 80.0
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

void whereAmI();
void drivePolar(float angle, float distance, float percent);
void drivePolarNew(float angle, float distance, float percent);
bool driveToCoordinate(float x, float y, float percent);
bool driveToCoordinateNew(float x, float y, float percent);
bool turnToAngle(float angle);
void turnCC(float degrees);
void turnC(float degrees);
void bicepStretch();
void bicepSlowFlex();
void bicepFlex();
void bicepHalfFlex();
void driveUpHill(float percent);
void buttonDecision(int direction);
float getRPSX();
float getRPSY();
float getRPSHeading();
void bicepSlowFlex(int ms);
void RPSbutton();


int main()
{
    spin.SetMin(512);
    spin.SetMax(2600);
    bicep.SetMin(1211);
    bicep.SetMax(2100);
    bicepStretch();
    SD.OpenLog();
    RPS.InitializeTouchMenu();

    bicepSlowFlex(1000);

    float light = 3.3;
    int direction;

    courseOffsetX = getRPSX() - 0.9;
    courseOffsetY = getRPSY() - 7.6;

    if(Battery.Voltage() < 10.8)
    {
        LCD.WriteAt("Charge Me!",0,0);
        LCD.WriteAt(Battery.Voltage(),0,40);
        return 0;
	}


    LCD.WriteLine("MOVE ME!!   ");
    LCD.WriteLine(courseOffsetX);
    LCD.WriteLine(courseOffsetY);

    while(getRPSX() < 15){}

    Sleep(1000);


    whereAmI();

/*
    float RPSbuttonX = RPS.X();
    float RPSbuttonY = RPS.Y();
    driveToCoordinate(curX-8,curY,MOTOR_SPEED);*/

    while(turnToAngle(90))
    {
        Sleep(1000);
        whereAmI();
    }

    whereAmI();

    
    while(driveToCoordinate(15.6 + courseOffsetX,25.9 + courseOffsetY,MOTOR_SPEED))
    {
        Sleep(1000);
        whereAmI();
    }
    bicepFlex();


    while(RPS.X()>0)
    {
        bicepStretch();
        Buzzer.Buzz(0);
        bicepSlowFlex(500);
        LCD.WriteLine("COVER ME BROOOOOOO!!!!!!");

    }

    Buzzer.Buzz(1);

    LCD.Clear(FEHLCD::White);
    LCD.PrintImage(35,0);
    LCD.PrintLogo(130,92);
    LCD.SetFontColor(FEHLCD::Black);

    //Wait for light
    long startingTime = TimeNowSec();
    while(!start)
    {
        light = CdS_Cell.Value();
        if (light < 2.7){
            start = true;
        }
        if (TimeNowSec()-startingTime >= 30){
            start = true;
        }
        whereAmI();


        //LCD.WriteAt(curX,0,0);
        //LCD.WriteAt(curY,0,40);
    }


    //////////////////////////////////////////////
    /// START OF MATCH
    /////////////////////////////////////////////

    driveToCoordinateNew(curX, lightY + courseOffsetY, MOTOR_SPEED);
    //Drive to light
    while(driveToCoordinateNew(lightX + courseOffsetX, lightY + courseOffsetY, MOTOR_SPEED))
    {
        Sleep(1000);
        whereAmI();
    }
    //Pick a light and drive to it
    Sleep(500);
    if(CdS_Cell.Value() < 0.8){
        LCD.WriteAt("red",0,0);
        direction = 1;
    }else{
        LCD.WriteAt("blue",0,0);
        direction = 0;

    }
    //LCD.WriteAt(CdS_Cell.Value(),0,120);
    buttonDecision(direction);
    curAngle = RPS.Heading();
    turnToAngle(90);
    driveToCoordinateNew(curX-14, curY+3, MOTOR_SPEED);

    driveToCoordinateNew(2.2 + courseOffsetX,8 + courseOffsetY,MOTOR_SPEED);

    driveToCoordinateNew(curX, curY - 4, MOTOR_SPEED);

    driveToCoordinateNew(curX + 6, curY, MOTOR_SPEED);

    Sleep(1000);

    whereAmI();

    driveToCoordinateNew(curX - 3, curY, MOTOR_SPEED);

    driveToCoordinateNew(curX+2, curY + 4, MOTOR_SPEED);


    //drive to wrench

    whereAmI();
    while(turnToAngle(90))
    {
        Sleep(1000);
        whereAmI();
    }


    while(driveToCoordinate(11 + courseOffsetX, 13.1 + courseOffsetY,MOTOR_SPEED))
    {
        Sleep(1000);
        whereAmI();
    }

    while(turnToAngle(90))
    {
        Sleep(1000);
        whereAmI();
    }

    //Pick up wrench and drive to ramp
    bicepStretch();
    Sleep(500);
    driveToCoordinateNew(curX-5,curY,MOTOR_SPEED/2);
    Sleep(500);
    driveToCoordinateNew(curX+0.7,curY,MOTOR_SPEED/2);
    Sleep(500);
    bicepSlowFlex(1000);
    Sleep(500);
    driveToCoordinateNew(curX+3.5,curY+8,MOTOR_SPEED);
    driveToCoordinateNew(curX-4, curY , MOTOR_SPEED);
    turnToAngle(0);
    //Drive up the hill
    driveUpHill(MOTOR_SPEED);
    Sleep(1000);
    whereAmI();
    driveToCoordinateNew(curX+4, curY+6 , MOTOR_SPEED);
    //Turn to face garage
    turnToAngle(45);
    Sleep(1000);
    whereAmI();

    //Drive to road leading up to garage
    driveToCoordinate(14.0 + courseOffsetX, 56.2 + courseOffsetY, MOTOR_SPEED);
    Sleep(1000);
    while(RPS.X()<0)
    {
        driveToCoordinate(curX + 1, curY - 1, MOTOR_SPEED);
        Sleep(1000);
    }

    curAngle = getRPSHeading();
    driveToCoordinate(14.0 + courseOffsetX, 56.2 + courseOffsetY, MOTOR_SPEED);
    while(RPS.X()<0)
    {
        driveToCoordinate(curX + 1, curY - 1, MOTOR_SPEED);
        Sleep(1000);
    }
    turnToAngle(45);
    bicepHalfFlex();
    //Drive to garage and deposit wrench
    drivePolar(0,13.5,MOTOR_SPEED);
    bicepStretch();
    Sleep(1000);
    drivePolar(180,12.9,MOTOR_SPEED);
    Sleep(500);
    //DRIVE TO AND SPIN THE BOY
    int turnChoice = RPS.FuelType();
    if (turnChoice == 1){
        spin.SetDegree(0);
    }else{
        spin.SetDegree(180);
    }

    Sleep(1000);

    whereAmI();

    while(driveToCoordinate(21.1 + courseOffsetX,62.7 + courseOffsetY,MOTOR_SPEED))
    {
       Sleep(1000);
       whereAmI();
    }

    while(turnToAngle(45))
    {
        Sleep(1000);
        whereAmI();
    }

    while(driveToCoordinate(21.1 + courseOffsetX,62.7 + courseOffsetY,MOTOR_SPEED))
    {
       Sleep(1000);
       whereAmI();
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
    driveToCoordinateNew(14.8 + courseOffsetX, 55.7 + courseOffsetY, MOTOR_SPEED);
    driveToCoordinateNew(curX,curY - 10,MOTOR_SPEED);
    //Drive towards ramp
    driveToCoordinateNew(curX + 10,curY - 7,MOTOR_SPEED);
    //Turn and go backwards down the ramp
    turnToAngle(0);
    Sleep(1000);
    whereAmI();
    driveToCoordinateNew(curX,curY-18,MOTOR_SPEED);
    bicepFlex();
    turnCC(180);

    Sleep(1000);
    whereAmI();

    //Drive to starting box
    driveToCoordinateNew(curX-12,curY,MOTOR_SPEED);
    //End the run
    driveToCoordinateNew(curX,curY+25,MOTOR_SPEED);
    return 0;


}


#define STRETCH_POSITION 175

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

void bicepSlowFlex(int ms){
    long start = TimeNowMSec();
    while(TimeNowMSec() - start < ms)
    {
        bicep.SetDegree(STRETCH_POSITION-(TimeNowMSec()-start)*STRETCH_POSITION/ms);
    }

}

void whereAmI()
{
    curX = getRPSX();
    curY = getRPSY();
    curAngle = getRPSHeading();
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

    while((abs(BLPos) + abs(FRPos)) / 2 - abs(YEnd) < -allowableError || (abs(BRPos) + abs(FLPos)) / 2 - abs(XEnd) < -allowableError)
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


void drivePolarNew(float angle, float distance, float percent)
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

    float lastX = getRPSX();
    float lastY = getRPSY();
    bool correctionMade = false;

    int absoluteDirection = (int)(angle + curAngle) % 360;



    double lastTime = TimeNow();

    while((abs(BLPos) + abs(FRPos)) / 2 - abs(YEnd) < -allowableError || (abs(BRPos) + abs(FLPos) / 2 - abs(XEnd)) < -allowableError)
    {

        float slowdownFactorY = min(abs((abs(BLPos) + abs(FRPos)) / 2 - abs(YEnd)) / 12,1);
        float slowdownFactorX = min(abs((abs(BRPos) + abs(FLPos)) / 2 - abs(XEnd)) / 12,1);


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
            float RPSX = getRPSX();
            float RPSY = getRPSY();
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




                if(abs(angleError)<10)
                {
                    float a = cos(PI/180*absoluteDirection);
                    float b = sin(PI/180*absoluteDirection);
                    float c = -1*(a*curX+b*curY);
                    float distance = a * RPSX + b * RPSY + c;
                    distance *=-500/81;
                    YEnd += -cos(angle*PI/180)*distance;
                    XEnd += sin(angle*PI/180)*distance;

                    FLPredicted += sin(angle*PI/180)*distance;
                    FRPredicted += -cos(angle*PI/180)*distance;
                    BLPredicted += -cos(angle*PI/180)*distance;
                    BRPredicted += sin(angle*PI/180)*distance;
                    correctionMade = true;
                }
                lastX = RPSX;
                lastY = RPSY;
            }
        }



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
    curAngle = getRPSHeading();
}


// 0 is by garage
// 90 is wrench
// 180 is car

bool driveToCoordinate(float x, float y, float percent)
{
    SD.Printf("%f,%f\n",x,y);
    if(abs(x-curX) < .4 && abs(y-curY)<.4)
        return false;

    float angle = atan((y-curY)/(x-curX))*180/PI;
    if(x-curX < 0)
    {
        angle += 180;
    }

    angle = angle  - 90 -  curAngle;



    float distance = sqrt((y-curY)*(y-curY)+(x-curX)*(x-curX));

    drivePolar(angle, distance, percent);

    curX = x;
    curY = y;



	return true;

}

bool driveToCoordinateNew(float x, float y, float percent)
{
    SD.Printf("%f,%f\n",x,y);
    if(abs(x-curX) < .4 && abs(y-curY)<.4)
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


    drivePolarNew(angle, distance, percent);

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


    if(angleError > 1)
        turnCC(angleError);
    else if(angleError < -1)
        turnC(-angleError);
    else
        return false;

    curAngle = angle;
    return true;
}


void driveUpHill(float percent)
{
    percent = percent / 2;
    float XSpeed = cos(45*PI/180)* percent;
    float YSpeed = sin(45*PI/180)* percent;

    setFrontRightSpeed(YSpeed);
    setBackLeftSpeed(YSpeed);
    setFrontLeftSpeed(XSpeed);
    setBackRightSpeed(XSpeed);
    
    float accel = 0;
    
    bool touchedHill = false;
    while(!touchedHill || accel > .07)
    {
        accel = abs(Accel.Y());
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
    if(degrees<10)
    {
        setFrontLeftSpeed(-30);
        setFrontRightSpeed(30);
        setBackLeftSpeed(-30);
        setBackRightSpeed(30);
        Sleep(degrees/52.5);
    }
    else
    {
        int counts = degrees / 2.1;
        frontRightEncoder.ResetCounts();
        frontLeftEncoder.ResetCounts();
        backLeftEncoder.ResetCounts();
        backRightEncoder.ResetCounts();
        while(frontLeftEncoder.Counts() + frontRightEncoder.Counts() + backLeftEncoder.Counts() + backRightEncoder.Counts() < counts*4)
        {
            int offset = degrees - (frontLeftEncoder.Counts() + frontRightEncoder.Counts() + backLeftEncoder.Counts() + backRightEncoder.Counts())/4*2.1;
            float speed = 20+offset * 80/180;
            setFrontLeftSpeed(-speed);
            setFrontRightSpeed(speed);
            setBackLeftSpeed(-speed);
            setBackRightSpeed(speed);
        }
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
    curAngle = (int)(curAngle + degrees) % 360;

}
void turnC(float degrees)
{
    if(degrees<10)
    {
        setFrontLeftSpeed(30);
        setFrontRightSpeed(-30);
        setBackLeftSpeed(30);
        setBackRightSpeed(-30);
        Sleep(degrees/52.5);
    }
    else
    {
        int counts = degrees / 2.1;
        frontRightEncoder.ResetCounts();
        frontLeftEncoder.ResetCounts();
        backLeftEncoder.ResetCounts();
        backRightEncoder.ResetCounts();
        while(frontLeftEncoder.Counts() + frontRightEncoder.Counts() + backLeftEncoder.Counts() + backRightEncoder.Counts() < counts*4)
        {
            int offset = degrees - (frontLeftEncoder.Counts() + frontRightEncoder.Counts() + backLeftEncoder.Counts() + backRightEncoder.Counts())/4*2.1;
            float speed = 20+offset * 80/180;
            setFrontLeftSpeed(speed);
            setFrontRightSpeed(-speed);
            setBackLeftSpeed(speed);
            setBackRightSpeed(-speed);
        }
    }
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
    curAngle = (int)(curAngle - degrees) % 360;
}

void buttonDecision(int direction)
{
    turnToAngle(90);
    if(direction)
    {
        driveToCoordinate(curX-3,curY,MOTOR_SPEED);
        driveToCoordinateNew(curX, curY-3.2, MOTOR_SPEED/2);
        driveToCoordinate(curX+3,curY+2,MOTOR_SPEED);
        driveToCoordinate(curX,curY-2.5,MOTOR_SPEED);
    }
    else
    {
       driveToCoordinateNew(curX+3.5,curY,MOTOR_SPEED);
       driveToCoordinateNew(curX, curY-3.2, MOTOR_SPEED/2);
       driveToCoordinate(curX-3,curY+2,MOTOR_SPEED);
       driveToCoordinate(curX,curY-2.5,MOTOR_SPEED);
    }
    long startTime = TimeNowMSec();
    int trials = 0;
    while(RPS.IsDeadzoneActive() != 2 && trials < 3)
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
            frontLeft.Stop();
            frontRight.Stop();
            backLeft.Stop();
            backRight.Stop();
        }
        if (TimeNowMSec()-startTime >= 7000){
            whereAmI();
            driveToCoordinateNew(lightX + courseOffsetX, lightY + courseOffsetY, MOTOR_SPEED);
            turnToAngle(90);
            driveToCoordinateNew(curX, curY-3.2, MOTOR_SPEED/2);
            startTime = TimeNowMSec();
            trials++;
        }
    }
    whereAmI();
    driveToCoordinateNew(lightX + courseOffsetX, lightY + courseOffsetY, MOTOR_SPEED);
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
float getRPSX(){
    float val = RPS.X();
    while(val<0)
        val = RPS.X();
    return val;
}
float getRPSY(){
    float val = RPS.Y();
    while(val<0)
        val = RPS.Y();
    return val;
}
float getRPSHeading(){
    getRPSX();
    return RPS.Heading();
}
void RPSbutton(){
    LCD.Clear();
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
