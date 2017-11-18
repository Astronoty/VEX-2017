#pragma config(Sensor, dgtl3,  liftEnc,        sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  backLeftEnc,    sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  frontRightEnc,  sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  frontLeftEnc,   sensorQuadEncoder)
#pragma config(Sensor, dgtl11, backRightEnc,   sensorQuadEncoder)
#pragma config(Motor,  port2,           backRight,     tmotorVex393HighSpeed_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port3,           backLeft,      tmotorVex393HighSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port4,           claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           leftLift,      tmotorVex393HighSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port6,           rightLift,     tmotorVex393HighSpeed_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port7,           mogoLift,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           frontRight,    tmotorVex393HighSpeed_MC29, openLoop, driveRight)
#pragma config(Motor,  port9,           frontLeft,     tmotorVex393HighSpeed_MC29, openLoop, reversed, driveLeft)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

void pre_auton()
{
  bStopTasksBetweenModes = true;

}
#define PID_INTEGRAL_LIMIT  50
#define PID_DRIVE_MAX       127
#define PID_DRIVE_MIN     (-127)

static float pidRequestedLeft;
static float pidRequestedRight;
static float pidRequestedLift;
static float Kp = 1;
static float Ki = 0;
static float Kd = 0;
static float leftDriveScale = 1; //Multiply faster motor by this so speed is equal with slower motor
static float rightDriveScale = 1;
static bool pidActive = true;

static float KpLift = 2;

//FUCK YOU AND YOUR BULLSHIT NSA I FUCKING KNOW MY CODE WORKS AND YOU'RE MAKING ME INSUCURE ABOUT MY SHIT
//STOP BEING FUCKING ASSHOLES AND SETTING MY BACKRIGHT ENCODER TO 1000 CONSTANTLY I LITERALLY JUST CHECKED
//THIS SHIT IN ANOTHER PROGRAM AND THE ENCODER FUCKING WORKS
//GO TROLL SOME OTHER PROGRAMMER YOU FAT CUNTS

task pidController()
{
	//Declare variables
	float  pidSensorCurrentFrontLeft, pidSensorCurrentFrontRight, pidSensorCurrentBackLeft, pidSensorCurrentBackRight, pidSensorCurrentLift; //Current Value
	float  pidErrorFrontLeft, pidErrorFrontRight, pidErrorBackLeft, pidErrorBackRight, pidErrorLift; //Current Error
	float  pidLastErrorFrontLeft, pidLastErrorFrontRight, pidLastErrorBackLeft, pidLastErrorBackRight, pidLastErrorLift; //Last Error
	float  pidIntegralFrontLeft, pidIntegralFrontRight, pidIntegralBackLeft, pidIntegralBackRight, pidIntegralLift; //Integral
	float  pidDerivativeFrontLeft, pidDerivativeFrontRight, pidDerivativeBackLeft, pidDerivativeBackRight, pidDerivativeLift; //Derivative
	float  pidDriveFrontLeft, pidDriveFrontRight, pidDriveBackLeft, pidDriveBackRight, pidDriveLift; //Drive setting

	//Clear sensor values and initialize variables
	SensorValue[frontRightEnc] = 0;
	SensorValue[frontLeftEnc] = 0;
	SensorValue[backRightEnc] = 0;
	SensorValue[backLeftEnc] = 0;
	SensorValue[liftEnc] = 0;

	pidErrorFrontLeft = 0;
	pidErrorFrontRight = 0;
	pidErrorBackLeft = 0;
	pidErrorBackRight = 0;
	pidErrorLift = 0;

	pidLastErrorFrontLeft = 0;
	pidLastErrorFrontRight = 0;
	pidLastErrorBackLeft = 0;
	pidLastErrorBackRight = 0;
	pidLastErrorLift = 0;

	pidIntegralFrontLeft = 0;
	pidIntegralFrontRight = 0;
	pidIntegralBackLeft = 0;
	pidIntegralBackRight = 0;
	pidIntegralLift = 0;


	pidDerivativeFrontLeft = 0;
	pidDerivativeFrontRight = 0;
	pidDerivativeBackLeft = 0;
	pidDerivativeBackRight = 0;
	pidDerivativeLift = 0;

	while(true)
	{
		if(pidActive)
		{
			//Calculate proportional portion
			pidSensorCurrentBackLeft = SensorValue[backLeftEnc];
			pidSensorCurrentBackRight = SensorValue[backRightEnc];
			pidSensorCurrentFrontLeft = SensorValue[frontLeftEnc];
			pidSensorCurrentFrontRight = SensorValue[frontRightEnc];
			pidSensorCurrentLift = SensorValue[liftEnc];

			pidErrorFrontLeft = pidSensorCurrentFrontLeft - pidRequestedLeft;
			pidErrorFrontRight = pidSensorCurrentFrontRight - pidRequestedRight;
			pidErrorBackLeft = pidSensorCurrentBackLeft - pidRequestedLeft;
			pidErrorBackRight = pidSensorCurrentBackRight - pidRequestedRight;
			pidErrorLift = pidSensorCurrentLift - pidRequestedLift;

			// Calculate integral portion
			// Add the error to the integral if it is outside the controllable range of values
			if(fabs(pidErrorFrontLeft) < PID_INTEGRAL_LIMIT)
				pidIntegralFrontLeft = pidIntegralFrontLeft + pidErrorFrontLeft;
			else
				pidIntegralFrontLeft = 0;

			if(fabs(pidErrorFrontRight) < PID_INTEGRAL_LIMIT)
				pidIntegralFrontRight = pidIntegralFrontRight + pidErrorFrontRight;
			else
				pidIntegralFrontRight = 0;

			if(fabs(pidErrorBackLeft) < PID_INTEGRAL_LIMIT)
				pidIntegralBackLeft = pidIntegralBackLeft + pidErrorBackLeft;
			else
				pidIntegralBackLeft = 0;

			if(fabs(pidErrorBackRight) < PID_INTEGRAL_LIMIT)
				pidIntegralBackRight = pidIntegralBackRight + pidErrorBackRight;
			else
				pidIntegralBackRight = 0;

			// Calculate derivitave portion
			pidDerivativeFrontLeft = pidErrorFrontLeft - pidLastErrorFrontLeft;
			pidDerivativeFrontRight = pidErrorFrontRight - pidLastErrorFrontRight;
			pidDerivativeBackLeft = pidErrorBackLeft - pidLastErrorBackLeft;
			pidDerivativeBackRight = pidErrorBackRight - pidLastErrorBackRight;

			pidLastErrorFrontLeft  = pidErrorFrontLeft;
			pidLastErrorFrontRight = pidErrorFrontRight;
			pidLastErrorBackLeft  = pidErrorBackLeft;
			pidLastErrorBackRight = pidErrorBackRight;

			// calculate drive
			pidDriveFrontLeft = (Kp * pidErrorFrontLeft) + (Ki * pidIntegralFrontLeft) + (Kd * pidDerivativeFrontLeft);
			pidDriveFrontRight = (Kp * pidErrorFrontRight) + (Ki * pidIntegralFrontRight) + (Kd * pidDerivativeFrontRight);
			pidDriveBackLeft = (Kp * pidErrorBackLeft) + (Ki * pidIntegralBackLeft) + (Kd * pidDerivativeBackLeft);
			pidDriveBackRight = (Kp * pidErrorBackRight) + (Ki * pidIntegralBackRight) + (Kd * pidDerivativeBackRight);
			pidDriveLift = (KpLift*pidErrorLift); //Remember to add integral and derivative if desired

			// limit drive
			if( pidDriveFrontLeft > PID_DRIVE_MAX )
				pidDriveFrontLeft = PID_DRIVE_MAX;
			if( pidDriveFrontLeft < PID_DRIVE_MIN )
				pidDriveFrontLeft = PID_DRIVE_MIN;

			if( pidDriveBackLeft > PID_DRIVE_MAX )
				pidDriveBackLeft = PID_DRIVE_MAX;
			if( pidDriveBackLeft < PID_DRIVE_MIN )
				pidDriveBackLeft = PID_DRIVE_MIN;

			if( pidDriveFrontRight > PID_DRIVE_MAX )
				pidDriveFrontRight = PID_DRIVE_MAX;
			if( pidDriveFrontRight < PID_DRIVE_MIN )
				pidDriveFrontRight = PID_DRIVE_MIN;

			if( pidDriveBackRight > PID_DRIVE_MAX )
				pidDriveBackRight = PID_DRIVE_MAX;
			if( pidDriveBackRight < PID_DRIVE_MIN )
				pidDriveBackRight = PID_DRIVE_MIN;


			// DRIVE BOI DRIVE
			motor[frontLeft] = (int)((float)pidDriveFrontLeft * leftDriveScale);
			motor[frontRight] = (int)((float)pidDriveFrontRight * rightDriveScale);
			motor[backLeft] = (int)((float)pidDriveBackLeft * (-1.0)*leftDriveScale);
			motor[backRight] = (int)((float)pidDriveBackRight * (-1.0)*rightDriveScale);
			motor[leftLift] = (int)((float)pidDriveLift * (-1.0));
			motor[rightLift] = (int)((float)pidDriveLift * (-1.0));

			writeDebugStreamLine("PidError Back Right is: %f", pidErrorBackRight);
				writeDebugStreamLine("PidError Back Left is: %f", pidErrorBackLeft);
					writeDebugStreamLine("PidError Front Right is: %f", pidErrorFrontRight);
						writeDebugStreamLine("PidError Front Left is: %f", pidErrorFrontLeft);
			}
		else // delete the emails, i mean variables
		{
			pidSensorCurrentFrontLeft = 0;
			pidSensorCurrentFrontRight = 0;
			pidSensorCurrentBackLeft = 0;
			pidSensorCurrentBackRight = 0;
			pidSensorCurrentLift = 0;
			pidErrorFrontLeft = 0;
			pidErrorFrontRight = 0;
			pidErrorBackLeft = 0;
			pidErrorBackRight = 0;
			pidErrorLift = 0;
			pidLastErrorFrontLeft = 0;
			pidLastErrorFrontRight = 0;
			pidLastErrorBackLeft = 0;
			pidLastErrorBackRight = 0;
			pidLastErrorLift = 0;
			pidIntegralFrontLeft = 0;
			pidIntegralFrontRight = 0;
			pidIntegralBackLeft = 0;
			pidIntegralBackRight = 0;
			pidDerivativeFrontLeft = 0;
			pidIntegralFrontRight = 0;
			pidDerivativeBackLeft = 0;
			pidIntegralBackRight = 0;
			pidDriveFrontLeft= 0;
			pidDriveFrontRight = 0;
			pidDriveBackLeft= 0;
			pidDriveBackRight = 0;
			motor[frontLeft] = 0;
			motor[frontRight] = 0;
			motor[backLeft] = 0;
			motor[backRight] = 0;
			motor[leftLift] = 0;
			motor[rightLift] = 0;
		}

	}
}

void drive(int anglesLeft, int anglesRight)
{
	SensorValue[frontLeftEnc] = 0;
	SensorValue[frontRightEnc] = 0;
	SensorValue[backLeftEnc] = 0;
	SensorValue[backRightEnc] = 0;


	pidRequestedLeft = anglesLeft;
	pidRequestedRight = anglesRight;


}

void setLift(int position)
{
	pidRequestedLift = position;
}

task autonomous()
{
	/*
	startTask(pidController);
	motor[claw] = -100;
	setLift(105);
	drive(-1550,-1550);
	wait1Msec(3000);
	motor[mogoLift] = -127;
	wait1Msec(1500);
	motor[mogoLift] = 0;

	motor[claw] = 127;
	wait1Msec(400);
	motor[claw] = 0;
	wait1Msec(1000);

	drive(1000,1000);
	wait1Msec(2000);
	drive(600,-600);
	wait1Msec(1500);
	drive(-1100,-1500);
	wait1Msec(3000);
	drive(-600,-600);
	wait1Msec(800);
	setLift(80);
	motor[goalPusher] = 127;
	wait1Msec(3000);
	motor[goalPusher] = -127;
	drive(800,800);
	wait1Msec(3000);
	motor[goalPusher] = 0;
  stopTask(pidController);
  */

  	startTask(pidController);
	motor[claw] = -100;
	setLift(60);
	drive(-1150, -1150);
	wait1Msec(3000);
	motor[claw] = 127;
	wait1Msec(400);
	motor[claw] = 0;
	wait1Msec(400);
	motor[claw] = -127;
	wait1Msec(400);
	motor[claw] = 0;
	wait1Msec(500);
	drive(-350, -350);
	wait1Msec(1000);
	motor[mogoLift] = -127;
	wait1Msec(1500);
	motor[mogoLift] = 0;

	wait1Msec(1000);

	drive(1000,1000);
	wait1Msec(2000);
	drive(600,-600);
	wait1Msec(1500);
	drive(-1100,-1500);
	wait1Msec(3000);
	drive(-600,-600);
	wait1Msec(800);
	setLift(80);
	motor[goalPusher] = 127;
	wait1Msec(3000);
	motor[goalPusher] = -127;
	drive(800,800);
	wait1Msec(3000);
	motor[goalPusher] = 0;
  stopTask(pidController);
}


task usercontrol()
{
	while(true)
		{
			//Split Arcade
			motor[backLeft]  = (vexRT[Ch3] + vexRT[Ch1]);
			motor[frontLeft] = (-vexRT[Ch3] - vexRT[Ch1] );
			motor[backRight] = (vexRT[Ch3] - vexRT[Ch1] );
			motor[frontRight] = (-vexRT[Ch3] + vexRT[Ch1]);

			//Control Lift
			if(vexRT[Btn5U] == 1)
			{
				motor[leftLift] = -127;
				motor[rightLift] = -127;
			}
			else
			if(vexRT[Btn5D] == 1)
			{
				motor[leftLift] = 127;
				motor[rightLift] = 127;
			}
			else
			{
				if(SensorValue[liftEnc] < 140)
				{
					motor[leftLift] = 20;
					motor[rightLift] = 20;
				}

				else
				{
					motor[leftLift] = -20;
					motor[rightLift] = -20;
				}
			}

			if(vexRT[Btn7U])
			{
				if(SensorValue[liftEnc] > 115)
				{
					motor[leftLift] = -127;
					motor[rightLift] = -127;
				}
				else
				{
					motor[leftLift] = 20;
					motor[rightLift] = 20;
				}
			}
			//Claw Control
			if(vexRT[Btn6U] == 1)
			{
				motor[claw] = 127;
			}
			else
			if(vexRT[Btn6D] == 1)
			{
				motor[claw] = -127;
			}
			else
			{
				motor[claw] = -70;
			}

			//Mobile goal lift
			if(vexRT[Btn8U] == 1)
			{
				motor[mogoLift] = -127;
			}
			else
			if (vexRT[Btn8D] == 1)
			{
				motor[mogoLift] = 127;
			}
			else
			{
				motor[mogoLift] = 0;
			}

			//Mobile goal pusher
			if(vexRT[Btn8R])
			{
				motor[goalPusher] = 127;
			}
			else
			if(vexRT[Btn8L])
			{
				motor[goalPusher] = -127;
			}
			else
			{
				motor[goalPusher] = 0;
			}
		}


}
