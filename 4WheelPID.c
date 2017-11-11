#pragma config(Sensor, dgtl1,  lift,           sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  backRight,      sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  backLeft,       sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  frontRight,     sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  frontLeft,      sensorQuadEncoder)
#pragma config(Motor,  port2,           backRight,     tmotorVex393HighSpeed_MC29, openLoop, driveRight)
#pragma config(Motor,  port3,           backLeft,      tmotorVex393HighSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port4,           claw,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           leftLift,      tmotorVex393HighSpeed_MC29, openLoop, driveLeft)
#pragma config(Motor,  port6,           rightLift,     tmotorVex393HighSpeed_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port7,           mogoLift,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           frontRight,    tmotorVex393HighSpeed_MC29, openLoop, driveRight)
#pragma config(Motor,  port9,           frontLeft,     tmotorVex393HighSpeed_MC29, openLoop, driveLeft)

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"


#define PID_INTEGRAL_LIMIT  50
#define PID_DRIVE_MAX       127
#define PID_DRIVE_MIN     (-127)

static float pidRequestedFrontLeft, pidRequestedFrontRight, pidRequestedBackLeft, pidRequestedBackRight;
static float Kp = .55;
static float Ki = 0;
static float Kd = 0;
static float leftDriveScale = 1; //Multiply faster motor by this so speed is equal with slower motor
static float rightDriveScale = 1;
static bool pidActive = true;

void pre_auton()
{
  bStopTasksBetweenModes = false;
}

task pidController()
{
	//Declare variables
	float  pidSensorCurrentFrontLeft, pidSensorCurrentFrontRight, pidSensorCurrentBackLeft, pidSensorCurrentBackRight; //Current Value
	float  pidErrorFrontLeft, pidErrorFrontRight, pidErrorBackLeft, pidErrorBackRight; //Current Error
	float  pidLastErrorFrontLeft, pidLastErrorFrontRight, pidLastErrorBackLeft, pidLastErrorBackRight; //Last Error
	float  pidIntegralFrontLeft, pidIntegralFrontRight, pidIntegralBackLeft, pidIntegralBackRight; //Integral
	float  pidDerivativeFrontLeft, pidDerivativeFrontRight, pidDerivativeBackLeft, pidDerivativeBackRight; //Derivative
	float  pidDriveFrontLeft, pidDriveFrontRight, pidDriveBackLeft, pidDriveBackRight; //Drive setting

	//Clear sensor values and initialize variables
	SensorValue[frontRight] = 0;
	SensorValue[frontLeft] = 0;
	SensorValue[backRight] = 0;
	SensorValue[backLeft] = 0;

	pidErrorFrontLeft = 0;
	pidErrorFrontRight = 0;
	pidErrorBackRight = 0;
	pidErrorBackRight = 0;

	pidLastErrorFrontLeft = 0;
	pidLastErrorFrontRight = 0;
	pidLastErrorBackRight = 0;
	pidLastErrorBackRight = 0;

	pidIntegralFrontLeft = 0;
	pidIntegralFrontRight = 0;
	pidIntegralBackLeft = 0;
	pidIntegralBackRight = 0;


	pidDerivativeFrontLeft = 0;
	pidDerivativeFrontRight = 0;
	pidDerivativeBackLeft = 0;
	pidDerivativeBackRight = 0;

	while(true)
	{
		if(pidActive)
		{
			//Calculate proportional portion
			pidSensorCurrentBackLeft = SensorValue[backLeft];
			pidSensorCurrentBackRight = SensorValue[backRight];
			pidSensorCurrentFrontLeft = SensorValue[frontLeft];
			pidSensorCurrentFrontRight = SensorValue[frontRight];

			pidErrorFrontLeft = pidSensorCurrentFrontLeft - pidRequestedFrontLeft;
			pidErrorFrontRight = pidSensorCurrentFrontRight - pidRequestedFrontRight;
			pidErrorBackLeft = pidSensorCurrentBackLeft - pidRequestedBackLeft;
			pidErrorBackRight = pidSensorCurrentBackRight - pidRequestedBackRight;

			// Calculate integral portion
			// Add the error to the integral if it is outside the controllable range of values
			if(abs(pidErrorFrontLeft) < PID_INTEGRAL_LIMIT)
				pidIntegralFrontLeft = pidIntegralFrontLeft + pidErrorFrontLeft;
			else
				pidIntegralFrontLeft = 0;

			if(abs(pidErrorFrontRight) < PID_INTEGRAL_LIMIT)
				pidIntegralFrontRight = pidIntegralFrontRight + pidErrorFrontRight;
			else
				pidIntegralFrontRight = 0;

			if(abs(pidErrorBackLeft) < PID_INTEGRAL_LIMIT)
				pidIntegralBackLeft = pidIntegralBackLeft + pidErrorBackLeft;
			else
				pidIntegralBackLeft = 0;

			if(abs(pidErrorBackRight) < PID_INTEGRAL_LIMIT)
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
			motor[frontLeft] = (int)((float)pidDriveFrontLeft * (-1.0)*leftDriveScale);
			motor[frontRight] = (int)((float)pidDriveFrontRight * rightDriveScale);
			motor[backLeft] = (int)((float)pidDriveBackLeft * (-1.0)*leftDriveScale);
			motor[backRight] = (int)((float)pidDriveBackRight * rightDriveScale);
		}
		else // delete the emails, i mean variables
		{
			pidSensorCurrentFrontLeft = 0;
			pidSensorCurrentFrontRight = 0;
			pidSensorCurrentBackLeft = 0;
			pidSensorCurrentBackRight = 0;
			pidErrorFrontLeft = 0;
			pidErrorFrontRight = 0;
			pidErrorBackLeft = 0;
			pidErrorBackRight = 0;
			pidLastErrorFrontLeft = 0;
			pidLastErrorFrontRight = 0;
			pidLastErrorBackLeft = 0;
			pidLastErrorBackRight = 0;
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
		}

	}
}

task autonomous()
{
		startTask(pidController);
}


task usercontrol()
{

	while(true)
	{
		//Split Arcade
		motor[backLeft]  = (vexRT[Ch1] + vexRT[Ch3]);
		motor[frontLeft] = (vexRT[Ch1] + vexRT[Ch3]);
		motor[backRight] = (vexRT[Ch1] - vexRT[Ch3]);
		motor[frontRight] = (vexRT[Ch1] - vexRT[Ch3]);

		//Control Lift
		if(vexRT[Btn5U] == 1)
		{
			motor[leftLift] = 127;
			motor[rightLift] = 127;
		}
		else
		if(vexRT[Btn5D] == 1)
		{
			motor[leftLift] = -127;
			motor[rightLift] = -127;
		}
		else
		{
			motor[leftLift] = -20;
			motor[rightLift] = -20;
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
			motor[claw] = 0;
		}

		//Mobile goal lift
		if(vexRT [Btn8U] == 1)
		{
			motor[mogoLift] = -127;
		}
		else
		if (vexRT [Btn8D] == 1)
		{
			motor[mogoLift] = 127;
		}
		else
		{
			motor [mogoLift] = 0;
		}

	}

}
