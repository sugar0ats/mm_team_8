/*
 * pid.c
 */

#include "motors.h"
#include "encoders.h"
#include <math.h>


int angleError = 0;
int oldAngleError = 0;
float distanceError = 0.0;
float oldDistanceError = 0.0; //distanceError

int angFlag = 0;
int disFlag = 0;
int angEps = 5;
float distEps = 3;
float kPw = 0.03;
float kDw = 0.0;
float kPx = 0.5;
float kDx = 0.0;
int count = 0;

int withinBound = 0;

int goalAngle = 0;
int goalDistance = 0;
int8_t done = 0;

void resetPID(void) {
	count = 0;
	angleError = 0;
	distanceError = 0;
	oldAngleError = 0;
	oldDistanceError = 0;
	resetEncoders();
	resetMotors();
	done = 0;

}

void updatePID() {

    int leftEncoder = getLeftEncoderCounts();
    int rightEncoder = getRightEncoderCounts();


//    if (leftEncoder < 0) {
//    	leftEncoder = -leftEncoder;
//    }
//    if (rightEncoder < 0) {
//    	rightEncoder = -rightEncoder;
//    }

//    if (goalAngle < 0) {
//    	angleError = goalAngle + (rightEncoder + leftEncoder);
//    } else {
//    	angleError = goalAngle - (rightEncoder + leftEncoder);
//    }

    angleError = goalAngle - (leftEncoder - rightEncoder);

    float angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
    oldAngleError = angleError;
    if (angleCorrection > 0.5f) {
    	angleCorrection = 0.5f;
    } else if (angleCorrection < -0.5f) {
    	angleCorrection = -0.5f;
    }

    distanceError = goalDistance - (rightEncoder + leftEncoder) / 2.0;

    float distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
    oldDistanceError = distanceError;

    // capping distance
    if (distanceCorrection > 0.5f) {
    	distanceCorrection = 0.5f;
    } else if (distanceCorrection < -0.5f) {
    	distanceCorrection = -0.5f;
    }


    float leftSpeed = distanceCorrection + angleCorrection;
    float rightSpeed = distanceCorrection - angleCorrection;

//     float leftSpeed = angleCorrection;
//     float rightSpeed = -angleCorrection;

//    if (angleError < bound && distanceError < bound) {
//    	withinBound = 1;
//    } else {
//    	withinBound = 0;
//    }

    // capping angle
     if (goalAngle != 0 || goalDistance != 0) {
    	 if (fabs(angleError) < angEps && fabs(distanceError) < distEps) {
    		 done = 1;
    	 }
//    	 if (angleError < 0) {
//    	     	 if (-1 * angleError < angEps) {
//    	     		 count++;
//    	     	 } else {
//    	     		 count = 0;
//    	     	 }
//    	      } else {
//    	     	 if (angleError < angEps) {
//    	     		 count++;
//    	     	 } else {
//    	     		 count = 0;
//    	     	 }
//    	      }

     }


    setMotorLPWM(leftSpeed);
    setMotorRPWM(rightSpeed);
}

void setPIDGoalD(int16_t distance) {
	/*
	 * For assignment 3.1: this function does not need to do anything.
	 * For assignment 3.2: this function should set a variable that stores the goal distance.
	 */
	goalDistance = distance;
}

void setPIDGoalA(int16_t angle) {
	/*
	 * For assignment 3.1: this function does not need to do anything
	 * For assignment 3.2: This function should set a variable that stores the goal angle.
	 */
	goalAngle = angle;
}

int8_t PIDdone(void) { // There is no bool type in C. True/False values are represented as 1 or 0.
	/*
	 * For assignment 3.1: this function does not need to do anything (your rat should just drive straight indefinitely)
	 * For assignment 3.2:This function should return true if the rat has achieved the set goal. One way to do this by having updatePID() set some variable when
	 * the error is zero (realistically, have it set the variable when the error is close to zero, not just exactly zero). You will have better results if you make
	 * PIDdone() return true only if the error has been sufficiently close to zero for a certain number, say, 50, of SysTick calls in a row.
	 */

//	angErrorCopy = angleError;
//	distErrorCopy = distanceError;

//	if (angErrorCopy < 0) {
//		angErrorCopy *= -1;
//	}
//	if (distErrorCopy < 0) {
//		distErrorCopy *= -1;

//	}

	return done;
}
