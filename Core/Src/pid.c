/*
 * pid.c
 */

#include "motors.h"
#include "encoders.h"
#include <math.h>

volatile float dbg_leftCounts;
volatile float dbg_rightCounts;
volatile float dbg_e_dist;
volatile float dbg_e_ang;
volatile float dbg_distCorr;
volatile float dbg_angCorr;
volatile float dbg_leftSpeed;
volatile float dbg_rightSpeed;

int angleError = 0;
int oldAngleError = 0;
float distanceError = 0.0;
float oldDistanceError = 0.0; //distanceError

int angFlag = 0;
int disFlag = 0;
int angEps = 25;
float distEps = 40;
float kPw = 0.0007;
float kDw = 0.00015;
float kPx = 0.005;
float kDx = 0.0006;
float kIw = 0.0;
float kIx = 0.0;
int count = 0;
static float sumAng  = 0.0;
static float sumDist = 0.0;

int withinBound = 0;

int goalAngle = 0;
int goalDistance = 0;
int8_t done = 0;

void resetPID(void) {
	goalDistance = 0;
	count = 0;
	angleError = 0;
	distanceError = 0;
	oldAngleError = 0;
	oldDistanceError = 0;
	sumAng  = 0.0f;
	sumDist = 0.0f;
	resetEncoders();
	resetMotors();
	done = 0;

}

void updatePID(float dt)  {

	int16_t leftEncoder  = getLeftEncoderCounts();
	int16_t rightEncoder = -getRightEncoderCounts();
//	resetEncoders();

	dbg_leftCounts = leftEncoder;
	dbg_rightCounts = rightEncoder;

	    // distance measure & error
	float distMeas = (leftEncoder + rightEncoder) * 0.5f;
	float e_dist   = goalDistance - distMeas;

	dbg_e_dist = e_dist;

	    // **use** the file‐scope accumulator here, not a new one
	sumDist       += e_dist * dt;
	float derr_dist = (e_dist - oldDistanceError) / dt;

	    // 2) angle measure & error
	float e_ang     = goalAngle - (leftEncoder - rightEncoder);
	    sumAng        += e_ang * dt;
	float derr_ang  = (e_ang - oldAngleError) / dt;

	dbg_e_ang = e_ang;

        // PID outputs with integral terms added
    float distCorr = kPx * e_dist + /* add:*/ kIx * sumDist + kDx * derr_dist;
    float angCorr  = kPw * e_ang  + /* add:*/ kIw * sumAng  + kDw * derr_ang;

    if (goalAngle == 0) {
    	angCorr = 0;
    } else {
    	if (angCorr > 0.55f) {
    	        	angCorr = 0.55f;
    	        } else if (angCorr < -0.55f) {
    	        	angCorr = -0.55f;
    	        }
    }

         if (goalDistance != 0) {
        	 if (fabs(e_dist) < distEps) {
        		 done = 1;
         }


//        	 if (angleError < 0) {
//        	     	 if (-1 * angleError < angEps) {
//        	     		 count++;
//        	     	 } else {
//        	     		 count = 0;
//        	     	 }
//        	      } else {
//        	     	 if (angleError < angEps) {
//        	     		 count++;
//        	     	 } else {
//        	     		 count = 0;
//        	     	 }
//        	      }

         }

         if (goalAngle != 0) {
                 	         	 if (fabs(e_dist) < angEps) {
                 	         		 done = 1;
                 	          }
                 	 }

    // change 1: commented out angCorr
//    float angCorr = 0.0f;

    dbg_distCorr = distCorr;
    dbg_angCorr = angCorr;

        // combine, clamp, send to motors
    float leftSpeed  = limitPWM(distCorr - angCorr);
    float rightSpeed = limitPWM(distCorr + angCorr);

    dbg_leftSpeed = leftSpeed;
    dbg_rightSpeed = rightSpeed;

    setMotorLPWM(leftSpeed);
    setMotorRPWM(rightSpeed);

        // 5) save for next
    oldDistanceError = e_dist;
    oldAngleError    = e_ang;

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

//    angleError = goalAngle - (leftEncoder - rightEncoder);
//
//    float angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
//    oldAngleError = angleError;
//    if (angleCorrection > 0.5f) {
//    	angleCorrection = 0.5f;
//    } else if (angleCorrection < -0.5f) {
//    	angleCorrection = -0.5f;
//    }
//
//    distanceError = goalDistance - (rightEncoder + leftEncoder) / 2.0;
//
//    float distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
//    oldDistanceError = distanceError;
//
//    // capping distance
//    if (distanceCorrection > 0.5f) {
//    	distanceCorrection = 0.5f;
//    } else if (distanceCorrection < -0.5f) {
//    	distanceCorrection = -0.5f;
//    }
//
//
//    float leftSpeed = distanceCorrection + angleCorrection;
//    float rightSpeed = distanceCorrection - angleCorrection;

//     float leftSpeed = angleCorrection;
//     float rightSpeed = -angleCorrection;

//    if (angleError < bound && distanceError < bound) {
//    	withinBound = 1;
//    } else {
//    	withinBound = 0;
//    }

    // capping angle
//     if (goalAngle != 0 || goalDistance != 0) {
//    	 if (fabs(angleError) < angEps && fabs(distanceError) < distEps) {
//    		 done = 1;
//    	 }
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


//    setMotorLPWM(leftSpeed);
//    setMotorRPWM(rightSpeed);
//

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
