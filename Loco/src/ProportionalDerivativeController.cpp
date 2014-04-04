//
//  ProportionalDerivativeController.cpp
//  Loco
//
//  Created by damian on 03/04/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#include "ProportionalDerivativeController.h"

// for IS_ZERO and SGN
#include "AnimationSharedStuff.h"


/**
 
 from SimBiCon
 
 This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
 Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
 the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque
 returned is expressed in the coordinate frame of the 'parent'.
 */
Ogre::Vector3 ProportionalDerivativeController::computePDTorque(const Ogre::Quaternion& qRel, const Ogre::Quaternion& qRelD, const Ogre::Vector3& wRel, const Ogre::Vector3& wRelD, double kp, double kd, double strength )
{
	
	Ogre::Vector3 torque(0,0,0);
	
	//	Quaternion qErr = qRel.getComplexConjugate() * qRelD;
	Ogre::Quaternion qErr = qRel.UnitInverse();
	qErr = qErr * qRelD;
	
	//qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
	Ogre::Vector3 qErrV( qErr.x, qErr.y, qErr.z );
	double sinTheta = qErrV.length();
	if (sinTheta>1)
		sinTheta = 1;
	if (IS_ZERO(sinTheta)){
		//avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
	}else{
		double absAngle = 2 * asin(sinTheta);
		torque = qErrV;
		torque *= 1/sinTheta * absAngle * (kp) * SGN(qErr.w);
		//		torque = qErr.v/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
	}
	
	//qErr represents the rotation from the desired child frame to the actual child frame, which
	//means that the torque is now expressed in child coordinates. We need to express it in parent coordinates!
	torque = qRel * torque;
	//the angular velocities are stored in parent coordinates, so it is ok to add this term now
	torque += (wRelD - wRel) * (kd);
	torque *= strength;
	
	return torque;
	
}


Ogre::Vector3 ProportionalDerivativeController::computePDForce( const Ogre::Vector3& p, const Ogre::Vector3& pD, const Ogre::Vector3& v, const Ogre::Vector3& vD, double kp, double kd )
{
	// PD: out = Kp * Err   +   Kd * dErr/dt
	Ogre::Vector3 force = kp*(pD-p);
	force += kd*(vD-v);
	
	return force;
	
}

