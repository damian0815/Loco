//
//  BoneDriverPD.cpp
//  Loco
//
//  Created on 17/10/13.
//
//

#include "BoneDriverPD.h"
#include <Ogre/OgrePrerequisites.h>
#include <Ogre/OgreVector3.h>
#include <Ogre/OgreBone.h>
#include "Euler.h"
#include "Utilities.h"


BoneDriverPD::BoneDriverPD( Ogre::SharedPtr<DriveableBone> bone, float strength )
: mBone(bone), mStrength(strength), mPositionActive(false), mOrientationActive(false)
{
}

void BoneDriverPD::setTargetOrientation( const Ogre::Quaternion& target )
{
	mTargetOrientation = target;
	mTargetOrientation.normalise();
	mOrientationActive = true;
}

void BoneDriverPD::setTargetPosition( const Ogre::Vector3& target )
{
	mTargetPosition = target;
	mPositionActive = true;
}


/**
 
 from SimBiCon
 
	This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
	Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
	the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque 
	returned is expressed in the coordinate frame of the 'parent'.
 */
static Ogre::Vector3 computePDTorque(const Ogre::Quaternion& qRel, const Ogre::Quaternion& qRelD, const Ogre::Vector3& wRel, const Ogre::Vector3& wRelD, double kp, double kd, double strength )
{
	
	Ogre::Vector3 torque;
	
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


void BoneDriverPD::updateTorque()
{
	if ( !mOrientationActive )
		return;
	
	// current orientation in parent's frame
	Ogre::Quaternion qRel = mBone->getBone()->getOrientation();
	/*Ogre::Real length = */qRel.normalise();
	
	// desired orientation in parent's frame
	Ogre::Quaternion qRelD = mTargetOrientation;
	
	// current angular velocity in parent's frame
	Ogre::Vector3 wRel = mBone->getAngularVelocity();
	
	// desired angular velocity in parent's frame
	// zero for now
	Ogre::Vector3 wRelD = Ogre::Vector3::ZERO;
	
	//BLog("%15s: ", mBone->getBone()->getName().c_str());
	
	//Ogre::Euler torque = computePDTorque(qRel, qRelD, wRel, wRelD, mBone->getKp(), mBone->getKd(), mStrength);
	Ogre::Vector3 torqueV3 = computePDTorque(qRel, qRelD, wRel, wRelD, mBone->getKp(), mBone->getKd(), mStrength);
	
	/*if ( mBone->getBone()->getName() == "LegUpper.R" ) {
		BLog("%15s: qRel %s wRel %s", mBone->getBone()->getName().c_str(), describe(qRel).c_str(), describe(wRel).c_str() );
	}*/
	
	// apply the torque
	//Ogre::Vector3 torqueV3( torque.getYaw().valueRadians(), torque.getPitch().valueRadians(), torque.getRoll().valueRadians() );
	mBone->addTorque(torqueV3);
	
}

void BoneDriverPD::updateForce()
{
	if ( !mPositionActive )
		return;
		
	// position (world)
	Ogre::Vector3 p = mBone->getBone()->_getDerivedPosition();
	// target position
	Ogre::Vector3 pD = mTargetPosition;
	// velocity (local)
	Ogre::Vector3 v = mBone->getVelocity();
	// target velocity
	Ogre::Vector3 vD = Ogre::Vector3::ZERO;
	
	// PD: out = Kp*Err +  Kd * dErr/dt
	Ogre::Vector3 force = mBone->getKp()*(pD-p);
	//BLog("current pos %s, target %s, force %s", describe(p).c_str(), describe(pD).c_str(), describe(force).c_str() );
	// convert to local space
	Ogre::Node* parent = mBone->getBone()->getParent();
	if ( parent ) {
		force = parent->_getDerivedOrientation().Inverse()*force;
	}
	
	force += mBone->getKd() * (vD-v);
	//BLog(" after applying Kd, force %s", describe(force).c_str());
	
	
	// add to the bone
	mBone->addForce(force);
}

