//
//  ForwardDynamicsJointDriverPD.cpp
//  Loco
//
//  Created on 13/11/13.
//
//

#include "ForwardDynamicsJointDriverPD.h"
#include <Ogre/OgrePrerequisites.h>
#include <Ogre/OgreVector3.h>
#include <Ogre/OgreBone.h>
#include "Euler.h"
#include "Utilities.h"
#include "OgreBulletDynamicsRigidBody.h"
#include "OgreBulletConverter.h"
#include "OgreBulletCollisionsDebugLines.h"

using namespace OgreBulletDynamics;
using namespace OgreBulletCollisions;

ForwardDynamicsJointDriverPD::ForwardDynamicsJointDriverPD( Ogre::SharedPtr<ForwardDynamicsJoint> joint, float strength )
: mJoint(joint), mStrength(strength), mOrientationActive(false), mAngularVelocityActive(false)
{
}

void ForwardDynamicsJointDriverPD::setTargetOrientationWorld( const Ogre::Quaternion& target )
{
	mTargetOrientation = target;
	mTargetOrientation.normalise();
	mOrientationActive = true;
}

void ForwardDynamicsJointDriverPD::setTargetAngularVelocityWorld( const Ogre::Vector3& target )
{
	mTargetAngularVelocity = target;
	mAngularVelocityActive = true;
}

/**
 
 from SimBiCon
 
 This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
 Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
 the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque
 returned is expressed in the coordinate frame of the 'parent'.
 */
Ogre::Vector3 ForwardDynamicsJointDriverPD::computePDTorque(const Ogre::Quaternion& qRel, const Ogre::Quaternion& qRelD, const Ogre::Vector3& wRel, const Ogre::Vector3& wRelD, double kp, double kd, double strength )
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


void ForwardDynamicsJointDriverPD::updateTorque()
{
	if ( !mOrientationActive ) {
		return;
	}
	
	auto parent = mJoint->getParentFdb();
	auto child = mJoint->getChildFdb();
	
	// current orientation in local frame
	Ogre::Quaternion qRel = child->getOrientationWorld();
	OgreAssert(fabsf((qRel.w*qRel.w+qRel.x*qRel.x+qRel.y*qRel.y+qRel.z*qRel.z)-1.0) < 0.0001, "qRel is not normalized");
	//qRel.normalise();
	// desired orientation in local frame
	Ogre::Quaternion qRelD = mTargetOrientation;
	
	// current angular velocity in parent's frame
	//Ogre::Vector3 wRel = parent->getOrientationWorld().Inverse() * BtOgreConverter::to( child->getBody()->getBulletRigidBody()->getAngularVelocity());
	Ogre::Vector3 wRel = BtOgreConverter::to( child->getBody()->getBulletRigidBody()->getAngularVelocity());
	// desired angular velocity in parent's frame
	Ogre::Vector3 wRelD = (mAngularVelocityActive ? mTargetAngularVelocity : Ogre::Vector3::ZERO);
	
	Ogre::Vector3 torqueV3 = computePDTorque(qRel, qRelD, wRel, wRelD, child->getKp(), child->getKd(), mStrength);
	
	if ( child->getName() == "LegUpper.L" ) {
		BLog("%15s: qRel %s wRel %s", child->getName().c_str(), describe(qRel).c_str(), describe(wRel).c_str() );
	}
	
	// torque is already in world space
	mJoint->addTorque(torqueV3);
	
}

void ForwardDynamicsJointDriverPD::debugDraw( OgreBulletCollisions::DebugLines* debugLines )
{
	if ( getOrientationActive() )
	{
		//const Ogre::ColourValue color = Ogre::ColourValue(0.2,0.5,0.5);
		const Ogre::ColourValue targetColor = Ogre::ColourValue(1.0,1.0,0.5);
		Ogre::Vector3 offs(0,0,0.1);
		auto headPos = mJoint->getPositionWorld();
		//auto tailPos = mJoint->getTailPositionWorld();
		auto tailOffset = getTargetOrientation()*Ogre::Vector3::UNIT_Y;
		OgreAssert( fabsf(tailOffset.squaredLength()-1.0) < 0.0001, "bad" );
		auto tailPosTarget = headPos + tailOffset;
		
		// convert to debugLines space
		headPos = debugLines->convertWorldToLocalPosition(headPos+offs);
		//tailPos = debugLines->convertWorldToLocalPosition(tailPos+offs);
		tailPosTarget = debugLines->convertWorldToLocalPosition(tailPosTarget+offs);
		
		//debugLines->addLine(headPos, tailPos, color);
		debugLines->addLine(headPos, tailPosTarget, targetColor);
		/*auto tailPos = headPos + body->getBody()->getSceneNode()->convertLocalToWorldOrientation(it.second->getTargetOrientation())*Ogre::Vector3::UNIT_Y;
		 headPos = debugLines->getParentNode()->convertWorldToLocalPosition(headPos);
		 tailPos = debugLines->getParentNode()->convertWorldToLocalPosition(tailPos);
		 
		 debugLines->addLine(headPos+offs, tailPos+offs, color);*/
	}
}

