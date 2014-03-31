//
//  ForwardDynamicsBodyDriverPD.cpp
//  Loco
//
//  Created on 13/11/13.
//
//

#include "ForwardDynamicsBodyDriverPD.h"
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

ForwardDynamicsBodyDriverPD::ForwardDynamicsBodyDriverPD( Ogre::SharedPtr<ForwardDynamicsBody> body, float strength )
: mBody(body), mStrength(strength), mOrientationActive(false), mAngularVelocityActive(false)
{
}

void ForwardDynamicsBodyDriverPD::setTargetOrientationWorld( const Ogre::Quaternion& target )
{
	mTargetOrientation = target;
	mTargetOrientation.normalise();
	mOrientationActive = true;
}

void ForwardDynamicsBodyDriverPD::setTargetAngularVelocityWorld( const Ogre::Vector3& target )
{
	mTargetAngularVelocity = target;
	mAngularVelocityActive = true;
}


void ForwardDynamicsBodyDriverPD::updateTorque()
{
	if ( !mOrientationActive ) {
		return;
	}
	
	// current orientation in local frame
	Ogre::Quaternion qRel = mBody->getOrientationWorld();
	/*Ogre::Real length = */qRel.normalise();
	
	// desired orientation in local frame
	Ogre::Quaternion qRelD = mTargetOrientation;
	
	// current angular velocity in parent's frame
	Ogre::Vector3 wRel = BtOgreConverter::to(mBody->getBody()->getBulletRigidBody()->getAngularVelocity());
	
	// desired angular velocity in parent's frame
	Ogre::Vector3 wRelD = (mAngularVelocityActive ? mTargetAngularVelocity : Ogre::Vector3::ZERO);
	
	//BLog("%15s: ", mBone->getBone()->getName().c_str());
	
	//Ogre::Euler torque = computePDTorque(qRel, qRelD, wRel, wRelD, mBone->getKp(), mBone->getKd(), mStrength);
	Ogre::Vector3 torqueV3 = computePDTorque(qRel, qRelD, wRel, wRelD, mBody->getInertiaTensor(), mBody->getKp(), mBody->getKd(), mStrength);
	
	/*if ( mBody->getName() == "SpineBase" ) {
		BLog("%15s: qRel %s wRel %s", mBody->getName().c_str(), describe(qRel).c_str(), describe(wRel).c_str() );
	}*/
	
	// apply the torque
	//Ogre::Vector3 torqueV3( torque.getYaw().valueRadians(), torque.getPitch().valueRadians(), torque.getRoll().valueRadians() );
	
	mBody->addTorque(torqueV3);
	
}

void ForwardDynamicsBodyDriverPD::debugDraw( OgreBulletCollisions::DebugLines* debugLines )
{
	if ( getOrientationActive() )
	{
		//const Ogre::ColourValue color = Ogre::ColourValue(0.2,0.5,0.5);
		const Ogre::ColourValue targetColor = Ogre::ColourValue(1.0,1.0,0.5);
		Ogre::Vector3 offs(0,0,0.1);
		auto headPos = mBody->getHeadPositionWorld();
		//auto tailPos = mBody->getTailPositionWorld();
		auto tailOffset = getTargetOrientation()*Ogre::Vector3::UNIT_Y;
		//OgreAssert( fabsf(tailOffset.squaredLength()-1.0) < 0.0001, "bad" );
		auto tailPosTarget = headPos + tailOffset;
		
		// add offset
		headPos += offs;
		tailPosTarget += offs;
		
		//debugLines->addLine(headPos, tailPos, color);
		debugLines->addLine(headPos, tailPosTarget, targetColor);
		debugLines->addAxes((headPos+tailPosTarget)*0.5f, getTargetOrientation(), 0.04f);
		/*auto tailPos = headPos + body->getBody()->getSceneNode()->convertLocalToWorldOrientation(it.second->getTargetOrientation())*Ogre::Vector3::UNIT_Y;
		 headPos = debugLines->getParentNode()->convertWorldToLocalPosition(headPos);
		 tailPos = debugLines->getParentNode()->convertWorldToLocalPosition(tailPos);
		 
		 debugLines->addLine(headPos+offs, tailPos+offs, color);*/
	}
}

