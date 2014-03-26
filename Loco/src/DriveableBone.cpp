//
//  DriveableBone.cpp
//  Loco
//
//  Created on 17/10/13.
//
//

#include "DriveableBone.h"
#include <Ogre/OgreBone.h>
#include "AnimationSharedStuff.h"
#include "Utilities.h"
#include "Euler.h"
#include "OgreBulletCollisionsDebugLines.h"

using namespace std;
using namespace picojson;

DriveableBone::DriveableBone( Ogre::Bone* bone, float boneLength, float boneMass, float kp, float kd )
: mBone(bone), mAngularMomentum(0,0,0), mTorque(0,0,0), mForce(0,0,0), mVelocity(0,0,0), mMass(boneMass), mMaxAbsTorque(100.0f), mLength(boneLength), mKp(kp), mKd(kd), mPrevOrientation(bone->getInitialOrientation()), mCoalescedRelativeRotation(Ogre::Quaternion::IDENTITY)
{
	// construct inverse inertia tensor
	calculateInverseInertiaTensor();
	BLog("%15s: pos %s mass %f length %f kp %f kd %f", mBone->getName().c_str(), describe(mBone->getPosition()).c_str(), mMass, mLength, mKp, mKd );
	
}


DriveableBone::DriveableBone( Ogre::Bone* bone, picojson::value& source )
: mAngularMomentum(0,0,0), mBone(bone), mTorque(0,0,0), mVelocity(0,0,0), mForce(0,0,0), mPrevOrientation(bone->getInitialOrientation()), mCoalescedRelativeRotation(Ogre::Quaternion::IDENTITY)
{
	deserialize(source);
	calculateInverseInertiaTensor();
	BLog("%15s: pos %s mass %f length %f kp %f kd %f", mBone->getName().c_str(), describe(mBone->getPosition()).c_str(), mMass, mLength, mKp, mKd );
}

void DriveableBone::calculateInverseInertiaTensor()
{
	// solid cylinder
	// http://en.wikipedia.org/wiki/List_of_moment_of_inertia_tensors
	float h = mLength;
	float r = h*0.1f; // 10% width? shrug.
	float m = mMass;
	float t1 = 0.33333f*m*(3.0f*r*r + h*h);
	float t2 = 0.5f*m*(r*r);
	
	// invert of a
	mInverseInertiaTensor = Ogre::Matrix3(1.0f/t1,0,0,
										  0,1.0f/t1,0,
										  0,0,1.0f/t2);
	
	/*
	mInverseInertiaTensor = Ogre::Matrix3(0.33333f*mMass*mLength*mLength,0,0,
										  0,0,0,
										  0,0,0.33333f*mMass*mLength*mLength
										  ).Inverse();*/
	 
//	mInverseInertiaTensor = Ogre::Matrix3::IDENTITY;
}

picojson::value DriveableBone::serialize() const
{
	object contents;
	contents["length"] = value(mLength);
	contents["mass"] = value(mMass);
	contents["maxTorque"] = value(mMaxAbsTorque);
	contents["kP"] = value(mKp);
	contents["kD"] = value(mKd);
	return value(contents);
}

void DriveableBone::deserialize( picojson::value& value )
{
	object contents = value.get<object>();
	mMass = contents["mass"].get<double>();
	mLength = contents["length"].get<double>();
	mMaxAbsTorque = contents["maxTorque"].get<double>();
	mKp = contents["kP"].get<double>();
	mKd = contents["kD"].get<double>();
	
}

Ogre::Vector3 DriveableBone::limitTorque(const Ogre::Vector3& torque)
{
	Ogre::Vector3 limitedTorque;
	limitedTorque.x = MAX(MIN(torque.x, mMaxAbsTorque), -mMaxAbsTorque);
	limitedTorque.y = MAX(MIN(torque.y, mMaxAbsTorque), -mMaxAbsTorque);
	limitedTorque.z = MAX(MIN(torque.z, mMaxAbsTorque), -mMaxAbsTorque);
	return limitedTorque;
	
}

/*
Ogre::Vector3 clampTorqueRateOfChange( const Ogre::Vector3& oldTorque, const Ogre::Vector3& torque )
{
	Ogre::Vector3 deltaT = torque-oldTorque;

	double maxTorqueRateOfChange = 2000;
	// clamp torque rate of change
	deltaT.x = MAX(MIN(deltaT.x, maxTorqueRateOfChange), -maxTorqueRateOfChange);
	deltaT.y = MAX(MIN(deltaT.x, maxTorqueRateOfChange), -maxTorqueRateOfChange);
	deltaT.z = MAX(MIN(deltaT.x, maxTorqueRateOfChange), -maxTorqueRateOfChange);

	Ogre::Vector3 clampedTorque = oldTorque + deltaT;
	return clampedTorque;
}*/

void DriveableBone::addTorque( const Ogre::Vector3& torque )
{
	mTorque += torque;
}

void DriveableBone::addForce( const Ogre::Vector3& force )
{
	mForce += force;
}


Ogre::Vector3 DriveableBone::getAngularVelocity() const
{
	return mAngularMomentum * mInverseInertiaTensor;
}

void DriveableBone::update( float timeStep )
{
	// limit the torque
	Ogre::Vector3 limitedTorque = limitTorque(mTorque);
	/*if ( mBone->getName() == "LegUpper.R" ) {
		BLog("%15s: torque %s momentum %s", mBone->getName().c_str(), describe(limitedTorque).c_str(), describe(mAngularMomentum).c_str() );
	}*/
	// clamp the torque's rate of change
	// limitedTorque = clampTorqueRateOfChange(mPrevTorque, torque);
	
	// apply the torque to the bone at its center of mass
	
	// I = m*r^2
	//float momentOfInertia = mMass*(mLength*0.5f)*(mLength*0.5f);
	
	// torque = Ia where a = angular acceleration
	// -> a = torque/I
	//Ogre::Vector3 angularAcceleration = limitedTorque/momentOfInertia;
	
	// update the angular velocity
	//mAngularVelocity += angularAcceleration*timeStep;
	
	mAngularMomentum += limitedTorque*timeStep;
	
	// update the bone's actual orientation by applying the angular velocity over the timestep
	Ogre::Quaternion q = mPrevOrientation;
	
	
	//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
	Ogre::Vector3 axis;
	Ogre::Vector3 angvel = getAngularVelocity();
	float	fAngle = angvel.length();
	//limit the angular motion
	float angularMotionThreshold = M_PI_4;
	if (fAngle*timeStep > angularMotionThreshold)
	{
		fAngle = angularMotionThreshold / timeStep;
	}
	
	if ( fAngle < 0.001f )
	{
		// use Taylor's expansions of sync function
		axis   = angvel*( 0.5f*timeStep-(timeStep*timeStep*timeStep)*0.020833333333f*fAngle*fAngle );
	}
	else
	{
		// sync(fAngle) = sin(c*fAngle)/t
		axis   = angvel*( sinf(0.5f*fAngle*timeStep)/fAngle );
	}
	Ogre::Quaternion dorn (cosf( fAngle*timeStep*0.5f ),axis.x,axis.y,axis.z);
	Ogre::Quaternion orn0 = q;
	
	Ogre::Quaternion newQ = dorn * orn0;
	
	/* 
	 
	 http://gamedev.stackexchange.com/questions/18036/problem-representing-torque-as-a-quaternion
	 
	 Then you multiply the angular momentum by the inverse of the inertia tensor, to get an angular velocity, and integrate it to get a quaternion for the orientation by using the rule: dq/dt = 1/2 omega q, where q is the quaternion representing the current orientation of the body, and omega is the angular velocity vector. Omega has to be converted to a quat by placing 0 in the scalar (real) component, so you can multiply it by q using quaternion multiplication.*/
	
	
	newQ.normalise();
//	Ogre::Quaternion newQ = q*makeEulerQuaternion( getAngularVelocity()*timeStep );
	
	// limit rotation
	if ( shouldLimitOrientation() )
	{
		Ogre::Quaternion qRelToBind = mBone->getInitialOrientation().Inverse()*newQ;
		qRelToBind.normalise();
		Ogre::Radian angle;
		//Ogre::Vector3 axis;
		qRelToBind.ToAngleAxis( angle, axis );
		if ( fabsf(angle.valueRadians()) > M_PI_2 ) {
			float scalingFactor = M_PI_2/fabsf(angle.valueRadians());
			angle *= scalingFactor;
			qRelToBind = Ogre::Quaternion( angle, axis );
			newQ = qRelToBind*mBone->getInitialOrientation();
			newQ.normalise();
		}
	}
	
	// store the newQ as 'prev' orientation
	mPrevOrientation = newQ;
	
	// apply relative local stuff
	newQ = newQ * mCoalescedRelativeRotation;
	// finally, assign
	mBone->setOrientation(newQ);
	
	/*
	if ( mBone->getName()=="LegLower.L" ) {
		BLog("%10s rotated from %s to %s (momentum %s speed %s)", mBone->getName().c_str(), describe(q).c_str(), describe(newQ).c_str(), describe(mAngularMomentum).c_str(),  describe(getAngularVelocity()).c_str()  );
	}*/
	
	
	// apply force
	mVelocity += mForce*timeStep/mMass;
	Ogre::Vector3 newPosition = mBone->getPosition() + mVelocity*timeStep;
	mBone->setPosition(newPosition);

		
	
	// reset the torque
	mTorque = Ogre::Vector3(0,0,0);
	// reset the force
	mForce = Ogre::Vector3(0,0,0);
	// reset the coalesced relative rotation
	mCoalescedRelativeRotation = Ogre::Quaternion::IDENTITY;
}	
	// damp the momentum
	//mAngularMomentum *= 0.0f;


Ogre::Vector3 DriveableBone::getCenterOfMassWorld() const
{
	// find the center of mass in the parent's frame
	Ogre::Vector3 myPos(0,0,0); // myPos in this bone's frame is 0,0,0
	Ogre::Quaternion myOrientation = Ogre::Quaternion::IDENTITY;
	Ogre::Vector3 accumulatedChildPos(0,0,0);
	int count = 0;
	for ( int i=0; i<mBone->numChildren(); i++ ) {
		Ogre::Bone* child = dynamic_cast<Ogre::Bone*>(mBone->getChild(i));
		if ( child ) {
			count++;
			accumulatedChildPos += myOrientation*child->getPosition();
		}
	}
	
	if ( count>0 ) {
		accumulatedChildPos /= (float)count;
	}
	// centre of mass == average of ( my position, the average of my childrens' positions )
	Ogre::Vector3 comLocal = (myPos + accumulatedChildPos)*0.5f;
	return mBone->convertLocalToWorldPosition(comLocal);
}

Ogre::Vector3 DriveableBone::getHeadPositionWorld() const
{
	return mBone->convertLocalToWorldPosition(Ogre::Vector3(0,0,0));
}


bool DriveableBone::shouldLimitOrientation()
{
	return false;
}

void DriveableBone::debugDraw( OgreBulletCollisions::DebugLines* debugLines, const Ogre::ColourValue& boneColour, float axisLength, const Ogre::Vector3& offset )
{
	auto bonePos = getBone()->_getDerivedPosition() + offset;
	// draw line to parent
	auto parent = getBone()->getParent();
	if ( parent ) {
		auto parentPos = getBone()->getParent()->_getDerivedPosition() + offset;
		debugLines->addLine( bonePos, parentPos, boneColour );
	}
	if ( axisLength>0.0f ) {
		// draw axes
		auto boneOrientation = getBone()->_getDerivedOrientation();
		float distance = axisLength;
		Ogre::Vector3 xAxis, yAxis, zAxis;
		boneOrientation.ToAxes(xAxis, yAxis, zAxis);
		debugLines->addLine(bonePos, bonePos+xAxis*distance, Ogre::ColourValue::Red);
		debugLines->addLine(bonePos, bonePos+yAxis*distance, Ogre::ColourValue::Green);
		debugLines->addLine(bonePos, bonePos+zAxis*distance, Ogre::ColourValue::Blue);
	}
}

void DriveableBone::addRelativeRotation(const Ogre::Quaternion& relativeRot )
{
	mCoalescedRelativeRotation = mCoalescedRelativeRotation*relativeRot;
}


void DriveableBone::reset()
{
	mAngularMomentum = Ogre::Vector3(0,0,0);
	mVelocity = Ogre::Vector3(0,0,0);
	//BLog("bone named %s taking orientation %s", getBone()->getName().c_str(), describe(mBone->getOrientation()).c_str());
	mPrevOrientation = mBone->getOrientation();
}

