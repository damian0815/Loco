//
//  DriveableBone.h
//  Loco
//
//  Created on 17/10/13.
//
//

#ifndef __Loco__DriveableBone__
#define __Loco__DriveableBone__

#include <iostream>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wshorten-64-to-32"

#include <Ogre/OgreVector3.h>
#include <Ogre/OgreQuaternion.h>
#include <Ogre/OgreMatrix3.h>

#pragma clang diagnostic pop

#include "Serializable.h"
#include <string>


namespace Ogre {
	class Bone;
};

namespace OgreBulletCollisions {
	class DebugLines;
}

/*! @class DriveableBone
 
 Wraps an ogre bone that can be driven using torque, force or relative rotation. It is expected that the bone will not be otherwise manipulated (in position on orientation terms) between calls to update().
 
 */

class DriveableBone : public Serializable
{
public:
	DriveableBone( Ogre::Bone* bone, picojson::value& source );
	DriveableBone( Ogre::Bone* bone, float boneMass, float boneLength, float Kp, float Kd );
	
	/*! @abstract Velocity is in local space. */
	Ogre::Vector3 getVelocity() const { return mVelocity; }
	/*! @abstract Angular velocity is in local space. */
	Ogre::Vector3 getAngularVelocity() const;
	/// return the center of mass of this bone, in skeletal world coordinates
	Ogre::Vector3 getCenterOfMassWorld() const;
	/*! return the head position of this bone, in skeletal world coordinates */
	Ogre::Vector3 getHeadPositionWorld() const;
	
	/*! @abstract Add torque, in parent coordinate space. Reset to 0 after update. */
	void addTorque( const Ogre::Vector3 &torque );
	/*! @abstract Add force, in parent coordinate space. Reset to 0 after update. */
	void addForce( const Ogre::Vector3& force );
	/*! @abstract Add a relative orientation (rotation) in parent coordinate space. Will be removed at the next update step. Reset to IDENTITY after update. */
	void addRelativeRotation( const Ogre::Quaternion& rotation );

	/*! @abstract Clear angular momentum and velocity, and re-read the current bone orientation to use as a baseline. Does not clear any force, torque or relative rotation added since the last frame. */
	void reset();
	/*! @abstract Reset bone to baseline, apply torque to angular momentum and force to velocity, then apply angular momentum and velocity to the bone. Finally, apply the additional relative rotation. */
	void update( float dt );
	
	Ogre::Bone* getBone() const { return mBone; }
	float getLength() const { return mLength; }
	float getMass() const { return mMass; }
	float getKd() const { return mKd; }
	float getKp() const { return mKp; }
	float getMaxAbsTorque() const { return mMaxAbsTorque; }
	
	picojson::value serialize() const;
	void deserialize( picojson::value& source );
	
	void debugDraw( OgreBulletCollisions::DebugLines *debugLines, const Ogre::ColourValue& boneColour, float axisLength=0.0f, const Ogre::Vector3& offset=Ogre::Vector3::ZERO );
	
private:
	void calculateInverseInertiaTensor();
	Ogre::Vector3 limitTorque(const Ogre::Vector3& torque);
	bool shouldLimitOrientation();
	
	Ogre::Bone* mBone;
	float mMass;
	float mLength;
	float mKp, mKd; // constants for PD controller
	Ogre::Vector3 mTorque;
	Ogre::Vector3 mAngularMomentum;
	
	Ogre::Vector3 mForce;
	Ogre::Vector3 mVelocity;
	
	Ogre::Quaternion mPrevOrientation; // the non-relative orientation, ie orientation before applying the relative rotations
	Ogre::Quaternion mCoalescedRelativeRotation;
	
	Ogre::Matrix3 mInverseInertiaTensor;
	float mMaxAbsTorque;
};

#endif /* defined(__Loco__DriveableBone__) */
