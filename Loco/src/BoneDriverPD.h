//
//  BoneDriverPD.h
//  Loco
//
//  Created on 17/10/13.
//
//

#ifndef __Loco__BoneDriverPD__
#define __Loco__BoneDriverPD__

#include <iostream>

#include "AnimationSharedStuff.h"
#include "DriveableBone.h"
#include <Ogre/OgreQuaternion.h>
#include <Ogre/OgreSharedPtr.h>

/*! @abstract Proportional derivative bone orientation/position driver. */

class BoneDriverPD
{
public:
	BoneDriverPD( Ogre::SharedPtr<DriveableBone> bone, float strength=1.0f );
	
	/*! @abstract Set target orientation, in bone's local space. */
	void setTargetOrientation( const Ogre::Quaternion& targetOrientation );
	Ogre::Quaternion& getTargetOrientation() { return mTargetOrientation; }
	/*! @abstract Set target position, in world space. */
	void setTargetPosition( const Ogre::Vector3& targetPosition );

	/*! @abstract Enable/disable driving the bone to its target orientation */
	void setOrientationActive(bool tf) { mOrientationActive=tf; }
	bool getOrientationActive() { return mOrientationActive; }

	/*! @abstract Update the torque on the driven bone using the target orientation. */
	void updateTorque();
	/*! @abstract Update the force on the driven bone using the target position. */
	void updateForce();
	
private:
	Ogre::SharedPtr<DriveableBone> mBone;
	
	Ogre::Quaternion mTargetOrientation;
	Ogre::Vector3 mTargetPosition;
	
	float mStrength;
	
	float mPositionActive, mOrientationActive;
	
};


#endif /* defined(__Loco__BoneDriverPD__) */
