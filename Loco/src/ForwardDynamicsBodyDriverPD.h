//
//  ForwardDynamicsBodyDriverPD.h
//  Loco
//
//  Created on 13/11/13.
//
//

#ifndef __Loco__ForwardDynamicsBodyDriverPD__
#define __Loco__ForwardDynamicsBodyDriverPD__

#include <iostream>

#include "AnimationSharedStuff.h"
#include "DriveableBone.h"
#include <Ogre/OgreQuaternion.h>
#include <Ogre/OgreSharedPtr.h>
#include "ForwardDynamicsBody.h"

namespace OgreBulletCollisions {
	class DebugLines;
};

/*! @abstract Proportional derivative bone orientation/position driver. */

class ForwardDynamicsBodyDriverPD
{
public:
	ForwardDynamicsBodyDriverPD( Ogre::SharedPtr<ForwardDynamicsBody> body, float strength=1.0f );
	
	/*! @abstract Set target orientation, in body's local space. */
	//void setTargetOrientationLocal( const Ogre::Quaternion& targetOrientation );
	/*! @abstract Set target orientation, in world space. */
	void setTargetOrientationWorld( const Ogre::Quaternion& targetOrientation );
	const Ogre::Quaternion& getTargetOrientation() const { return mTargetOrientation; }
	bool getOrientationActive() const { return mOrientationActive; }
	/*! @abstract Stop tracking the target orientation. */
	void unsetTargetOrientation() { mOrientationActive = false; }
	
	/*! @abstract Update the torque on the driven bone using the target orientation. */
	void updateTorque();
	
	void debugDraw( OgreBulletCollisions::DebugLines* debugLines );
	
private:
	Ogre::SharedPtr<ForwardDynamicsBody> mBody;
	
	Ogre::Quaternion mTargetOrientation;
	
	float mStrength;
	
	float mOrientationActive;
	
};



#endif /* defined(__Loco__ForwardDynamicsBodyDriverPD__) */
