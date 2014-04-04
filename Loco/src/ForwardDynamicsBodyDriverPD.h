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
#include "ProportionalDerivativeController.h"

namespace OgreBulletCollisions {
	class DebugLines;
};

/*! @brief Proportional derivative bone orientation/position driver. */

class ForwardDynamicsBodyDriverPD: public ProportionalDerivativeController
{
public:
	ForwardDynamicsBodyDriverPD( Ogre::SharedPtr<ForwardDynamicsBody> body, float strength=1.0f );
	
	/*! @brief Set target orientation, in world space. */
	void setTargetOrientationWorld( const Ogre::Quaternion& targetOrientation );
	const Ogre::Quaternion& getTargetOrientation() const { return mTargetOrientation; }
	bool getOrientationActive() const { return mOrientationActive; }
	/*! @brief Stop tracking the target orientation. */
	void unsetTargetOrientation() { mOrientationActive = false; }
	
	/*! @brief Set target angular velocity. Must set a target orientation as well for this to do anything. */
	void setTargetAngularVelocityWorld( const Ogre::Vector3& targetAngularVelocity );
	bool getAngularVelocityActive() const { return mAngularVelocityActive; }
	/*! @brief Stop tracking the target orientation. */
	void unsetTargetAngularVelocity() { mAngularVelocityActive = false; }
	
	/*! @brief Update the torque on the driven bone using the target orientation. */
	void updateTorque();
	
	void debugDraw( OgreBulletCollisions::DebugLines* debugLines );
	
private:
	Ogre::SharedPtr<ForwardDynamicsBody> mBody;
	
	Ogre::Quaternion mTargetOrientation;
	Ogre::Vector3 mTargetAngularVelocity;
	
	float mStrength;
	
	float mOrientationActive;
	float mAngularVelocityActive;
	
};



#endif /* defined(__Loco__ForwardDynamicsBodyDriverPD__) */
