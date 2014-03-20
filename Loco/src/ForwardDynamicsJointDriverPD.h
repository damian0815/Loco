//
//  ForwardDynamicsJointDriverPD.h
//  Loco
//
//  Created on 13/11/13.
//
//

#ifndef __Loco__ForwardDynamicsJointDriverPD__
#define __Loco__ForwardDynamicsJointDriverPD__

#include <iostream>

#include "AnimationSharedStuff.h"
#include "DriveableBone.h"
#include <Ogre/OgreQuaternion.h>
#include <Ogre/OgreSharedPtr.h>
#include "ForwardDynamicsJoint.h"

namespace OgreBulletCollisions {
	class DebugLines;
};

/*! @brief Proportional derivative bone orientation/position driver. */

class ForwardDynamicsJointDriverPD
{
public:
	ForwardDynamicsJointDriverPD( Ogre::SharedPtr<ForwardDynamicsJoint> joint, float strength=1.0f );
	
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
	
	/**
	 
	 from SimBiCon
	 
	 This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
	 Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
	 the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque
	 returned is expressed in the coordinate frame of the 'parent'.
	 */
	static Ogre::Vector3 computePDTorque(const Ogre::Quaternion& qRel, const Ogre::Quaternion& qRelD, const Ogre::Vector3& wRel, const Ogre::Vector3& wRelD, double kp, double kd, double strength );
	
private:
	Ogre::SharedPtr<ForwardDynamicsJoint> mJoint;
	
	Ogre::Quaternion mTargetOrientation;
	Ogre::Vector3 mTargetAngularVelocity;
	
	float mStrength;
	
	float mOrientationActive;
	float mAngularVelocityActive;
	
};



#endif /* defined(__Loco__ForwardDynamicsJointDriverPD__) */
