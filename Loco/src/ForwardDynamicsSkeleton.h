//
//  ForwardDynamicsSkeleton.h
//  Loco
//
//  Created on 28/10/13.
//
//

/*! @brief Controller to handle torques, velocities, etc of the articulated character. */

#ifndef __Loco__ForwardDynamicsSkeleton__
#define __Loco__ForwardDynamicsSkeleton__

#include <map>
#include <string>
#include <set>
#include <Ogre/OgreSharedPtr.h>
#include "DriveableSkeleton.h"
#include "picojson.h"
#include "Utilities.h"
#include "ForwardDynamicsBody.h"
#include "ForwardDynamicsJoint.h"
#include "ForwardDynamicsJointDriverPD.h"

class btTypedConstraint;
class btRigidBody;
class btCollisionShape;

namespace Ogre {
	class SceneNode;
}

namespace OgreBulletDynamics {
	class DynamicsWorld;
	class DebugLines;
	class RigidBody;
}

namespace OgreBulletCollisions {
	class CollisionShape;
}


#include <iostream>

class ForwardDynamicsSkeleton
{
public:
	ForwardDynamicsSkeleton( OgreBulletDynamics::DynamicsWorld* dynamicsWorld, Ogre::SharedPtr<DriveableSkeleton> outputSkeleton, const picojson::value& params );
	
	void debugDraw( OgreBulletCollisions::DebugLines* debugLines );
	
	std::set<std::string> getAllBodyNames();
	
	/*! @return The requested body. Assertion is raised if body name is not recognized. */
	Ogre::SharedPtr<ForwardDynamicsBody> getBody(std::string bodyName);
	/*! @return The joint from the childBodyName to its parent, or NULL if none exists. */
	Ogre::SharedPtr<ForwardDynamicsJoint> getJointToChild(std::string childBodyName );
	Ogre::SharedPtr<ForwardDynamicsJoint> getJointBetween(std::string parentBodyName, std::string childBodyName);
	std::vector<Ogre::SharedPtr<ForwardDynamicsJoint> > getAllJointsWithParent(std::string parentBodyName);
	/*! @return The name of the parent body, or "" if no parent exists. */
	std::string getParentBodyName(std::string childBodyName);
	/*! @return The parent body. Assertion is raised if no parent exists. */
	Ogre::SharedPtr<ForwardDynamicsBody> getParentBody(const std::string &childBodyName) { return getBody(getParentBodyName(childBodyName)); }
	
	/*! @brief Accumulate the given torque, in world space. Torques will be applied on the next call to update. */
	void addJointTorque( std::string jointName, Ogre::Vector3 torque );
	
	/*! @brief Copy all the body positions and orientations to the skeleton. */
	void update(float dt);
	
	void reset();
	
	void setCollisionsEnabled( bool enabled );
	
	/*! @brief Set the target orientation for the given body to be solved via PD. bodyName must have a joint in which it is child. */
	void setOrientationTarget( const std::string &bodyName, const Ogre::Quaternion &orientationWorld );
	/*! @brief Clear any previously assigned orientation target for the given body. bodyName must have a joint in which it is child. */
	void clearOrientationTarget( const std::string &bodyName );
	
	/*! @brief Set the target angular velocity for the given body to be solved via PD. bodyName must have a joint in which it is child. */
	void setAngularVelocityTarget( const std::string& bodyName, const Ogre::Vector3& angularVelocityWorld );
	/*! @brief Clear any previously assigned angular velocity target for the given body. bodyName must have a joint in which it is child. */
	void clearAngularVelocityTarget( const std::string& bodyName );
	
	/*! @brief Sum of masses of all bodies. */
	float getTotalMass() { return mTotalMass; }
	
	Ogre::Vector3 getCenterOfMassWorld();
	/*! @brief Calculated similar to center of mass but using velocity of each body rather than CoM. */
	Ogre::Vector3 getCenterOfMassVelocityWorld();
	
private:
	
	std::map<std::string, Ogre::SharedPtr<ForwardDynamicsBody> > mBodies;
	std::map<std::string, Ogre::SharedPtr<ForwardDynamicsJoint> > mJoints;
	std::map<std::string, Ogre::SharedPtr<ForwardDynamicsJointDriverPD> > mPDJointDrivers;
	
	Ogre::SceneNode* mPhysicsRootSceneNode;
	
	Ogre::SharedPtr<DriveableSkeleton> mDriveableSkeleton;
	float mTotalMass;
	
};

#endif /* defined(__Loco__ForwardDynamicsSkeleton__) */
