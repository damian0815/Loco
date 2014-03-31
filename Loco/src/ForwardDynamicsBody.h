//
//  ForwardDynamicsBody.h
//  Loco
//
//  Created on 30/10/13.
//
//

#ifndef __Loco__ForwardDynamicsBody__
#define __Loco__ForwardDynamicsBody__

#include <iostream>
#include <Ogre/OgreSharedPtr.h>

#include "DriveableBone.h"
#include "picojson.h"
#include <map>
#include "OgreBulletDynamicsRigidBody.h"


class ForwardDynamicsSkeleton;

namespace Ogre {
	class SceneNode;
}

namespace OgreBulletDynamics {
	class DynamicsWorld;
}

namespace OgreBulletCollisions {
	class CollisionShape;
	class DebugLines;
	
}

class ForwardDynamicsBody
{
public:
	ForwardDynamicsBody( Ogre::SharedPtr<DriveableBone> parentBone, std::vector<Ogre::SharedPtr<DriveableBone> > childBones, ForwardDynamicsSkeleton* owner, OgreBulletDynamics::DynamicsWorld* world, const picojson::value& bodyDef );
	~ForwardDynamicsBody();
	
	/*! @brief Return the head position (== position of parent joint) in world space. */
	Ogre::Vector3 getHeadPositionWorld();
	/*! @brief Return the head position (== position of parent joint) in local space (CoM == (0,0,0)). */
	Ogre::Vector3 getHeadPositionLocal() { return mParentPositionLocal; }
	/*! @brief Return the tail position (== position of child joint) in world space. */
	Ogre::Vector3 getTailPositionWorld(std::string tailName="");
	/*! @brief Return the tail position (== position of child joint) in local space (CoM == (0,0,0)). */
	Ogre::Vector3 getTailPositionLocal(std::string tailName="");
	/*! @brief Return the orientation in world space. */
	Ogre::Quaternion getOrientationWorld();
	/*! @brief Return the rest orientation (the orientation on init) in local space. */
	Ogre::Quaternion getParentRelativeRestOrientation() { return mRestOrientationParentRelative; }
	void setParentRelativeRestOrientation(const Ogre::Quaternion& q) { mRestOrientationParentRelative =q; }
	/*! @brief Return the position of the center of mass in world space. */
	Ogre::Vector3 getCoMWorld();
	
	Ogre::Vector3 convertLocalToWorldPosition(const Ogre::Vector3& localPos) const { return mBody->getSceneNode()->convertLocalToWorldPosition(localPos); }
	Ogre::Vector3 convertWorldToLocalPosition(const Ogre::Vector3& worldPos) const { return mBody->getSceneNode()->convertWorldToLocalPosition(worldPos); }
	
	Ogre::Quaternion convertLocalToWorldOrientation(const Ogre::Quaternion& localOri) const { return mBody->getSceneNode()->convertLocalToWorldOrientation(localOri); }
	Ogre::Quaternion convertWorldToLocalOrientation(const Ogre::Quaternion& worldOri) const { return mBody->getSceneNode()->convertWorldToLocalOrientation(worldOri); }
	
	/**
	 This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, the resulting velocity will be expressed in world coordinates.
	 */
	Ogre::Vector3 getAbsoluteVelocityForLocalPoint(const Ogre::Vector3& localPoint);
	
	std::string getName() { return mParentBoneName; }
	unsigned int getNumChildren() { return mChildPositionsLocal.size(); }
	std::string getAnyChildName();
	
	OgreBulletDynamics::RigidBody* getBody() { return mBody; }
	
	void debugDraw(OgreBulletCollisions::DebugLines* debugLines);
	
	void reset( Ogre::SharedPtr<DriveableBone> parentBone, Ogre::SceneNode* skeletonRootNode );
	
	/*! @brief add torque to be applied to this body. will be moved up to the joint, if possible, at the last possible moment. */
	void addTorque( const Ogre::Vector3& torque ) { mTorque += torque; }
	/*! @brief apply torque to this body and reset */
	void applyTorque();
	/*! @brief get existing torque to be applied */
	const Ogre::Vector3& getTorque() { return mTorque; }
	/*! @brief clear torque */
	void clearTorque() { mTorque = Ogre::Vector3::ZERO; }
	/*! @brief apply torque limit to given torque vector */
	void limitTorque( Ogre::Vector3& torqueToLimit );
	
	
	void addImpulse( const Ogre::Vector3& impulse );
	
	float getMass() const;
	float getKp() const { return mKp; }
	float getKd() const { return mKd; }
	float getMaxTorque() const { return mMaxTorque; }
	Ogre::Matrix3 getInertiaTensor() const { return mBody->getInertiaTensor(); }
	
private:
	
	
	ForwardDynamicsSkeleton* mOwnerSkeleton;
	
	std::string mParentBoneName;
	float mKp, mKd, mMaxTorque;
	Ogre::Vector3 mTorqueScale;
	
	Ogre::Vector3 mTorque;
	
	OgreBulletCollisions::CollisionShape* mCollisionShape;
	OgreBulletDynamics::RigidBody* mBody;
	
	Ogre::Vector3 mParentPositionLocal; // position of the parent in local coordinates
	std::map<std::string,Ogre::Vector3> mChildPositionsLocal; // position of the child in local coordinates
	
	Ogre::Quaternion mRestOrientationParentRelative;
};


#endif /* defined(__Loco__ForwardDynamicsBody__) */
