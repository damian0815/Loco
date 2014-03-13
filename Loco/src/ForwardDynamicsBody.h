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
	ForwardDynamicsBody( Ogre::SharedPtr<DriveableBone> parentBone, std::vector<Ogre::SharedPtr<DriveableBone> > childBones, Ogre::SceneNode* skeletonRootNode, OgreBulletDynamics::DynamicsWorld* world, const picojson::value& bodyDef );
	~ForwardDynamicsBody();
	
	/*! @abstract Return the head position (== position of parent joint) in world space. */
	Ogre::Vector3 getHeadPositionWorld();
	/*! @abstract Return the head position (== position of parent joint) in local space (CoM == (0,0,0)). */
	Ogre::Vector3 getHeadPositionLocal() { return mParentPositionLocal; }
	/*! @abstract Return the tail position (== position of child joint) in world space. */
	Ogre::Vector3 getTailPositionWorld(std::string tailName="");
	/*! @abstract Return the tail position (== position of child joint) in local space (CoM == (0,0,0)). */
	Ogre::Vector3 getTailPositionLocal(std::string tailName="");
	/*! @abstract Return the orientation in world space. */
	Ogre::Quaternion getOrientationWorld();
	/*! @abstract Return the orientation in local space (ie relative to the scene node that is the parent of this bone. */
	Ogre::Quaternion getOrientationLocal();
	/*! @abstract Return the rest orientation (the orientation on init) in local space. */
	Ogre::Quaternion getParentRelativeRestOrientation() { return mRestOrientationLocal; }
	void setParentRelativeRestOrientationLocal(const Ogre::Quaternion& q) { mRestOrientationLocal =q; }
	/*! @abstract Return the position of the center of mass in world space. */
	Ogre::Vector3 getCoMWorld();
	
	Ogre::Vector3 convertLocalToWorldPosition(const Ogre::Vector3& localPos) const { return mBody->getSceneNode()->convertLocalToWorldPosition(localPos); }
	Ogre::Vector3 convertWorldToLocalPosition(const Ogre::Vector3& worldPos) const { return mBody->getSceneNode()->convertWorldToLocalPosition(worldPos); }
	
	Ogre::Quaternion convertLocalToWorldOrientation(const Ogre::Quaternion& localOri) const { return mBody->getSceneNode()->convertLocalToWorldOrientation(localOri); }
	Ogre::Quaternion convertWorldToLocalOrientation(const Ogre::Quaternion& worldOri) const { return mBody->getSceneNode()->convertWorldToLocalOrientation(worldOri); }
	
	std::string getName() { return mParentBoneName; }
	unsigned int getNumChildren() { return mChildPositionsLocal.size(); }
	std::string getAnyChildName();
	
	OgreBulletDynamics::RigidBody* getBody() { return mBody; }
	
	void debugDraw(OgreBulletCollisions::DebugLines* debugLines);
	
	void reset( Ogre::SharedPtr<DriveableBone> parentBone, Ogre::SceneNode* skeletonRootNode );
	
	void addTorque( const Ogre::Vector3& torque ) { mTorque += torque; }
	void applyTorque();
	
	void addImpulse( const Ogre::Vector3& impulse );
	
	
	float getMass();
	float getKp() { return mKp; }
	float getKd() { return mKd; }
	
private:
	
	std::string mParentBoneName;
	float mKp, mKd, mMaxTorque;
	Ogre::Vector3 mTorqueScale;
	
	Ogre::Vector3 mTorque, mPrevTorque;
	
	OgreBulletCollisions::CollisionShape* mCollisionShape;
	OgreBulletDynamics::RigidBody* mBody;
	
	Ogre::Vector3 mParentPositionLocal; // position of the parent in local coordinates
	std::map<std::string,Ogre::Vector3> mChildPositionsLocal; // position of the child in local coordinates
	
	Ogre::Quaternion mRestOrientationLocal;
};


#endif /* defined(__Loco__ForwardDynamicsBody__) */
