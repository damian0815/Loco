//
//  ForwardDynamicsJoint.h
//  Loco
//
//  Created on 07/11/13.
//
//

#ifndef __Loco__ForwardDynamicsJoint__
#define __Loco__ForwardDynamicsJoint__

//#define LocoForwardDynamicsJoint_ConvertTorqueToForce

#include <iostream>
#include "picojson.h"
#include <map>
#include "ForwardDynamicsBody.h"
#include <Ogre/OgreVector3.h>

namespace Ogre {
}

namespace OgreBulletDynamics {
	class DynamicsWorld;
}

namespace OgreBulletCollisions {
	class CollisionShape;
	class DebugLines;
}


class btTypedConstraint;

class ForwardDynamicsJoint {
public:
	
	static float kLimitDamping;
	static float kLimitSoftness;

	static float kNormalCFM;
	static float kLimitCFM;
	static float kLimitERP;

	static float kLinearLimitDamping;
	static float kLinearLimitSoftness;
	
	ForwardDynamicsJoint( OgreBulletDynamics::DynamicsWorld* dynamicsWorld, Ogre::SharedPtr<ForwardDynamicsBody> parentFdb, Ogre::SharedPtr<ForwardDynamicsBody> childFdb, picojson::object jointDef );
	~ForwardDynamicsJoint();
	
	Ogre::SharedPtr<ForwardDynamicsBody> getParentFdb() { return mParentFdb; }
	Ogre::SharedPtr<ForwardDynamicsBody> getChildFdb() { return mChildFdb; }
	
	Ogre::Vector3 getPositionWorld();
	Ogre::Vector3 getPositionInParentSpace();
	Ogre::Vector3 getPositionInChildSpace();
	
	void addTorque( const Ogre::Vector3& torque );
	const Ogre::Vector3& getTorque() const { return mTorque; }
	/*! @brief Apply the torque to parent and child FDBs, then clear torque vector. */
	void applyTorque();
	
	/*! @brief clear torque */
	void clearTorque();
	
	void debugDraw(OgreBulletCollisions::DebugLines* debugLines);
	
	std::string getName() { return mParentFdb->getName()+"-"+mChildFdb->getName(); }
	
	btTypedConstraint* getBulletConstraint() { return mConstraint; }
	
private:
	
	OgreBulletDynamics::DynamicsWorld* mDynamicsWorld;
	
	btTypedConstraint* mConstraint;
	btTypedConstraint* mSecondaryConstraint;
	Ogre::SharedPtr<ForwardDynamicsBody> mParentFdb;
	Ogre::SharedPtr<ForwardDynamicsBody> mChildFdb;
	
	Ogre::Vector3 mPosParent, mPosChild;
	Ogre::Vector3 mTorque;
	Ogre::Vector3 mDebugPrevTorque;
};

#endif /* defined(__Loco__ForwardDynamicsJoint__) */
