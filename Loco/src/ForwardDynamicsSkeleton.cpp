//
//  ForwardDynamicsSkeleton.cpp
//  Loco
//
//  Created on 28/10/13.
//
//

#include "ForwardDynamicsSkeleton.h"
#include <Ogre/OgreSceneManager.h>
#include <Ogre/OgreBone.h>
#include "OgreBulletDynamicsRigidBody.h"
#include "OgreBulletDynamicsWorld.h"
#include "btRigidBody.h"
#include "btTransform.h"
#include "OgreBulletConverter.h"
#include "Utilities.h"
#include "btHingeConstraint.h"
#include "btPoint2PointConstraint.h"
#include "ForwardDynamicsJoint.h"


// featherstone
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"




using namespace OgreBulletCollisions;
using namespace OgreBulletDynamics;

using namespace picojson;


static std::string getNameForFDBoneBetween( string parentName, string childName )
{
	return parentName+"-"+childName;
}

ForwardDynamicsSkeleton::ForwardDynamicsSkeleton( DynamicsWorld* dynamicsWorld, Ogre::SharedPtr<DriveableSkeleton> outputSkeleton, const value& params )
: mDriveableSkeleton(outputSkeleton),
mPhysicsRootSceneNode(dynamicsWorld->getSceneManager()->getRootSceneNode())
{
	/*
	// setup featherstone multibody
	// count joints
	unsigned int jointCount = 0;
	float baseMass = 0;
	const string rootJointName = "SpineBase";
	for ( const auto bodyDefValue: bodyDefs ) {
		object bodyDef = bodyDefValue.get<object>();
		// get bone
		auto bodyName = bodyDef["name"].get<string>();
		auto bone = mDriveableSkeleton->getBone(bodyName);
		if ( bodyName == rootJointName ) {
			baseMass = bone->getMass();
		}
		
		// get children
		if ( bodyDef.count("joints") ) {
			auto childJointDefs = bodyDef["joints"].get<object>();
			jointCount += childJointDefs.size();
		}
	}
	
	mFeatherstoneMultiBody = new btMultiBody(int jointCount,                // NOT including the base
											 btScalar mTotalMass,                // mass of base
															  const btVector3 &inertia,    // inertia of base, in base frame; assumed diagonal
															  bool fixed_base_,           // whether the base is fixed (true) or can move (false)
															  bool can_sleep_);
		
	 */
	// ...
	object controllerDef = params.get<object>();
	if ( controllerDef.count("limitSoftness") ) {
		ForwardDynamicsJoint::kLimitSoftness = controllerDef.at("limitSoftness").get<double>();
	}
	if ( controllerDef.count("limitDamping") ) {
		ForwardDynamicsJoint::kLimitDamping = controllerDef.at("limitDamping").get<double>();
	}
	if ( controllerDef.count("normalCFM") ) {
		ForwardDynamicsJoint::kNormalCFM = controllerDef.at("normalCFM").get<double>();
	}
	if ( controllerDef.count("limitCFM") ) {
		ForwardDynamicsJoint::kLimitCFM = controllerDef.at("limitCFM").get<double>();
	}
	if ( controllerDef.count("limitERP") ) {
		ForwardDynamicsJoint::kLimitERP = controllerDef.at("limitERP").get<double>();
	}
		
	
	array bodyDefs = controllerDef["bones"].get<array>();
	for ( const auto bodyDefValue: bodyDefs ) {
		object bodyDef = bodyDefValue.get<object>();
		
		// get bone
		auto bodyName = bodyDef["name"].get<string>();
		auto bone = mDriveableSkeleton->getBone(bodyName);
		mTotalMass += bone->getMass();
		
		// get children
		vector<Ogre::SharedPtr<DriveableBone> > childBones;
		if ( bodyDef.count("joints") ) {
			auto childJointDefs = bodyDef["joints"].get<object>();
			for ( const auto it: childJointDefs ) {
				childBones.push_back(mDriveableSkeleton->getBone(it.first));
			}
		}
			
		// create the forward dynamics bone object
		auto fdb = Ogre::SharedPtr<ForwardDynamicsBody>( new ForwardDynamicsBody( bone, childBones, this, dynamicsWorld, bodyDefValue ) );
		
		// save
		mBodies[bodyName] = fdb;
		
		// create pd driver
		mPDBodyDrivers[bodyName] = Ogre::SharedPtr<ForwardDynamicsBodyDriverPD>( new ForwardDynamicsBodyDriverPD( fdb ) );
	}
	
	// set rest orientations
	for ( const auto bodyDefValue: bodyDefs ) {
		object bodyDef = bodyDefValue.get<object>();
		auto bodyName = bodyDef["name"].get<string>();
		
		if ( getParentBodyName(bodyName) != "" ) {
			auto parentBody = getParentBody(bodyName);
			auto body = getBody(bodyName);
			Ogre::Quaternion parentOrientationInv = parentBody->getOrientationWorld().Inverse();
			body->setParentRelativeRestOrientation( parentOrientationInv * body->getOrientationWorld() );
		}
	}
		
	
	
	// create constraints
	// the parent contains the joint definition
	for ( const auto bodyDefValue: bodyDefs ) {
		object bodyDef = bodyDefValue.get<object>();
		auto parentName = bodyDef["name"].get<string>();
		
		// get the parent FDB, if it exists
		if ( !mBodies.count(parentName) ) {
			continue;
		}
		auto parentFdb = mBodies[parentName];
		
		if ( bodyDef.count("joints") ) {
			auto childJointDefs = bodyDef["joints"].get<object>();
			for ( const auto it: childJointDefs ) {
				// get the child FDB, if it exists
				string childName = it.first;
				if ( !mBodies.count(childName) ) {
					continue;
				}
				
				auto childFdb = mBodies[childName];
				object jointDef = it.second.get<object>();
				
				auto fdj = Ogre::SharedPtr<ForwardDynamicsJoint>( new ForwardDynamicsJoint( dynamicsWorld, parentFdb, childFdb, jointDef ));
				
				// save
				string fdjName = parentName+"-"+childName;
				mJoints[fdjName] = fdj;
			}
		}
	}
			

	

}

void ForwardDynamicsSkeleton::debugDraw( OgreBulletCollisions::DebugLines* debugLines )
{
	for ( const auto it: mBodies )
	{
		it.second->debugDraw(debugLines);
	}
	
	for ( const auto it: mJoints )
	{
		it.second->debugDraw(debugLines);
	}
	
	for ( const auto it: mPDBodyDrivers )
	{
		it.second->debugDraw(debugLines);

	}
	
	
	/*
	for ( const auto it: mBodies )
	{
		Ogre::Vector3 center = it.second->getWorldPosition();
		btCapsuleShape* capsule = dynamic_cast<btCapsuleShape*>(mCollisionShapes[it.first]);
		float halfLength = 0.1f;
		if ( capsule ) {
			halfLength = capsule->getHalfHeight();
		}
		debugLines->addCross(center, halfLength, Ogre::ColourValue(0.5f, 0.8f, 0.5f));
		
		// draw line from parent to this one, if possible
		auto childBone = mDriveableSkeleton->getBone(it.first)->getBone();
		if ( childBone->getParent() ) {
			string parentName = childBone->getParent()->getName();
			if ( mBodies.count(parentName) ) {
				Ogre::Vector3 childPos = center;
				Ogre::Vector3 parentPos = mBodies[parentName]->getWorldPosition();
				debugLines->addLine( parentPos, childPos, Ogre::ColourValue(1.0f, 0.5f, 0.3f));
			}
		}
	}*/
	
}

void ForwardDynamicsSkeleton::update( float dt )
{
	
	// add PD driver torques
	// this adds torque to the bodies
	for ( auto it: mPDBodyDrivers ) {
		it.second->updateTorque();
	}
	
#ifdef LocoForwardDynamicsJoint_ConvertTorqueToForce
	// pull torque from child bodies and put onto joints instead
	for ( auto it: mJoints ) {
		const Ogre::Vector3& torque = it.second->getChildFdb()->getTorque();
		it.second->addTorque(torque);
		it.second->getChildFdb()->clearTorque();
	}
#endif
	// add joint torques (from gravity compensation)
	for ( auto it: mJoints ) {
		it.second->applyTorque();
	}
	
	// actually apply the torques
	for ( auto it: mBodies ) {
		// this clears the mTorque, also
		it.second->applyTorque();
	}
	
	// copy fdb positions back to skeleton
	
	// order is important
	deque<Ogre::SharedPtr<DriveableBone> > queue;
	// start at the root, work outwards (breadth first)
	auto root = mDriveableSkeleton->getRootBone();
	string firstBoneInForwardDynamics;
	queue.push_back(root);

	bool first = true;
	while ( queue.size()>0 ) {
		auto bone = queue.front();
		queue.pop_front();
		
		// process the bone
		string boneName = bone->getBone()->getName();
		if ( mBodies.count(boneName) ) {
			
			
			auto fdb = mBodies[boneName];
			Ogre::Quaternion orientationW = fdb->getOrientationWorld();
			 
			// update orientation
			/*
			if ( fdb->getNumChildren()==1 ) {
				// the objects might not be perfectly aligned. so, also get the direction vector between parent and child
				string parentName = getParentBodyName(fdb->getName());
				if ( parentName.length()>0 ) {
					string childName = fdb->getAnyChildName();
					if ( mBodies.count(childName) ) {
						auto parentFdb = getBody(parentName);
						auto childFdb = getBody(childName);
						// get the direction vector
						Ogre::Vector3 childParentDelta = parentFdb->getTailPositionWorld(boneName) - childFdb->getHeadPositionWorld();
						Ogre::Vector3 childParentDirection = childParentDelta.normalisedCopy();
						// rotate orientationW to match childParentDirection
						Ogre::Quaternion orientationW = fdb->getOrientationWorld();
						Ogre::Vector3 axes[3];
						orientationW.ToAxes(axes);
						Ogre::Vector3 fdbYAxis = axes[2];
						Ogre::Quaternion q = fdbYAxis.getRotationTo(childParentDirection);
						
						// multiply
						orientationW = q*orientationW;
						// convert back to local
						orientation = fdb->getBody()->getSceneNode()->getParent()->convertWorldToLocalOrientation(orientationW);
					}
				}
			}*/
				
			if ( bone->getBone()->getParent() ) {
				bone->getBone()->setOrientation(bone->getBone()->getParent()->convertWorldToLocalOrientation(orientationW));
			} else {
				bone->getBone()->setOrientation(orientationW);
			}
		
			if ( first ) {
				firstBoneInForwardDynamics = bone->getBone()->getName();
				first = false;
			}

		}
			
		// add children
		for ( unsigned int i= 0; i<bone->getBone()->numChildren(); i++ ) {
			Ogre::Bone* child = dynamic_cast<Ogre::Bone*>(bone->getBone()->getChild(i));
			if ( child && mDriveableSkeleton->hasBone(child->getName())) {
				queue.push_back(mDriveableSkeleton->getBone(child->getName()));
			}
		}
	}

	
	
	
	// move the skeleton's node position to the root position of the skeleton, and cancel the skeleton root's offset
	Ogre::SceneNode* skelRoot = mDriveableSkeleton->getRootSceneNode();
	auto rootBone = mDriveableSkeleton->getBone(firstBoneInForwardDynamics);
	auto rootFdb = mBodies.at(firstBoneInForwardDynamics);
	Ogre::Vector3 rootPosWorld = rootFdb->getHeadPositionWorld();
	// move the root bone to skeleton (0,0,0)
	rootBone->getBone()->setPosition(Ogre::Vector3::ZERO);
	// then move the skeleton's scene node to the old root position.
	skelRoot->_setDerivedPosition(rootPosWorld);

	
}

std::vector<Ogre::SharedPtr<ForwardDynamicsJoint> > ForwardDynamicsSkeleton::getAllJointsWithParent(string parentName)
{
	vector<Ogre::SharedPtr<ForwardDynamicsJoint> > joints;
	for ( const auto& it: mJoints ) {
		if ( it.second->getParentFdb()->getName() == parentName ) {
			joints.push_back(it.second);
		}
	}
	return joints;
}

Ogre::SharedPtr<ForwardDynamicsBody> ForwardDynamicsSkeleton::getBody(string partName)
{
	OgreAssert( mBodies.count(partName), "unrecognized body name" );
	return mBodies.at(partName);
}

Ogre::SharedPtr<ForwardDynamicsJoint> ForwardDynamicsSkeleton::getJointBetween(string parentBodyName, string childBodyName)
{
	string jointName = parentBodyName+"-"+childBodyName;
	OgreAssert( mJoints.count(jointName), "unrecognized parent/child combination");
	return mJoints.at(jointName);
}

Ogre::SharedPtr<ForwardDynamicsJoint> ForwardDynamicsSkeleton::getJointToParent(string childBodyName)
{
	string parentName = getParentBodyName( childBodyName );
	if ( parentName.length()==0 ) {
		return Ogre::SharedPtr<ForwardDynamicsJoint>();
	}
	
	return getJointBetween( parentName, childBodyName );
}

string ForwardDynamicsSkeleton::getParentBodyName(string childBodyName)
{
	string childBoneName = childBodyName;
	auto parentBone = mDriveableSkeleton->getBone(childBoneName)->getBone()->getParent();
	if ( parentBone )
		return parentBone->getName();
	else
		return "";
}

void ForwardDynamicsSkeleton::setCollisionsEnabled( bool enabled )
{
	for ( auto it: mBodies ) {
		int flags = it.second->getBody()->getBulletRigidBody()->getCollisionFlags();
		if ( enabled ) {
			flags &= ~btCollisionObject::CF_NO_CONTACT_RESPONSE;
		} else {
			flags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;
		}
		it.second->getBody()->getBulletRigidBody()->setCollisionFlags(flags);
	}
}

void ForwardDynamicsSkeleton::reset()
{
	Ogre::SceneNode* skelRoot = mDriveableSkeleton->getRootSceneNode();
	for ( auto it: mBodies ) {
		auto bone = mDriveableSkeleton->getBone(it.first);
		it.second->reset( bone, skelRoot );
	}
	for ( auto it: mJoints ) {
		it.second->clearTorque();
	}
}


void ForwardDynamicsSkeleton::addJointTorque( std::string jointName, Ogre::Vector3 torque )
{
	// ogre vector3 does not default to zero
	mJoints.at(jointName)->addTorque(torque);
}

set<string> ForwardDynamicsSkeleton::getAllBodyNames()
{
	set<string> names;
	for ( auto it: mBodies ) {
		names.insert(it.first);
	}
	return names;
}

void ForwardDynamicsSkeleton::setOrientationTarget( std::string bodyName, Ogre::Quaternion orientationWorld )
{
	//Ogre::Quaternion orientationLocal = getBody(bodyName)->convertWorldToLocalOrientation(orientationWorld);
	mPDBodyDrivers.at(bodyName)->setTargetOrientationWorld(orientationWorld);
}

void ForwardDynamicsSkeleton::clearOrientationTarget( std::string bodyName  )
{
	mPDBodyDrivers.at(bodyName)->unsetTargetOrientation();
}

void ForwardDynamicsSkeleton::setAngularVelocityTarget( const std::string& bodyName, Ogre::Vector3& angularVelocityWorld )
{
	mPDBodyDrivers.at(bodyName)->setTargetAngularVelocityWorld(angularVelocityWorld);
}

void ForwardDynamicsSkeleton::clearAngularVelocityTarget( const std::string& bodyName )
{
	mPDBodyDrivers.at(bodyName)->unsetTargetAngularVelocity();
}

Ogre::Vector3 ForwardDynamicsSkeleton::getCenterOfMassWorld()
{
	Ogre::Vector3 CoMWorldAccumulated(0,0,0);
	float massAccumulated = 0;
	for ( const auto it: mBodies ) {
		float mass = it.second->getMass();
		CoMWorldAccumulated += it.second->getCoMWorld()*mass;
		massAccumulated += mass;
	}
	if ( massAccumulated>0 ) {
		return CoMWorldAccumulated / massAccumulated;
	} else {
		return Ogre::Vector3(0,0,0);
	}
}


Ogre::Vector3 ForwardDynamicsSkeleton::getCenterOfMassVelocityWorld()
{
	Ogre::Vector3 accum(0,0,0);
	float massAccum = 0;
	for ( const auto it: mBodies ) {
		float mass = it.second->getMass();
		accum += it.second->getBody()->getLinearVelocity()*mass;
		massAccum += mass;
	}
	if ( massAccum>0 ) {
		return accum / massAccum;
	} else {
		return Ogre::Vector3(0,0,0);
	}
}


