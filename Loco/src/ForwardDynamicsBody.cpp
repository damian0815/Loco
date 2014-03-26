//
//  ForwardDynamicsBody.cpp
//  Loco
//
//  Created on 30/10/13.
//
//

#include "ForwardDynamicsBody.h"
#include <Ogre/OgreBone.h>
#include <Ogre/OgreAxisAlignedBox.h>
#include "OgreBulletCollisionsCapsuleShape.h"
#include "OgreBulletCollisionsBoxShape.h"
#include "OgreBulletCollisionsSphereShape.h"
#include "OgreBulletConverter.h"
#include "OgreBulletDynamicsRigidBody.h"
#include "ForwardDynamicsSkeleton.h"
#include "OgreBulletCollisionsDebugLines.h"
#include "Utilities.h"

#include <string>
using namespace std;
using namespace OgreBulletDynamics;
using namespace OgreBulletCollisions;

using namespace picojson;



ForwardDynamicsBody::ForwardDynamicsBody( Ogre::SharedPtr<DriveableBone> parentBone, std::vector<Ogre::SharedPtr<DriveableBone> > childBones, ForwardDynamicsSkeleton* owner, OgreBulletDynamics::DynamicsWorld* world,  const value& bodyDefVal )
: mParentBoneName(parentBone->getBone()->getName()), mKp(parentBone->getKp()), mKd(parentBone->getKd()), mMaxTorque(parentBone->getMaxAbsTorque()), mRestOrientationParentRelative(Ogre::Quaternion::IDENTITY), mTorque(0,0,0), mOwnerSkeleton(owner)
{
	object bodyDef = bodyDefVal.get<object>();
	
	
	string shape = bodyDef["shape"].get<string>();
	mCollisionShape = NULL;
	// the position of the center of mass of the collision shape in world coordinates
	
	if ( bodyDef.count("kP") ) {
		mKp = bodyDef.at("kP").get<double>();
	}
	if ( bodyDef.count("kD") ) {
		mKd = bodyDef.at("kD").get<double>();
	}
	if ( bodyDef.count("maxTorque") ) {
		mMaxTorque = bodyDef.at("maxTorque").get<double>();
	}

	mTorqueScale = Ogre::Vector3(1,1,1);
	if ( bodyDef.count("torqueScale") ) {
		mTorqueScale = Serializable::DecodeVector3( bodyDef["torqueScale"] );
	}

	Ogre::Vector3 axis = Ogre::Vector3::UNIT_Y;
	if ( shape == "rod" ) {
		
		OgreAssert(childBones.size()==1, "Wrong number of child bones, should be 1");
		auto childBone = childBones[0];
		OgreAssert(childBone->getBone()->getParent()->getName()==mParentBoneName, "child/parent bone mismatch: can only create links between parents and their immediate children");
		float height = childBone->getBone()->getPosition().length();
		// radius = height * (radius factor)
		float radius = height*0.1f;
		if ( bodyDef.count("radiusFactor") ) {
			radius = bodyDef["radiusFactor"].get<double>()*height;
		} else if ( bodyDef.count("radius") ) {
			radius = bodyDef["radius"].get<double>();
		} else {
			BLog("Bone %s missing radius or radiusFactor for rod shape", mParentBoneName.c_str() );
		}
		// take off the cap sizes and leave a little space
		height -= radius*2.5f;
		
		if ( bodyDef.count("heightFactor") ) {
			height = height*bodyDef["heightFactor"].get<double>();
		}
		
		//height *= 0.5f;
		mCollisionShape = new CapsuleCollisionShape( radius, height, axis );
		
		Ogre::Vector3 parentToChildLocal = childBone->getBone()->getPosition();
		mParentPositionLocal = -parentToChildLocal*0.5f;
		
	} else if ( shape == "box" ) {
		
		Ogre::Vector3 boundsVector(0,0,0);
		if ( bodyDef.count("shapeSize") ) {
			boundsVector = Serializable::DecodeVector3(bodyDef["shapeSize"])*0.5f;
			mParentPositionLocal = Ogre::Vector3(0,0,0);
			
		} else {
			
			// calculate dimensions of the box from the children
			Ogre::AxisAlignedBox aabb;
			// add the parent pos
			aabb.merge(Ogre::Vector3::ZERO);
			// aabb should be in parent bone space
			for ( auto childBone: childBones ) {
				aabb.merge(childBone->getBone()->getPosition());
			}
			
			// create the box shape
			boundsVector = aabb.getHalfSize();
			// create a margin
			mParentPositionLocal = -aabb.getCenter();
			
			boundsVector *= 0.9f;
		}
			
		mCollisionShape = new BoxCollisionShape( boundsVector );
		
		
		
	} else if ( shape == "sphere" ) {
		
		// get sphere radius
		float radius = bodyDef["radius"].get<double>();
		OgreAssert(radius>0, "bad json");
		
		
		// create the sphere shape
		mCollisionShape = new SphereCollisionShape( radius );
		mParentPositionLocal = axis*-radius;
	
		
	} else {
		OgreAssert(false, "Unrecognized bone type");
	}
	
	// offset the center of mass
	Ogre::Vector3 comOffset = Ogre::Vector3::ZERO;
	if ( bodyDef.count("comOffsetLocal") ) {
		array offs = bodyDef["comOffsetLocal"].get<array>();
		comOffset.x = offs[0].get<double>();
		comOffset.y = offs[1].get<double>();
		comOffset.z = offs[2].get<double>();
	}
	mParentPositionLocal -= comOffset;
	

	
	// set parent/child vectors
	for ( auto childBone: childBones ) {
		OgreAssert(childBone->getBone()->getParent()->getName()==mParentBoneName, "child/parent bone mismatch: can only create links between parents and their immediate children");
		Ogre::Vector3 cpL = childBone->getBone()->getPosition()+mParentPositionLocal;
		mChildPositionsLocal[childBone->getBone()->getName()] = cpL;
	}
	
	// create a rigid body
	float mass = parentBone->getMass();
	float restitution = 0.3f;
	float friction = 0.5f;
	if ( bodyDef.count("friction") ) {
		friction = bodyDef["friction"].get<double>();
	}
	if ( bodyDef.count("restitution") ) {
		restitution = bodyDef["restitution"].get<double>();
	}
	
	auto skeletonRootNode = mOwnerSkeleton->getSkeletonRootSceneNode();
	Ogre::Vector3 comPositionSkel = parentBone->getBone()->convertLocalToWorldPosition(-mParentPositionLocal);
	Ogre::Vector3 comPositionW = skeletonRootNode->convertLocalToWorldPosition(comPositionSkel);
	
	Ogre::Quaternion orientationSkel = parentBone->getBone()->convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);
	Ogre::Quaternion orientationW = skeletonRootNode->convertLocalToWorldOrientation(orientationSkel);
	
	
	mBody = new RigidBody( mParentBoneName+".RigidBody", world);
	Ogre::SceneNode* boneNode = world->getSceneManager()->getRootSceneNode()->createChildSceneNode(mParentBoneName+".SceneNode");
	mBody->setShape(boneNode, mCollisionShape, restitution, friction, mass, comPositionW, orientationW );
	// don't let the bone fall asleep
	mBody->disableDeactivation();
	//btTransform transform( OgreBtConverter::to(parentBone->getBone()->_getDerivedOrientation()), OgreBtConverter::to(comPositionW) );
	//mBody->getBulletRigidBody()->setWorldTransform(transform);
	//mBody->setPosition(childBoneCenter);
	//mBody->setOrientation(parentBone->getBone()->_getDerivedOrientation());
	//mBody->setKinematicObject(true);
	
	// disable collisions
	mBody->getBulletRigidBody()->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
	
	// set coll
	//mCollisionShape->getBulletShape()->setMargin(0.01f);
	
	//mBody->showDebugShape(true);
	
	reset( parentBone, skeletonRootNode );
	
}

ForwardDynamicsBody::~ForwardDynamicsBody()
{
	delete mBody;
	delete mCollisionShape;
	mBody->getSceneNode()->getParentSceneNode()->removeAndDestroyChild(mParentBoneName+".SceneNode");
}

Ogre::Vector3 ForwardDynamicsBody::getHeadPositionWorld()
{
	return mBody->getSceneNode()->convertLocalToWorldPosition(mParentPositionLocal);
	
	/*
	const btTransform& transformW = mBody-> getBulletRigidBody()->getCenterOfMassTransform();
	
	btVector3 headPosW = transformW*OgreBtConverter::to( mParentPositionLocal );
	return BtOgreConverter::to(headPosW);*/
}

Ogre::Vector3 ForwardDynamicsBody::getTailPositionLocal(string tailName)
{
	if ( tailName.size()==0 ) {
		OgreAssert( mChildPositionsLocal.size()==1, "cannot automatically find tail name");
		tailName = (*mChildPositionsLocal.begin()).first;
	}
	OgreAssert( mChildPositionsLocal.count(tailName), "Unknown tail name");
	return mChildPositionsLocal[tailName];
}

Ogre::Vector3 ForwardDynamicsBody::getTailPositionWorld(string tailName)
{
	return mBody->getSceneNode()->convertLocalToWorldPosition(getTailPositionLocal(tailName));
	/*
	const btTransform& transformW = mBody->getBulletRigidBody()->getCenterOfMassTransform();
	
	btVector3 tailPosW = transformW*OgreBtConverter::to( getTailPositionLocal(tailName) );
	return BtOgreConverter::to(tailPosW);*/
}

Ogre::Vector3 ForwardDynamicsBody::getCoMWorld()
{
	return mBody->getSceneNode()->convertLocalToWorldPosition(Ogre::Vector3::ZERO);
	/*
	return BtOgreConverter::to(mBody->getBulletRigidBody()->getCenterOfMassPosition());*/
}

/*
Ogre::Quaternion ForwardDynamicsBody::getOrientationLocal()
{
	return mBody->getSceneNode()->getOrientation();
}*/

Ogre::Quaternion ForwardDynamicsBody::getOrientationWorld()
{
	return mBody->getSceneNode()->convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);
}

void ForwardDynamicsBody::debugDraw( OgreBulletCollisions::DebugLines* debugLines )
{
	Ogre::Vector3 head = getHeadPositionWorld();
	// draw to each tail
	for ( const auto it: mChildPositionsLocal ) {
		Ogre::Vector3 tail = getTailPositionWorld(it.first);
		debugLines->addLine( head, tail, Ogre::ColourValue(1.0f, 0.5f, 0.3f));
	}
	
	// draw CoM
	//debugLines->addCross( getCoMWorld(), 0.2f, Ogre::ColourValue(1.0f, 0.3f, 0.5f));
	
	// draw axis at the CoM	
	debugLines->addAxes( getCoMWorld(), getOrientationWorld(), 0.05f);
	
	/*
	auto parent = mOwnerSkeleton->getBody(mParentBoneName);
	if ( !parent.isNull() ) {
		Ogre::Quaternion parentOrientationWorld = parent->getOrientationWorld();
		debugLines->addAxes( getCoMWorld(), parentOrientationWorld*getParentRelativeRestOrientation(), 0.03f);
	}*/
	
}

void ForwardDynamicsBody::reset( Ogre::SharedPtr<DriveableBone> parentBone, Ogre::SceneNode* skeletonRootNode )
{
	Ogre::Vector3 comPositionSkel = parentBone->getBone()->convertLocalToWorldPosition(-mParentPositionLocal);
	Ogre::Vector3 comPositionW = skeletonRootNode->convertLocalToWorldPosition(comPositionSkel);
	
	Ogre::Quaternion orientationSkel = parentBone->getBone()->convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);
	Ogre::Quaternion orientationW = skeletonRootNode->convertLocalToWorldOrientation(orientationSkel);
	
	mBody->setPosition(comPositionW);
	mBody->setOrientation(orientationW);
	btTransform worldTrans(OgreBtConverter::to(orientationW), OgreBtConverter::to(comPositionW));
	mBody->getBulletRigidBody()->setWorldTransform(worldTrans);
	// clear velocities
	mBody->getBulletRigidBody()->setAngularVelocity(btVector3(0,0,0));
	mBody->getBulletRigidBody()->setLinearVelocity(btVector3(0,0,0));
	mBody->getBulletRigidBody()->clearForces();
	
	clearTorque();
	
}

std::string ForwardDynamicsBody::getAnyChildName()
{
	for ( auto it: mChildPositionsLocal ) {
		return it.first;
	}
	// none found
	return "";
}

void ForwardDynamicsBody::limitTorque( Ogre::Vector3& t )
{
	// convert to local coordinates
	t = getOrientationWorld().Inverse()*t;
	
	// scale and limit
	t.x = mTorqueScale.x * min(max(t.x,-mMaxTorque),mMaxTorque);
	t.y = mTorqueScale.y * min(max(t.y,-mMaxTorque),mMaxTorque);
	t.z = mTorqueScale.z * min(max(t.z,-mMaxTorque),mMaxTorque);
	
	// convert back to global coordinates
	t = getOrientationWorld()*t;
}

void ForwardDynamicsBody::applyTorque()
{
	// clamp torque
	Ogre::Vector3 t = mTorque;
	
	limitTorque(t);
	
	mBody->getBulletRigidBody()->applyTorque(OgreBtConverter::to(t));

	/*if ( getName()=="SpineBase" ) {
		BLog("torque: %s  rate of change: %s", describe(mTorque).c_str(), describe(mTorque-mPrevTorque).c_str() );
	}*/
	clearTorque();
}

float ForwardDynamicsBody::getMass()
{
	float invMass = mBody->getBulletRigidBody()->getInvMass();
	
	if ( fabsf(invMass)<FLT_EPSILON )
		return 0;
	else
		return 1.0f/invMass;
}

void ForwardDynamicsBody::addImpulse( const Ogre::Vector3& impulse )
{
	mBody->getBulletRigidBody()->applyCentralImpulse(OgreBtConverter::to(impulse));
}

/**
 This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
 resulting velocity will be expressed in world coordinates.
 */
Ogre::Vector3 ForwardDynamicsBody::getAbsoluteVelocityForLocalPoint(const Ogre::Vector3& localPoint){
	//we need to compute the vector r, from the origin of the body to the point of interest
	btVector3 r = OgreBtConverter::to(localPoint);
	//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
	btVector3 v = getBody()->getBulletRigidBody()->getAngularVelocity().cross(r) + getBody()->getBulletRigidBody()->getLinearVelocity();
	return BtOgreConverter::to(v);
}


