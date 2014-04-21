//
//  PoseController.cpp
//  Loco
//
//  Created on 18/10/13.
//
//

#include "ForwardDynamicsSkeletonController.h"

#include <Ogre/OgreException.h>
#include <Ogre/OgreSkeleton.h>
#include <Ogre/OgreBone.h>
#include <set>
#include <string>
#include "Utilities.h"
#include "TwoLinkIK.h"
#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsRigidBody.h"

using namespace std;
using namespace OgreBulletDynamics;
using namespace OgreBulletCollisions;


ForwardDynamicsSkeletonController::ForwardDynamicsSkeletonController(Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, OgreBulletDynamics::DynamicsWorld* dynamicsWorld, OgreBulletDynamics::RigidBody* groundObject, const picojson::value& jsonSource )
: SkeletonController( skelRootSceneNode, skeleton, jsonSource ), mDynamicsWorld(dynamicsWorld), mGroundBody(groundObject),
mLeftFootInContact(false), mRightFootInContact(false), mLeftFootGroundContactTime(0), mRightFootGroundContactTime(0)
{
	picojson::object jsonRoot = jsonSource.get<picojson::object>();
	// create forward dynamics
	createForwardDynamicsSkeleton(jsonRoot["forwardDynamics"] );
}

ForwardDynamicsSkeletonController::~ForwardDynamicsSkeletonController()
{
	// release all environment constraints
	std::set<std::string> boundBodies;
	for ( const auto& it: mEnvironmentBindingConstraints ) {
		boundBodies.insert(it.first);
	}
	for ( const auto& name: boundBodies ) {
		releaseBodyFromEnvironmentBinding(name);
	}
}

void ForwardDynamicsSkeletonController::update(float dt)
{
	if ( !mEnabled ) {
		return;
	}

	// choose forward dynamics if possible
	if ( !mForwardDynamicsSkeleton.isNull() ) {
		// update
		mForwardDynamicsSkeleton->update(dt);
		// reset the bones (copy current orientations) so that their positions/orientaiotns aren't overwritten
		for ( const auto bn: mDriveableSkeleton->getAllBoneNames() ) {
			mDriveableSkeleton->getBone(bn)->reset();
		}
	}
	
	SkeletonController::update(dt);
	
	// check for contacts
	RigidBody* groundObject = mGroundBody;
	btManifoldPoint result;
	
	// 50ms = contact
	const float FOOT_CONTACT_THRESH_TIME = 0.05f;
	
	// left foot
	RigidBody* leftFootObject = mForwardDynamicsSkeleton->getBody("Foot.L")->getBody();
	float oldLeftFootGroundContactTime = mLeftFootGroundContactTime;
	if ( checkForObjectPairCollision( leftFootObject, groundObject, result ) ) {
		mLeftFootGroundContactTime = MAX(0.0f,mLeftFootGroundContactTime)+dt;
		//BLog(" + left foot contact time %f", mLeftFootGroundContactTime );
		if ( oldLeftFootGroundContactTime<FOOT_CONTACT_THRESH_TIME && mLeftFootGroundContactTime>=FOOT_CONTACT_THRESH_TIME ) {
			if ( !mLeftFootInContact ) {
				btVector3 worldPosOnGround = result.getPositionWorldOnB();
				//BLog("left foot strike at %s", describe(BtOgreConverter::to(worldPosOnGround)).c_str() );
				mLeftFootInContact = true;
				footGroundContactStateChanged( "L", true, BtOgreConverter::to(worldPosOnGround) );
			}
		}
	} else {
		//BLog(" - left foot contact time %f", mLeftFootGroundContactTime );
		mLeftFootGroundContactTime = MAX(0.0f,MIN(mLeftFootGroundContactTime,FOOT_CONTACT_THRESH_TIME)-dt);
		if ( oldLeftFootGroundContactTime>0.0f && mLeftFootGroundContactTime<=0.0f ) {
			if ( mLeftFootInContact ) {
				mLeftFootInContact = false;
				footGroundContactStateChanged( "L", false, Ogre::Vector3::ZERO );
			}
		}
	}
	
	// right foot
	RigidBody* rightFootObject = mForwardDynamicsSkeleton->getBody("Foot.R")->getBody();
	float oldRightFootGroundContactTime = mRightFootGroundContactTime;
	if ( checkForObjectPairCollision( rightFootObject, groundObject, result ) ) {
		mRightFootGroundContactTime = MAX(0.0f,mRightFootGroundContactTime)+dt;
		//BLog(" + right foot contact time %f", mRightFootGroundContactTime );
		if ( oldRightFootGroundContactTime<FOOT_CONTACT_THRESH_TIME && mRightFootGroundContactTime>=FOOT_CONTACT_THRESH_TIME ) {
			if ( !mRightFootInContact ) {
				btVector3 worldPosOnGround = result.getPositionWorldOnB();
				//BLog("right foot strike at %s", describe(BtOgreConverter::to(worldPosOnGround)).c_str() );
				mRightFootInContact = true;
				footGroundContactStateChanged( "R", true, BtOgreConverter::to(worldPosOnGround) );
			}
		}
	} else {
		//BLog(" - right foot contact time %f", mRightFootGroundContactTime );
		mRightFootGroundContactTime = MAX(0.0f,MIN(mRightFootGroundContactTime,FOOT_CONTACT_THRESH_TIME)-dt);
		if ( oldRightFootGroundContactTime>0.0f && mRightFootGroundContactTime<=0.0f ) {
			if ( mRightFootInContact ) {
				//BLog("right foot lift");
				mRightFootInContact = false;
				footGroundContactStateChanged( "R", false, Ogre::Vector3::ZERO );
			}
		}
	}

}

void ForwardDynamicsSkeletonController::footGroundContactStateChanged( const std::string& whichFoot, bool contact, const Ogre::Vector3& contactPos )
{
	BLog("%s foot %s at %s", whichFoot.c_str(), contact?"strike":"lift", describe(contactPos).c_str() );
}


bool ForwardDynamicsSkeletonController::checkForObjectPairCollision( OgreBulletDynamics::RigidBody* objA, OgreBulletDynamics::RigidBody* objB, btManifoldPoint& result )
{
	// local struct to get the result. this is a bit of a mess.
	struct ContactResultCallback: public btCollisionWorld::ContactResultCallback
	{
		bool mCollisionFound;
		btManifoldPoint mCollisionPoint;
		
		ContactResultCallback()
		: btCollisionWorld::ContactResultCallback(),
		mCollisionFound(false)
		{
		}
		
		virtual	btScalar	addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
		{
			mCollisionPoint = cp;
			mCollisionFound = true;
			return 0;
		}
	};
	
	
	ContactResultCallback resultCallback;
	mDynamicsWorld->getBulletDynamicsWorld()->contactPairTest( objA->getBulletObject(), objB->getBulletObject(), resultCallback);
	if ( resultCallback.mCollisionFound ) {
		result = resultCallback.mCollisionPoint;
		return true;
	} else {
		return false;
	}
	
	
}



void ForwardDynamicsSkeletonController::reflectPelvisInSpine( float shoulderMirror, float neckMirror )
{
	auto pelvis = mDriveableSkeleton->getBone("SpineBase");
	vector<Ogre::SharedPtr<DriveableBone> > spine;
	spine.push_back(mDriveableSkeleton->getBone("SpineMid"));
	spine.push_back(mDriveableSkeleton->getBone("SpineTop"));
	vector<float> relativePos;
	// sum the spine bones
	float totalDistance = 0.0f;
	for ( auto b: spine ) {
		totalDistance += b->getBone()->getPosition().length();
	}
	// because the spine base is inverted, add a bit on to the bottom
	float spineBaseHack = 0.25f;
	totalDistance += spineBaseHack;
	// we want to distribute this much over the spine, equally
	Ogre::Quaternion relPelvisOrientation = pelvis->getBone()->getInitialOrientation().Inverse()*pelvis->getBone()->getOrientation();
	// double it
	Ogre::Quaternion qToDistribute = Ogre::Quaternion::Slerp(shoulderMirror, Ogre::Quaternion::IDENTITY, relPelvisOrientation);
	
	// distribute
	float currPos = spineBaseHack;
	float prevPos = 0.0f;
	for ( auto b: spine ) {
		currPos += b->getBone()->getPosition().length();
		float amount = currPos-prevPos;
		float amountPct = amount/totalDistance;
		Ogre::Quaternion q = Ogre::Quaternion::Slerp(amountPct, Ogre::Quaternion::IDENTITY, qToDistribute);
		b->getBone()->setOrientation(b->getBone()->getInitialOrientation()*q);
		//BLog("distributed %f", amountPct);
		prevPos = currPos;
	}
	
	// cancel in neck
	auto neck = mDriveableSkeleton->getBone("Neck");
	auto head = mDriveableSkeleton->getBone("Head");
	Ogre::Quaternion neckCancelQ = Ogre::Quaternion::Slerp(-neckMirror*0.5f, Ogre::Quaternion::IDENTITY, qToDistribute);
	OgreAssert(false, "Not implemented");
	/*
	mBoneDrivers["Neck"]->setTargetOrientation(neck->getBone()->getInitialOrientation()*neckCancelQ);
	mBoneDrivers["Head"]->setTargetOrientation(head->getBone()->getInitialOrientation()*neckCancelQ);*/
	
	
	
}

void ForwardDynamicsSkeletonController::clearLegIKForwardDynamics( string whichLeg )
{
	string upperLegName = "LegUpper."+whichLeg;
	string lowerLegName = "LegLower."+whichLeg;
	string footName = "Foot."+whichLeg;
	mForwardDynamicsSkeleton->clearOrientationTarget(upperLegName);
	mForwardDynamicsSkeleton->clearOrientationTarget(lowerLegName);
	//mForwardDynamicsSkeleton->clearOrientationTarget(footName);
}

void ForwardDynamicsSkeletonController::solveLegIK( const string& whichLeg, const Ogre::Vector3 &footTargetPos, Ogre::Quaternion& upperLegTargetWorld, Ogre::Quaternion& lowerLegTargetWorld, float kneeOut, float kneeUp )
{
	auto pelvis = mForwardDynamicsSkeleton->getBody("SpineBase");
	string upperLegName = "LegUpper."+whichLeg;
	string lowerLegName = "LegLower."+whichLeg;
	string footName = "Foot."+whichLeg;
	auto upperLeg = mForwardDynamicsSkeleton->getBody(upperLegName);
	auto lowerLeg = mForwardDynamicsSkeleton->getBody(lowerLegName);
	auto foot = mForwardDynamicsSkeleton->getBody(footName);
	
	Ogre::Vector3 p1 = upperLeg->getHeadPositionWorld();
	Ogre::Vector3 p2 = footTargetPos;
	
	// calculate a plane normal
	
	// start with global UNIT X, relative to root
	Ogre::Vector3 planeNormal = Ogre::Vector3::UNIT_X;
	// rotate by knee out and knee up angles
	float kneeOutAng = kneeOut*-M_PI_4+(0.25f*M_PI_4);
	if ( whichLeg == "L" ) {
		kneeOutAng = -kneeOutAng;
	}
	float kneeUpAng = kneeUp*-M_PI_4+(0.5f*M_PI_4);
	// note
	Ogre::Quaternion kneeOutQ( Ogre::Radian(kneeOutAng), Ogre::Vector3::UNIT_Y );
	Ogre::Quaternion kneeUpQ( Ogre::Radian(kneeUpAng), Ogre::Vector3::UNIT_X );
	planeNormal = kneeOutQ*kneeUpQ*planeNormal;
	// convert to pelvis space
	Ogre::Vector3 n = /*pelvis->getOrientationWorld().Inverse()**/planeNormal;
	//mDebugNEndpoint_World = pelvis->getBone()->convertLocalToWorldPosition(n);
	
	
	// continue
	Ogre::Vector3 vParent = (upperLeg->getTailPositionLocal()-upperLeg->getHeadPositionLocal());
	Ogre::Vector3 nParent = mKneeAxis; // -UNIT_X
	/*if ( whichLeg=="L") {
		nParent = -nParent;
	}*/
	Ogre::Vector3 vChild = (lowerLeg->getTailPositionLocal()-lowerLeg->getHeadPositionLocal());
	
	Ogre::Quaternion qP;
	Ogre::Quaternion qC;
	TwoLinkIK::getIKOrientations(p1, p2, n, vParent, nParent, vChild, &qP, &qC);
	
	// convert to world orientation and return
	// first qP
	upperLegTargetWorld = qP;
	// then qC
	lowerLegTargetWorld = upperLegTargetWorld*qC;
	
}

void ForwardDynamicsSkeletonController::solveLegIKAndApply( const std::string& whichLeg, const Ogre::Vector3 &footTargetPos, bool preserveFootOrientation, float kneeOut, float kneeUp )
{
	
	Ogre::Quaternion qPWorld, qCWorld;
	solveLegIK( whichLeg, footTargetPos, qPWorld, qCWorld, kneeOut, kneeUp );

	mForwardDynamicsSkeleton->setOrientationTarget("LegUpper."+whichLeg, qPWorld);
	mForwardDynamicsSkeleton->setOrientationTarget("LegLower."+whichLeg, qCWorld);
	if ( preserveFootOrientation ) {
		Ogre::Quaternion footOrientation = mForwardDynamicsSkeleton->getBody("Foot."+whichLeg)->getOrientationWorld();
		mForwardDynamicsSkeleton->setOrientationTarget("Foot."+whichLeg, footOrientation);
	}
	
	return;

}


void ForwardDynamicsSkeletonController::createForwardDynamicsSkeleton( const picojson::value& params )
{
	mForwardDynamicsSkeleton = Ogre::SharedPtr<ForwardDynamicsSkeleton>(new ForwardDynamicsSkeleton( mDynamicsWorld, mDriveableSkeleton, params ));
}

void ForwardDynamicsSkeletonController::debugDraw()
{
	SkeletonController::debugDraw();
	
	// draw target orientations
	/*
	Ogre::ColourValue color = Ogre::ColourValue(0.2,1.0,0.5);
	for( auto it: mBoneDrivers ) {
		string boneName = it.first;
		auto bone = mDriveableSkeleton->getBone(boneName);
		if ( it.second->getOrientationActive() ) {
			Ogre::Vector3 offs(0,0,0.1);
			auto headPos = bone->getHeadPositionWorld();
			auto tailPos = headPos + bone->getBone()->convertLocalToWorldOrientation(it.second->getTargetOrientation())*(bone->getLength()*Ogre::Vector3::UNIT_Y);
			mDebugLines->addLine(headPos+offs, tailPos+offs, color);
		}
	}*/
	
	
	if ( mForwardDynamicsSkeleton.get() ) {
		mForwardDynamicsSkeleton->debugDraw(mDebugLines);
	}
}

void ForwardDynamicsSkeletonController::reset()
{
	// reset skeleton
	mDriveableSkeleton->reset();
	
	// reset forward dynamics
	if ( mForwardDynamicsSkeleton.get() ) {
		mForwardDynamicsSkeleton->reset();
	}
}

void ForwardDynamicsSkeletonController::releaseBodyFromEnvironmentBinding( const std::string& bodyName )
{
	if ( mEnvironmentBindingConstraints.count(bodyName) ) {
		auto binding = mEnvironmentBindingConstraints[bodyName];
		mDynamicsWorld->getBulletDynamicsWorld()->removeConstraint(binding.get());
		mEnvironmentBindingConstraints.erase(bodyName);
	}
}

void ForwardDynamicsSkeletonController::bindBodyToEnvironment(const std::string &bodyName)
{
	if ( mEnvironmentBindingConstraints.count(bodyName) ) {
		releaseBodyFromEnvironmentBinding(bodyName);
	}
	
	auto body = mForwardDynamicsSkeleton->getBody(bodyName);
	
//	Ogre::SharedPtr<btTypedConstraint> binding( new btPoint2PointConstraint( *body->getBody()->getBulletRigidBody(), OgreBtConverter::to(body->getHeadPositionLocal()) ) );
	Ogre::Vector3 axisWorld = body->getHeadPositionWorld();
	
	/*
	Ogre::Vector3 axisInA = body->convertWorldToLocalPosition(axisWorld);
	
	Ogre::Vector3 axisInB = axisWorld;
	// convert axisInB to ground body space if necessary
	if ( mGroundBody->getSceneNode() ) {
		axisInB = mGroundBody->getSceneNode()->convertWorldToLocalPosition(axisWorld);
	}
	Ogre::Quaternion orientationA = body->getOrientationWorld();
	
	btTransform transformA( OgreBtConverter::to(orientationA), OgreBtConverter::to(axisInA) );
	btTransform transformB( btQuaternion::getIdentity(), OgreBtConverter::to(axisInB) );
	// make transform
	// use reference frame B so that we can unbind the Y axis in world space
	
	Ogre::SharedPtr<btTypedConstraint> binding( new btFixedConstraint( *body->getBody()->getBulletRigidBody(), *mGroundBody->getBulletRigidBody(), transformA, transformB ) );
	*/
	
	
	btRigidBody* bodyABody = body->getBody()->getBulletRigidBody();
	btRigidBody* bodyBBody = mGroundBody->getBulletRigidBody();
	
	btTransform constraintWorldTransform = bodyBBody->getWorldTransform();
	constraintWorldTransform.setOrigin(OgreBtConverter::to(axisWorld));
	
	btTransform aFrame = bodyABody->getWorldTransform().inverse() * constraintWorldTransform;
	btTransform bFrame = bodyBBody->getWorldTransform().inverse() * constraintWorldTransform;
	
	Ogre::SharedPtr<btTypedConstraint> binding(  new btFixedConstraint( *bodyABody, *bodyBBody, aFrame, bFrame ) );

	
	/*
	btGeneric6DofConstraint* constraint = (btGeneric6DofConstraint*)(binding.get());
	// lock tx,tz
	constraint->setLimit(0, 0, 0);
	constraint->setLimit(2, 0, 0);
	// free ty
	constraint->setLimit(1, 1, 0);
	// lock rotation
	constraint->setLimit(3, 0, 0);
	constraint->setLimit(4, 0, 0);
	constraint->setLimit(5, 0, 0);*/
	
	mDynamicsWorld->getBulletDynamicsWorld()->addConstraint(binding.get());
	mEnvironmentBindingConstraints[bodyName] = binding;
}


