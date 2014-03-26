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


ForwardDynamicsSkeletonController::ForwardDynamicsSkeletonController(Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, OgreBulletDynamics::DynamicsWorld* dynamicsWorld, const picojson::value& jsonSource )
: SkeletonController( skelRootSceneNode, skeleton, jsonSource ), mDynamicsWorld(dynamicsWorld)
{
	picojson::object jsonRoot = jsonSource.get<picojson::object>();
	// create forward dynamics
	createForwardDynamicsSkeleton(jsonRoot["forwardDynamics"] );
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


