//
//  SkeletonController.cpp
//  Loco
//
//  Copyright (c) 2013 bg. All rights reserved.
//

#include "SkeletonController.h"
#include <set>
#include <Ogre/OgreSkeleton.h>
#include <Ogre/OgreBone.h>
#include <Ogre/OgreSceneNode.h>
#include "OgreBulletCollisionsDebugLines.h"
#include "Utilities.h"
#include "TwoLinkIK.h"

using namespace std;

SkeletonController::SkeletonController( Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, const picojson::value& jsonSource )
: mEnabled(false), mDebugLines(0)

{
	
	picojson::value skelParams = jsonSource.get<picojson::object>().at("skeleton");
	mDriveableSkeleton = Ogre::SharedPtr<DriveableSkeleton>(new DriveableSkeleton(skeleton, skelRootSceneNode, skelParams));
	
	// load knee angle
	picojson::value kneeAxisValue = jsonSource.get<picojson::object>().at("kneeAxis");
	mKneeAxis = Serializable::DecodeVector3(kneeAxisValue);
	
	// fans
	createFans();
	
	// breathing driver
	if ( mDriveableSkeleton->hasBone("SpineTop.Breath") )
		mBreathingDriver = Ogre::SharedPtr<BoneDriverBreath>( new BoneDriverBreath(mDriveableSkeleton->getBone("SpineTop.Breath"), mDriveableSkeleton->getBone("SpineTop"), Ogre::Vector3::UNIT_Y, Ogre::Vector3::UNIT_X) );
	
}

SkeletonController::~SkeletonController()
{
	setShowDebugInfo(false);
}

void SkeletonController::createFans()
{
	// create fan bone drivers
	Ogre::Skeleton::BoneIterator it = mDriveableSkeleton->getSkeleton()->getBoneIterator();
	while ( it.hasMoreElements() ) {
		Ogre::Bone* b = it.getNext();
		string name = b->getName();
		if ( name.find("Fan.")==0 ) {
			bool gotFan = false;
			// find index of next .
			string::size_type pctStart = 4;
			string::size_type nextDot = name.find(".", pctStart);
			
			if ( nextDot != std::string::npos ) {
				string percent = name.substr( pctStart, nextDot-pctStart );
				int pct = strtol(percent.c_str(), NULL, 0);
				string::size_type finalDot = name.find(".", nextDot);
				
				if ( finalDot != std::string::npos ) {
					
					string sourceName = name.substr( finalDot+1 );
					BLog("found bone %15s -> %3i%% of %s", name.c_str(), pct, sourceName.c_str() );
					// make the fan using a BoneDriverCopy
					Ogre::Bone* source = mDriveableSkeleton->getBone(sourceName)->getBone();
					Ogre::Bone* target = b;
					float pctFloat = (float)pct/100.0f;
					auto fan = Ogre::SharedPtr<BoneDriverCopy>( new BoneDriverCopy( source, target, pctFloat ) );
					mFanBones.push_back(fan);
					gotFan = true;
				}
			}
			if ( !gotFan ) {
				BLog("found bone %15s but couldn't decode to fan", name.c_str() );
			}
		}
	}

}


std::string SkeletonController::serialize()
{
	return mDriveableSkeleton->serialize();
}

void SkeletonController::setEnabled( bool tf )
{
	Ogre::Skeleton* skel = mDriveableSkeleton->getSkeleton();
	auto it = skel->getBoneIterator();
	while ( it.hasMoreElements() ) {
		// set the bone to manually controlled
		Ogre::Bone* b = it.getNext();
		b->setManuallyControlled(tf);
	}
	
	if ( tf && !mEnabled ) {
		// reset all the bones
		set<string> boneNames = mDriveableSkeleton->getAllBoneNames();
		for ( auto bn: boneNames ) {
			mDriveableSkeleton->getBone(bn)->reset();
		}
	}
	
	mEnabled = tf;
}

void SkeletonController::update(float dt)
{
	if ( !mEnabled ) {
		return;
	}
	
	// apply the angular velocities
	mDriveableSkeleton->update(dt);

	// update fans
	for ( auto fan: mFanBones ) {
		fan->update();
	}
	// update breathing
	if ( mBreathingDriver.get() )
		mBreathingDriver->update(dt);
	
	// update debug lines
	if ( mDebugLines ) {
		mDebugLines->clear();
		debugDraw();
		mDebugLines->draw();
	}
	
}

void SkeletonController::debugDraw()
{
	// show bones
	auto allBones = mDriveableSkeleton->getAllBoneNames();
	for ( auto boneName: allBones ) {
		auto bone = mDriveableSkeleton->getBone(boneName);
		bone->debugDraw( mDebugLines, Ogre::ColourValue(0.3f, 0.3f, 0.3f ), 0.05f );
	}
	
	for ( auto fan: mFanBones ) {
		fan->debugDraw( mDebugLines, Ogre::ColourValue(0.8f, 0.3f, 0.8f ), 0.05f );
	}
	
	// show center of mass
	Ogre::Vector3 CoM = mDriveableSkeleton->getCenterOfMassWorld();
	mDebugLines->addCross( CoM, 0.01f, Ogre::ColourValue( 1.0f, 1.0f, 0.0f ) /* yellow */ );
	
	/*
	mDebugLines->addCross( mDebugFootTarget_World, 0.01f, Ogre::ColourValue( 0.0f, 1.0f, 1.0f )  );
	
	mDebugLines->addLine(mDriveableSkeleton->getBone("LegUpper.L")->getBone()->_getDerivedPosition(), mDebugKneeTarget_World, Ogre::ColourValue( 1.0f, 0.0f, 1.0f ) );
	//mDebugLines->addLine(mDebugFootTarget_World, mDebugKneeTarget_World, Ogre::ColourValue( 1.0f, 0.0f, 1.0f ) );
	mDebugLines->addLine(mDriveableSkeleton->getBone("SpineBase")->getBone()->_getDerivedPosition(), mDebugNEndpoint_World, Ogre::ColourValue( 1,1,1 ) );
	 */
}

void SkeletonController::setShowDebugInfo( bool tf )
{
	if ( tf && !mDebugLines ) {
		mDebugLines = new OgreBulletCollisions::DebugLines();
		mDriveableSkeleton->getRootSceneNode()->attachObject(mDebugLines);
		
	} else if ( !tf && mDebugLines ) {
		mDriveableSkeleton->getRootSceneNode()->detachObject(mDebugLines);
		delete mDebugLines;
		mDebugLines = NULL;
	}
	
	
}

