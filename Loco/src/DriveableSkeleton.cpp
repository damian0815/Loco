//
//  DriveableSkeleton.cpp
//  Loco
//
//  Created on 18/10/13.
//
//

#include "DriveableSkeleton.h"

#include "Utilities.h"
#include <Ogre/OgreSkeleton.h>
#include <Ogre/OgreBone.h>
#include <Ogre/OgreSceneNode.h>
#include "picojson.h"

using namespace std;
using namespace picojson;

#include <deque>

 /*
      Kinesiology http://www.exrx.net/Kinesiology/Segments.html
      Percentages of total body weight
 
     Segment         Males   Females Average
     Head            8.26    8.2     8.23
     Whole Trunk     55.1    53.2    54.15
     Thorax          20.1    17.02   18.56
     Abdomen         13.06   12.24   12.65
     Pelvis          13.66   15.96   14.81
     Total Arm       5.7     4.97    5.335
     Upper Arm       3.25    2.9     3.075
     Forearm         1.87    1.57    1.72
     Hand            0.65    0.5     0.575
     Forearm & Hand  2.52    2.07    2.295
     Total Leg       16.68   18.43   17.555
     Thigh           10.5    11.75   11.125
     Leg             4.75    5.35    5.05
     Foot            1.43    1.33    1.38
     Leg & Foot      6.18    6.68    6.43
      */


DriveableSkeleton::DriveableSkeleton( Ogre::Skeleton* skeleton, Ogre::SceneNode* sceneNode, const picojson::value& jsonRoot )
: mSkeleton(skeleton), mSkeletonRootNode(sceneNode), mSkeletonRootRestPosition(sceneNode->getPosition()), mSkeletonRootRestOrientation(sceneNode->getOrientation())
{
	
	// root is a picojson::object
	object jsonRootObject = jsonRoot.get<object>();
	array jsonBones = jsonRootObject["bones"].get<array>();
	for ( auto jB: jsonBones ) {
		object boneObject = jB.get<object>();
		// get the bone
		string boneName = boneObject["name"].get<string>();
		Ogre::Bone* bone = mSkeleton->getBone(boneName);
		OgreAssert(bone, "Bad bone name in json");
		
		value params = boneObject["params"];
		
		// construct the driveable bone and store locally
		auto db = Ogre::SharedPtr<DriveableBone>( new DriveableBone(bone, params) );
		mDriveableBones[boneName] = db;
	}
	
	/*
	// also dump the skeleton's root bones
	Ogre::Skeleton::BoneIterator it = mSkeleton->getRootBoneIterator();
	while ( it.hasMoreElements() ) {
		Ogre::Bone* root = it.getNext();
		BLog("found root %s with parent %x", root->getName().c_str(), root->getParent() );
	}*/
}

string DriveableSkeleton::serialize()
{
	object jsonRootObject;
	array bones;
	for ( auto db: mDriveableBones ) {
		object boneObject;
		boneObject["name"] = value(db.first);
		value params = db.second->serialize();
		boneObject["params"] = params;
		bones.push_back(value(boneObject));
	}
	// store on root object
	jsonRootObject["bones"] = value(bones);
	
	return value(jsonRootObject).serialize();
}

set<string> DriveableSkeleton::getAllBoneNames() const
{
	set<string> names;
	for ( const auto it: mDriveableBones ) {
		names.insert(it.first);
	}
	return names;
}

bool DriveableSkeleton::hasBone( string name ) const
{
	return mDriveableBones.count(name);
}

Ogre::SharedPtr<DriveableBone> DriveableSkeleton::getBone( string name ) const
{
	return mDriveableBones.at(name);
}

Ogre::SharedPtr<DriveableBone> DriveableSkeleton::getRootBone() const
{
	Ogre::Skeleton::BoneIterator it = mSkeleton->getRootBoneIterator();
	while ( it.hasMoreElements() ) {
		Ogre::Bone* b = it.getNext();
		string name = b->getName();
		if ( hasBone(name) ) {
			return getBone(name);
		}
	}
	OgreAssert(false, "couldn't find a root bone");
}

void DriveableSkeleton::update( float dt )
{
	for ( const auto it: mDriveableBones ) {
		it.second->update(dt);
	}
	
}

Ogre::Vector3 DriveableSkeleton::getCenterOfMassWorld() const
{
	Ogre::Vector3 accumulated(0,0,0);
	float accumulatedMass = 0.0f;
	for ( const auto db: mDriveableBones ) {
		Ogre::Vector3 boneComWorld = db.second->getCenterOfMassWorld();
		float mass = db.second->getMass();
		accumulated += boneComWorld*mass;
		accumulatedMass += mass;
	}
	// divide by accumulated mass
	Ogre::Vector3 comWorld(0,0,0);
	if ( accumulatedMass>0.0f ) {
		comWorld = accumulated/accumulatedMass;
	}
	return comWorld;/*mSkeletonRootNode->convertLocalToWorldPosition(comWorld)*/;
	
}

void DriveableSkeleton::reset()
{
	mSkeleton->reset(true);
	
	mSkeletonRootNode->setPosition(mSkeletonRootRestPosition);
	mSkeletonRootNode->setOrientation(mSkeletonRootRestOrientation);
	for ( auto db: mDriveableBones ) {
		db.second->reset();
	}
}
