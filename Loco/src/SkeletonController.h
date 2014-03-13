//
//  SkeletonController.h
//  Loco
//
//  Copyright (c) 2013 bg. All rights reserved.
//

#ifndef __Loco__SkeletonController__
#define __Loco__SkeletonController__

#include <iostream>
#include <string>
#include <map>
#include <vector>
#include "DriveableSkeleton.h"
#include <Ogre/OgreSharedPtr.h>
#include "BoneDriverCopy.h"
#include "BoneDriverBreath.h"


namespace Ogre {
	class Skeleton;
	class SceneNode;
};

namespace OgreBulletCollisions {
	class DebugLines;
};


class SkeletonController
{
public:
	SkeletonController( Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skel, const picojson::value& jsonSource );
	virtual ~SkeletonController() ;
		
	/*! @abstract Enable/disable this controller.
	 @discussion Walks through the skeleton and turns on or off manual control for each of the bones in the driveable skeleton. If enabling, client should make sure that the skeleton is not currently animated (ie, all animation states should be disabled). */
	void setEnabled( bool tf );
	/*! @abstract Disabled by default. */
	bool isEnabled() { return mEnabled; }
	
	/*! @abstract Update the driveable skeleton. Has no effect if isEnabled() is false. */
	virtual void update( float dt );
	
	/*! @abstract Return a json representation that can be loaded using the jsonSource constructor. */
	std::string serialize();
	
	void setShowDebugInfo( bool show );
	bool getShowDebugInfo() { return mDebugLines != NULL; }
	
	virtual void debugDraw();
	
protected:
	
	/*! @abstract Create fan bones by searching bone tree for 'Fan.xx.yyyyy', meaning copy xx percent of bone yyyyy's orientation. */
	void createFans();
	
	Ogre::SharedPtr<DriveableBone> getBone(std::string name) { return mDriveableSkeleton->getBone(name); }
	
	
	bool mEnabled;
		
	Ogre::SharedPtr<DriveableSkeleton> mDriveableSkeleton;
	std::vector<Ogre::SharedPtr<BoneDriverCopy> > mFanBones;
	Ogre::SharedPtr<BoneDriverBreath> mBreathingDriver;
	
	
	/*Ogre::Vector3 mDebugFootTarget_World;
	Ogre::Vector3 mDebugKneeTarget_World;
	Ogre::Vector3 mDebugNEndpoint_World;*/
	OgreBulletCollisions::DebugLines* mDebugLines;
	
	Ogre::Vector3 mKneeAxis;
	
	
};

#endif /* defined(__Loco__SkeletonController__) */
