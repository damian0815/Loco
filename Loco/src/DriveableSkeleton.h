//
//  DriveableSkeleton.h
//  Loco
//
//  Created on 18/10/13.
//
//

#ifndef __Loco__DriveableSkeleton__
#define __Loco__DriveableSkeleton__

#include <iostream>
#include "DriveableBone.h"
#include "BoneDriverPD.h"
#include "PoseSnapshot.h"
#include <Ogre/OgreSharedPtr.h>
#include <set>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "picojson.h"

namespace Ogre {
	class Skeleton;
	class SceneNode;
};

class DriveableSkeleton
{
public:
	/*! @abstract Create driven bones as defined in jsonParams. 
	 @arg skeletonRootNode The scene node that the skeleton's root joint is parented to. */
	DriveableSkeleton( Ogre::Skeleton* skeleton, Ogre::SceneNode* skeletonRootNode, const picojson::value& jsonParams );
	
	set<string> getAllBoneNames() const;
	
	bool hasBone( string name ) const;
	Ogre::SharedPtr<DriveableBone> getBone( string name ) const;
	Ogre::SharedPtr<DriveableBone> getRootBone() const;
	
	/*! @abstract Apply each of the angular velocities for each member driveableBone over dt. */
	void update(float dt);
	
	/*! @abstract Return the center of mass in world space. */
	Ogre::Vector3 getCenterOfMassWorld() const;
	
	PoseSnapshot snapshotPose();
	
	/*! @abstract Reset all bones to their binding positions. */
	void reset();
	
	Ogre::Skeleton* getSkeleton() { return mSkeleton; }
	
	std::string serialize();
	
	/*! @return The scene node that the skeleton's root joint is parentd to. */
	Ogre::SceneNode* getRootSceneNode() { return mSkeletonRootNode; }
	
private:
	
	std::map<std::string,Ogre::SharedPtr<DriveableBone> > mDriveableBones;
	
	Ogre::Skeleton* mSkeleton;
	Ogre::SceneNode* mSkeletonRootNode;
	Ogre::Quaternion mSkeletonRootRestOrientation;
	Ogre::Vector3 mSkeletonRootRestPosition;
};


#endif /* defined(__Loco__DriveableSkeleton__) */
