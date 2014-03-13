//
//  PoseSnapshot.h
//  Loco
//
//  Created on 18/10/13.
//
//

#ifndef __Loco__PoseSnapshot__
#define __Loco__PoseSnapshot__

#include <iostream>
#include <string>
#include <set>
#include <map>
#include <Ogre/OgreQuaternion.h>
#include <Ogre/OgreSharedPtr.h>
#include "Serializable.h"

using namespace std;

class DriveableSkeleton;

class PoseSnapshot: public Serializable
{
public:
	PoseSnapshot() {};
	/*! @abstract Create a pose snapshot from the given source skeleton. If specified, bonesToSnapshot says which bones to take, otherwise take all found. */
	PoseSnapshot( Ogre::SharedPtr<DriveableSkeleton> sourceSkeleton, set<string> bonesToSnapshot = set<string>() );
	
	/*! @abstract Add the given bone's given orientation to the snapshot. If an orientation exists for this bone, overwrite it. */
	void addOrientation( std::string boneName, Ogre::Quaternion boneLocalOrientation );
	
	picojson::value serialize() const;
 	void deserialize(picojson::value& source);
	
private:
	map<string,Ogre::Quaternion> mOrientations;
	
	friend class ForwardDynamicsSkeletonController;
	
};

#endif /* defined(__Loco__PoseSnapshot__) */
