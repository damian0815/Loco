//
//  PoseSnapshot.cpp
//  Loco
//
//  Created on 18/10/13.
//
//

#include "PoseSnapshot.h"
#include "DriveableSkeleton.h"
#include <Ogre/OgreBone.h>
#include "Utilities.h"

PoseSnapshot::PoseSnapshot( Ogre::SharedPtr<DriveableSkeleton> sourceSkeleton, set<string> bonesToSnapshot )
{
	if ( bonesToSnapshot.size()==0 ) {
		bonesToSnapshot = sourceSkeleton->getAllBoneNames();
	}
	
	for ( string boneName: bonesToSnapshot ) {
		auto db = sourceSkeleton->getBone(boneName);
		OgreAssert(db.get(), "No matching bone");
		Ogre::Quaternion q = db->getBone()->getOrientation();
		addOrientation(boneName, q);
		BLog("Snapshotted %10s -> %s", boneName.c_str(), describe(q).c_str() );
	}
}


void PoseSnapshot::addOrientation( std::string boneName, Ogre::Quaternion q )
{
	mOrientations[boneName] = q;
}

picojson::value PoseSnapshot::serialize() const
{
	return picojson::value();
}

void PoseSnapshot::deserialize(picojson::value &source)
{
}
