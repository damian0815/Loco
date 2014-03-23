//
//  BoneDriverCopy.h
//  Loco
//
//  Created on 27/10/13.
//
//

#ifndef __Loco__BoneDriverCopy__
#define __Loco__BoneDriverCopy__

#include <iostream>

namespace Ogre {
	class Bone;
}

namespace OgreBulletCollisions {
	class DebugLines;
}

#include <Ogre/OgreColourValue.h>
#include <Ogre/OgreVector3.h>

class BoneDriverCopy
{
public:
	BoneDriverCopy( Ogre::Bone* sourceBone, Ogre::Bone* targetBone, float orientationCopyPct );
	
	void debugDraw( OgreBulletCollisions::DebugLines* debugLines, const Ogre::ColourValue& colour, float axisSize, const Ogre::Vector3& offset=Ogre::Vector3::ZERO );
	void update();
	
private:
	
	Ogre::Bone* mSource;
	Ogre::Bone* mTarget;
	float mOrientationCopyPct;
	
	
	
};

#endif /* defined(__Loco__BoneDriverCopy__) */
