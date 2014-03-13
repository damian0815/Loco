//
//  BoneDriverBreath.h
//  Loco
//
//  Created on 27/10/13.
//
//

#ifndef __Loco__BoneDriverBreath__
#define __Loco__BoneDriverBreath__

#include <iostream>

#include <Ogre/OgreSharedPtr.h>
#include <Ogre/OgreVector3.h>
#include <Ogre/OgreSharedPtr.h>
#include "DriveableBone.h"

class BoneDriverBreath
{
public:
	BoneDriverBreath( Ogre::SharedPtr<DriveableBone> breathBone, Ogre::SharedPtr<DriveableBone> upperSpineBone, const Ogre::Vector3& forwardAxis, const Ogre::Vector3& rightAxis );
	
	void update(float dt);
	
private:
	
	float mPhase; // 0..1 is a complete cycle
	
	Ogre::SharedPtr<DriveableBone> mBreathBone;
	Ogre::SharedPtr<DriveableBone> mUpperSpineBone;
	
	Ogre::Vector3 mForward;
	Ogre::Vector3 mRight;
	
};

#endif /* defined(__Loco__BoneDriverBreath__) */
