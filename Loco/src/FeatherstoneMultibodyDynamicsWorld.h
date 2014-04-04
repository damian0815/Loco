//
//  FeatherstoneMultibodyDynamicsWorld.h
//  Loco
//
//  Created by damian on 28/03/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#ifndef __Loco__FeatherstoneMultibodyDynamicsWorld__
#define __Loco__FeatherstoneMultibodyDynamicsWorld__

#include <iostream>

#include "OgreBulletDynamicsWorld.h"

class FeatherstoneMultibodyDynamicsWorld: public OgreBulletDynamics::DynamicsWorld
{
public:
    FeatherstoneMultibodyDynamicsWorld(Ogre::SceneManager *mgr, const Ogre::AxisAlignedBox &bounds, const Ogre::Vector3 &gravity, bool init=true );

protected:
	
};


#endif /* defined(__Loco__FeatherstoneMultibodyDynamicsWorld__) */

