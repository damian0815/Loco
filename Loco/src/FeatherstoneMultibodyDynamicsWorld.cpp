//
//  FeatherstoneMultibodyDynamicsWorld.cpp
//  Loco
//
//  Created by damian on 28/03/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#include "FeatherstoneMultibodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

FeatherstoneMultibodyDynamicsWorld::FeatherstoneMultibodyDynamicsWorld(Ogre::SceneManager *mgr, const Ogre::AxisAlignedBox &bounds,  const Ogre::Vector3 &gravity, bool init )
: OgreBulletDynamics::DynamicsWorld(mgr,bounds,gravity,false)
{
	if ( init ) {
		// DynamicsWorld already made a constraint solver but it's wrong
		delete mConstraintsolver;
		
		//Use the btMultiBodyConstraintSolver for Featherstone btMultiBody support
		btMultiBodyConstraintSolver* sol = new btMultiBodyConstraintSolver();
		mConstraintsolver = sol;
		
		//use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
		btMultiBodyDynamicsWorld* world = new btMultiBodyDynamicsWorld(mDispatcher, mBroadphase, static_cast<btMultiBodyConstraintSolver*>(mConstraintsolver), &mDefaultCollisionConfiguration);
		mWorld = world;
	}
}
