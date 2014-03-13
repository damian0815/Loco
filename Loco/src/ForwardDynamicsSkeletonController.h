//
//  PoseController.h
//  Loco
//
//  Created on 18/10/13.
//
//

#ifndef __Loco__PoseController__
#define __Loco__PoseController__

namespace Ogre {
	class Skeleton;
	class SceneNode;
};

namespace OgreBulletDynamics {
	class DynamicsWorld;
}

#include "SkeletonController.h"
#include <Ogre/OgreVector3.h>
#include "ForwardDynamicsSkeleton.h"
#include <map>

class ForwardDynamicsSkeletonController: public SkeletonController
{
public:
	ForwardDynamicsSkeletonController(Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, OgreBulletDynamics::DynamicsWorld* dynamicsWorld, const picojson::value& jsonSource );
	virtual ~ForwardDynamicsSkeletonController() {};
	
	void createForwardDynamicsSkeleton( const picojson::value& params );
	
	virtual void update( float dt );
	
	/*! @abstract Reset skeleton to its rest state, reset forward dynamics to rest state. */
	virtual void reset();
	
private:
	
	
protected:
	virtual void debugDraw();
	
	/*! @abstract Solve leg ik for the given leg ("L" or "R"), for the given foot target pos in world space. */
	void solveLegIKForwardDynamics( std::string whichLeg, Ogre::Vector3 footTargetPos, float pelvisHeightAboveFeetTarget=-1.0f, bool preserveFootOrientation=false, float kneeOut = 0.25f, float kneeUp = 0.5f );
	/*! @abstract Clear influence of a previously calculated IK forward dynamics solution for the given leg */
	void clearLegIKForwardDynamics( std::string whichLeg );
	
	
	/*! @abstract Reflect the current pelvis orientation in the spine. */
	void reflectPelvisInSpine( float shoulderMirror = 1.5f, float neckMirror = 0.5f );

	
	Ogre::SharedPtr<ForwardDynamicsSkeleton> mForwardDynamicsSkeleton;

	OgreBulletDynamics::DynamicsWorld* mDynamicsWorld;
	
private:
	
	std::map<std::string,Ogre::SharedPtr<PoseSnapshot> > mPoseSnapshots;
	
};

#endif /* defined(__Loco__PoseController__) */
