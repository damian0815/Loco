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
	class RigidBody;
}

#include "SkeletonController.h"
#include <Ogre/OgreVector3.h>
#include "ForwardDynamicsSkeleton.h"
#include <map>

class ForwardDynamicsSkeletonController: public SkeletonController
{
public:
	ForwardDynamicsSkeletonController(Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, OgreBulletDynamics::DynamicsWorld* dynamicsWorld, OgreBulletDynamics::RigidBody* groundBody, const picojson::value& jsonSource );
	virtual ~ForwardDynamicsSkeletonController();
	
	void createForwardDynamicsSkeleton( const picojson::value& params );
	
	virtual void update( float dt );
	
	/*! @brief Reset skeleton to its rest state, reset forward dynamics to rest state. */
	virtual void reset();
	
	
private:
	
	
protected:
	virtual void debugDraw();
	
	/*! @brief Solve leg ik for the given leg ("L" or "R"), for the given foot target pos in world space. */
	void solveLegIKAndApply( const std::string &whichLeg, const Ogre::Vector3 &footTargetPos, bool preserveFootOrientation=false, float kneeOut = 0.25f, float kneeUp = 0.5f );
	void solveLegIK( const std::string& whichLeg, const Ogre::Vector3& footTargetPos, Ogre::Quaternion& upperLegTargetWorld, Ogre::Quaternion& lowerLegTargetWorld, float kneeOut = 0.25f, float kneeUp = 0.5f );
	/*! @brief Clear influence of a previously calculated IK forward dynamics solution for the given leg */
	void clearLegIKForwardDynamics( std::string whichLeg );
	
	
	/*! @brief Reflect the current pelvis orientation in the spine. */
	void reflectPelvisInSpine( float shoulderMirror = 1.5f, float neckMirror = 0.5f );

	/*! @brief Check for a collision between the two objects. */
	bool checkForObjectPairCollision( OgreBulletDynamics::RigidBody* objA, OgreBulletDynamics::RigidBody* objB, btManifoldPoint& result );
	
	/*! @brief Called when the foot contact state changes. Subclasses should override. 
	 @param whichFoot "L" or "R"
	 @param contact true if contact has just occurred, else false.
	 @param contactPos World position where the contact occurred, only valid if contect==true. */
	virtual void footGroundContactStateChanged( const std::string& whichFoot, bool contact, const Ogre::Vector3& contactPos );
	
	/*! @brief Bind the given body to the ground/environment at its current position. */
	void bindBodyToEnvironment( const std::string& bodyName );
	/*! @brief Release a body from its ground/environment binding. */
	void releaseBodyFromEnvironmentBinding( const std::string& bodyName );
	
	Ogre::SharedPtr<ForwardDynamicsSkeleton> mForwardDynamicsSkeleton;

	OgreBulletDynamics::DynamicsWorld* mDynamicsWorld;
	OgreBulletDynamics::RigidBody* mGroundBody;
	
	
	
private:
	
	float mLeftFootGroundContactTime, mRightFootGroundContactTime;
	bool mLeftFootInContact, mRightFootInContact;
	
	std::map<std::string,Ogre::SharedPtr<PoseSnapshot> > mPoseSnapshots;
	
	std::map<std::string,Ogre::SharedPtr<btTypedConstraint> > mEnvironmentBindingConstraints;
	
};

#endif /* defined(__Loco__PoseController__) */
