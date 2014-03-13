//
//  VirtualModelSkeletonController.h
//  Loco
//
//  Created on 07/11/13.
//
//

#ifndef __Loco__VirtualModelSkeletonController__
#define __Loco__VirtualModelSkeletonController__

#include "ForwardDynamicsSkeletonController.h"
#include "VirtualModelFSM.h"
#include "VirtualModelMotionGenerator.h"

#include <iostream> 
class btPoint2PointConstraint;

class VirtualModelSkeletonController: public ForwardDynamicsSkeletonController
{
public:
	VirtualModelSkeletonController( Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, OgreBulletDynamics::DynamicsWorld* dynamicsWorld, OgreBulletDynamics::RigidBody* groundPlaneBody, const picojson::value& jsonSource );
	virtual ~VirtualModelSkeletonController() {};
	
	void pushModel(Ogre::Vector3 force);
	
	virtual void update(float dt);
	
	virtual void debugDraw();
	
private:
	
	/*! @abstract Compute body target orientations from the finite state machine and apply as PD targets to the bodies. */
	void evaluateMotionTargets( float deltaTime );
	
	std::string getStanceLegSuffix() { return (mMotionGenerator->getStanceIsLeft()?"L":"R"); }
	std::string getSwingLegSuffix() { return (!mMotionGenerator->getStanceIsLeft()?"L":"R"); }
	
	void computeGravityCompensationTorques( );
 
	/*! @abstract Compute the torques that mimick the effect of applying a force on a rigid body, at some point. It works best if the end joint is connected to something that is grounded, otherwise (I think) this is just an approximation.
		@arg startName The joint to start at. The function will work backwards toward the root. 
		@arg pLocal Where to apply the force, in joint-local coordinates.
		@arg fGlobal The force to apply in global coordinates. 
		@arg torques Output torques to apply
	 */
	void computeJointTorquesEquivalentToForce(std::string startName, const Ogre::Vector3& pLocal, const Ogre::Vector3& fGlobal /*, std::string endName=""*/);
	
	Ogre::Vector3 computeCoMVirtualForce();
	void COMJT();
	
	bool mDoGravityCompensation, mDoCoMVirtualForce, mDoFootIK;
	float mGravityCompensationFactor, mCoMVirtualForceFactor;
	
	float mCoMkP, mCoMkD;
	Ogre::Vector3 mPrevCoM, mCoMVelocity;
	
	float mPelvisHeightAboveFeet;

	Ogre::Vector3 mFootTargetL, mFootTargetR, mCoMTarget;
	float mLeftKneeOut, mRightKneeOut;
	Ogre::Vector3 mDebugCoMVirtualForce;
	
	bool mDoubleStance;
	float mFootIKPlacementWidth;
	
	OgreBulletDynamics::RigidBody* mGroundPlaneBody;
	
	
	Ogre::SharedPtr<VirtualModelMotionGenerator> mMotionGenerator;
	
	Ogre::SharedPtr<VirtualModelFSM> mFiniteStateMachine;
	
};

#endif /* defined(__Loco__VirtualModelSkeletonController__) */
