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
	
	/*! @brief Compute body target orientations from the finite state machine and apply as PD targets to the bodies. */
	void evaluateMotionTargets( float deltaTime );
	
	bool getStanceIsLeft() { return mMotionGenerator->getStanceIsLeft(); }
	std::string getStanceLegSuffix() { return (mMotionGenerator->getStanceIsLeft()?"L":"R"); }
	std::string getSwingLegSuffix() { return (!mMotionGenerator->getStanceIsLeft()?"L":"R"); }
	
	Ogre::SharedPtr<ForwardDynamicsJoint> getStanceAnkle();
	Ogre::SharedPtr<ForwardDynamicsJoint> getSwingAnkle();
	Ogre::SharedPtr<ForwardDynamicsJoint> getSwingHip();
	Ogre::SharedPtr<ForwardDynamicsJoint> getStanceHip();
	
	void computeGravityCompensationTorques( );
 
	/*! @brief Compute the torques that mimick the effect of applying a force on a rigid body, at some point. It works best if the end joint is connected to something that is grounded, otherwise (I think) this is just an approximation.
		@arg startName The joint to start at. The function will work backwards toward the root. 
		@arg pLocal Where to apply the force, in joint-local coordinates.
		@arg fGlobal The force to apply in global coordinates. 
		@arg torques Output torques to apply
	 */
	void computeJointTorquesEquivalentToForce(std::string startName, const Ogre::Vector3& pLocal, const Ogre::Vector3& fGlobal /*, std::string endName=""*/);
	
	void computeIKSwingLegTargets(double dt);
	void computeIKQandW( Ogre::SharedPtr<ForwardDynamicsJoint> parentJoint, Ogre::SharedPtr<ForwardDynamicsJoint> childJ, const Ogre::Vector3& parentAxis, const Ogre::Vector3& parentNormal, const Ogre::Vector3& childNormal, const Ogre::Vector3& childEndEffector, const Ogre::Vector3& wP, bool computeAngVelocities, const Ogre::Vector3& futureWP, double dt);
	
	/**
		This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity v
		is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
		initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
	*/
	Ogre::Vector3 getSwingFootTargetLocation(double t, const Ogre::Vector3& com, const Ogre::Quaternion& charFrameToWorld);
	
	/**
		determine the estimate desired location of the swing foot, given the etimated position of the COM, and the phase
	*/
	Ogre::Vector3 computeSwingFootLocationEstimate( Ogre::Vector3 com, float phi );
	
	/**
	 returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
	 on the assumption that the character will come to a stop by taking a step at that location. The step location
	 is expressed in the character's frame coordinates.
	 */
	Ogre::Vector3 computeIPStepLocation();
	
	/**
		modify the coronal location of the step so that the desired step width results.
	 */
	float adjustCoronalStepLocation( float initialCentralStepLocation );
	
	/**
	 This method is used to compute torques for the stance leg that help achieve a desired speed in the sagittal and lateral planes
	 */
	void computeLegTorques( Ogre::SharedPtr<ForwardDynamicsJoint> ankleJoint, Ogre::SharedPtr<ForwardDynamicsJoint> kneeJoint, Ogre::SharedPtr<ForwardDynamicsJoint> hipJoint /*, std::vector<ContactPoint> *cfs*/);
	
	/**
	 This method is used to compute the torques that need to be applied to the stance and swing hips, given the
	 desired orientation for the root and the swing hip. The coordinate frame that these orientations are expressed
	 relative to is computed in this method. It is assumed that the stanceHipToSwingHipRatio variable is
	 between 0 and 1, and it corresponds to the percentage of the total net vertical force that rests on the stance
	 foot.
	 */
	void computeHipTorques(const Ogre::Quaternion& qRootD, float stanceHipToSwingHipRatio, Ogre::Vector3 ffRootTorque);
	
	/**
	 This method is used to return the ratio of the weight that is supported by the stance foot.
	 */
	float getStanceFootWeightRatio(/*DynamicArray<ContactPoint> *cfs*/);
	
	Ogre::Vector3 computeCoMVirtualForce();
	void COMJT();
	
	// this is for v
	Ogre::Vector3 getV() { return getCoMVelocityInCharacterFrame(); }
	Ogre::Vector3 getCoMVelocityInCharacterFrame() { return mCharacterFrame.Inverse() * mCoMVelocity; }
	Ogre::Vector3 getD() { return mCharacterFrame.Inverse() * (mCoM - getStanceAnkle()->getChildFdb()->getCoMWorld()); }
	
	
	void setDesiredSwingFootLocation( float phi, float dt );
	void setKneeBend( float bend, bool swingAlso=false );
	void setUpperBodyPose( float leanSagittal, float leanCoronal, float twist);
	
	// inherited
	virtual void footGroundContactStateChanged( const std::string& whichFoot, bool contact, const Ogre::Vector3& contactPos );
	
	/*! @brief Bind the stance foot to ground. */
	void bindStanceFootToGround();
	
	float mGravityCompensationFactor, mCoMVirtualForceFactor;
	float mRootPredictiveTorqueFactor;
	
	float mCoMkP, mCoMkD;
	Ogre::Vector3 mCoM, mPrevCoM, mCoMVelocity;
	float mCoMVelocitySmoothingFactor;
	
	float mTargetCoMVelocitySagittal, mTargetCoMVelocityCoronal;
	
	Ogre::Vector3 mFFRootTorque;
	Ogre::Quaternion mDebugTargetRootOrientation;
	Ogre::Vector3 mDebugRootTorque;
	
	Ogre::Quaternion mCharacterFrame;
	Ogre::Vector3 mInitialSwingFootPosition; // swing foot position at start of this phi loop
	
	float mPelvisHeightAboveFeet;
	Ogre::Vector3 mSwingLegPlaneOfRotation;

	Ogre::Vector3 mFootTargetL, mFootTargetR;
	Ogre::Vector3 mFootIPTargetL, mFootIPTargetR;
	float mLeftKneeOut, mRightKneeOut;
	Ogre::Vector3 mDebugCoMVirtualForce;
	
	float mStepWidth;
	float mLegLength;	
	
	bool mDoGravityCompensation, mDoCoMVirtualForce;
	bool mDoMotionGeneration, mDoSwingLegTargets, mDoHipTorques;
	bool mDoSwingLegGravityCompensation, mDoStanceLegGravityCompensation;
	
	Ogre::SharedPtr<VirtualModelMotionGenerator> mMotionGenerator;
	
	Ogre::SharedPtr<VirtualModelFSM> mFiniteStateMachine;
	
};

#endif /* defined(__Loco__VirtualModelSkeletonController__) */
