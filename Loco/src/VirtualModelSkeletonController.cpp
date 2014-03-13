//
//  VirtualModelSkeletonController.cpp
//  Loco
//
//  Created on 07/11/13.
//
//

#include "VirtualModelSkeletonController.h"

#include "OgreBulletDynamicsRigidBody.h"
#include "OgreBulletConverter.h"
#include "btRigidBody.h"
#include "btPoint2PointConstraint.h"

using namespace OgreBulletCollisions;

VirtualModelSkeletonController::VirtualModelSkeletonController(Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, OgreBulletDynamics::DynamicsWorld* dynamicsWorld, OgreBulletDynamics::RigidBody* groundPlaneBody, const picojson::value& jsonSource )
: ForwardDynamicsSkeletonController(skelRootSceneNode,skeleton,dynamicsWorld,jsonSource), mFootTargetL(0,0,0), mFootTargetR(0,0,0), mCoMTarget(0,0,0), mDoGravityCompensation(false), mDoCoMVirtualForce(false), mDoFootIK(true),
mCoMkP(100.0f), mCoMkD(4.0f), mCoMVelocity(0,0,0), mPrevCoM(0,0,0), mLeftKneeOut(0.5), mRightKneeOut(0.5),
mGroundPlaneBody(groundPlaneBody), mFootIKPlacementWidth(0.3f), mDoubleStance(false)
{
	picojson::object jsonRoot = jsonSource.get<picojson::object>();

	mPrevCoM = mDriveableSkeleton->getRootSceneNode()->convertLocalToWorldPosition(mDriveableSkeleton->getCenterOfMassWorld());
	mCoMTarget = mPrevCoM;
	
	// set the foot to be static
	/*btRigidBody* rb = mForwardDynamicsSkeleton->getBody("Foot.L")->getBody()->getBulletRigidBody();
	rb->setCollisionFlags( rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
	rb = mForwardDynamicsSkeleton->getBody("Foot.R")->getBody()->getBulletRigidBody();
	rb->setCollisionFlags( rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );*/
	
	// set the pelvis/root to be static
	//mForwardDynamicsSkeleton->getBody("SpineBase")->getBody()->getBulletRigidBody()->setMassProps(0, btVector3(0,0,0));
	
	mDoGravityCompensation = jsonRoot["enableGravityCompensation"].get<bool>();
	if ( jsonRoot.count("gravityCompensationFactor") ) {
		mGravityCompensationFactor = jsonRoot["gravityCompensationFactor"].get<double>();
	}
	mDoCoMVirtualForce = jsonRoot["enableCoMVirtualForce"].get<bool>();
	if ( jsonRoot.count("CoMVirtualForceFactor") ) {
		mCoMVirtualForceFactor = jsonRoot["CoMVirtualForceFactor"].get<double>();
	}
	
	mDoFootIK = jsonRoot["enableFootIK"].get<bool>();
	if ( jsonRoot.count("footIKDoubleStance") ) {
		mDoubleStance = jsonRoot["footIKDoubleStance"].get<bool>();
	}
	if ( jsonRoot.count("footIKPlacementWidth") ) {
		mFootIKPlacementWidth = jsonRoot["footIKPlacementWidth"].get<double>();
	}
	
	if ( jsonRoot.count("CoMKd") ) {
		mCoMkD = jsonRoot["CoMKd"].get<double>();
	}
	if ( jsonRoot.count("CoMKp") ) {
		mCoMkP = jsonRoot["CoMKp"].get<double>();
	}
		
	// set current pose to be the target pose
	
	//set<string> bodies = mForwardDynamicsSkeleton->getAllBodyNames();
	/*
	set<string>bodies;
	bodies.insert("Neck");
	bodies.insert("Head");
	bodies.insert("SpineBase");
	for ( auto bn: bodies ) {
		Ogre::Quaternion orientationTarget = mForwardDynamicsSkeleton->getBody(bn)->getOrientationWorld();
		mForwardDynamicsSkeleton->setOrientationTarget(bn, orientationTarget);
	}*/
	
	mPelvisHeightAboveFeet = (mForwardDynamicsSkeleton->getBody("SpineBase")->getCoMWorld() - (mForwardDynamicsSkeleton->getBody("Foot.L")->getHeadPositionWorld()+mForwardDynamicsSkeleton->getBody("Foot.R")->getHeadPositionWorld())*0.5f).y;
	mPelvisHeightAboveFeet *= 1.02f;
	
	// create FSM
	auto fsmParams = jsonRoot["finiteStateMachine"];
	mFiniteStateMachine = Ogre::SharedPtr<VirtualModelFSM>( new VirtualModelFSM(fsmParams) );
	
	// create motion genertaro
	auto motionGeneratorParams = jsonRoot["motionGenerator"];
	mMotionGenerator = Ogre::SharedPtr<VirtualModelMotionGenerator>( new VirtualModelMotionGenerator(motionGeneratorParams) );
	mMotionGenerator->setActiveMotion("splineTest");
	
	/*
	// set constraint iterations on the legs
	mForwardDynamicsSkeleton->getJointBetween("LegLower.L", "Foot.L")->getBulletConstraint()->setOverrideNumSolverIterations(20);
	mForwardDynamicsSkeleton->getJointBetween("LegUpper.L", "LegLower.L")->getBulletConstraint()->setOverrideNumSolverIterations(20);
	mForwardDynamicsSkeleton->getJointBetween("SpineBase", "LegUpper.L")->getBulletConstraint()->setOverrideNumSolverIterations(20);
	mForwardDynamicsSkeleton->getJointBetween("LegLower.R", "Foot.R")->getBulletConstraint()->setOverrideNumSolverIterations(20);
	mForwardDynamicsSkeleton->getJointBetween("LegUpper.R", "LegLower.R")->getBulletConstraint()->setOverrideNumSolverIterations(20);
	mForwardDynamicsSkeleton->getJointBetween("SpineBase", "LegUpper.R")->getBulletConstraint()->setOverrideNumSolverIterations(20);*/
}




/**
 from cartwheel3d
 
 
 This method is used to compute the torques that mimick the effect of applying a force on
 a rigid body, at some point. It works best if the end joint is connected to something that
 is grounded, otherwise (I think) this is just an approximation.
 
 This function works by making use of the formula:
 
 t = J' * f, where J' is dp/dq, where p is the position where the force is applied, q is
 'sorta' the relative orientation between links. It makes the connection between the velocity
 of the point p and the relative angular velocities at each joint. Here's an example of how to compute it.
 
 Assume: p = pBase + R1 * v1 + R2 * v2, where R1 is the matrix from link 1 to whatever pBase is specified in,
 and R2 is the rotation matrix from link 2 to whatever pBase is specified in, v1 is the point from link 1's
 origin to link 2's origin (in link 1 coordinates), and v2 is the vector from origin of link 2 to p
 (in link 2 coordinates).
 
 dp/dt = d(R1 * v1)/dt + d(R2 * v2)/dt = d R1/dt * v1 + d R2/dt * v2, and dR/dt = wx * R, where wx is
 the cross product matrix associated with the angular velocity w
 so dp/dt = w1x * R1 * v1 + w2x * R2 * v2, and w2 = w1 + wRel
 
 = [-(R1*v1 + R2*v2)x   -(R2*v1)x ] [w1   wRel]', so the first matrix is the Jacobian.
 The first entry is the cross product matrix of the vector (in 'global' coordinates) from the
 origin of link 1 to p, and the second entry is the vector (in 'global' coordinates) from
 the origin of link 2 to p (and therein lies the general way of writing this).
 */
void VirtualModelSkeletonController::computeJointTorquesEquivalentToForce(std::string startBodyName, const Ogre::Vector3& pLocal, const Ogre::Vector3& fGlobal )
{
	//starting from the start joint, going towards the end joint, get the origin of each link, in world coordinates,
	//and compute the vector to the global coordinates of pLocal.
	
	auto currentJoint = mForwardDynamicsSkeleton->getJointToParent(startBodyName);
	auto startFdb = currentJoint->getChildFdb();
	Ogre::Vector3 pGlobal = startFdb->getBody()->getSceneNode()->convertLocalToWorldPosition(pLocal);
	// Ogre::Vector3 pGlobal = currentJoint->child->getWorldCoordinates(pLocal);
	
	while ( !currentJoint.isNull() ) {
		//while (currentJoint != end) {
		//OgreAssert(currentJoint, "VirtualModelController::computeJointTorquesEquivalentToForce --> end was not a parent of start...");
		
		Ogre::Vector3 tmpV = pGlobal - currentJoint->getPositionWorld();
		//tmpV = Vector3d(currentJoint->parent->getWorldCoordinates(currentJoint->pJPos), pGlobal);
		Ogre::Vector3 tmpT = tmpV.crossProduct(fGlobal);
		mForwardDynamicsSkeleton->addJointTorque(currentJoint->getName(), -tmpT);
		//torques[currentJoint->id] -= tmpT;
		//currentJoint = currentJoint->parent->pJoint;
		currentJoint = mForwardDynamicsSkeleton->getJointToParent(currentJoint->getParentFdb()->getName());
	}
	
	//OgreAssert( (endBodyName.length()==0&&currentBody.isNull()) || (currentBody->getName()==endBodyName), "looks like startBodyName was not a descendent of endBodyName" );
	
	//and we just have to do it once more for the end joint, if it's not NULL
	/*
	if (end != NULL){
		tmpV = Ogre::Vector3(currentJoint->parent->getWorldCoordinates(currentJoint->pJPos), pGlobal);
		torques[currentJoint->id] -= tmpV.crossProductWith(fGlobal);
	}*/
}




void VirtualModelSkeletonController::evaluateMotionTargets( float deltaTime )
{
	
	mFiniteStateMachine->update(deltaTime);
	
	set<string> bodyNames = mForwardDynamicsSkeleton->getAllBodyNames();
	static const string swingHipName = "LegUpper."+getSwingLegSuffix();
	static const string stanceHipName = "LegUpper."+getStanceLegSuffix();
	
	mMotionGenerator->update(deltaTime);

	// create character reference frame
	Ogre::Quaternion characterFrame = mDriveableSkeleton->getRootSceneNode()-> convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);
	characterFrame = characterFrame * Ogre::Quaternion( Ogre::Radian(M_PI), Ogre::Vector3::UNIT_X );
	
	// walk through all the bodies
	for ( string bodyName: bodyNames ) {
		
		if ( mMotionGenerator->hasTarget(bodyName) ) {
			// get the orientation
			Ogre::Quaternion q = mMotionGenerator->getTarget(bodyName);
			// convert to world orientation, based on the reference frame
			Ogre::Quaternion orientationW;
			VirtualModelMotionComponent::ReferenceFrame refFrame = mMotionGenerator->getReferenceFrame(bodyName);
			if ( refFrame == VirtualModelMotionComponent::RF_Character ) {
				orientationW = characterFrame * q;
			} else {
				q = mForwardDynamicsSkeleton->getBody(bodyName)->getParentRelativeRestOrientation()*q;
				orientationW = mForwardDynamicsSkeleton->getParentBody(bodyName)->convertLocalToWorldOrientation(q);
			}
			
			mForwardDynamicsSkeleton->setOrientationTarget(bodyName, orientationW);
		} else {
			// clear any previously set target
			mForwardDynamicsSkeleton->clearOrientationTarget(bodyName);
		}
		
		/*
		if ( mFiniteStateMachine->hasTargetOrientation(bodyName) ) {
			// get the orientation
			Ogre::Quaternion q = mFiniteStateMachine->getTargetOrientation(bodyName);
			// convert to world orientation, based on the reference frame
			Ogre::Quaternion orientationW;
			VirtualModelFSMState::TargetReferenceFrame refFrame = mFiniteStateMachine->getReferenceFrame(bodyName);
			if ( refFrame == VirtualModelFSMState::TRF_World ) {
				orientationW = q;
			} else {
				q = mForwardDynamicsSkeleton->getBody(bodyName)->getParentRelativeRestOrientation()*q;
				orientationW = mForwardDynamicsSkeleton->getParentBody(bodyName)->convertLocalToWorldOrientation(q);
			}
				
			mForwardDynamicsSkeleton->setOrientationTarget(bodyName, orientationW);
		}*/
	}
}


void VirtualModelSkeletonController::update( float dt )
{
	// update CoM velocity
	Ogre::Vector3 currentCoM = mDriveableSkeleton->getRootSceneNode()->convertLocalToWorldPosition(mDriveableSkeleton->getCenterOfMassWorld());
	mCoMVelocity = (currentCoM-mPrevCoM)/dt;
	mPrevCoM = currentCoM;
	
	Ogre::Vector3 lFootHead = mForwardDynamicsSkeleton->getBody("Foot.L")->getHeadPositionWorld();
	Ogre::Vector3 rFootHead = mForwardDynamicsSkeleton->getBody("Foot.R")->getHeadPositionWorld();
	
	/*
	// select stance leg
	// the timer is to prevent stance leg from swapping too often
	Ogre::Vector3 lFootCoM = mForwardDynamicsSkeleton->getBody("Foot.L")->getCoMWorld();
	Ogre::Vector3 rFootCoM = mForwardDynamicsSkeleton->getBody("Foot.R")->getCoMWorld();
	if ( mTimeSinceLastStanceLegSwap>mStanceLegSwitchMinTime )
	{
		// select the stance leg
		// find which foot the com is more likely to be over
		Ogre::Vector3 testCoM = currentCoM;
		testCoM.y = lFootCoM.y;
		float lFootCoMDistance = (testCoM-lFootCoM).squaredLength();
		testCoM.y = rFootCoM.y;
		float rFootCoMDistance = (testCoM-rFootCoM).squaredLength();
		string oldStanceLeg = mStanceLeg;
		if ( lFootCoMDistance<rFootCoMDistance ) {
			setStanceLeg("L");
		} else {
			setStanceLeg("R");
		}
		if ( oldStanceLeg != mStanceLeg ) {
			mTimeSinceLastStanceLegSwap = 0.0f;
		}
	}
	// force stance leg to swap every 2 seconds
	if ( mTimeSinceLastStanceLegSwap>2.0f ) {
		if ( mStanceLeg=="L" ) {
			setStanceLeg("R");
		} else {
			setStanceLeg("L");
		}
		mTimeSinceLastStanceLegSwap = 0.0f;
	}*/
	
	// update CoM target
	if ( mDoubleStance ) {
		mCoMTarget = (lFootHead+rFootHead)*0.5f;
	} else {
		if ( mMotionGenerator->getStanceIsLeft() ) {
			mCoMTarget = (lFootHead*0.75f+rFootHead*0.25f);
		} else {
			mCoMTarget = (lFootHead*0.25f+rFootHead*0.75f);
		}
	}
	mCoMTarget.y += mPelvisHeightAboveFeet;
	
	
///////// begin simbicon import
	//evaluate the target orientation for every joint, using the SIMBICON state information
	evaluateMotionTargets( dt );
	
	/*
	//now overwrite the target angles for the swing hip and the swing knee in order to ensure foot-placement control
	if (doubleStanceMode == false)
		computeIKSwingLegTargets(0.001);
	
	computePDTorques(cfs);
	
	//bubble-up the torques computed from the PD controllers
	bubbleUpTorques();*/
///////// end simbicon import
	
	
	if ( mDoGravityCompensation ) {
		computeGravityCompensationTorques();
	}

	if ( mDoFootIK ) {
		
		clearLegIKForwardDynamics("R");
		clearLegIKForwardDynamics("L");
		
		
		
		Ogre::Vector3 leftFootTargetPos(0,0,0);
		Ogre::Vector3 rightFootTargetPos(0,0,0);
		mLeftKneeOut = 0.05f;
		mRightKneeOut = 0.4f;
		
		// choose which leg to use as basis based on stance leg selection
		static const float kKneeOutSmoothing = 0.995f;
		if ( mMotionGenerator->getStanceIsLeft() ) {
			leftFootTargetPos = mForwardDynamicsSkeleton->getBody("Foot.L")->getHeadPositionWorld();
			leftFootTargetPos.y = 0;
			rightFootTargetPos = leftFootTargetPos + Ogre::Vector3(mFootIKPlacementWidth,0,0);
	/*		mLeftKneeOut = mLeftKneeOut*kKneeOutSmoothing+0.0f*(1.0f-kKneeOutSmoothing);
			mRightKneeOut = mRightKneeOut*kKneeOutSmoothing+1.0f*(1.0f-kKneeOutSmoothing);*/
			solveLegIKForwardDynamics("L", leftFootTargetPos, leftFootTargetPos.y+mPelvisHeightAboveFeet, true, mLeftKneeOut );
			
		} else {
			rightFootTargetPos = mForwardDynamicsSkeleton->getBody("Foot.R")->getHeadPositionWorld();
			rightFootTargetPos.y = 0;
			leftFootTargetPos = rightFootTargetPos - Ogre::Vector3(mFootIKPlacementWidth,0,0);
			/*
			mLeftKneeOut = mLeftKneeOut*kKneeOutSmoothing+1.0f*(1.0f-kKneeOutSmoothing);
			mRightKneeOut = mRightKneeOut*kKneeOutSmoothing+0.0f*(1.0f-kKneeOutSmoothing);*/
			solveLegIKForwardDynamics("R", rightFootTargetPos, rightFootTargetPos.y+mPelvisHeightAboveFeet, true, mRightKneeOut );
		}
		
		if ( mDoubleStance ) {
			if ( !mMotionGenerator->getStanceIsLeft() ) {
				solveLegIKForwardDynamics("L", leftFootTargetPos, leftFootTargetPos.y+mPelvisHeightAboveFeet, true, mLeftKneeOut );
			} else {
				solveLegIKForwardDynamics("R", rightFootTargetPos, rightFootTargetPos.y+mPelvisHeightAboveFeet, true, mRightKneeOut );
			}
		}
			
		
		mFootTargetL = leftFootTargetPos;
		mFootTargetR = rightFootTargetPos;
	}
		
	
	if ( mDoCoMVirtualForce )
		COMJT();
		
	
	ForwardDynamicsSkeletonController::update(dt);
	
}


void VirtualModelSkeletonController::computeGravityCompensationTorques( )
{
	Ogre::Vector3 gravity = mDynamicsWorld->getGravity();
	//gravity = Ogre::Vector3(0,1,0);
	std::set<std::string> names = mForwardDynamicsSkeleton->getAllBodyNames();
	
	// skip the 'stance leg'
	set<string> skipNames;
	/*
	skipNames.insert("LegUpper."+mStanceLeg);
	skipNames.insert("LegLower."+mStanceLeg);
	skipNames.insert("Foot."+mStanceLeg);*/
	/*
	skipNames.insert("LegUpper.L");
	skipNames.insert("LegLower.L");
	skipNames.insert("Foot.L");
	skipNames.insert("LegUpper.R");
	skipNames.insert("LegLower.R");
	skipNames.insert("Foot.R");*/
		
	for ( auto name: names )
	{
		if ( skipNames.count(name) ) {
			continue;
		}
		auto fdb = mForwardDynamicsSkeleton->getBody(name);
		float invMass = fdb->getBody()->getBulletRigidBody()->getInvMass();
		if ( invMass==0 ) {
			BLog("body %s has mass==0, skipping", name.c_str());
		} else {
			// don't do this if there's no parent
			if ( !mForwardDynamicsSkeleton->getJointToParent(name).isNull() ) {
				Ogre::Vector3 force = -gravity/invMass;
				force *= mGravityCompensationFactor;
				computeJointTorquesEquivalentToForce( name, Ogre::Vector3::ZERO, force );
			}
		}
	}
	
	
}

Ogre::Vector3 VirtualModelSkeletonController::computeCoMVirtualForce()
{
	Ogre::Vector3 currentCoM = mDriveableSkeleton->getRootSceneNode()->convertLocalToWorldPosition(mDriveableSkeleton->getCenterOfMassWorld());
	Ogre::Vector3 targetCoMDelta = mCoMTarget - currentCoM;
	//targetCoMDelta.y = 0;
	
	//BLog("CoM %s, target %s, target delta %8.5f", describe(currentCoM).c_str(), describe(mTargetCoM).c_str(), targetCoMDelta.length() );
	/*
	Ogre::Vector3 virtualForce = targetCoMDelta*mCoMkP;
	Ogre::Vector3 targetCoMVelocityDelta = -mCoMVelocity;
	virtualForce += targetCoMVelocityDelta*mCoMkD;*/
	Ogre::Vector3 virtualForce = targetCoMDelta.normalisedCopy();
	
	return virtualForce;
}

void VirtualModelSkeletonController::COMJT(/*DynamicArray<ContactPoint> *cfs*/)
{
	//applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
	// dp/dq, where p is the COM.
	string whichLeg = mMotionGenerator->getStanceIsLeft()?"L":"R";
	string footName = "Foot."+whichLeg;
	string lowerLegName = "LegLower."+whichLeg;
	string upperLegName = "LegUpper."+whichLeg;
	string pelvisName = "SpineBase";
	string lowerBackName = "SpineMid";
	string midBackName = "SpineTop";


	
	/*
	int lBackIndex = character->getJointIndex("pelvis_lowerback");
	int mBackIndex = character->getJointIndex("lowerback_torso");*/

	double m = 0;
	/*
	ArticulatedRigidBody* tibia = character->joints[stanceAnkleIndex]->parent;
	ArticulatedRigidBody* femur = character->joints[stanceKneeIndex]->parent;
	ArticulatedRigidBody* pelvis = character->joints[stanceHipIndex]->parent;
	ArticulatedRigidBody* lBack = character->joints[lBackIndex]->child;
	ArticulatedRigidBody* mBack = character->joints[mBackIndex]->child;*/
	auto tibia = mForwardDynamicsSkeleton->getBody(lowerLegName);
	auto femur = mForwardDynamicsSkeleton->getBody(upperLegName);
	auto pelvis = mForwardDynamicsSkeleton->getBody(pelvisName);
	auto lBack = mForwardDynamicsSkeleton->getBody(lowerBackName);
	auto mBack = mForwardDynamicsSkeleton->getBody(midBackName);
	
	/*
	Point3d anklePos = character->joints[stanceAnkleIndex]->child->getWorldCoordinates(character->joints[stanceAnkleIndex]->cJPos);
	Point3d kneePos = character->joints[stanceKneeIndex]->child->getWorldCoordinates(character->joints[stanceKneeIndex]->cJPos);
	Point3d hipPos = character->joints[stanceHipIndex]->child->getWorldCoordinates(character->joints[stanceHipIndex]->cJPos);
	Point3d lbackPos = character->joints[lBackIndex]->child->getWorldCoordinates(character->joints[lBackIndex]->cJPos);
	Point3d mbackPos = character->joints[mBackIndex]->child->getWorldCoordinates(character->joints[mBackIndex]->cJPos);*/
	
	Ogre::Vector3 anklePos = tibia->getTailPositionWorld();
	Ogre::Vector3 kneePos = tibia->getHeadPositionWorld();
	Ogre::Vector3 hipPos = femur->getHeadPositionWorld();
	Ogre::Vector3 lBackPos = lBack->getHeadPositionWorld();
	Ogre::Vector3 mBackPos = mBack->getHeadPositionWorld();
	
	//total mass...
	//m = tibia->props.mass + femur->props.mass + pelvis->props.mass + lBack->props.mass + mBack->props.mass;
	m = tibia->getMass() + femur->getMass() + pelvis->getMass() + lBack->getMass() + mBack->getMass();

	
	Ogre::Vector3 fA = computeCoMVirtualForce()*mCoMVirtualForceFactor;
	mDebugCoMVirtualForce = fA;

	Ogre::Vector3 f1 = (tibia->getCoMWorld()-anklePos) * tibia->getMass() +
		(femur->getCoMWorld()-anklePos) * femur->getMass() +
		(pelvis->getCoMWorld()-anklePos) * pelvis->getMass() +
		(lBack->getCoMWorld()-anklePos) * lBack->getMass() +
		(mBack->getCoMWorld()-anklePos) * mBack->getMass();
	f1 /= m;
	
	Ogre::Vector3 f2 =
		(femur->getCoMWorld()-kneePos) * femur->getMass() +
		(pelvis->getCoMWorld()-kneePos) * pelvis->getMass() +
		(lBack->getCoMWorld()-kneePos) * lBack->getMass() +
		(mBack->getCoMWorld()-kneePos) * mBack->getMass();
	f2 /= m;
	
	Ogre::Vector3 f3 =
		(pelvis->getCoMWorld()-hipPos) * pelvis->getMass() +
		(lBack->getCoMWorld()-hipPos) * lBack->getMass() +
		(mBack->getCoMWorld()-hipPos) * mBack->getMass();
	f3 /= m;
	
	Ogre::Vector3 f4 =
		(lBack->getCoMWorld()-lBackPos) * lBack->getMass() +
		(mBack->getCoMWorld()-lBackPos) * mBack->getMass();
	f4 /= m;
	
	Ogre::Vector3 f5 =
		(mBack->getCoMWorld()-mBackPos) * mBack->getMass();
	f5 /= m;
	
	
	Ogre::Vector3 ankleTorque = f1.crossProduct(fA);
	//BLog("ankleTorque: %s", describe(ankleTorque).c_str());
	//Ogre::Vector3 ankleTorque(0,0,0);
	//preprocessAnkleVTorque(stanceAnkleIndex, cfs, &ankleTorque);

	string ankleJointName = lowerLegName+"-"+footName;
	string kneeJointName = upperLegName+"-"+lowerLegName;
	string hipJointName = pelvisName+"-"+upperLegName;
	string lBackJointName = pelvisName+"-"+lowerBackName;
	string mBackJointName = lowerBackName+"-"+midBackName;
	
	mForwardDynamicsSkeleton->addJointTorque(ankleJointName, ankleTorque);
	mForwardDynamicsSkeleton->addJointTorque(kneeJointName, f2.crossProduct(fA));
	mForwardDynamicsSkeleton->addJointTorque(hipJointName, f3.crossProduct(fA));
	
	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	mForwardDynamicsSkeleton->getBody(pelvisName)->addTorque(-f3.crossProduct(fA));
	
	mForwardDynamicsSkeleton->addJointTorque(lBackJointName, -f4.crossProduct(fA)*0.5);
	mForwardDynamicsSkeleton->addJointTorque(mBackJointName, -f5.crossProduct(fA)*0.3);
}

void VirtualModelSkeletonController::debugDraw()
{
	ForwardDynamicsSkeletonController::debugDraw();
	
	mDebugLines->addCross(mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(mFootTargetL), 0.1, Ogre::ColourValue(0.3, 0.8, 0.3));
	mDebugLines->addCross(mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(mFootTargetR), 0.1, Ogre::ColourValue(0.3, 0.8, 0.3));
	
	mDebugLines->addCross(mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(mCoMTarget), 0.1, Ogre::ColourValue(0.8, 0.3, 0.5f));
	
	// draw com virtual force
	Ogre::Vector3 com = mDriveableSkeleton->getRootSceneNode()->convertLocalToWorldPosition( mDriveableSkeleton->getCenterOfMassWorld());
	mDebugLines->addLine(mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(com), mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(com+mDebugCoMVirtualForce), Ogre::ColourValue(0.2, 0.2, 1.0) );
}

void VirtualModelSkeletonController::pushModel(Ogre::Vector3 force)
{
	string pelvisName = "SpineBase";
	mForwardDynamicsSkeleton->getBody(pelvisName)->addImpulse(force);
}

