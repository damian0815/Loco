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
#include "TwoLinkIK.h"
#include "MathUtilities.h"

using namespace OgreBulletCollisions;
using namespace Ogre;

VirtualModelSkeletonController::VirtualModelSkeletonController(Ogre::SceneNode* skelRootSceneNode, Ogre::Skeleton* skeleton, OgreBulletDynamics::DynamicsWorld* dynamicsWorld, OgreBulletDynamics::RigidBody* groundPlaneBody, const picojson::value& jsonSource )
: ForwardDynamicsSkeletonController(skelRootSceneNode,skeleton,dynamicsWorld,groundPlaneBody,jsonSource), mFootTargetL(0,0,0), mFootTargetR(0,0,0),
mCoMVirtualForceKp(100.0f), mCoMVirtualForceKd(4.0f), mCoMVirtualForceScale(1,1,1),
mCoMVelocity(0,0,0), mCoM(0,0,0), mCoMVelocitySmoothingFactor(0.2f),
mLeftKneeOut(0.5), mRightKneeOut(0.5), mKneeBend(0.0f),
mStepWidth(0.3f), mSwingLegPlaneOfRotation(Ogre::Vector3::UNIT_X),
mTargetCoMVelocitySagittal(0.0), mTargetCoMVelocityCoronal(0.0), mRootPredictiveTorqueFactor(0),
mDoGravityCompensation(true), mDoCoMVirtualForce(true), mDoHipTorques(true), mDoMotionGeneration(true), mDoSwingLegTargets(true), mDoSwingLegGravityCompensation(true), mDoStanceLegGravityCompensation(false)
{
	picojson::object jsonRoot = jsonSource.get<picojson::object>();

	mCoM = mForwardDynamicsSkeleton->getCenterOfMassWorld();
	
	// set the foot to be static
	/*btRigidBody* rb = mForwardDynamicsSkeleton->getBody("Foot.L")->getBody()->getBulletRigidBody();
	rb->setCollisionFlags( rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
	rb = mForwardDynamicsSkeleton->getBody("Foot.R")->getBody()->getBulletRigidBody();
	rb->setCollisionFlags( rb->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );*/
	
	// set the pelvis/root to be static
	// fix the root
	// fix the pelvis
	// immobilize root
	// fix root
	// fix pelvis
	//mForwardDynamicsSkeleton->getBody("SpineBase")->getBody()->getBulletRigidBody()->setMassProps(0, btVector3(0,0,0));
	
	mDoGravityCompensation = jsonRoot["enableGravityCompensation"].get<bool>();
	if ( jsonRoot.count("gravityCompensationFactor") ) {
		mGravityCompensationFactor = jsonRoot["gravityCompensationFactor"].get<double>();
	}
	mDoCoMVirtualForce = jsonRoot["enableCoMVirtualForce"].get<bool>();
	
	// com virtual force
	if ( jsonRoot.count("CoMVirtualForce") ) {
		picojson::object comVF = jsonRoot["CoMVirtualForce"].get<picojson::object>();
		if ( comVF.count("kD") ) {
			mCoMVirtualForceKd = comVF["kD"].get<double>();
		}
		if ( comVF.count("kP") ) {
			mCoMVirtualForceKp = comVF["kP"].get<double>();
		}
		if ( comVF.count("sagittalScale") ) {
			mCoMVirtualForceScale.z = comVF["sagittalScale"].get<double>();
		}
		if ( comVF.count("coronalScale") ) {
			mCoMVirtualForceScale.x = comVF["coronalScale"].get<double>();
		}
		if ( comVF.count("axialScale") ) {
			mCoMVirtualForceScale.y = comVF["axialScale"].get<double>();
		}

	}
	
	mDoHipTorques = jsonRoot["enableHipTorques"].get<bool>();
	mDoMotionGeneration = jsonRoot["enableMotionGeneration"].get<bool>();
	mDoSwingLegTargets = jsonRoot["enableMotionGenerationSwingLegTargets"].get<bool>();
	if ( jsonRoot.count("enableSwingLegGravityCompensation") ) {
		mDoSwingLegGravityCompensation = jsonRoot["enableSwingLegGravityCompensation"].get<bool>();
	}
	if ( jsonRoot.count("enableStanceLegGravityCompensation") ) {
		mDoStanceLegGravityCompensation = jsonRoot["enableStanceLegGravityCompensation"].get<bool>();
	}
		
	if ( jsonRoot.count("CoMVelocitySmoothingFactor") ) {
		mCoMVelocitySmoothingFactor = jsonRoot["CoMVelocitySmoothingFactor"].get<double>();
	}
	
	if ( jsonRoot.count("stepWidth") ) {
		mStepWidth = jsonRoot["stepWidth"].get<double>();
	}
	if ( jsonRoot.count("kneeBend") ) {
		mKneeBend = -jsonRoot["kneeBend"].get<double>();
	}
	
	if ( jsonRoot.count("targetCoMVelocitySagittal") ) {
		mTargetCoMVelocitySagittal = jsonRoot["targetCoMVelocitySagittal"].get<double>();
	}
	if ( jsonRoot.count("targetCoMVelocityCoronal") ) {
		mTargetCoMVelocityCoronal = jsonRoot["targetCoMVelocityCoronal"].get<double>();
	}
		
	if ( jsonRoot.count("rootPredictiveTorqueFactor") ) {
		mRootPredictiveTorqueFactor = jsonRoot["rootPredictiveTorqueFactor"].get<double>();
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
	
	mLegLength = (mForwardDynamicsSkeleton->getJointBetween("SpineBase", "LegUpper.L")->getPositionWorld() -
				  mForwardDynamicsSkeleton->getJointBetween( "LegLower.L", "Foot.L")->getPositionWorld()).length();
	
	// create FSM
	auto fsmParams = jsonRoot["finiteStateMachine"];
	if ( !fsmParams.is<picojson::null>() ) {
		mFiniteStateMachine = Ogre::SharedPtr<VirtualModelFSM>( new VirtualModelFSM(fsmParams) );
	}
	
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
	
	// bind the stance foot to the ground
	if ( jsonRoot.count("bindLeftFootOnStartup") && jsonRoot["bindLeftFootOnStartup"].get<bool>() ) {
		bindBodyToEnvironment("Foot.L");
	}
	if ( jsonRoot.count("bindRightFootOnStartup") && jsonRoot["bindRightFootOnStartup"].get<bool>() ) {
		bindBodyToEnvironment("Foot.R");
	}
}

Ogre::SharedPtr<ForwardDynamicsJoint> VirtualModelSkeletonController::getStanceAnkle()
{
	string suffix = getStanceLegSuffix();
	return mForwardDynamicsSkeleton->getJointBetween( "LegLower."+suffix, "Foot."+suffix );
}
Ogre::SharedPtr<ForwardDynamicsJoint> VirtualModelSkeletonController::getSwingAnkle()
{
	string suffix = getSwingLegSuffix();
	return mForwardDynamicsSkeleton->getJointBetween( "LegLower."+suffix, "Foot."+suffix );
}
Ogre::SharedPtr<ForwardDynamicsJoint> VirtualModelSkeletonController::getSwingHip()
{
	string suffix = getSwingLegSuffix();
	return mForwardDynamicsSkeleton->getJointBetween( "SpineBase", "LegUpper."+suffix);
}
Ogre::SharedPtr<ForwardDynamicsJoint> VirtualModelSkeletonController::getStanceHip()
{
	string suffix = getStanceLegSuffix();
	return mForwardDynamicsSkeleton->getJointBetween( "SpineBase", "LegUpper."+suffix);
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
	
	if ( !mFiniteStateMachine.isNull() ) {
		mFiniteStateMachine->update(deltaTime);
	}
	
	// upadate motion generator
	bool phaseLooped = mMotionGenerator->update(deltaTime);
	if ( phaseLooped ) {
		auto swingAnkle = getSwingAnkle();
		mInitialSwingFootPosition = swingAnkle->getPositionWorld();
		
		// bind/unbind feet as appropriate
		string stanceFootName = "Foot."+getStanceLegSuffix();
		string swingFootName = "Foot."+getSwingLegSuffix();
		releaseBodyFromEnvironmentBinding(swingFootName);
		bindBodyToEnvironment(stanceFootName);
	}
	
	
	std::set<string> bodyNames = mForwardDynamicsSkeleton->getAllBodyNames();
	static const string swingHipName = "LegUpper."+getSwingLegSuffix();
	static const string stanceHipName = "LegUpper."+getStanceLegSuffix();

	// create character reference frame
	//Ogre::Quaternion characterFrame = mDriveableSkeleton->getRootSceneNode()-> convertLocalToWorldOrientation(Ogre::Quaternion::IDENTITY);
	//characterFrame = characterFrame * Ogre::Quaternion( Ogre::Radian(M_PI), Ogre::Vector3::UNIT_X );
	
	// walk through all the bodies
	for ( string bodyName: bodyNames ) {
		
		if ( mMotionGenerator->hasTargetOrientationForBody(bodyName) ) {
			// get the orientation
			Ogre::Quaternion q = mMotionGenerator->getTargetOrientationForBody(bodyName);
			// convert to world orientation, based on the reference frame
			Ogre::Quaternion orientationW;
			VirtualModelMotionComponent::ReferenceFrame refFrame = mMotionGenerator->getReferenceFrameForOrientation(bodyName);
			if ( refFrame == VirtualModelMotionComponent::RF_Character ) {
				orientationW = mCharacterFrame * q;
			} else if ( refFrame == VirtualModelMotionComponent::RF_Parent ) {
				q = mForwardDynamicsSkeleton->getBody(bodyName)->getParentRelativeRestOrientation()*q;
				orientationW = mForwardDynamicsSkeleton->getParentBody(bodyName)->convertLocalToWorldOrientation(q);
			} else if ( refFrame == VirtualModelMotionComponent::RF_World ) {
				orientationW = q;
			} else {
				OgreAssert( false, "unrecognized reference frame");
			}
			
			mForwardDynamicsSkeleton->setOrientationTarget(bodyName, orientationW);
			
			// store spine base
			if ( bodyName == "SpineBase") {
				mDebugTargetRootOrientation = orientationW;
			}
		} else {
			// clear any previously set target
			mForwardDynamicsSkeleton->clearOrientationTarget(bodyName);
		}
		
	}
}

/**
	This method is used to compute the desired orientation and angular velocity for a parent RB and a child RB, relative to the grandparent RB and
	parent RB repsectively. The input is:
		- the index of the joint that connects the grandparent RB to the parent RB, and the index of the joint between parent and child

		- the distance from the parent's joint with its parent to the location of the child's joint, expressed in parent coordinates

		- two rotation normals - one that specifies the plane of rotation for the parent's joint, expressed in grandparent coords, 
		  and the other specifies the plane of rotation between the parent and the child, expressed in parent's coordinates.

	    - The position of the end effector, expressed in child's coordinate frame

		- The desired position of the end effector, expressed in world coordinates

		- an estimate of the desired position of the end effector, in world coordinates, some dt later - used to compute desired angular velocities
*/

void VirtualModelSkeletonController::computeIKQandW( Ogre::SharedPtr<ForwardDynamicsJoint> parentJoint, Ogre::SharedPtr<ForwardDynamicsJoint> childJoint, const Vector3& parentAxis, const Vector3& parentNormal, const Vector3& childNormal, const Vector3& childEndEffector, const Vector3& wP, bool computeAngVelocities, const Vector3& futureWP, double dt)
{
	// parentJoint is the joint between the grandparent RB and the parent
	//this is the grandparent - most calculations will be done in its coordinate frame
	auto gParent = parentJoint->getParentFdb();
	//this is the reduced character space where we will be setting the desired orientations and ang vels.
	//ReducedCharacterState rs(&desiredPose);

	//the desired relative orientation between parent and grandparent
	Quaternion qParent;
	//and the desired relative orientation between child and parent
	Quaternion qChild;


	TwoLinkIK::getIKOrientations(parentJoint->getPositionInParentSpace(), gParent->convertWorldToLocalPosition(wP), parentNormal, parentAxis, childNormal, childEndEffector, &qParent, &qChild);
	//TwoLinkIK::getIKOrientations(parentJoint->getParentJointPosition(), gParent->getLocalCoordinates(wP), parentNormal, parentAxis, childNormal, childEndEffector, &qParent, &qChild);

	mForwardDynamicsSkeleton->setOrientationTarget(childJoint->getParentFdb()->getName(), qChild);
	mForwardDynamicsSkeleton->setOrientationTarget(parentJoint->getParentFdb()->getName(), qParent);
	
	/*
	controlParams[parentJIndex].relToFrame = false;
	controlParams[childJIndex].relToFrame = false;
	rs.setJointRelativeOrientation(qChild, childJIndex);
	rs.setJointRelativeOrientation(qParent, parentJIndex);
	*/

	Ogre::Vector3 wParentD(0,0,0);
	Ogre::Vector3 wChildD(0,0,0);

	if (computeAngVelocities)
	{
		//the joint's origin will also move, so take that into account, by subbing the offset by which it moves to the
		//futureTarget (to get the same relative position to the hip)
		Ogre::Vector3 velOffset = gParent->getAbsoluteVelocityForLocalPoint(parentJoint->getPositionInParentSpace());

		Quaternion qParentF;
		Quaternion qChildF;
		TwoLinkIK::getIKOrientations(parentJoint->getPositionInParentSpace(), gParent->convertWorldToLocalPosition(futureWP + velOffset * -dt), parentNormal, parentAxis, childNormal, childEndEffector, &qParentF, &qChildF);

		//Quaternion qDiff = qParentF * qParent.getComplexConjugate();
		Quaternion qDiff = qParentF * OgreQuaternionGetComplexConjugate(qParent);
		//wParentD = qDiff.v * 2.0f/dt;
		wParentD = Vector3(qDiff.x,qDiff.y,qDiff.z) * 2.0f/dt;
		//the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we'll account for that
		const btVector3& gParentAngVel = gParent->getBody()->getBulletRigidBody()->getAngularVelocity();
		//wParentD -= gParent->getLocalCoordinates(gParent->getAngularVelocity());
		wParentD -= gParent->convertWorldToLocalPosition(BtOgreConverter::to(gParentAngVel));

		//qDiff = qChildF * qChild.getComplexConjugate();
		qDiff = qChildF * OgreQuaternionGetComplexConjugate(qChild);
		//wChildD = qDiff.v * 2.0f/dt;
		wChildD = Vector3(qDiff.x,qDiff.y,qDiff.z) * 2.0f/dt;

		//make sure we don't go overboard with the estimates, in case there are discontinuities in the trajectories...
		OgreVector3ClampAllAxes(wChildD,-5,5);
		OgreVector3ClampAllAxes(wParentD,-5,5);
		//boundToRange(&wChildD.x, -5, 5);boundToRange(&wChildD.y, -5, 5);boundToRange(&wChildD.z, -5, 5);
		//boundToRange(&wParentD.x, -5, 5);boundToRange(&wParentD.y, -5, 5);boundToRange(&wParentD.z, -5, 5);
	}

	mForwardDynamicsSkeleton->setAngularVelocityTarget(childJoint->getParentFdb()->getName(), wChildD);
	mForwardDynamicsSkeleton->setAngularVelocityTarget(parentJoint->getParentFdb()->getName(), wParentD);
	//rs.setJointRelativeAngVelocity(wChildD, childJIndex);
	//rs.setJointRelativeAngVelocity(wParentD, parentJIndex);

}




/**
 This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
 is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
 initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
 */
Ogre::Vector3 VirtualModelSkeletonController::getSwingFootTargetLocation(double t, const Ogre::Vector3& com, const Ogre::Quaternion& charFrameToWorld)
{
	OgreAssert ( mMotionGenerator->hasTargetPositionForBody("Foot.SWING"), "must have Foot.SWING target" );
	Vector3 step = mMotionGenerator->getTargetPositionForBody( "Foot.SWING" );
	OgreAssert( mMotionGenerator->getReferenceFrameForPosition( "Foot.SWING") == VirtualModelMotionComponent::RF_Character, "wrong reference frame, should be character" );
	float stepY = step.y;
	step.y = 0;
	//now transform this vector into world coordinates
	step = charFrameToWorld*step;
	//add it to the com location
	step = com + step;
	//finally, set the desired height of the foot
	step.y = stepY;
	
	// now the delta
	if ( mMotionGenerator->hasTargetPositionForBody( "FootDelta.SWING" ) )
	{
		OgreAssert( mMotionGenerator->getReferenceFrameForPosition( "FootDelta.SWING") == VirtualModelMotionComponent::RF_Character, "wrong reference frame, should be character" );
		float sign = getStanceIsLeft() ? 1.0f : -1.0f;
		Ogre::Vector3 delta = mMotionGenerator->getTargetPositionForBody( "FootDelta.SWING" );
		delta.x *= sign;
		step += delta;
	}
	
	return step;
}

void VirtualModelSkeletonController::computeIKSwingLegTargets(double dt)
{
	Ogre::Vector3 pNow, pFuture;
	
	//note, V is already expressed in character frame coordinates.
	float phi = mMotionGenerator->getPhi();
	pNow = getSwingFootTargetLocation(phi, mCoM, mCharacterFrame);
	if ( getStanceIsLeft() ) {
		mFootTargetR = pNow;
	} else {
		mFootTargetL = pNow;
	}
	
	// swing leg
	// now
	string swingLeg = getStanceIsLeft()?"R":"L";
	Ogre::Quaternion upperLegTargetWorld, lowerLegTargetWorld;
	float kneeOut = 0.25f;
	solveLegIK( swingLeg, pNow, upperLegTargetWorld, lowerLegTargetWorld, kneeOut );
	
	// future
	Ogre::Quaternion upperLegTargetFutureWorld, lowerLegTargetFutureWorld;
	pFuture = getSwingFootTargetLocation(MIN(phi+dt, 1), mCoM + mCoMVelocity * dt, mCharacterFrame);
	solveLegIK( swingLeg, pFuture, upperLegTargetFutureWorld, lowerLegTargetFutureWorld, kneeOut );
	
	// delta
	// we could use complex conjugate instead of inverse because q's are normalised here
	Ogre::Quaternion qDiffUpperLeg = upperLegTargetFutureWorld * lowerLegTargetWorld.Inverse();
	Ogre::Vector3 upperLegAngularVelocityTargetWorld = Ogre::Vector3(qDiffUpperLeg.x,qDiffUpperLeg.y,qDiffUpperLeg.z)*dt*0.5f;
	Ogre::Quaternion qDiffLowerLeg = lowerLegTargetFutureWorld * lowerLegTargetWorld.Inverse();
	Ogre::Vector3 lowerLegAngularVelocityTargetWorld = Ogre::Vector3(qDiffLowerLeg.x,qDiffLowerLeg.y,qDiffLowerLeg.z)*dt*0.5f;
	
	//make sure we don't go overboard with the estimates, in case there are discontinuities in the trajectories...
	OgreVector3ClampAllAxes(upperLegAngularVelocityTargetWorld,-5.0f,5.0f);
	OgreVector3ClampAllAxes(lowerLegAngularVelocityTargetWorld,-5.0f,5.0f);
	
	// apply
	string swingUpperLegName = "LegUpper."+swingLeg;
	string swingLowerLegName = "LegLower."+swingLeg;
	//string swingFootName = "Foot."+swingLeg;
	
	mForwardDynamicsSkeleton->setOrientationTarget(swingUpperLegName, upperLegTargetWorld);
	mForwardDynamicsSkeleton->setAngularVelocityTarget(swingUpperLegName, upperLegAngularVelocityTargetWorld);
	mForwardDynamicsSkeleton->setOrientationTarget(swingLowerLegName, lowerLegTargetWorld);
	mForwardDynamicsSkeleton->setAngularVelocityTarget(swingLowerLegName, lowerLegAngularVelocityTargetWorld);
	
}

Ogre::Vector3 VirtualModelSkeletonController::computeIPStepLocation()
{
	Vector3 step(0,0,0);
	auto stanceAnkle = getStanceAnkle();
	double h = fabsf(mCoM.y - stanceAnkle->getPositionWorld().y);
	Ogre::Vector3 v = getCoMVelocityInCharacterFrame();
	step.x = v.x * sqrt(h/9.8 + v.x * v.x / (4*9.8*9.8)) * 1.3;
	step.z = v.z * sqrt(h/9.8 + v.z * v.z / (4*9.8*9.8)) * 1.1;
	step.y = 0;
	if ( getStanceIsLeft() )
		mFootIPTargetR = step;
	else
		mFootIPTargetL = step;
	return step;
}

float VirtualModelSkeletonController::adjustCoronalStepLocation(float stepLocation)
{
	// todo
	stepLocation += getStanceIsLeft()?-mStepWidth:mStepWidth;
	return stepLocation;
}

Ogre::Vector3 VirtualModelSkeletonController::computeSwingFootLocationEstimate( Ogre::Vector3 comPos, float phi )
{
	Vector3 step = computeIPStepLocation();

	//applying the IP prediction would make the character stop, so take a smaller step if you want it to walk faster, or larger
	//if you want it to go backwards
	step.z -= mTargetCoMVelocitySagittal / 20;
	//and adjust the stepping in the coronal plane in order to account for desired step width...
	step.x = adjustCoronalStepLocation(step.x);

	float legLength = mLegLength;
	step.z = min(max(-0.4f*legLength,step.z),0.4f*legLength);
	step.x = min(max(-0.4f*legLength,step.x),0.4f*legLength);

	Vector3 result(0,0,0);
	// com is in world space
	Vector3 swingFootStartPos = mInitialSwingFootPosition;
	Vector3 initialStep = swingFootStartPos - comPos;
	initialStep = mCharacterFrame.Inverse() * initialStep;
	//when phi is small, we want to be much closer to where the initial step is - so compute this quantity in character-relative coordinates
	//now interpolate between this position and initial foot position - but provide two estimates in order to provide some gradient information
	float t = (1.0f-phi);
	t = t * t;
	clamp(t, 0, 1);

	/*
	Vector3 suggestedViaPoint(0,0,0);
	alternateFootTraj.clear();
	bool needToStepAroundStanceAnkle = false;
	if (phase < 0.8 && shouldPreventLegIntersections && getPanicLevel() < 0.5)
		needToStepAroundStanceAnkle = detectPossibleLegCrossing(step, &suggestedViaPoint);
	if (needToStepAroundStanceAnkle){
		//use the via point...
		Vector3d currentSwingStepPos(comPos, lowLCon->swingFoot->state.position);
		currentSwingStepPos = lowLCon->characterFrame.inverseRotate(initialStep);currentSwingStepPos.y = 0;		
		//compute the phase for the via point based on: d1/d2 = 1-x / x-phase, where d1 is the length of the vector from
		//the via point to the final location, and d2 is the length of the vector from the swing foot pos to the via point...
		double d1 = (step - suggestedViaPoint).length(); double d2 = (suggestedViaPoint - currentSwingStepPos).length(); if (d2 < 0.0001) d2 = d1 + 0.001;
		double c =  d1/d2;
		double viaPointPhase = (1+phase*c)/(1+c);
		//now create the trajectory...
		alternateFootTraj.addKnot(0, initialStep);
		alternateFootTraj.addKnot(viaPointPhase, suggestedViaPoint);
		alternateFootTraj.addKnot(1, step);
		//and see what the interpolated position is...
		result = alternateFootTraj.evaluate_catmull_rom(1-t);
//		tprintf("t: %lf\n", 1-t);
	}else{*/
		result = step*(1.0f-t) + initialStep*t;
	//}

	result.y = 0;

/*
	suggestedFootPosDebug = result;
*/
	return result;
	
}

void VirtualModelSkeletonController::setDesiredSwingFootLocation( float phi, float dt )
{
	Vector3 step = computeSwingFootLocationEstimate(mCoM, phi);
	
	OgreAssert( mMotionGenerator->hasComponent("Foot.SWING Position"), "Must have 'Foot.SWING Position' component in JSON");
	auto& swingFootPos = mMotionGenerator->getComponentReference("Foot.SWING Position");
	// coronal = x, sagittal = z
	swingFootPos.mSplineX.at(0) = make_pair(phi, step.x);
	swingFootPos.mSplineZ.at(0) = make_pair(phi, step.z);
	//lowLCon->swingFootTrajectoryCoronal.setKnotValue(0, step.x);
	//lowLCon->swingFootTrajectorySagittal.setKnotValue(0, step.z);
	
	step = computeSwingFootLocationEstimate(mCoM + mCoMVelocity*dt, phi+dt);
	swingFootPos.mSplineX.at(1) = make_pair(phi+dt, step.x);
	swingFootPos.mSplineZ.at(1) = make_pair(phi+dt, step.z);
	//lowLCon->swingFootTrajectoryCoronal.setKnotValue(1, step.x);
	//lowLCon->swingFootTrajectorySagittal.setKnotValue(1, step.z);
	//to give some gradient information, here's what the position will be a short time later...
	/*
	 lowLCon->swingFootTrajectorySagittal.setKnotPosition(0, lowLCon->phi);
	 lowLCon->swingFootTrajectorySagittal.setKnotPosition(1, lowLCon->phi+dt);
	 
	 lowLCon->swingFootTrajectoryCoronal.setKnotPosition(0, lowLCon->phi);
	 lowLCon->swingFootTrajectoryCoronal.setKnotPosition(1, lowLCon->phi+dt);*/
}

void VirtualModelSkeletonController::setKneeBend( float kneeBend, bool swingAlso )
{
	OgreAssert( mMotionGenerator->hasComponent("LegLower.STANCE Orientation"), "Must have 'LegLower.STANCE Orientation' component in JSON");
	auto& stanceKneeOri = mMotionGenerator->getComponentReference("LegLower.STANCE Orientation");
	Ogre::Vector3 offs = stanceKneeOri.getOffset();
	offs.x = kneeBend;
	stanceKneeOri.setOffset( offs );
		
	if (swingAlso) {
		OgreAssert( mMotionGenerator->hasComponent("LegLower.SWING Orientation"), "Must have 'LegLower.SWING Orientation' component in JSON");
		auto swingKneeOri = mMotionGenerator->getComponentReference("LegLower.SWING Orientation");
		Ogre::Vector3 offs = swingKneeOri.getOffset();
		offs.x = kneeBend;
		swingKneeOri.setOffset( offs );
	}
}

void VirtualModelSkeletonController::setUpperBodyPose( float leanSagittal, float leanCoronal, float twist)
{
	int sign = getStanceIsLeft()?(-1):(1);
	
	OgreAssert( mMotionGenerator->hasComponent("SpineBase Orientation"), "Must have 'SpineBase Orientation' component in JSON");
	auto& rootComponent = mMotionGenerator->getComponentReference("SpineBase Orientation");
	rootComponent.setOffset( Ogre::Vector3(leanCoronal*sign, twist*sign*0, leanSagittal ) );
	
	OgreAssert( mMotionGenerator->hasComponent("SpineMid Orientation"), "Must have 'SpineMid Orientation' component in JSON");
	auto& spine1Component = mMotionGenerator->getComponentReference("SpineMid Orientation");
	spine1Component.setOffset( Vector3( leanCoronal*1.5*sign, twist*1.5*sign, leanSagittal*1.5 ) );
	
	OgreAssert( mMotionGenerator->hasComponent("SpineTop Orientation"), "Must have 'SpineTop Orientation' component in JSON");
	auto& spine2Component = mMotionGenerator->getComponentReference("SpineTop Orientation");
	spine2Component.setOffset( Vector3( leanCoronal*2.5*sign, twist*sign,  leanSagittal*2.5 ) );
	
	OgreAssert( mMotionGenerator->hasComponent("Neck Orientation"), "Must have 'Neck Orientation' component in JSON");
	auto& neckComponent = mMotionGenerator->getComponentReference("Neck Orientation");
	neckComponent.setOffset( Vector3( leanCoronal*1.0*sign, twist*3.0*sign, leanSagittal*3.0 ) );
}

/*
Ogre::Vector3 VirtualModelSkeletonController::getForceOnFoot( bool stance, cfs )
{
	return Ogre::Vector3(0,-1,0);
}*/


float VirtualModelSkeletonController::getStanceFootWeightRatio()
{
	//return 1.0f;
	return 0.8f;
	/*
	if (getStanceIsLeft()) {
		return 1.0f;
	} else {
		return 0.0f;
	}*/
	
	/*
	Vector3 stanceFootForce = getForceOnFoot(true, cfs);
	Vector3 swingFootForce = getForceOnFoot(false, cfs);
	Ogre::Vector3 up = -gravity.normalisedCopy();
	float totalYForce = (stanceFootForce + swingFootForce).dotProduct(up);
	
	if ( fabsf(totalYForce)<FLT_EPSILON )
		return -1;
	else
		return stanceFootForce.dotProduct(up) / totalYForce;*/
}


void VirtualModelSkeletonController::update( float dt )
{
	// clear state
	mFFRootTorque = Ogre::Vector3::ZERO;
	
	// update CoM
	Ogre::Vector3 prevCoM = mCoM;
	mCoM = mForwardDynamicsSkeleton->getCenterOfMassWorld();
	// update CoM velocity
	Ogre::Vector3 newCoMVelocity = (mCoM-prevCoM)/dt;
	//Ogre::Vector3 newCoMVelocity = mForwardDynamicsSkeleton->getCenterOfMassVelocityWorld();
	// low pass filter the CoM velocity
	float alpha = powf(mCoMVelocitySmoothingFactor,dt);
	mCoMVelocity = mCoMVelocity*alpha + newCoMVelocity*(1.0f-alpha);
	
	// get heading, store as mCharacterFrame
	Ogre::Radian heading( mForwardDynamicsSkeleton->getBody("SpineBase")->getOrientationWorld().getYaw().valueRadians());
	//mCharacterFrame = Ogre::Quaternion( heading, Ogre::Vector3::UNIT_Y );
	Ogre::Vector3 gravityDirection = mDynamicsWorld->getGravity().normalisedCopy();
	if ( gravityDirection.isZeroLength() ) {
		gravityDirection = -Ogre::Vector3::UNIT_Y;
	}
	mCharacterFrame = Ogre::Quaternion( heading, -gravityDirection );

	// simStepPlan
	{
		/*
		// store the swing foot start pos
		if (lowLCon->phi <= 0.01)
			swingFootStartPos = lowLCon->swingFoot->getWorldCoordinates(bip->joints[lowLCon->swingAnkleIndex]->cJPos);*/
	
		//compute desired swing foot location...
		setDesiredSwingFootLocation( mMotionGenerator->getPhi(), dt );
		
		// rest of body
		// compensate for hip
		Ogre::Quaternion rootQ = mCharacterFrame.Inverse()*mForwardDynamicsSkeleton->getBody("SpineBase")->getOrientationWorld();
		
		/*float targetSagittalLean = 0.2f*rootQ.getPitch().valueRadians();
		
		 float currentCoronalLean = rootQ.getRoll().valueRadians();
		float targetCoronalLean = -0.2f*currentCoronalLean;*/
		
		//BLog("ubCoronalLean: %f (roll %f)", targetCoronalLean, currentCoronalLean);
		float ubTwist = 0;
		float targetSagittalLean = 0;
		float targetCoronalLean = 0;
		setUpperBodyPose(targetSagittalLean, targetCoronalLean, ubTwist);
		
		// knee bend keeps the com lifted up
		setKneeBend( mKneeBend );

	}
	
	
	
///////// begin simbicon import
	//evaluate the target orientation for every joint, using the SIMBICON state information
	if ( mDoMotionGeneration ) {
		evaluateMotionTargets( dt );
		//now overwrite the target angles for the swing hip and the swing knee in order to ensure foot-placement control
		//if (doubleStanceMode == false)
		if ( mDoSwingLegTargets ) {
			computeIKSwingLegTargets(dt);
		}
	}
	

	
	/*computePDTorques(cfs);
	
	//bubble-up the torques computed from the PD controllers
	bubbleUpTorques();*/
///////// end simbicon import
	
	
	if ( mDoGravityCompensation ) {
		computeGravityCompensationTorques();
	}

	
	if ( mDoCoMVirtualForce ) {
		COMJT();
	}
		
	
	//and now separetely compute the torques for the hips - together with the feedback term, this is what defines simbicon
	// qRootD = desired root orientation in character frame
	if ( mDoHipTorques ) {
		Quaternion qRootD = mCharacterFrame.Inverse()*Ogre::Quaternion::IDENTITY;
		if ( mMotionGenerator->hasTargetOrientationForBody("SpineBase") ) {
			qRootD = mMotionGenerator->getTargetOrientationForBody("SpineBase");
			if ( mMotionGenerator->getReferenceFrameForOrientation("SpineBase") == VirtualModelMotionComponent::RF_World ) {
				// it's in world space
				qRootD = mCharacterFrame.Inverse()*qRootD;
			}
		}
		computeHipTorques(qRootD, getStanceFootWeightRatio(/*cfs*/), mFFRootTorque);
		mDebugTargetRootOrientation = mCharacterFrame*qRootD;
	}
	
	
	ForwardDynamicsSkeletonController::update(dt);
	
}


void VirtualModelSkeletonController::computeGravityCompensationTorques( )
{
	Ogre::Vector3 gravity = mDynamicsWorld->getGravity();
	//gravity = Ogre::Vector3(0,1,0);
	std::set<std::string> names = mForwardDynamicsSkeleton->getAllBodyNames();
	
	// skip the 'stance leg'
	std::set<string> skipNames;
	
	if ( !mDoStanceLegGravityCompensation ) {
		string suffix = getStanceLegSuffix();
		skipNames.insert("LegUpper."+suffix);
		skipNames.insert("LegLower."+suffix);
		skipNames.insert("Foot."+suffix);
	}
	if ( !mDoSwingLegGravityCompensation ) {
		string suffix = getSwingLegSuffix();
		skipNames.insert("LegUpper."+suffix);
		skipNames.insert("LegLower."+suffix);
		skipNames.insert("Foot."+suffix);
	}
		
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
				Ogre::Vector3 force = gravity/invMass;
				force *= mGravityCompensationFactor;
				computeJointTorquesEquivalentToForce( name, Ogre::Vector3::ZERO, force );
			}
		}
	}
	
	
}

Ogre::Vector3 VirtualModelSkeletonController::computeCoMVirtualForce()
{

	
	
	/* 
	 // from carthwheel-lib:
	 
	 d = Vector3d(stanceFoot->getCMPosition(), comPosition);
	//d is now in world coord frame, so we'll represent it in the 'character frame'
	d = characterFrame.inverseRotate(d);
	//compute v in the 'character frame' as well
	v = characterFrame.inverseRotate(comVelocity);0
	

	 */
	
	
	
	Vector3 v = getCoMVelocityInCharacterFrame();
	// vector from cm of stance foot to cm of character, in character frame
	Vector3 dWorld = (mCoM - getStanceAnkle()->getChildFdb()->getHeadPositionWorld());
	Vector3 d = mCharacterFrame.Inverse() * dWorld;
	
	// In character frame, CoM is at (0,0,0).
	// We want to move CoM so it is directly over the stance foot.
	// We also want the distance from d to CoM to be at least the leg length.
	Vector3 comTarget = Ogre::Vector3(-d.x,max(mLegLength-d.y,0.0f),0);
	
	// velocity target
	Vector3 vTarget( mTargetCoMVelocityCoronal, 0, mTargetCoMVelocitySagittal );
	
	Vector3 acceleration = ProportionalDerivativeController::computePDForce(Ogre::Vector3::ZERO, comTarget, v, vTarget, mCoMVirtualForceKp, mCoMVirtualForceKd);
	acceleration.x *= mCoMVirtualForceScale.x;
	acceleration.y *= mCoMVirtualForceScale.y;
	acceleration.z *= mCoMVirtualForceScale.z;
	Vector3 force = acceleration*mForwardDynamicsSkeleton->getTotalMass();
	
	/*
	// clamp so it doesn't get too large
	if ( fabsf(force.x)>100.0f || fabsf(force.z)>60.0f ) {
		BLog("clamped fA (%s)", describe(force).c_str() );
	}
	clamp(force.x, -100.0f, 100.0f);
	clamp(force.z, -60.0f, 60.0f);*/

	// convert to world frame
	force = mCharacterFrame*force;
	
	/*
	float velDSagittal = mTargetCoMVelocitySagittal;
	float velDCoronal = mTargetCoMVelocityCoronal;
	
	// coronal = x, sagittal = z
	//float comOffsetCorol = (1.0f-panicLevel) * mStepWidth;
	float comOffsetCoronal = 0.0f;
	
	
	//this is the desired acceleration of the center of mass
	Vector3 desA(0,0,0);
	// todo what's with these magic numbers (30, 20, 9) ?
	//desA.z = (velDSagittal - v.z) * 30.0f;
	//desA.z = 0;
	desA.z = (velDSagittal - v.z);
	//desA.x = (-d.x + comOffsetCoronal) * 20.0f + (velDCoronal - v.x) * 9.0f;
	desA.x = (velDCoronal - v.x);
	//BLog("[%s] v: %s, desA: %s", getSwingLegSuffix().c_str(), describe(v).c_str(), describe(desA).c_str() );
	
	
//	bool doubleStanceMode = false;
//	if (doubleStanceMode) {
//		Vector3d errV = characterFrame.inverseRotate(doubleStanceCOMError*-1);
//		desA.x = (-errV.x + comOffsetCoronal) * 20 + (velDCoronal - v.x) * 9;
//		desA.z = (-errV.z + comOffsetSagittal) * 10 + (velDSagittal - v.z) * 150;
//	}
	
	//and this is the force that would achieve that - make sure it's not too large...
	Vector3 fA = (desA) * mForwardDynamicsSkeleton->getTotalMass();
	fA *= mCoMVirtualForceFactor;
	if ( fabsf(fA.x)>100.0f || fabsf(fA.z)>60.0f ) {
		BLog("clamped fA (%s)", describe(fA).c_str() );
	}
	clamp(fA.x, -100.0f, 100.0f);
	clamp(fA.z, -60.0f, 60.0f);
	
	//now change this quantity to world coordinates...
	fA = mCharacterFrame * fA;
	
	*/
	
	mDebugCoMVirtualForce = force*0.001f;
	
	return force;
	

}

/**
 This method is used to compute torques for the stance leg that help achieve a desired speed in the sagittal and lateral planes
 */
void VirtualModelSkeletonController::computeLegTorques( Ogre::SharedPtr<ForwardDynamicsJoint> ankleJoint, Ogre::SharedPtr<ForwardDynamicsJoint> kneeJoint, Ogre::SharedPtr<ForwardDynamicsJoint> hipJoint /*, std::vector<ContactPoint> *cfs*/)
{
	Vector3 fA = computeCoMVirtualForce();
	
	Vector3 p = mCoM;
	
	Vector3 r = p - ankleJoint->getPositionWorld();
	
	Vector3 ankleTorque = r.crossProduct(fA);
	//preprocessAnkleVTorque(ankleIndex, cfs, &ankleTorque);
	ankleJoint->addTorque(ankleTorque);
	
	r = p - kneeJoint->getPositionWorld();
	kneeJoint->addTorque(r.crossProduct(fA));
	
	r = p - hipJoint->getPositionWorld();
	hipJoint->addTorque(r.crossProduct(fA));
	
	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	mFFRootTorque -= r.crossProduct(fA);
	
	auto spine1Joint = mForwardDynamicsSkeleton->getJointBetween("SpineBase", "SpineMid");
	r = p - spine1Joint->getPositionWorld();
	spine1Joint->addTorque(r.crossProduct(fA)*0.1f);
	
	auto spine2Joint = mForwardDynamicsSkeleton->getJointBetween("SpineMid", "SpineTop");
	r = p - spine2Joint->getPositionWorld();
	spine2Joint->addTorque(r.crossProduct(fA)*0.1f);
}


/**
 This method is used to compute the torques that need to be applied to the stance and swing hips, given the
 desired orientation for the root and the swing hip.
 */
void VirtualModelSkeletonController::computeHipTorques(const Ogre::Quaternion& qRootD, float stanceHipToSwingHipRatio, Ogre::Vector3 ffRootTorque)
{
	//compute the total torques that should be applied to the root and swing hip, keeping in mind that
	//the desired orientations are expressed in the character frame
	Vector3 swingHipTorque;
	
	float rootControlParamsStrength = 1.0f;
	if (stanceHipToSwingHipRatio < 0)
		rootControlParamsStrength = 0;
	
	//this is the desired orientation in world coordinates
	
	/*
	//	if (SimGlobals::forceHeadingControl == false){
	//qRootD is specified in the character frame, so just maintain the current heading
	//		qRootDW = characterFrame * qRootD;
	//	}else{
	//qRootDW needs to also take into account the desired heading
	float desiredHeading = 0;
	// could use UNIT_Y or -Gravity here
	Quaternion qRootDW = Ogre::Quaternion(Ogre::Radian(desiredHeading), Ogre::Vector3::UNIT_Y) * qRootD;
	//	}
	 */
	Quaternion qRootDW = mCharacterFrame * qRootD;
	
	float rootStrength = rootControlParamsStrength;
	rootStrength = max(min(rootStrength,1.0f),0.0f);
	
	rootControlParamsStrength = 1;
	
	//so this is the net torque that the root wants to see, in world coordinates
	auto root = mForwardDynamicsSkeleton->getBody("SpineBase");
	Vector3 rootAngularVelocity = mCharacterFrame*BtOgreConverter::to(root->getBody()->getBulletRigidBody()->getAngularVelocity());
	Vector3 rootTorque = ForwardDynamicsBodyDriverPD::computePDTorque(root->getOrientationWorld(), qRootDW, rootAngularVelocity, Vector3(0,0,0), root->getKp(), root->getKd(), rootControlParamsStrength);
	
	mDebugRootTorque = rootTorque;
	
	//root->addTorque(rootTorque);
	//return;
	
	
	
	rootTorque += ffRootTorque;
	
	
	
	
	auto swingHip = getSwingHip();
	swingHipTorque = swingHip->getTorque();
	
	//we need to compute the total torque that needs to be applied at the hips so that the final net torque on the root is rootTorque
	Vector3 rootMakeupTorque(0,0,0);
	auto relevantJoints = mForwardDynamicsSkeleton->getAllJointsWithParent("SpineBase");
	for ( auto joint: relevantJoints ) {
		rootMakeupTorque -= 0.5f*joint->getTorque();
	}
	rootMakeupTorque -= rootTorque;
	
	//add to the root makeup torque the predictive torque as well (only consider the effect of the torque in the lateral plane).
	Ogre::Vector3 d = getD();
	Vector3 rootPredictiveTorque(0, 0, mRootPredictiveTorqueFactor*9.8*d.x);
	rootMakeupTorque += mCharacterFrame*rootPredictiveTorque;
	
	//assume the stance foot is in contact...
	auto stanceHip = getStanceHip();
	Vector3 stanceHipTorque = stanceHip->getTorque();
	
	//now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
	//to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!
	stanceHipTorque += 0.5f* rootMakeupTorque * stanceHipToSwingHipRatio * rootStrength;
	swingHipTorque += 0.5f* rootMakeupTorque * (1-stanceHipToSwingHipRatio) * rootStrength;
	
	float stanceHipDamping = 0.2f;
	if( stanceHipDamping > 0 ) {
		float stanceHipMaxVelocity = 1.0f;
		
		Vector3 wRel = rootAngularVelocity - BtOgreConverter::to(stanceHip->getChildFdb()->getBody()->getBulletRigidBody()->getAngularVelocity());
		float wRelLen = wRel.length();
		if (wRelLen > stanceHipMaxVelocity ) {
			wRel = wRel * (stanceHipMaxVelocity/wRelLen);
		}
		stanceHipTorque -=  wRel * (stanceHipDamping * wRelLen);
	}
	
	//now transform the torque to child coordinates, apply torque limits and then change it back to world coordinates
	Quaternion qStanceHip = stanceHip->getChildFdb()->getOrientationWorld();
	stanceHipTorque = OgreQuaternionGetComplexConjugate(qStanceHip) * stanceHipTorque;
	//limitTorque(&stanceHipTorque, &controlParams[stanceHipIndex]);
	stanceHipTorque = qStanceHip * stanceHipTorque;
	
	Quaternion qSwingHip = swingHip->getChildFdb()->getOrientationWorld();
	swingHipTorque = OgreQuaternionGetComplexConjugate(qSwingHip) * swingHipTorque;
	//limitTorque(&swingHipTorque, &controlParams[swingHipIndex]);
	swingHipTorque = qSwingHip * swingHipTorque;
	
	//and done...
	stanceHip->clearTorque();
	swingHip->clearTorque();
	stanceHip->addTorque( stanceHipTorque );
	swingHip->addTorque( swingHipTorque );
}





void VirtualModelSkeletonController::COMJT(/*DynamicArray<ContactPoint> *cfs*/)
{
	//applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
	// dp/dq, where p is the COM.
	string whichLeg = getStanceLegSuffix();
	string footName = "Foot."+whichLeg;
	string lowerLegName = "LegLower."+whichLeg;
	string upperLegName = "LegUpper."+whichLeg;
	string pelvisName = "SpineBase";
	string lowerBackName = "SpineMid";
	string midBackName = "SpineTop";


	
	/*
	int lBackIndex = character->getJointIndex("pelvis_lowerback");
	int mBackIndex = character->getJointIndex("lowerback_torso");*/

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
	float m = tibia->getMass() + femur->getMass() + pelvis->getMass() + lBack->getMass() + mBack->getMass();

	
	Ogre::Vector3 fA = computeCoMVirtualForce();

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
	
	


	string ankleJointName = lowerLegName+"-"+footName;
	string kneeJointName = upperLegName+"-"+lowerLegName;
	string hipJointName = pelvisName+"-"+upperLegName;
	string lBackJointName = pelvisName+"-"+lowerBackName;
	string mBackJointName = lowerBackName+"-"+midBackName;
	
	Ogre::Vector3 ankleTorque = f1.crossProduct(fA);
	//BLog("ankle torque %s, knee %s, hip %s, lBack %s, mBack %s", describe(ankleTorque).c_str(), describe(f2.crossProduct(fA)).c_str(), describe(f3.crossProduct(fA)).c_str(), describe(-f4.crossProduct(fA)*0.5).c_str(), describe(-f5.crossProduct(fA)*0.3).c_str() );
	//BLog("ankleTorque: %s", describe(ankleTorque).c_str());
	//Ogre::Vector3 ankleTorque(0,0,0);
	//preprocessAnkleVTorque(stanceAnkleIndex, cfs, &ankleTorque);
	mForwardDynamicsSkeleton->addJointTorque(ankleJointName, ankleTorque);
	mForwardDynamicsSkeleton->addJointTorque(kneeJointName, f2.crossProduct(fA));
	mForwardDynamicsSkeleton->addJointTorque(hipJointName, f3.crossProduct(fA));
	
	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	mFFRootTorque -= f3.crossProduct(fA);
	
	mForwardDynamicsSkeleton->addJointTorque(lBackJointName, -f4.crossProduct(fA)*0.5);
	mForwardDynamicsSkeleton->addJointTorque(mBackJointName, -f5.crossProduct(fA)*0.3);
}

void VirtualModelSkeletonController::debugDraw()
{
	ForwardDynamicsSkeletonController::debugDraw();
	
	mDebugLines->addCross(mFootTargetL, 0.1, Ogre::ColourValue(0.3, 0.8, 0.3));
	mDebugLines->addCross(mFootTargetR, 0.1, Ogre::ColourValue(0.3, 0.8, 0.3));
	mDebugLines->addCross(mFootIPTargetL, 0.1, Ogre::ColourValue( 1.0, 1.0, 0.0f));
	mDebugLines->addCross(mFootIPTargetR, 0.1, Ogre::ColourValue( 1.0, 1.0, 0.0f));
	
	//mDebugLines->addTorqueVector(mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(rootCoMWorld), mDebugTargetRootOrientation, 0.1);
	Ogre::Vector3 rootCoMWorld = mForwardDynamicsSkeleton->getBody("SpineBase")->getCoMWorld();
	mDebugLines->addAxes(rootCoMWorld, mDebugTargetRootOrientation, 0.08f);
	mDebugLines->addTorque( rootCoMWorld+Ogre::Vector3(0,-0.02,0), mDebugRootTorque, 0.1 );
	
	mDebugLines->addCross(mCoM, 0.1, Ogre::ColourValue(0.5f, 1.0f, 0.0f));
	
	// draw com virtual force
	mDebugLines->addLine(mCoM, mCoM+mDebugCoMVirtualForce, Ogre::ColourValue(0, 0.5, 1.0) );
	mDebugLines->addLine(mCoM, mCoM+mCoMVelocity, Ogre::ColourValue(0.5, 0, 1.0));
}

void VirtualModelSkeletonController::pushModel(Ogre::Vector3 force)
{
	string pelvisName = "SpineBase";
	mForwardDynamicsSkeleton->getBody(pelvisName)->addImpulse(force);
}

void VirtualModelSkeletonController::footGroundContactStateChanged( const std::string& whichFoot, bool contact, const Ogre::Vector3& contactPos )
{
	if ( contact ) {
		bool leftFoot = "L"==whichFoot;
		mMotionGenerator->footStrikeOccurred( leftFoot );
	}
		
}
