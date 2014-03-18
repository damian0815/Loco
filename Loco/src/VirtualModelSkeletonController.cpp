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
: ForwardDynamicsSkeletonController(skelRootSceneNode,skeleton,dynamicsWorld,jsonSource), mFootTargetL(0,0,0), mFootTargetR(0,0,0), mCoMTarget(0,0,0), mDoGravityCompensation(false), mDoCoMVirtualForce(false), mDoFootIK(true),
mCoMkP(100.0f), mCoMkD(4.0f), mCoMVelocity(0,0,0), mCoM(0,0,0), mPrevCoM(0,0,0), mLeftKneeOut(0.5), mRightKneeOut(0.5),
mGroundPlaneBody(groundPlaneBody), mStepWidth(0.3f), mDoubleStance(false), mSwingLegPlaneOfRotation(Ogre::Vector3::UNIT_X)
{
	picojson::object jsonRoot = jsonSource.get<picojson::object>();

	mCoM = mDriveableSkeleton->getRootSceneNode()->convertLocalToWorldPosition(mDriveableSkeleton->getCenterOfMassWorld());
	mPrevCoM = mCoM;
	mCoMTarget = mCoM;
	
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
	if ( jsonRoot.count("stepWidth") ) {
		mStepWidth = jsonRoot["stepWidth"].get<double>();
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
		Quaternion qDiff = qParentF * OgreQuaternionGetConvexConjugate(qParent);
		//wParentD = qDiff.v * 2.0f/dt;
		wParentD = Vector3(qDiff.x,qDiff.y,qDiff.z) * 2.0f/dt;
		//the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we'll account for that
		const btVector3& gParentAngVel = gParent->getBody()->getBulletRigidBody()->getAngularVelocity();
		//wParentD -= gParent->getLocalCoordinates(gParent->getAngularVelocity());
		wParentD -= gParent->convertWorldToLocalPosition(BtOgreConverter::to(gParentAngVel));

		//qDiff = qChildF * qChild.getComplexConjugate();
		qDiff = qChildF * OgreQuaternionGetConvexConjugate(qChild);
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
		float sign = mMotionGenerator->getStanceIsLeft() ? 1.0f : -1.0f;
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
	if ( mMotionGenerator->getStanceIsLeft() ) {
		mFootTargetR = pNow;
	} else {
		mFootTargetL = pNow;
	}
	
	// swing leg
	string whichLeg = mMotionGenerator->getStanceIsLeft()?"R":"L";
	solveLegIK( whichLeg, pNow );
	
	
	/*
		
	pFuture = getSwingFootTargetLocation(MIN(phi+dt, 1), mCoM + mCoMVelocity * dt, mCharacterFrame);
	
	string swingUpperLegName = "LegUpper."+whichLeg;
	string swingLowerLegName = "LegLower."+whichLeg;
	string swingFootName = "Foot."+whichLeg;
	string swingToeName = "Toe."+whichLeg;
	
	auto swingHip = mForwardDynamicsSkeleton->getJointBetween(swingUpperLegName, swingLowerLegName);
	auto swingKnee = mForwardDynamicsSkeleton->getJointBetween(swingLowerLegName, swingFootName);
	auto swingAnkle = mForwardDynamicsSkeleton->getJointBetween(swingFootName, swingToeName);
	Ogre::Vector3 parentAxis = swingKnee->getPositionInParentSpace(); // == vector from hip to knee
	Ogre::Vector3 childAxis = swingAnkle->getPositionInParentSpace(); // == vector from knee to foot
	//Vector3d parentAxis = character->joints[swingHipIndex]->cJPos - character->joints[swingKneeIndex]->pJPos;
	//Vector3d childAxis = character->joints[swingKneeIndex]->cJPos - character->joints[swingAnkleIndex]->pJPos;
	
	Ogre::Vector3 childNormal(-1,0,0); // -1 in x
	computeIKQandW( swingHip, swingKnee, parentAxis, mSwingLegPlaneOfRotation, childNormal, childAxis, pNow, true, pFuture, dt);

	//computeIKQandW(swingHipIndex, swingKneeIndex, parentAxis, swingLegPlaneOfRotation, Vector3d(-1,0,0), childAxis, pNow, true, pFuture, dt);
	//	computeIKQandW(swingHipIndex, swingKneeIndex, Vector3d(0, -0.355, 0), Vector3d(1,0,0), Vector3d(1,0,0), Vector3d(0, -0.36, 0), pNow, true, pFuture, dt);*/
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
	return step;
}

float VirtualModelSkeletonController::adjustCoronalStepLocation(float stepLocation)
{
	// todo
	stepLocation += (mMotionGenerator->getStanceIsLeft())?-mStepWidth:mStepWidth;
	return stepLocation;
}

Ogre::Vector3 VirtualModelSkeletonController::computeSwingFootLocationEstimate( Ogre::Vector3 comPos, float phi )
{
	Vector3 step = computeIPStepLocation();

	/*
	//applying the IP prediction would make the character stop, so take a smaller step if you want it to walk faster, or larger
	//if you want it to go backwards
	step.z -= lowLCon->velDSagittal / 20;
	 */
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
	auto& stanceKneeOri = mMotionGenerator->getComponentReference("LegLower.STANCE Orientation");
	Ogre::Vector3 offs = stanceKneeOri.getOffset();
	offs.x = kneeBend;
	stanceKneeOri.setOffset( offs );
	
	if (swingAlso) {
		auto swingKneeOri = mMotionGenerator->getComponentReference("LegLower.SWING Orientation");
		offs = swingKneeOri.getOffset();
		offs.x = kneeBend;
		swingKneeOri.setOffset( offs );
	}
}

void VirtualModelSkeletonController::setUpperBodyPose( float leanSagittal, float leanCoronal, float twist)
{
	int sign = mMotionGenerator->getStanceIsLeft()?(-1):(1);
	
	auto& rootComponent = mMotionGenerator->getComponentReference("SpineBase Orientation");
	rootComponent.setOffset( Ogre::Vector3(leanSagittal, leanCoronal*sign, twist*sign*0 ) );
	
	auto& spine1Component = mMotionGenerator->getComponentReference("SpineMid Orientation");
	spine1Component.setOffset( Vector3( leanSagittal*1.5, leanCoronal*1.5*sign, twist*1.5*sign) );
	
	auto& spine2Component = mMotionGenerator->getComponentReference("SpineTop Orientation");
	spine2Component.setOffset( Vector3( leanSagittal*2.5, leanCoronal*2.5*sign, twist*sign ) );
	
	auto& neckComponent = mMotionGenerator->getComponentReference("Neck Orientation");
	neckComponent.setOffset( Vector3( leanSagittal*3.0, leanCoronal*1.0*sign, twist*3.0*sign ) );
}

void VirtualModelSkeletonController::update( float dt )
{
	// update CoM velocity
	mPrevCoM = mCoM;
	mCoM = mDriveableSkeleton->getRootSceneNode()->convertLocalToWorldPosition(mDriveableSkeleton->getCenterOfMassWorld());
	mCoMVelocity = (mCoM-mPrevCoM)/dt;
	
	// get heading, store as mCharacterFrame
	Ogre::Radian heading( mDriveableSkeleton->getRootSceneNode()->getOrientation().getYaw().valueRadians() + M_PI );
	mCharacterFrame = Ogre::Quaternion( heading, Ogre::Vector3::UNIT_Y );

	// simStepPlan
	{
		/*
		// store the swing foot start pos
		if (lowLCon->phi <= 0.01)
			swingFootStartPos = lowLCon->swingFoot->getWorldCoordinates(bip->joints[lowLCon->swingAnkleIndex]->cJPos);*/
	
		//compute desired swing foot location...
		setDesiredSwingFootLocation( mMotionGenerator->getPhi(), dt );
		
		// rest of body
		float ubSagittalLean = 0.0;
		float ubCoronalLean = 0;
		float ubTwist = 0;
		setUpperBodyPose(ubSagittalLean, ubCoronalLean, ubTwist);
		
		// knee bend keeps the com lifted up
		setKneeBend( -0.1 );
		

	}
	
	
	Ogre::Vector3 lFootHead = mForwardDynamicsSkeleton->getBody("Foot.L")->getHeadPositionWorld();
	Ogre::Vector3 rFootHead = mForwardDynamicsSkeleton->getBody("Foot.R")->getHeadPositionWorld();
	
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
	
	//now overwrite the target angles for the swing hip and the swing knee in order to ensure foot-placement control
	//if (doubleStanceMode == false)
	computeIKSwingLegTargets(dt);
	
	/*computePDTorques(cfs);
	
	//bubble-up the torques computed from the PD controllers
	bubbleUpTorques();*/
///////// end simbicon import
	
	
	if ( mDoGravityCompensation ) {
		computeGravityCompensationTorques();
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
	std::set<string> skipNames;
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
	
	mDebugLines->addCross(mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(mCoM), 0.1, Ogre::ColourValue(0.8, 0.3, 0.5f));
	
	// draw com virtual force
	mDebugLines->addLine(mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(mCoM), mDebugLines->getParentSceneNode()->convertWorldToLocalPosition(mCoM+mDebugCoMVirtualForce), Ogre::ColourValue(0.2, 0.2, 1.0) );
}

void VirtualModelSkeletonController::pushModel(Ogre::Vector3 force)
{
	string pelvisName = "SpineBase";
	mForwardDynamicsSkeleton->getBody(pelvisName)->addImpulse(force);
}

