//
//  ForwardDynamicsJoint.cpp
//  Loco
//
//  Created on 07/11/13.
//
//

#include "ForwardDynamicsJoint.h"
#include "ForwardDynamicsBody.h"
#include "OgreBulletDynamicsRigidBody.h"
#include "OgreBulletConverter.h"
#include "btRigidBody.h"
#include "Utilities.h"
#include "MathUtilities.h"

using namespace std;
using namespace OgreBulletCollisions;

float ForwardDynamicsJoint::mLimitDamping = 1.0;
float ForwardDynamicsJoint::mLimitSoftness = 0.5;
float ForwardDynamicsJoint::mLinearLimitDamping = 1.0;
float ForwardDynamicsJoint::mLinearLimitSoftness = 0.7;


ForwardDynamicsJoint::ForwardDynamicsJoint(OgreBulletDynamics::DynamicsWorld* dynamicsWorld, Ogre::SharedPtr<ForwardDynamicsBody> parentFdb, Ogre::SharedPtr<ForwardDynamicsBody> childFdb, picojson::object jointDef)
: mChildFdb(childFdb), mParentFdb(parentFdb), mDynamicsWorld(dynamicsWorld), mTorque(0,0,0), mDebugPrevTorque(0,0,0), mSecondaryConstraint(NULL)
{
	
	// setup values common to all constraints
	auto bodyA = parentFdb;
	auto bodyB = childFdb;
	btRigidBody* bodyABody = bodyA->getBody()->getBulletRigidBody();
	btRigidBody* bodyBBody = bodyB->getBody()->getBulletRigidBody();
	
	Ogre::Vector3 pivotInA = bodyA->getTailPositionLocal(childFdb->getName());
	Ogre::Vector3 pivotInB = bodyB->getHeadPositionLocal();
	
	Ogre::Quaternion orientationOfBRelativeToA = bodyA->getOrientationWorld().Inverse() * bodyB->getOrientationWorld();
	
	// get joint type
	string jointType = "hinge";
	if ( jointDef.count("type") ) {
		jointType = jointDef["type"].get<string>();
	}
	
	// create the joint
	btTypedConstraint* constraint = NULL;
	
	// hinge joint
	if ( jointType == "hinge" ) {
		string axis = "z";
		if ( jointDef.count("axis") ) {
			axis = jointDef["axis"].get<string>();
		}
		Ogre::Vector3 axisInB = Ogre::Vector3::UNIT_Z;
		if ( axis == "x" ) {
			axisInB = Ogre::Vector3::UNIT_X;
		} else if ( axis == "y" ) {
			axisInB = Ogre::Vector3::UNIT_Y;
		} else if ( axis == "z" ) {
			axisInB = Ogre::Vector3::UNIT_Z;
		} else {
			OgreAssert( false, "invalid axis in json");
		}
		
		Ogre::Vector3 axisInA = orientationOfBRelativeToA*axisInB;
		
		// create
		btHingeConstraint* con = new btHingeConstraint( *bodyABody, *bodyBBody, OgreBtConverter::to(pivotInA), OgreBtConverter::to(pivotInB), OgreBtConverter::to(axisInA), OgreBtConverter::to(axisInB) );
		
		if ( jointDef.count("minAngle") && jointDef.count("maxAngle") ) {
			con->setLimit(jointDef["minAngle"].get<double>(), jointDef["maxAngle"].get<double>());
		} else if ( jointDef.count("minAngle") || jointDef.count("maxAngle") ) {
			BLog("Warning: missing minANgle or maxAngle definition for joint connecting %s to %s", bodyA->getName().c_str(), bodyB->getName().c_str() );
		}
		
		constraint = con;
		
	} else if ( jointType == "coneTwist" ) {
		
		/*
		string twistAxis = "y";
		if ( jointDef.count("twistAxis") ) {
			twistAxis = jointDef["twistAxis"].get<string>();
		}*/
		
		
		// create
		// A frame goes from bodyA's local space to the constraint's local space
		
		/*
		// for cone-twist constraint:
		// constraint has twist axis x
		// child has it as y
		
		// constraintQ is childQ rotated -90 degrees about z
		Ogre::Quaternion constraintQWorld = bodyB->getOrientationWorld()*Ogre::Quaternion( Ogre::Radian(-M_PI_2), Ogre::Vector3::UNIT_Z );
		Ogre::Vector3 constraintPosWorld = bodyB->getHeadPositionWorld();
		btTransform constraintWorldTransform(OgreBtConverter::to(constraintQWorld), OgreBtConverter::to(constraintPosWorld));
		 */
		
		// for generic 6dof constraint:
		btTransform constraintWorldTransform(OgreBtConverter::to(bodyB->getOrientationWorld()), OgreBtConverter::to(bodyB->getHeadPositionWorld()));
		
		// get frames
		// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=7853&sid=21b3f9941f23b61ac3db08467f2da5c5#p7853
		btTransform aFrame = bodyABody->getWorldTransform().inverse() * constraintWorldTransform;
		btTransform bFrame = bodyBBody->getWorldTransform().inverse() * constraintWorldTransform;
		
		btGeneric6DofConstraint* con = new btGeneric6DofConstraint( *bodyABody, *bodyBBody, aFrame, bFrame, false );
		
		/*
		btConeTwistConstraint* con = new btConeTwistConstraint( *bodyABody, *bodyBBody, aFrame, bFrame);*/
		
		// load limits
		/*
		// for coneTwist:
		float swing1Span = M_PI;
		float swing2Span = M_PI;
		float twistSpan = M_PI;
		if ( jointDef.count("twistSpan") ) {
			twistSpan = jointDef["twistSpan"].get<double>();
		}
		if ( jointDef.count("swing1Span") ) {
			swing1Span = jointDef["swing1Span"].get<double>();
		}
		if ( jointDef.count("swing2Span") ) {
			swing2Span = jointDef["swing2Span"].get<double>();
		}
		 
		// defaults 1, 0.3, 1
		float softness = 0.9f;
		float biasFactor = 0.3f;
		float relaxationFactor = 1.0f;
		
		con->setLimit(swing1Span, swing2Span, twistSpan, softness, biasFactor, relaxationFactor );
		 */
		
		// for generic 6dof:
		
		// span
		float swing1Max = M_PI_2;
		float swing2Max = M_PI_2;
		float twistMax = M_PI_2;
		if ( jointDef.count("twistSpan") ) {
			twistMax = jointDef["twistSpan"].get<double>();
		}
		if ( jointDef.count("swing1Span") ) {
			swing1Max = jointDef["swing1Span"].get<double>();
		}
		if ( jointDef.count("swing2Span") ) {
			swing2Max = jointDef["swing2Span"].get<double>();
		}
		float swing1Min = -swing1Max;
		float swing2Min = -swing2Max;
		float twistMin = -twistMax;
		// min/max separate
		if ( jointDef.count("swing1Max") ) {
			swing1Max = jointDef.at("swing1Max").get<double>();
		}
		if ( jointDef.count("swing2Max") ) {
			swing2Max = jointDef.at("swing2Max").get<double>();
		}
		if ( jointDef.count("twistMax") ) {
			twistMax = jointDef.at("twistMax").get<double>();
		}
		if ( jointDef.count("swing1Min") ) {
			swing1Min = jointDef.at("swing1Min").get<double>();
		}
		if ( jointDef.count("swing2Min") ) {
			swing2Min = jointDef.at("swing2Min").get<double>();
		}
		if ( jointDef.count("twistMin") ) {
			twistMin = jointDef.at("twistMin").get<double>();
		}
		con->setAngularLowerLimit(btVector3(swing1Min,twistMin,swing2Min));
		con->setAngularUpperLimit(btVector3(swing1Max,twistMax,swing2Max));
		
		// make position limits extremely hard
		btTranslationalLimitMotor* linearMotor = con->getTranslationalLimitMotor();
		linearMotor->m_limitSoftness = mLinearLimitSoftness;
		linearMotor->m_damping = mLinearLimitDamping;
		for ( int i=0; i<3; i++ ) {
			linearMotor->m_enableMotor[i] = true;
			//linearMotor->m_stopERP[i] = 0.8;
			con->getRotationalLimitMotor(i)->m_enableMotor = true;
			con->getRotationalLimitMotor(i)->m_limitSoftness = mLimitSoftness;
			con->getRotationalLimitMotor(i)->m_damping = mLimitDamping;
		}
		//constraint->setParam(BT_CONSTRAINT_STOP_CFM, myCFMvalue, index)
		//constraint->setParam(BT_CONSTRAINT_STOP_ERP, myERPvalue, index)
		
		constraint = con;
		
	} else {
		OgreAssert(false, "unrecognized joint type in json");
	}
	
	OgreAssert(constraint, "didn't make a constraint");
	

	mConstraint = constraint;
	mDynamicsWorld->getBulletDynamicsWorld()->addConstraint(constraint);
	
	/*
	// create position constraint
	btPoint2PointConstraint* constraint2 = new btPoint2PointConstraint(*bodyABody, *bodyBBody, OgreBtConverter::to(pivotInA), OgreBtConverter::to(pivotInB) );
	mDynamicsWorld->getBulletDynamicsWorld()->addConstraint(constraint2);
	mSecondaryConstraint = constraint2;*/
	
}

ForwardDynamicsJoint::~ForwardDynamicsJoint()
{
	mDynamicsWorld->getBulletDynamicsWorld()->removeConstraint(mConstraint);
	delete mConstraint;
	mConstraint = NULL;
	if ( mSecondaryConstraint ) {
		mDynamicsWorld->getBulletDynamicsWorld()->removeConstraint(mSecondaryConstraint);
		delete mSecondaryConstraint;
		mSecondaryConstraint = NULL;
	}

}

Ogre::Vector3 ForwardDynamicsJoint::getPositionInParentSpace()
{
	return mPosParent;
}

Ogre::Vector3 ForwardDynamicsJoint::getPositionInChildSpace()
{
	return mPosChild;
}

Ogre::Vector3 ForwardDynamicsJoint::getPositionWorld()
{
	return getChildFdb()->getHeadPositionWorld();
}

void ForwardDynamicsJoint::addTorque( const Ogre::Vector3& t )
{
	mTorque += t;
}

void ForwardDynamicsJoint::clearTorque()
{
	mTorque = Ogre::Vector3::ZERO;
}



void ForwardDynamicsJoint::applyTorque()
{
	
	// clamp torque
	Ogre::Vector3 torqueParent = mTorque;
	Ogre::Vector3 torqueChild = mTorque;

	OgreVector3ClampAllAxes(torqueParent, -mParentFdb->getMaxTorque(), mParentFdb->getMaxTorque());
	OgreVector3ClampAllAxes(torqueChild, -mChildFdb->getMaxTorque(), mChildFdb->getMaxTorque());
	
	
#define CONVERT_TORQUE_TO_FORCE
#ifdef CONVERT_TORQUE_TO_FORCE
	
	/*
	 http://answers.unity3d.com/questions/407085/add-torque-at-position.html
	 
	 This works pretty much like spinning a pencil on the table. Try it. You will place two fingers on the opposite side of the pencil and push in the opposite direction. This results in a torque around the point in the middle of your fingers.
	 
	 Your custom AddTorqueAtPosition function needs to do the same thing. I'll call the position of the added torque hinge and the axis of the rotation torque now.
	 
	 First you need to find two vectors which are orthogonal to torque and to eachother. This is a bit tricky if you don't know the direction of hinge. I'd suggest to copy hinge into a dummy vector. Then, if hinge does not point into (1,0,0) use dummy and ortho = new vector3(1,0,0) in Vector3.OrthoNormalize(dummy, ortho). Otherwise use (0,1,0) as ortho. The last vector force can be found by using Vector3.Cross on 0.5*hinge and ortho. Remember the right hand rule to find the direction of the new vector. We use half the vector in our calculation, because we will use force twice.
	 
	 Now you can use addForceAtPosition. The parameters are: force is simply the force vector we calculated, position is the position of hinge + ortho. Do the same thing again with -force and hinge - ortho.
	 
	 Your complete AddTorqueAtPosition function should take a vector3 torque , a vector3 hinge and a Forcemode which you can pass to the addForceAtPosition funktions.
	 
	 This is 100% crafted by theory, so let me know whether this works or not.
	 */
	
	// find two vectors orthogonal to torque and to each other
	Ogre::Vector3 torqueAxis( mTorque.x, mTorque.y, mTorque.z );
	torqueAxis.normalise();
	
	Ogre::Vector3 ortho(1,0,0);
	if ( (torqueAxis-ortho).squaredLength()<FLT_EPSILON ) {
		ortho = Ogre::Vector3(0,1,0);
	}
	// orthoNormalize(torqueAxis	, ortho);
	Ogre::Vector3 dummy = torqueAxis;
	dummy.normalise();
	Ogre::Vector3 orthoNormIntermediate = dummy.crossProduct(ortho);
	orthoNormIntermediate.normalise();
	ortho = orthoNormIntermediate.crossProduct(dummy);
	
	// use clamped torques
	Ogre::Vector3 forceParent = (0.5f*torqueParent).crossProduct(ortho);
	Ogre::Vector3 forceChild = (0.5f*torqueChild).crossProduct(ortho);
		
	// apply force at hinge pos
	Ogre::Vector3 hingePos = getPositionWorld();
	Ogre::Vector3 pos1 = hingePos + ortho;
	Ogre::Vector3 pos2 = hingePos - ortho;
		
	// apply force to each of parent and child
	btVector3 forceBt = OgreBtConverter::to(forceParent);
	// apply to parent
	mParentFdb->getBody()->getBulletRigidBody()->applyForce(  forceBt, OgreBtConverter::to(mParentFdb->convertWorldToLocalPosition(pos1)) );
	mParentFdb->getBody()->getBulletRigidBody()->applyForce( -forceBt, OgreBtConverter::to(mParentFdb->convertWorldToLocalPosition(pos2)) );
	// apply to child
	forceBt = OgreBtConverter::to(forceChild);
	mChildFdb->getBody()->getBulletRigidBody()->applyForce( -forceBt, OgreBtConverter::to(mChildFdb->convertWorldToLocalPosition(pos1)) );
	mChildFdb->getBody()->getBulletRigidBody()->applyForce(  forceBt, OgreBtConverter::to(mChildFdb->convertWorldToLocalPosition(pos2)) );
	
	if ( getChildFdb()->getName() == "LegUpper.L" ) {
		BLog("LegUpper.L(const %llx): torque %s force %s", (unsigned long long)mConstraint, describe(torqueChild).c_str(), describe(forceChild).c_str() );
	}
	
	
#else
	// apply to parent
	mParentFdb->addTorque(mTorque);
	// apply to child
	mChildFdb->addTorque(-mTorque);
#endif
	
	// clear the torque
	mDebugPrevTorque = mTorque;
	mTorque = Ogre::Vector3::ZERO;
}

void ForwardDynamicsJoint::debugDraw(OgreBulletCollisions::DebugLines* debugLines)
{
	/*
	Ogre::Vector3 worldPos = getPositionWorld();
	debugLines->addCross( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), 0.1f, Ogre::ColourValue(1,0.3,0.8));
	
	debugLines->addLine( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), debugLines->getParentNode()->convertWorldToLocalPosition(worldPos+Ogre::Vector3(mDebugPrevTorque.x*0.1,0,0)), Ogre::ColourValue(1,0,0) );
	debugLines->addLine( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), debugLines->getParentNode()->convertWorldToLocalPosition(worldPos+Ogre::Vector3(0,mDebugPrevTorque.y*0.1,0)), Ogre::ColourValue(0,1,0) );
	debugLines->addLine( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), debugLines->getParentNode()->convertWorldToLocalPosition(worldPos+Ogre::Vector3(0,0,mDebugPrevTorque.z*0.1)), Ogre::ColourValue(0,0,1) );
	*/
	
	
	
}
