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
			linearMotor->m_stopERP[i] = 0.8;
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

void ForwardDynamicsJoint::reset()
{
	mTorque = Ogre::Vector3::ZERO;
}



void ForwardDynamicsJoint::applyTorque()
{
// #define CONVERT_TORQUE_TO_FORCE
#ifdef CONVERT_TORQUE_TO_FORCE
	
	/*
	 http://boards.straightdope.com/sdmb/showthread.php?t=268477
	 
	 Here you go: If you know u.v as well (period = dot product), then you can determine u. We have the identity
	
	(u x v) x w = (u.w) v - (v.w) u
	
	so this implies that
	
	(u x v) x v = (u.v) v - (v.v) u
	
	Rearranging gives us
	
	u = ((u.v) v - (u x v) x v) / (v.v)
	 */
	
	// T = F x r
	// F = ((F.r)*r - T x r) / (r.r)
	
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
	Ogre::Vector3 worldPos = getPositionWorld();
	debugLines->addCross( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), 0.1f, Ogre::ColourValue(1,0.3,0.8));
	
	debugLines->addLine( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), debugLines->getParentNode()->convertWorldToLocalPosition(worldPos+Ogre::Vector3(mDebugPrevTorque.x*0.1,0,0)), Ogre::ColourValue(1,0,0) );
	debugLines->addLine( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), debugLines->getParentNode()->convertWorldToLocalPosition(worldPos+Ogre::Vector3(0,mDebugPrevTorque.y*0.1,0)), Ogre::ColourValue(0,1,0) );
	debugLines->addLine( debugLines->getParentNode()->convertWorldToLocalPosition(worldPos), debugLines->getParentNode()->convertWorldToLocalPosition(worldPos+Ogre::Vector3(0,0,mDebugPrevTorque.z*0.1)), Ogre::ColourValue(0,0,1) );
	
	
	
	
}
