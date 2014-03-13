//
//  VirtualModelMotionGenerator.cpp
//  Loco
//
//  Created on 25/01/14.
//
//

#include "VirtualModelMotionGenerator.h"
#include <Ogre/OgreException.h>
#include <Ogre/OgreMatrix3.h>

using namespace picojson;
using namespace std;
using namespace Ogre;

VirtualModelMotionComponent::VirtualModelMotionComponent( value& paramsValue )
: mLastPosX(0), mLastPosY(0), mLastPosZ(0)
{
	object params = paramsValue.get<object>();
	
	mBodyName = params["bodyName"].get<string>();
	
	mReferenceFrame = RF_Parent;
	if ( params.count("referenceFrame") ) {
		string referenceFrame = params["referenceFrame"].get<string>();
		if ( referenceFrame == "parent" ) {
			mReferenceFrame = RF_Parent;
		} else if ( referenceFrame == "character" ) {
			mReferenceFrame = RF_Character;
		} else {
			OgreAssert( false, "unrecognized reference frame" );
		}
	}
	
	// splines
	// there might be one for each axis x,y,z
	string splineNames[3] = { "x", "y", "z" };
	std::vector< pair<float,float> >* targetVectors[3] = { &mSplineX, &mSplineY, &mSplineZ };
	// check each axis
	for ( int i=0; i<3; i++ ) {
		float lastTime = -1;
		if ( params.count(splineNames[i]) ) {
			// spline is defined as an array of knots, defined as pairs of floats
			array splineDef = params[splineNames[i]].get<array>();
			for ( auto knotValue: splineDef ) {
				// decode the knot
				array knot = knotValue.get<array>();
				OgreAssert(knot.size()==2, "bad spline definition");
				// read time
				float t = knot[0].get<double>();
				// check
				OgreAssert(t>=0 && t<=1.0, "time out of bounds (0..1)");
				OgreAssert(t>lastTime, "time values must be strictly increasing");
				lastTime = t;
				// read angle
				float theta = knot[1].get<double>();
				// store
				targetVectors[i]->push_back(make_pair(t,theta));
			}
		}
	}
		
}

Quaternion VirtualModelMotionComponent::evaluateAtTime( float phi )
{
	
	// evaluate each spline
	float xRot=0, yRot=0, zRot=0;
	if ( mSplineX.size() ) {
		xRot = evaluateCatmullRom( mSplineX, phi, mLastPosX );
	}
	if ( mSplineY.size() ) {
		yRot = evaluateCatmullRom( mSplineY, phi, mLastPosY );
	}
	if ( mSplineZ.size() ) {
		zRot = evaluateCatmullRom( mSplineZ, phi, mLastPosZ );
	}
	
	// combine in order XZY
	Matrix3 rotMat;
	rotMat.FromEulerAnglesXZY( Radian(xRot), Radian(yRot), Radian(zRot) );
	
	return Quaternion(rotMat);
}


VirtualModelMotion::VirtualModelMotion( value& paramsValue )
{
	object params = paramsValue.get<object>();
	
	mName = params["name"].get<string>();
	
	array trajectories = params["components"].get<array>();
	for ( auto trajectoryValue: trajectories ) {
		VirtualModelMotionComponent component(trajectoryValue);
		mComponents[component.getBodyName()] = component;
	}
}

Quaternion VirtualModelMotion::getTargetAtTime( const std::string& bodyName, float phi )
{
	VirtualModelMotionComponent& component = mComponents.at(bodyName);
	Ogre::Quaternion q = component.evaluateAtTime(phi);
	return q;

}

VirtualModelMotionGenerator::VirtualModelMotionGenerator( picojson::value& paramsValue )
: mPhi(0), mStanceIsLeft(true), mCycleDuration(1.0f)
{
	object params = paramsValue.get<object>();
	
	array motions = params["motions"].get<array>();
	for ( auto motionValue: motions ) {
		mMotions.push_back( VirtualModelMotion(motionValue) );
	}

}

void VirtualModelMotionGenerator::update(float dt)
{
	mPhi += dt/mCycleDuration;
	while ( mPhi>1.0 ) {
		mPhi -= 1.0;
		mStanceIsLeft = !mStanceIsLeft;
	}
}

bool VirtualModelMotionGenerator::hasTarget(const std::string &bodyName)
{
	string bodyNameStanceSwing = convertBodyNameToSwingOrStance(bodyName);
	return mMotions[mCurrentMotionIndex].hasTargetForBody(bodyNameStanceSwing);
}

Ogre::Quaternion VirtualModelMotionGenerator::getTarget( const std::string& bodyName )
{
	string bodyNameStanceSwing = convertBodyNameToSwingOrStance(bodyName);
	return mMotions[mCurrentMotionIndex].getTargetAtTime( bodyNameStanceSwing, mPhi );
}

VirtualModelMotionComponent::ReferenceFrame VirtualModelMotionGenerator::getReferenceFrame(const std::string &bodyName)
{
	string bodyNameStanceSwing = convertBodyNameToSwingOrStance(bodyName);
	return mMotions[mCurrentMotionIndex].getReferenceFrame(bodyNameStanceSwing);
}

bool VirtualModelMotionGenerator::setActiveMotion( const std::string& motionName )
{
	for ( size_t i=0; i<mMotions.size(); i++ ) {
		if ( mMotions[i].getName() == motionName ) {
			mCurrentMotionIndex = (unsigned int)i;
			return true;
		}
	}
	OgreAssert( false, "motion name not recognized" );
	return false;
}

string VirtualModelMotionGenerator::convertBodyNameToSwingOrStance( const std::string& bodyName )
{
	// check for '.L' or '.R' on the end
	if ( bodyName.size()>2 ) {
		const string& sideSuffix = bodyName.substr(bodyName.size()-2);
		const string& baseName = bodyName.substr(0,bodyName.size()-2);
		if ( ".L" == sideSuffix ) {
			if ( mStanceIsLeft ) {
				return baseName+string(".STANCE");
			} else {
				return baseName+string(".SWING");
			}
		} else if ( ".R" == sideSuffix ) {
			if ( !mStanceIsLeft ) {
				return baseName+string(".STANCE");
			} else {
				return baseName+string(".SWING");
			}
		}
	}
	
	// fallback
	return bodyName;
			
}

