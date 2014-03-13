//
//  VirtualModelFSM.cpp
//  Loco
//
//  Created on 22/01/14.
//
//

#include "VirtualModelFSM.h"
#include <Ogre/OgreVector3.h>
#include <Ogre/OgreException.h>
#include "Serializable.h"

using namespace std;
using namespace picojson;
using namespace Ogre;

VirtualModelFSMState::VirtualModelFSMState( const value& paramsValue )
{
	// read params out of json
	object params = paramsValue.get<object>();
	
	// duration
	mDuration = params["duration"].get<double>();

	
	// individual joint targets
	array targets = params["targets"].get<array>();
	for ( auto targetValue: targets ) {
		object target = targetValue.get<object>();
		string name = target["bodyName"].get<string>();
		// get target angle as angle/axis
		Vector3 axis = Serializable::DecodeVector3(target["axis"]);
		float angle = target["angle"].get<double>();
		// convert to quaternion
		Quaternion q( Radian(angle), axis );
		mTargetOrientations[name] = q;
		// check for a reference frame
		TargetReferenceFrame referenceFrame = TRF_Parent;
		if ( target.count("referenceFrame") ) {
			string frame = target["referenceFrame"].get<string>();
			if ( frame == "world" ) {
				referenceFrame = TRF_World;
			} else if ( frame == "parent" ) {
				referenceFrame = TRF_Parent;
			} else {
				OgreAssert(false, "unrecognized reference frame in json");
			}
		}
		mTargetReferenceFrames[name] = referenceFrame;
	}
}

VirtualModelFSM::VirtualModelFSM( const value& paramsValue )
{
	// read FSM states out of json
	object params = paramsValue.get<object>();
	
	array states = params["states"].get<array>();
	for ( auto stateValue: states ) {
		VirtualModelFSMState state(stateValue);
		mStates.push_back(state);
	}
	
	mCurrentStateTime = 0;
	mCurrentState = 0;
}

void VirtualModelFSM::update(float deltaTime)
{
	mCurrentStateTime += deltaTime;
	while ( mCurrentStateTime > getCurrentState().mDuration ) {
		mCurrentStateTime -= getCurrentState().mDuration;
		mCurrentState = (mCurrentState+1) % mStates.size();
	}
}


bool VirtualModelFSM::hasTargetOrientation( const std::string& bodyName )
{
	return getCurrentState().mTargetOrientations.count(bodyName) > 0;
}

Ogre::Quaternion VirtualModelFSM::getTargetOrientation(const std::string &bodyName)
{
	return getCurrentState().mTargetOrientations.at(bodyName);
}

VirtualModelFSMState::TargetReferenceFrame VirtualModelFSM::getReferenceFrame(const std::string &bodyName)
{
	return getCurrentState().mTargetReferenceFrames.at(bodyName);
}
