//
//  VirtualModelFSM.h
//  Loco
//
//  Created on 22/01/14.
//
//

#ifndef __Loco__VirtualModelFSM__
#define __Loco__VirtualModelFSM__

#include <iostream>
#include <string>
#include <vector>
#include <Ogre/OgreQuaternion.h>
#include "picojson.h"

class VirtualModelFSMState
{
public:
	VirtualModelFSMState( const picojson::value& params );
	
	float mDuration;
	std::map<std::string, Ogre::Quaternion > mTargetOrientations;
	
	typedef enum TargetReferenceFrame_ {
		TRF_World,
		TRF_Parent
	} TargetReferenceFrame;
	std::map<std::string, TargetReferenceFrame > mTargetReferenceFrames;
};

class VirtualModelFSM
{
public:
	VirtualModelFSM( const picojson::value& params );
	
	/*! @abstract Update the FSM, including switching states if necessary. */
	void update(float deltaTime);
	
	/*! @return true if the current state defines a target orientation for the given body name, otherwise false. */
	bool hasTargetOrientation( const std::string& bodyName );
	/*! @return The target orientation for the given body name in the current fsm state. Check getReferenceFrame to determine which reference frame the target is expressed in. */
	Ogre::Quaternion getTargetOrientation( const std::string& bodyName );

	/*! @return The reference frame for the target orientation for the given bodyName. */
	VirtualModelFSMState::TargetReferenceFrame getReferenceFrame( const std::string& bodyName );
	
private:
	
	const VirtualModelFSMState& getCurrentState() const { return mStates.at(mCurrentState); }
	
	std::vector<VirtualModelFSMState> mStates;
	int mCurrentState;
	float mCurrentStateTime;
	
};


#endif /* defined(__Loco__VirtualModelFSM__) */
