//
//  VirtualModelMotionGenerator.h
//  Loco
//
//  Created on 25/01/14.
//
//

#ifndef __Loco__VirtualModelMotionGenerator__
#define __Loco__VirtualModelMotionGenerator__

#include <iostream>
#include <Ogre/OgreQuaternion.h>
#include <Ogre/OgreVector3.h>
#include <string>
#include <map>
#include <vector>
#include "picojson.h"
#include "Spline1D.h"

class VirtualModelMotionComponent
{
public:
	VirtualModelMotionComponent();
	VirtualModelMotionComponent( picojson::value& params );
	VirtualModelMotionComponent( const std::string& name );
	
	typedef enum _ReferencFrame {
		RF_Parent,
		RF_Character,
		RF_World
	} ReferenceFrame;
	
	const std::string& getBodyName() const { return mBodyName; }
	ReferenceFrame getReferenceFrame() const { return mReferenceFrame; }
	
	void phaseSwapped( bool stanceIsLeft );
	Ogre::Vector3 evaluateAtTime( float phi );
	
	// raw access
	Spline1D mSplineX;
	Spline1D mSplineY;
	Spline1D mSplineZ;
	
	const Ogre::Vector3& getOffset() const { return mOffset; }
	void setOffset( const Ogre::Vector3 &offs ) { mOffset = offs; }
		
private:
	std::string mBodyName;
	ReferenceFrame mReferenceFrame;
	
	Ogre::Vector3 mOffset;
	// if true, the spline is inverted on phase swap (but not the offset)
	bool mInvertXOnPhaseSwap, mInvertYOnPhaseSwap, mInvertZOnPhaseSwap;
	float mSignX, mSignY, mSignZ;
	
	unsigned int mLastPosX, mLastPosY, mLastPosZ;
	
};


class VirtualModelMotion
{
public:
	VirtualModelMotion( picojson::value& params );
	
	bool hasTargetForBody( const std::string& bodyName ) const { return mComponents.count(bodyName) > 0; }
	
	/*! @brief For orientations, the returned value rerpresents Euler angles and should be evaluated XZY. phi is 0..1. */
	Ogre::Vector3 getTargetAtTime( const std::string& bodyName, float phi );
	VirtualModelMotionComponent::ReferenceFrame getReferenceFrame( const std::string& bodyName ) const { return mComponents.at(bodyName).getReferenceFrame(); }
	
	/*! @brief Call to inform the motion that the phase swapped. stanceIsLeft indicates the new phase. */
	void phaseSwapped( bool stanceIsLeft );
	
	const std::string& getName() const { return mName; }
	
	bool hasComponent( const std::string& name ) { return mComponents.count(name); }
	void addEmptyComponent( const std::string& name );
	VirtualModelMotionComponent& getComponent( const std::string& name ) { return mComponents.at(name); }
	
private:
	
	std::string mName;
	std::map< std::string, VirtualModelMotionComponent> mComponents;
	
};


class VirtualModelMotionGenerator
{
public:
	VirtualModelMotionGenerator( picojson::value& params );
	
	/*! @brief Set the duration of a half-cycle of animation (one cycle of the motion without swapping stance/swing). Default 1.0f. */
	void setCycleDuration( float seconds ) { mCycleDuration = seconds; }
	
	/*! @brief Advance time by the given amount. Automatically swap stance/swing at the end of each cycle. 
	 @return true if phi just reset (stance/swing was swapped). */
	bool update( float deltaTime );
	
	/*! @brief Call to inform the motion generator that a foot strike occurred. If the foot is the current swing foot, it's likely (but not guaranteed) that next time update() is called the phase will swap. 
	 @param leftFoot true if the left foot struck, false if the right. */
	void footStrikeOccurred( bool leftFoot );
	
	/*! @brief Set the active motion to the given motionName.
	 @return true on success, false if motion name was not found. */
	bool setActiveMotion( const std::string& motionName );
	/*! @brief For the currently active motion, return if there is a target orientation for the given bodyName. 
	 @remarks Targets are named "<bodyName> Orientation" (or "<bodyName>.<SWING|STANCE> Orientation" for bodies with swing/stance parts that map to .L/.R depending on which leg is Swing or Stance) */
	bool hasTargetOrientationForBody( const std::string& bodyName );
	/*! @brief For the currently active motion, return the target orientation for the given bodyName at the current time. Use getReferenceFrame to determine what the orientation is relative to. */
	Ogre::Quaternion getTargetOrientationForBody( const std::string& bodyName );
	/*! @brief For the currently active motion, return the reference frame for the given bodyName. */
	VirtualModelMotionComponent::ReferenceFrame getReferenceFrameForOrientation( const std::string& bodyName );
	
	/*! @brief For the currently active motion, return if there is a target orientation for the given bodyName.
	 @remarks Targets are named "<bodyName> Position" (or "<bodyName>.<SWING|STANCE> Position" for bodies with swing/stance parts that map to .L/.R depending on which leg is Swing or Stance) */
	bool hasTargetPositionForBody( const std::string& bodyName );
	/*! @brief For the currently active motion, return the target position for the given bodyName at the current time. Use getReferenceFrame to determine what the position is relative to. */
	Ogre::Vector3 getTargetPositionForBody( const std::string& bodyName );
	VirtualModelMotionComponent::ReferenceFrame getReferenceFrameForPosition( const std::string& bodyName );
	
	bool hasComponent( const std::string& componentName );
	VirtualModelMotionComponent& getComponentReference( const std::string& componentName );
	
	float getPhi() { return mPhi; }
	bool getStanceIsLeft() { return mStanceIsLeft; }
	
private:
	
	std::string convertBodyNameToSwingOrStance( const std::string& bodyName );
	std::string getTargetOrientationNameForBodyName(const std::string& bodyName);
	
	unsigned int mCurrentMotionIndex;
	std::vector< VirtualModelMotion> mMotions;
	
	float mCycleDuration;
	float mPhi;
	// set by footStrikeOccurred if foot==swing foot
	bool mSwingStrikeOccurred;
	
	bool mStanceIsLeft;
};






#endif /* defined(__Loco__VirtualModelMotionGenerator__) */
