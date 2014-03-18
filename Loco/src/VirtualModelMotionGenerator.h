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

class VirtualModelMotionComponent
{
public:
	VirtualModelMotionComponent() {};
	VirtualModelMotionComponent( picojson::value& params );
	
	typedef enum _ReferencFrame {
		RF_Parent,
		RF_Character,
		RF_World
	} ReferenceFrame;
	
	const std::string& getBodyName() const { return mBodyName; }
	ReferenceFrame getReferenceFrame() const { return mReferenceFrame; }
	
	void phaseSwapped();
	Ogre::Vector3 evaluateAtTime( float phi );
	
	// raw access
	std::vector< std::pair<float,float> > mSplineX;
	std::vector< std::pair<float,float> > mSplineY;
	std::vector< std::pair<float,float> > mSplineZ;
	
	const Ogre::Vector3& getOffset() const { return mOffset; }
	void setOffset( const Ogre::Vector3 &offs ) { mOffset = offs; }
		
private:
	std::string mBodyName;
	ReferenceFrame mReferenceFrame;
	
	Ogre::Vector3 mOffset;
	// if true, the spline is inverted on phase swap (but not the offset)
	bool mInvertOnPhaseSwap;
	float mSign;
	
	unsigned int mLastPosX, mLastPosY, mLastPosZ;
	
	inline static float evaluateCatmullRom( const std::vector< std::pair<float,float> >& spline, float phi, unsigned int &lastPos );
};


class VirtualModelMotion
{
public:
	VirtualModelMotion( picojson::value& params );
	
	bool hasTargetForBody( const std::string& bodyName ) const { return mComponents.count(bodyName) > 0; }
	
	/*! @brief For orientations, the returned value rerpresents Euler angles and should be evaluated XZY. phi is 0..1. */
	Ogre::Vector3 getTargetAtTime( const std::string& bodyName, float phi );
	VirtualModelMotionComponent::ReferenceFrame getReferenceFrame( const std::string& bodyName ) const { return mComponents.at(bodyName).getReferenceFrame(); }
	
	void phaseSwapped();
	
	const std::string& getName() const { return mName; }
	
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
	
	VirtualModelMotionComponent& getComponentReference( const std::string& componentName );
	
	float getPhi() { return mPhi; }
	bool getStanceIsLeft() { return mStanceIsLeft; }
	
private:
	
	std::string convertBodyNameToSwingOrStance( const std::string& bodyName );
	
	unsigned int mCurrentMotionIndex;
	std::vector< VirtualModelMotion> mMotions;
	
	float mCycleDuration;
	float mPhi;
	
	bool mStanceIsLeft;
};






/**
 This method returns the index of the first knot whose value is larger than the parameter value t. If no such index exists (t is larger than any
 of the values stored), then values.size() is returned.
 
 from cartwheel-3d
 
 */
static inline int getFirstLargerIndex(double t, const std::vector< std::pair<float,float> >& spline, unsigned int &lastIndex )
{
	int size = spline.size();
	if( size == 0 ) 
		return 0;
	if( t < spline[(lastIndex+size-1)%size].first )
		lastIndex = 0;
	for (int i = 0; i<size;i++){
		int index = (i + lastIndex) % size;
		if (t < spline[index].first ) {
			lastIndex = index;
			return index;
		}
	}
	return size;
}

/**
 This method interprets the trajectory as a Catmul-Rom spline, and evaluates it at the point t
 
 from cartwheel-3d
 */
float VirtualModelMotionComponent::evaluateCatmullRom( const std::vector< std::pair<float,float> >& spline, float phi, unsigned int &lastPos )
{
	float t = phi;
	
	int size = spline.size();
	if( size == 0 ) return 0.0f;
	if (t<=spline[0].first) return spline[0].second;
	if (t>=spline[size-1].first) return spline[size-1].second;
	int index = getFirstLargerIndex(t, spline, lastPos);
	
	//now that we found the interval, get a value that indicates how far we are along it
	t = (t-spline[index-1].first) / (spline[index].first-spline[index-1].first);
	
	//approximate the derivatives at the two ends
	float t0, t1, t2, t3;
	float p0, p1, p2, p3;
	p0 = (index-2<0)?(spline[index-1].second):(spline[index-2].second);
	p1 = spline[index-1].second;
	p2 = spline[index].second;
	p3 = (index+1>=size)?(spline[index].second):(spline[index+1].second);
	
	t0 = (index-2<0)?(spline[index-1].first):(spline[index-2].first);
	t1 = spline[index-1].first;
	t2 = spline[index].first;
	t3 = (index+1>=size)?(spline[index].first):(spline[index+1].first);
	
	float d1 = (t2-t0);
	float d2 = (t3-t1);
	
#define TINY FLT_EPSILON
	if (d1 > -TINY && d1  < 0) d1 = -TINY;
	if (d1 < TINY && d1  >= 0) d1 = TINY;
	if (d2 > -TINY && d2  < 0) d2 = -TINY;
	if (d2 < TINY && d2  >= 0) d2 = TINY;
	
#ifdef FANCY_SPLINES
	float m1 = (p2 - p0) * (1-(t1-t0)/d1);
	float m2 = (p3 - p1) * (1-(t3-t2)/d2);
#else
	float m1 = (p2 - p0)*0.5;
	float m2 = (p3 - p1)*0.5;
#endif
	
	t2 = t*t;
	t3 = t2*t;
	
	//and now perform the interpolation using the four hermite basis functions from wikipedia
	return p1*(2*t3-3*t2+1) + m1*(t3-2*t2+t) + p2*(-2*t3+3*t2) + m2 * (t3 - t2);
}


#endif /* defined(__Loco__VirtualModelMotionGenerator__) */
