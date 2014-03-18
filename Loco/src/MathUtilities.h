//
//  MathUtilities.h
//  Loco
//
//  Created on 15/03/14.
//
//

#ifndef __Loco__MathUtilities__
#define __Loco__MathUtilities__

#include <iostream>
#include <Ogre/OgreQuaternion.h>

extern Ogre::Quaternion OgreQuaternionGetConvexConjugate( const Ogre::Quaternion& q );
extern void OgreVector3ClampAllAxes( Ogre::Vector3& target, float minVal, float maxVal );
static void clamp( float &target, float minVal, float maxVal ) { target = std::min(std::max(minVal,target),maxVal); }

#endif /* defined(__Loco__MathUtilities__) */
