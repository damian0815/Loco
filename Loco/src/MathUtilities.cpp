//
//  MathUtilities.cpp
//  Loco
//
//  Created on 15/03/14.
//
//

#include "MathUtilities.h"
#include <Ogre/OgreVector3.h>

using namespace std;

Ogre::Quaternion OgreQuaternionGetConvexConjugate( const Ogre::Quaternion& q )
{
	return Ogre::Quaternion(q.w, -q.x, -q.y, -q.z);
}

void OgreVector3ClampAllAxes( Ogre::Vector3& target, float minVal, float maxVal )
{
	target.x = min(max(target.x,minVal), maxVal);
	target.y = min(max(target.y,minVal), maxVal);
	target.z = min(max(target.z,minVal), maxVal);
}