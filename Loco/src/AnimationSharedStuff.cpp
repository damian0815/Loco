//
//  AnimationSharedStuff.cpp
//  Loco
//
//  Created on 18/10/13.
//
//

#include <stdio.h>
#include "AnimationSharedStuff.h"
#include <Ogre/OgreVector3.h>

double getScalarPart( const Ogre::Quaternion& q )
{
	return q.w;
}

Ogre::Vector3 getVectorPart( const Ogre::Quaternion& q )
{
	return Ogre::Vector3( q.x, q.y, q.z );
}

Ogre::Quaternion makeEulerQuaternion( const Ogre::Vector3& euler )
{
	Ogre::Quaternion eulerQ = Ogre::Quaternion( Ogre::Radian(euler.y), Ogre::Vector3::UNIT_Y ) * Ogre::Quaternion( Ogre::Radian(euler.x), Ogre::Vector3::UNIT_X ) * Ogre::Quaternion( Ogre::Radian(euler.z), Ogre::Vector3::UNIT_Z );
	return eulerQ;
}

Ogre::Vector3 makeEulerVector( const Ogre::Quaternion& rotation )
{
	return Ogre::Vector3( rotation.getPitch(false).valueRadians(), rotation.getYaw(false).valueRadians(), rotation.getRoll(false).valueRadians() );
}



Ogre::Vector3 rotateEuler( const Ogre::Quaternion& q, const Ogre::Vector3& euler )
{
	Ogre::Quaternion result = q*makeEulerQuaternion(euler);
	return makeEulerVector(result);
}