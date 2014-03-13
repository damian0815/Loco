//
//  Utilities.cpp
//  Loco
//
//  Created on 18/10/13.
//
//

#include "Utilities.h"
#include <Ogre/OgreVector3.h>

std::string describe( const Ogre::Quaternion& q )
{
	char buf[1024];
	snprintf(buf, 1024, "(%2.7f %2.7f %2.7f %2.7f)", q.w, q.x, q.y, q.z );
	return buf;
}

std::string describe( const Ogre::Vector3& v )
{
	char buf[1024];
	snprintf(buf, 1024, "(%2.7f,%2.7f,%2.7f)", v.x, v.y, v.z );
	return buf;
}
