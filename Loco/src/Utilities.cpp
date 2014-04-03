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
	snprintf(buf, 1024, "(% 6.4f % 6.4f % 6.4f % 6.4f)", q.w, q.x, q.y, q.z );
	return buf;
}

std::string describe( const Ogre::Vector3& v )
{
	char buf[1024];
	snprintf(buf, 1024, "(% 6.4f,% 6.4f,% 6.4f)", v.x, v.y, v.z );
	return buf;
}

std::string describe( const Ogre::Vector3& v, const char* floatFormat )
{
	char buf1[1024], buf2[1024];
	snprintf(buf1, 1024, "(%s,%s,%s)", floatFormat, floatFormat, floatFormat );
	snprintf(buf2, 1024, buf1, v.x, v.y, v.z );
	return buf2;
}
