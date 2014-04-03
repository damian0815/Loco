//
//  Utilities.h
//  Loco
//
//  Created on 18/10/13.
//
//

#ifndef __Loco__Utilities__
#define __Loco__Utilities__

#ifdef DEBUG
#define DEBUG_PRINT
#endif

#ifdef DEBUG_PRINT
#define BLog(format, ...) printf("%s: " format "\n", __PRETTY_FUNCTION__, ##__VA_ARGS__)
#else
#define BLog(format, ...) do {} while (0)
#endif

#include <Ogre/OgreQuaternion.h>
#include <string>
std::string describe( const Ogre::Quaternion& q );
std::string describe( const Ogre::Vector3& q );
std::string describe( const Ogre::Vector3& v, const char* floatFormat );

#endif /* defined(__Loco__Utilities__) */
