//
//  Serializable.cpp
//  Loco
//
//  Copyright (c) 2013 bg. All rights reserved.
//

#include "Serializable.h"
#include <Ogre/OgreException.h>

using namespace picojson;

Ogre::Vector3 Serializable::DecodeVector3( const value& source )
{
	OgreAssert( source.is<array>(), "Source is not an array" );
	array vdata = source.get<array>();
	float x = vdata[0].get<double>();
	float y = vdata[1].get<double>();
	float z = vdata[2].get<double>();
	return Ogre::Vector3(x,y,z);
	
}

