//
//  Serializable.h
//  Loco
//
//  Created on 19/10/13.
//
//

#ifndef Loco_Serializable_h
#define Loco_Serializable_h

#include "picojson.h"
#include <Ogre/OgreVector3.h>

class Serializable
{
public:
	virtual ~Serializable() {};
	virtual picojson::value serialize() const = 0;
	virtual void deserialize( picojson::value& source ) = 0;
	
	static Ogre::Vector3 DecodeVector3( const picojson::value& source );
};


#endif
