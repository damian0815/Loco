//
//  Spline3D.h
//  Loco
//
//  Created by damian on 05/05/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#ifndef __Loco__Spline3D__
#define __Loco__Spline3D__

#include <iostream>
#include "Spline1D.h"
#include <Ogre/OgreVector3.h>

class Spline3D
{
public:
	
	void addKnot( float t, Ogre::Vector3 value );
	
	Ogre::Vector3 evaluateCatmullRom( float t );
	
private:
	Spline1D mSplineX;
	Spline1D mSplineY;
	Spline1D mSplineZ;
	
};


#endif /* defined(__Loco__Spline3D__) */
