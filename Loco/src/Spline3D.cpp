//
//  Spline3D.cpp
//  Loco
//
//  Created by damian on 05/05/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#include "Spline3D.h"

void Spline3D::addKnot( float t, Ogre::Vector3 value )
{
	mSplineX.addKnot(t, value.x);
	mSplineY.addKnot(t, value.y);
	mSplineZ.addKnot(t, value.z);
}


Ogre::Vector3 Spline3D::evaluateCatmullRom( float t )
{
	Ogre::Vector3 result;
	result.x = mSplineX.evaluateCatmullRom(t);
	result.y = mSplineY.evaluateCatmullRom(t);
	result.z = mSplineZ.evaluateCatmullRom(t);
	return result;
}