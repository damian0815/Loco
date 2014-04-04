//
//  ProportionalDerivativeController.h
//  Loco
//
//  Created by damian on 03/04/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#ifndef __Loco__ProportionalDerivativeController__
#define __Loco__ProportionalDerivativeController__

#include <iostream>

#include <Ogre/OgreVector3.h>
#include <Ogre/OgreQuaternion.h>

class ProportionalDerivativeController
{
public:
/**
 
 from SimBiCon
 
 This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
 Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
 the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque
 returned is expressed in the coordinate frame of the 'parent'.
 */
	static Ogre::Vector3 computePDTorque(const Ogre::Quaternion& qRel, const Ogre::Quaternion& qRelD, const Ogre::Vector3& wRel, const Ogre::Vector3& wRelD, double kp, double kd, double strength=1.0 );
	
	/**
	As computePDTorque but returns a force rather than a torque
	 */
	static Ogre::Vector3 computePDForce( const Ogre::Vector3& p, const Ogre::Vector3& pD, const Ogre::Vector3& v, const Ogre::Vector3& vD, double kp, double kd );
};

#endif /* defined(__Loco__ProportionalDerivativeController__) */
