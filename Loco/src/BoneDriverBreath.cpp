//
//  BoneDriverBreath.cpp
//  Loco
//
//  Created on 27/10/13.
//
//

#include "BoneDriverBreath.h"
#include <Ogre/OgreBone.h>


BoneDriverBreath::BoneDriverBreath( Ogre::SharedPtr<DriveableBone> breathBone, Ogre::SharedPtr<DriveableBone> upperSpineBone, const Ogre::Vector3& forwardAxis, const Ogre::Vector3& rightAxis )
: mBreathBone(breathBone), mUpperSpineBone(upperSpineBone), mPhase(0.0f)
{
	Ogre::Quaternion invBreath = mBreathBone->getBone()->getInitialOrientation().Inverse();
	mForward = invBreath*-forwardAxis;
	mRight = invBreath*-rightAxis;
}

float calculateAlpha( float phase, float lopsidedness )
{
	phase = fmodf(phase,1.0f);
	float alpha = 0.5f+0.5f*cosf(powf(phase,lopsidedness)*2.0f*M_PI);
	return alpha;
}

void BoneDriverBreath::update(float dt)
{
	// normal breath rate = 12-20 breaths per minute = (12/60 - 20/60 Hz)
	//
	float speed = (16.0f/60.0f);
	mPhase += speed * dt;
	
	//
	float lopsidedness = 1.5f; // 1.0f=breathe in and out are equal, higher = breathe in is faster
	float alpha = calculateAlpha(mPhase, lopsidedness);
	//float alpha = 0.5f+0.5f*sinf(mPhase*M_PI*2.0);
	
	// do the scale
	// we scale outwards only
	float maxForwardScale = 1.1f;
	float maxRightScale = 1.05f;
	
	// not sure if this is correct
	Ogre::Vector3 forwardScaleAdd = (maxForwardScale-1.0f)*alpha*mForward;
	Ogre::Vector3 rightScaleAdd = (maxRightScale-1.0f)*alpha*mRight;
	Ogre::Vector3 scaleVector = Ogre::Vector3(1,1,1)+forwardScaleAdd+rightScaleAdd;
	mBreathBone->getBone()->setScale(scaleVector);
	
	// rotate the upper spine a few degrees about x
	// spine rot is ~-1/6 out of phase with scale
	float spineRotAlpha = calculateAlpha(mPhase + (1.0f-0.13f), 1.2f);
	Ogre::Quaternion spineRot( Ogre::Degree(2.0f*(-0.7f+spineRotAlpha)), -Ogre::Vector3::UNIT_X );
	mUpperSpineBone->addRelativeRotation(spineRot);
	
}

