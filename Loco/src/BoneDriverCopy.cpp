//
//  BoneDriverCopy.cpp
//  Loco
//
//  Created on 27/10/13.
//
//

#include "BoneDriverCopy.h"
#include <Ogre/OgreBone.h>
#include "OgreBulletCollisionsDebugLines.h"

BoneDriverCopy::BoneDriverCopy( Ogre::Bone* sourceBone, Ogre::Bone* targetBone, float orientationCopyPct )
: mSource(sourceBone), mTarget(targetBone), mOrientationCopyPct(orientationCopyPct)
{
	mTarget->setOrientation(mSource->getInitialOrientation());
	mTarget->setBindingPose();
}

void BoneDriverCopy::update()
{
	Ogre::Quaternion relQ = mSource->getInitialOrientation().Inverse() * mSource->getOrientation() ;
	
	// multiply by pct
	float pct = mOrientationCopyPct;
	
	Ogre::Quaternion partial = Ogre::Quaternion::Slerp( pct, Ogre::Quaternion::IDENTITY, relQ, true /* shortest path */ );
	mTarget->setOrientation( mTarget->getInitialOrientation()*partial );
	
}

void BoneDriverCopy::debugDraw( OgreBulletCollisions::DebugLines* debugLines, const Ogre::ColourValue& boneColour, float axisLength )
{
	Ogre::Vector3 offset(0,0,0);
	auto bonePos = mTarget->_getDerivedPosition() + offset;
	// draw line to parent
	auto parent = mTarget->getParent();
	if ( parent ) {
		auto parentPos = mTarget->getParent()->_getDerivedPosition() + offset;
		debugLines->addLine( bonePos, parentPos, boneColour );
	}
	if ( axisLength>0.0f ) {
		// draw axes
		auto boneOrientation = mTarget->_getDerivedOrientation();
		float distance = axisLength;
		Ogre::Vector3 xAxis, yAxis, zAxis;
		boneOrientation.ToAxes(xAxis, yAxis, zAxis);
		debugLines->addLine(bonePos, bonePos+xAxis*distance, Ogre::ColourValue::Red);
		debugLines->addLine(bonePos, bonePos+yAxis*distance, Ogre::ColourValue::Green);
		debugLines->addLine(bonePos, bonePos+zAxis*distance, Ogre::ColourValue::Blue);
	}
}
