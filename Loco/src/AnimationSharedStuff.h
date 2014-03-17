//
//  AnimationSharedStuff.h
//  Loco
//
//  Copyright (c) 2013 bg. All rights reserved.
//

#ifndef Loco_AnimationSharedStuff_h
#define Loco_AnimationSharedStuff_h

#import <Ogre/OgreQuaternion.h>

typedef enum _BoneAxis
{
	BoneAxisX,
	BoneAxisY,
	BoneAxisZ // longitudinal axis
} BoneAxis;

//
//double getScalarPart( const Ogre::Quaternion& q );
//Ogre::Vector3 getVectorPart( const Ogre::Quaternion& q );
///*! @brief Make a quaternion from the euler angles in radians. */
//Ogre::Quaternion makeEulerQuaternion( const Ogre::Vector3& eulerAngles );
///*! @brief Make a Vector3 by extracting euler angles in radians. */
//Ogre::Vector3 makeEulerVector( const Ogre::Quaternion& rotation );
///*! @brief Return the euler result of rotating q by euler, everything in radians. */
//Ogre::Vector3 rotateEuler( const Ogre::Quaternion& q, const Ogre::Vector3& euler );
//
/**
	This macro checks to see if the value of x is zero within epsilon: -epsilon<x<epsilon
*/
#define ZERO_WITHIN_EPSILON(x) ((x)>-FLT_EPSILON && (x)<FLT_EPSILON)


/**
	This macro checks to see if the value of x is less than y, within epsilon
*/
#define LESS_THAN_WITHIN_EPSILON(x,y) ((x)<(y)+FLT_EPSILON)


/**
	This macro checks to see if the value of x is greater than y, within epsilon
*/
#define GREATER_THAN_WITHIN_EPSILON(x,y) ((x)>(y)-FLT_EPSILON)


/**
	This macro checks to see if the values of x and y are equal within epsilon
*/
#define EQUAL_TO_WITHIN_EPSILON(x,y) ZERO_WITHIN_EPSILON((x)-(y))

/**
	and some shortcuts to the macros above
*/
#define IS_ZERO(x)			ZERO_WITHIN_EPSILON(x)
#define LESS_THAN(x,y)		LESS_THAN_WITHIN_EPSILON(x,y)
#define GREATER_THAN(x,y)	GREATER_THAN_WITHIN_EPSILON(x,y)
#define EQUAL_TO(x,y)		EQUAL_TO_WITHIN_EPSILON(x,y)

/**
	Computes the value of x in radians
*/
#define RAD(x) (((x) * PI)/180.0)

/**
	And this computes the value of x in degrees
*/
#define DEG(x) ((x * 180)/PI)

/**
	macros for max and mins
*/
#ifndef MAX
#define MAX(x, y) (((x)<(y))?(y):(x))
#endif
#ifndef MIN
#define MIN(x, y) (((x)>(y))?(y):(x))
#endif

#define SQR(x) ((x)*(x))

#define SGN(x) (((x)<0)?(-1):(1))
#endif


