//
//  Segment.h
//  Loco
//
//  Created by damian on 05/05/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#ifndef __Loco__Segment__
#define __Loco__Segment__

#include <iostream>
#include <Ogre/OgreVector3.h>

// from cartwheel-3d

/*==========================================================================================================================================*
 *	This class provides the implementation of a segment, and a few useful method associated with them.                                      *
 *==========================================================================================================================================*/

class Segment{
public:
	//the end points of the segment
	Ogre::Vector3 a, b;
	
	Segment(const Ogre::Vector3& a_, const Ogre::Vector3& b_);
	
	Segment(){
		a = Ogre::Vector3(0,0,0);
		b = Ogre::Vector3(0,0,0);
	}
	
	/**
	 Copy constructor
	 */
	Segment(const Segment& other);
	
	/**
	 Copy operator
	 */
	Segment& operator = (const Segment& other);
	
	
	~Segment();
	
	/**
	 This method returns the point on the current segment that is closest to the point c that is passed in as a paremter.
	 */
	void getClosestPointTo(const Ogre::Vector3& c, Ogre::Vector3* result) const;
	
	/**
	 This method returns the segment that connects the closest pair of points - one on the current segment, and one on the segment that is passed in. The
	 'a' point of the resulting segment will lie on the current segment, while the 'b' point lies on the segment that is passed in as a parameter.
	 */
	void getShortestSegmentTo(const Segment& other, Segment* result) const;

	
};




#endif /* defined(__Loco__Segment__) */
