//
//  Segment.cpp
//  Loco
//
//  Created by damian on 05/05/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

// from cartwheel3d

#include "Segment.h"
#include "AnimationSharedStuff.h"

/**
 Constructor
 */
Segment::Segment(const Ogre::Vector3& a_, const Ogre::Vector3& b_){
	this->a = a_;
	this->b = b_;
}

/**
 Destructor
 */
Segment::~Segment(){
	
}


/**
 Copy constructor
 */
Segment::Segment(const Segment& other){
	this->a = other.a;
	this->b = other.b;
}

/**
 Copy operator
 */
Segment& Segment::operator = (const Segment& other){
	this->a = other.a;
	this->b = other.b;
	
	return *this;
}


/**
 This method returns the point on the current segment that is closest to the point c that is passed in as a paremter.
 */
void Segment::getClosestPointTo(const Ogre::Vector3& c, Ogre::Vector3* result) const{
	//we'll return the point d that belongs to the segment, such that: cd . ab = 0
	//BUT, we have to make sure that this point belongs to the segment. Otherwise, we'll return the segment's end point that is closest to D.
	Ogre::Vector3 v = b-a;
	
	double len_squared = v.dotProduct(v);
	//if a==b, it means either of the points can qualify as the closest point
	if (IS_ZERO(len_squared)){
		*result = a;
		return;
	}
	
	double mu = (c-a).dotProduct(v) / len_squared;
	if (mu<0)
		mu = 0;
	if (mu>1)
		mu = 1;
	//the point d is at: a + mu * ab
	*result = a + v*mu;

}

/**
 This method returns the segment that connects the closest pair of points - one on the current segment, and one on the segment that is passed in. The
 'a' point of the resulting segment will lie on the current segment, while the 'b' point lies on the segment that is passed in as a parameter.
 */
void Segment::getShortestSegmentTo(const Segment& other, Segment* result) const{
	//MAIN IDEA: the resulting segment should be perpendicular to both of the original segments. Let point c belong to the current segment, and d belong to
	//the other segment. Then a1b1.cd = 0 and a2b2.cd = 0. From these two equations with two unknowns, we need to get the mu_c and mu_d parameters
	//that will let us compute the points c and d. Of course, we need to make sure that they lie on the segments, or adjust the result if they don't.
	
	//unfortunately, there are quite a few cases we need to take care of. Here it is:
	Ogre::Vector3 tmp1 = other.a - a;
	Ogre::Vector3 tmp2 = b - a;
	Ogre::Vector3 tmp3 = other.b - other.a;
	double A = tmp1.dotProduct(tmp2);
	double B = tmp2.dotProduct(tmp3);
	double C = tmp2.dotProduct(tmp2);
	double D = tmp3.dotProduct(tmp3);
	double E = tmp1.dotProduct(tmp3);
	
	//now a few special cases:
	if (IS_ZERO(C)){
		//current segment has 0 length
		result->a = this->a;
		other.getClosestPointTo(this->a, &result->b);
		return;
	}
	if (IS_ZERO(D)){
		//other segment has 0 length
		this->getClosestPointTo(other.a, &result->a);
		result->b = other.a;
		return;
	}
	
	if (IS_ZERO(C*D - B*B)){
		//this means that the two segments are coplanar and parallel. In this case, there are
		//multiple segments that are perpendicular to the two segments (lines before the truncation really).
		
		//we need to get the projection of the other segment's end point on the current segment
		double mu_a2 = tmp1.dotProduct(tmp2) / (tmp2.dotProduct(tmp2));
		double mu_b2 = (other.b - a).dotProduct(tmp2) / (tmp2.dotProduct(tmp2));
		
		
		//we are now interested in the parts of the segments that are in common between the two input segments
		if (mu_a2<0) mu_a2 = 0;
		if (mu_a2>1) mu_a2 = 1;
		if (mu_b2<0) mu_b2 = 0;
		if (mu_b2>1) mu_b2 = 1;
		
		//closest point on the current segment must lie at the midpoint of mu_a2 and mu_b2
		result->a = a + tmp2*((mu_a2 + mu_b2)/2.0);
		
		other.getClosestPointTo(result->a, &result->b);
		return;
	}
	
	//ok, now we'll find the general solution for two lines in space:
	double mu_c = (A*D - E*B) / (C*D-B*B);
	double mu_d = (mu_c*B - E) / D;
	
	//if the D point or the C point lie outside their respective segments, clamp the values
	if (mu_c<0) mu_c = 0;
	if (mu_c>1) mu_c = 1;
	if (mu_d<0) mu_d = 0;
	if (mu_d>1) mu_d = 1;
	
	result->a = a + tmp2*mu_c;
	result->b = other.a + tmp3*mu_d;

}

