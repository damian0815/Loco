//
//  Spline1D.h
//  Loco
//
//  Created by damian on 05/05/14.
//  Copyright (c) 2014 bg. All rights reserved.
//

#ifndef __Loco__Spline1D__
#define __Loco__Spline1D__

#include <iostream>
#include <vector>
#include <float.h>

class Spline1D
{
public:
	
	bool empty() { return mSpline.empty(); }

	void addKnot( float t, float theta ) { mSpline.push_back(std::make_pair(t,theta)); };
	void alterKnot( unsigned int whichKnot, float t, float theta ) { mSpline.at(whichKnot).first = t; mSpline.at(whichKnot).second = theta; }

	
	inline float evaluateCatmullRom( float phi, unsigned int &lastPos ) const;
	inline float evaluateCatmullRom( float phi ) const;
	inline float evaluateLinear( float phi, unsigned int &lastPos ) const;
	inline float evaluateLinear( float phi ) const;
	
private:
	std::vector< std::pair<float,float> > mSpline;
	
	
};


/**
 This method returns the index of the first knot whose value is larger than the parameter value t. If no such index exists (t is larger than any
 of the values stored), then values.size() is returned.
 
 from cartwheel-3d
 
 */
static inline int getFirstLargerIndex(double t, const std::vector< std::pair<float,float> >& spline, unsigned int &lastIndex )
{
	int size = spline.size();
	if( size == 0 )
		return 0;
	if( t < spline[(lastIndex+size-1)%size].first )
		lastIndex = 0;
	for (int i = 0; i<size;i++){
		int index = (i + lastIndex) % size;
		if (t < spline[index].first ) {
			lastIndex = index;
			return index;
		}
	}
	return size;
}

/**
 This method interprets the trajectory as a Catmul-Rom spline, and evaluates it at the point t
 
 from cartwheel-3d
 */


float Spline1D::evaluateCatmullRom(  float phi, unsigned int &lastPos ) const
{
	const std::vector< std::pair<float,float> >& spline = mSpline;
	float t = phi;
	
	int size = spline.size();
	if( size == 0 ) return 0.0f;
	if (t<=spline[0].first) return spline[0].second;
	if (t>=spline[size-1].first) return spline[size-1].second;
	int index = getFirstLargerIndex(t, spline, lastPos);
	
	//now that we found the interval, get a value that indicates how far we are along it
	t = (t-spline[index-1].first) / (spline[index].first-spline[index-1].first);
	
	//approximate the derivatives at the two ends
	float t0, t1, t2, t3;
	float p0, p1, p2, p3;
	p0 = (index-2<0)?(spline[index-1].second):(spline[index-2].second);
	p1 = spline[index-1].second;
	p2 = spline[index].second;
	p3 = (index+1>=size)?(spline[index].second):(spline[index+1].second);
	
	t0 = (index-2<0)?(spline[index-1].first):(spline[index-2].first);
	t1 = spline[index-1].first;
	t2 = spline[index].first;
	t3 = (index+1>=size)?(spline[index].first):(spline[index+1].first);
	
	float d1 = (t2-t0);
	float d2 = (t3-t1);
	
#define TINY FLT_EPSILON
	if (d1 > -TINY && d1  < 0) d1 = -TINY;
	if (d1 < TINY && d1  >= 0) d1 = TINY;
	if (d2 > -TINY && d2  < 0) d2 = -TINY;
	if (d2 < TINY && d2  >= 0) d2 = TINY;
	
#ifdef FANCY_SPLINES
	float m1 = (p2 - p0) * (1-(t1-t0)/d1);
	float m2 = (p3 - p1) * (1-(t3-t2)/d2);
#else
	float m1 = (p2 - p0)*0.5;
	float m2 = (p3 - p1)*0.5;
#endif
	
	t2 = t*t;
	t3 = t2*t;
	
	//and now perform the interpolation using the four hermite basis functions from wikipedia
	return p1*(2*t3-3*t2+1) + m1*(t3-2*t2+t) + p2*(-2*t3+3*t2) + m2 * (t3 - t2);
}

float Spline1D::evaluateCatmullRom(float phi) const
{
	unsigned int dummy =0;
	return evaluateCatmullRom(phi, dummy);
}

/**
 This method performs linear interpolation to evaluate the trajectory at the point t
 
 from cartwheel-3d
 */
float Spline1D::evaluateLinear( float t, unsigned int &lastPos ) const
{
	int size = mSpline.size();
	if ( size==0 ) return 0.0f;
	// flat extend -
	if ( t<=mSpline[0].first ) return mSpline[0].second;
	// flat extend +
	if ( t>=mSpline[size-1].first ) return mSpline[size-1].second;
	// go
	int index = getFirstLargerIndex(t, mSpline, lastPos);
		
	//now linearly interpolate between index-1 and index
	t = (t-mSpline[index-1].first) / (mSpline[index].first-mSpline[index-1].first);
	return (mSpline[index-1].second) * (1-t) + (mSpline[index].second) * t;
}

float Spline1D::evaluateLinear( float t ) const
{
	unsigned int dummy=0;
	return evaluateLinear( t, dummy );
}

#endif /* defined(__Loco__Spline1D__) */
