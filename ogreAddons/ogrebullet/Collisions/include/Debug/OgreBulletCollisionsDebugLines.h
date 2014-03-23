/***************************************************************************

This source file is part of OGREBULLET
(Object-oriented Graphics Rendering Engine Bullet Wrapper)
For the latest info, see http://www.ogre3d.org/phpBB2addons/viewforum.php?f=10

Copyright (c) 2007 tuan.kuranes@gmail.com (Use it Freely, even Statically, but have to contribute any changes)



Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/
#ifndef _OgreBulletCollisions_DEBUGLines_H_
#define _OgreBulletCollisions_DEBUGLines_H_

#include "OgreBulletCollisionsPreRequisites.h"


namespace OgreBulletCollisions
{
    //------------------------------------------------------------------------------------------------
    class  DebugLines : public Ogre::SimpleRenderable
    {
    public:
        DebugLines(void);
        ~DebugLines(void);

        void addLine (const Ogre::Vector3 &start, const Ogre::Vector3 &end)
        {
			// DOES NOT CONVERT TO LOCAL COORDINATES
            _points.push_back (start);
            _points.push_back (end);
			// default white
			_colours.push_back( Ogre::ColourValue( 1, 1, 1, 1 ) );
			_colours.push_back( Ogre::ColourValue( 1, 1, 1, 1 ) );
        }
		
		void addLine (const Ogre::Vector3 &startW, const Ogre::Vector3 &endW, const Ogre::ColourValue& colour )
		{
			Ogre::Vector3 start = convertWorldToLocalPosition(startW);
			Ogre::Vector3 end = convertWorldToLocalPosition(endW);
			
            _points.push_back (start);
            _points.push_back (end);
			_colours.push_back( colour );
			_colours.push_back( colour );
		}

        void addLine(Ogre::Real start_x, Ogre::Real start_y, Ogre::Real start_z, 
            Ogre::Real end_x, Ogre::Real end_y, Ogre::Real end_z)
        {
            addLine (Ogre::Vector3(start_x,start_y,start_z),
                Ogre::Vector3(end_x,end_y,end_z));
        }
		void addCross( const Ogre::Vector3& centerW, float size, const Ogre::ColourValue& colour )
		{
			//Ogre::Vector3 center = convertWorldToLocalPosition(centerW);
			Ogre::Vector3 up(0,1,0), right(1,0,0), forward(0,0,1);
			addLine( centerW+up*size*0.5f, centerW-up*size*0.5f, colour );
			addLine( centerW+forward*size*0.5f, centerW-forward*size*0.5f, colour );
			addLine( centerW+right*size*0.5f, centerW-right*size*0.5f, colour );
		}
		void addAxes( const Ogre::Vector3& centerW, const Ogre::Quaternion& orientationW, float size )
		{
			Ogre::Vector3 up, right, forward;
			orientationW.ToAxes( right, up, forward );
			addLine( centerW+right*size, centerW, Ogre::ColourValue::Red );
			addLine( centerW+up*size, centerW, Ogre::ColourValue::Green );
			addLine( centerW+forward*size, centerW, Ogre::ColourValue::Blue );
		}
		void addTorque( const Ogre::Vector3& centerW, const Ogre::Vector3& torqueW, float scale )
		{
			Ogre::Vector3 up(0,1,0), right(1,0,0), forward(0,0,1);
			addLine( centerW+right*torqueW.x*scale, centerW, Ogre::ColourValue::Red );
			addLine( centerW+up*torqueW.y*scale, centerW, Ogre::ColourValue::Green );
			addLine( centerW+forward*torqueW.z*scale, centerW, Ogre::ColourValue::Blue );
			
		}



		void addPoint( const Ogre::Vector3 &ptW, const Ogre::ColourValue& colour ) {
            clear();
			Ogre::Vector3 pt = convertWorldToLocalPosition(ptW);
			
            _points.push_back(pt);
			_colours.push_back( colour );
		}
        void addPoint (const Ogre::Vector3 &pt)
        {
			addPoint(pt, Ogre::ColourValue( 1, 1, 1, 1 ) );
        }

        void addPoint (Ogre::Real x, Ogre::Real y, Ogre::Real z)
        {
            addPoint (Ogre::Vector3(x, y, z));
        }
		
		Ogre::Vector3 convertWorldToLocalPosition(Ogre::Vector3 worldPos)
		{
			return getParentNode()->convertWorldToLocalPosition(worldPos);
		}
		Ogre::Quaternion convertWorldToLocalOrientation(Ogre::Quaternion worldOri)
		{
			return getParentNode()->convertWorldToLocalOrientation(worldOri);
		}


        void draw ();
        void clear ();

        Ogre::Real getSquaredViewDepth (const Ogre::Camera *cam) const;
        Ogre::Real getBoundingRadius (void) const;

    protected:

        Vector3Array _points;
		std::vector<Ogre::ColourValue> _colours;
        bool _drawn;
		bool mHasColours;
		
        static bool _materials_created;
    };
}
#endif //_OgreBulletCollisions_DEBUGLines_H_

