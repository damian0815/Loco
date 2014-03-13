//
//  Picker.h
//  bulletTest7
//
//  Created on 02/04/13.
//
//

#ifndef __bulletTest7__Picker__
#define __bulletTest7__Picker__

#include <iostream>

#include <OGRE/Ogre.h>

class MeshTools
{
public:
	
	
	typedef struct _PickerResult {
		// the object that was picked
		Ogre::MovableObject *pickedObject;
		// the point that the raycast thinks is the pick point
		Ogre::Vector3 pickPoint;
		
		// index of intersected triangle
		int pickedTriangleIndex;
		// index of nearest vertex on intersected triangle
		int pickedVertexIndex;
		// closest distance from the ray to the picked vertex
		float pickedVertexDistance;
	} PickerResult;
	PickerResult pickQuery(Ogre::RaySceneQuery* raySceneQuery);
	
	static void getMeshInformation(const Ogre::MeshPtr mesh,
                                   size_t &vertex_count,
                                   Ogre::Vector3* &vertices,
                                   size_t &index_count,
                                   unsigned long* &indices,
                                   const Ogre::Vector3 &position,
                                   const Ogre::Quaternion &orient,
								   const Ogre::Vector3 &scale);
protected:
	
	

};


#endif /* defined(__bulletTest7__Picker__) */
