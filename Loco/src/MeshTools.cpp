//
//  MeshTools.cpp
//  bulletTest7
//
//  Created on 02/04/13.
//
//

#include "MeshTools.h"

void MeshTools::getMeshInformation(const Ogre::MeshPtr mesh,
                                   size_t &vertex_count,
                                   Ogre::Vector3* &vertices,
                                   size_t &index_count,
                                   unsigned long* &indices,
                                   const Ogre::Vector3 &position,
                                   const Ogre::Quaternion &orient,
                                   const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;
	
    vertex_count = index_count = 0;
	
    // Calculate how many vertices and indices we're going to need
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }
        // Add the indices
        index_count += submesh->indexData->indexCount;
    }
	
    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned long[index_count];
	
    added_shared = false;
	
    // Run through the submeshes again, adding the data into the arrays
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
		
        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
		
        if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }
			
            const Ogre::VertexElement* posElem =
			vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
			
            Ogre::HardwareVertexBufferSharedPtr vbuf =
			vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());
			
            unsigned char* vertex =
			static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
			
            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            // as second argument. So make it float, to avoid trouble when Ogre::Real will
            // be comiled/typedefed as double:
            //Ogre::Real* pReal;
            float* pReal;
			
            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);
                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }
			
            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }
		
        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;
		
        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
		
        unsigned long* pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);
		
        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;
		
        if ( use32bitindexes )
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
            }
        }
        else
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) +
				static_cast<unsigned long>(offset);
            }
        }
		
        ibuf->unlock();
        current_offset = next_offset;
    }
};


MeshTools::PickerResult MeshTools::pickQuery(Ogre::RaySceneQuery* raySceneQuery)
{
	// TODO: destroy queries using scenemgr::destroyRayQuery or reuse one query object by storing it in a member variable
    //mRaySceneQuery = mSceneMgr->createRayQuery(Ogre::Ray(), queryMask);
	
    //mRaySceneQuery->setRay(ray);
	
    Ogre::RaySceneQueryResult& query_result = raySceneQuery->execute();
	
	
    // at this point we have raycast to a series of different objects bounding boxes.
    // we need to test these different objects to see which is the first polygon hit.
    // there are some minor optimizations (distance based) that mean we wont have to
    // check all of the objects most of the time, but the worst case scenario is that
    // we need to test every triangle of every object.
    Ogre::Real closest_distance = -1.0f;
	PickerResult closest_result_details;
	closest_result_details.pickedObject = NULL;
	
    Ogre::MovableObject *closest_movable;
    for (size_t qr_idx = 0; qr_idx < query_result.size(); qr_idx++)
    {
        // Debug:
        //Ogre::LogManager::getSingletonPtr()->logMessage(query_result[qr_idx].movable->getName());
        //Ogre::LogManager::getSingletonPtr()->logMessage(query_result[qr_idx].movable->getMovableType());
		
		
        // stop checking if we have found a raycast hit that is closer
        // than all remaining entities
        if ((closest_distance >= 0.0f) &&
            (closest_distance < query_result[qr_idx].distance))
        {
            break;
        }
		
        // only check this result if its a hit against an entity
        if ((query_result[qr_idx].movable != NULL) &&
            ((query_result[qr_idx].movable->getMovableType().compare("Entity") == 0)
             ||query_result[qr_idx].movable->getMovableType().compare("ManualObject") == 0))
        {
            // mesh data to retrieve
            size_t vertex_count;
            size_t index_count;
            Ogre::Vector3 *vertices;
            unsigned long *indices;
			
            // get the mesh information
            if(query_result[qr_idx].movable->getMovableType().compare("Entity") == 0)
			{
                // For entities
                // get the entity to check
                Ogre::Entity *pentity = static_cast<Ogre::Entity*>(query_result[qr_idx].movable);
				
                getMeshInformation(pentity->getMesh(), vertex_count, vertices, index_count, indices,
                                              pentity->getParentNode()->_getDerivedPosition(),
                                              pentity->getParentNode()->_getDerivedOrientation(),
                                              pentity->getParentNode()->_getDerivedScale());
            }
			else
			{
				assert(false && "manual objects not implemented");
				/*
                // For manualObjects
                // get the entity to check
                Ogre::ManualObject *pmanual = static_cast<Ogre::ManualObject*>(query_result[qr_idx].movable);
				
                InputGeom::getManualMeshInformation(pmanual, vertex_count, vertices, index_count, indices,
                                                    pmanual->getParentNode()->_getDerivedPosition(),
                                                    pmanual->getParentNode()->_getDerivedOrientation(),
                                                    pmanual->getParentNode()->_getDerivedScale());*/
            }
			
            // test for hitting individual triangles on the mesh
            bool new_closest_found = false;
            for (int i = 0; i < static_cast<int>(index_count); i += 3)
            {
                // check for a hit against this triangle
                std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(raySceneQuery->getRay(), vertices[indices[i]],
                                                                         vertices[indices[i+1]], vertices[indices[i+2]], true, false);
				
                // if it was a hit check if its the closest
                if (hit.first)
                {
                    if ((closest_distance < 0.0f) ||
                        (hit.second < closest_distance))
                    {
                        // this is the closest so far, save it off
                        closest_distance = hit.second;
                        new_closest_found = true;
						closest_result_details.pickedObject = query_result[qr_idx].movable;
						closest_result_details.pickedTriangleIndex = i/3;
						
						Ogre::Vector3 rayPos = raySceneQuery->getRay().getPoint(closest_distance);
						float distance0 = vertices[indices[i]].squaredDistance(rayPos);
						float distance1 = vertices[indices[i+1]].squaredDistance(rayPos);
						float distance2 = vertices[indices[i+2]].squaredDistance(rayPos);
						if (distance0<=distance1 && distance0<=distance2) {
							// vertex 0 is closest
							closest_result_details.pickedVertexDistance = sqrtf(distance0);
							closest_result_details.pickedVertexIndex = indices[i];
						} else if ( distance1<=distance2 && distance1<=distance0 ) {
							// vertex 1 is closest
							closest_result_details.pickedVertexDistance = sqrtf(distance1);
							closest_result_details.pickedVertexIndex = indices[i+1];
						} else /*if ( distance2<=distance0 && distance2<=distance1 )*/ {
							// vertex 2 is closest
							closest_result_details.pickedVertexDistance = sqrtf(distance2);
							closest_result_details.pickedVertexIndex = indices[i+2];
						}
							
                    }
                }
            }
			
            // free the verticies and indicies memory
            delete[] vertices;
            delete[] indices;
			
            // if we found a new closest raycast for this object, update the
            // closest_result before moving on to the next object.
            if (new_closest_found)
            {
                closest_result_details.pickPoint = raySceneQuery->getRay().getPoint(closest_distance);
                if(query_result[qr_idx].movable != NULL)
                    closest_movable = query_result[qr_idx].movable;
            }
        }
    }
	
    // return the result
	return closest_result_details;
}