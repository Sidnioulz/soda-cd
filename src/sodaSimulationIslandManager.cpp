/*
 *** Some methods of these classes are derived from Bullet Physics:
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2011 Erwin Coumans  http://bulletphysics.org
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *** The code that differs from the original source is licensed as is:
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include "LinearMath/btScalar.h"
#include "sodaSimulationIslandManager.h"
#include "sodaDynamicsWorld.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

#include "LinearMath/btQuickprof.h"

sodaSimulationIslandManager::sodaSimulationIslandManager(sodaDynamicsWorld *sodaWorld) :
    m_splitIslands(true),
    sodaWorld(sodaWorld)
{
    foreignCache = new sodaSimulationIslandForeignerCache();
}

sodaSimulationIslandManager::~sodaSimulationIslandManager()
{
    delete foreignCache;
}


void sodaSimulationIslandManager::initUnionFind(int n)
{
    m_unionFind.reset(n);
}


void sodaSimulationIslandManager::findUnions(btDispatcher */*dispatcher*/, sodaDynamicsWorld *colWorld)
{
    btOverlappingPairCache *pairCachePtr = colWorld->getPairCache();
    const int numOverlappingPairs = pairCachePtr->getNumOverlappingPairs();
    if (numOverlappingPairs)
    {
        btBroadphasePair *pairPtr = pairCachePtr->getOverlappingPairArrayPtr();

        for (int i=0;i<numOverlappingPairs;i++)
        {
            const btBroadphasePair& collisionPair = pairPtr[i];
            btCollisionObject *colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
            btCollisionObject *colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

            if (((colObj0) && ((colObj0)->mergesSimulationIslands())) &&
                ((colObj1) && ((colObj1)->mergesSimulationIslands())))
            {

                m_unionFind.unite((colObj0)->getIslandTag(),
                    (colObj1)->getIslandTag());
            }
        }
    }


    {
        const btAlignedObjectArray<sodaPersistentForeignerManifold *> &manifolds = sodaWorld->getContactResultCallback().getManifolds();
        int manifoldSize = manifolds.size();
        const sodaPersistentForeignerManifold *foreignManifold = 0;

        for(int i=0; i<manifoldSize; ++i)
        {
            foreignManifold = manifolds[i];

            // Use foreign manifolds to find more unions
            const btCollisionObject *local = (const btCollisionObject*) foreignManifold->getLocalBody();
            const btCollisionObject *foreigner = (const btCollisionObject*) foreignManifold->getForeignBody();

            // If local and foreigner were identified and are suitable for island merge
            if (local && local->mergesSimulationIslands() && foreigner && foreigner->mergesSimulationIslands())
            {
#ifndef NDEBUG
                sodaEntity *obEnt = static_cast<sodaEntity *>(local->getUserPointer());
                sodaDynamicEntity *obEW; CellBorderEntity *cbe;
                if((obEW = dynamic_cast<sodaDynamicEntity *>(obEnt)) != 0)
                    qDebug() << "Local is " << obEW->getDisplayName() << "owned by" << obEW->getOwnerId() << "and we are in " << sodaWorld->getWorldId();
                else if((cbe = dynamic_cast<CellBorderEntity *>(obEnt)) != 0)
                    qDebug() << "Local border is " << cbe->getDisplayName();
                obEnt = static_cast<sodaEntity *>(foreigner->getUserPointer());
                if((obEW = dynamic_cast<sodaDynamicEntity *>(obEnt)) != 0)
                    qDebug() << "foreigner is " << obEW->getDisplayName() << "owned by" << obEW->getOwnerId() << "and we are in " << sodaWorld->getWorldId();
                else if((cbe = dynamic_cast<CellBorderEntity *>(obEnt)) != 0)
                    qDebug() << "foreigner border is " << cbe->getDisplayName();
#endif

                m_unionFind.unite(local->getIslandTag(), foreignCache->getIslandTag(foreigner));
            }
        }
    }
}

#ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION
void   sodaSimulationIslandManager::updateActivationState(sodaDynamicsWorld *colWorld, btDispatcher *dispatcher)
{
    // Put the index into m_controllers into m_tag
    int index = 0;

    // Island tags for local bodies
    {
#ifndef NDEBUG
        qDebug() << "sodaSimulationIslandManager(" << sodaWorld->getWorldId() << ")::updateActivationState(); " << colWorld->getCollisionObjectArray().size() << "(" << sodaWorld->getLogicWorld()->getEntities().size() << "+" << sodaWorld->getLogicWorld()->getStaticEntities().size() << "+" << sodaWorld->getLogicWorld()->getLocalGrid()->getNbBorders() << ") objects to update; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

        for(int i=0; i<colWorld->getCollisionObjectArray().size(); i++)
        {
            btCollisionObject *collisionObject= colWorld->getCollisionObjectArray()[i];

            // Normal case of self-managed btCollisionObjects
            if(sodaWorld->ownsCollisionObject(collisionObject))
            {
                //Adding filtering here
                if (!collisionObject->isStaticOrKinematicObject())
                    collisionObject->setIslandTag(index++);

                collisionObject->setCompanionId(-1);
                collisionObject->setHitFraction(btScalar(1.));
            }
            // Pointers to foreign objects within a dynamics world
            else
            {
#ifndef NDEBUG
                const sodaDynamicEntity *foreignerEnt = dynamic_cast<const sodaDynamicEntity*>(static_cast<const sodaEntity *>(collisionObject->getUserPointer()));
                qDebug() << "sodaSimulationIslandManager(" << sodaWorld->getWorldId() << ")::updateActivationState(); " << foreignerEnt->getDisplayName() << "has been added to the foreign cache with island tag" << index << "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
                if(!collisionObject->isStaticOrKinematicObject())
                    foreignCache->setIslandTag(collisionObject, index++);
                else
                    foreignCache->setIslandTag(collisionObject, -1);

                foreignCache->setCompanionId(collisionObject, -1);
                foreignCache->setHitFraction(collisionObject, btScalar(1.));
            }
        }
    }

//    // Island tags for foreign bodies in the foreignCache
//    {
//        const btAlignedObjectArray<sodaPersistentForeignerManifold *> &manifolds = sodaWorld->getContactResultCallback().getManifolds();
//        int manifoldSize = manifolds.size();
//        const sodaPersistentForeignerManifold *foreignManifold=0;

//        for(int i=0; i<manifoldSize; ++i)
//        {
//            foreignManifold = manifolds[i];

//            // Use foreign manifolds to find more unions
//            const btCollisionObject *foreigner = (const btCollisionObject*) foreignManifold->getForeignBody();

//            if(foreigner)
//            {
//#ifndef NDEBUG
//                const sodaDynamicEntity *foreignerEnt = dynamic_cast<const sodaDynamicEntity*>(static_cast<const sodaEntity *>(foreigner->getUserPointer()));
//                qDebug() << "sodaSimulationIslandManager(" << sodaWorld->getWorldId() << ")::updateActivationState(); " << foreignerEnt->getDisplayName() << "has been added to the foreign cache with island tag" << index << "; Thread " << QString().sprintf("%p", QThread::currentThread());
//#endif
//                if(!foreigner->isStaticOrKinematicObject())
//                    foreignCache->setIslandTag(foreigner, index++);

//                foreignCache->setCompanionId(foreigner, -1);
//                foreignCache->setHitFraction(foreigner, btScalar(1.));
//            }
//        }
//    }

    // do the union find
    initUnionFind(index);
    findUnions(dispatcher,colWorld);
}

void   sodaSimulationIslandManager::storeIslandActivationState(sodaDynamicsWorld *colWorld)
{
    // put the islandId ('find' value) into m_tag
    int index = 0;
    int i;
    for (i=0;i<colWorld->getCollisionObjectArray().size();i++)
    {
        btCollisionObject *collisionObject= colWorld->getCollisionObjectArray()[i];

        if(sodaWorld->ownsCollisionObject(collisionObject))
        // Normal case of self-managed btCollisionObjects
        {
            if (!collisionObject->isStaticOrKinematicObject())
            {
                collisionObject->setIslandTag( m_unionFind.find(index) );
                //Set the correct object offset in Collision Object Array
                m_unionFind.getElement(index).m_sz = i;
                collisionObject->setCompanionId(-1);
                index++;
            } else
            {
                collisionObject->setIslandTag(-1);
                collisionObject->setCompanionId(-2);
            }
        }
        // Pointers to foreign objects within a dynamics world
        else
        {
            if (!collisionObject->isStaticOrKinematicObject())
            {
                foreignCache->setIslandTag(collisionObject, m_unionFind.find(index));
                //Set the correct object offset in Collision Object Array
                m_unionFind.getElement(index).m_sz = i;
                foreignCache->setCompanionId(collisionObject, -1);
                index++;
            } else
            {
                foreignCache->setIslandTag(collisionObject, -1);
                foreignCache->setCompanionId(collisionObject, -2);
            }
        }
    }

//    // Store foreigners' state in the cache
//    {
//        const btAlignedObjectArray<sodaPersistentForeignerManifold *> &manifolds = sodaWorld->getContactResultCallback().getManifolds();
//        int manifoldSize = manifolds.size();
//        const sodaPersistentForeignerManifold *foreignManifold=0;

//        for(int i=0; i<manifoldSize; ++i)
//        {
//            foreignManifold = manifolds[i];

//            // Use foreign manifolds to find more unions
//            const btCollisionObject *foreigner = (const btCollisionObject*) foreignManifold->getForeignBody();

//            if(foreigner)
//            {
//                if (!foreigner->isStaticOrKinematicObject())
//                {
//                    foreignCache->setIslandTag(foreigner, m_unionFind.find(index));
//                    //Set the correct object offset in Collision Object Array
//                    m_unionFind.getElement(index).m_sz = i;
//                    foreignCache->setCompanionId(foreigner, -1);
//                    index++;
//                } else
//                {
//                    foreignCache->setIslandTag(foreigner, -1);
//                    foreignCache->setCompanionId(foreigner, -2);
//                }
//            }
//        }
//    }
}


#else //STATIC_SIMULATION_ISLAND_OPTIMIZATION
void	sodaSimulationIslandManager::updateActivationState(sodaDynamicsWorld *colWorld,btDispatcher *dispatcher)
{

    initUnionFind( int (colWorld->getCollisionObjectArray().size()));

    // put the index into m_controllers into m_tag
    int index = 0;
    int i;
    for(i=0; i<colWorld->getCollisionObjectArray().size(); i++)
    {
        btCollisionObject *collisionObject= colWorld->getCollisionObjectArray()[i];

        // Normal case of self-managed btCollisionObjects
        if(sodaWorld->ownsCollisionObject(collisionObject))
        {
            collisionObject->setIslandTag(index++);
            collisionObject->setCompanionId(-1);
            collisionObject->setHitFraction(btScalar(1.));
        }
        // Pointers to foreign objects within a dynamics world
        else
        {
            foreignCache->setIslandTag(collisionObject, index++);
            foreignCache->setCompanionId(collisionObject, -1);
            foreignCache->setHitFraction(collisionObject, btScalar(1.));
        }
    }

    // do the union find
    findUnions(dispatcher,colWorld);
}

void	sodaSimulationIslandManager::storeIslandActivationState(sodaDynamicsWorld *colWorld)
{
    // put the islandId ('find' value) into m_tag
    int index = 0;
    int i;
    for (i=0;i<colWorld->getCollisionObjectArray().size();i++)
    {
        btCollisionObject *collisionObject= colWorld->getCollisionObjectArray()[i];

        // Normal case of self-managed btCollisionObjects
        if(sodaWorld->ownsCollisionObject(collisionObject))
        {
            if (!collisionObject->isStaticOrKinematicObject())
            {
                collisionObject->setIslandTag( m_unionFind.find(index) );
                collisionObject->setCompanionId(-1);
            } else
            {
                collisionObject->setIslandTag(-1);
                collisionObject->setCompanionId(-2);
            }
            index++;
        }
        // Pointers to foreign objects within a dynamics world
        else
        {
            if (!collisionObject->isStaticOrKinematicObject())
            {
                foreignCache->setIslandTag(collisionObject, m_unionFind.find(index));
                //Set the correct object offset in Collision Object Array
                m_unionFind.getElement(index).m_sz = i;
                foreignCache->setCompanionId(collisionObject, -1);
            } else
            {
                foreignCache->setIslandTag(collisionObject, -1);
                foreignCache->setCompanionId(collisionObject, -2);
            }
            index++;
        }
    }
}

#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION

int sodaSimulationIslandManager::getIslandId(const btPersistentManifold *lhs)
{
    int islandId;
    const btCollisionObject *rcolObj0 = static_cast<const btCollisionObject*>(lhs->getBody0());
    const btCollisionObject *rcolObj1 = static_cast<const btCollisionObject*>(lhs->getBody1());

    if(sodaWorld->ownsCollisionObject(rcolObj0))
        islandId = rcolObj0->getIslandTag();
    else
        islandId = foreignCache->getIslandTag(rcolObj0);

    if(islandId < 0)
    {
        if(sodaWorld->ownsCollisionObject(rcolObj1))
            islandId = rcolObj1->getIslandTag();
        else
            islandId = foreignCache->getIslandTag(rcolObj1);
    }

    return islandId;
}

void sodaSimulationIslandManager::buildIslands(btDispatcher *dispatcher, sodaDynamicsWorld *collisionWorld)
{
    BT_PROFILE("islandUnionFindAndQuickSort");

    btCollisionObjectArray& collisionObjects = collisionWorld->getCollisionObjectArray();

    m_islandmanifold.resize(0);

    //we are going to sort the unionfind array, and store the element id in the size
    //afterwards, we clean unionfind, to make sure no-one uses it anymore

    getUnionFind().sortIslands();
    int numElem = getUnionFind().getNumElements();

    int endIslandIndex=1;
    int startIslandIndex;


    //update the sleeping state for bodies, if all are sleeping
    for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
    {
        int islandId = getUnionFind().getElement(startIslandIndex).m_id;
        for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (getUnionFind().getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
        {
        }

        //int numSleeping = 0;

        bool allSleeping = true;

        int idx;
        for (idx=startIslandIndex;idx<endIslandIndex;idx++)
        {
            int i = getUnionFind().getElement(idx).m_sz;

            btCollisionObject *colObj0 = collisionObjects[i];
            int objIslandTag = -2;
            if(sodaWorld->ownsCollisionObject(colObj0))
                objIslandTag = colObj0->getIslandTag();
            else
                objIslandTag = foreignCache->getIslandTag(colObj0);

            if ((objIslandTag != islandId) && (objIslandTag != -1))
            {
                printf("error in island management\n");
            }

            btAssert((objIslandTag == islandId) || (objIslandTag == -1));
            if (objIslandTag == islandId)
            {
                if (colObj0->getActivationState()== ACTIVE_TAG)
                {
                    allSleeping = false;
                }
                if (colObj0->getActivationState()== DISABLE_DEACTIVATION)
                {
                    allSleeping = false;
                }
            }
        }


        if (allSleeping)
        {
            int idx;
            for (idx=startIslandIndex;idx<endIslandIndex;idx++)
            {
                int i = getUnionFind().getElement(idx).m_sz;
                btCollisionObject *colObj0 = collisionObjects[i];
                int objIslandTag = -2;
                if(sodaWorld->ownsCollisionObject(colObj0))
                    objIslandTag = colObj0->getIslandTag();
                else
                    objIslandTag = foreignCache->getIslandTag(colObj0);


                if ((objIslandTag != islandId) && (objIslandTag != -1))
                {
//					printf("error in island management\n");
                }

                btAssert((objIslandTag == islandId) || (objIslandTag == -1));

                if (objIslandTag == islandId)
                {
                    colObj0->setActivationState( ISLAND_SLEEPING );
                }
            }
        } else
        {

            int idx;
            for (idx=startIslandIndex;idx<endIslandIndex;idx++)
            {
                int i = getUnionFind().getElement(idx).m_sz;

                btCollisionObject *colObj0 = collisionObjects[i];
                int objIslandTag = -2;
                if(sodaWorld->ownsCollisionObject(colObj0))
                    objIslandTag = colObj0->getIslandTag();
                else
                    objIslandTag = foreignCache->getIslandTag(colObj0);

                if ((objIslandTag != islandId) && (objIslandTag != -1))
                {
                    printf("error in island management#2\n");
                }

                btAssert((objIslandTag == islandId) || (objIslandTag == -1));

                if (objIslandTag == islandId)
                {
                    if ( colObj0->getActivationState() == ISLAND_SLEEPING)
                    {
                        if(sodaWorld->ownsCollisionObject(colObj0))
                        {
                            colObj0->setActivationState( WANTS_DEACTIVATION);
                            colObj0->setDeactivationTime(0.f);
                        }
                        else
                        {
                            foreignCache->setActivationState(colObj0, WANTS_DEACTIVATION);
                            foreignCache->setDeactivationTime(colObj0, 0.f);
                        }
                    }
                }
            }
        }
    }


    int i;
    int maxNumManifolds = dispatcher->getNumManifolds();

//#define SPLIT_ISLANDS 1
//#ifdef SPLIT_ISLANDS


//#endif //SPLIT_ISLANDS


    for (i=0;i<maxNumManifolds ;i++)
    {
         btPersistentManifold *manifold = dispatcher->getManifoldByIndexInternal(i);

         btCollisionObject *colObj0 = static_cast<btCollisionObject*>(manifold->getBody0());
         btCollisionObject *colObj1 = static_cast<btCollisionObject*>(manifold->getBody1());

         //TODO: (note from Bullet 2.8 in buildIslands()) check sleeping conditions!
         if (((colObj0) && colObj0->getActivationState() != ISLAND_SLEEPING) ||
            ((colObj1) && colObj1->getActivationState() != ISLAND_SLEEPING))
        {

            //kinematic objects don't merge islands, but wake up all connected objects
            if (colObj0->isKinematicObject() && colObj0->getActivationState() != ISLAND_SLEEPING)
            {
                colObj1->activate();
            }
            if (colObj1->isKinematicObject() && colObj1->getActivationState() != ISLAND_SLEEPING)
            {
                colObj0->activate();
            }
            if(m_splitIslands)
            {
                //filtering for response
                if (dispatcher->needsResponse(colObj0,colObj1))
                    m_islandmanifold.push_back(manifold);
            }
        }
    }


    // Manage manifolds of foreign bodies
    {
        const btAlignedObjectArray<sodaPersistentForeignerManifold *> &manifolds = sodaWorld->getContactResultCallback().getManifolds();
        int manifoldSize = manifolds.size();
        sodaPersistentForeignerManifold *foreignManifold = 0;

        for(int i=0; i<manifoldSize; ++i)
        {
            foreignManifold = manifolds[i];

            // Use foreign manifolds to find more unions
            btCollisionObject *local = (btCollisionObject*) foreignManifold->getLocalBody();
            btCollisionObject *foreigner = (btCollisionObject*) foreignManifold->getForeignBody();

            //TODO: (note from Bullet 2.8 in buildIslands()) check sleeping conditions!
            if (((local) && local->getActivationState() != ISLAND_SLEEPING) ||
               ((foreigner) && foreignCache->getActivationState(foreigner) != ISLAND_SLEEPING))
            {
                //kinematic objects don't merge islands, but wake up all connected objects
                if (local->isKinematicObject() && local->getActivationState() != ISLAND_SLEEPING)
                {
                    if(!(foreigner->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT|btCollisionObject::CF_KINEMATIC_OBJECT)))
                    {
                        foreignCache->setActivationState(foreigner, ACTIVE_TAG);
                        foreignCache->setDeactivationTime(foreigner, btScalar(0.));
                    }
                }
                if (foreigner->isKinematicObject() && foreignCache->getActivationState(foreigner) != ISLAND_SLEEPING)
                {
                    local->activate();
                }
                if(m_splitIslands)
                {
                    //filtering for response (will have to check if manifolds are normal or foreign in the future)
                    if (dispatcher->needsResponse(local, foreigner))
                        m_islandmanifold.push_back(foreignManifold);
                }
            }
        }
    }
}



//TODO: (note from Bullet 2.8 in buildAndProcessIslands()) this is random access, it can be walked 'cache friendly'!
void sodaSimulationIslandManager::buildAndProcessIslands(btDispatcher *dispatcher, sodaDynamicsWorld *collisionWorld, IslandCallback *callback)
{
    btCollisionObjectArray& collisionObjects = collisionWorld->getCollisionObjectArray();

    buildIslands(dispatcher,collisionWorld);

    int endIslandIndex=1;
    int startIslandIndex;
    int numElem = getUnionFind().getNumElements();

    BT_PROFILE("processIslands");

    if(!m_splitIslands)
    {
        btPersistentManifold **manifold = dispatcher->getInternalManifoldPointer();
        int maxNumManifolds = dispatcher->getNumManifolds();

        //TODO: btForeignManifold array also in param with maxNumForeignManifolds
        //TODO: modify IslandCallback to solve constraints without impacting foreigners (will include Inter solver com framework for result exchange - depends on type of computation) - requires access to ownsCollisionObject() from the world that owns the solver

        callback->ProcessIsland(&collisionObjects[0],collisionObjects.size(),manifold,maxNumManifolds, -1);
    }
    else
    {
        // Sort manifolds, based on islands
        // Sort the vector using predicate and std::sort
        //std::sort(islandmanifold.begin(), islandmanifold.end(), btPersistentManifoldSortPredicate);

        int numManifolds = int (m_islandmanifold.size());

        //we should do radix sort, it it much faster (O(n) instead of O (n log2(n))
        m_islandmanifold.quickSort(sodaPersistentManifoldSortPredicate(this));

        //now process all active islands (sets of manifolds for now)

        int startManifoldIndex = 0;
        int endManifoldIndex = 1;

        //int islandId;



    //	printf("Start Islands\n");

        //traverse the simulation islands, and call the solver, unless all objects are sleeping/deactivated
        for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
        {
            int islandId = getUnionFind().getElement(startIslandIndex).m_id;


               bool islandSleeping = true;

                    for (endIslandIndex = startIslandIndex;(endIslandIndex<numElem) && (getUnionFind().getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
                    {
                            int i = getUnionFind().getElement(endIslandIndex).m_sz;
                            btCollisionObject *colObj0 = collisionObjects[i];
                            m_islandBodies.push_back(colObj0);
                            if (colObj0->isActive())
                                    islandSleeping = false;
                    }


            //find the accompanying contact manifold for this islandId
            int numIslandManifolds = 0;
            btPersistentManifold **startManifold = 0;

            if (startManifoldIndex<numManifolds)
            {
                int curIslandId = getIslandId(m_islandmanifold[startManifoldIndex]);
                if (curIslandId == islandId)
                {
                    startManifold = &m_islandmanifold[startManifoldIndex];

                    for (endManifoldIndex = startManifoldIndex+1;(endManifoldIndex<numManifolds) && (islandId == getIslandId(m_islandmanifold[endManifoldIndex]));endManifoldIndex++)
                    {

                    }
                    /// Process the actual simulation, only if not sleeping/deactivated
                    numIslandManifolds = endManifoldIndex-startManifoldIndex;
                }

            }

            if (!islandSleeping)
            {
                callback->ProcessIsland(&m_islandBodies[0],m_islandBodies.size(),startManifold,numIslandManifolds, islandId);
    //			printf("Island callback of size:%d bodies, %d manifolds\n",islandBodies.size(),numIslandManifolds);
            }

            if (numIslandManifolds)
            {
                startManifoldIndex = endManifoldIndex;
            }

            m_islandBodies.resize(0);
        }
    } // else if(!splitIslands)

}
