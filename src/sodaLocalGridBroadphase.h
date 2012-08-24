/*
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
#ifndef BTLOCALGRIDBROADPHASE_H
#define BTLOCALGRIDBROADPHASE_H

#include <QtDebug>
#include <btBulletCollisionCommon.h>
#include "sodaEntity.h"

// Forward declaration
class sodaLogicWorld;

/*! \class sodaLocalGridBroadphase
  * \brief A broadphase implementation making use of LocalGrid.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a spatial subdivision uniform grid broadphase, based
  * on the LocalGrid that already exists within sodaLogicWorlds.
  *
  * btBroadphaseProxy objects are manipulated instead of direct entities. These objects
  * are created by the Bullet world (in our case, BulletManagerWorld), directly from the
  * btCollisionShapes. A method exists to retrieve obEntity instances from these:
  * BulletManagerWorld::getEntityFromProxy().
  */
class sodaLocalGridBroadphase : public btBroadphaseInterface
{
public:
    /*!
     * \brief Default constructor.
     * \param world the world that this broadphase works on
     * \param overlappingPairCache pointer to an already allocated pair cache that can be used
     * \return a new btLocalGridBroadphase
     */
    sodaLocalGridBroadphase(sodaLogicWorld *world = 0, btOverlappingPairCache *overlappingPairCache = 0);

    /*!
     * \brief Default destructor.
     */
    ~sodaLocalGridBroadphase();

    /*!
     * \brief Creates a btBroadphaseProxy for an entity of this object's world.
     * \param aabbMin the min coordinates of this entity's AABB
     * \param aabbMax the max coordinates of this entity's AABB
     * \param shapetype the Bullet shape type of the entity's btCollisionShape
     * \param userPtr a user pointer (set to the btCollisionShape of the object by Bullet)
     * \param collisionFilterGroup the Collision Filter Group of the entity (see tutorial on Collision callback)
     * \param collisionFilterMask the Collision Filter Mask of the entity
     * \param dispatcher the btDispatcher that manages narrow-phase computations
     * \param multiSapProxy a null pointer for API compatibility
     * \return a new btBroadphaseProxy
     */
    virtual btBroadphaseProxy *createProxy(const btVector3 &aabbMin, const btVector3 &aabbMax, int shapetype, void *userPtr , short int collisionFilterGroup, short int collisionFilterMask, btDispatcher *dispatcher, void *multiSapProxy);

    /*!
     * \brief Deletes an existing btBroadphaseProxy.
     * \param proxy the proxy to delete
     * \param dispatcher the dispatcher in use with this broadphase object
     */
    virtual void destroyProxy(btBroadphaseProxy *proxy, btDispatcher *dispatcher);

    /*!
     * \brief Tests whether two proxies' AABBs overlap.
     * \param proxy0 the first proxy
     * \param proxy1 the second proxy
     * \return whether the AABBs overlap
     */
    bool aabbOverlap(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1);

    /*!
     * \brief Tests whether two AABBs overlap.
     * \param aabb0Min min coords of the first AABB
     * \param aabb0Max max coords of the first AABB
     * \param aabb1Min min coords of the second AABB
     * \param aabb1Max max coords of the second AABB
     * \return whether the AABBs overlap
     */
    bool aabbOverlap(const btVector3 &aabb0Min, const btVector3 &aabb0Max, const btVector3 &aabb1Min, const btVector3 &aabb1Max);

    /*!
     * \brief Calculates the pairs of entities that overlap.
     * \param dispatcher the dispatcher which will later process overlapping pairs
     */
    virtual void calculateOverlappingPairs(btDispatcher *dispatcher);

    /*!
     * \brief Retrieves the AABB of a btBroadphaseProxy by writing it into parameters.
     * \param proxy the proxy whose AABB to get
     * \param aabbMin placeholder for the min coordinates of the AABB
     * \param aabbMax placeholder for the max coordinates of the AABB
     */
    inline void getAabb(btBroadphaseProxy *proxy, btVector3 &aabbMin, btVector3 &aabbMax) const
    {
        aabbMin = proxy->m_aabbMin;
        aabbMax = proxy->m_aabbMax;
    }

    /*!
     * \brief Sets the AABB of a btBroadphaseProxy using parameters.
     * \param proxy the proxy whose AABB to set
     * \param aabbMin the min coordinates of the AABB
     * \param aabbMax the max coordinates of the AABB
     */
    inline void setAabb(btBroadphaseProxy *proxy, const btVector3 &aabbMin, const btVector3 &aabbMax, btDispatcher */*dispatcher*/)
    {
        proxy->m_aabbMin = aabbMin;
        proxy->m_aabbMax = aabbMax;
    }

    /*!
     * \brief A function that does nothing, declared for API compatibility.
     * \warning Not yet implemented.
     */
    virtual void rayTest(const btVector3 &/*rayFrom*/,const btVector3 &/*rayTo*/, btBroadphaseRayCallback &/*rayCallback*/, const btVector3 &/*aabbMin=btVector3(0,0,0)*/, const btVector3 &/*aabbMax = btVector3(0,0,0)*/)
    {}

    /*!
     * \brief Tests what entities overlap a given AABB and calls a callback for each of these entities.
     * \param aabbMin the min coordinates of the AABB to test
     * \param aabbMax the max coordinates of the AABB to test
     * \param callback the callback to call for overlapping entities
     */
    virtual void aabbTest(const btVector3 &aabbMin, const btVector3 &aabbMax, btBroadphaseAabbCallback &callback);

    /*!
     * \brief Returns a pointer to overlapping pair cache of this btLocalGridBroadphase.
     * \return a pointer to this object's overlapping pair cache
     */
    inline virtual btOverlappingPairCache *getOverlappingPairCache()
    {
        return m_pairCache;
    }

    /*!
     * \brief Returns a constant pointer to overlapping pair cache of this btLocalGridBroadphase.
     * \return a constant pointer to this object's overlapping pair cache
     */
    inline virtual const btOverlappingPairCache *getOverlappingPairCache() const
    {
        return m_pairCache;
    }

    /*!
     * \brief Returns a pointer to the border crossing pair cache of this btLocalGridBroadphase.
     * \return a pointer to this object's border cache
     */
    inline virtual btOverlappingPairCache *getBorderCrossingPairCache()
    {
        return m_borderCache;
    }

    /*!
     * \brief Returns a constant pointer to the border crossing pair cache of this btLocalGridBroadphase.
     * \return a constant pointer to this object's border cache
     */
    inline virtual const btOverlappingPairCache *getBorderCrossingPairCache() const
    {
        return m_borderCache;
    }

    /*!
     * \brief Returns the axis aligned bounding box in the 'global' coordinate frame.
     * \param aabbMin the btVector3 in which to write the min coordinates of the AABB
     * \param aabbMax the btVector3 in which to write the max coordinates of the AABB
     */
    inline virtual void getBroadphaseAabb(btVector3 &aabbMin, btVector3 &aabbMax) const
    {
        aabbMin.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
        aabbMax.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
    }

    /*!
     * \brief A function that does nothing, declared for API compatibility.
     * \warning Not yet implemented.
     */
    inline virtual void resetPool(btDispatcher */*dispatcher*/)
    {}

    /*!
     * \brief Prints statistics about the btLocalGridBroadphase.
     * \warning Not yet implemented.
     */
    inline virtual void printStats()
    {
        qWarning() << "printStats is not implemented yet for btLocalGridBroadphase.";
    }

    /*!
     * \brief Sets a sodaLogicWorld to use for the spatial subdivision broadphase.
     * \param newWorld the world to use
     */
    inline void setWorld(sodaLogicWorld *newWorld)
    {
        world = newWorld;
    }

    /*!
     * \brief Removes any previously set sodaLogicWorld for the broadphase, which will not return any collision anymore
     */
    inline void unsetWorld()
    {
        world = 0;
    }

    /*!
     * \brief Returns a pointer to this broadphase's sodaLogicWorld.
     * \return a pointer to this btLocalGridBroadphase's sodaLogicWorld
     */
    inline sodaLogicWorld *getWorld() const
    {
        return world;
    }

private:
    sodaLogicWorld            *world;             /*!< Pointer to the sodaLogicWorld containing the grid used within this broadphase */
    btOverlappingPairCache  *m_pairCache;       /*!< Pointer to the cache used for sodaDynamicEntity-sodaDynamicEntity overlaps */
    btOverlappingPairCache  *m_borderCache;     /*!< Pointer to the cache used for sodaDynamicEntity-CellBorderEntity overlaps (always owned by self) */
    bool                    m_ownsPairCache;    /*!< Whether the object pointed to by m_pairCache belongs to this object */

};

#endif // BTLOCALGRIDBROADPHASE_H
