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
#include "localgrid.h"

struct btLocalGridProxy : public btBroadphaseProxy
{
    btLocalGridProxy() :
        parentEntity(0)
    {}

    btLocalGridProxy(const btVector3& minpt,const btVector3& maxpt,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,void* multiSapProxy) :
        btBroadphaseProxy(minpt,maxpt,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy),
        parentEntity(0)
    {
        parentEntity = static_cast<obEntity *>(userPtr);
    }

    obEntity *parentEntity;
};

class btLocalGridBroadphase : public btBroadphaseInterface
{
public:
    btLocalGridBroadphase(LocalGrid *grid = 0, btOverlappingPairCache *overlappingPairCache = 0);
        ~btLocalGridBroadphase();

    virtual btBroadphaseProxy *createProxy(const btVector3 &aabbMin, const btVector3 &aabbMax, int shapetype, void *userPtr , short int collisionFilterGroup, short int collisionFilterMask, btDispatcher *dispatcher, void *multiSapProxy);

    virtual void destroyProxy(btBroadphaseProxy *proxyOrg, btDispatcher *dispatcher);

    bool aabbOverlap(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1);

    virtual void calculateOverlappingPairs(btDispatcher *dispatcher);

    inline btLocalGridProxy *castProxy(btBroadphaseProxy *proxy)
    {
        return static_cast<btLocalGridProxy*>(proxy);
    }

    inline const btLocalGridProxy *castProxy(btBroadphaseProxy *proxy) const
    {
        return static_cast<const btLocalGridProxy*>(proxy);
    }

    inline void getAabb(btBroadphaseProxy *proxy, btVector3 &aabbMin, btVector3 &aabbMax) const
    {
        aabbMin = proxy->m_aabbMin;
        aabbMax = proxy->m_aabbMax;
    }

    inline void setAabb(btBroadphaseProxy *proxy, const btVector3 &aabbMin, const btVector3 &aabbMax, btDispatcher */*dispatcher*/)
    {
        proxy->m_aabbMin = aabbMin;
        proxy->m_aabbMax = aabbMax;
    }


    virtual void rayTest(const btVector3 &rayFrom,const btVector3 &rayTo, btBroadphaseRayCallback &rayCallback, const btVector3 &aabbMin=btVector3(0,0,0), const btVector3 &aabbMax = btVector3(0,0,0));

    virtual void aabbTest(const btVector3 &aabbMin, const btVector3 &aabbMax, btBroadphaseAabbCallback &callback);

    inline virtual btOverlappingPairCache *getOverlappingPairCache()
    {
        return m_pairCache;
    }

    inline virtual const btOverlappingPairCache *getOverlappingPairCache() const
    {
        return m_pairCache;
    }

    ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
    ///will add some transform later
    inline virtual void getBroadphaseAabb(btVector3 &aabbMin,btVector3 &aabbMax) const
    {
        aabbMin.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
        aabbMax.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
    }






    virtual void resetPool(btDispatcher *dispatcher)
    {
    }


    inline virtual void printStats()
    {
        qWarning() << "printStats is not implemented yet for btLocalGridBroadphase.";
    }






















    /*!
     * \brief Sets a LocalGrid to use for the spatial subdivision broadphase.
     * \param grid the grid to use
     */
    inline void setLocalGrid(LocalGrid *newGrid)
    {
        grid = newGrid;
    }

    /*!
     * \brief Removes any previously set LocalGrid for the broadphase, which will not return any collision anymore
     */
    inline void unsetLocalGrid()
    {
        grid = 0;
    }

private:
    LocalGrid               *grid;              /*!< A pointer to the LocalGrid used within this broadphase */
    btOverlappingPairCache  *m_pairCache;
    bool                    m_ownsPairCache;

};

#endif // BTLOCALGRIDBROADPHASE_H
