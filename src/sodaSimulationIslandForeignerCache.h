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
#ifndef SODASIMULATIONISLANDFOREIGNERCACHE_H
#define SODASIMULATIONISLANDFOREIGNERCACHE_H

#include <QHash>
#include <btBulletCollisionCommon.h>
#include "sodaDynamicEntity.h"


/*! \class sodaSimulationIslandForeignerCache
  * \brief A cache for storing attributes of btCollisionObjects that cannot be modified, and needed by a sodaSimulationIslandManager.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a cache of btCollisionObject attributes, that can be used when
  * the objects belong to another sodaDynamicsWorld and cannot be modified by the
  * sodaSimulationIslandManager that requires setting and getting such attributes.
  */
class sodaSimulationIslandForeignerCache
{
public:
    /*! \brief Default constructor.
      * \return a new sodaSimulationIslandForeignerCache
      */
    explicit sodaSimulationIslandForeignerCache();

    /*!
     * \brief Adds a btCollisionObject to all maps of the cache using its current values in its original sodaDynamicsWorld.
     * \param obj the object to add
     */
    inline void addCollisionObject(const btCollisionObject *obj)
    {
        islandTagMap.insert(obj, obj->getIslandTag());
        companionIdMap.insert(obj, obj->getCompanionId());
        hitFractionMap.insert(obj, obj->getHitFraction());
        activationStateMap.insert(obj, obj->getActivationState());
        deactivationTimeMap.insert(obj, obj->getDeactivationTime());
    }

    /*!
     * \brief Removes a btCollisionObject from all maps of the cache.
     * \param obj the object to remove
     */
    inline void removeCollisionObject(const btCollisionObject *obj)
    {
        islandTagMap.remove(obj);
        companionIdMap.remove(obj);
        hitFractionMap.remove(obj);
        activationStateMap.remove(obj);
        deactivationTimeMap.remove(obj);
    }

    /*!
     * \brief Returns the cached island tag of an object.
     * \param obj the btCollisionObject to query
     * \return the object's cached island tag, or -1 if none
     */
    inline int getIslandTag(const btCollisionObject *obj) const
    {
        return islandTagMap.value(obj, -1);
    }

    /*!
     * \brief Sets an island tag for the object in the cache.
     * \param obj the btCollisionObject to modify
     * \param tag the object's new island tag
     */
    inline void setIslandTag(const btCollisionObject *obj, int tag)
    {
#ifndef NDEBUG
        const sodaDynamicEntity *foreignerEnt = dynamic_cast<const sodaDynamicEntity*>(static_cast<const sodaEntity *>(obj->getUserPointer()));
        qDebug() << "sodaSimulationIslandForeignerCache()::setIslandTag("<< foreignerEnt->getDisplayName() << ", " << tag << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

        islandTagMap.insert(obj, tag);
    }

    /*!
     * \brief Returns the cached companion id of an object.
     * \param obj the btCollisionObject to query
     * \return the object's cached companion id, or -1 if none
     */
    inline int getCompanionId(const btCollisionObject *obj) const
    {
        return companionIdMap.value(obj, -1);
    }

    /*!
     * \brief Sets an companion id for the object in the cache.
     * \param obj the btCollisionObject to modify
     * \param id the object's new companion id
     */
    inline void setCompanionId(const btCollisionObject *obj, int id)
    {
        companionIdMap.insert(obj, id);
    }

    /*!
     * \brief Returns the cached hit fraction of an object.
     * \param obj the btCollisionObject to query
     * \return the object's cached hit fraction, or -1 if none
     */
    inline btScalar getHitFraction(const btCollisionObject *obj) const
    {
        return hitFractionMap.value(obj, -1);
    }

    /*!
     * \brief Sets a hit fraction for the object in the cache.
     * \param obj the btCollisionObject to modify
     * \param fraction the object's new hit fraction
     */
    inline void setHitFraction(const btCollisionObject *obj, btScalar fraction)
    {
        hitFractionMap.insert(obj, fraction);
    }

    /*!
     * \brief Returns the cached activation state of an object.
     * \param obj the btCollisionObject to query
     * \return the object's cached activation state, or DISABLE_SIMULATION if none
     */
    inline int getActivationState(const btCollisionObject *obj) const
    {
        return activationStateMap.value(obj, DISABLE_SIMULATION);
    }

    /*!
     * \brief Sets an activation state for the object in the cache.
     * \param obj the btCollisionObject to modify
     * \param state the new state of the object
     */
    inline void setActivationState(const btCollisionObject *obj, int state)
    {
        activationStateMap.insert(obj, state);
    }

    /*!
     * \brief Returns the cached deactivation time of an object.
     * \param obj the btCollisionObject to query
     * \return the object's cached deactivation time, or 0 if none
     */
    inline btScalar getDeactivationTime(const btCollisionObject *obj) const
    {
        return deactivationTimeMap.value(obj, 0.f);
    }

    /*!
     * \brief Sets a deactivation time for the object in the cache.
     * \param obj the btCollisionObject to modify
     * \param time the object's new deactivation time
     */
    inline void setDeactivationTime(const btCollisionObject *obj, btScalar time)
    {
        deactivationTimeMap.insert(obj, time);
    }

private:
    QHash<const btCollisionObject *, int> islandTagMap;             /*!< Cache for island tags */
    QHash<const btCollisionObject *, int> companionIdMap;           /*!< Cache for companion ids */
    QHash<const btCollisionObject *, btScalar> hitFractionMap;      /*!< Cache for hit fractions */
    QHash<const btCollisionObject *, int> activationStateMap;       /*!< Cache for activation states */
    QHash<const btCollisionObject *, btScalar> deactivationTimeMap; /*!< Cache for deactivation times */
};

#endif // SODASIMULATIONISLANDFOREIGNERCACHE_H
