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
#ifndef SODACONTACTRESULTCALLBACK_H
#define SODACONTACTRESULTCALLBACK_H

#include <btBulletCollisionCommon.h>
#include <QPair>
#include "sodaPersistentForeignerManifold.h"

// Forward declaration
class sodaDynamicsWorld;

/*! \struct sodaCRCLastTimePredicate
  * \brief A predicate that sorts sodaPersistentForeignerManifolds per decreasing last time.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  */
struct sodaCRCLastTimePredicate
{
    /*!
     * \brief Sorting operator.
     * \param lhs the first manifold
     * \param rhs the second manifold
     * \return whether the first manifold has a higher last time than the second one
     */
    bool operator() (const sodaPersistentForeignerManifold *lhs, const sodaPersistentForeignerManifold *rhs)
    {
        return lhs->getLastTime() > rhs->getLastTime();
    }
};

/*! \class sodaContactResultCallback
  * \brief An override of Bullet's ContactResultCallback for foreign-local collisions.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements a btCollisionWorld::ContactResultCallback that has a cache
  * of special btPersistentManifolds with pointers to a local and a foreign object.
  * It can be used to keep track of foreign btCollisionObjects that need collision
  * processing in a sodaDynamicsWorld.
  */
class sodaContactResultCallback : public btCollisionWorld::ContactResultCallback {
public:
    /*! \brief Default constructor.
      * \param world the Bullet world which btCollisionObjects to be used are from
      * \return a new sodaContactResultCallback
      */
    sodaContactResultCallback(sodaDynamicsWorld *world);

    /*!
     * \brief Destructor.
     */
    ~sodaContactResultCallback();

    /*!
     * \brief Adds a collision result to the callback in case of contact and sets the collision flag.
     * \param cp the contact point between both objects
     * \param colObj0 the first btCollisionObject
     * \param colObj1 the second btCollisionObject
     * \return an arbitrary unused value
     */
    virtual btScalar addSingleResult(btManifoldPoint& cp,
                                     const btCollisionObject* colObj0, int /*partId0*/, int /*index0*/,
                                     const btCollisionObject* colObj1, int /*partId1*/, int /*index1*/);

    /*!
     * \brief Removes cached btPersistentManifolds associated to a timestamp lower than the time parameter.
     * \param time the timestamp from which btPersistentManifolds should be kept in the callback's cache
     * \return the list of foreign btCollisionObjects that are not needed in the world anymore
     */
    btAlignedObjectArray<btCollisionObject *> removeOutdatedManifolds(const btScalar &time);

    /*!
     * \brief Resets a flag turned to true by addSingleResult when a collision occured on last contactPairTest call.
     *
     * \see getCollisionFlag()
     */
    inline void resetCollisionFlag()
    {
        colOccurred = false;
    }

    /*!
     * \brief Returns the collision flag set to true by a contactPairTest call in which a collision was detected.
     *
     * \see resetCollisionFlag()
     */
    inline bool getCollisionFlag() const
    {
        return colOccurred;
    }

    /*!
     * \brief Returns the list of btPersistentManifolds in the callback's cache.
     * \return an array of sodaPersistentForeignerManifold objects representing current local-foreign collisions
     */
    inline const btAlignedObjectArray<sodaPersistentForeignerManifold *> &getManifolds() const
    {
        return collisionManifolds;
    }

protected:
    sodaDynamicsWorld                    *sodaWorld;            /*!< Pointer to the world that will call contactPairTest() with this callback */
    sodaPersistentForeignerManifoldArray collisionManifolds;    /*!< Cache of special foreign-local btPersistentManifolds for existing collisions */
    bool                                 colOccurred;           /*!< Flag set to true by addSingleResult() and to false by resetCollisionFlag() */
    int                                  resultCount;           /*!< Internal count used to cleanup collisionManifolds when requested by users */
};

#endif // SODACONTACTRESULTCALLBACK_H
