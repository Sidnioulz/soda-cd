/*
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Quentin Avril <quentin.avril@irisa.fr>
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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef SODAPERSISTENTFOREIGNERMANIFOLD_H
#define SODAPERSISTENTFOREIGNERMANIFOLD_H

#include <btBulletCollisionCommon.h>
#include "sodaPersistentForeignerManifoldArray.h"

/*! \class sodaPersistentForeignerManifold
  * \brief An override of btPersistentManifold with foreign and local body getters and setters.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an overlay to btPersistentManifold that just gives an explicit
  * semantic sense to body0 and body1 and that can be used as a manifold between
  * a foreign and local bodies. It does not provide any sanity checks but should
  * be seen as an helper to SODA developers: if foreign and local bodies are used
  * consistently in setters, they will also be consistent in getters (without the
  * need for extra checks).
  *
  * body0: local body
  * body1: foreign body
  */
class sodaPersistentForeignerManifold : public btPersistentManifold
{
public:
    /*!
     * \brief Default constructor
     * \param localBody the local btCollisionObject
     * \param foreignBody the foreign btCollisionObject
     * \param ContactBreakingThreshold see Bullet documentation
     * \param ContactProcessingThreshold see Bullet documentation
     * \return a new sodaPersistentForeignerManifold
     */
    sodaPersistentForeignerManifold(void *localBody, void *foreignBody, int, btScalar ContactBreakingThreshold, btScalar ContactProcessingThreshold) :
        btPersistentManifold(localBody, foreignBody, 0, ContactBreakingThreshold, ContactProcessingThreshold),
        lastTimeActive(0)
    {
    }

    /*!
     * \brief Sets the bodies to be used in this manifold.
     * \param localBody the local btCollisionObject
     * \param foreignBody the foreign btCollisionObject
     */
    inline void setBodies(void *localBody, void *foreignBody)
    {
        btPersistentManifold::setBodies(localBody, foreignBody);
    }

    /*!
     * \brief Sets the timestamp at which the contact points stored in this manifold were last valid.
     * \param newTime the new last valid time
     */
    inline void setLastTime(const btScalar &newTime)
    {
        lastTimeActive = newTime;
    }

    /*!
     * \brief Retrieves the timestamp at which the contact points stored in this manifold were last valid.
     * \return the last valid time
     */
    inline btScalar getLastTime() const
    {
        return lastTimeActive;
    }

    /*!
     * \brief Returns a pointer to the local btCollisionObject.
     * \return a void pointer to the local object that has been set in this manifold
     */
    inline void *getLocalBody()
    {
        return getBody0();
    }

    /*!
     * \brief Returns a constant pointer to the local btCollisionObject.
     * \return a constant void pointer to the local object that has been set in this manifold
     */
    inline const void *getLocalBody() const
    {
        return getBody0();
    }

    /*!
     * \brief Returns a pointer to the foreign btCollisionObject.
     * \return a void pointer to the foreign object that has been set in this manifold
     */
    inline void *getForeignBody()
    {
        return getBody1();
    }


    /*!
     * \brief Returns a constant pointer to the foreign btCollisionObject.
     * \return a constant void pointer to the foreign object that has been set in this manifold
     */
    inline const void *getForeignBody() const
    {
        return getBody1();
    }

protected:

    btScalar lastTimeActive;        /*!< Last time at which the contact points within this manifold were valid */

};

#endif // SODAPERSISTENTFOREIGNERMANIFOLD_H
