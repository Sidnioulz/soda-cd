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
#ifndef BULLETMANAGER_H
#define BULLETMANAGER_H

#include <btBulletDynamicsCommon.h>
#include "sodaLocalGridBroadphase.h"
#include "sodaLogicWorld.h"
#include "sodaDynamicsWorld.h"


/*! \class BulletManager
  * \brief A wrapper for Bullet physics library initialization and management.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements a Bullet physics engine, initializes its various components, and
  * provides an interface to step simulations and get physics answers from it.
  * It is meant to be used by a sodaLogicWorld with a sodaLocalGrid.
  */
class BulletManager
{
public:
    /*!
      * \brief Default constructor.
      * \return a new BulletManager
      */
    explicit BulletManager(sodaLogicWorld *world);

    /*!
      * \brief Default destructor.
      */
    ~BulletManager();

    /*!
      * \brief Returns the dynamics world of the bullet instance.
      * \return the btDynamicsWorld of this instance
      */
    inline sodaDynamicsWorld* getDynamicsWorld() const
    {
        return dynamicsWorld;
    }

private:
    sodaLogicWorld                        *sodaWorld;         /*!< The SODA object for a World */

    sodaLocalGridBroadphase             *broadphase;        /*!< Broadphase algorithm interface for collision detection */
    btDefaultCollisionConfiguration     *collisionConfig;   /*!< Configuration of the Bullet engine */
    btCollisionDispatcher               *dispatcher;        /*!< the exact detection algorithm */
    btSequentialImpulseConstraintSolver *solver;            /*!< the physics response solver */

    sodaDynamicsWorld                   *dynamicsWorld;     /*!< The Bullet Physics representation of the world */
};

#endif // BULLETMANAGER_H
