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
#ifndef SODASIMULATIONISLANDMANAGER_H
#define SODASIMULATIONISLANDMANAGER_H

#include "BulletCollision/CollisionDispatch/btUnionFind.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

#include "sodaSimulationIslandForeignerCache.h"

// Forward declarations
class sodaDynamicsWorld;
class btCollisionObject;
class btDispatcher;
class btPersistentManifold;

/*! \class sodaSimulationIslandManager
  * \brief A rewritten version of the btSimulationIslandManager that handles pointers to foreign btCollisionObjects.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an override of btSimulationIslandManager that handles pointers
  * to foreign btCollisionObjects and that allows building islands that include
  * them, but does not yet propose constraint solving with foreign bodies.
  */
class sodaSimulationIslandManager
{
    friend class sodaPersistentManifoldSortPredicate;

public:
    /*!
      * \brief Default constructor.
      * \param sodaWorld the world that simulates objects in this island manager
      * \return a new sodaSimulationIslandManager
      */
    sodaSimulationIslandManager(sodaDynamicsWorld *sodaWorld);

    /*!
     * \brief Destructor.
     */
    ~sodaSimulationIslandManager();

    /*!
     * \brief Initializes the union find object in which islands are stored.
     * \param n unknown parameter. See Bullet documentation.
     */
    void initUnionFind(int n);

    /*!
     * \brief Returns the union find object in which islands are stored.
     * \return a reference to the btUnionFind of this island manager
     */
    inline btUnionFind& getUnionFind()
    {
        return m_unionFind;
    }

    /*!
     * \brief Updates the activation state of bodies depending on their island. See Bullet documentation.
     * \param collisionWorld the sodaDynamicsWorld linked to this sodaSimulationIslandManager
     * \param dispatcher the btDispatcher linked to this sodaSimulationIslandManager
     */
    void updateActivationState(sodaDynamicsWorld *collisionWorld, btDispatcher *dispatcher);

    /*!
     * \brief Stores the new activation state of bodies. See Bullet documentation.
     * \param collisionWorld the sodaDynamicsWorld linked to this sodaSimulationIslandManager
     */
    void storeIslandActivationState(sodaDynamicsWorld *collisionWorld);

    /*!
     * \brief Unknown function. See Bullet documentation.
     * \param dispatcher the btDispatcher linked to this sodaSimulationIslandManager
     * \param collisionWorld the sodaDynamicsWorld linked to this sodaSimulationIslandManager
     */
    void findUnions(btDispatcher *dispatcher, sodaDynamicsWorld *collisionWorld);

    /*!
     * \brief A callback for processing simulation islands.
     */
    struct IslandCallback
    {
        virtual ~IslandCallback() {}

        /*!
         * \brief Processes an island. See Bullet documentation.
         * \param bodies pointers to bodies within the island
         * \param numBodies number of bodies
         * \param manifolds manifolds containing contact information between bodies
         * \param numManifolds number of manifolds
         * \param islandId Id of the simulation island
         */
        virtual void ProcessIsland(btCollisionObject **bodies, int numBodies, btPersistentManifold **manifolds, int numManifolds, int islandId) = 0;
    };

    /*!
     * \brief Builds and processes collisions on simulation islands. See Bullet documentation.
     * \param dispatcher the btDispatcher linked to this sodaSimulationIslandManager
     * \param collisionWorld the sodaDynamicsWorld linked to this sodaSimulationIslandManager
     * \param callback the callback used to process islands
     */
    void buildAndProcessIslands(btDispatcher *dispatcher, sodaDynamicsWorld *collisionWorld, IslandCallback *callback);

    /*!
     * \brief Builds simulation islands. See Bullet documentation.
     * \param dispatcher the btDispatcher linked to this sodaSimulationIslandManager
     * \param collisionWorld the sodaDynamicsWorld linked to this sodaSimulationIslandManager
     */
    void buildIslands(btDispatcher *dispatcher, sodaDynamicsWorld *collisionWorld);

    /*!
     * \brief Apparently tells whether islands should be processed separately. See Bullet documentation.
     * \return whether islands should be split
     */
    bool getSplitIslands()
    {
        return m_splitIslands;
    }

    /*!
     * \brief Apparently sets whether islands should be processed separately. See Bullet documentation.
     * \param doSplitIslands whether islands should be split
     */
    void setSplitIslands(bool doSplitIslands)
    {
        m_splitIslands = doSplitIslands;
    }

protected:

    /*! \class sodaPersistentManifoldSortPredicate
      * \brief This class routes calls to operator< between btPersistentManifold objects.
      * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
      *
      * This class is a simple sorting method class that routes calls to operator<
      * between btPersistentManifold objects, making sure to handle those that contain
      * objects from a foreign sodaDynamicsWorld.
      */
    class sodaPersistentManifoldSortPredicate
    {
    public:
        /*!
          * \brief Default constructor.
          * \param mgr the sodaSimulationIslandManager this sorter operates with
          * \return a new sodaPersistentManifoldSortPredicate
          */
        sodaPersistentManifoldSortPredicate(sodaSimulationIslandManager *mgr) :
            mgr(mgr)
        {
        }

        /*!
          * \brief Operator() that performs the sorting.
          * \param lhs the manifold that will be checked to have a smaller id than its counterpart
          * \param rhs the manifold that will be checked to have a bigger id than its counterpart
          * \return whether lhs' island Id is smaller than rhs'.
          */
        SIMD_FORCE_INLINE bool operator() (const btPersistentManifold *lhs, const btPersistentManifold *rhs)
        {
            return mgr->getIslandId(lhs) < mgr->getIslandId(rhs);
        }

    private:
            sodaSimulationIslandManager *mgr;   /*!< Pointer to the sodaSimulationIslandManager that makes use of this object */
    };


    /*!
     * \brief Returns the island Id for a given manifold.
     * \param lhs the manifold
     * \return the Id if the first body of the manifold, or of the second if the first's wasn't a valid Id
     */
    int getIslandId(const btPersistentManifold *lhs);

    btUnionFind                                  m_unionFind;       /*!< the structure that stores islands */

    btAlignedObjectArray<btPersistentManifold *> m_islandmanifold;  /*!< an array to stores btPersistentManifolds of an island? */
    btAlignedObjectArray<btCollisionObject *>    m_islandBodies;    /*!< an array to stores btCollisionObjects of an island? */

    bool                                         m_splitIslands;    /*!< whether islands should be split */

    sodaDynamicsWorld                            *sodaWorld;        /*!< pointer to the related sodaDynamicsWorld */
    sodaSimulationIslandForeignerCache           *foreignCache;     /*!< cache for foreign btCollisionObject attributes */

};

#endif // SODASIMULATIONISLANDMANAGER_H

