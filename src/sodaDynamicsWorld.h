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
#ifndef SODADYNAMICSWORLD_H
#define SODADYNAMICSWORLD_H

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <QList>

#include "sodaContactResultCallback.h"
#include "sodaSimulationIslandManager.h"
#include "sodaLocalGridBroadphase.h"
#include "sodaLogicWorld.h"

/*! \class sodaDynamicsWorld
  * \brief An override of a Bullet dynamics world that can synchronize with another instance.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements a Bullet dynamics world with a stepSimulation
  * function that can synchronize with another instance (specifically, the
  * physics response function).
  */
class sodaDynamicsWorld : public btDiscreteDynamicsWorld
{
public:
    /*!
      * \brief Default constructor.
      * \param logicWorld the sodaLogicWorld that owns the entities simulated in this sodaDynamicsWorld.
      * \param dispatcher the Bullet collision dispatcher (provides manifolds for colliding objects)
      * \param broadphase the broadphase algorithm (computes broad-phase pairs for dispatcher)
      * \param solver the physics constraint solver (solves collisions using dispatcher's manifolds)
      * \param config the collision configuration
      * \return a new sodaDynamicsWorld
      */
    sodaDynamicsWorld(sodaLogicWorld *logicWorld, btCollisionDispatcher *&dispatcher, sodaLocalGridBroadphase *&broadphase, btSequentialImpulseConstraintSolver*&solver, btDefaultCollisionConfiguration*&config);

    /*!
      * \brief Destructor. Contains the destructor code of btDiscreteDynamicsWorld and btCollisionWorld.
      */
    ~sodaDynamicsWorld();

    /*!
      * \brief Performs one step of physics simulation.
      * \param timeStep the length of simulation to perform
      * \param maxSubSteps the maximum number of substeps in this iteration
      * \param fixedTimeStep the time step that each substep must last at least
      * \return the number of substeps performed
      *
      * From the Bullet documentation:
      * stepSimulation proceeds the simulationover 'timeStep', units in preferably in seconds.
      *
      * By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
      * in order to keep the simulation real-time, the maximum number of substeps can be clamped to
      *'maxSubSteps'. You can disable subdividing the timestep/substepping by passing maxSubSteps=0
      * as second argument to stepSimulation, but in that case you have to keep the timeStep constant.
      */
    int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.)/btScalar(60));

    /*!
     * \brief Redefinition of getNumCollisionObjects() that excludes pointers to non-owned btCollisionObjects.
     * \return the number of actually self-owned btCollisionObject instances
     */
    int	getNumCollisionObjects() const;

    /*!
     * \brief Returns the ContactResultCallback used by this object to detection collisions with foreign objects.
     * \return a constant reference to this sodaDynamicsWorld's sodaContactResultCallback.
     */
    inline const sodaContactResultCallback &getContactResultCallback() const
    {
        return cb;
    }

    /*!
     * \brief Returns, if possible, the Id of the sodaLogicWorld associated to this sodaDynamicsWorld.
     * \return a sodaLogicWorld id if applicable, NullWorldId otherwise
     */
    inline short getWorldId() const
    {
        if(logicWorld)
            return logicWorld->getId();
        else
            return sodaLogicWorld::NullWorldId;
    }

    /*!
     * \brief Returns the sodaLogicWorld associated to this sodaDynamicsWorld.
     * \return a pointer to the sodaLogicWorld that owns this object
     */
    inline sodaLogicWorld *getLogicWorld()
    {
        return logicWorld;
    }

    /*!
     * \brief Returns the sodaLogicWorld associated to this sodaDynamicsWorld.
     * \return a constant pointer to the sodaLogicWorld that owns this object
     */
    inline const sodaLogicWorld *getLogicWorld() const
    {
        return logicWorld;
    }

    /*!
     * \brief Returns the local time of the sodaDynamicsWorld.
     * \return the local time of this sodaDynamicsWorld
     */
    inline btScalar getLocalTime()
    {
        return m_localTime;
    }

    /*!
     * \brief Tells whether a collision object is owned by this sodaDynamicsWorld.
     * \param obj the object to test
     * \return whether the object has the same owner id as the world
     */
    inline bool ownsCollisionObject(const btCollisionObject *obj)
    {
        return m_objectOwnerIdCache.value(obj, sodaLogicWorld::NullWorldId) == getWorldId();
    }

protected:
    /*!
     * \brief Calls the sodaLogicWorld to check if any synchronization queries were sent by foreign sodaLogicWorlds for a given simulation time.
     * \param simulTime the time at which synchronization requests are to be checked for
     */
    void readExternalSyncRequests(const btScalar &simulTime);

    /*!
     * \brief Waits until other worlds (those with a registered border traversal) reached the same simulation timestep for synchronous simulation.
     * \param simulTime the time at which to wait for neighbors
     */
    void waitForNeighbors(const btScalar &simulTime);

    /*!
     * \brief Waits until other worlds finished the current step simulation.
     * \param neighbors the ids of neighbors to wait for
     * \param simulTime the time at which to wait for neighbors
     *
     * \note Calling this function at the end of each step simulation is necessary
     * to make sure that other sodaDynamicsWorlds don't rely on objects owned by this
     * sodaDynamicsWorld and that may be deleted at the start of next step.
     */
    void waitEndStep(const QList<short> &neighbors, const btScalar &simulTime);

    /*!
      * \brief Internal function that performs the actual simulation.
      * \param timeStep the length of simulation to perform
      *
      * From the Bullet documentation:
      * apply gravity, predict motion
      * perform collision detection
      * solve contact and other joint constraints
      * CallbackTriggers();
      * integrate transforms
      * update vehicle simulation
      */
    void internalSingleStepSimulation(btScalar timeStep);

    /*!
     * \brief Adds a btCollisionObject to the underlying btCollisionWorld. See official Bullet documentation.
     * \param collisionObject the btCollisionObject to add
     * \param collisionFilterGroup please refer to the Bullet documentation
     * \param collisionFilterMask please refer to the Bullet documentation
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void addCollisionObject(btCollisionObject *collisionObject, short int collisionFilterGroup, short int collisionFilterMask);

    /*!
     * \brief Adds a foreign btCollisionObject to the underlying btCollisionWorld.
     * \param foreignId the id of the sodaEntity this btCollisionObject belongs to
     * \param collisionObject the btCollisionObject to add
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     *
     * This method, contrarily to addCollisionObject(), specifies a foreign
     * world id.
     */
    void addForeignCollisionObject(const short &foreignId, btCollisionObject *collisionObject);

    /*!
     * \brief Removes a btCollisionObject from the underlying btCollisionWorld. See official Bullet documentation.
     * \param collisionObject please refer to the Bullet documentation
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void removeCollisionObject(btCollisionObject *collisionObject);

    /*!
     * \brief Returns the broadphase interface used by this world.
     * \return the pointer to the btLocalGridBroadphase used in this world
     */
    sodaLocalGridBroadphase *getBroadphase()
    {
        return broadphase;
    }

    /*!
     * \brief Updates AABBs of all bodies. See official Bullet documentation.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void updateAabbs();

    /*!
     * \brief Performs one pass of discrete collision detection. See official Bullet documentation.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void performDiscreteCollisionDetection();

    /*!
     * \brief Builds simulation islands taking into account foreign colliding bodies.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void calculateSimulationIslands();

    /*!
     * \brief Solves constraints on current simulation islands.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void solveConstraints(btContactSolverInfo &solverInfo);

    /*!
     * \brief Retrieves the simulation island manager of this world.
     * \return a pointer to the sodaSimulationIslandManager of this world
     */
    sodaSimulationIslandManager *getSimulationIslandManager()
    {
        return m_islandManager;
    }

    /*!
     * \brief Retrieves the simulation island manager of this world.
     * \return a constant pointer to the sodaSimulationIslandManager of this world
     */
    const sodaSimulationIslandManager *getSimulationIslandManager() const
    {
        return m_islandManager;
    }

    /*!
     * \brief Saves the kinematic state of something (??). See official Bullet documentation.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void saveKinematicState(btScalar timeStep);

    /*!
     * \brief Synchronizes motion states (??). See official Bullet documentation.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void synchronizeMotionStates();

    /*!
     * \brief Serializes rigid bodies (??). See official Bullet documentation.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in PEPSI's. Modify at your own
     * expenses.
     */
    void serializeRigidBodies(btSerializer *serializer);

private:
    /*!
     * \brief Internal function that adds a foreign entity and the local Cell where it exists to a map more convenient for collision detection.
     * \param foreigner the foreign sodaDynamicEntity
     * \param coord the coordinates of the Cell where this foreign entity is located
     * \param foreignersPerCellMap the map to fill
     */
    void _addToForeignerMap(sodaDynamicEntity const * const foreigner, const CellBorderCoordinates &coord, QMap<btVector3, QSet<sodaDynamicEntity const *> > &foreignersPerCellMap);

    sodaContactResultCallback         cb;                             /*!< Callback for custom collision checks with foreign objects */
    sodaLogicWorld                      *logicWorld;                    /*!< The World's application logic in SODA */
    sodaLocalGridBroadphase           *broadphase;                    /*!< The broad-phase collision detection algorithm interface */
    sodaSimulationIslandManager       *m_islandManager;               /*!< The SimulationIsland manager of this object */
    EntityOverlappedCellsPerWorld     borderTraversedNeighbors;       /*!< List, for each neighbor, of border overlapping entities from this world */
    QHash<const btCollisionObject *, short> m_objectOwnerIdCache;     /*!< Array of owner Ids for all btCollisionObjects in the btCollisionWorld */
    bool                              aborted;                        /*!< Whether the running of the Simulation was aborted in the parent sodaLogicWorld */
};












#endif // SODADYNAMICSWORLD_H
