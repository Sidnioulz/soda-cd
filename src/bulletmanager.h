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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <btBulletDynamicsCommon.h>
#include "btlocalgridbroadphase.h"
#include "localgrid.h"

/*! \class BulletManagerWorld
  * \brief An override of a Bullet dynamics world that can synchronize with another instance.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements a Bullet dynamics world with a stepSimulation
  * function that can synchronize with another instance (specifically, the
  * physics response function).
  */
class BulletManagerWorld : public btDiscreteDynamicsWorld
{
public:
    /*!
      * \brief Default constructor.
      * \param dispatcher the Bullet collision dispatcher
      * \param broadphase the broadphase algorithm
      * \param solver the physics constraint solver
      * \param config the collision configuration
      * \return a new BulletManager
      */
    BulletManagerWorld(btCollisionDispatcher*&, btLocalGridBroadphase*&, btSequentialImpulseConstraintSolver*&, btDefaultCollisionConfiguration*&);

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
     * \param collisionObject please refer to the Bullet documentation
     * \param collisionFilterGroup please refer to the Bullet documentation
     * \param collisionFilterMask please refer to the Bullet documentation
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in SODA CD. Modify at your own
     * expenses.
     */
    void addCollisionObject(btCollisionObject *collisionObject, short int collisionFilterGroup, short int collisionFilterMask);

    /*!
     * \brief Removes a btCollisionObject from the underlying btCollisionWorld. See official Bullet documentation.
     * \param collisionObject please refer to the Bullet documentation
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in SODA CD. Modify at your own
     * expenses.
     */
    void removeCollisionObject(btCollisionObject *collisionObject);

    /*!
     * \brief Returns the broadphase interface used by this world.
     * \return the pointer to the btLocalGridBroadphase used in this world
     */
    btLocalGridBroadphase *getBroadphase()
    {
        return broadphase;
    }

//    /*!
//     * \brief Adds a btRigidBody to the world. See official Bullet documentation.
//     * \param body the btRigidBody to addignored
//     *
//     * \note This function is an override of the original one from Bullet, it has
//     * been written so as to make a special use of  collision filter groups.
//     */
//    void addRigidBody(btRigidBody *body);

//    /*!
//     * \brief Adds a btRigidBody to the world, ignoring the user-set collision filter.
//     * \param body the btRigidBody to add
//     * \param group a CollisionFilterGroup parameter that will be ignored
//     * \param mask a CollisionFilterMask parameter that will be ignored
//     *
//     * \note This function is an override of the original one from Bullet, it has
//     * been written so as to disable collision filter groups that are used for
//     * other purposes in SODA CD.
//     */

//    void addRigidBody(btRigidBody* body, short group, short mask);


    /*!
     * \brief Updates AABBs of all bodies. See official Bullet documentation.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in SODA CD. Modify at your own
     * expenses.
     */
    void updateAabbs();

    /*!
     * \brief Performs one pass of discrete collision detection. See official Bullet documentation.
     *
     * \note The Bullet documentation does not tell what this function does,
     * and thus it is not properly documented in SODA CD. Modify at your own
     * expenses.
     */
    void performDiscreteCollisionDetection();

private:
    btLocalGridBroadphase* broadphase;                  //!< the broadphase algorithm interface
};

/*! \class BulletManager
  * \brief A wrapper for Bullet physics library initialization and management.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements a Bullet physics engine, initializes its various components, and
  * provides an interface to step simulations and get physics answers from it.
  * It is meant to be used by a PhysicsWorld with a LocalGrid.
  */
class BulletManager
{
public:
    /*!
      * \brief Default constructor.
      * \return a new BulletManager
      */
    explicit BulletManager();

    /*!
      * \brief Default destructor.
      */
    ~BulletManager();

    /*!
     * \brief Sets a PhysicsWorld to use for the spatial subdivision broadphase.
     * \param world the world to use
     */
    inline void setBroadphaseWorld(PhysicsWorld *world)
    {
        broadphase->setWorld(world);
    }

    /*!
     * \brief Removes any previously set PhysicsWorld for the broadphase.
     */
    inline void unsetBroadphaseWorld()
    {
        broadphase->unsetWorld();
    }

    /*!
      * \brief Returns the dynamics world of the bullet instance.
      * \return the btDynamicsWorld of this instance
      */
    inline btDynamicsWorld* getDynamicsWorld() const
    {
        return dynamicsWorld;
    }

private:
    btLocalGridBroadphase* broadphase;                  //!< the broadphase algorithm interface
    btDefaultCollisionConfiguration* collisionConfig;   //!< the configuration of the engine
    btCollisionDispatcher* dispatcher;                  //!< the exact detection algorithm
    btSequentialImpulseConstraintSolver* solver;        //!< the physics response solver
    BulletManagerWorld* dynamicsWorld;                  //!< the bullet representation of the world
};

#endif // MAINWINDOW_H
