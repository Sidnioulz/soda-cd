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
#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreString.h>
#include "bulletmanager.h"
#include "circulartransformbuffer.h"
#include "localgrid.h"
#include "obRigidBody.h"
#include "obEntityWrapper.h"

//! An obEntityWrapper instance associated with a time unit that represents a temporal event.
typedef QPair<obEntityWrapper *, btScalar> TimedEntity;

//! Queue of TimedEntity instances.
typedef QQueue<TimedEntity > TimedEntityQueue;

/*! \class PhysicsWorld
  * \brief The class that manages a Physics rendering engine for a subset of the Simulation.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is responsible of the physics simulation of a subset of the total Simulation.
  * It contains a BulletManager, a LocalGrid and is the interface for management of entities.
  */
class PhysicsWorld
{
public:
    /*!
      * \brief Default constructor.
      * \param targetTimeStep the duration of a simulation time step
      * \return a new PhysicsWorld
      */
    PhysicsWorld(const btScalar &targetTimeStep);

    /*!
      * \brief Default destructor.
      */
    virtual ~PhysicsWorld();

    /*!
      * \brief Starts the collision detection thread.
      */
    void startSimulation();

    /*!
      * \brief Returns the manager that manages the Bullet physics engine.
      * \return the BulletManager of this world
      */
    BulletManager* getBulletManager() const;

    /*!
      * \brief Returns the id of this world.
      * \return the id of this world
      */
    int getId() const;

    /*!
      * \brief Returns the circular buffer containing Bullet computed transforms.
      * \return the CircularTransformBuffer of this world
      */
    CircularTransformBuffer* getCircularBuffer() const;

    /*!
      * \brief Enqueues an entity so that it is added at a specific time in the simulation.
      * \param obEnt the entity to add
      * \param targetTime the time at which the entity should start existing
	  *
	  * \see _addEntity();
      */
    void addEntity(obEntityWrapper *obEnt, btScalar targetTime);

    /*!
      * \brief Enqueues an entity so that it is removed at a specific time in the simulation.
      * \param obEnt the entity to remove
      * \param targetTime the time at which the entity should stop existing
	  *
	  * \see _removeEntity();
      */
    void removeEntity(obEntityWrapper *obEnt, btScalar targetTime);

    /*!
     * \brief Returns a reference to this PhysicsWorld's entities.
     * \return a constant reference to the vector containing the dynamic entities of the world
     */
    inline const QVector<obEntityWrapper*> &getEntities() const
    {
        return entities;
    }

    /*!
     * \brief Returns a reference to this PhysicsWorld's static btRigidBody objects.
     * \return a constant reference to the vector containing the static entities of the world
     */
    inline const QVector<btRigidBody *> &getStaticEntities() const
    {
        return globalStaticEntities;
    }

    /*!
     * \brief Draws cubes inside all owned Cells of the Grid.
     */
    void drawCells();

    /*!
     * \brief Creates all the CellBorderEntity objects needed for this PhysicsWorld's LocalGrid.
     */
    void setupLocalGridBorders();

	/*!
	  * \brief Returns the LocalGrid of this PhysicsWorld.
	  * \return a pointer to this world's LocalGrid
	  */
	inline LocalGrid *getLocalGrid() const
	{
		return localGrid;
	}

    /*!
      * \brief Initializes the content of the PhysicsWorld scene.
      */
    void createScene();

    /*!
      * \brief Assigns a LocalGrid to this PhysicsWorld.
      * \param local the LocalGrid to assign to this PhysicsWorld
      *
      * When this function is called, the PhysicsWorld automatically
      * adds all entities from the LocalGrid into its own entities
      * vector, and updates its local grid pointer.
      */
    void assignLocalGrid(LocalGrid *local);

    static const short NullWorldId;         //!< a value used when a world ID is needed but there is no corresponding world instance
    static const short UnknownWorldId;      //!< a value used when the owner world of an object is not known yet
    static const short IdBeingProcessed;    //!< a value used when the owner world of an object is being computed


private:
    /*!
      * \brief Adds an entity wrapper to this physics world.
      * \param obEnt the entity to add
      */
    void _addEntity(obEntityWrapper *obEnt);

    /*!
      * \brief Adds an Cell border entity to this physics world.
      * \param cbEnt the Cell border to add
      */
    void _addCellBorder(CellBorderEntity *cbEnt);

    /*!
     * \brief Removes from an entity container anything that matches a given entity's name.
     * \param container the container to remove an obEntityWrapper from
     * \param obEnt the obEntityWrapper to remove
     *
     * \note There is apparently no reason why this method couldn't be refactored to
     * directly use an Ogre::String as a second parameter if it makes things easier.
     */
    void _entityVectoryRemovalMethod(QVector<obEntityWrapper *> &container, obEntityWrapper *obEnt);

	/*!
      * \brief Removes an entity wrapper from this physics world.
	  * \param obEnt the entity to remove
      */
    void _removeEntity(obEntityWrapper *obEnt);


    static const int          NbColors = 12;             //!< number of available colors for the per-thread coloring of entities
    static const int          EntityColors[NbColors][3]; //!< table containing color codes used to distinguish entity thread owners

    short                     id;                        //!< a number associated only to this object and used for entity naming
    btScalar                  targetTimeStep;            //!< the target time step of the application
    QVector<obEntityWrapper*> entities;                  //!< a vector for the existing rigid bodies
    QVector<btRigidBody *>    globalStaticEntities;      //!< a vector to easily manage environment static entities like the floor
    CircularTransformBuffer   *buffer;                   //!< pointer to the buffer on which the Bullet engine writes object positions
    btScalar                  currentTime;               //!< current time of the physics simulation
    LocalGrid                 *localGrid;                //!< pointer to the local grid containing the entities of this PhysicsWorld
    BulletManager             *bulletManager;            //!< manager of the bullet physics engine

    QMutex                    entityMutex;               //!< a mutex for inserting and deleting entities between CD iterations
    TimedEntityQueue          entityAdditionQueue;       //!< a queue for objects to be added between next iterations of the collision detection algorithm
    TimedEntityQueue          entityRemovalQueue;        //!< a queue for objects to be removed between next iterations of the collision detection algorithm

    static short              WorldIdCounter;            //!< a counter to make sure world IDs are unique

	/*! \class BulletFakeCCDThread
	  * \brief A thread that is responsible of collision detection.
	  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
	  *
	  * This class is a QThread that launches collision detection passes continuously
	  * until stopped, and writes its output in the CircularTransformBuffer associated
	  * with its parent class.
	  */
	class BulletFakeCCDThread : public QThread {
	public:
		/*!
		  * \brief Default constructor.
		  * \param world the PhysicsWorld for which collision detection is being done.
		  * \return a new BulletFakeCCDThread
		  */
		BulletFakeCCDThread(PhysicsWorld *world);

		/*!
		  * \brief Callback called whenever a CD pass is done by Bullet. Copies object positions to the buffer.
		  * \param world pointer to the Bullet world
		  * \param timeStep length of the time step being simulated
		  */
		static void myTickCallback(btDynamicsWorld *world, btScalar timeStep);

		/*!
		  * \brief Initializes the thread and sets the CD pass callback.
		  */
		void init();

		/*!
		  * \brief Runs the infinite CD loop.
		  *
		  * \todo TODO makes sense to interrupt current CD iteration if new objects inserted, in order to avoid wasting computing time.
		  */
		void run();

	private:
        PhysicsWorld *world;  /**< The PhysicsWorld to which this thread belongs */

    } CDInterface;       //!< the thread in which collision detection is performed

};

#endif // PHYSICSWORLD_H
