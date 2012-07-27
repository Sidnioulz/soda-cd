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

// Forward declarations
class Simulation;
class PhysicsWorldAsyncEventLoop;
class PhysicsWorldThread;
class PhysicsWorldWorker;
class PhysicsWorld;
// Needed for metatypes because of compiler parser restrictions on damn C++ macros!
typedef QMap<obEntityWrapper *, QVector<CellBorderCoordinates> > EntityOverlappedCellsMap;

//! An obEntityWrapper instance associated with a time unit that represents a temporal event.
typedef QPair<obEntityWrapper *, btScalar> TimedEntity;

//! Queue of TimedEntity instances.
typedef QQueue<TimedEntity > TimedEntityQueue;

/*! \class PhysicsWorldAsyncEventLoop
  * \brief An event loop with a list of slots for each event it can manage.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an event loop that has a list of available slots. Each of these slots
  * is run specifically on that loop, when processEvents() is called.
  */
class PhysicsWorldAsyncEventLoop : public QEventLoop
{
    Q_OBJECT

public:
    explicit PhysicsWorldAsyncEventLoop(PhysicsWorld *parent);

public slots:
    void onSynchronizationReady(const short &senderId, const QList<short> &neighbors, const btScalar &simulatedTime);

private:
    PhysicsWorld    *parent;        /*!< Pointer to the parent PhysicsWorld that owns this object */
};


/*! \class PhysicsWorldThread
  * \brief A thread with a public event loop starting method.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a QThread with a running event loop.
  */
class PhysicsWorldThread : public QThread
{
    Q_OBJECT

    friend class PhysicsWorld;

public:
    /*!
      * \brief Default constructor.
      * \param parent the parent PhysicsWorld of this thread
      * \return a new PhysicsWorldThread
      */
    explicit PhysicsWorldThread(PhysicsWorld *parent);

    /*!
      * \brief Default destructor.
      */
    ~PhysicsWorldThread();

    /*!
     * \brief Starts the thread and attaches a PhysicsWorldWorker to it.
     * \param worker the PhysicsWorldWorker that will run in this PhysicsWorldThread
     * \param loop the asynchronous event loop used by the worker
     */
    void startAndAttachWorker(PhysicsWorldWorker *worker, PhysicsWorldAsyncEventLoop *loop);

    /*!
     * \brief Tells the worker to abort its jobs, and adds an exit event to the event loop.
     */
    void exitEventLoop();

public slots:
    /*!
     * \brief Run method of the thread, that just runs an event loop.
     */
    void run();

private slots:
    /*!
     * \brief Actually exits the event loop, should be called by the last event of the loop.
     * \param code return code of the exit function
     */
    void _exitEventLoop(int code);

    /*!
     * \brief Informs that the thread is ready to run, and allows leaving the startAndAttachWorker() method.
     */
    void setReadyStatus();

signals:
    /*!
     * \brief A signal to which the PhysicsWorldThread's shutdown helper and the PhysicsWorldWorker are connected.
     */
    void aboutToStop();

private:
    PhysicsWorld                    *parent;            /*!< Pointer to the parent PhysicsWorld that owns this object */
    QSignalMapper                   *shutDownHelper;    /*!< An object to help shutting the thread down properly */
    QWaitCondition                  waitCondition;      /*!< A wait condition that makes sure the startAndAttachWorker() method leaves only when the thread is actually started */
    QMutex                          mutex;              /*!< A mutex that makes sure the startAndAttachWorker() method leaves only when the thread is actually started */
};









/*! \class PhysicsWorldWorker
  * \brief A worker that contains methods executed in a PhysicsWorldThread.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class contains utilities that are run within a PhysicsWorldThread.
  * These methods must respect the obligation to shut down when a given flag is set
  * in the object, and they all run sequentially within the same thread.
  *
  * When a method from PhysicsWorldWorker contains a blocking call, it should be
  * possible to unblock this call from a parent PhysicsWorld object running in
  * the main thread. See CircularTransformBuffer for an example.
  */
class PhysicsWorldWorker: public QObject
{
    Q_OBJECT

public:
    /*!
      * \brief Default constructor.
      * \param parent the PhysicsWorld this worker operates on
      * \return a new PhysicsWorldWorker
      */
    explicit PhysicsWorldWorker(PhysicsWorld *parent);

    /*!
      * \brief Default destructor.
      */
    ~PhysicsWorldWorker();

public slots:
    /*!
     * \brief Called when the PhysicsWorldThread will shutdown soon, raises a shutdown flag within the PhysicsWorldWorker.
     */
    void onThreadStopping();

    /*!
     * \brief Runs one pass of simulation, and schedules another one in the event loop if the shutdown flag isn't raised.
     */
    void runOnePass();

    /*!
     * \brief Called when a neighbor's entity overlaps the border between that neighbor and the parent PhysicsWorld.
     * \param neighbor the neighbor whose object overlaps
     * \param objects the overlapping objects and the lists of cells they overlap
     * \param time the timestamp at which the intrusion occurred (from neighbor's clock)
     */
    void onBorderTraversed(const PhysicsWorld *&neighbor, const EntityOverlappedCellsMap &objects, const btScalar &time);

    /*!
     * \brief Called when an obEntityWrapper's ownership is transfered to this PhysicsWorldWorker's parent.
     * \param neighbor the PhysicsWorld that gave the obEntityWrapper
     * \param object the givenobEntityWrapper
     * \param time the timestamp from which the obEntityWrapper should be included in the physics engine
     */
    void onOwnershipTransfer(const PhysicsWorld *&neighbor, const obEntityWrapper *&object, const btScalar &time);

private:
    PhysicsWorld    *parent;        /*!< Pointer to the parent PhysicsWorld that owns this object */
    bool            _shuttingDown;  /*!< Flag indicating whether the worker should shut down or keep running passes */
};
























/*! \class PhysicsWorld
  * \brief The class that manages a Physics rendering engine for a subset of the Simulation.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is responsible of the physics simulation of a subset of the total Simulation.
  * It contains a BulletManager, a LocalGrid and is the interface for management of entities.
  */
class PhysicsWorld : public QObject
{
    Q_OBJECT

    friend class PhysicsWorldAsyncEventLoop;
    friend class PhysicsWorldWorker;
    friend class BulletManagerWorld;

public:
    /*!
      * \brief Default constructor.
      * \param simulation the Simulation this world belongs to
      * \param targetTimeStep the duration of a simulation time step
      * \return a new PhysicsWorld
      */
    explicit PhysicsWorld(Simulation &simulation, const btScalar &targetTimeStep);

	/*!
	  * \brief Default destructor.
	  */
	virtual ~PhysicsWorld();

    /*!
      * \brief Starts the simulation thread.
      */
    void startSimulation();

    /*!
      * \brief Stops the simulation thread.
      */
    void stopSimulation();

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
      *
      * \deprecated Please use _addEntity() directly whenever possible.
      */
    void addEntity(obEntityWrapper *obEnt, btScalar targetTime);

    /*!
      * \brief Enqueues an entity so that it is removed at a specific time in the simulation.
      * \param obEnt the entity to remove
      * \param targetTime the time at which the entity should stop existing
	  *
	  * \see _removeEntity();
      *
      * \deprecated Please use _removeEntity() directly whenever possible.
      */
    void removeEntity(obEntityWrapper *obEnt, btScalar targetTime);

    /*!
     * \brief Returns a read-only reference to this PhysicsWorld's entities.
     * \return a constant reference to the vector containing the dynamic entities of the world
     */
    inline const QVector<obEntityWrapper*> &getEntities() const
    {
        return entities;
    }

    /*!
     * \brief Returns a read-write reference to this PhysicsWorld's entities.
     * \param int dummy parameter to allow non-const references
     * \return a constant reference to the vector containing the dynamic entities of the world
     */
    inline QVector<obEntityWrapper*> &getEntities(int)
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
      * \brief Returns the LocalGrid of this PhysicsWorld.
      * \return a pointer to this world's LocalGrid
      */
    inline const btScalar &getCurrentTime() const
    {
        return currentTime;
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

    //TODO: document getNeighbor
    PhysicsWorld *getNeighbor(const short neighborId) const;

    //TODO: document messageNeighbor
    bool messageNeighbor(PhysicsWorld *neighbor, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    //TODO: document messageNeighbor
    bool messageNeighbor(const short neighborId, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    //TODO: document asyncMessageNeighbor
    bool asyncMessageNeighbor(PhysicsWorld *neighbor, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    //TODO: document asyncMessageNeighbor
    bool asyncMessageNeighbor(const short neighborId, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    static const short NullWorldId;               //!< a value used when a world ID is needed but there is no corresponding world instance
    static const short UnknownWorldId;            //!< a value used when the owner world of an object is not known yet
    static const short IdBeingProcessed;          //!< a value used when the owner world of an object is being computed

    static const int   NbColors = 12;             //!< number of available colors for the per-thread coloring of entities
    static const int   EntityColors[NbColors][3]; //!< table containing color codes used to distinguish entity thread owners


protected:

    /*!
     * \typedef MessageType
     * \brief Different types of incoming IPC messages.
     */
    typedef enum __messageType {
        SyncReady=0,
        BorderTraversed=1
    } MessageType;

    /*!
     * \struct IncomingMessage
     * \brief The IncomingMessage struct contains an incoming message for IPC during a simulation step pass.
     * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
     */
    struct IncomingMessage {
        short senderId;
        MessageType messageType;
        QVariant data;
    };


    void _waitForNeighbors(const QList<short> &neighbors, const btScalar &simulatedTime);

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

    /*!
     * \brief Callback called whenever a CD pass is done by Bullet. Copies object transforms to the buffer.
     * \param world pointer to the Bullet world
     * \param timeStep length of the time step being simulated
     */
    static void _tickCallback(btDynamicsWorld *world, btScalar timeStep);

    Simulation                &simulation;               /*!< the Simulation this world belongs to */

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

    PhysicsWorldAsyncEventLoop *incomingLoop;            /*!< An event loop for incoming messages that must be received during processing of a pass */
    PhysicsWorldThread        worldThread;               /*!< The thread in which CD passes run, and in which messages are received */
    PhysicsWorldWorker        *worker;                   /*!< The worker object that contains the CD passes code and that is run in worldThread */
    QTimer                    timer;                     /*!< A timer used to spam runOnePass() events in the event loop */
    QMap<btScalar, QList<IncomingMessage> > incomingQueue; /*!< A map with incoming messages for different time steps */

    static short              WorldIdCounter;            //!< a counter to make sure world IDs are unique
};

#endif // PHYSICSWORLD_H
