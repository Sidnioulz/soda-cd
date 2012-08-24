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
#ifndef sodaLogicWorld_H
#define sodaLogicWorld_H

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreString.h>
#include "circulartransformbuffer.h"
#include "sodaLocalGrid.h"
#include "sodaRigidBody.h"
#include "sodaDynamicEntity.h"
#include "typedefs.h"

// Forward declarations
class BulletManager;
class sodaDynamicsWorld;
class Simulation;
class sodaLogicWorldAsyncEventLoop;
class sodaLogicWorldThread;
class sodaLogicWorldWorker;
class sodaLogicWorld;

/*! \class sodaLogicWorldAsyncEventLoop
  * \brief An event loop with a list of slots for each event it can manage.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an event loop that has a list of available slots. Each of these slots
  * is run specifically on that loop, when processEvents() is called.
  */
class sodaLogicWorldAsyncEventLoop : public QEventLoop
{
    Q_OBJECT

public:
    /*!
      * \brief Default constructor.
      * \param parent the parent sodaLogicWorld of this thread
      * \return a new sodaLogicWorldAsyncEventLoop
      */
    explicit sodaLogicWorldAsyncEventLoop(sodaLogicWorld *parent);

public slots:
    /*!
     * \brief Method called when a neighbor world informs this object that it is ready for synchronization.
     * \param senderId the Id of the ready neighbor
     * \param neighbors the list of cells containing foreign entities in the neighbor's world
     * \param simulTime the timestamp of the simulation step corresponding to this synchronization
     */
    void onSynchronizationReady(const short &senderId, const EntityOverlappedCellsPerWorld &neighbors, const btScalar &simulTime);

    /*!
     * \brief Method called when a neighbor world informs that it is ready to synchronize end of simulation steps.
     * \param senderId the Id of the ready neighbor
     * \param neighbors the list of other synchronized neighbors (for debug)
     * \param simulTime the timestamp of the simulation step corresponding to this synchronization
     */
    void onStepEnd(const short &senderId, const QList<short> &neighbors, const btScalar &simulTime);

    /*!
     * \brief Method called when the Simulation's run is being aborted (to avoid blocking waiting for a message).
     */
    void onAbortRun();

private:
    sodaLogicWorld    *parent;        /*!< Pointer to the parent sodaLogicWorld that owns this object */
};


/*! \class sodaLogicWorldThread
  * \brief A thread with a public event loop starting method.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a QThread with a running event loop.
  */
class sodaLogicWorldThread : public QThread
{
    Q_OBJECT

    friend class sodaLogicWorld;

public:
    /*!
      * \brief Default constructor.
      * \param parent the parent sodaLogicWorld of this thread
      * \return a new sodaLogicWorldThread
      */
    explicit sodaLogicWorldThread(sodaLogicWorld *parent);

    /*!
      * \brief Default destructor.
      */
    ~sodaLogicWorldThread();

    /*!
     * \brief Starts the thread and attaches a sodaLogicWorldWorker to it.
     * \param worker the sodaLogicWorldWorker that will run in this sodaLogicWorldThread
     * \param loop the asynchronous event loop used by the worker
     */
    void startAndAttachWorker(sodaLogicWorldWorker *worker, sodaLogicWorldAsyncEventLoop *loop);

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
     * \brief A signal to which the sodaLogicWorldThread's shutdown helper and the sodaLogicWorldWorker are connected.
     */
    void aboutToStop();

private:
    sodaLogicWorld                    *parent;            /*!< Pointer to the parent sodaLogicWorld that owns this object */
    QSignalMapper                   *shutDownHelper;    /*!< An object to help shutting the thread down properly */
    QWaitCondition                  waitCondition;      /*!< A wait condition that makes sure the startAndAttachWorker() method leaves only when the thread is actually started */
    QMutex                          mutex;              /*!< A mutex that makes sure the startAndAttachWorker() method leaves only when the thread is actually started */
};









/*! \class sodaLogicWorldWorker
  * \brief A worker that contains methods executed in a sodaLogicWorldThread.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class contains utilities that are run within a sodaLogicWorldThread.
  * These methods must respect the obligation to shut down when a given flag is set
  * in the object, and they all run sequentially within the same thread.
  *
  * When a method from sodaLogicWorldWorker contains a blocking call, it should be
  * possible to unblock this call from a parent sodaLogicWorld object running in
  * the main thread. See CircularTransformBuffer for an example.
  */
class sodaLogicWorldWorker: public QObject
{
    Q_OBJECT

    friend class sodaLogicWorld;

public:
    /*!
      * \brief Default constructor.
      * \param parent the sodaLogicWorld this worker operates on
      * \return a new sodaLogicWorldWorker
      */
    explicit sodaLogicWorldWorker(sodaLogicWorld *parent);

    /*!
      * \brief Default destructor.
      */
    ~sodaLogicWorldWorker();

public slots:
    /*!
     * \brief Called when the sodaLogicWorldThread will shutdown soon, raises a shutdown flag within the sodaLogicWorldWorker.
     */
    void onThreadStopping();

    /*!
     * \brief Runs one pass of simulation, and schedules another one in the event loop if the shutdown flag isn't raised.
     */
    void runOnePass();

    /*!
     * \brief Called when a neighbor's entity overlaps the border between that neighbor and the parent sodaLogicWorld.
     * \param neighbor the neighbor whose object overlaps
     * \param objects the overlapping objects and the lists of cells they overlap
     * \param time the timestamp at which the intrusion occurred (from neighbor's clock)
     */
    void onBorderTraversed(const sodaLogicWorld *&neighbor, const EntityOverlappedCellsMap &objects, const btScalar &time);

    /*!
     * \brief Called when a sodaDynamicEntity's ownership is transfered to this sodaLogicWorldWorker's parent.
     * \param neighbor the sodaLogicWorld that gave the sodaDynamicEntity
     * \param object the given sodaDynamicEntity
     * \param time the timestamp from which the sodaDynamicEntity should be included in the physics engine
     */
    void onOwnershipTransfer(const sodaLogicWorld *&neighbor, const sodaDynamicEntity *&object, const btScalar &time);

protected:
    sodaLogicWorld    *parent;        /*!< Pointer to the parent sodaLogicWorld that owns this object */
    bool            _shuttingDown;  /*!< Flag indicating whether the worker should shut down or keep running passes */
};
























/*! \class sodaLogicWorld
  * \brief The class that manages a Physics rendering engine for a subset of the Simulation.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is responsible of the physics simulation of a subset of the total Simulation.
  * It contains a BulletManager, a sodaLocalGrid and is the interface for management of entities.
  */
class sodaLogicWorld : public QObject
{
    Q_OBJECT

    friend class sodaLogicWorldAsyncEventLoop;
    friend class sodaLogicWorldWorker;
    friend class sodaDynamicsWorld;

public:

    /*!
      * \brief Default constructor.
      * \param simulation the Simulation this world belongs to
      * \param targetTimeStep the duration of a simulation time step
      * \return a new sodaLogicWorld
      */
    explicit sodaLogicWorld(Simulation &simulation, const btScalar &targetTimeStep);

	/*!
	  * \brief Default destructor.
	  */
    virtual ~sodaLogicWorld();

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
    void addEntity(sodaDynamicEntity *obEnt, btScalar targetTime);

    /*!
      * \brief Enqueues an entity so that it is removed at a specific time in the simulation.
      * \param obEnt the entity to remove
      * \param targetTime the time at which the entity should stop existing
	  *
	  * \see _removeEntity();
      *
      * \deprecated Please use _removeEntity() directly whenever possible.
      */
    void removeEntity(sodaDynamicEntity *obEnt, btScalar targetTime);

    /*!
     * \brief Returns a read-only reference to this sodaLogicWorld's entities.
     * \return a constant reference to the vector containing the dynamic entities of the world
     */
    inline const QVector<sodaDynamicEntity*> &getEntities() const
    {
        return entities;
    }

    /*!
     * \brief Returns a read-write reference to this sodaLogicWorld's entities.
     * \param dummy dummy parameter to allow non-const references
     * \return a constant reference to the vector containing the dynamic entities of the world
     */
    inline QVector<sodaDynamicEntity*> &getEntities(int dummy)
    {
        return entities;
    }

    /*!
     * \brief Returns a reference to this sodaLogicWorld's static btRigidBody objects.
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
     * \brief Creates all the CellBorderEntity objects needed for this sodaLogicWorld's sodaLocalGrid.
     */
    void setupLocalGridBorders();

    /*!
      * \brief Returns the sodaLocalGrid of this sodaLogicWorld.
      * \return a pointer to this world's sodaLocalGrid
      */
    inline sodaLocalGrid *getLocalGrid() const
    {
        return localGrid;
    }

    /*!
      * \brief Returns the sodaLocalGrid of this sodaLogicWorld.
      * \return a pointer to this world's sodaLocalGrid
      */
    inline const btScalar &getCurrentTime() const
    {
        return currentTime;
    }

    /*!
      * \brief Initializes the content of the sodaLogicWorld scene.
      */
    void createScene();

    /*!
      * \brief Assigns a sodaLocalGrid to this sodaLogicWorld.
      * \param local the sodaLocalGrid to assign to this sodaLogicWorld
      *
      * When this function is called, the sodaLogicWorld automatically
      * adds all entities from the sodaLocalGrid into its own entities
      * vector, and updates its local grid pointer.
      */
    void assignLocalGrid(sodaLocalGrid *local);

    /*!
     * \brief Retrieves a pointer to a neighbor given its Id.
     * \param neighborId the Id of the wanted neighbor
     * \return a pointer to the sodaLogicWorld whose Id was given, or null if it couldn't be found
     *
     * \warning This function implies that all worlds have access to all other worlds'
     * memory address. May not always be nicely scalable in a message-passing implementation
     * with huge numbers of small-sized worlds.
     */
    sodaLogicWorld *getNeighbor(const short neighborId) const;

    /*!
     * \brief Sends a message to a neighbor sodaLogicWorld using a pointer to it.
     * \param neighbor the neighbor that will receive the message
     * \param method the method called to process the message - must be a slot from sodaLogicWorldWorker
     * \param val0 first parameter for method
     * \param val1 second parameter for method
     * \param val2 third parameter for method
     * \param val3 forth parameter for method
     * \param val4 fifth parameter for method
     * \param val5 sixth parameter for method
     * \param val6 seventh parameter for method
     * \param val7 eighth parameter for method
     * \param val8 ninth parameter for method
     * \param val9 last parameter for method
     * \return weither the message could be sent
     *
     * \note There are no guarantees that the message will be processed or even received.
     */
    bool messageNeighbor(sodaLogicWorld *neighbor, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    /*!
     * \brief Sends a message to a neighbor sodaLogicWorld using its Id.
     * \param neighborId Id of the neighbor to whom the message must be sent
     * \param method the method called to process the message - must be a slot from sodaLogicWorldWorker
     * \param val0 first parameter for method
     * \param val1 second parameter for method
     * \param val2 third parameter for method
     * \param val3 forth parameter for method
     * \param val4 fifth parameter for method
     * \param val5 sixth parameter for method
     * \param val6 seventh parameter for method
     * \param val7 eighth parameter for method
     * \param val8 ninth parameter for method
     * \param val9 last parameter for method
     * \return weither the message could be sent
     *
     * \note There are no guarantees that the message will be processed or even received.
     */
    bool messageNeighbor(const short neighborId, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    /*!
     * \brief Sends an asynchronous message to a neighbor sodaLogicWorld using a pointer to it.
     * \param neighbor the neighbor that will receive the message
     * \param method the method called to process the message - must be a slot from sodaLogicWorldAsyncEventLoop
     * \param val0 first parameter for method
     * \param val1 second parameter for method
     * \param val2 third parameter for method
     * \param val3 forth parameter for method
     * \param val4 fifth parameter for method
     * \param val5 sixth parameter for method
     * \param val6 seventh parameter for method
     * \param val7 eighth parameter for method
     * \param val8 ninth parameter for method
     * \param val9 last parameter for method
     * \return weither the message could be sent
     *
     * \note There are no guarantees that the message will be processed or even received.
     */
    bool asyncMessageNeighbor(sodaLogicWorld *neighbor, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    /*!
     * \brief Sends an asynchronous message to a neighbor sodaLogicWorld using its Id.
     * \param neighborId Id of the neighbor to whom the message must be sent
     * \param method the method called to process the message - must be a slot from sodaLogicWorldAsyncEventLoop
     * \param val0 first parameter for method
     * \param val1 second parameter for method
     * \param val2 third parameter for method
     * \param val3 forth parameter for method
     * \param val4 fifth parameter for method
     * \param val5 sixth parameter for method
     * \param val6 seventh parameter for method
     * \param val7 eighth parameter for method
     * \param val8 ninth parameter for method
     * \param val9 last parameter for method
     * \return weither the message could be sent
     *
     * \note There are no guarantees that the message will be processed or even received.
     */
    bool asyncMessageNeighbor(const short neighborId, const char *method, QGenericArgument val0 = QGenericArgument(0), QGenericArgument val1 = QGenericArgument(), QGenericArgument val2 = QGenericArgument(), QGenericArgument val3 = QGenericArgument(), QGenericArgument val4 = QGenericArgument(), QGenericArgument val5 = QGenericArgument(), QGenericArgument val6 = QGenericArgument(), QGenericArgument val7 = QGenericArgument(), QGenericArgument val8 = QGenericArgument(), QGenericArgument val9 = QGenericArgument()) const;

    static const short NullWorldId;               //!< a value used when a world ID is needed but there is no corresponding world instance
    static const short UnknownWorldId;            //!< a value used when the owner world of an object is not known yet
    static const short IdBeingProcessed;          //!< a value used when the owner world of an object is being computed

    static const int   NbColors = 12;             //!< number of available colors for the per-thread coloring of entities
    static const int   EntityColors[NbColors][3]; //!< table containing color codes used to distinguish entity thread owners


protected:
    //! \brief Different types of incoming IPC messages.
    typedef enum __messageType {
        SyncReady=0,
        BorderTraversed=1,
        FrameEndSync=2,
        AbortRun=3
    } MessageType;

    /*!
     * \struct IncomingMessage
     * \brief The IncomingMessage struct contains an incoming message for IPC during a simulation step pass.
     * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
     */
    struct IncomingMessage {
        short senderId;             /*!< The Id of the sodaLogicWorld sending the message */
        MessageType messageType;    /*!< The type of message sent */
        QVariant data;              /*!< Data associated to the message (depends upon type, see source code for examples */
    };

    /*!
     * \brief Appends a message to the queue of incoming messages of the sodaLogicWorld, as a response to IPC method calls.
     * \param simulTime the time at which the event is to be considered
     * \param msg the message to append
     */
    void appendMessageToQueue(const btScalar &simulTime, const IncomingMessage &msg);

    /*!
     * \brief Reads requests from neighbor sodaLogicWorlds to synchronize at a given simulation time.
     * \param syncNeighbors the list of neighbors with associated belonging dynamic entities and cells where they overlap
     * \param simulTime the timestamp of the simulation step concerned by the request
     */
    void _readExternalSyncRequests(EntityOverlappedCellsPerWorld &syncNeighbors, const btScalar &simulTime);

    /*!
     * \brief Internal function of sodaLogicWorld to wait for a group of neighbors for synchronous collision handling.
     * \param neighbors the neighbors to wait for
     * \param simulTime the current simulation time, at which waiting is required
     * \param borderTraversedNeighbors a hashmap in which to store foreign entities per neighbor
     */
    bool _waitForNeighbors(const QList<short> &neighbors, const btScalar &simulTime, EntityOverlappedCellsPerWorld &borderTraversedNeighbors);

    /*!
     * \brief Internal function of sodaLogicWorld to wait for the end of a synchronous simulation step.
     * \param neighbors the neighbors to wait for
     * \param simulTime the current simulation time, at which waiting is required
     */
    bool _waitEndStep(const QList<short> &neighbors, const btScalar &simulTime);

private:
    /*!
      * \brief Adds an entity wrapper to this physics world.
      * \param obEnt the entity to add
      */
    void _addEntity(sodaDynamicEntity *obEnt);

    /*!
      * \brief Adds an Cell border entity to this physics world.
      * \param cbEnt the Cell border to add
      */
    void _addCellBorder(CellBorderEntity *cbEnt);

    /*!
     * \brief Removes from an entity container anything that matches a given entity's name.
     * \param container the container to remove a sodaDynamicEntity from
     * \param obEnt the sodaDynamicEntity to remove
     *
     * \note There is apparently no reason why this method couldn't be refactored to
     * directly use an Ogre::String as a second parameter if it makes things easier.
     */
    void _entityVectoryRemovalMethod(QVector<sodaDynamicEntity *> &container, sodaDynamicEntity *obEnt);

	/*!
      * \brief Removes an entity wrapper from this physics world.
	  * \param obEnt the entity to remove
      */
    void _removeEntity(sodaDynamicEntity *obEnt);

    /*!
     * \brief Callback called whenever a CD pass is done by Bullet. Copies object transforms to the buffer.
     * \param world pointer to the Bullet world
     * \param timeStep length of the time step being simulated
     */
    static void _tickCallback(btDynamicsWorld *world, btScalar timeStep);

    Simulation                &simulation;               /*!< the Simulation this world belongs to */

    short                     id;                        //!< a number associated only to this object and used for entity naming
    btScalar                  targetTimeStep;            //!< the target time step of the application
    QVector<sodaDynamicEntity*> entities;                //!< a vector for the existing rigid bodies
    QVector<btRigidBody *>    globalStaticEntities;      //!< a vector to easily manage environment static entities like the floor
    CircularTransformBuffer   *buffer;                   //!< pointer to the buffer on which the Bullet engine writes object positions
    btScalar                  currentTime;               //!< current time of the physics simulation
    sodaLocalGrid             *localGrid;                //!< pointer to the local grid containing the entities of this sodaLogicWorld
    BulletManager             *bulletManager;            //!< manager of the bullet physics engine

    QMutex                    entityMutex;               //!< a mutex for inserting and deleting entities between CD iterations
    TimedEntityQueue          entityAdditionQueue;       //!< a queue for objects to be added between next iterations of the collision detection algorithm
    TimedEntityQueue          entityRemovalQueue;        //!< a queue for objects to be removed between next iterations of the collision detection algorithm

    sodaLogicWorldAsyncEventLoop *incomingLoop;            /*!< An event loop for incoming messages that must be received during processing of a pass */
    sodaLogicWorldThread        worldThread;               /*!< The thread in which CD passes run, and in which messages are received */
    sodaLogicWorldWorker        *worker;                   /*!< The worker object that contains the CD passes code and that is run in worldThread */
    QTimer                    timer;                     /*!< A timer used to spam runOnePass() events in the event loop */
    QMap<btScalar, QList<IncomingMessage> > incomingQueue; /*!< A map with incoming messages for different time steps */
    QMutex                    incomingQueueMutex;        /*!< A useless mutex setup in prevision of future development mistakes */

    static short              WorldIdCounter;            //!< a counter to make sure world IDs are unique
};

#endif // sodaLogicWorld_H
