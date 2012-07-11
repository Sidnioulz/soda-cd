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
#include <QtDebug>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <Ogre.h>
#include <limits>

#include "utils.h"
#include "physicsworld.h"
#include "obEntityWrapper.h"
#include "ogreresources.h"
#include "cellborderentity.h"
#include "simulation.h"


PhysicsWorldThread::PhysicsWorldThread(QObject *parent) :
    QThread(parent)
{
#ifndef NDEBUG
    qWarning() << "PhysicsWorldThread()::PhysicsWorldThread " << QString().sprintf("%p", QThread::currentThread());
#endif

    // Setup an object that will queue a termination signal when we want to leave the thread
    // Better than leaving the thread directly, which could cause memory corruptions
    shutDownHelper = new QSignalMapper;
    shutDownHelper->setMapping(this, 0);
    connect(this, SIGNAL(aboutToStop()), shutDownHelper, SLOT(map()));
    connect(shutDownHelper, SIGNAL(mapped(int)), this, SLOT(_exitEventLoop()), Qt::DirectConnection);

#ifndef NDEBUG
    qWarning() << "PhysicsWorldThread()::PhysicsWorldThread constructed; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
}

void PhysicsWorldThread::startAndAttachWorker(PhysicsWorldWorker *worker)
{
#ifndef NDEBUG
    qWarning() << "PhysicsWorldThread()::launchWorker(" << worker << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    // Make sure to leave this function only upon actual thread starting
    connect(this, SIGNAL(started()), this, SLOT(setReadyStatus()), Qt::DirectConnection);

    // Start the thread
    start();

    // Make sure that the worker and the shutdown helper run their events on this thread
    worker->moveToThread(this);
    shutDownHelper->moveToThread(this);

    // Make sure the worker will be aware of the thread shutting down (so that it aborts its events left to run)
    connect(this, SIGNAL(aboutToStop()), worker, SLOT(onThreadStopping()));

    // Wait for the thread to be actually started
    mutex.lock();
    waitCondition.wait(&mutex);

#ifndef NDEBUG
    qWarning() << "PhysicsWorldThread()::launchWorker(" << worker << "); Worker launched and thread started; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
}

void PhysicsWorldThread::exitEventLoop()
{
#ifndef NDEBUG
    qWarning() << "PhysicsWorldThread()::exitEventLoop(); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    emit aboutToStop();
}

PhysicsWorldThread::~PhysicsWorldThread()
{
#ifndef NDEBUG
    qWarning() << "PhysicsWorldThread()::~PhysicsWorldThread(); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    delete shutDownHelper;
}

void PhysicsWorldThread::run()
{
    QThread::exec();
}

void PhysicsWorldThread::_exitEventLoop()
{
#ifndef NDEBUG
    qWarning() << "PhysicsWorldThread()::_exitEventLoop(); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    exit();
}

void PhysicsWorldThread::setReadyStatus()
{
    waitCondition.wakeAll();
}















PhysicsWorldWorker::PhysicsWorldWorker(PhysicsWorld *parent) :
        QObject(),
        parent(parent),
        _shuttingDown(false)
{
}

PhysicsWorldWorker::~PhysicsWorldWorker()
{
    qDebug() << "PhysicsWorldWorker(" << parent->id << ")::~PhysicsWorldWorker(); Thread " << QString().sprintf("%p", QThread::currentThread());
}

void PhysicsWorldWorker::onThreadStopping()
{
    qDebug() << "PhysicsWorldWorker(" << parent->id << ")::onThreadStopping(); Shutdown flag now set, will abort future events as much as possible; Thread " << QString().sprintf("%p", QThread::currentThread());
    _shuttingDown = true;
}

void PhysicsWorldWorker::runOnePass()
{
    // Avoid additional computations due to non-empty event queue upon thread shutting down
    if(_shuttingDown)
        return;

#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << parent->id << ")::runOnePass(" << parent->currentTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    // Stores the moment at which the simulation must be rewinded after an object insertion or removal
    btScalar rewindTime = std::numeric_limits<btScalar>::max();
    bool rewind=false;

    // Manage entity and grid queues before the next simulation
    //FIXME: the code to add/remove entities below is probably partly wrong now.
    parent->entityMutex.lock();
//        while(!parent->entityAdditionQueue.isEmpty())
//        {
//            QPair<obEntityWrapper *, btScalar> entity = parent->entityAdditionQueue.dequeue();
//            parent->_addEntity(entity.first);
//            rewindTime = qMin(rewindTime, entity.second);
//            rewind = true;
//        }
    while(!parent->entityRemovalQueue.isEmpty())
    {
        QPair<obEntityWrapper *, btScalar> entity = parent->entityRemovalQueue.dequeue();
        parent->_removeEntity(entity.first);
        rewindTime = qMin(rewindTime, entity.second);
        rewind = true;
    }
    parent->entityMutex.unlock();

    // Rollback to a given time step (not yet implemented)
    if(rewind && rewindTime < parent->currentTime)
    {
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << parent->id << ")::runOnePass(" << parent->currentTime << "); " << "World " << parent->id << "added/removed entity at time" << rewindTime << "but is already at" << parent->currentTime << "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    }

    // Simulate a step
    parent->getBulletManager()->getDynamicsWorld()->stepSimulation(parent->targetTimeStep);

    // While the application is not stopping, queue another pass
    if(!_shuttingDown)
        QMetaObject::invokeMethod(this, "runOnePass", Qt::QueuedConnection);
#ifndef NDEBUG
    else
        qDebug() << "PhysicsWorld(" << parent->id << ")::runOnePass(" << parent->currentTime << "); Did not queue next pass, shutting down; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
}

void PhysicsWorldWorker::onTerritoryIntrusion(const PhysicsWorld *&neighbor, const QVector<CellBorderCoordinates> &coords)
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << parent->id << ")::onTerritoryIntrusion(" << (neighbor ? neighbor->getId() : PhysicsWorld::NullWorldId) << ", Vector[" << coords.size() << "]); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    //TODO
}

void PhysicsWorldWorker::onOwnershipTransfer(const PhysicsWorld *&neighbor, const obEntityWrapper *&object, const btScalar &time)
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" <<  parent->id << ")::onOwnershipTransfer(" << neighbor->getId() << ", " << object->getDisplayName() << ", " << time << "); FOR NOW WE IGNORE TIME!; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    obEntityWrapper *obEnt = const_cast<obEntityWrapper *>(object);
//    obEnt->setStatus(obEntity::NormalStatus);
//     parent->_addEntity(obEnt);
}















short PhysicsWorld::WorldIdCounter = 1;
const short PhysicsWorld::NullWorldId = -500;
const short PhysicsWorld::UnknownWorldId = -1;
const short PhysicsWorld::IdBeingProcessed = -2;

const int PhysicsWorld::EntityColors[PhysicsWorld::NbColors][3] = {
    {140, 0, 26}, // Burgundy
    {51, 102, 153}, // Azure
    {251, 231, 128}, // Banana Mania
    {147, 197, 146}, // Pistachio

    {170, 156, 143}, // Light gray
    {145, 95, 109}, // Mauve Taupe
    {75, 0, 130}, // Pigment Indigo
    {0, 123, 167}, // Cerulean

    {252, 194, 0}, // Golden Poppy
    {226, 114, 91}, // Terra Cotta
    {191, 255, 0}, // Lime
    {201, 192, 187} // Pale Silver
};

PhysicsWorld::PhysicsWorld(const Simulation &simulation, const btScalar &targetTimeStep) :
    simulation(simulation),
    id(WorldIdCounter++), targetTimeStep(targetTimeStep), entities(), globalStaticEntities(),
    buffer(new CircularTransformBuffer(this)), currentTime(0),
    localGrid(0), bulletManager(new BulletManager()), entityMutex(),
    entityAdditionQueue(), entityRemovalQueue(), worldThread(this), timer()
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::PhysicsWorld();";
#endif

    // Setup broadphase
    bulletManager->setBroadphaseWorld(this);

    // Set tick callback that will be called to write transforms to the buffer
    getBulletManager()->getDynamicsWorld()->setInternalTickCallback(_tickCallback, static_cast<void *>(this));

    // Initial CircularTransformBuffer entry
    //FIXME: in this record, status's should be inserted? Use addRecord;
    obEntityTransformRecordList *initialPositions = new obEntityTransformRecordList(0);
    for(int i=0; i<entities.size(); ++i)
        initialPositions->addTransform(entities[i], entities[i]->getRigidBody()->getBulletBody()->getWorldTransform());
    buffer->appendTimeStep(initialPositions);

    // Build the worker object that will run in this PhysicsWorld's thread
    worker = new PhysicsWorldWorker(this);

#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::PhysicsWorld(); Constructed.";
#endif
}

PhysicsWorld::~PhysicsWorld()
{
    stopSimulation();
    delete worker;

    QListIterator<TimedEntity> addIt(entityAdditionQueue);
    while(addIt.hasNext())
    {
        obEntityWrapper *obEnt = addIt.next().first;
        delete obEnt;
    }
    entityAdditionQueue.clear();

    QListIterator<TimedEntity> remIt(entityRemovalQueue);
    while(remIt.hasNext())
    {
        obEntityWrapper *obEnt = remIt.next().first;
        _removeEntity(obEnt);
    }
    entityRemovalQueue.clear();

    while(!entities.isEmpty())
        _removeEntity(entities[0]);

    if(bulletManager)
        delete bulletManager;

    for(int i=0; i<globalStaticEntities.size(); ++i)
        delete globalStaticEntities[i];
    globalStaticEntities.clear();

    if(localGrid)
        delete localGrid;
}

void PhysicsWorld::startSimulation()
{
    // Start the thread with the worker attached to it
    worldThread.startAndAttachWorker(worker);

    //TODO: re-enable writes on the CircularTransformBuffer

    // Call runOnePass, which will automatically trigger the next one until shutting down
    QMetaObject::invokeMethod(worker, "runOnePass", Qt::QueuedConnection);
}

void PhysicsWorld::stopSimulation()
{
    if(worldThread.isRunning())
    {
        // Tell the worker to shutdown and queue a thread shutdown event that will stop the thread once its event queue is emptied
        worldThread.exitEventLoop();

        // Stop all potential blocking calls that might prevent the worker from finishing
        // Remember that this is called from the main thread, so race conditions must be properly managed
        buffer->abortAllWrites();

        // Wait for the worker to return from the last pass, and for the event queue to reach the thread shutdown event
        worldThread.wait();
    }
}

BulletManager* PhysicsWorld::getBulletManager() const
{
    return bulletManager;
}

int PhysicsWorld::getId() const
{
    return id;
}

CircularTransformBuffer* PhysicsWorld::getCircularBuffer() const
{
    return buffer;
}

//TODO: modify queue to add/remove objects according to targetTime and to store targetTime // ROLLBACK&PROPAGATE
void PhysicsWorld::addEntity(obEntityWrapper *obEnt, btScalar targetTime)
{
#ifndef NDEBUG
    qWarning() << "PhysicsWorld(" << id << ")::addEntity(" << obEnt->getDisplayName() << "," << targetTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    entityMutex.lock();
    entityAdditionQueue.enqueue(TimedEntity(obEnt, targetTime));
    entityMutex.unlock();
}

void PhysicsWorld::_addEntity(obEntityWrapper *obEnt)
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::_addEntity(" << obEnt->getDisplayName() << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    if(localGrid)
        localGrid->addEntity(obEnt);
    entities.append(obEnt);

    getBulletManager()->getDynamicsWorld()->addRigidBody(obEnt->getRigidBody()->getBulletBody());
    obEnt->setColor(EntityColors[id%NbColors][0]/255.f, EntityColors[id%NbColors][1]/255.f, EntityColors[id%NbColors][2]/255.f);
    obEnt->setOwnerWorld(this);
}

void PhysicsWorld::_addCellBorder(CellBorderEntity *cbEnt)
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::_addCellBorder(" << cbEnt->getDisplayName() << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    if(localGrid)
        localGrid->addCellBorder(cbEnt);
    getBulletManager()->getDynamicsWorld()->addRigidBody(cbEnt->getRigidBody()->getBulletBody());

    cbEnt->setColor(EntityColors[id%NbColors][0]/255.f, EntityColors[id%NbColors][1]/255.f, EntityColors[id%NbColors][2]/255.f);
}

void PhysicsWorld::_entityVectoryRemovalMethod(QVector<obEntityWrapper *> &container, obEntityWrapper *obEnt)
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::_entityVectoryRemovalMethod(" << obEnt->getDisplayName() << "); Start; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    Q_ASSERT(obEnt != 0);
    qDebug() << "BEGIN CONTAINER " << container.size();

    for(int i=0; i<container.size(); ++i)
        if(container[i]->getName() == obEnt->getName())
            container.remove(i--);

#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::_entityVectoryRemovalMethod(" << obEnt->getDisplayName() << "); End; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
}

void PhysicsWorld::removeEntity(obEntityWrapper *obEnt, btScalar targetTime)
{
#ifndef NDEBUG
    qWarning() << "PhysicsWorld(" << id << ")::removeEntity(" << obEnt->getDisplayName() << "," << targetTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    entityMutex.lock();
    entityRemovalQueue.enqueue(TimedEntity(obEnt, targetTime));

#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::removeEntity(" << obEnt->getDisplayName() << "," << targetTime << "); Entity enqueued for removal; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    entityMutex.unlock();
}

void PhysicsWorld::_removeEntity(obEntityWrapper *obEnt)
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::_removeEntity(" << obEnt->getDisplayName() << "); at " << currentTime << "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    // Remove the entity from the BulletWorld
    getBulletManager()->getDynamicsWorld()->removeRigidBody(obEnt->getRigidBody()->getBulletBody());

    // Remove the entity from the grid if it still is within a grid cell (it may be outside of the grid if the deletion is a result of it moving out of bounds)
    if(localGrid && obEnt->getRigidBody()->getMotionState()->getLocalGrid())
        localGrid->removeEntity(obEnt);

    // Remove the entity from the world's vector
    _entityVectoryRemovalMethod(entities, obEnt);
    obEnt->unsetOwnerWorld();

	// Delete the object if it is not in the simulation space anymore
    if(obEnt->getStatus() == obEntity::OutOfSimulationSpace && simulation.getWorldType() != GridInformation::ClosedWorld)
	{
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << id << ")::_removeEntity(" << obEnt->getDisplayName() << "); Entity has left simulation space, should do something about it (ignore if object went below floor in SidewiseOpenWorld); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
	}

#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::_removeEntity(" << obEnt->getDisplayName() << "); Entity effectively removed; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
}

void PhysicsWorld::assignLocalGrid(LocalGrid *local)
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::assignLocalGrid(" << local << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    // If there is already a grid, remove all of its entities and delete it
    if(localGrid)
    {
        LocalGrid *old = localGrid;
        localGrid = 0;

        for(LocalGrid::iterator it=old->begin(); it!=old->end(); it++)
        {
            Cell &cell = *it;
            if(cell.getEntities() != NULL)
            {
                QVectorIterator<obEntityWrapper *> entIt(*cell.getEntities());
                while(entIt.hasNext())
                    _removeEntity(entIt.next());
            }
        }

        delete old;
    }

    // Add to the world all the entities present within the new grid
    for(LocalGrid::iterator it=local->begin(); it!=local->end(); it++)
    {
        Cell &cell = *it;

        // Add entities to the world
        if(cell.getEntities() != NULL)
        {
            QVectorIterator<obEntityWrapper *> entIt(*cell.getEntities());
            while(entIt.hasNext())
                _addEntity(entIt.next());
        }
    }

    // Set the new grid of this world, so that entities can query the world for its grid
    localGrid = local;

    //TODO: every local object should also be inserted into the LocalGrid here (first must code LocalGrid hierarchies)
}

void PhysicsWorld::drawCells()
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::drawCells(); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    for(LocalGrid::iterator it=localGrid->begin(); it!=localGrid->end(); it++)
    {
        Cell &c = *it;
        if(c.getOwnerId() == this->id)
        {
            btVector3 coords = Utils::btVectorFromBlitz(it.position());
            btVector3 position = localGrid->getGridInformation()->toCenteredWorldCoordinates(localGrid->getResolution(), coords);
//            position.setX(position.x() + (int)(rand() % 40 - 20));
            btVector3 scale(localGrid->getGridInformation()->getCellLength() / (btVector3(102, 102, 102)));

            Ogre::String entityName = "cell_id:" + /*Ogre::StringConverter::toString(id) +
                    "x:" +*/ Ogre::StringConverter::toString(coords.x()) +
                    "y:" + Ogre::StringConverter::toString(coords.y()) +
                    "z:" + Ogre::StringConverter::toString(coords.z());

            Ogre::Vector3 ogrePos = Utils::vectorFromBullet(position);
            Ogre::Vector3 ogreScale = Utils::vectorFromBullet(scale);

            obEntityWrapper *obEnt = new obEntityWrapper(entityName, "cube.mesh", ogrePos, Ogre::Quaternion::IDENTITY, true, ogreScale, 0);
            obEnt->setMaterialName("Surfaces/RockDirt");
            obEnt->setColor(EntityColors[id%NbColors][0]/255.f, EntityColors[id%NbColors][1]/255.f, EntityColors[id%NbColors][2]/255.f);
        }
    }
}

void PhysicsWorld::setupLocalGridBorders()
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::setupLocalGridBorders(); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    // Create Cell border entities
    for(LocalGrid::iterator it=localGrid->begin(); it!=localGrid->end(); it++)
    {
        // Instantiate the CellBorderEntity of the Cell
        btVector3 coords = Utils::btVectorFromBlitz(it.position());
        QVector<bool> cons = localGrid->getCellBorders(coords);

        for(int i=0; i<cons.size(); ++i)
            if(cons[i])
                _addCellBorder(new CellBorderEntity(localGrid, CellBorderCoordinates(coords, i)));
    }
}

void PhysicsWorld::createScene()
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::createScene(); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    // Create a floor
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0,0,0));

    btDefaultMotionState *motionState = new btDefaultMotionState(transform);
    btCollisionShape *shape = new btStaticPlaneShape(btVector3(0,1,0), 0);
    btVector3 localInertia;
    shape->calculateLocalInertia(0, localInertia);

    btRigidBody *floorBody = new btRigidBody(0, motionState, shape, localInertia);

    // Add it to the physics world
    bulletManager->getDynamicsWorld()->addRigidBody(floorBody);
    globalStaticEntities.append(floorBody);
}

PhysicsWorld *PhysicsWorld::getNeighbor(const short neighborId) const
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::getNeighbor(" << neighborId << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    return simulation.getWorldFromId(neighborId);
}

bool PhysicsWorld::messageNeighbor(PhysicsWorld *neighbor, const char *method, QGenericArgument val0, QGenericArgument val1, QGenericArgument val2, QGenericArgument val3, QGenericArgument val4, QGenericArgument val5, QGenericArgument val6, QGenericArgument val7, QGenericArgument val8, QGenericArgument val9) const
{
#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << (neighbor ? neighbor->getId() : PhysicsWorld::NullWorldId) << ", " << method << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    if(neighbor)
    {
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << (neighbor ? neighbor->getId() : PhysicsWorld::NullWorldId) << ", " << method << "); Invoking method; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        QMetaObject::invokeMethod(neighbor->worker, method, Qt::QueuedConnection, val0, val1, val2, val3, val4, val5, val6, val7, val8, val9);
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << (neighbor ? neighbor->getId() : PhysicsWorld::NullWorldId) << ", " << method << "); Message sent; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        return true;
    }
    else
    {
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << (neighbor ? neighbor->getId() : PhysicsWorld::NullWorldId) << ", " << method << "); Could not find neighbor, aborting; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        return false;
    }
}

//TODO: document messageNeighbor
bool PhysicsWorld::messageNeighbor(const short neighborId, const char *method, QGenericArgument val0, QGenericArgument val1, QGenericArgument val2, QGenericArgument val3, QGenericArgument val4, QGenericArgument val5, QGenericArgument val6, QGenericArgument val7, QGenericArgument val8, QGenericArgument val9) const
{
#ifndef NDEBUG
	qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << neighborId << ", " << method << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

	PhysicsWorld *neighbor = getNeighbor(neighborId);
	if(neighbor)
	{
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << neighborId << ", " << method << "); Invoking method; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        QMetaObject::invokeMethod(neighbor->worker, method, Qt::QueuedConnection, val0, val1, val2, val3, val4, val5, val6, val7, val8, val9);
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << neighborId << ", " << method << "); Message sent; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
		return true;
	}
	else
    {
#ifndef NDEBUG
        qDebug() << "PhysicsWorld(" << id << ")::messageNeighbor(" << neighborId << ", " << method << "); Could not find neighbor, aborting; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        return false;
    }
}

void PhysicsWorld::_tickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    PhysicsWorld *w = static_cast<PhysicsWorld *>(world->getWorldUserInfo());

#ifndef NDEBUG
    qDebug() << "PhysicsWorld(" << w->id << ")::_tickCallback(" << w->currentTime << ", " << timeStep << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    w->currentTime += timeStep;

    obEntityTransformRecordList *list = new obEntityTransformRecordList(w->currentTime);
    for(int i=0; i<w->entities.size(); ++i)
    {
        //TODO: optimize this call (use a reference for entities)
        list->addTransform(w->entities[i], w->entities[i]->getRigidBody()->getBulletBody()->getWorldTransform());

//        if(w->entities[i]->getStatus() & (obEntity::OutOfWorld | obEntity::OutOfSimulationSpace))
//            w->_removeEntity(w->entities[i]);
    }

    w->buffer->appendTimeStep(list);
}
