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

PhysicsWorld::PhysicsWorld(const btScalar &targetTimeStep) :
    id(WorldIdCounter++), targetTimeStep(targetTimeStep), entities(), borders(), buffer(new CircularTransformBuffer()), currentTime(0),
    localGrid(0), bulletManager(new BulletManager()), entityMutex(),
    entityAdditionQueue(), entityRemovalQueue(), CDInterface(this)
{
    CDInterface.init();
}

PhysicsWorld::~PhysicsWorld()
{
    if(CDInterface.isRunning())
        CDInterface.exit();

    QListIterator<TimedEntity> addIt(entityAdditionQueue);
    while(addIt.hasNext())
    {
        obEntityWrapper *obEnt = addIt.next().first;
        entities.remove(obEnt->getName());
        delete obEnt;
    }
    entityAdditionQueue.clear();


    QMapIterator<Ogre::String, obEntityWrapper *> it(entities);
    while(it.hasNext())
        delete it.next().value();
    entities.clear();

    if(localGrid)
        delete localGrid;

    if(bulletManager)
        delete bulletManager;
}

void PhysicsWorld::startSimulation()
{
    CDInterface.start();
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

void PhysicsWorld::addEntity(obEntityWrapper *obEnt, btScalar targetTime)
{
    entityMutex.lock();
    entityAdditionQueue.enqueue(TimedEntity(obEnt, targetTime));
	entityMutex.unlock();
}

void PhysicsWorld::_addEntity(obEntityWrapper *obEnt)
{
    if(localGrid)
        localGrid->addEntity(obEnt);
    entities.insert(obEnt->getName(), obEnt);
    getBulletManager()->getDynamicsWorld()->addRigidBody(obEnt->getRigidBody()->getBulletBody());
    obEnt->setColor(EntityColors[id%NbColors][0]/255.f, EntityColors[id%NbColors][1]/255.f, EntityColors[id%NbColors][2]/255.f);
    obEnt->setOwnerWorld(this);
}

void PhysicsWorld::_addCellBorder(CellBorderEntity *cbEnt)
{
    if(localGrid)
        localGrid->addCellBorder(cbEnt);
    getBulletManager()->getDynamicsWorld()->addRigidBody(cbEnt->getRigidBody()->getBulletBody());

    cbEnt->setColor(EntityColors[id%NbColors][0]/255.f, EntityColors[id%NbColors][1]/255.f, EntityColors[id%NbColors][2]/255.f);
}

//TODO: modify queue to add/remove objects according to targetTime and to store targetTime
void PhysicsWorld::removeEntity(obEntityWrapper *obEnt, btScalar targetTime)
{
    entityMutex.lock();
    entityRemovalQueue.enqueue(TimedEntity(obEnt, targetTime));
    entityMutex.unlock();
}

void PhysicsWorld::_removeEntity(obEntityWrapper *obEnt)
{
	if(localGrid)
		localGrid->removeEntity(obEnt);
    entities.remove(obEnt->getName());
    getBulletManager()->getDynamicsWorld()->removeRigidBody(obEnt->getRigidBody()->getBulletBody());
    obEnt->unsetOwnerWorld();
}

void PhysicsWorld::assignLocalGrid(LocalGrid *local)
{
//    local->setOwnerId(id);

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
    for(LocalGrid::iterator it=localGrid->begin(); it!=localGrid->end(); it++)
    {
        Cell &c = *it;
        if(c.getOwnerId() == this->id)
        {
            btVector3 coords = Utils::btVectorFromBlitz(it.position());
            btVector3 position = localGrid->getGridInformation()->toCenteredWorldCoordinates(localGrid->getResolution(), coords);
//            position.setX(position.x() + (int)(rand() % 40 - 20));
            btVector3 scale(localGrid->getGridInformation()->getCellLength() / (btVector3(102, 102, 102)));

            Ogre::String entityName = "cell" + Ogre::StringConverter::toString(coords.x()) + Ogre::StringConverter::toString(coords.y()) + Ogre::StringConverter::toString(coords.z());

            Ogre::Vector3 ogrePos = Utils::vectorFromBullet(position);
            Ogre::Vector3 ogreScale = Utils::vectorFromBullet(scale);

            obEntityWrapper *obEnt = new obEntityWrapper(entityName, "cube.mesh", ogrePos, Ogre::Quaternion::IDENTITY, true, ogreScale, 0);
            obEnt->setMaterialName("Surfaces/RockDirt");
            obEnt->setColor(EntityColors[id%NbColors][0]/255.f, EntityColors[id%NbColors][1]/255.f, EntityColors[id%NbColors][2]/255.f);

            qDebug() << position.x() << position.y() << position.z();
            qDebug() << obEnt->getCenteredPosition().x() << obEnt->getCenteredPosition().y() << obEnt->getCenteredPosition().z();
        }
    }
}

void PhysicsWorld::setupLocalGridBorders()
{
    // Create Cell border entities
    for(LocalGrid::iterator it=localGrid->begin(); it!=localGrid->end(); it++)
    {
        // Instantiate the CellBorderEntity of the Cell
        btVector3 coords = Utils::btVectorFromBlitz(it.position());
        QVector<bool> cons = localGrid->getConnectionsToOtherWorlds(coords);

        for(int i=0; i<cons.size(); ++i)
            if(cons[i])
                _addCellBorder(new CellBorderEntity(localGrid, CellBorderCoordinates(coords, i)));
    }
}

void PhysicsWorld::createScene()
{
    // Create a floor
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0,1,0));

    btDefaultMotionState *motionState = new btDefaultMotionState(transform);
    btCollisionShape *shape = new btStaticPlaneShape(btVector3(0,1,0), 0);
    btVector3 localInertia;
    shape->calculateLocalInertia(0, localInertia);

    btRigidBody *floorBody = new btRigidBody(0, motionState, shape, localInertia);

    // Store a pointer to the world the object belongs to
//    floorBody->setUserPointer(this);

    // Add it to the physics world
    bulletManager->getDynamicsWorld()->addRigidBody(floorBody);
}

PhysicsWorld::BulletFakeCCDThread::BulletFakeCCDThread(PhysicsWorld *world) :
    world(world)
{
}

//TODO: must include a pointer to last computed buffer entry st. only moved objects are inserted?
void PhysicsWorld::BulletFakeCCDThread::myTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    PhysicsWorld *w = static_cast<PhysicsWorld *>(world->getWorldUserInfo());
    w->currentTime += timeStep;

    obEntityTransformRecordList *list = new obEntityTransformRecordList(w->currentTime);
    QMapIterator<Ogre::String, obEntityWrapper*> it(w->entities);
    while(it.hasNext())
    {
        QMap<Ogre::String, obEntityWrapper *>::const_iterator entity = it.next();
        list->addTransform(entity.value(), entity.value()->getRigidBody()->getBulletBody()->getWorldTransform());
    }

    w->buffer->appendTimeStep(list);
}

void PhysicsWorld::BulletFakeCCDThread::init()
{
	world->getBulletManager()->getDynamicsWorld()->getDispatchInfo().m_useContinuous = true;
        world->getBulletManager()->getDynamicsWorld()->setInternalTickCallback(myTickCallback, static_cast<void *>(this->world));
}

//TODO: investigate motionStates instead of this to see which performs better
//TODO: proper thread joining when quitting
void PhysicsWorld::BulletFakeCCDThread::run()
{
    // Initialization
    obEntityTransformRecordList *initialPositions = new obEntityTransformRecordList(0);
    QMapIterator<Ogre::String, obEntityWrapper*> it(world->entities);
    while(it.hasNext())
    {
            QMap<Ogre::String, obEntityWrapper *>::const_iterator entity = it.next();
            initialPositions->addTransform(entity.value(), entity.value()->getRigidBody()->getBulletBody()->getWorldTransform());
    }

    // Stores the moment at which the simulation must be rewinded after an object insertion or removal
    btScalar rewindTime;
    bool rewind;

    // Infinite CD loop
    for(;;)
    {
        rewindTime = std::numeric_limits<btScalar>::max();
        rewind=false;

        // Manage entity and grid queues before the next simulation
        world->entityMutex.lock();
        while(!world->entityAdditionQueue.isEmpty())
        {
            QPair<obEntityWrapper *, btScalar> entity = world->entityAdditionQueue.dequeue();
            world->_addEntity(entity.first);
            rewindTime = qMin(rewindTime, entity.second);
            rewind = true;
        }
        while(!world->entityRemovalQueue.isEmpty())
        {
            QPair<obEntityWrapper *, btScalar> entity = world->entityRemovalQueue.dequeue();
            world->_removeEntity(entity.first);
            rewindTime = qMin(rewindTime, entity.second);
            rewind = true;
        }
        world->entityMutex.unlock();

        // Rollback to a given time step (not yet implemented)
        if(rewind && rewindTime < world->currentTime)
            qDebug() << "Must rewind at time " << rewindTime;

        //NOTE: show the density of cells at ground-level to assess quality of spatial subdivision cell size
//        Array<Cell, 2> groundSlice = world->localGrid->operator ()(Range::all(), 0, Range::all());
//        int sumDens = 0;
//        for(LocalGrid::iterator it = world->localGrid->begin(); it != world->localGrid->end(); it++)
//        {
//            const QVector<obEntityWrapper *> *vect = (*it).getEntities();
//            if(vect)
//                sumDens += vect->size();
//        }
//        qDebug() << "Average ground density: " << (float) sumDens / groundSlice.size();

        // Simulate a step
        world->getBulletManager()->getDynamicsWorld()->stepSimulation(world->targetTimeStep);
//        qDebug() << "Thread #" << world->id << " computed pass " << i << " at time " << world->currentTime;
    }

    this->exit();
}
