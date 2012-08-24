#include "sodaContactResultCallback.h"
#include "sodaDynamicsWorld.h"

sodaContactResultCallback::sodaContactResultCallback(sodaDynamicsWorld *world) :
    sodaWorld(world),
    colOccurred(false),
    resultCount(0)
{
}

sodaContactResultCallback::~sodaContactResultCallback()
{
    collisionManifolds.clear();
}

btScalar sodaContactResultCallback::addSingleResult(btManifoldPoint& cp,
                                 const btCollisionObject* colObj0, int, int,
                                 const btCollisionObject* colObj1, int, int)
{
    // Use foreign manifolds to find more unions
    const btCollisionObject *local = 0;
    const btCollisionObject *foreigner = 0;

    // Check if colObj0 is the foreigner
    const sodaEntity *tmpEntity = static_cast<const sodaEntity *>(colObj0->getUserPointer());

    // CellBorderEntity way of reaching owner id
    if(tmpEntity->getType() == sodaEntity::CellBorderEntityType)
    {
        const CellBorderEntity *cbEnt = dynamic_cast<const CellBorderEntity *>(tmpEntity);
        if(cbEnt->getLocalGrid()->getOwnerId() != sodaWorld->getWorldId())
        {
            local = colObj1;
            foreigner = colObj0;
        }
        else
        {
            local = colObj0;
            foreigner = colObj1;
        }
    }
    // DynamicEntity way of reaching owner id
    else if(tmpEntity->getType() == sodaEntity::DynamicEntityType)
    {
        const sodaDynamicEntity *obEnt = dynamic_cast<const sodaDynamicEntity *>(tmpEntity);
        if(obEnt->getOwnerId() != sodaWorld->getWorldId())
        {
            local = colObj1;
            foreigner = colObj0;
        }
        else
        {
            local = colObj0;
            foreigner = colObj1;
        }
    }

#ifndef NDEBUG
    // Check that both bodies were properly located
    if(!foreigner || !local)
    {
        qWarning() << "sodaContactResultCallback::addSingleResult(); Failed to locate local or foreign body; Thread " << QString().sprintf("%p", QThread::currentThread());
        return -1;
    }
#endif

    // Look for the manifold in the cache from previous step
    int foundIndex = collisionManifolds.findLinearSearch(local, foreigner);
    if(foundIndex != collisionManifolds.size())
    {
        sodaPersistentForeignerManifold *&manifold = collisionManifolds.at(foundIndex);
        manifold->clearManifold();
        manifold->addManifoldPoint(cp);
        manifold->setLastTime(sodaWorld->getLocalTime());
    }
    // First time that we see this manifold
    else
    {
        sodaPersistentForeignerManifold *manifold = new sodaPersistentForeignerManifold(const_cast<btCollisionObject *>(local), const_cast<btCollisionObject *>(foreigner), 0, 0, 0);
        manifold->addManifoldPoint(cp);
        manifold->setLastTime(sodaWorld->getLocalTime());
        collisionManifolds.push_back(manifold);
    }

    // Update collision flag of the ContactResultCallback for later querying
    colOccurred = cp.getDistance() <= 0;
    resultCount++;

    return 0; // not used
}

btAlignedObjectArray<btCollisionObject *> sodaContactResultCallback::removeOutdatedManifolds(const btScalar &time)
{
    // Array used to return objects to remove from world
    btAlignedObjectArray<btCollisionObject *> foreigners;

    // Clear whole array
    if(time < 0)
    {
        // Allocate memory in one go for efficiency
        foreigners.reserve(collisionManifolds.size());

        int i=0, arraySize = collisionManifolds.size();
        for(; i<arraySize; ++i)
            foreigners.push_back((btCollisionObject*) collisionManifolds.at(i)->getForeignBody());

        collisionManifolds.clear();
    }
    // Clear past elements
    else
    {
        // Sort the array by decreasing last activation time
        collisionManifolds.quickSort(sodaCRCLastTimePredicate());

        // Allocate memory for the few objects to remove
        foreigners.reserve(collisionManifolds.size() - resultCount);

        // Push the foreign objects of the manifolds to remove in the foreigners array
        int i=resultCount, arraySize = collisionManifolds.size();
        for(; i<arraySize; ++i)
            foreigners.push_back((btCollisionObject*) collisionManifolds.at(i)->getForeignBody());

        // Automatically remove elements after the current result count
        collisionManifolds.resize(resultCount);
    }

    colOccurred = false;
    resultCount = 0;

    return foreigners;
}
