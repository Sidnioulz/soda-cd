#include "btlocalgridbroadphase.h"
#include "physicsworld.h"
#include <QVector>

btLocalGridBroadphase::btLocalGridBroadphase(PhysicsWorld *world, btOverlappingPairCache* overlappingPairCache) :
    world(world),
    m_pairCache(overlappingPairCache),
    m_ownsPairCache(false)
{
    if (!overlappingPairCache)
    {
        void* mem = btAlignedAlloc(sizeof(btHashedOverlappingPairCache),16);
        m_pairCache = new (mem)btHashedOverlappingPairCache();
        m_ownsPairCache = true;
    }
}

btLocalGridBroadphase::~btLocalGridBroadphase()
{
    if (m_ownsPairCache)
    {
        m_pairCache->~btOverlappingPairCache();
        btAlignedFree(m_pairCache);
    }
}

btBroadphaseProxy *btLocalGridBroadphase::createProxy(const btVector3 &aabbMin, const btVector3 &aabbMax, int /*shapetype*/, void *userPtr, short collisionFilterGroup, short collisionFilterMask, btDispatcher *dispatcher, void *multiSapProxy)
{
    btAssert(aabbMin[0]<= aabbMax[0] && aabbMin[1]<= aabbMax[1] && aabbMin[2]<= aabbMax[2]);

    btLocalGridProxy *proxy0 = new btLocalGridProxy(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, multiSapProxy);
//    entity->setProxy(proxy0);

    return proxy0;
}

void btLocalGridBroadphase::destroyProxy(btBroadphaseProxy *proxyOrg, btDispatcher *dispatcher)
{
        btLocalGridProxy *proxy0 = static_cast<btLocalGridProxy*>(proxyOrg);

//        proxy0->parentEntity->unsetProxy();
        proxy0->m_clientObject = 0;

        m_pairCache->removeOverlappingPairsContainingProxy(proxyOrg,dispatcher);
}

bool btLocalGridBroadphase::aabbOverlap(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1)
{
    return proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] && proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0] &&
           proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] && proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1] &&
           proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] && proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2];

}

void btLocalGridBroadphase::calculateOverlappingPairs(btDispatcher *dispatcher)
{
    //MISSING: border-entity collisions
    //MISSING: static env. collisions

    // Don't do anything until a grid has been set
    if(!world->getLocalGrid())
        return;

    LocalGrid *grid = world->getLocalGrid();
    const btVector3 &nbCells = grid->getGridInformation()->getGridAtResolution(grid->getResolution())->getNbCells();

    // A table that stores pointers to lists of entities that possibly overlap with those of the current Cell
    const QVector<obEntityWrapper *> *entityVectors[2][2][2];

    // Quicker pointer to entityVectors[0][0][0] for code readability
    const QVector<obEntityWrapper *> *entities;

    // Reference to the static entities defined in the linked world.
    const QVector<btRigidBody *> &staticEnts = world->getStaticEntities();

    // Browse through all Cells
    for(int x=0; x<nbCells.x(); ++x)
        for(int y=0; y<nbCells.y(); ++y)
            for(int z=0; z<nbCells.z(); ++z)
            {
                Cell &c = grid->at(x, y, z);
                entityVectors[0][0][0] = entities = c.getEntities();

                // Perform operations only if the world has entities
                if(entities)
                {
                    // Check all dynamic entities against their static counterparts
                    for(int i=0; i<entities->size(); ++i)
                        for(int j=0; j<staticEnts.size(); ++j)
                            if(true)
//                            if(aabbOverlap(entities->at(i)->getRigidBody()->getBulletBody()->getBroadphaseProxy(), staticEnts[j]->getBroadphaseProxy()))
                            {
                                //NOTE: might be necessary to check if it's in the cache first?
                                m_pairCache->addOverlappingPair(entities->at(i)->getRigidBody()->getBulletBody()->getBroadphaseProxy(), staticEnts[j]->getBroadphaseProxy());
                            }

                    // First setup a cache table telling which next cells also have entities to check against
                    for(int i=0; i<2; ++i)
                        for(int j=0; j<2; ++j)
                            for(int k=0; k<2; ++k)
                            {
                                // 0,0,0 is the current Cell.
                                if(i || j || k)
                                {
                                    if(!grid->ownedByAnotherWorld(x+i, y+j, z+k))
                                    {
                                        Cell &o = grid->at(i, j, k);
                                        entityVectors[i][j][k] = o.getEntities();
                                    }
                                    else
                                        entityVectors[i][j][k] = 0;
                                }
                            }

                    // Now browse this Cell's entity vector and check each entity for collisions
                    // against those of all other entity vectors
                    for(int u=0; u<entities->size(); ++u)
                    {
                        // First check against entities of the same Cell
                        for(int v=u+1; v<entities->size(); ++v)
                            if(true)
//                            if(aabbOverlap(entities->at(u)->getRigidBody()->getBulletBody()->getBroadphaseProxy(), entities->at(v)->getRigidBody()->getBulletBody()->getBroadphaseProxy()))
                            {
                                //NOTE: might be necessary to check if it's in the cache first?
                                m_pairCache->addOverlappingPair(entities->at(u)->getRigidBody()->getBulletBody()->getBroadphaseProxy(), entities->at(v)->getRigidBody()->getBulletBody()->getBroadphaseProxy());
                            }

                        // Then of neighboring Cells
                        for(int i=0; i<2; ++i)
                            for(int j=0; j<2; ++j)
                                for(int k=0; k<2; ++k)
                                {
                                    if(entityVectors[i][j][k])
                                        for(int v=0; v<entityVectors[i][j][k]->size(); ++v)
                                            if(true)
//                                            if(aabbOverlap(entities->at(u)->getRigidBody()->getBulletBody()->getBroadphaseProxy(), entityVectors[i][j][k]->at(v)->getRigidBody()->getBulletBody()->getBroadphaseProxy()))
                                            {
                                                //NOTE: might be necessary to check if it's in the cache first?
                                                m_pairCache->addOverlappingPair(entities->at(u)->getRigidBody()->getBulletBody()->getBroadphaseProxy(), entityVectors[i][j][k]->at(v)->getRigidBody()->getBulletBody()->getBroadphaseProxy());
                                            }
                                }
                    }
                }
            }
}

void btLocalGridBroadphase::rayTest(const btVector3 &/*rayFrom*/,const btVector3 &/*rayTo*/, btBroadphaseRayCallback &rayCallback, const btVector3 &/*aabbMin*/, const btVector3 &/*aabbMax*/)
{
    // Don't do anything until a grid has been set
    if(!world->getLocalGrid())
        return;

    LocalGrid *grid = world->getLocalGrid();
    const btVector3 &nbCells = grid->getGridInformation()->getGridAtResolution(grid->getResolution())->getNbCells();
    const QVector<obEntityWrapper *> *entities;

    for(int x=0; x<nbCells.x(); ++x)
        for(int y=0; y<nbCells.y(); ++y)
            for(int z=0; z<nbCells.z(); ++z)
            {
                Cell &c = grid->at(x, y, z);
                entities = c.getEntities();

                if(entities)
                    for(int i=0; i<entities->size(); ++i)
                    {
                        btBroadphaseProxy *proxy = entities->at(i)->getRigidBody()->getBulletBody()->getBroadphaseProxy();

                        if(proxy)
                            rayCallback.process(proxy);
                    }
            }
}

void btLocalGridBroadphase::aabbTest(const btVector3 &aabbMin, const btVector3 &aabbMax, btBroadphaseAabbCallback &callback)
{
    // Don't do anything until a grid has been set
    if(!world->getLocalGrid())
        return;

    LocalGrid *grid = world->getLocalGrid();
    const btVector3 &nbCells = grid->getGridInformation()->getGridAtResolution(grid->getResolution())->getNbCells();
    const QVector<obEntityWrapper *> *entities;

    for(int x=0; x<nbCells.x(); ++x)
        for(int y=0; y<nbCells.y(); ++y)
            for(int z=0; z<nbCells.z(); ++z)
            {
                Cell &c = grid->at(x, y, z);
                entities = c.getEntities();

                if(entities)
                    for(int i=0; i<entities->size(); ++i)
                    {
                        btBroadphaseProxy *proxy = entities->at(i)->getRigidBody()->getBulletBody()->getBroadphaseProxy();

                        if(proxy && TestAabbAgainstAabb2(aabbMin, aabbMax, proxy->m_aabbMin, proxy->m_aabbMax))
                            callback.process(proxy);
                    }
            }
}
