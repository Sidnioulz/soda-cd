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
        void* mem = btAlignedAlloc(sizeof(btSortedOverlappingPairCache),16);
        m_pairCache = new (mem)btSortedOverlappingPairCache();
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

    btBroadphaseProxy *proxy0 = new btBroadphaseProxy(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, multiSapProxy);
//    entity->setProxy(proxy0);

    return proxy0;
}

void btLocalGridBroadphase::destroyProxy(btBroadphaseProxy *proxyOrg, btDispatcher *dispatcher)
{
//        btBroadphaseProxy *proxy0 = static_cast<btBroadphaseProxy*>(proxyOrg);

//        proxy0->parentEntity->unsetProxy();
        proxyOrg->m_clientObject = 0;

        m_pairCache->removeOverlappingPairsContainingProxy(proxyOrg,dispatcher);
}

//FIXME: wrong conditions
bool btLocalGridBroadphase::aabbOverlap(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1)
{
//    return true;

//    return (proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] || proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0]) &&
//           (proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] || proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1]) &&
//           (proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] || proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2]);

    return (proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] && proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0]) &&
           (proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] && proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1]) &&
           (proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] && proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2]);
}

//FIXME: wrong conditions
bool btLocalGridBroadphase::aabbOverlap(const btVector3 &aabb0Min, const btVector3 &aabb0Max, const btVector3 &aabb1Min, const btVector3 &aabb1Max)
{
    return (aabb0Min[0] <= aabb1Max[0] && aabb1Min[0] <= aabb0Max[0]) &&
           (aabb0Min[1] <= aabb1Max[1] && aabb1Min[1] <= aabb0Max[1]) &&
           (aabb0Min[2] <= aabb1Max[2] && aabb1Min[2] <= aabb0Max[2]);
}

void btLocalGridBroadphase::calculateOverlappingPairs(btDispatcher *dispatcher)
{
    qDebug() << "calculateOverlappingPairs() BEGIN:" << m_pairCache->getNumOverlappingPairs();
    //MISSING: border-entity collisions
    //MISSING: static env. collisions

    // Don't do anything until a grid has been set
    if(!world->getLocalGrid())
        return;

    LocalGrid *grid = world->getLocalGrid();
//    const btVector3 &nbCells = grid->getGridInformation()->getGridAtResolution(grid->getResolution())->getNbCells();

    // A table that stores pointers to lists of entities that possibly overlap with those of the current Cell. 0,0,0 is not set
    const QVector<obEntityWrapper *> *entityVectors[2][2][2];
    entityVectors[0][0][0] = 0;

    // Pointer to the current Cell's entities
    const QVector<obEntityWrapper *> *entities;

    // Grid size information used to browse the grid
    const btVector3 &offset = grid->getOffset();
    const btVector3 uBound = grid->getLength() + offset;

    // Reference to the static entities defined in the linked world.
    const QVector<btRigidBody *> &staticEnts = world->getStaticEntities();
    const int &staticEntSize = staticEnts.size();

    for(int x=offset.x(); x<uBound.x(); ++x)
        for(int y=offset.y(); y<uBound.y(); ++y)
            for(int z=offset.z(); z<uBound.z(); ++z)
    {
        // Get the Cell and its coordinates
        const Cell &cell = grid->at(x, y, z);

        entities = cell.getEntities();
        if(entities)
        {
            const int &entSize = entities->size();
            // Perform collision detection against static bodies first
            for(int i=0; i<entSize; ++i)
            {
//                entities->at(i)->getRigidBody()->getBulletBody()->getAabb(ent0Min, ent0Max);
                for(int j=0; j<staticEntSize; ++j)
                {
//                    staticEnts[j]->getAabb(ent1Min, ent1Max);

                    if(aabbOverlap(entities->at(i)->getBroadphaseProxy(), staticEnts[j]->getBroadphaseHandle()))
                    {
                        if(!m_pairCache->findPair(entities->at(i)->getBroadphaseProxy(), staticEnts[j]->getBroadphaseHandle()))
                            m_pairCache->addOverlappingPair(entities->at(i)->getBroadphaseProxy(), staticEnts[j]->getBroadphaseHandle());
                    }
                    else
                    {
                        if(m_pairCache->findPair(entities->at(i)->getBroadphaseProxy(), staticEnts[j]->getBroadphaseHandle()))
                            m_pairCache->removeOverlappingPair(entities->at(i)->getBroadphaseProxy(), staticEnts[j]->getBroadphaseHandle(), dispatcher);
                    }
                }
            }

            // Now setup a cache table telling which adjacent cells also have entities to check against
            for(int i=0; i<2; ++i)
                for(int j=0; j<2; ++j)
                    for(int k=0; k<2; ++k)
                    {
                        // 0,0,0 is not used, ignore it
                        if(i || j || k)
                        {
                            if(!grid->ownedByAnotherWorld(x+i, y+j, z+k))
                            {
                                Cell &o = grid->at(x+i, y+j, z+k);
                                entityVectors[i][j][k] = o.getEntities();
                            }
                            else
                                entityVectors[i][j][k] = 0;
                        }
                    }




            // Now browse this Cell's entity vector and check each entity for collisions
            // against those of all other entity vectors
            for(int u=0; u<entSize; ++u)
            {
                // First check against entities of the same Cell (avoiding duplicates)
                for(int v=u+1; v<entSize; ++v)
                    if(aabbOverlap(entities->at(u)->getBroadphaseProxy(), entities->at(v)->getBroadphaseProxy()))
                    {
                        if(!m_pairCache->findPair(entities->at(u)->getBroadphaseProxy(), entities->at(v)->getBroadphaseProxy()))
                            m_pairCache->addOverlappingPair(entities->at(u)->getBroadphaseProxy(), entities->at(v)->getBroadphaseProxy());
                    }
                    else
                    {
                        if(m_pairCache->findPair(entities->at(u)->getBroadphaseProxy(), entities->at(v)->getBroadphaseProxy()))
                            m_pairCache->removeOverlappingPair(entities->at(u)->getBroadphaseProxy(), entities->at(v)->getBroadphaseProxy(), dispatcher);
                    }

                // Then of neighboring Cells (ignoring entityVectors[0][0][0] since it is set to null)
                for(int i=0; i<2; ++i)
                    for(int j=0; j<2; ++j)
                        for(int k=0; k<2; ++k)
                        {
                            if(entityVectors[i][j][k])
                            {
                                const int &neighborSize = entityVectors[i][j][k]->size();
                                for(int v=0; v<neighborSize; ++v)
                                    if(aabbOverlap(entities->at(u)->getBroadphaseProxy(), entityVectors[i][j][k]->at(v)->getBroadphaseProxy()))
                                    {
                                        if(!m_pairCache->findPair(entities->at(u)->getBroadphaseProxy(), entityVectors[i][j][k]->at(v)->getBroadphaseProxy()))
                                            m_pairCache->addOverlappingPair(entities->at(u)->getBroadphaseProxy(), entityVectors[i][j][k]->at(v)->getBroadphaseProxy());
                                    }
                                    else
                                    {
                                        if(m_pairCache->findPair(entities->at(u)->getBroadphaseProxy(), entityVectors[i][j][k]->at(v)->getBroadphaseProxy()))
                                            m_pairCache->removeOverlappingPair(entities->at(u)->getBroadphaseProxy(), entityVectors[i][j][k]->at(v)->getBroadphaseProxy(), dispatcher);
                                    }
                            }
                        }
            }
        }
    }

    qDebug() << "calculateOverlappingPairs() END:" << m_pairCache->getNumOverlappingPairs();
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
                        btBroadphaseProxy *proxy = entities->at(i)->getBroadphaseProxy();

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
                        btBroadphaseProxy *proxy = entities->at(i)->getBroadphaseProxy();

                        if(proxy && TestAabbAgainstAabb2(aabbMin, aabbMax, proxy->m_aabbMin, proxy->m_aabbMax))
                            callback.process(proxy);
                    }
            }
}
