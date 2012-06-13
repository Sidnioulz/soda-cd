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

    void* mem = btAlignedAlloc(sizeof(btSortedOverlappingPairCache),16);
    m_borderCache = new (mem)btSortedOverlappingPairCache();
}

btLocalGridBroadphase::~btLocalGridBroadphase()
{
    if (m_ownsPairCache)
    {
        m_pairCache->~btOverlappingPairCache();
        btAlignedFree(m_pairCache);
    }

    m_borderCache->~btOverlappingPairCache();
    btAlignedFree(m_borderCache);
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

        m_pairCache->removeOverlappingPairsContainingProxy(proxyOrg, dispatcher);
        m_borderCache->removeOverlappingPairsContainingProxy(proxyOrg, dispatcher);
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
    qDebug() << "calculateOverlappingPairs("<<world->getId()<<")  BEGIN:" << m_pairCache->getNumOverlappingPairs() << "E-E \t&" << m_borderCache->getNumOverlappingPairs() << "E-B";
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

    // A table that stores pointers to lists of borders that possibly overlap with entities of the current Cell. 0,0,0 is not set
    const QVector<CellBorderEntity *> *borderVectors[2][2][2];
    borderVectors[0][0][0] = 0;

    // Pointers to the current Cell's entities and borders
    const QVector<obEntityWrapper *> *entities;
    const QVector<CellBorderEntity *> *borders;

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
        borders = cell.getBorders();

        if(entities)
        {
            // Cache sizes
            const int &entSize = entities->size();
            const int &borderSize = borders? borders->size() : 0;

            // Perform collision detection against static bodies first
            for(int i=0; i<entSize; ++i)
            {
                for(int j=0; j<staticEntSize; ++j)
                {
                    if(aabbOverlap(entities->at(i)->getBroadphaseHandle(), staticEnts[j]->getBroadphaseHandle()))
                    {
                        if(!m_pairCache->findPair(entities->at(i)->getBroadphaseHandle(), staticEnts[j]->getBroadphaseHandle()))
                            m_pairCache->addOverlappingPair(entities->at(i)->getBroadphaseHandle(), staticEnts[j]->getBroadphaseHandle());
                    }
                    else
                    {
                        if(m_pairCache->findPair(entities->at(i)->getBroadphaseHandle(), staticEnts[j]->getBroadphaseHandle()))
                            m_pairCache->removeOverlappingPair(entities->at(i)->getBroadphaseHandle(), staticEnts[j]->getBroadphaseHandle(), dispatcher);
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
                                borderVectors[i][j][k] = o.getBorders();
                            }
                            else
                            {
                                entityVectors[i][j][k] = 0;
                                borderVectors[i][j][k] = 0;
                            }
                        }
                    }




            // Now browse this Cell's entity vector and check each entity for collisions
            // against those of all other entity vectors
            for(int u=0; u<entSize; ++u)
            {
                // First check against entities of the same Cell (avoiding duplicates)
                for(int v=u+1; v<entSize; ++v)
                {
                    if(aabbOverlap(entities->at(u)->getBroadphaseHandle(), entities->at(v)->getBroadphaseHandle()))
                    {
                        if(!m_pairCache->findPair(entities->at(u)->getBroadphaseHandle(), entities->at(v)->getBroadphaseHandle()))
                            m_pairCache->addOverlappingPair(entities->at(u)->getBroadphaseHandle(), entities->at(v)->getBroadphaseHandle());
                    }
                    else
                    {
                        if(m_pairCache->findPair(entities->at(u)->getBroadphaseHandle(), entities->at(v)->getBroadphaseHandle()))
                            m_pairCache->removeOverlappingPair(entities->at(u)->getBroadphaseHandle(), entities->at(v)->getBroadphaseHandle(), dispatcher);
                    }
                }

                // Borders of the Cell itself, going to a special buffer
                for(int v=0; v<borderSize; ++v)
                {
                    if(aabbOverlap(entities->at(u)->getBroadphaseHandle(), borders->at(v)->getBroadphaseHandle()))
                    {
                        if(!m_borderCache->findPair(entities->at(u)->getBroadphaseHandle(), borders->at(v)->getBroadphaseHandle()))
                            m_borderCache->addOverlappingPair(entities->at(u)->getBroadphaseHandle(), borders->at(v)->getBroadphaseHandle());
                    }
                    else
                    {
                        if(m_borderCache->findPair(entities->at(u)->getBroadphaseHandle(), borders->at(v)->getBroadphaseHandle()))
                            m_borderCache->removeOverlappingPair(entities->at(u)->getBroadphaseHandle(), borders->at(v)->getBroadphaseHandle(), dispatcher);
                    }
                }

                // Then of neighboring Cells (ignoring entity/borderVectors[0][0][0] since it is set to null) - both entity and border collisions
                for(int i=0; i<2; ++i)
                    for(int j=0; j<2; ++j)
                        for(int k=0; k<2; ++k)
                        {
                            if(entityVectors[i][j][k])
                            {
                                const int &neighborSize = entityVectors[i][j][k]->size();
                                for(int v=0; v<neighborSize; ++v)
                                    if(aabbOverlap(entities->at(u)->getBroadphaseHandle(), entityVectors[i][j][k]->at(v)->getBroadphaseHandle()))
                                    {
                                        if(!m_pairCache->findPair(entities->at(u)->getBroadphaseHandle(), entityVectors[i][j][k]->at(v)->getBroadphaseHandle()))
                                            m_pairCache->addOverlappingPair(entities->at(u)->getBroadphaseHandle(), entityVectors[i][j][k]->at(v)->getBroadphaseHandle());
                                    }
                                    else
                                    {
                                        if(m_pairCache->findPair(entities->at(u)->getBroadphaseHandle(), entityVectors[i][j][k]->at(v)->getBroadphaseHandle()))
                                            m_pairCache->removeOverlappingPair(entities->at(u)->getBroadphaseHandle(), entityVectors[i][j][k]->at(v)->getBroadphaseHandle(), dispatcher);
                                    }
                            }

                            if(borderVectors[i][j][k])
                            {
                                const int &neighborSize = borderVectors[i][j][k]->size();
                                for(int v=0; v<neighborSize; ++v)
                                    if(aabbOverlap(entities->at(u)->getBroadphaseHandle(), borderVectors[i][j][k]->at(v)->getBroadphaseHandle()))
                                    {
                                        if(!m_borderCache->findPair(entities->at(u)->getBroadphaseHandle(), borderVectors[i][j][k]->at(v)->getBroadphaseHandle()))
                                            m_borderCache->addOverlappingPair(entities->at(u)->getBroadphaseHandle(), borderVectors[i][j][k]->at(v)->getBroadphaseHandle());
                                    }
                                    else
                                    {
                                        if(m_borderCache->findPair(entities->at(u)->getBroadphaseHandle(), borderVectors[i][j][k]->at(v)->getBroadphaseHandle()))
                                            m_borderCache->removeOverlappingPair(entities->at(u)->getBroadphaseHandle(), borderVectors[i][j][k]->at(v)->getBroadphaseHandle(), dispatcher);
                                    }
                            }
                        }
            }
        }
    }

    qDebug() << "calculateOverlappingPairs("<<world->getId()<<") END:  " << m_pairCache->getNumOverlappingPairs() << "E-E \t&" << m_borderCache->getNumOverlappingPairs() << "E-B";
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
                        btBroadphaseProxy *proxy = entities->at(i)->getBroadphaseHandle();

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
                        btBroadphaseProxy *proxy = entities->at(i)->getBroadphaseHandle();

                        if(proxy && TestAabbAgainstAabb2(aabbMin, aabbMax, proxy->m_aabbMin, proxy->m_aabbMax))
                            callback.process(proxy);
                    }
            }
}
