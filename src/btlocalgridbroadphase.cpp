#include "btlocalgridbroadphase.h"

#include <QVector>

//TODO: créer obEntity (mère de obEW et CBE), écrire btLGProxy, qui contient pointeur vers obEntity, ajouter pointeur vers proxy à obEntity (ou à obRigidBody??)


btLocalGridBroadphase::btLocalGridBroadphase(LocalGrid *grid, btOverlappingPairCache* overlappingPairCache) :
    grid(grid),
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

    obEntity *entity = static_cast<obEntity *>(userPtr);
    entity->setProxy(proxy0);

//    if(entity->getType() == obEntity::obEntityWrapperType)
//    {
//        obEntityWrapper *obEnt = dynamic_cast<obEntityWrapper *>(entity);
//    }

    return proxy0;
}

void btLocalGridBroadphase::destroyProxy(btBroadphaseProxy *proxyOrg, btDispatcher *dispatcher)
{
        btLocalGridProxy *proxy0 = static_cast<btLocalGridProxy*>(proxyOrg);

        proxy0->parentEntity->unsetProxy();
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
    // Don't do anything until a grid has been set
    if(!grid)
        return;

    const btVector3 &nbCells = grid->getGridInformation()->getGridAtResolution(grid->getResolution())->getNbCells();

    // A table that stores pointers to lists of entities that possibly overlap with those of the current Cell
    const QVector<obEntityWrapper *> *entityVectors[2][2][2];

    // Quicker pointer to entityVectors[0][0][0] for code readability
    const QVector<obEntityWrapper *> *entities;

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
                    // First setup a cache table telling which next cells also have entities to check against
                    for(int i=0; i<2; ++i)
                        for(int j=0; j<2; ++j)
                            for(int k=0; k<2; ++k)
                            {
                                // 0,0,0 is the current Cell.
                                if(i || j || k)
                                {
                                    if(!grid->outOfBounds(x+i, y+j, z+k))
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
                            //FIXME: should be performed on proxies?
                            if(aabbOverlap(entities->at(u)->getProxy(), entities->at(v)->getProxy()))
                            {
                                //NOTE: might be necessary to check if it's in the cache first?
                                m_pairCache->addOverlappingPair(entities->at(u)->getProxy(), entities->at(v)->getProxy());
                            }

                        // Then of neighboring Cells
                        for(int i=0; i<2; ++i)
                            for(int j=0; j<2; ++j)
                                for(int k=0; k<2; ++k)
                                {
                                    if(entityVectors[i][j][k])
                                        for(int v=0; v<entityVectors[i][j][k]->size(); ++v)
                                            //FIXME: should be performed on proxies?
                                            if(aabbOverlap(entities->at(u)->getProxy(), entityVectors[i][j][k]->at(v)->getProxy()))
                                            {
                                                //NOTE: might be necessary to check if it's in the cache first?
                                                m_pairCache->addOverlappingPair(entities->at(u)->getProxy(), entityVectors[i][j][k]->at(v)->getProxy());
                                            }
                                }
                    }
                }
            }
}

void btLocalGridBroadphase::rayTest(const btVector3 &/*rayFrom*/,const btVector3 &/*rayTo*/, btBroadphaseRayCallback &rayCallback, const btVector3 &/*aabbMin*/, const btVector3 &/*aabbMax*/)
{
    // Don't do anything until a grid has been set
    if(!grid)
        return;

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
                        btLocalGridProxy *proxy = entities->at(i)->getProxy();

                        if(proxy)
                            rayCallback.process(proxy);
                    }
            }
}

void btLocalGridBroadphase::aabbTest(const btVector3 &aabbMin, const btVector3 &aabbMax, btBroadphaseAabbCallback &callback)
{
    // Don't do anything until a grid has been set
    if(!grid)
        return;

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
                        btLocalGridProxy *proxy = entities->at(i)->getProxy();

                        if(proxy && TestAabbAgainstAabb2(aabbMin, aabbMax, proxy->m_aabbMin, proxy->m_aabbMax))
                            callback.process(proxy);
                    }
            }
}
