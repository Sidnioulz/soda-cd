/*
 *** Some methods of these classes are derived from Bullet Physics:
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2011 Erwin Coumans  http://bulletphysics.org
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *** The code that differs from the original source is licensed as is:
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
#include <btBulletCollisionCommon.h>
#include "sodaDynamicsWorld.h"
#include "sodaSimulationIslandManager.h"

//TODO: redefine btCollisionWorld::convexSweepTest so that non-owned objects are not tested when not using the broadphase brute-force

namespace {
    SIMD_FORCE_INLINE	int	btGetConstraintIslandId(const btTypedConstraint* lhs)
    {
        int islandId;

        const btCollisionObject& rcolObj0 = lhs->getRigidBodyA();
        const btCollisionObject& rcolObj1 = lhs->getRigidBodyB();
        islandId= rcolObj0.getIslandTag()>=0?rcolObj0.getIslandTag():rcolObj1.getIslandTag();
        return islandId;

    }

    class btSortConstraintOnIslandPredicate
    {
        public:

            bool operator() ( const btTypedConstraint* lhs, const btTypedConstraint* rhs )
            {
                int rIslandId0,lIslandId0;
                rIslandId0 = btGetConstraintIslandId(rhs);
                lIslandId0 = btGetConstraintIslandId(lhs);
                return lIslandId0 < rIslandId0;
            }
    };
}

sodaDynamicsWorld::sodaDynamicsWorld(sodaLogicWorld *logicWorld, btCollisionDispatcher *&dispatcher, sodaLocalGridBroadphase *&broadphase, btSequentialImpulseConstraintSolver *&solver, btDefaultCollisionConfiguration *&config) :
//    btDiscreteDynamicsWorld(dispatcher, new btSimpleBroadphase(), solver, config)
    btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config),
    cb(this),
    logicWorld(logicWorld),
    broadphase(broadphase),
    m_objectOwnerIdCache(),
    aborted(false)
{
    void* mem = btAlignedAlloc(sizeof(sodaSimulationIslandManager),16);
    m_islandManager = new (mem) sodaSimulationIslandManager(this);
}

sodaDynamicsWorld::~sodaDynamicsWorld()
{
    //only delete it when we created it
    if (m_ownsIslandManager)
    {
        m_islandManager->~sodaSimulationIslandManager();
        btAlignedFree( m_islandManager);
    }
    if (m_ownsConstraintSolver)
    {
        m_constraintSolver->~btConstraintSolver();
        btAlignedFree(m_constraintSolver);
    }

    //clean up remaining objects
    int i;
    for (i=0;i<m_collisionObjects.size();i++)
    {
        btCollisionObject* collisionObject= m_collisionObjects[i];

        if(ownsCollisionObject(collisionObject))
        {
            btBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
            if (bp)
            {
                //
                // only clear the cached algorithms
                //
                getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
                getBroadphase()->destroyProxy(bp,m_dispatcher1);
                collisionObject->setBroadphaseHandle(0);
            }
        }
    }
}

int sodaDynamicsWorld::stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{

    startProfiling(timeStep);

    BT_PROFILE("stepSimulation");

    int numSimulationSubSteps = 0;

    if (maxSubSteps)
    {
        //fixed timestep with interpolation
        m_localTime += timeStep;
        if (m_localTime >= fixedTimeStep)
        {
            numSimulationSubSteps = int( m_localTime / fixedTimeStep);
            m_localTime -= numSimulationSubSteps * fixedTimeStep;
        }
    } else
    {
        //variable timestep
        fixedTimeStep = timeStep;
        m_localTime = timeStep;
        if (btFuzzyZero(timeStep))
        {
            numSimulationSubSteps = 0;
            maxSubSteps = 0;
        } else
        {
            numSimulationSubSteps = 1;
            maxSubSteps = 1;
        }
    }

    //process some debugging flags
    if(getDebugDrawer())
    {
        btIDebugDraw* debugDrawer = getDebugDrawer ();
        gDisableDeactivation = (debugDrawer->getDebugMode() & btIDebugDraw::DBG_NoDeactivation) != 0;
    }
    if(numSimulationSubSteps)
    {
        saveKinematicState(fixedTimeStep);

        applyGravity();

        //clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
        int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps)? maxSubSteps : numSimulationSubSteps;

        for(int i=0;i<clampedSimulationSteps;i++)
        {
            internalSingleStepSimulation(fixedTimeStep);

            // internalSingleStepSimulation contains blocking calls, so we need to abort if the aborted flag was turned on
            if(aborted)
                return -1;

            synchronizeMotionStates();
        }

    }
    else
    {
        synchronizeMotionStates();
    }

    clearForces();

#ifndef BT_NO_PROFILE
    CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE

    return numSimulationSubSteps;
}

void sodaDynamicsWorld::readExternalSyncRequests(const btScalar &simulTime)
{
#ifndef NDEBUG
    qDebug() << "BulletManagerWorld(" << getWorldId() << ")::readExternalSyncRequests(" << simulTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    logicWorld->_readExternalSyncRequests(borderTraversedNeighbors, simulTime);
}

void sodaDynamicsWorld::waitForNeighbors(const btScalar &simulTime)
{
#ifndef NDEBUG
    const QList<short> &neighbors = borderTraversedNeighbors.keys();
    QString neighborsListStr("Neighbors: ");
    for(int i=0; i<neighbors.size(); ++i)
        neighborsListStr.append(QString("%1, ").arg(neighbors.at(i)));

    qDebug() << "BulletManagerWorld(" << getWorldId() << ")::waitForNeighbors(" << neighborsListStr << "Time: " << simulTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    aborted |= logicWorld->_waitForNeighbors(borderTraversedNeighbors.keys(), simulTime, borderTraversedNeighbors);
}

void sodaDynamicsWorld::waitEndStep(const QList<short> &neighbors, const btScalar &simulTime)
{
#ifndef NDEBUG
    QString neighborsListStr("Neighbors: ");
    for(int i=0; i<neighbors.size(); ++i)
        neighborsListStr.append(QString("%1, ").arg(neighbors.at(i)));

    qDebug() << "BulletManagerWorld(" << getWorldId() << ")::waitEndFrame(" << neighborsListStr << "Time: " << simulTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    aborted |= logicWorld->_waitEndStep(neighbors, simulTime);
}

int	sodaDynamicsWorld::getNumCollisionObjects() const
{
    int num=0;

    QHash<const btCollisionObject *, short>::const_iterator it = m_objectOwnerIdCache.begin();
    for(; it!=m_objectOwnerIdCache.end(); ++it)
    {
        if(it.value() == getWorldId())
            ++num;
    }

    return num;
}

void sodaDynamicsWorld::saveKinematicState(btScalar timeStep)
{
    for (int i=0;i<m_collisionObjects.size();i++)
    {
        btCollisionObject* colObj = m_collisionObjects[i];
        if(ownsCollisionObject(colObj))
        {
            btRigidBody* body = btRigidBody::upcast(colObj);
            if (body && body->getActivationState() != ISLAND_SLEEPING)
            {
                if (body->isKinematicObject())
                {
                    //to calculate velocities next frame
                    body->saveKinematicState(timeStep);
                }
            }
        }
    }
}

void sodaDynamicsWorld::synchronizeMotionStates()
{
    BT_PROFILE("synchronizeMotionStates");
    if (m_synchronizeAllMotionStates)
    {
        //iterate  over all collision objects
        for ( int i=0;i<m_collisionObjects.size();i++)
        {
            btCollisionObject* colObj = m_collisionObjects[i];
            if(ownsCollisionObject(colObj))
            {
                btRigidBody* body = btRigidBody::upcast(colObj);
                if (body)
                    synchronizeSingleMotionState(body);
            }
        }
    } else
    {
        //iterate over all active rigid bodies
        for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
        {
            btRigidBody* body = m_nonStaticRigidBodies[i];
            if (body->isActive())
                synchronizeSingleMotionState(body);
        }
    }
}

void sodaDynamicsWorld::serializeRigidBodies(btSerializer *serializer)
{
    int i;
    //serialize all collision objects
    for (i=0;i<m_collisionObjects.size();i++)
    {
        btCollisionObject* colObj = m_collisionObjects[i];
        if (ownsCollisionObject(colObj) && colObj->getInternalType() & btCollisionObject::CO_RIGID_BODY)
        {
            int len = colObj->calculateSerializeBufferSize();
            btChunk* chunk = serializer->allocate(len,1);
            const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
            serializer->finalizeChunk(chunk,structType,BT_RIGIDBODY_CODE,colObj);
        }
    }

    for (i=0;i<m_constraints.size();i++)
    {
        btTypedConstraint* constraint = m_constraints[i];
        int size = constraint->calculateSerializeBufferSize();
        btChunk* chunk = serializer->allocate(size,1);
        const char* structType = constraint->serialize(chunk->m_oldPtr,serializer);
        serializer->finalizeChunk(chunk,structType,BT_CONSTRAINT_CODE,constraint);
    }
}

void sodaDynamicsWorld::_addToForeignerMap(sodaDynamicEntity const * const foreigner, const CellBorderCoordinates &coord, QMap<btVector3, QSet<sodaDynamicEntity const *> > &foreignersPerCellMap)
{
#ifndef NDEBUG
    qDebug() << "BulletManagerWorld(" << getWorldId() << ")::_addToForeignerMap(" << foreigner->getDisplayName() << ", Coord:[" << coord.x() << coord.y() << coord.z() << "], ...); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    // Use the direction of the border with the foreigner's Cell to mask
    // surrounding cells in which the foreigner needs not be checked for collisions.
    int x0=-1, y0=-1, z0=-1;
    int x1=1, y1=1, z1=1;

    //TODO: manage corners once multi-directional CellBorderCoordinates are available

    if(coord.direction() == GridInformation::Left)
        x0=0;
    else if(coord.direction() == GridInformation::Right)
        x1=0;
    else if(coord.direction() == GridInformation::Bottom)
        y0=0;
    else if(coord.direction() == GridInformation::Top)
        y1=0;
    else if(coord.direction() == GridInformation::Back)
        z0=0;
    else if(coord.direction() == GridInformation::Front)
        z1=0;

    // Add the foreigner to the map for all locally owned Cells
    for(int x=x0; x<=x1; ++x)
        for(int y=y0; y<=y1; ++y)
            for(int z=z0; z<=z1; ++z)
            {
                btVector3 tmpCoord = coord+btVector3(x,y,z);

                if(!logicWorld->getLocalGrid()->cellNotOwnedBySelf(tmpCoord))
                {
                    QSet<sodaDynamicEntity const *> foreignersInCell = foreignersPerCellMap.value(tmpCoord, QSet<sodaDynamicEntity const *>());
                    foreignersInCell.insert(foreigner);
                    foreignersPerCellMap.insert(tmpCoord, foreignersInCell);
                }
            }
}

void sodaDynamicsWorld::internalSingleStepSimulation(btScalar timeStep)
{
    BT_PROFILE("internalSingleStepSimulation");

    if(0 != m_internalPreTickCallback)
    {
        (*m_internalPreTickCallback)(this, timeStep);
    }

    predictUnconstraintMotion(timeStep);

    btDispatcherInfo& dispatchInfo = getDispatchInfo();

    dispatchInfo.m_timeStep = timeStep;
    dispatchInfo.m_stepCount = 0;
    dispatchInfo.m_debugDraw = getDebugDrawer();

    // Perform local detections and fill borderTraversedNeighbors with local border collisions
    performDiscreteCollisionDetection();

    //TODO: postprocess border traversals here to find special corner cases

    // Add notified remote border collisions
    readExternalSyncRequests(this->m_localTime);

    // Only wait for direct neighbors which require synchronization, if border traversals
    if(!borderTraversedNeighbors.isEmpty())
    {
        // Blocking call - cancel current simulation if aborted
        waitForNeighbors(this->m_localTime);
        if(aborted)
            return;


#ifndef NDEBUG
        qDebug() << "BulletManagerWorld(" << getWorldId() << ")::internalSingleStepSimulation(" << m_localTime << "); borderTraversedNeighbors contains " << borderTraversedNeighbors.size() << "neighbors; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        QMap<btVector3, QSet<sodaDynamicEntity const *> > foreignersPerCellMap;

        // Read through borderTraversedNeighbors to get lists of colliding objects, for each cell
        QHashIterator<short, EntityOverlappedCellsMap> neighborIter(borderTraversedNeighbors);
        while(neighborIter.hasNext())
        {
            const EntityOverlappedCellsMap &map = neighborIter.next().value();
#ifndef NDEBUG
            const short &nId = neighborIter.key();
            qDebug() << "BulletManagerWorld(" << getWorldId() << ")::internalSingleStepSimulation(" << m_localTime << "); borderTraversedNeighbors[" << nId << "] contains " << map.size() << "entities; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

            QHashIterator<sodaDynamicEntity *, QVector<CellBorderCoordinates> > entityIter(map);

            while(entityIter.hasNext())
            {
                entityIter.next();
                sodaDynamicEntity const * const foreigner = entityIter.key();
                const QVector<CellBorderCoordinates> &foreignerCoords = entityIter.value();

                // If this entity is external, make sure it will be checked for collisions in its cells
                if(foreigner->getOwnerId() != getWorldId())
                {
                    foreach(const CellBorderCoordinates &coord, foreignerCoords)
                        _addToForeignerMap(foreigner, coord, foreignersPerCellMap);
                }
            }
        }

        //FIXME: removed objects should not be in cb because they should not be called in contactPairTest - problem normally superseded by onEndFrame

        // Perform pairwise collision detections between all foreign entities in
        // foreignersPerCellMap and all local entities within same Cells.
        sodaLocalGrid *lg = logicWorld->getLocalGrid();
        QMapIterator<btVector3, QSet<sodaDynamicEntity const *> > perCellIter(foreignersPerCellMap);
        while(perCellIter.hasNext())
        {
            perCellIter.next();

            const btVector3 &coord = perCellIter.key();
            const QSet<sodaDynamicEntity const *> &foreigners = perCellIter.value();

            const Cell &c = lg->at(coord);
            if(c.getEntities() != 0)
                foreach(sodaDynamicEntity *local, *c.getEntities())
                    foreach(sodaDynamicEntity const *foreigner, foreigners)
                    {
                        cb.resetCollisionFlag();
                        contactPairTest(local->getRigidBody()->getBulletBody(), foreigner->getRigidBody()->getBulletBody(), cb);

#ifndef NDEBUG
                        if(cb.getCollisionFlag())
                        {
                            qDebug() << "sodaDynamicsWorld::internalSingleStepSimulation(" << timeStep << "); Collision between " << local->getDisplayName() << " and " << foreigner->getDisplayName();
                        }
#endif
                    }
        }

        // Remove from the ContactResultCallback's manifolds the objects that do not collide anymore
        {
            btAlignedObjectArray<btCollisionObject *> oldObjects = cb.removeOutdatedManifolds(m_localTime);
            int i = 0, arraySize = oldObjects.size();
            for(; i<arraySize; ++i)
                removeCollisionObject(oldObjects[i]);

            // From the ContactResultCallback, add into the world all foreign btCollisionObjects
            const btAlignedObjectArray<sodaPersistentForeignerManifold *> &manifolds = cb.getManifolds();
            i = 0, arraySize = manifolds.size();
            for(; i<arraySize; ++i)
            {
                btCollisionObject *foreigner = (btCollisionObject *) manifolds[i]->getForeignBody();
                sodaDynamicEntity *obEnt = dynamic_cast<sodaDynamicEntity *>(static_cast<sodaEntity *>(foreigner->getUserPointer()));

                if(obEnt)
                {
                    if(!m_objectOwnerIdCache.contains(foreigner))
                        addForeignCollisionObject(obEnt->getOwnerId(), foreigner);
                }
                else
                    qWarning() << "BulletManagerWorld(" << getWorldId() << ")::internalSingleStepSimulation(" << m_localTime << "); Could not find entity attached to btCollisionObject" << foreigner << "; Thread " << QString().sprintf("%p", QThread::currentThread());
            }



        }

#ifndef NDEBUG
        qDebug() << "BulletManagerWorld(" << getWorldId() << ")::internalSingleStepSimulation(" << m_localTime << "); Detected " << cb.getManifolds().size() << " collisions with foreigners; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

        calculateSimulationIslands();





//        foreach(island in islands)
//        {
//            if(islandContainsForeigner())
//            mergeIslandWithForeign(island)
//        }

        //TODO: Parallel version of solveConstraints for all islands existing across worlds, and sequential solver for the local ones
        getSolverInfo().m_timeStep = timeStep;
        solveConstraints(getSolverInfo());

        //TODO: integrate transfer of objects in this sync'd function. We have positions after integrateTransforms
        integrateTransforms(timeStep);

        //TODO: cleanup cb
        //TODO: later, cache synced neighbors

        // Final sync to avoid damages because of race conditions
        waitEndStep(borderTraversedNeighbors.keys(), this->m_localTime);

    }
    // No need to synchronize
    else
    {
        calculateSimulationIslands();
        getSolverInfo().m_timeStep = timeStep;
        solveConstraints(getSolverInfo());
        integrateTransforms(timeStep);
    }

    updateActions(timeStep);

    //NOTE: might require checking and warning of neighbor on foreign objects
    updateActivationState(timeStep);

    if(0 != m_internalTickCallback)
    {
        (*m_internalTickCallback)(this, timeStep);
    }
}

void sodaDynamicsWorld::solveConstraints(btContactSolverInfo &solverInfo)
{
    BT_PROFILE("solveConstraints");

    struct InplaceSolverIslandCallback : public sodaSimulationIslandManager::IslandCallback
    {

        btContactSolverInfo&	m_solverInfo;
        btConstraintSolver*		m_solver;
        btTypedConstraint**		m_sortedConstraints;
        int						m_numConstraints;
        btIDebugDraw*			m_debugDrawer;
        btStackAlloc*			m_stackAlloc;
        btDispatcher*			m_dispatcher;

        btAlignedObjectArray<btCollisionObject*> m_bodies;
        btAlignedObjectArray<btPersistentManifold*> m_manifolds;
        btAlignedObjectArray<btTypedConstraint*> m_constraints;


        InplaceSolverIslandCallback(
            btContactSolverInfo& solverInfo,
            btConstraintSolver*	solver,
            btTypedConstraint** sortedConstraints,
            int	numConstraints,
            btIDebugDraw*	debugDrawer,
            btStackAlloc*			stackAlloc,
            btDispatcher* dispatcher)
            :m_solverInfo(solverInfo),
            m_solver(solver),
            m_sortedConstraints(sortedConstraints),
            m_numConstraints(numConstraints),
            m_debugDrawer(debugDrawer),
            m_stackAlloc(stackAlloc),
            m_dispatcher(dispatcher)
        {

        }


        InplaceSolverIslandCallback& operator=(InplaceSolverIslandCallback& other)
        {
            btAssert(0);
            (void)other;
            return *this;
        }
        virtual	void ProcessIsland(btCollisionObject** bodies,int numBodies,btPersistentManifold**	manifolds,int numManifolds, int islandId)
        {
            if (islandId<0)
            {
                if (numManifolds + m_numConstraints)
                {
                    ///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
                    m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,&m_sortedConstraints[0],m_numConstraints,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
                }
            } else
            {
                    //also add all non-contact constraints/joints for this island
                btTypedConstraint** startConstraint = 0;
                int numCurConstraints = 0;
                int i;

                //find the first constraint for this island
                for (i=0;i<m_numConstraints;i++)
                {
                    if (btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
                    {
                        startConstraint = &m_sortedConstraints[i];
                        break;
                    }
                }
                //count the number of constraints in this island
                for (;i<m_numConstraints;i++)
                {
                    if (btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
                    {
                        numCurConstraints++;
                    }
                }

                if (m_solverInfo.m_minimumSolverBatchSize<=1)
                {
                    ///only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
                    if (numManifolds + numCurConstraints)
                    {
                        m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,startConstraint,numCurConstraints,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
                    }
                } else
                {

                    for (i=0;i<numBodies;i++)
                        m_bodies.push_back(bodies[i]);
                    for (i=0;i<numManifolds;i++)
                        m_manifolds.push_back(manifolds[i]);
                    for (i=0;i<numCurConstraints;i++)
                        m_constraints.push_back(startConstraint[i]);
                    if ((m_constraints.size()+m_manifolds.size())>m_solverInfo.m_minimumSolverBatchSize)
                    {
                        processConstraints();
                    } else
                    {
                        //printf("deferred\n");
                    }
                }
            }
        }
        void	processConstraints()
        {
            if (m_manifolds.size() + m_constraints.size()>0)
            {

                btCollisionObject** bodies = m_bodies.size()? &m_bodies[0]:0;
                btPersistentManifold** manifold = m_manifolds.size()?&m_manifolds[0]:0;
                btTypedConstraint** constraints = m_constraints.size()?&m_constraints[0]:0;

                m_solver->solveGroup( bodies,m_bodies.size(),manifold, m_manifolds.size(),constraints, m_constraints.size() ,m_solverInfo,m_debugDrawer,m_stackAlloc,m_dispatcher);
            }
            m_bodies.resize(0);
            m_manifolds.resize(0);
            m_constraints.resize(0);

        }

    };



    //sorted version of all btTypedConstraint, based on islandId
    btAlignedObjectArray<btTypedConstraint*>	sortedConstraints;
    sortedConstraints.resize( m_constraints.size());
    int i;
    for (i=0;i<getNumConstraints();i++)
    {
        sortedConstraints[i] = m_constraints[i];
    }

//	btAssert(0);



    sortedConstraints.quickSort(btSortConstraintOnIslandPredicate());

    btTypedConstraint** constraintsPtr = getNumConstraints() ? &sortedConstraints[0] : 0;

    InplaceSolverIslandCallback	solverCallback(	solverInfo,	m_constraintSolver, constraintsPtr,sortedConstraints.size(),	m_debugDrawer,m_stackAlloc,m_dispatcher1);

    m_constraintSolver->prepareSolve(getNumCollisionObjects(), getDispatcher()->getNumManifolds());

    /// solve all the constraints for this island
    m_islandManager->buildAndProcessIslands(getDispatcher(), this, &solverCallback);

    solverCallback.processConstraints();

    m_constraintSolver->allSolved(solverInfo, m_debugDrawer, m_stackAlloc);
}


void sodaDynamicsWorld::calculateSimulationIslands()
{
    BT_PROFILE("calculateSimulationIslands");

    m_islandManager->updateActivationState(this, getDispatcher());

    {
        int i;
        int numConstraints = int(m_constraints.size());
        for (i=0;i< numConstraints ; i++ )
        {
            btTypedConstraint* constraint = m_constraints[i];
            if (constraint->isEnabled())
            {
                //FIXME: make this take into account origin of rigid body for getIslandTag()
                const btRigidBody* colObj0 = &constraint->getRigidBodyA();
                const btRigidBody* colObj1 = &constraint->getRigidBodyB();

                if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
                ((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
                {
                    if (colObj0->isActive() || colObj1->isActive())
                    {
                        m_islandManager->getUnionFind().unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
                    }
                }
            }
        }
    }

    // Integrate foreigner entities into islands and merge islands if necessary
    {

    }

    //TODO: islandManager should flag islands with foreigners, and an island should explicitely be able to deliver the info

    //Store the island id in each body
    m_islandManager->storeIslandActivationState(this);
}

void sodaDynamicsWorld::addCollisionObject(btCollisionObject *collisionObject, short int collisionFilterGroup, short int collisionFilterMask)
{
//    qDebug() << "addCollisionObject" << QThread::currentThreadId();
    btAssert(collisionObject);

    //check that the object isn't already added
    btAssert( m_collisionObjects.findLinearSearch(collisionObject)  == m_collisionObjects.size());

    m_collisionObjects.push_back(collisionObject);
    m_objectOwnerIdCache.insert(collisionObject, getWorldId());

    //calculate new AABB
    btTransform trans = collisionObject->getWorldTransform();

    btVector3	minAabb;
    btVector3	maxAabb;
    collisionObject->getCollisionShape()->getAabb(trans,minAabb,maxAabb);

    int type = collisionObject->getCollisionShape()->getShapeType();
    collisionObject->setBroadphaseHandle( getBroadphase()->createProxy(
        minAabb,
        maxAabb,
        type,
        collisionObject,
        collisionFilterGroup,
        collisionFilterMask,
        m_dispatcher1,0
        ));
}

void sodaDynamicsWorld::addForeignCollisionObject(const short &foreignId, btCollisionObject *collisionObject)
{
#ifndef NDEBUG
    qDebug() << "BulletManagerWorld(" << getWorldId() << ")::addForeignCollisionObject(" << foreignId << ", " << collisionObject << ", " << m_localTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    btAssert(collisionObject);

    //check that the object isn't already added
    btAssert( m_collisionObjects.findLinearSearch(collisionObject)  == m_collisionObjects.size());

    m_collisionObjects.push_back(collisionObject);
    m_objectOwnerIdCache.insert(collisionObject, foreignId);
}

void sodaDynamicsWorld::removeCollisionObject(btCollisionObject *collisionObject)
{
//    qDebug() << "removeCollisionObject" << QThread::currentThreadId();
    //bool removeFromBroadphase = false;

    if(ownsCollisionObject(collisionObject))
    {
        btBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
        if (bp)
        {
            //
            // only clear the cached algorithms
            //
            getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
            getBroadphase()->getBorderCrossingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
            getBroadphase()->destroyProxy(bp,m_dispatcher1);
            collisionObject->setBroadphaseHandle(0);
        }
    }

    //swapremove
    m_collisionObjects.remove(collisionObject);
    m_objectOwnerIdCache.remove(collisionObject);
}

void sodaDynamicsWorld::updateAabbs()
{
    BT_PROFILE("updateAabbs");

//    btTransform predictedTrans;
    for ( int i=0;i<m_collisionObjects.size();i++)
    {
        btCollisionObject* colObj = m_collisionObjects[i];

        //only update aabb of active objects
        if ((m_forceUpdateAllAabbs || colObj->isActive()) && ownsCollisionObject(colObj))
        {
            updateSingleAabb(colObj);
        }
    }
}

void sodaDynamicsWorld::performDiscreteCollisionDetection()
{
    BT_PROFILE("performDiscreteCollisionDetection");

    btDispatcherInfo& dispatchInfo = getDispatchInfo();

    updateAabbs();

    {
        BT_PROFILE("calculateOverlappingPairs");
        m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);
    }


    btDispatcher* dispatcher = getDispatcher();
    {
        BT_PROFILE("dispatchAllCollisionPairs");
        if (dispatcher)
            dispatcher->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(),dispatchInfo,m_dispatcher1);
    }


    {
        BT_PROFILE("listOverlappingBorders");
        borderTraversedNeighbors.clear();

        sodaLocalGridBroadphase *bdPhase = static_cast<sodaLocalGridBroadphase *>(m_broadphasePairCache);
        const btBroadphasePairArray &array = bdPhase->getBorderCrossingPairCache()->getOverlappingPairArray();

        sodaDynamicEntity *obEnt = 0;
        CellBorderEntity *border = 0;
        CellBorderCoordinates otherSideCoords;

        sodaLocalGrid *localGrid = logicWorld->getLocalGrid();
        Q_ASSERT(localGrid != 0);

        const int &bdArraySize = array.size();
        for(int i=0; i<bdArraySize; ++i)
        {
            btCollisionObject *collObj0 = static_cast<btCollisionObject *>(array[i].m_pProxy0->m_clientObject);
            sodaEntity *entity0 = static_cast<sodaEntity *>(collObj0->getUserPointer());
            if(entity0->getType() == sodaEntity::DynamicEntityType)
            {
                obEnt = dynamic_cast<sodaDynamicEntity *>(entity0);
//                qDebug() << "performDiscreteCollisionDetection("<< world->getId() <<"):" << obEnt->getDisplayName() << "collides with a border";
                obEnt->setStatus(sodaEntity::CrossingBorder);
            }
            else
            {
                border = dynamic_cast<CellBorderEntity *>(entity0);
                border->setStatus(sodaEntity::Overlapped);
            }

            btCollisionObject *collObj1 = static_cast<btCollisionObject *>(array[i].m_pProxy1->m_clientObject);
            sodaEntity *entity1 = static_cast<sodaEntity *>(collObj1->getUserPointer());
            if(entity1->getType() == sodaEntity::DynamicEntityType)
            {
                obEnt = dynamic_cast<sodaDynamicEntity *>(entity1);
//                qDebug() << "performDiscreteCollisionDetection("<< world->getId() <<"):" << obEnt->getDisplayName() << "collides with a border";
                obEnt->setStatus(sodaEntity::CrossingBorder);
            }
            else
            {
                border = dynamic_cast<CellBorderEntity *>(entity1);
                border->setStatus(sodaEntity::Overlapped);
            }

            const CellBorderCoordinates &coord = border->getCoordinates();
            coord.getOtherSide(otherSideCoords);

            // This is not a global space border but a border between worlds, append the entity to that neighbor's list
            if(localGrid->getGridInformation()->isWithinWorldCellBounds(otherSideCoords))
            {
                const short &neighborId = localGrid->at(otherSideCoords).getOwnerId();
                QHash<sodaDynamicEntity *, QVector<CellBorderCoordinates> > mapForNeighbor = borderTraversedNeighbors.value(neighborId, QHash<sodaDynamicEntity *, QVector<CellBorderCoordinates> >());

                QVector<CellBorderCoordinates> coordsForEnt = mapForNeighbor.value(obEnt, QVector<CellBorderCoordinates>());
                coordsForEnt.append(otherSideCoords);
                mapForNeighbor.insert(obEnt, coordsForEnt);

                borderTraversedNeighbors.insert(neighborId, mapForNeighbor);
            }
        }

        //Send a message to all neighbors having a traversed border
        QHash<short, QHash<sodaDynamicEntity *, QVector<CellBorderCoordinates> > >::const_iterator it = borderTraversedNeighbors.begin();
        while(it!=borderTraversedNeighbors.end())
        {
            logicWorld->messageNeighbor(it.key(),
                                   "onBorderTraversed",
                                   Q_ARG(sodaLogicWorld *, logicWorld),
                                   Q_ARG(EntityOverlappedCellsMap, it.value()),
                                   Q_ARG(btScalar, logicWorld->getCurrentTime()));

            it++;
        }
    }
}
