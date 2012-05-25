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
 *
 *TODO: write in copyright that some code here is from Bullet!
 */
#include <iostream>
#include <QtDebug>
#include <btBulletCollisionCommon.h>
#include "bulletmanager.h"
#include "obEntityWrapper.h"
#include "cellborderentity.h"
#include "btlocalgridbroadphase.h"

using namespace std;

BulletManagerWorld::BulletManagerWorld(btCollisionDispatcher *&dispatcher, btLocalGridBroadphase *&broadphase, btSequentialImpulseConstraintSolver *&solver, btDefaultCollisionConfiguration *&config) :
    btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config) //FIXME:
{
}

int BulletManagerWorld::stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
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

void BulletManagerWorld::internalSingleStepSimulation(btScalar timeStep)
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

    performDiscreteCollisionDetection();




//TODO: later, cache synced neighbors

/*
    [...]

    performDiscreteCollisionDetection();

    List of Collision pairs borderCols = checkCollisionsWithBorders();

    // Synchronized version
    if(borderCols is not empty)
    {

        List of Worlds neighbors = findInvolvedNeighbors()

        waitForNeighbors(neighbors) // Case where we are the most advanced


        foreach(object o in borderCols)
        {
            synchronizeCellInformationAtBorder(n, borderCols) // This function may add border-crossing objects to borderCols, that are owned by n and also crossing the border at the same time.


        }

        foreach(object o in borderCols)
        {
            detectCollisionsAround(o);
        }


        calculateSimulationIslands();
        syncWithNeighborIslands(neighbors);

        foreach(island in islands)
        {
            if (world accountable for island solving)
            solveConstraints(island)
        }

        syncSolvedIslands(neighbors);

        solveConstraints() // for normal objects


    }
    // No need to synchronize
    else
    {
        calculateSimulationIslands();
        [...]
        solveConstraints();
    }

    [...]

*/

//    int numManifolds = this->getDispatcher()->getNumManifolds();
//    for (int i=0;i<numManifolds;i++)
//    {
//        btPersistentManifold* contactManifold =  this->getDispatcher()->getManifoldByIndexInternal(i);
//        btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
//        btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());



//        int numContacts = contactManifold->getNumContacts();
//        for (int j=0;j<numContacts;j++)
//        {
//            btManifoldPoint& pt = contactManifold->getContactPoint(j);
//            if (pt.getDistance()<0.f)
//            {
//                const btVector3& ptA = pt.getPositionWorldOnA();
//                const btVector3& ptB = pt.getPositionWorldOnB();
//                const btVector3& normalOnB = pt.m_normalWorldOnB;

    //TODO: obEntity class, static_cast to obEntity, then dynamic_cast to son instances

//                qDebug() << obA->getCollisionShape()->getUserPointer();
//                qDebug() << obB->getCollisionShape()->getUserPointer();

////                obEntityWrapper *obEntA = dynamic_cast<obEntityWrapper *>(obA->getCollisionShape()->getUserPointer());
////                if(obEntA)
////                    qDebug() << "obEntA is " << obEntA->getName().c_str();
////                else
////                {
////                    CellBorderEntity *cbeA = dynamic_cast<CellBorderEntity *>(obA->getCollisionShape()->getUserPointer());
////                    if(cbeA)
////                        qDebug() << "cellBorderA is " << cbeA->getCoordinates().x() << cbeA->getCoordinates().y() << cbeA->getCoordinates().z();

////                }


////                obEntityWrapper *obEntB = dynamic_cast<obEntityWrapper *>(obB->getCollisionShape()->getUserPointer());
////                qDebug() << "obEntB is " << obEntB->getName().c_str();
////                if(obEntB)
////                    qDebug() << "obEntB is " << obEntB->getName().c_str();
////                else
////                {
////                    CellBorderEntity *cbeB = dynamic_cast<CellBorderEntity *>(obB->getCollisionShape()->getUserPointer());
////                    if(cbeB)
////                        qDebug() << "cellBorderB is " << cbeB->getCoordinates().x() << cbeB->getCoordinates().y() << cbeB->getCoordinates().z();

////                }

//            }
//        }
//    }



    calculateSimulationIslands();

    getSolverInfo().m_timeStep = timeStep;

    solveConstraints(getSolverInfo());

    integrateTransforms(timeStep);

    updateActions(timeStep);

    updateActivationState( timeStep );

    if(0 != m_internalTickCallback)
    {
        (*m_internalTickCallback)(this, timeStep);
    }
}


BulletManager::BulletManager() :
    broadphase(0),
    collisionConfig(0),
    dispatcher(0),
    solver(0),
    dynamicsWorld(0)
{
    broadphase = new btLocalGridBroadphase();
    collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    solver = new btSequentialImpulseConstraintSolver; //TODO parallelise

    // Create the world using all above algorithms
    dynamicsWorld = new BulletManagerWorld(dispatcher, broadphase, solver, collisionConfig);

    // Set the gravity orientation of the world (here 'y' is upwards)
    dynamicsWorld->setGravity(btVector3(0,-10,0));
}

BulletManager::~BulletManager()
{
    // Cleanup world
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfig;
    delete broadphase;
}
