/*
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Quentin Avril <quentin.avril@inria.fr>
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
#include "obMotionState.h"
#include "obdynamicrigidbody.h"
#include "physicsworld.h"
#include "localgrid.h"
#include "utils.h"

obMotionState::obMotionState(obEntityWrapper *parent) : parentBody(parent), grid(0)
{
}

void obMotionState::setLocalGrid(LocalGrid *newGrid)
{
    grid = newGrid;
    btVector3 coords = grid->getGridInformation()->toCellCoordinates(grid->getResolution(), parentBody->getCenteredPosition());
	if(grid->at(coords).containsEntity(parentBody))
        lastCellCoords = coords;
    else
        lastCellCoords = btVector3(INFINITY, INFINITY, INFINITY);
}

void obMotionState::unsetLocalGrid()
{
    grid = 0;
}

void obMotionState::getWorldTransform(btTransform &/*worldTrans*/) const
{
#ifndef NDEBUG
//    qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : PhysicsWorld::NullWorldId) << ")::getWorldTransform(...); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
}

void obMotionState::setWorldTransform(const btTransform &worldTrans)
{
#ifndef NDEBUG
//    qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : PhysicsWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    if(grid && parentBody)
    {
        btVector3 coords = grid->getGridInformation()->toCellCoordinates(grid->getResolution(), parentBody->getCenteredPosition());
        if(coords != lastCellCoords)
        {
            if(grid->cellNotOwnedBySelf(coords))
            {

                // Cell coordinates are outside of the simulation scene boundaries (because margin ensures we don't run out of bounds when there is a neighbor)
                if(grid->cellOutOfBounds(coords))
                {
                    qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : PhysicsWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << ");" <<
                          "Entity '" << parentBody->getDisplayName() <<
                          "' owned by '" << parentBody->getOwnerWorld() <<
                          "' in Grid '" << grid->getOwnerId() <<
                          "' (" << lastCellCoords.x() << "," << lastCellCoords.y() << "," << lastCellCoords.z() << ")" <<
                          "  is out of bounds (" << coords.x() << "," << coords.y() << "," << coords.z() << ", LG bounds: " << grid->displayBoundsInfo() << ")" <<
                          "; Thread " << QString().sprintf("%p", QThread::currentThread());

                    parentBody->setStatus(obEntity::OutOfSimulationSpace);
                    parentBody->getOwnerWorld()->removeEntity(parentBody, parentBody->getOwnerWorld()->getCurrentTime());
                    unsetLocalGrid();

                    //TODO: manage object's future
                }

                // Cell coordinates correspond to another PhysicsWorld: transfer object
                else
                {
                    PhysicsWorld *newParent = parentBody->getOwnerWorld()->getNeighbor(grid->at(coords).getOwnerId());

                    if(newParent)
                    {
#ifndef NDEBUG
                        qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : PhysicsWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << ");" <<
                              "Entity '" << parentBody->getDisplayName() <<
                              "' owned by '" << parentBody->getOwnerWorld() <<
                              "' in Grid '" << grid->getOwnerId() <<
                              "' (" << lastCellCoords.x() << "," << lastCellCoords.y() << "," << lastCellCoords.z() << ")" <<
                              "  has moved to a cell owned by " << newParent->getId() <<
                              "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

                        // Update status
                        parentBody->setStatus(obEntity::OutOfWorld);

                        const btScalar &eventTime = parentBody->getOwnerWorld()->getCurrentTime();

                        //FIXME: check that what's below is good
                        // Update owner world
                        // Update local grid pointer
                        unsetLocalGrid();
                        parentBody->getOwnerWorld()->removeEntity(parentBody, eventTime);
                        parentBody->getOwnerWorld()->messageNeighbor(newParent,
                                               "onOwnershipTransfer",
                                               Q_ARG(PhysicsWorld *, parentBody->getOwnerWorld()),
                                               Q_ARG(obEntityWrapper *, parentBody),
                                               Q_ARG(btScalar, eventTime));
                    }
                    else
                    {
                        qWarning() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : PhysicsWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << ");" <<
                              "Entity '" << parentBody->getDisplayName() <<
                              "' owned by '" << parentBody->getOwnerWorld() <<
                              "' in Grid '" << grid->getOwnerId() <<
                              "' (" << lastCellCoords.x() << "," << lastCellCoords.y() << "," << lastCellCoords.z() << ")" <<
                              "  has moved to a cell for which no parent could be found (cell owner: " << grid->at(coords).getOwnerId() << ")" <<
                              "; Thread " << QString().sprintf("%p", QThread::currentThread());
                    }
                }
            }
            else
            {
                grid->at(lastCellCoords).removeEntity(parentBody);
                grid->at(coords).addEntity(parentBody);
                lastCellCoords = coords;
            }
        }
    }
}
