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
#include "sodaMotionState.h"
#include "sodaDynamicRigidBody.h"
#include "sodaLogicWorld.h"
#include "sodaLocalGrid.h"
#include "sodaUtils.h"

sodaMotionState::sodaMotionState(sodaDynamicEntity *parent) :
    parentBody(parent),
    grid(0),
    lastCellCoords()
{
}

sodaMotionState::sodaMotionState(sodaDynamicEntity *parent, const btMotionState &other) :
    btMotionState(other),
    parentBody(parent),
    grid(0),
    lastCellCoords()
{
}

void sodaMotionState::setLocalGrid(sodaLocalGrid *newGrid)
{

    grid = newGrid;
    btVector3 coords = grid->getGridInformation()->toCellCoordinates(grid->getResolution(), parentBody->getCenteredPosition());

#ifndef NDEBUG
    qDebug() << "obMotionState(" << parentBody->getDisplayName() << ")::setLocalGrid(" << (newGrid? newGrid->getOwnerId(): sodaLogicWorld::NullWorldId) << "); in (" << coords.x() << coords.y() << coords.z() << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

	if(grid->at(coords).containsEntity(parentBody))
        lastCellCoords = coords;
    else
    {
#ifndef NDEBUG
    qWarning() << "obMotionState(" << parentBody->getDisplayName() << ")::setLocalGrid(" << (newGrid? newGrid->getOwnerId(): sodaLogicWorld::NullWorldId) << "); in (" << coords.x() << coords.y() << coords.z() << "); Coordinates did not contain entity, set random last cell coordinates; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        lastCellCoords = btVector3(INFINITY, INFINITY, INFINITY);
    }
}

void sodaMotionState::unsetLocalGrid()
{
    grid = 0;
}

void sodaMotionState::getWorldTransform(btTransform &/*worldTrans*/) const
{
#ifndef NDEBUG
//    qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : sodaLogicWorld::NullWorldId) << ")::getWorldTransform(...); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
}

void sodaMotionState::setWorldTransform(const btTransform &worldTrans)
{
#ifndef NDEBUG
//    qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : sodaLogicWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    if(grid && parentBody)
    {
        btVector3 coords = grid->getGridInformation()->toCellCoordinates(grid->getResolution(), parentBody->getCenteredPosition());
        if(coords != lastCellCoords)
        {
            if(grid->cellNotOwnedBySelf(coords))
            {
                // Cell coordinates are outside of the simulation scene boundaries (because margin ensures we don't run out of bounds when there is a neighbor)
				if(!grid->getGridInformation()->isWithinWorldCellBounds(coords))
                {
                    qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : sodaLogicWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << ");" <<
                          "Entity '" << parentBody->getDisplayName() <<
                          "' owned by '" << parentBody->getOwnerWorld() <<
                          "' in Grid '" << grid->getOwnerId() <<
                          "' (" << lastCellCoords.x() << "," << lastCellCoords.y() << "," << lastCellCoords.z() << ")" <<
                          "  is out of bounds (" << coords.x() << "," << coords.y() << "," << coords.z() << ", LG bounds: " << grid->displayBoundsInfo() << ")" <<
                          "; Thread " << QString().sprintf("%p", QThread::currentThread());

                    // Update status
                    parentBody->setStatus(sodaEntity::Removed);

                    // Remove entity from Cell, remove pointer to sodaLocalGrid
                    grid->at(lastCellCoords).removeEntity(parentBody);
                    unsetLocalGrid();

                    // Detach from current world
                    parentBody->getOwnerWorld()->removeEntity(parentBody, parentBody->getOwnerWorld()->getCurrentTime());

                    //TODO: manage object's future
                }

                // Cell coordinates correspond to another sodaLogicWorld: transfer object
                else
                {
                    sodaLogicWorld *newParent = parentBody->getOwnerWorld()->getNeighbor(grid->at(coords).getOwnerId());

                    if(newParent)
                    {
#ifndef NDEBUG
                        qDebug() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : sodaLogicWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << ");" <<
                              "Entity '" << parentBody->getDisplayName() <<
                              "' owned by '" << parentBody->getOwnerWorld() <<
                              "' in Grid '" << grid->getOwnerId() <<
                              "' (" << lastCellCoords.x() << "," << lastCellCoords.y() << "," << lastCellCoords.z() << ")" <<
                              "  has moved to a cell owned by " << newParent->getId() <<
                              "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

                        // Update status
                        parentBody->setStatus(sodaEntity::OutOfWorld);
                        const btScalar &eventTime = parentBody->getOwnerWorld()->getCurrentTime();

                        // Remove entity from Cell, remove pointer to sodaLocalGrid
                        grid->at(lastCellCoords).removeEntity(parentBody);
                        unsetLocalGrid();

                        // Detach from current world and send to new world
                        sodaLogicWorld *oldOwner = parentBody->getOwnerWorld();
                        oldOwner->messageNeighbor(newParent,
                                               "onOwnershipTransfer",
                                               Q_ARG(sodaLogicWorld *, oldOwner),
                                               Q_ARG(sodaDynamicEntity *, new sodaDynamicEntity(*parentBody)),
                                               Q_ARG(btScalar, eventTime));
                        oldOwner->removeEntity(parentBody, eventTime);
                    }
                    else
                    {
                        //FIXME: should never happen, happens a lot - linked to problems with non-adjacency and grid.resize() that doesn't properly manage margins
                        qWarning() << "setWorldTransform(" << (parentBody? parentBody->getOwnerId() : sodaLogicWorld::NullWorldId) << ")::setWorldTransform(" << worldTrans.getOrigin().x() << worldTrans.getOrigin().y() << worldTrans.getOrigin().z() << ");" <<
                              "Entity '" << parentBody->getDisplayName() <<
                              "' owned by '" << parentBody->getOwnerWorld() <<
                              "' in Grid '" << grid->getOwnerId() <<
                              "' (" << lastCellCoords.x() << "," << lastCellCoords.y() << "," << lastCellCoords.z() << ")" <<
                              "  has moved to a cell for which no parent could be found (cell owner: " << grid->at(coords).getOwnerId() << ")" <<
                              "; Thread " << QString().sprintf("%p", QThread::currentThread());
                    }
                }
            }

			// Entity still within world's control, transfer it to new Cell
            else
            {
                grid->at(lastCellCoords).removeEntity(parentBody);
                grid->at(coords).addEntity(parentBody);
                lastCellCoords = coords;
            }
        }
    }
}
