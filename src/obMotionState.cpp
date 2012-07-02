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

void obMotionState::getWorldTransform(btTransform &worldTrans) const
{
}

void obMotionState::setWorldTransform(const btTransform &worldTrans)
{
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
                    qDebug() << "Entity '" << parentBody->getName().c_str() << "' in Grid '" << grid->getOwnerId() << "' is out of bounds\t(" << coords.x()<<"," << coords.y()<<"," << coords.z() << ", LG bounds:\t" << grid->displayBoundsInfo() << ")";

                    parentBody->setStatus(obEntity::OutOfSimulationSpace);
                    grid->at(lastCellCoords).removeEntity(parentBody);
                    unsetLocalGrid();

                    //TODO: manage object's future
                }




                // Cell coordinates correspond to another PhysicsWorld: transfer object
                else
                {
//                    qDebug() << "Entity '" << parentBody->getName().c_str() << "' in Grid '" << grid->getOwnerId() << "' is in a foreign cell (" << grid->at(coords).getOwnerId() << ").";


                    PhysicsWorld *newParent = parentBody->getOwnerWorld()->getNeighbor(grid->at(coords).getOwnerId());
                    qDebug() << "Entity '" << parentBody->getName().c_str() << "' in Grid '" << grid->getOwnerId() << "' to be transferred to " << newParent << ".";

                    if(newParent)
                    {
                        // Update status
                        parentBody->setStatus(obEntity::OutOfWorld);

                        const btScalar &eventTime = parentBody->getOwnerWorld()->getCurrentTime();

                        // Update owner world
                        parentBody->getOwnerWorld()->removeEntity(parentBody, eventTime);
                        newParent->addEntity(parentBody, eventTime);

                        // Update local grid pointer
                        unsetLocalGrid();
                        setLocalGrid(newParent->getLocalGrid());
                    }
                    else
                    {
                        qWarning() << "Error on transferring'" << parentBody->getName().c_str() << "' to new parent world" << grid->at(coords).getOwnerId();
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
