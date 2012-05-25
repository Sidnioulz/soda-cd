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
 */
#include <QtAlgorithms>
#include <QtCore>
#include "obMotionState.h"
#include "localgrid.h"
#include "utils.h"
#include "physicsworld.h"


using namespace blitz;


LocalGrid::LocalGrid(GridInformation *gridInfo, const short &ownerId, const QVector<int> &margin, const btVector3 &length, const btVector3 &offset) :
    resolution(gridInfo->getBestTerritoryResolution()),
    parent(0),
    child(0),
    gridInfo(gridInfo),
    ownerId(ownerId),
    margin(margin.size() == GridInformation::NB_DIRECTIONS ? margin : QVector<int>(GridInformation::NB_DIRECTIONS, 0)),
    length(length),
    offset(offset)
{
    Array::resize(Range(offset.x()-margin[GridInformation::Left], length.x()+offset.x()-1+margin[GridInformation::Right]),
           Range(offset.y()-margin[GridInformation::Bottom], length.y()+offset.y()-1+margin[GridInformation::Top]),
           Range(offset.z()-margin[GridInformation::Front], length.z()+offset.z()-1-margin[GridInformation::Back]));
    Array::operator = (Cell(PhysicsWorld::UnknownWorldId));

    qDebug() << offset.x() << offset.y() << offset.z();
    std::cout << "Lower bound:  " << lbound() << std::endl;
    std::cout << "Upper bound:  " << ubound() << std::endl;
    std::cout << "Grid nbCells: " << size() << std::endl << std::endl;
}

LocalGrid::~LocalGrid()
{
    if(child)
        delete child;
    child = 0;
}

void LocalGrid::resize(const QVector<int> &newMargin, const btVector3 &newLength, const btVector3 &newOffset)
{
    //TODO: _fixMargin function that makes a new margin adapted to the global world borders.
    QVector<int> nmargin = newMargin.size() == GridInformation::NB_DIRECTIONS ? newMargin : QVector<int>(GridInformation::NB_DIRECTIONS, 0);
    margin = nmargin;
    length = newLength;
    offset = newOffset;

    Array<Cell, 3> tmp = Array::copy();
    Array::resize(newLength.x() + newMargin[GridInformation::Left] + newMargin[GridInformation::Right],
            newLength.y() + newMargin[GridInformation::Bottom] + newMargin[GridInformation::Top],
            newLength.z() + newMargin[GridInformation::Front] + newMargin[GridInformation::Back]);
    Array::reindexSelf(TinyVector<int, 3>(newOffset.x(), newOffset.y(), newOffset.z()));

    for(Array<Cell, 3>::const_iterator it = tmp.begin(); it != tmp.end(); it++)
    {
        if(Array::isInRange(it.position()))
            Array::operator ()(it.position()) = *it;
    }
}

QString LocalGrid::displayBoundsInfo() const
{
    QString str = QString("lb: %1;%2;%3\tub: %4;%5;%6")
            .arg(offset.x()).arg(offset.y()).arg(offset.z())
            .arg(ubound()[0]).arg(ubound()[1]).arg(ubound()[2]);

    return str;
}

void LocalGrid::addEntity(obEntityWrapper *obEnt)
{
    // Get the Cell coordinates in which to add the entity
    btVector3 cellCoords = gridInfo->toCellCoordinates(gridInfo->getBestTerritoryResolution(), obEnt->getCenteredPosition());

    // Check that it exists and get it
    Q_ASSERT(!outOfBounds(cellCoords));
    Cell &cell = at(cellCoords);

    // Add the entity
    cell.addEntity(obEnt);
    obEnt->getRigidBody()->getMotionState()->setLocalGrid(this);
}

void LocalGrid::addCellBorder(CellBorderEntity *cbEnt)
{
    // Get the Cell in which to add the entity
    Cell &cell = at(cbEnt->getCoordinates());
    cell.addCellBorder(cbEnt);
}

void LocalGrid::removeEntity(obEntityWrapper *obEnt)
{
	// Get the Cell in which the entity should be
    btVector3 cellCoords = gridInfo->toCellCoordinates(gridInfo->getBestTerritoryResolution(), obEnt->getCenteredPosition());
    Cell &cell = at(cellCoords);

	// Remove the entity from the Cell, if it was actually inside it
	if(!cell.removeEntity(obEnt))
	{
        qWarning() << "Entity '" << obEnt->getName().c_str() << "' is not present within the Cell that matches its coordinates (" << cellCoords.x() << ", " << cellCoords.y() << ", " << cellCoords.z() << ").";
	}

    obEnt->getRigidBody()->getMotionState()->unsetLocalGrid();
}

void LocalGrid::getNeighbors(const btVector3 &position, QVector<Cell *> &neighbors, QVector<btVector3> &nPositions)
{
    for(int i=-1; i<2; ++i)
        for(int j=-1; j<2; ++j)
            for(int k=-1; k<2; ++k)
                if((i!=0 || j!=0 || k!=0) && !outOfBounds(btVector3(i, j, k) + position))
                {
                    if(!neighbors.contains(&at(position)))
                    {
                        neighbors.append(&at(position));
                        nPositions.append(btVector3(i, j, k) + position);
                    }
                }
}

void LocalGrid::getUnownedNeighbors(const btVector3 &position, QVector<Cell *> &neighbors, QVector<btVector3> &nPositions)
{
    for(int i=-1; i<2; ++i)
        for(int j=-1; j<2; ++j)
            for(int k=-1; k<2; ++k)
                if((i!=0 || j!=0 || k!=0) && !outOfBounds(btVector3(i, j, k) + position))
                {
                    Cell &nCell = at(position);
                    if(nCell.getOwnerId() != PhysicsWorld::IdBeingProcessed &&  nCell.getOwnerId() != ownerId && !neighbors.contains(&at(position)))
                    {
                        neighbors.append(&at(position));
                        nPositions.append(btVector3(i, j, k) + position);
                    }
                }
}

short LocalGrid::resolveOwnership(Cell &cell, const btVector3 &position)
{
    short finalId = ownerId;
    QVector<Cell *> neighbors;
    QVector<btVector3> nPositions;

    getUnownedNeighbors(position, neighbors, nPositions);

    // Look for the next non-owned neighbor
    for(int i=0; i<neighbors.size(); ++i)
    {
        Cell &neighbor = (*neighbors[i]);
        const btVector3 &neighborPos = nPositions[i];

        // We found a neighbor owned by another world, so the currentConnectedCells must be
        // marked to NullWorlId for further negotiation with neighbors.
        if(connectedToAnotherWorld(neighborPos))
            finalId = PhysicsWorld::NullWorldId;

        // There is another Cell that needs to be given a WorldId, let's explore its neighbors
        else if(neighbor.getOwnerId() == PhysicsWorld::UnknownWorldId)
        {
            cell.setOwnerId(PhysicsWorld::IdBeingProcessed);
            getUnownedNeighbors(neighborPos, neighbors, nPositions);
        }
    }

    cell.setOwnerId(finalId);

    return finalId;
}

void LocalGrid::getEmptyCellCoordinates(QVector<QPair<btVector3, short> > &output) const
{
    for(Array<Cell, 3>::const_iterator it = begin(); it != end(); it++)
    {
        const Cell &cell = *it;

        if(cell.getOwnerId() == PhysicsWorld::NullWorldId)
            output.append(QPair<btVector3, short>(Utils::btVectorFromBlitz(it.position()), this->ownerId));
    }
}

void LocalGrid::setCellOwnedBy(const btVector3 &coords, const short id)
{
    if(!outOfBounds(coords))
    {
        if(at(coords).getOwnerId() != PhysicsWorld::NullWorldId)
        {
            qWarning() << "Warning, changed owner of Cell " << coords.x() << coords.y() << coords.z() << " while it already had one (" << at(coords).getOwnerId() << ")" << " to " << id;
            if(at(coords).getEntities() != 0 && at(coords).getEntities()->size() != 0)
                qWarning() << "\t " << at(coords).getEntities()->size() << "entities in this cell";
        }

        at(coords).setOwnerId(id);
    }
    else
        qDebug() << "Grid " << ownerId << "oob " << coords.x() << coords.y() << coords.z() << id;
}

QVector<btVector3> LocalGrid::resolveEmptyCellOwnerships()
{
    //FIXME: this whole function is broken
    QVector<btVector3> nowAssigned;

    // For each cell of the LocalGrid's data, try to find an owner if it doesn't have one
    for(Array<Cell, 3>::iterator it = begin(); it != end(); it++)
    {
        Cell &cell = *it;

        // If cell ID is unknown, check if Cell is part of a closed block
        if(cell.getOwnerId() == PhysicsWorld::UnknownWorldId)
        {
            //NOTE: the good version might be it.position() + offset
            /*short finalId = */resolveOwnership(cell, Utils::btVectorFromBlitz(it.position()));

//            if(finalId != PhysicsWorld::NullWorldId)
//                nowAssigned.append(Utils::btVectorFromBlitz(it.position()));
        }
    }

    qDebug() << "World " << this->getOwnerId() << " has retrieved " << nowAssigned.size() << " cells.";

    return nowAssigned;
}

bool LocalGrid::connectedToAnotherWorld(const btVector3 &coord) const
{
    if(!(coord.x()>lbound(0) && coord.x()<ubound(0) &&
        coord.y()>lbound(1) && coord.y()<ubound(1) &&
        coord.z()>lbound(2) && coord.z()<ubound(2)))
        return true;

    short id = at(coord).getOwnerId();
    return (id != ownerId && id != PhysicsWorld::UnknownWorldId && id != PhysicsWorld::IdBeingProcessed);
}

QVector<bool> LocalGrid::getConnectionsToOtherWorlds(const btVector3 &coord) const
{
    QVector<bool> cons(6, false);

    // Don't report connections to other worlds for Cells that are not owned
    if(ownedByAnotherWorld(coord))
        return cons;


//    short Topid =at(coord+btVector3(0,1,0)).getOwnerId();
//    short Bottomid =at(coord+btVector3(0,-1,0)).getOwnerId();
//    short Rightid =at(coord+btVector3(1,0,0)).getOwnerId();
//    short Leftid =at(coord+btVector3(-1,0,0)).getOwnerId();
//    short Frontid =at(coord+btVector3(0,0,1)).getOwnerId();
//    short Backid =at(coord+btVector3(0,0,-1)).getOwnerId();

    cons[GridInformation::Top] = ownedByAnotherWorld(coord+btVector3(0,1,0)) && gridInfo->getGridAtResolution(resolution)->withinWorldCellBounds(coord+btVector3(0,1,0));
    cons[GridInformation::Bottom] = ownedByAnotherWorld(coord+btVector3(0,-1,0)) && gridInfo->getGridAtResolution(resolution)->withinWorldCellBounds(coord+btVector3(0,-1,0));

    cons[GridInformation::Right] = ownedByAnotherWorld(coord+btVector3(1,0,0)) && gridInfo->getGridAtResolution(resolution)->withinWorldCellBounds(coord+btVector3(1,0,0));
    cons[GridInformation::Left] = ownedByAnotherWorld(coord+btVector3(-1,0,0)) && gridInfo->getGridAtResolution(resolution)->withinWorldCellBounds(coord+btVector3(-1,0,0));

    cons[GridInformation::Front] = ownedByAnotherWorld(coord+btVector3(0,0,1)) && gridInfo->getGridAtResolution(resolution)->withinWorldCellBounds(coord+btVector3(0,0,1));
    cons[GridInformation::Back] = ownedByAnotherWorld(coord+btVector3(0,0,-1)) && gridInfo->getGridAtResolution(resolution)->withinWorldCellBounds(coord+btVector3(0,0,-1));

    return cons;
}

bool LocalGrid::ownedByAnotherWorld(const btVector3 &coord) const
{
    if(outOfBounds(coord))
        return true;

    short id = at(coord).getOwnerId();
    return (id != ownerId && id != PhysicsWorld::IdBeingProcessed);
}
