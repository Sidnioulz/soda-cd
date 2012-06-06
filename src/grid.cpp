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
#include <QtCore/qmath.h>
#include <QtDebug>
#include "grid.h"
#include <sstream>


const btVector3 GridInformation::InvalidCellCoordinates(-1, -1, -1);

InvalidResolutionException::InvalidResolutionException(const GridInformation * const grid, const int res)
{
    std::stringstream sstr;
    sstr << "Can't reach resolution " << res << " with current grid that has no " <<
            (grid->getParent() == NULL ? (grid->getChild() == NULL? "parent and child" : "parent") : "child") <<
            " and is at resolution " << grid->getResolution() << ".";
    msg = sstr.str();
}

GridInformation *GridInformation::grid = 0;

GridInformation::GridInformation(const int resolution, const btVector3 &spaceLen, const btVector3 &biggestObjectSize, GridInformation *parent) :
    resolution(resolution),
    bestTerritoryResolution(0),
	parent(parent),
	child(0),
    spaceLen(spaceLen),
    nbCells(qMax(resolution, qFloor(spaceLen.x() / (biggestObjectSize.x()+1))),
            qMax(resolution, qFloor(spaceLen.y() / (biggestObjectSize.y()+1))),
            qMax(resolution, qFloor(spaceLen.z() / (biggestObjectSize.z()+1)))),
    cellLen(spaceLen / nbCells)
{
    qDebug() << "####\nGRID INFORMATION\n" << resolution;
    qDebug() << nbCells.x() << ", " << nbCells.y() << ", " << nbCells.z();
    qDebug() << cellLen.x() << ", " << cellLen.y() << ", " << cellLen.z() << "\n";
    qDebug() << "####";
}

GridInformation::~GridInformation()
{
    if(child)
        delete child;
    child = 0;
}

GridInformation *GridInformation::getGrid(const btScalar initScaleRatio, const btVector3 &sceneSize, const btVector3 &biggestObjectSize, bool reset)
{
    if(reset && grid != 0)
    {
        delete grid;
        grid = 0;
    }

	if(grid == 0)
    {
        // Find ideal depth, so that all cells are populated with objects ]0.25, 0.5[ fold their size
        int depth=1, mult;
        // 1 to 2 scale ratio means a single cell is enough
        if(initScaleRatio > 2)
        {
            // Else every time the scale ratio doubles, we need one more depth layer
            for(depth=2, mult=3; initScaleRatio >= mult*2; mult*=2)
                ++depth;
        }

        GridInformation *prevGrid=0;
        btVector3 objSizePerRes(biggestObjectSize);
        for(int d=0; d<depth; ++d)
        {
            GridInformation *dGrid = new GridInformation(d+1, sceneSize, objSizePerRes, prevGrid);
            objSizePerRes /=2;

            // Let grid point to the root of the hierarchy, and connect child grids to their parents
            if(prevGrid)
                prevGrid->setChild(dGrid);
            else
                grid=dGrid;

            prevGrid=dGrid;
        }
    }

    return grid;
}

GridInformation *GridInformation::getGridAtResolution(const int res) throw(InvalidResolutionException)
{
    if(resolution<res)
    {
        if(child)
            return child->getGridAtResolution(res);
        else
            throw InvalidResolutionException(this, res);
    }
    else if(resolution>res)
    {
        if(parent)
            return parent->getGridAtResolution(res);
        else
            throw InvalidResolutionException(this, res);
    }
    else
        return this;
}

btVector3 GridInformation::toCellCoordinates(const int cellRes, const btVector3 &coords) const throw(InvalidResolutionException)
{
    if(resolution<cellRes)
    {
        if(child)
            return child->toCellCoordinates(cellRes, coords);
        else
            throw InvalidResolutionException(this, cellRes);
    }
    else if(resolution>cellRes)
    {
        if(parent)
            return parent->toCellCoordinates(cellRes, coords);
        else
            throw InvalidResolutionException(this, cellRes);
    }

//    btVector3 cellCoords = (coords/cellLen);// + (nbCells * btVector3(0.5f, 0, 0.5f)); // Cancels the need for negative offsets (useless with Blitz++ arrays)


//    cellCoords.setX(qFloor(cellCoords.x()));
//    cellCoords.setY(qFloor(cellCoords.y()));
//    cellCoords.setZ(qFloor(cellCoords.z()));

//    return cellCoords;

    return btVector3(qFloor((coords.x() + spaceLen.x()/2) / cellLen.x()),
                     qFloor(coords.y() / cellLen.y()),
                     qFloor((coords.z() + spaceLen.z()/2) / cellLen.z()));
}

btVector3 GridInformation::toWorldCoordinates(const int cellRes, const btVector3 &cellCoords) const throw(InvalidResolutionException)
{
    if(resolution<cellRes)
    {
        if(child)
            return child->toWorldCoordinates(cellRes, cellCoords);
        else
            throw InvalidResolutionException(this, cellRes);
    }
    else if(resolution>cellRes)
    {
        if(parent)
            return parent->toWorldCoordinates(cellRes, cellCoords);
        else
            throw InvalidResolutionException(this, cellRes);
    }

    return cellCoords * cellLen - btVector3(spaceLen.x()/2, 0, spaceLen.z()/2);
}

btVector3 GridInformation::toCenteredWorldCoordinates(const int cellRes, const btVector3 &cellCoords) const throw(InvalidResolutionException)
{
    if(resolution<cellRes)
    {
        if(child)
            return child->toCenteredWorldCoordinates(cellRes, cellCoords);
        else
            throw InvalidResolutionException(this, cellRes);
    }
    else if(resolution>cellRes)
    {
        if(parent)
            return parent->toCenteredWorldCoordinates(cellRes, cellCoords);
        else
            throw InvalidResolutionException(this, cellRes);
    }

    return (cellCoords + btVector3(0.5, 0.5, 0.5)) * cellLen - btVector3(spaceLen.x()/2, 0, spaceLen.z()/2);
}

btVector3 GridInformation::toDirectedWorldCoordinates(const int cellRes, const CellBorderCoordinates &cellCoords) const throw(InvalidResolutionException)
{
    if(resolution<cellRes)
    {
        if(child)
            return child->toDirectedWorldCoordinates(cellRes, cellCoords);
        else
            throw InvalidResolutionException(this, cellRes);
    }
    else if(resolution>cellRes)
    {
        if(parent)
            return parent->toDirectedWorldCoordinates(cellRes, cellCoords);
        else
            throw InvalidResolutionException(this, cellRes);
    }

    btVector3 directedAdjustment(0.5, 0.5, 0.5);
    if(cellCoords.direction() == GridInformation::Right)
        directedAdjustment.setX(1);
    else if(cellCoords.direction() == GridInformation::Top)
        directedAdjustment.setY(1);
    else if(cellCoords.direction() == GridInformation::Front)
        directedAdjustment.setZ(1);
    else if(cellCoords.direction() == GridInformation::Left)
        directedAdjustment.setX(0);
    else if(cellCoords.direction() == GridInformation::Bottom)
        directedAdjustment.setY(0);
    else if(cellCoords.direction() == GridInformation::Back)
        directedAdjustment.setZ(0);

    return (cellCoords + directedAdjustment) * cellLen - btVector3(spaceLen.x()/2, 0, spaceLen.z()/2);
}

int GridInformation::getBestTerritoryResolution() const
{
    if(bestTerritoryResolution == 0)
    {
        //TODO: implement getBestTerritoryResolution, think of inlining the return part
		return 1;
    }
    else
        return bestTerritoryResolution;
}
