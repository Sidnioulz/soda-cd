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
#ifndef SPATIALSUBDIVISION_GRID_H
#define SPATIALSUBDIVISION_GRID_H

#include <btBulletDynamicsCommon.h>
#include <exception>
#include <string>
#include "cellborderentity.h"

// Forward declaration
class GridInformation;

/*! \class InvalidResolutionException
  * \brief An exception for methods that try to operate on a resolution that doesn't exist within the GridInformation.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an exception thrown by methods that operate on a GridInformation's resolution, when the resolution
  * for which it wishes to operate is not defined in the GridInformation (either lower than the furthest parent's , or
  * higher than the furthest child's).
  */
class InvalidResolutionException : public std::exception
{
public:
    /*!
      * \brief Default constructor.
      * \param grid the GridInformation for which an invalid resolution was queried
      * \param res the invalid resolution asked for
      * \return a new InvalidResolutionException
      */
    InvalidResolutionException(const GridInformation * const grid, const int res);

    /*!
      * \brief Default destructor.
      */
    virtual ~InvalidResolutionException() throw()
    {}

    /*!
      * \brief Returns the data to display when throwing an exception.
      * \return the message string of this exception
      */
    virtual const char* what() const throw()
    {
        return msg.c_str();
    }

private:
    std::string msg; //!< Holder for the message string of this exception
};



/*! \class GridInformation
  * \brief A uniform grid for spatial subdivision with parent and child grids of different resolution.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class represents one layer of a hierarchy of uniform 3D grids. Only the structure of the grid is defined,
  * but this class cannot be used to store data within cells. It gives information of the space in which the
  * GridInformation exists, the number and size of cells, a hierarchy resolution, and pointers to higher and lower
  * resolution uniform grids.
  */
class GridInformation
{
public:

    //! An enumeration of all possible 1D directions in a 3D space. Currently used for the margin array.
    typedef enum __Directions {
        Bottom=0,                                           /*!< Move to the bottom  (y-1) */
        Left=1,                                             /*!< Move to the left    (x-1) */
        Front=2,                                            /*!< Move to the front   (z+1) */
        Right=3,                                            /*!< Move to the right   (x+1) */
        Back=4,                                             /*!< Move to the back    (z-1) */
        Top=5,                                              /*!< Move to the top     (y+1) */
        NB_DIRECTIONS=6                                     /*!< Do not use in production code. */
    } Directions;                                           /*!< These enums allow represent a one-dimensional move in the 3D array */

    //! A variable that represents Cell coordinates that cannot exist
    const static btVector3 InvalidCellCoordinates;

    /*!
      * \brief Returns the GridInformation that matches the current simulation's 3D space.
      * \return a pointer to the GridInformation for the current simulation
      *
      * This function returns a pointer to a static GridInformation that represents the currently
      * simulated world. If the grid has not been created using the version of getGrid() with
      * parameters, this function may return NULL.
      */
    static inline const GridInformation *getGrid()
    {
        return grid;
    }

    /*!
      * \brief Returns the GridInformation that matches the current simulation's 3D space, after creating it if necessary.
      * \param initScaleRatio scale ratio between biggest and smallest objects that will populate the grid
      * \param sceneSize length of the scene on all axes
      * \param biggestObjectSize the size of the biggest object of the scene
      * \param reset whether to reset the current GridInformation and create a new one
      * \return a pointer to the GridInformation for the current simulation
      *
      * This function returns a pointer to a static GridInformation that represents the currently
      * simulated world, using the given parameters to create it if necessary.
      */
    static GridInformation *getGrid(const btScalar initScaleRatio, const btVector3 &sceneSize, const btVector3 &biggestObjectSize, bool reset = false);

    /*!
      * \brief Sets the child GridInformation of this object to the one given as a parameter.
      * \param childGrid the child GridInformation for this object
      */
    inline void setChild(GridInformation *childGrid)
    {
        child = childGrid;
    }

    /*!
      * \brief Returns the child GridInformation of this object.
      * \return the child GridInformation for this object
      */
    inline GridInformation *getChild() const
    {
        return child;
    }

    /*!
      * \brief Sets the parent GridInformation of this object to the one given as a parameter.
      * \param parentGrid the parent GridInformation for this object
      */
    inline void setParent(GridInformation *parentGrid)
    {
        parent = parentGrid;
    }

    /*!
      * \brief Returns the parent GridInformation of this object.
      * \return the parent GridInformation for this object
      */
    inline GridInformation *getParent() const
    {
        return parent;
    }

    /*!
      * \brief Returns the resolution of this GridInformation.
      * \return the resolution of this GridInformation
      */
    inline int getResolution() const
    {
        return resolution;
    }

    /*!
      * \brief Returns the number of cells per axis for this GridInformation.
      * \return a btVector3 containing the number of cells on each axis
      */
    inline btVector3 getNbCells() const
    {
        return nbCells;
    }

    /*!
      * \brief Returns the length of each Each in this GridInformation.
      * \return a btVector3 containing the length of cells on each axis
      */
    inline btVector3 getCellLength() const
    {
        return cellLen;
    }

    /*!
      * \brief Returns the GridInformation from the hierarchy at a given resolution.
      * \param res the resolution for which the GridInformation is wanted
      * \return the GridInformation at the given resolution
      */
    GridInformation *getGridAtResolution(const int res) throw(InvalidResolutionException);

    /*!
      * \brief Transforms a set of scene coordinates into Cell coordinates for a given resolution.
      * \param cellRes the resolution for which coordinates are wanted
      * \param coords the coordinates in the scene space
      * \return the Cell coordinates at the wanted resolution
      */
    btVector3 toCellCoordinates(const int cellRes, const btVector3 &coords) const throw(InvalidResolutionException);

    /*!
      * \brief Transforms a set of Cell coordinates at a given resolution into world coordinates.
      * \param cellRes the resolution for which coordinates are given
      * \param cellCoords the coordinates in the Cell space
      * \return the real world coordinates of the point at the origin of the Cell.
      *
      * This function returns the world coordinates of a Cell's bottom-left-back corner.
      */
    btVector3 toWorldCoordinates(const int cellRes, const btVector3 &cellCoords) const throw(InvalidResolutionException);

    /*!
      * \brief Transforms Cell coordinates at a given resolution into the world coordinates of the middle of the Cell.
      * \param cellRes the resolution for which coordinates are given
      * \param cellCoords the coordinates in the Cell space
      * \return the real world coordinates of the point at the middle of the Cell
      *
      * This function returns the world coordinates of a Cell's center.
      */
    btVector3 toCenteredWorldCoordinates(const int cellRes, const btVector3 &cellCoords) const throw(InvalidResolutionException);

    /*!
      * \brief Transforms a set of directed Cell coordinates at a given resolution into world coordinates.
      * \param cellRes the resolution for which coordinates are given
      * \param cellCoords the coordinates in the Cell space, with a directional component
      * \return the real world coordinates of the point at the middle of the Cell
      *
      * This function returns the world coordinates of a Cell's corner depending on
      * the direction component of the CellBorderCoordinates:
      *
      * Bottom:   bottom - left  - back
      * Left:     bottom - left  - back
      * Back:     bottom - left  - back
      * Top:      top    - left  - back
      * Right:    bottom - right - back
      * Front:    bottom - left  - front
      */
    btVector3 toDirectedWorldCoordinates(const int cellRes, const CellBorderCoordinates &cellCoords) const throw(InvalidResolutionException);

    /*!
      * \brief Returns a list of coordinates for all the Cells that exist at this GridInformation's resolution.
      * \return a list of centered coordinates for every Cell that exists at this object's resolution
      */
    QList<btVector3> getAllCellCoordinates() const;

    /*!
      * \brief Computes the best resolution to define territories on, given a GridInformation hierarchy which this object is part of.
      * \return the best resolution to define territories on for this GridInformation hierarchy
      *
      * \todo implement this function, think of inlining the return part
      */
    int getBestTerritoryResolution() const;

    /*!
     * \brief Tells whether a set of given Cell coordinates are within this GridInformation's world bounds.
     * \param coord the coordinates to check for
     * \return whether the coordinates are within the bounds of the global simulated world
     */
    bool withinWorldCellBounds(const btVector3 &coord) const
    {
        return true;
        return (coord.x() >= -nbCells.x()/2) && (coord.x() < nbCells.x()/2) &&
                (coord.y() >= 0) && (coord.y() < nbCells.y()) &&
                (coord.z() >= -nbCells.z()/2) && (coord.z() < nbCells.z()/2);
    }

private:
	/*!
      * \brief Default constructor.
      * \param resolution the resolution of this GridInformation
      * \param spaceLen the size of the scene space that this GridInformation corresponds to
      * \param biggestObjectSize the size of the biggest object that will populate the grid
      * \param parent a pointer to a parent GridInformation if known
      * \return a new GridInformation.
      */
    explicit GridInformation(const int resolution, const btVector3 &spaceLen, const btVector3 &biggestObjectSize, GridInformation *parent=0);

    /*!
      * \brief Default destructor.
      */
    virtual ~GridInformation();

    int resolution;                     //!< Resolution of this GridInformation
    int bestTerritoryResolution;        //!< Best resolution to manage territories for the scene represented by this GridInformation
    GridInformation *parent;            //!< Parent of this GridInformation
    GridInformation *child;             //!< Child of this GridInformation
    btVector3 spaceLen;                 //!< Size of each axis of the scene space attached to this GridInformation
    btVector3 nbCells;                  //!< Number of cells for this GridInformation's resolution, for each axis of the scene
    btVector3 cellLen;                  //!< Length of cells on each axis for this GridInformation

    static GridInformation *grid;       //!< Pointer to a static GridInformation that represents the current simulation

    //WARNING: storage overhead is big with little cells on grid, but big with big cells for margins
};

#endif // SPATIALSUBDIVISION_GRID_H
