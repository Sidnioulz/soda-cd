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
#ifndef SPATIALSUBDIVISION_LOCALGRID_H
#define SPATIALSUBDIVISION_LOCALGRID_H
#include <QVector>

#include "grid.h"
#include "cell.h"

#include <blitz/array.h>
using namespace blitz;

// Forward declaration
class PhysicsWorld;

/*! \class LocalGrid
  * \brief A 3D grid of Cells containing obEntityWrappers, that is specific to a given PhysicsWorld.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a Blitz++ 3D array of Cell objects, that contain a list of obEntityWrapper objects
  * existing in the grid. The grid corresponds to the area of the scene managed by a single PhysicsWorld.
  */
class LocalGrid : public QObject, public Array<Cell, 3>
{
	Q_OBJECT

    //FIXME: many more functions should be private
    //TODO: how to expand grids when scene sizes expanded? in motionstate?
    //FIXME: neighbor discovery through margin!
    //FIXME: consider the difference in each function semantics between WORLD SCENE border and LocalGrid shared bored with another grid

public:
    typedef Array<Cell,3>::iterator iterator;               /*!< Iterators to browse Cells within the array */
    typedef Array<Cell,3>::const_iterator const_iterator;   /*!< Iterators to browse Cells within the array, in read-only mode */

    /*!
      * \brief Default constructor.
      * \param gridInfo the GridInformation that matches the current simulation's parameters
      * \param ownerId the id of this LocalGrid's owner
      * \param margin the margin to apply to the 3D array for resize optimization (not used yet)
      * \param length the length of the part of the scene managed in this grid
      * \param offset offset of the left-bottom-back corner of the grid wrt. to the global scene (cell coordinates)
      * \return a new LocalGrid
      */
    LocalGrid(GridInformation *gridInfo, const short &ownerId, const QVector<int> &margin, const btVector3 &length, const btVector3 &offset);

    /*!
      * \brief Default destructor.
      */
    ~LocalGrid();

    /*!
      * \brief Resizes a LocalGrid to the new size passed as a parameter. New Cells are initialized empty and with an unknown owner.
      * \param margin the margin to apply to the 3D array for resize optimization (not used yet)
      * \param length the length of the part of the scene managed in this grid
      * \param offset offset of the left-bottom-back corner of the grid wrt. to the global scene (cell coordinates)
      *
      * \warning This function is extremely unefficient.
      */
    void resize(const QVector<int> &margin, const btVector3 &length, const btVector3 &offset);

    /*!
     * \brief Returns a string containing textual information on the offsets and bounds of this LocalGrid.
     * \return a QString made of the offset and bounds of this LocalGrid
     */
    QString displayBoundsInfo() const;


    /*!
     * \brief Returns the grid information on the current simulation.
     * \return a pointer to the current simulation's GridInformation
     */
    inline GridInformation *getGridInformation() const
    {
        return gridInfo;
    }

    /*!
     * \brief Returns the id of this grid's owner.
     * \return the id of this LocalGrid's owner
     */
    inline short getOwnerId() const
    {
        return ownerId;
    }

    /*!
     * \brief Changes the owner id of this grid.
     * \param id the id of the new owner of the LocalGrid
     */
    inline void setOwnerId(const short id)
    {
        ownerId = id;
    }

    /*!
     * \brief Adds an entity to this LocalGrid, in the corresponding Cell.
     * \param obEnt the obEntityWrapper to add to this LocalGrid
     */
    void addEntity(obEntityWrapper *obEnt);

    /*!
     * \brief Adds a Cell border to a Cell of this LocalGrid.
     * \param cbEnt the Cell border entity to add
     */
    void addCellBorder(CellBorderEntity *cbEnt);

    /*!
     * \brief Removes an entity from this LocalGrid, if it is found in the Cell matching its coordinates.
     * \param obEnt the obEntityWrapper to remove from this LocalGrid
     */
	void removeEntity(obEntityWrapper *obEnt);

    /*!
     * \brief Adds the neighbors of a given Cell to a vector passed as a paremeter.
     * \param position the cell coordinates of the Cell whose neighbors are wanted
     * \param neighbors a vector to store pointers to the Cell's neighbors
     * \param nPositions a vector to store the positions of the Cell's neighbors
     */
    void getNeighbors(const btVector3 &position, QVector<Cell *> &neighbors, QVector<btVector3> &nPositions);

    /*!
     * \brief Adds the neighbors of a given Cell to a vector passed as a paremeter, provided that they are not owned by this LocalGrid.
     * \param position the cell coordinates of the Cell whose neighbors are wanted
     * \param neighbors a vector to store pointers to the Cell's neighbors
     * \param nPositions a vector to store the positions of the Cell's neighbors
     *
     * This function adds the neighbors of a given Cell to a vector passed as a paremeter,
     * and their positions to another parameter vector, provided that they are not owned
     * by this LocalGrid. Precisely, the Cells must not have the same owner id as this
     * LocalGrid, and they must not have PhysicsWorld::IdBeingProcessed as an id (in order
     * to allow browsing based on the owner id without infinite loops).
     */
    void getUnownedNeighbors(const btVector3 &position, QVector<Cell *> &neighbors, QVector<btVector3> &nPositions);

    /*!
     * \brief Takes a Cell whose owner is unknown and computes whether it can be safely owned by this LocalGrid.
     * \param cell the reference to the Cell whose owner is being computed
     * \param position the coordinates of the parameter Cell in the 3D array
     * \return the final Id of the given Cell
     *
     * This function takes a Cell whose owner is unknown and computes whether it
     * is connected to another world's Cells or to the border of the grid, or
     * whether it is surrounded by already owned Cells. It will also compute the
     * owner id of neighbor Cells who have not been assigned an owner either.
     */
    short resolveOwnership(Cell &cell, const btVector3 &position);

    /*!
     * \brief Adds to a given output vector the list of all Cells who don't have an owner.
     * \param output the vector to which these Cells should be added.
     *
     * This function adds to a given output vector the coordinates of all Cells
     * whose id is PhysicsWorld::NullWorldId, also adding the id of this LocalGrid
     * to each of these entries (in order to differentiate between Cells that have
     * a null id between several LocalGrids).
     */
    void getEmptyCellCoordinates(QVector<QPair<btVector3, short> > &output) const;

    /*!
     * \brief Tells this LocalGrid that the Cell pointed to by coords is owned by the PhysicsWorld whose id is given.
     * \param coords the coordinates of the owned Cell
     * \param id the id of the owning PhysicsWorld
     */
    void setCellOwnedBy(const btVector3 &coords, const short id);

    /*!
     * \brief Returns a reference to the Cell located at the given coordinates.
     * \param coord the coordinates of the wanted Cell
     * \return a reference to the wanted Cell
     *
     * \warning You should check for out of bounds coordinates before calling this function.
     */
    inline Cell &at(const btVector3 &coord)
    {
        return Array::operator ()((int)coord.x(), (int)coord.y(), (int)coord.z());
    }

    /*!
     * \brief Returns a reference to the Cell located at the given coordinates.
     * \param x the x coordinate of the wanted Cell
     * \param y the y coordinate of the wanted Cell
     * \param z the z coordinate of the wanted Cell
     * \return a reference to the wanted Cell
     *
     * \warning You should check for out of bounds coordinates before calling this function.
     */
    inline Cell &at(const int &x, const int &y, const int &z)
    {
        return Array::operator ()(x, y, z);
    }

    /*!
     * \brief Returns a constant reference to the Cell located at the given coordinates.
     * \param coord the coordinates of the wanted Cell
     * \return a constant reference to the wanted Cell
     *
     * \sa inline Cell &at(const btVector3 &coord)
     */
    inline const Cell &at(const btVector3 &coord) const
    {
        return Array::operator ()((int)coord.x(), (int)coord.y(), (int)coord.z());
    }

    /*!
     * \brief Returns a constant reference to the Cell located at the given coordinates.
     * \param x the x coordinate of the wanted Cell
     * \param y the y coordinate of the wanted Cell
     * \param z the z coordinate of the wanted Cell
     * \return a constant reference to the wanted Cell
     *
     * \sa inline Cell &at(const int &x, const int &y, const int &z)
     */
    inline const Cell &at(const int &x, const int &y, const int &z) const
    {
        return Array::operator ()(x, y, z);
    }

    /*!
     * \brief Tries to find, for all Cells with an unknown id, whether the id can be deduced already or if it must be negociated later with neighbors.
     * \return the list of coordinates of Cells that are now owned by the LocalGrid
     *
     * \todo Unit tests for this function.
     * \warning Function very likely broken.
     */
    QVector<btVector3> resolveEmptyCellOwnerships();

    /*!
     * \brief Tells whether a set of coordinates is out of the array bounds.
     * \param x the x coordinate to check for
     * \param y the y coordinate to check for
     * \param z the z coordinate to check for
     * \return whether they are out of bounds
     */
    inline bool cellOutOfBounds(const int &x, const int &y, const int &z) const
    {
        return !(x>=lbound(0) && x<=ubound(0) &&
                 y>=lbound(1) && y<=ubound(1) &&
                 z>=lbound(2) && z<=ubound(2));
    }

    /*!
     * \brief Tells whether a set of coordinates is out of the array bounds.
     * \param coord the coordinates to check for
     * \return whether they are out of bounds
     */
    inline bool cellOutOfBounds(const btVector3 &coord) const
    {
        return !(coord.x()>=lbound(0) && coord.x()<=ubound(0) &&
                 coord.y()>=lbound(1) && coord.y()<=ubound(1) &&
                 coord.z()>=lbound(2) && coord.z()<=ubound(2));
    }

    /*!
     * \brief Tells whether a Cell is owned by this LocalGrid.
     * \param coord the coordinates of the Cell to check
     * \return true if the matching Cell is NOT owned by this LocalGrid
     *
     * This function tells whether a given Cell is owned by this LocalGrid.
     * If it returns true, it may either be owned by another LocalGrid or be
     * outside of the global simulation space bounds.
     */
    bool cellNotOwnedBySelf(const btVector3 &coord) const;

    /*!
     * \brief Tells whether a Cell is owned by this LocalGrid.
     * \param x the x coordinate of the Cell to check
     * \param y the y coordinate of the Cell to check
     * \param z the z coordinate of the Cell to check
     * \return true if the matching Cell is NOT owned by this LocalGrid
     *
     * This function tells whether a given Cell is owned by this LocalGrid.
     * If it returns true, it may either be owned by another LocalGrid or be
     * outside of the global simulation space bounds.
     */
    bool cellNotOwnedBySelf(const int &x, const int &y, const int &z) const;

    /*!
     * \brief Returns a set of coordinates that is roughly in the middle of the LocalGrid.
     * \return a btVector3 of the coordinates at the middle of this LocalGrid
     */
    inline btVector3 getCenteredPosition() const
    {
        return gridInfo->toWorldCoordinates(resolution, (offset + (length / 2)));
//       (offset + (length / 2)) * gridInfo->getGridAtResolution(gridInfo->getBestTerritoryResolution())->getCellLength();
    }

    /*!
      * \brief Returns the LocalGrid's resolution.
      * \return the LocalGrid's resolution
      */
    inline const int &getResolution() const
    {
        return resolution;
    }

    /*!
      * \brief Returns the LocalGrid's length.
      * \return the LocalGrid's length
      */
    inline const btVector3 &getLength() const
    {
        return length;
    }

    /*!
      * \brief Returns the LocalGrid's offset.
      * \return the LocalGrid's offset
      */
    inline const btVector3 &getOffset() const
    {
        return offset;
    }

    /*!
     * \brief Tells whether the Cell at given coordinates is adjacent to unowned inhabitable Cells or is itself unowned.
     * \param coord the coordinates to check
     * \return whether the matching Cell belongs to another world or is on the grid border
     *
     * This function, precisely, tells whether a given Cell is directly adjacent to a Cell that, or is itself a Cell that:
     *   - belongs to another PhysicsWorld
     *   - is out of the global simulation space bounds with an OpenWorld configuration
     */
    bool cellAdjacentToOrIsUnownedCell(const btVector3 &coord) const;

    /*!
     * \brief Retrieves the list of directions for which a Cell is connected to another world.
     * \param coord the coordinates to check
     * \return a vector of six booleans (one per direction), valued true when a neighbor Cell belongs to another world, false otherwise
     *
     * This function retrieves the list of directions for which a Cell is connected to
     * another world. When the Cell targeted by coord is owned by a different world
     * than this, the whole vector is set to false. It is up to clients to verify who
     * owns the Cell before calling this function if they need a distinction to be made.
     */
    QVector<bool> getCellBorders(const btVector3 &coord) const;


private:
    int resolution;                 /*!< the resolution of this LocalGrid. Fixed to 1 for now */
    LocalGrid *parent;              /*!< the LocalGrid of lower resolution. Not used yet */
    LocalGrid *child;               /*!< the LocalGrid of upper resolution. Not used yet */

    GridInformation *gridInfo;      /*!< the GridInformation that matches the scene being simulated in the PhysicsWorld that owns this LocalGrid */
    short ownerId;                  /*!< the id of the PhysicsWorld that owns this LocalGrid */
    QVector<int> margin;            /*!< the margin between the first Cell of the LocalGrid that is owned by its owner and the border of the array. Should always be > 1 */
    btVector3 length;               /*!< the length (in number of Cells) of this LocalGrid */
    btVector3 offset;               /*!< the offset (in number of Cells) between the start of this LocalGrid (including margin) and the scene being simulated */
};

#endif // SPATIALSUBDIVISION_LOCALGRID_H
