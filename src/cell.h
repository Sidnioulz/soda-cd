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
#ifndef CELL_H
#define CELL_H

#include <QSharedPointer>
#include <QVector>
#include "sodaDynamicEntity.h"
#include "cellborderentity.h"

/*! \class Cell
  * \brief Represents the cell of a 3D grid, containing entities and owned by a physics world.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class represents a cell in the sodaLocalGrid class. It is owned by a sodaLogicWorld,
  * and contains a list of sodaDynamicEntity instances. The size and location of the cell are
  * defined in the sodaLocalGrid of the sodaLogicWorld that owns it.
  *
  * The pointer to the structure of the Cell can be set to NULL, in which case the Cell
  * represents a neighbour Cell for whose content the owner sodaLocalGrid has no information.
  */
class Cell
{
public:

    /*! \brief Default constructor.
      * \return a new Cell
      */
    Cell();

    /*! \brief Constructor with an owner ID.
      * \param ownerId the id of the owner of this cell
      * \return a new Cell
      */
    Cell(const short ownerId);

    /*! \brief Copy constructor.
      * \param other the Cell to copy
      * \return a new Cell
      */
    Cell(const Cell &other);

    /*!
     * \brief Destructor.
     */
    ~Cell();

    /*! \brief Changes the owner id of this Cell.
      * \param id the new owner id
      */
    inline void setOwnerId(const short id)
    {
        ownerId=id;
    }

    /*! \brief Returns the owner id of this Cell.
      * \return the owner id of the Cell
      */
    inline short getOwnerId() const
    {
        return ownerId;
    }

    /*! \brief Adds an entity to this Cell.
      * \param entity the sodaDynamicEntity to add to the Cell
      */
    void addEntity(sodaDynamicEntity *entity);

    /*! \brief Adds an entity to this Cell.
      * \param entity the sodaDynamicEntity to add to the Cell
      */
    void addCellBorder(CellBorderEntity *entity);

    /*! \brief Tells when an entity is within this Cell.
      * \param entity the sodaDynamicEntity to check
      * \return whether this entity is within the Cell
      */
    inline bool containsEntity(sodaDynamicEntity *entity)
    {
        if(entities.isNull())
            return false;
        else
            return entities->contains(entity);
    }

    /*! \brief Remove an entity from this Cell.
      * \param entity the sodaDynamicEntity to remove from the Cell
	  * \return whether the entity could be removed or not
	  */
    bool removeEntity(sodaDynamicEntity *entity);

    /*! \brief Returns the entities of the Cell.
      * \return a pointer to the write-protected sodaDynamicEntity vector
      */
    inline const QVector<sodaDynamicEntity *> *getEntities() const
    {
        return entities.data();
    }

    /*! \brief Returns the entities of the Cell. The dummy parameter indicates the returned vector can be modified.
      * \return a pointer to the sodaDynamicEntity vector for modification
      */
    inline QVector<sodaDynamicEntity *> *getEntities(int)
    {
        return entities.data();
    }

    /*! \brief Returns the borders of the Cell.
      * \return a pointer to the write-protected CellBorderEntity vector
      */
    inline const QVector<CellBorderEntity *> *getBorders() const
    {
        return borders.data();
    }

    /*! \brief Returns the borders of the Cell. The dummy parameter indicates the returned vector can be modified.
      * \return a pointer to the CellBorderEntity vector for modification
      */
    inline QVector<CellBorderEntity *> *getBorders(int)
    {
        return borders.data();
    }

private:
    QSharedPointer<QVector<sodaDynamicEntity *> >    entities; //!< A container for the entities whose center point is within the Cell.
    QSharedPointer<QVector<CellBorderEntity *> >   borders;  //!< A container for entities that simulate a physics border for a Cell.
    short                                          ownerId;  //!< Id of the sodaLogicWorld that owns entities within this Cell.
};
#endif // CELL_H
