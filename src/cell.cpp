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
#include <QMapIterator>
#include <QVectorIterator>
#include "cell.h"
#include "physicsworld.h"


namespace {
    //! Deletes CellBorderEntities in a QVector<CellBorderEntity *>
    void deleteCellBorderEntity(QVector<CellBorderEntity *> *vector)
    {
        if(vector)
            qDeleteAll(*vector);
        delete vector;
    }
}


Cell::Cell() :
    entities(0),
    borders(0),
    ownerId(PhysicsWorld::NullWorldId)
{
}

Cell::Cell(const short ownerId) :
    entities(0),
    borders(0),
    ownerId(ownerId)
{
}

Cell::Cell(const Cell &other) :
    entities(other.entities),
    borders(other.borders),
    ownerId(other.ownerId)
{
}

Cell::~Cell()
{
}


void Cell::addEntity(obEntityWrapper *entity)
{
    Q_ASSERT(entity != NULL);

    if(entities.isNull())
        entities = QSharedPointer<QVector<obEntityWrapper *> >(new QVector<obEntityWrapper *>(1, entity));
    else
    {
        Q_ASSERT(!entities->contains(entity));
        entities->append(entity);
    }
}

void Cell::addCellBorder(CellBorderEntity *entity)
{
    Q_ASSERT(entity != NULL);

    if(borders.isNull())
        borders = QSharedPointer<QVector<CellBorderEntity *> >(new QVector<CellBorderEntity *>(1, entity), deleteCellBorderEntity);
    else
    {
        Q_ASSERT(!borders->contains(entity));
        borders->append(entity);
    }
}

bool Cell::removeEntity(obEntityWrapper *entity)
{
    if(!entities.isNull())
        for(int i=0; i<entities->size(); ++i)
            if(entity == entities->at(i))
            {
                entities->remove(i);

                if(entities->isEmpty())
                    entities.clear();

                return true;
            }

    return false;
}
