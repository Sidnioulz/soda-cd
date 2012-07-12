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
#ifndef OBENTITY_H
#define OBENTITY_H

#include <QtDebug>

/*! \class obEntity
  * \brief An abstract class used to allow third-parties to store pointers to various kinds of entities.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is just a wrapper to different kind of inheriting entity classes, created to allow
  * storing pointers to entities and retrieving their original class via the dynamic_cast C++ feature.
  */
class obEntity
{
public:

    /*!
     * \brief Default constructor.
     * \return a new obEntity
     */
    obEntity();

    /*!
     * \brief Retrieves the type of the obEntity. Still requires manual management of all supported entity types by clients of obEntity.
     * \return a short number indicative of the type of entity pointed to
     */
    virtual short getType() const = 0;

    static const short obEntityWrapperType = 1;         /*!< See obEntityWrapper class. */
    static const short CellBorderEntityType = 2;        /*!< See CellBorderEntity class. */

    /*!
     * \variable EntityStatus
     * \brief Set of bits corresponding to different possible collision statuses for an object in SODA
     */
    typedef enum __EntityStatus {
        NonExistant = 0,                                /*!< Entity doesn't exist */
        NormalStatus            = (1<<1),               /*!< Entity is in a normal status (usual status of Entities) */
        CrossingBorder          = (1<<2),               /*!< Entity is crossing a border */
        OutOfWorld              = (1<<3),               /*!< Entity has left its world and should be transferred with IPC */
        Removed                 = (1<<4),               /*!< Entity has left the global simulation space and will be deleted */
        Overlapped              = (1<<5)               /*!< Border entity is being overlapped */
    } EntityStatus;

    /*!
     * \brief Sets the status of the obEntity to a new value.
     * \param newStatus the new status of the obEntity
     */
    inline void setStatus(const EntityStatus &newStatus)
    {
        status = newStatus;
    }

    /*!
     * \brief Retrieves the status of the obEntity.
     * \return the status of the obEntity
     */
    inline const EntityStatus &getStatus() const
    {
        return status;
    }

protected:
    EntityStatus status;                                /*!< Status of the obEntity */
};

#endif // OBENTITY_H
