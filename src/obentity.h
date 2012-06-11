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
     * \brief Retrieves the type of the obEntity. Still requires manual management of all supported entity types by clients of obEntity.
     * \return a short number indicative of the type of entity pointed to
     */
    virtual short getType() const = 0;

    static const short obEntityWrapperType = 1;         /*!< See obEntityWrapper class. */
    static const short CellBorderEntityType = 2;        /*!< See CellBorderEntity class. */
};

#endif // OBENTITY_H
