/*
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Quentin Avril <quentin.avril@irisa.fr>
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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef SODAPERSISTENTFOREIGNERMANIFOLDARRAY_H
#define SODAPERSISTENTFOREIGNERMANIFOLDARRAY_H

#include <btBulletCollisionCommon.h>

// Forward declaration
class sodaPersistentForeignerManifold;

/*! \class sodaPersistentForeignerManifoldArray
  * \brief An override of btPersistentManifold with foreign and local body getters and setters.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a btAlignedObjectArray of sodaPersistentForeignerManifolds with
  * a special linear search method using pointers to bodies rather than a fully
  * built sodaPersistentForeignerManifold.
  */
class sodaPersistentForeignerManifoldArray : public btAlignedObjectArray<sodaPersistentForeignerManifold *>
{
public:
    /*!
     * \brief Override of findLinearSearch() that uses pointers to bodies
     * \param local pointer to the first body of the sodaPersistentForeignerManifold
     * \param foreign pointer to the second body of the sodaPersistentForeignerManifold
     * \return the index of the found body, or the size of the array in case of failure
     */
    int findLinearSearch(const void *local, const void *foreign);
};

#endif // SODAPERSISTENTFOREIGNERMANIFOLDARRAY_H
