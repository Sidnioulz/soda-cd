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
#ifndef OBENTITYTRANSFORMRECORD_H
#define OBENTITYTRANSFORMRECORD_H

#include <btBulletDynamicsCommon.h>
#include "obEntityWrapper.h"

/*! \struct obEntityTransformRecord
  * \brief A structure that contains a btTransform and a pointer to the obEntityWrapper to which it can be applied.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class represents the position and rotation information of an obEntityWrapper
  * at a given time.
  */
struct obEntityTransformRecord {
    obEntityWrapper         *obEnt;             /*!< Pointer to the obEntityWrapper to which the transform can be applied */
    btTransform             transform;          /*!< Position and rotation of the object */
    btVector3               linearVelocity;     /*!< Linear velocity of the entity */
    btVector3               angularVelocity;    /*!< Angular velocity of the entity */

    obEntity::EntityStatus  status;             /*!< Current status for visual feedback */

    /*!
     * \brief Default constructor.
     * \return a new obEntityTransformRecord
     */
    obEntityTransformRecord();

    /*!
     * \brief Constructor with parameters.
     * \param obEnt the entity that this object applies to
     * \param transform the transform to be stored
     * \return a new obEntityTransformRecord
     */
    obEntityTransformRecord(obEntityWrapper *obEnt, const btTransform &transform = btTransform());

    /*!
     * \brief Redefinition of the equal operator. Two obEntityTransformRecords are equal of they concern the same obEntityWrapper.
     * \param other the other obEntityTransformRecord
     * \return true if the obEntityTransformRecords concern the same obEntityWrapper, false otherwise
     *
     * \note If you want to compare the values of two obEntityTransformRecords on the
     * same obEntityWrapper, you may use this == other && this.transform == other.transform.
     */
    inline bool operator ==(const obEntityTransformRecord &other) const
    {
        return obEnt == other.obEnt;
    }

    /*!
     * \brief Redefinition of the not equal operator. Two obEntityTransformRecords are not equal of they concern different obEntityWrappers.
     * \param other the other obEntityTransformRecord
     * \return true if the obEntityTransformRecords concern different obEntityWrappers, false otherwise
     *
     * \note If you want to compare the values of two obEntityTransformRecords on the
     * same obEntityWrapper, you may use this != other || this.transform != other.transform.
     */
    inline bool operator!=(const obEntityTransformRecord &other) const
    {
        return !(*this == other);
    }
};


#endif // OBENTITYTRANSFORMRECORD_H
