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
#include "sodaDynamicEntity.h"

/*! \struct obEntityTransformRecord
  * \brief A structure that contains a btTransform and a pointer to the sodaDynamicEntity to which it can be applied.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class represents the position and rotation information of a sodaDynamicEntity
  * at a given time.
  */
struct obEntityTransformRecord {
    Ogre::String            entityName;         /*!< Name of the entity */
    btTransform             transform;          /*!< Position and rotation of the object */
    btVector3               linearVelocity;     /*!< Linear velocity of the entity */
    btVector3               angularVelocity;    /*!< Angular velocity of the entity */

    sodaEntity::EntityStatus  status;             /*!< Current status for visual feedback */

    /*!
     * \brief Default constructor.
     * \return a new obEntityTransformRecord
     */
    obEntityTransformRecord();

    /*!
     * \brief Constructor with parameters.
     * \param entityName the name of the entity
     * \param status the status of the entity
     * \param transform the transform to be stored
     * \return a new obEntityTransformRecord
     */
    obEntityTransformRecord(Ogre::String entityName, const sodaEntity::EntityStatus &status, const btTransform &transform = btTransform());

    /*!
     * \brief Redefinition of the equal operator. Two obEntityTransformRecords are equal of they concern the same sodaDynamicEntity.
     * \param other the other obEntityTransformRecord
     * \return true if the obEntityTransformRecords concern the same sodaDynamicEntity, false otherwise
     *
     * \note If you want to compare the values of two obEntityTransformRecords on the
     * same sodaDynamicEntity, you may use this == other && this.transform == other.transform.
     */
    inline bool operator ==(const obEntityTransformRecord &other) const
    {
        return entityName == other.entityName;
    }

    /*!
     * \brief Redefinition of the not equal operator. Two obEntityTransformRecords are not equal of they concern different sodaDynamicEntities.
     * \param other the other obEntityTransformRecord
     * \return true if the obEntityTransformRecords concern different sodaDynamicEntities, false otherwise
     *
     * \note If you want to compare the values of two obEntityTransformRecords on the
     * same sodaDynamicEntity, you may use this != other || this.transform != other.transform.
     */
    inline bool operator!=(const obEntityTransformRecord &other) const
    {
        return !(*this == other);
    }
};


#endif // OBENTITYTRANSFORMRECORD_H
