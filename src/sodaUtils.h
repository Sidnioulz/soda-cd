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
#ifndef OGREBULLETUTILS_H
#define OGREBULLETUTILS_H

#include <QString>
#include <OgreVector3.h>
#include <OgreEntity.h>
#include <OgreString.h>
#include <OgreQuaternion.h>
#include <btBulletDynamicsCommon.h>
#include <blitz/array.h>

/*! \namespace sodaUtils
  * \brief Various type conversion utilities.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This file contains type conversion utilities from and to Qt, Bullet and Ogre.
  */
namespace sodaUtils
{
    //! Converts a btVector3 into a Ogre::Vector3.
    static inline Ogre::Vector3 vectorFromBullet(const btVector3 &vector)
    {
        return Ogre::Vector3(vector.getX(),vector.getY(), vector.getZ());
    }

    //! Converts a Ogre::Vector3 into a btVector3.
    static inline btVector3 btVectorFromOgre(const Ogre::Vector3 &vector)
    {
        return btVector3(vector.x, vector.y, vector.z);
    }

    //! Converts a Ogre::Vector3 into a btVector3.
    static inline btVector3 btVectorFromBlitz(const blitz::TinyVector<int, 3> &vector)
    {
        return btVector3(vector[0], vector[1], vector[2]);
    }

    //! Converts a Ogre::String into a QString.
    static inline QString qStringFromOgre(const Ogre::String &str)
    {
        return QString(str.c_str());
    }

    //! Converts a Ogre::Quaternion into a btQuaternion.
    static inline btQuaternion btQuaternionFromOgre(const Ogre::Quaternion &quat)
    {
        return btQuaternion(btScalar(quat.x),btScalar(quat.y),btScalar(quat.z),btScalar(quat.w));
    }

    //! Converts a btQuaternion into a Ogre::Quaternion.
    static inline Ogre::Quaternion quaternionFromBullet(const btQuaternion &quat)
    {
        return Ogre::Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
    }
}

#endif // OGREBULLETUTILS_H
