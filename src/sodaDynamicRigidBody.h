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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef SODADYNAMICRIGIDBODY_H
#define SODADYNAMICRIGIDBODY_H

#include <Ogre.h>
#include <OgreVector3.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include "sodaMotionState.h"
#include "sodaRigidBody.h"

// Forward declaration
class sodaDynamicEntity;

/*! \class sodaDynamicRigidBody
  * \brief A dynamic rigid body
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * The dynamic version of sodaRigidBody includes a special implementation of motion
  * states (obMotionState).
  */
class sodaDynamicRigidBody : public sodaRigidBody
{
public:
    /*!
      * \brief Default constructor.
      * \param parent the parent sodaDynamicEntity
      * \param pos the initial position of the body
      * \param quat the initial rotation of the body
      * \param scale the scale of the body with regard to the mesh default size
      * \param mass the mass of the body
      * \return a new sodaDynamicRigidBody
      */
    sodaDynamicRigidBody(sodaDynamicEntity *parent,
                       const btVector3 &pos,
                       const btQuaternion &quat,
                       const btVector3 &scale = btVector3(1,1,1),
                       const int mass = 0);
    /*!
      * \brief Special copy constructor.
      * \param parent the parent sodaDynamicEntity of this instance
      * \param other the sodaDynamicRigidBody whose parameters to copy
      * \return a new sodaDynamicRigidBody
      *
      * \warning This constructor does not copy the other sodaDynamicRigidBody's btRigidBody.
      * Clients still have to create the btRigidBody themselves after constructing the
      * sodaDynamicRigidBody.
      */
    sodaDynamicRigidBody(sodaDynamicEntity *parent,
                       const sodaDynamicRigidBody &other);

	/*!
	  * \brief Returns the parent sodaDynamicEntity of this rigid body
	  * \return the parent sodaDynamicEntity of this object
	  */
	inline sodaDynamicEntity *getParent() const
	{
		return parent;
	}

	/*!
	  * \brief Gets the motion state of this rigid body.
	  * \return the obMotionState of this object's btRigidBody
	  */
	inline sodaMotionState *getMotionState() const
	{
		return mState;
	}

private:
    /*!
      * \brief Creates a motion state for the Bullet rigid body.
      * \return a pointer to a new obMotionState for this obDynamicRigidBody
      */
    virtual btMotionState *_createMotionState();

    sodaDynamicEntity     *parent;        /**< Parent entity of this rigid body */
    sodaMotionState       *mState;        /**< Custom Bullet motion state */
};

#endif // SODADYNAMICRIGIDBODY_H
