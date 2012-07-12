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
#ifndef OGREOBJECTSTATE_H
#define OGREOBJECTSTATE_H

#include <Ogre.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

// Forward declarations
class obEntityWrapper;
class obDynamicRigidBody;
class LocalGrid;

/*! \class obMotionState
  * \brief A class for managing the position and orientation of a obRigidBody.
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class contains the getWorldTransform and setWorldTransform methods of the motionState
  * corresponding to dual Ogre-Bullet implementations of rigid bodies from the obRigidBody class.
  *
  * It is used to match Bullet object coordinate changes (expressed as btMotionState objects) to
  * the Ogre versions of these objects, by overriding the methods of this class to use Ogre object
  * coordinates.
  *
  * \note For historical reasons, this class is named obMotionState. It could be named sodaMotionState.
  */
class obMotionState : public btMotionState
{
public:
    /*!
      * \brief Default constructor.
      * \param parent the rigid body that this motion state belongs to
      * \return a new obMotionState
      */
    obMotionState(obEntityWrapper *parent);


    /*!
      * \brief Special copy constructor.
      * \param parent the rigid body that this motion state belongs to
      * \param other the btMotionState whose parameters to copy
      * \return a new obMotionState
      *
      * \warning This constructor does not copy the other obMotionState's grid and
      * cell coordinates, but uses another btMotionState's information instead of
      * being empty.
      */
    obMotionState(obEntityWrapper *parent, const btMotionState &other);

    /*!
      * \brief Returns the LocalGrid this object is within, or NULL if the object is outside its previous LocalGrid.
      * \return the LocalGrid that owns the obMotionState's parent, or NULL if the parent is already out of the grid
      */
    inline LocalGrid *getLocalGrid() const
    {
        return grid;
    }

    /*!
      * \brief Sets the LocalGrid this object is associated with (it must be within the grid).
      * \param newGrid the LocalGrid that owns the obMotionState's parent
      */
    void setLocalGrid(LocalGrid *newGrid);

    /*!
      * \brief Sets the LocalGrid this object is associated with to NULL (use to detach object from a grid).
      */
    void unsetLocalGrid();

    /*!
      * \brief Returns the world transform of this motion state.
      * \param worldTrans a reference to the variable where to store the world transform
      */
    void getWorldTransform(btTransform &worldTrans) const;

    /*!
      * \brief Changes the world transform of this motion state to a given one.
      * \param worldTrans the transform to set this motion state to
      */
    void setWorldTransform(const btTransform &worldTrans);

    /*!
      * \brief Returns the last known Cell coordinates of this motion state.
      * \return the last known Cell coordinates of this object
      */
    inline btVector3 getCellCoordinates() const
    {
        return lastCellCoords;
    }

private:
	obEntityWrapper *parentBody;      //!< The rigid body that uses this motion state implementation
	LocalGrid       *grid;            //!< The grid that owns the object
	btVector3       lastCellCoords;   //!< Last known cell coordinates of the object
};

#endif // OGREOBJECTSTATE_H
