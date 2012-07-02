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
#ifndef CELLBORDERENTITY_H
#define CELLBORDERENTITY_H

#include "obEntityWrapper.h" // class EntityAlreadyExistsException

// Forward declaration
class LocalGrid;

/*! \class CellBorderCoordinates
  * \brief A class that indicates the coordinates of a Cell and the direction of a border of this Cell.
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a btVector3 that embeds a directional component. The 3D coordinates
  * represent the location of the Cell in a Cell space, and the direction represents
  * the direction of the border pointed to by an instance of CellBorderCoordinates.
  */
class CellBorderCoordinates : public btVector3
{
public:

    /*!
     * \brief Default constructor.
     * \return a new empty CellBorderCoordinates
     */
    CellBorderCoordinates() :
        btVector3()
    {
    }

    /*!
     * \brief Long constructor.
     * \param x the x component of the Cell coordinates
     * \param y the y component of the Cell coordinates
     * \param z the z component of the Cell coordinates
     * \param direction the direction of this border
     * \return a new CellBorderCoordinates
     */
    CellBorderCoordinates(const btScalar &x, const btScalar &y, const btScalar &z, const int &direction) : btVector3(x, y, z)
    {
        setDirection(direction);
    }

    /*!
     * \brief Compact constructor.
     * \param vect a 3D space vector containing Cell coordinates
     * \param direction the direction of this border
     * \return a new CellBorderCoordinates
     */
    CellBorderCoordinates(const btVector3 &vect, const int &direction) : btVector3(vect)
    {
        setDirection(direction);
    }

    /*!
     * \brief Retrieves the direction of these CellBorderCoordinates.
     * \return the direction of this border
     */
    inline int direction() const
    {
        return w();
    }

    /*!
     * \brief Sets the direction of these CellBorderCoordinates.
     * \param direction the new direction of this border
     */
    inline void setDirection(const int &direction)
    {
        setW(direction);
    }
};


/*! \class CellBorderEntity
  * \brief An entity that is used to detect objects crossing a Cell's border.
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements special static rigid bodies that are used to detect when
  * an obEntityWrapper crosses a Cell's border.
  */
class CellBorderEntity : public obEntity
{
public:
    /*!
     * \brief Default constructor.
     * \param grid the LocalGrid in which this CellBorderEntity will exist
     * \param coords the coordinates in the LocalGrid associated with a direction
     * \return a new CellBorderEntity
     */
    CellBorderEntity(LocalGrid *grid,
                     const CellBorderCoordinates &coords) throw(EntityAlreadyExistsException);

    /*! \brief Default destructor.
      */
    virtual ~CellBorderEntity();

    /*!
     * \brief Returns the type of Entity this object is
     * \return CellBorderEntityType
     */
    inline virtual short getType() const
    {
        return CellBorderEntityType;
    }

    /*! \brief Gets the rigid body of the entity.
      * \return the obRigidBody of the entity
      */
    inline obRigidBody* getRigidBody() const
    {
        return obBody;
    }

    /*!
     * \brief Gets the Cell space coordinates of the entity.
     * \return a btVector3 containing the Cell coordinates of this entity
     */
    inline const CellBorderCoordinates &getCoordinates() const
    {
        return coords;
    }

    /*!
     * \brief Returns the Broadphase proxy to use in btLocalGridBroadphase.
     * \return a pointer to the btBroadphaseProxy of this entity
     *
     * \warning The btBroadphaseProxy currently returned may not be the good one.
     */
    inline btBroadphaseProxy *getBroadphaseHandle() const
    {
        return obBody->getBulletBody()->getBroadphaseProxy();
    }

    /*! \brief Sets a definite color for the entity.
      * \param r the amount of red in the color to set
      * \param g the amount of green in the color to set
      * \param b the amount of blue in the color to set
      */
    void setColor(const float &r, const float &g, const float &b);

private:
	obRigidBody				*obBody;			 /*!< The Ogre-Bullet rigid body of this Cell border */
    Ogre::Entity            *ogreEntity;         /*!< Ogre entity */
    LocalGrid               *grid;               /*!< The grid of which this CellBorderEntity is a border */

    CellBorderCoordinates	coords;		         /*!< Cell coordinates of the border entity, with a direction parameter */
	short					direction;			 /*!< Direction of the border entity on the Cell */
};

#endif // CELLBORDERENTITY_H
