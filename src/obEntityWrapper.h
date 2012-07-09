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
#ifndef ENTITYWRAPPER_H
#define ENTITYWRAPPER_H

#include <iostream>
#include <exception>
#include <OgreString.h>

#include "utils.h"
#include "obdynamicrigidbody.h"
#include "obentity.h"
#include "Parser/Animation.h"
#include "Parser/Action.h"
//TODO: make a btShape index that stores and reuses btShapes per name and scale, in order to save memory
//TODO: same for materials

// Forward declaration
class PhysicsWorld;

/*! \class EntityAlreadyExistsException
  * \brief An exception that occurs when a obEntityWrapper is created using the name of an already existing one.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  */
class EntityAlreadyExistsException : public std::exception
{
public:
    /*! \brief Default constructor.
      * \param name name of the already defined entity
      * \return a new EntityAlreadyExistsException
      */
    EntityAlreadyExistsException(const char* &name)
    {
        msg = "Entity name error: entity ";
        msg.append(name);
        msg.append(" already exists.");
    }

    /*! \brief Default constructor.
      * \param name name of the already defined entity
      * \return a new obEntityWrapper
      */
    EntityAlreadyExistsException(const Ogre::String &name)
    {
        msg = "Entity name error: entity ";
        msg.append(name);
        msg.append(" already exists.");
    }

    /*! \brief Default destructor.
      */
    virtual ~EntityAlreadyExistsException() throw()
    {}


    /*! \brief Returns the data to display when throwing an exception.
      * \return the message string of this exception
      */
    virtual const char* what() const throw()
    {
        return msg.c_str();
    }

private:
    std::string msg; //!< Holder for the message string of this exception
};

/*! \class obEntityWrapper
  * \brief A wrapper to manipulate entities both for Ogre and Bullet at the same time
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements entities that are both Ogre::Entity and obDynamicRigidBody.
  * It allows synchronous manipulation of object representations in Ogre and Bullet.
  *
  * \note For historical reasons, this class is named obEntityWrapper. A more suitable
  * name for it would be obDynamicEntity.
  */
class obEntityWrapper : public obEntity
{
public:
    /*! \brief Default constructor.
      * \param name unique name of the entity in the world
      * \param meshName name of the mesh file to use as a shape if the shape parameter is not filled
      * \param pos the initial position of the entity
      * \param quat the initial rotation of the entity
      * \param staticMesh whether the mesh represents a static object
      * \param scale the scale of the entity with regard to the mesh default size
      * \param mass the mass of the entity
      * \param shape bullet shape of the entity
      * \param randomColor if the object is not static, whether to give it a random color
      * \return a new obEntityWrapper
      * \throw EntityAlreadyExistsException if the specified name is already taken by another entity
      */
    obEntityWrapper(const Ogre::String &name,
                    const Ogre::String &meshName,
                    const Ogre::Vector3 &pos = Ogre::Vector3(0,0,0),
                    const Ogre::Quaternion &quat = Ogre::Quaternion::IDENTITY,
                    const bool staticMesh = false,
                    const Ogre::Vector3 &scale = Ogre::Vector3(1,1,1),
                    const int mass = 1,
                    btCollisionShape* shape = NULL,
                    const bool randomColor = false) throw(EntityAlreadyExistsException);

    /*! \brief Default destructor.
      */
	virtual ~obEntityWrapper();

    /*!
     * \brief Returns the type of Entity this object is
     * \return CellBorderEntityType
     */
    inline virtual short getType() const
    {
        return obEntityWrapperType;
    }

    /*! \brief Partially copies the current entity.
      * \return a new obEntityWrapper with the same mesh name.
      * \throw EntityAlreadyExistsException if the generated name is already taken by another entity
      * \deprecated This method makes a very partial copy of the current instance, and
      * it is heavily recommanded not to use it unless you know what you're doig. Please
      * check if the copy constructor fits your needs before using this method.
      *
      * This method creates a new instance of obEntityWrapper using the mesh name
      * of the current one, and setting it as a dynamic mesh, with default origin,
      * rotation and scale parameters.
      */
    obEntityWrapper* clone() const throw(EntityAlreadyExistsException);

    /*! \brief Tells whether the entity has a static mesh.
      * \return whether the object's mesh is static
      */
    inline bool hasStaticMesh() const
    {
        return staticMesh;
    }

    /*! \brief Tells whether the entity is static (static mesh or null mass)
      * \return whether the entity is static
      */
    inline bool isStatic() const
    {
        return hasStaticMesh() || obBody->getMass() == 0;
    }

    /*! \brief Gets the id of the physics world in which the entity exists and is managed.
      * \return the id of the PhysicsWorld that owns the entity
      */
	short getOwnerId() const;

    /*! \brief Gets the physics world in which the entity exists and is managed.
      * \return the PhysicsWorld of the entity
      */
	inline PhysicsWorld* getOwnerWorld() const
	{
		return world;
	}

    /*! \brief Modifies the physics world in which the entity exists and is managed.
      * \param newWorld the new primary physics world
      * \return a pointer to the previous PhysicsWorld (which may be null)
      */
    PhysicsWorld* setOwnerWorld(PhysicsWorld *newWorld);


    /*! \brief Removes to the pointer to the world in which the entity exists (means that it is not attached to any world at the moment).
      * \return a pointer to the previous PhysicsWorld (which may be null)
      */
    PhysicsWorld* unsetOwnerWorld();

    /*! \brief Returns a list of worlds in which the entity exists, but that do not manage its collisions.
	  * \return a const pointer to the vector of worlds in which this object exists
      */
	const QVector<PhysicsWorld*> *getSecondaryWorlds() const;

    /*! \brief Adds a physics world to the list of those in which the entity exists.
      * \param newSecWorld the new secondary physics world
      */
    void addSecondaryWorld(PhysicsWorld *newSecWorld);

    /*! \brief Removes a physics world from the list of those in which the entity exists.
      * \param oldSecWorld the physics world to be removed
      */
    void removeSecondaryWorld(PhysicsWorld *oldSecWorld);

    /*! \brief Gets the size of the entity's bounding box.
      * \return the size of the Ogre::Entity's bounding box scaled with the scale of the obEntityWrapper
      */
    inline btVector3 getSize() const
	{
        return Utils::btVectorFromOgre(getScale() * ogreEntity->getBoundingBox().getSize());
	}

    /*! \brief Gets the animation of the mesh.
      * \return the Animation associated to the mesh
      */
    Animation* getAnimation() const;

    /*! \brief Tells whether the entity has an animation.
      * \return whether the entity has an animation
      */
    bool hasAnimation() const;

    /*! \brief Changes the animation of the entity to a given value.
      * \param anim the animation to give to the entity
      */
    void setAnimation(Animation* anim);

    /*! \brief Gets the position of the entity.
      * \return the vector representing the position
      */
    inline const btVector3 &getPosition() const
	{
		return obBody->getPosition();
	}

    /*! \brief Gets the position of the center of the entity.
      * \return the position of the entity's bottom-left-front corner plus half its size
      */
    inline btVector3 getCenteredPosition() const
	{
        return obBody->getPosition() + (getSize() / 2);
	}

    /*! \brief Gets the orientation of the entity.
      * \return the Quaternion representing the orientation
      */
	inline Ogre::Quaternion getOrientation() const
	{
		return Utils::quaternionFromBullet(obBody->getBulletBody()->getOrientation());
	}

    /*! \brief Changes the frame start number of the entity.
      * \param newFrameStart the new frame start number
      */
    void setFrameStart(const int newFrameStart);

    /*! \brief Gets the frame start number of the entity.
      * \return the frame start number of the entity
      */
    int getFrameStart() const;

    /*! \brief Sets the entity to use the material matching the name passed as a parameter.
      * \param newName the name of the new material to use
      */
	inline void setMaterialName(const Ogre::String &newName)
	{
		materialName = newName;
		ogreEntity->setMaterialName(newName);
	}

    /*! \brief Gets the name of the material used by the entity.
      * \return the name of the entity's material
      */
	inline Ogre::String getMaterialName() const
	{
		return materialName;
	}

    /*! \brief Gets the rigid body of the entity.
	  * \return the obDynamicRigidBody of the entity
      */
    inline obDynamicRigidBody *getRigidBody() const
	{
		return obBody;
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

    /*! \brief Gets the Ogre part of the entity.
      * \return the Ogre::Entity of the entity
      */
	inline Ogre::Entity* getOgreEntity() const
	{
		return ogreEntity;
	}

    /*! \brief Gets the name of the entity's mesh.
      * \return the mesh name of the entity
      */
	inline Ogre::String getMeshName() const
	{
		return meshName;
	}

    /*! \brief Gets the name of this object's Ogre entity.
      * \return the name of the Ogre::Entity associated to this obEntityWrapper
      */
	inline Ogre::String getName() const
	{
		return ogreEntity->getName();
	}

	/*! \brief Gets the mass of the entity.
	  * \return the mass of the entity
	  */
	inline int getMass() const
	{
		return obBody->getMass();
	}

    /*! \brief Gets the scale of the entity.
      * \return the vector representing the scale
      */
	inline Ogre::Vector3 getScale() const
	{
		return obBody->getSceneNode()->getScale();
	}

    /*! \brief Tells whether the color of the entity was randomly chosen.
      * \return whether the entity has a random color
      */
	inline bool hasRandomColor() const
	{
		return randomColor;
	}

    /*! \brief Sets a definite color for the entity.
      * \param r the amount of red in the color to set
      * \param g the amount of green in the color to set
      * \param b the amount of blue in the color to set
      */
	inline void setColor(const float r, const float g, const float b)
	{
        setColor(r, g, b, false);
	}

protected:
    /*! \brief Sets a definite color for the entity.
      * \param r the amount of red in the color to set
      * \param g the amount of green in the color to set
      * \param b the amount of blue in the color to set
      * \param updateRandomColorFlag whether to set the randomColor flag to false
      */
    void setColor(const float r, const float g, const float b, const bool updateRandomColorFlag);

    /*! \brief Sets a random color for the entity.
      */
    void setRandomColor();

private:
    static unsigned int nextInLine;              //!< A number used to generate unique names for objects

	obDynamicRigidBody      *obBody;             //!< Ogre-Bullet rigid body of the entity
    Ogre::Entity            *ogreEntity;         //!< Ogre entity
    Ogre::String            meshName;            //!< Name of the entity's mesh
    PhysicsWorld            *world;              //!< Bullet physics world that manages the entity's collisions
    QVector<PhysicsWorld *> *secWorlds;          //!< Other worlds in which the entity is located

	//FIXME: many probably useless parameters below. Clean it up.
    Animation               *animation;          //!< Current animation of the entity wrapper (not used yet)
    bool                    staticMesh;          //!< Tells whether the entity is static or dynamic
    int                     frameStart;          //!< Starting frame number of the entity (not used yet)
	Ogre::String            materialName;        //!< Name of the entity's material
	bool                    randomColor;         //!< Whether the entity is randomly colored
};

#endif // ENTITYWRAPPER_H
