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

#include "sodaUtils.h"
#include "sodaDynamicRigidBody.h"
#include "sodaEntity.h"
#include "Parser/Animation.h"
#include "Parser/Action.h"

// Forward declaration
class sodaLogicWorld;

/*! \class EntityAlreadyExistsException
  * \brief An exception that occurs when a sodaDynamicEntity is created using the name of an already existing one.
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
      * \return a new sodaDynamicEntity
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

/*! \class sodaDynamicEntity
  * \brief A wrapper to manipulate entities both for Ogre and Bullet at the same time
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements entities that are both Ogre::Entity and sodaDynamicRigidBody.
  * It allows synchronous manipulation of object representations in Ogre and Bullet.
  *
  * \note For historical reasons, this class is named sodaDynamicEntity. A more suitable
  * name for it would be sodaDynamicEntity.
  *
  * \attention Ownership of an sodaDynamicEntity is not transferable from world to world.
  * Any sodaDynamicEntity that is created within a world should be deleted within this world.
  * The general practice when messaging an sodaDynamicEntity is to make a deep copy of it,
  * and to let the recipient world decide what to do with it.
  */
class sodaDynamicEntity : public sodaEntity
{
public:
    //TODO: manage default status for sodaDynamicEntity (should be detached until added to a world, then normal)

    /*! \brief Default constructor.
      * \param name unique name of the entity in the world
      * \param meshName name of the mesh file to use as a shape if the shape parameter is not filled
      * \param pos the initial position of the entity
      * \param quat the initial rotation of the entity
      * \param staticMesh whether the mesh represents a static object
      * \param scale the scale of the entity with regard to the mesh default size
      * \param mass the mass of the entity
      * \param shape bullet shape of the entity
      * \return a new sodaDynamicEntity
      * \throw EntityAlreadyExistsException if the specified name is already taken by another entity
      */
    sodaDynamicEntity(const Ogre::String &name,
                    const Ogre::String &meshName,
                    const Ogre::Vector3 &pos = Ogre::Vector3(0,0,0),
                    const Ogre::Quaternion &quat = Ogre::Quaternion::IDENTITY,
                    const bool staticMesh = false,
                    const Ogre::Vector3 &scale = Ogre::Vector3(1,1,1),
                    const int mass = 1,
                    btCollisionShape* shape = NULL) throw(EntityAlreadyExistsException);




    /*! \brief Copy constructor. Creates a worldless sodaDynamicEntity.
      * \param other the sodaDynamicEntity to copy
      * \return a new sodaDynamicEntity
      * \throw EntityAlreadyExistsException if the specified name is already taken by another entity
      *
      * This constructor copies an sodaDynamicEntity from another sodaLogicWorld, but leaves it
      * unattached to any sodaLogicWorld. Both worlds also share a pointer to their Ogre::Entity.
      */
    sodaDynamicEntity(const sodaDynamicEntity &other) throw(EntityAlreadyExistsException);

    /*! \brief Default destructor.
      */
    ~sodaDynamicEntity();

    /*!
     * \brief Returns the type of Entity this object is
     * \return sodaDynamicEntityType
     */
    inline short getType() const
    {
        return DynamicEntityType;
    }

    /*! \brief Partially copies the current entity.
      * \return a new sodaDynamicEntity with the same mesh name.
      * \throw EntityAlreadyExistsException if the generated name is already taken by another entity
      * \deprecated This method makes a very partial copy of the current instance, and
      * it is heavily recommanded not to use it unless you know what you're doig. Please
      * check if the copy constructor fits your needs before using this method.
      *
      * This method creates a new instance of sodaDynamicEntity using the mesh name
      * of the current one, and setting it as a dynamic mesh, with default origin,
      * rotation and scale parameters.
      */
    sodaDynamicEntity* clone() const throw(EntityAlreadyExistsException);

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
      * \return the id of the sodaLogicWorld that owns the entity
      */
	short getOwnerId() const;

    /*! \brief Gets the physics world in which the entity exists and is managed.
      * \return the sodaLogicWorld of the entity
      */
    inline sodaLogicWorld* getOwnerWorld() const
	{
		return world;
	}

    /*! \brief Modifies the physics world in which the entity exists and is managed.
      * \param newWorld the new primary physics world
      * \return a pointer to the previous sodaLogicWorld (which may be null)
      */
    sodaLogicWorld* setOwnerWorld(sodaLogicWorld *newWorld);


    /*! \brief Removes to the pointer to the world in which the entity exists (means that it is not attached to any world at the moment).
      * \return a pointer to the previous sodaLogicWorld (which may be null)
      */
    sodaLogicWorld* unsetOwnerWorld();

    /*! \brief Returns a list of worlds in which the entity exists, but that do not manage its collisions.
	  * \return a const pointer to the vector of worlds in which this object exists
      */
    const QVector<sodaLogicWorld*> *getSecondaryWorlds() const;

      /*! \brief Gets the size of the entity's bounding box.
      * \return the size of the Ogre::Entity's bounding box scaled with the scale of the sodaDynamicEntity
      */
    inline btVector3 getSize() const
	{
        return sodaUtils::btVectorFromOgre(getScale() * ogreEntity->getBoundingBox().getSize());
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
        return sodaUtils::quaternionFromBullet(obBody->getBulletBody()->getOrientation());
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
        materialName.append(newName);
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
      * \return the sodaDynamicRigidBody of the entity
      */
    inline sodaDynamicRigidBody *getRigidBody() const
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

    /*! \brief Gets the name of the entity's mesh.
      * \return the mesh name of the entity
      */
	inline Ogre::String getMeshName() const
	{
		return meshName;
    }

    /*! \brief Gets the name of this object's Ogre entity.
      * \return the name of the Ogre::Entity associated to this sodaDynamicEntity
      */
    inline Ogre::String getName() const
    {
        return ogreEntity->getName();
    }

    /*! \brief Gets the display name of this object's Ogre entity.
      * \return the display name of this sodaDynamicEntity
      */
    inline const char *getDisplayName() const
    {
        return getName().c_str();
    }

	/*! \brief Gets the mass of the entity.
	  * \return the mass of the entity
	  */
	inline int getMass() const
	{
		return obBody->getMass();
    }

    /*!
      * \brief Gets the Ogre scene node of the entity's rigid body.
      * \return the SceneNode of the entity's body
      */
    inline Ogre::SceneNode* getSceneNode() const
    {
        return ogreNode.data();
    }

    /*! \brief Gets the scale of the entity.
      * \return the vector representing the scale
      */
	inline Ogre::Vector3 getScale() const
	{
        return getSceneNode()->getScale();
    }
    /*! \brief Sets a definite color for the entity.
      * \param r the amount of red in the color to set
      * \param g the amount of green in the color to set
      * \param b the amount of blue in the color to set
      */
    void setColor(const float r, const float g, const float b);

protected:
    /*! \brief Sets a random color for the entity.
      */
    void setRandomColor();

private:
    static unsigned int nextInLine;              //!< A number used to generate unique names for objects

    // Reference parameters, do not need to be copied over when an sodaDynamicEntity exists in several worlds
    QSharedPointer<Ogre::Entity>    ogreEntity;          //!< Shared pointer to the Ogre entity of this sodaDynamicEntity
    QSharedPointer<Ogre::SceneNode> ogreNode;            //!< Shared pointer to the Ogre Scene Node of this sodaDynamicEntity's rigid body
    Ogre::String                    meshName;            //!< Name of the entity's mesh
    Ogre::String                    materialName;        //!< Name of the entity's material

    // sodaLogicWorld specific parameters
    btCollisionShape                *shape;              //!< Bullet collision shape given to the sodaDynamicEntity's constructor
    sodaDynamicRigidBody            *obBody;             //!< Bullet rigid body of the entity
    sodaLogicWorld                    *world;              //!< Bullet physics world that manages the entity's collisions

    float                           red;                 /*!< Red component of the sodaDynamicEntity's color */
    float                           green;               /*!< Green component of the sodaDynamicEntity's color */
    float                           blue;                /*!< Blue component of the sodaDynamicEntity's color */

    // sodaLogicWorld specific parameters, not used yet
    bool                            staticMesh;          //!< Tells whether the entity is static or dynamic
    Animation                       *animation;          //!< Current animation of the entity wrapper (not used yet)
    int                             frameStart;          //!< Starting frame number of the entity (not used yet)
};

#endif // ENTITYWRAPPER_H
