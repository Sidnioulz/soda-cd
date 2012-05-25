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
#ifndef OBGHOSTENTITY_H
#define OBGHOSTENTITY_H

#include <OgreString.h>

#include "utils.h"
#include "obEntityWrapper.h"

/*! \class obGhostEntity
  * \brief An object that marks a given location in the 3D scene, without corresponding to an exhisting physics entity
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is merely used to locate a given point in the 3D scene. It
  * may contain in the future methods for drawing 3D objects, but does not
  * contain a Bullet rigid body. It may be used by clustering algorithms.
  *
  * \note This class uses only Ogre::Vector3 vectors since it is not designed for use with Bullet.
  */
class obGhostEntity
{
public:
    /*! \brief Default constructor.
      * \param name unique name of the entity in the world
      * \param visible whether the entity should be visible in the 3D world
      * \param meshName name of the mesh file to use as a shape if the shape parameter is not filled
      * \param pos the initial position of the entity
      * \param quat the initial rotation of the entity
      * \param scale the scale of the entity with regard to the mesh default size
      * \param randomColor if the object is not static, whether to give it a random color
      * \return a new obGhostEntity
      * \throw EntityAlreadyExistsException if the specified name is already taken by another entity
      */
    obGhostEntity(const Ogre::String &name,
                  const bool &visible,
                  const Ogre::String &meshName,
                  const Ogre::Vector3 &pos = Ogre::Vector3(0,0,0),
                  const Ogre::Quaternion &quat = Ogre::Quaternion::IDENTITY,
                  const Ogre::Vector3 &scale = Ogre::Vector3(1,1,1),
                  const bool &randomColor = false) throw(EntityAlreadyExistsException);

    /*! \brief Default destructor.
      */
    virtual ~obGhostEntity();

    /*!
     * \brief Sets the visibility of the obGhostEntity in the 3D scene.
     * \param visible whether the Ogre::Entity of this object should be visible or not
     */
    inline virtual void setVisible(const bool &visible)
    {
        ogreEntity->setVisible(visible);
    }

    /*!
     * \brief Tells whether the obGhostEntity is visible in the 3D scene.
     * \return whether the Ogre::Entity of this object is visible
     */
    inline virtual bool isVisible() const
    {
        return ogreEntity->isVisible();
    }

    /*! \brief Gets the size of the entity's bounding box.
      * \return the size of the Ogre::Entity's bounding box scaled with the scale of the obEntityWrapper
      */
    inline Ogre::Vector3 getSize() const
    {
        return getScale() * ogreEntity->getBoundingBox().getSize();
    }

    /*! \brief Gets the position of the entity.
      * \return the vector representing the position
      */
    inline Ogre::Vector3 getPosition() const
    {
        return position;
    }

    /*! \brief Gets the position of the center of the entity.
      * \return the position of the entity's bottom-left-front corner plus half its size
      */
    inline Ogre::Vector3 getCenteredPosition() const
    {
        return position + (getSize() / 2);
    }

    /*! \brief Gets the orientation of the entity.
      * \return the Quaternion representing the orientation
      */
    inline Ogre::Quaternion getOrientation() const
    {
        return quaternion;
    }

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

    /*!
      * \brief Gets the Ogre scene node of the rigid body.
      * \return the SceneNode of the body
      */
    inline Ogre::SceneNode* getSceneNode() const
    {
        return node;
    }

    /*!
      * \brief Sets the rotation and motion of a rigid body to be those in parameter
      * \param rotation the rotation to apply as a (x,y,z) vector
      * \param direction the motion to apply as a (x,y,z) vector
      */
    void setTransformation(const Ogre::Vector3 &rotation, const Ogre::Vector3 &direction);

    /*! \brief Gets the Ogre::Entity of this object.
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

    /*! \brief Gets the scale of the entity.
      * \return the vector representing the scale
      */
    inline Ogre::Vector3 getScale() const
    {
        return node->getScale();
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
    Ogre::Vector3           position;           /*!< Position of the object at creation time */
    Ogre::Quaternion        quaternion;         /*!< Object rotation at creation time */

    Ogre::String            materialName;       /*!< Name of the entity's material */
    bool                    randomColor;        /*!< Whether the entity is randomly colored */

    Ogre::Entity            *ogreEntity;        /*!< Ogre entity */
    Ogre::String            meshName;           /*!< Name of the entity's mesh */
    Ogre::SceneManager      *ogreSceneManager;  /*!< Ogre scene manager in which rendering is done */
    Ogre::SceneNode		    *node;				/*!< Ogre scene node of the body */
};

#endif // OBGHOSTENTITY_H
