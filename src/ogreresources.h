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
#ifndef OGRERESOURCES_H
#define OGRERESOURCES_H

#include <QMutex>
#include <Ogre.h>
#include <QtCore>

/*! \class OgreResources
  * \brief Manages Ogre resources that require the use of the Ogre::ResourceManager singleton.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class presents an interface for all operations that require the use of
  * the Ogre::ResourceManager singleton. It guarantees thread-safety by using
  * mutexes wherever needed.
  */

class OgreResources
{
public:

    /*!
      * \brief Creates, or retrieves if already existing, a Ogre::Material with a given name.
      * \param materialName the name of the new material
      * \return a pointer to the new (or existing) Ogre::Material
      */
    inline static Ogre::MaterialPtr createOrRetrieveMaterial(const Ogre::String &materialName)
    {
        return Ogre::MaterialManager::getSingleton().createOrRetrieve(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).first;
    }

    /*!
      * \brief Clones, or retrieves if already existing, a parent Ogre::Material into a new one with a given name. If parentName is NULL, creates a new blank material.
      * \param materialName the name of the new material
      * \param parentName the name of the parent material to clone
      * \return a pointer to the new (or existing) Ogre::Material
      */
    static Ogre::MaterialPtr createMaterialFromParent(const Ogre::String &materialName, const Ogre::String &parentName = Ogre::String());

    /*!
      * \brief Creates a colored material, possibly transparent, or retrieves it if already existing.
      * \param r the red component of the new color of the material
      * \param g the green component of the new color of the material
      * \param b the blue component of the new color of the material
      * \param f the alpha transparency parameter
      * \return a pointer to the new (or existing) Ogre::Material
      */
    static Ogre::MaterialPtr createColoredMaterial(const float &r, const float &g, const float &b, const float &f = 1.0f);

    /*!
      * \brief Locks the mutex used to restrict access to the Ogre SceneManager.
      *
      * This function locks the mutex used to restrict
      * access to the Ogre SceneManager. It should be used when
      * adding or removing nodes from another thread than the one
      * running the Ogre Widget.
      *
      * \warning Don't forget to call unlockSceneManagerMutex() after using this function.
      */
    static inline void lockSceneManagerMutex()
    {
        sceneManagerMutex.lock();
    }

    /*!
      * \brief Unlocks the mutex used to restrict access to the Ogre SceneManager.
      */
    static inline void unlockSceneManagerMutex()
    {
        sceneManagerMutex.unlock();
    }

    /*!
     * \brief Creates an Ogre root object for the whole application to use.
     * \return a pointer to the created Ogre::Root
     */
    static inline Ogre::Root *createRoot()
    {
#ifndef NDEBUG
        if(ogreRoot!=0)
            qFatal("The Ogre root object has already been initialized and not cleaned up ever since.");

#endif
        ogreRoot = new Ogre::Root("", "", "myOgre.log");
        return ogreRoot;
    }

    /*!
     * \brief Returns a pointer to the Ogre root object of the application.
     * \return a pointer to the Ogre::Root of this application, which will be null if createRoot() has not been called before
     */
    static inline Ogre::Root *getRoot()
    {
#ifndef NDEBUG
        if(ogreRoot==0)
            qFatal("Trying to access the Ogre root object before it was constructed.");

#endif
        return ogreRoot;
    }

    /*!
     * \brief Deletes the Ogre root object for this application.
     */
    static inline void deleteRoot()
    {
        delete ogreRoot;
        ogreRoot = 0;
    }

    /*!
     * \brief Creates an Ogre scene manager for the whole application to use.
     * \return a pointer to the created Ogre::SceneManager
     */
    static inline Ogre::SceneManager *createSceneManager()
    {
#ifndef NDEBUG
        if(ogreSceneManager!=0)
            qFatal("The Ogre scene manager has already been initialized and not cleaned up ever since.");

        else if(ogreRoot==0)
            qFatal("Cannot create an Ogre scene manager without creating an Ogre root object first.");
#endif
        ogreSceneManager = ogreRoot->createSceneManager(Ogre::ST_GENERIC);
        return ogreSceneManager;
    }

    /*!
     * \brief Returns a pointer to the Ogre scene manager of the application.
     * \return a pointer to the Ogre::SceneManager of this application, which will be null if createSceneManager() has not been called before
     *
     * \warning The Ogre scene manager should never be shared between threads.
     * This function does not manage resource locking for client threads.
     */
    static inline Ogre::SceneManager *getSceneManager()
    {
#ifndef NDEBUG
        if(ogreSceneManager==0)
            qFatal("Trying to access the Ogre scene manager before it was constructed.");

#endif
        return ogreSceneManager;
    }

    /*!
     * \brief Deletes the Ogre scene manager for this application.
     */
    static inline void deleteSceneManager()
    {
        delete ogreSceneManager;
        ogreSceneManager = 0;
    }

private:
    // Variables for materials
    static QMutex             materialMutex;      //!< A mutex for management of materials
    static QMutex             sceneManagerMutex;  //!< A mutex for access to the scene manager

    static Ogre::Root         *ogreRoot;          //!< The root object for the Ogre 3D engine */
    static Ogre::SceneManager *ogreSceneManager;  //!< The Ogre 3D scene manager of the application */
};

#endif // OGRERESOURCES_H
