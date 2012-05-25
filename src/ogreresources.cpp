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
#include "ogreresources.h"

QMutex              OgreResources::materialMutex;
Ogre::Root         *OgreResources::ogreRoot = 0;
Ogre::SceneManager *OgreResources::ogreSceneManager = 0;

Ogre::MaterialPtr OgreResources::createMaterialFromParent(const Ogre::String &materialName, const Ogre::String &parentName)
{
    // Lock the mutex to avoid race conditions between getting and creating the material
    materialMutex.lock();

    // Check if the material exists
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(materialName);

    // If it did not exist
    if(mat.isNull())
    {
        // Try to retrieve the parent material
        mat = Ogre::MaterialManager::getSingleton().getByName(parentName);

        // If the parent did not exist, create a new material and get its pointer
        if(mat.isNull())
            mat = createOrRetrieveMaterial(materialName);
        // Else, clone the parent and get a pointer to the new material
        else
        {
            mat->clone(materialName);
            mat = Ogre::MaterialManager::getSingleton().getByName(materialName);
        }
    }

    // Unlock the mutex now that the material is created
    materialMutex.unlock();

    return mat;
}
