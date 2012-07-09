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

QMutex              OgreResources::sceneManagerMutex;
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

Ogre::MaterialPtr OgreResources::createColoredMaterial(const float &r, const float &g, const float &b, const float &f)
{
    // Lock the mutex to avoid race conditions between getting and creating the material
    materialMutex.lock();

    // Create the material name using the color information
    Ogre::String materialName = "SODA/EmptyMaterial";
    materialName.append(QString("color#%1#%2#%3#%4").arg(r).arg(g).arg(b).arg(f).toAscii());

    // Check if the material exists
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(materialName);

    // If it did not exist, create it and color it
    if(mat.isNull())
    {
        // Create the material and a texture unit state
        mat = Ogre::MaterialManager::getSingleton().create(materialName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        mat.getPointer()->createTechnique()->createPass();

        Ogre::Pass *pass = mat.getPointer()->getTechnique(0)->getPass(0);
        pass->setDepthWriteEnabled(false);
        pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        pass->createTextureUnitState();

        // Color the material
        Ogre::TextureUnitState *ptus = mat.getPointer()->getTechnique(0)->getPass(0)->getTextureUnitState(0);
        ptus->setColourOperationEx(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, Ogre::ColourValue(r*1.4,g*1.4,b*1.4, 1));
        ptus->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.2f);
    }

    // Unlock the mutex now that the material is created
    materialMutex.unlock();

    return mat;
}





