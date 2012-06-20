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
#include "cellborderentity.h"
#include "utils.h"
#include "localgrid.h"
#include "ogreresources.h"

CellBorderEntity::CellBorderEntity(LocalGrid *grid, const CellBorderCoordinates &coords) throw(EntityAlreadyExistsException) :
	obBody(0),
//    ogreEntity(0),
    grid(grid),
    coords(coords),
    direction(coords.direction())
{
    Ogre::String name = "cell_id:" + Ogre::StringConverter::toString(grid->getOwnerId()) +
            "x:" + Ogre::StringConverter::toString(coords.x()) +
            "y:" + Ogre::StringConverter::toString(coords.y()) +
            "z:" + Ogre::StringConverter::toString(coords.z()) +
            "d:" + Ogre::StringConverter::toString(coords.direction());

	// If the scene manager doesn't already have an entity with this name
    if(!OgreResources::getSceneManager()->hasEntity(name))
    {
        // Compute the world coordinates of the object according to Cell coordinates and direction
        btVector3 worldCoords = grid->getGridInformation()->toDirectedWorldCoordinates(grid->getResolution(), coords);

        // Compute the shape of the body according to the wanted direction
        btVector3 bodyLen =  grid->getGridInformation()->getGridAtResolution(grid->getResolution())->getCellLength();
        if(direction == GridInformation::Top || direction == GridInformation::Bottom)
            bodyLen.setY(0.001);
        if(direction == GridInformation::Left || direction == GridInformation::Right)
            bodyLen.setX(0.001);
        if(direction == GridInformation::Front || direction == GridInformation::Back)
            bodyLen.setZ(0.001);

        // Create the rigid body
        obBody = new obRigidBody(this, name, Utils::vectorFromBullet(worldCoords), Ogre::Quaternion::IDENTITY);
        obBody->createSceneNode();
        obBody->createBorder(bodyLen, true);

        //TODO: create a OgreResources method to get a plane for a given direction and bodyLen.

        // Create an Ogre plane for the entity
        Ogre::Plane p;
        p.d = 0; // Actually useless according to Ogre documentation

        if(direction == GridInformation::Top)
        {
            p.normal = Ogre::Vector3(0, 1, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.x(), bodyLen.z(), 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Z);
        }
        else if(direction == GridInformation::Bottom)
        {
            p.normal = Ogre::Vector3(0, -1, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.x(), bodyLen.z(), 1, 1, true, 1, 1, 1, Ogre::Vector3::NEGATIVE_UNIT_Z);
        }
        else if(direction == GridInformation::Left)
        {
            p.normal = Ogre::Vector3(-1, 0, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.z(), bodyLen.y(), 1, 1, true, 1, 1, 1, Ogre::Vector3::NEGATIVE_UNIT_Y);
        }
        else if(direction == GridInformation::Right)
        {
            p.normal = Ogre::Vector3(1, 0, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.z(), bodyLen.y(), 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Y);
        }
        else if(direction == GridInformation::Back)
        {
            p.normal = Ogre::Vector3(0, 0, -1);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.y(), bodyLen.x(), 1, 1, true, 1, 1, 1, Ogre::Vector3::NEGATIVE_UNIT_X);
        }
        else if(direction == GridInformation::Front)
        {
            p.normal = Ogre::Vector3(0, 0, 1);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.y(), bodyLen.x(), 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_X);
        }

        ogreEntity = OgreResources::getSceneManager()->createEntity(name, name);
        ogreEntity->setMaterialName("Examples/Smoke");
        ogreEntity->setVisible(false);
        obBody->getSceneNode()->attachObject(ogreEntity);

        // Setup a user pointer for later use within the Bullet manager
        // This step is compulsory to be able to retrieve the entity from the broad-phase algorithm.
        obBody->getBulletBody()->getCollisionShape()->setUserPointer(this);
	}
    else
    {
        throw EntityAlreadyExistsException(name);
    }
}

CellBorderEntity::~CellBorderEntity()
{
	delete obBody;
}

void CellBorderEntity::setColor(const float &r, const float &g, const float &b)
{
    // Get the prefix of the entity's material, and then add a suffix for this color
//    Ogre::String prefix;
//    prefix.append(QString(materialName.c_str()).split(":").first().toAscii());

    // For now, the material name is hardcoded for all CellBorderEntities rather than having a materialName attribute
    Ogre::String materialName = "Examples/Smoke";
    materialName.append(QString("color#%1#%2#%3").arg(r).arg(g).arg(b).toAscii());

    // Get (or create) the corresponding material
    Ogre::MaterialPtr mat = OgreResources::createOrRetrieveMaterial(materialName);

    // Set lighting according to the color
    mat->setAmbient(r*1.4,g*1.4,b*1.4);
    mat->setDiffuse(r,g,b,1);
    mat->setSpecular(0,0,0,1);

    // Have the entity use the new material
    ogreEntity->setMaterial(mat);
}
