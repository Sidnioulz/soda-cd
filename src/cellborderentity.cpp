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
#include "sodaUtils.h"
#include "sodaLocalGrid.h"
#include "sodaOgreResources.h"

void CellBorderCoordinates::getOtherSide(CellBorderCoordinates &otherSideCoords) const
{
    if(direction() == GridInformation::Top)
    {
        otherSideCoords.setX(x());
        otherSideCoords.setY(y() + 1);
        otherSideCoords.setZ(z());
        otherSideCoords.setDirection(GridInformation::Bottom);
    }
    else if(direction() == GridInformation::Bottom)
    {
        otherSideCoords.setX(x());
        otherSideCoords.setY(y() - 1);
        otherSideCoords.setZ(z());
        otherSideCoords.setDirection(GridInformation::Top);
    }

    else if(direction() == GridInformation::Left)
    {
        otherSideCoords.setX(x() - 1);
        otherSideCoords.setY(y());
        otherSideCoords.setZ(z());
        otherSideCoords.setDirection(GridInformation::Right);
    }
    else if(direction() == GridInformation::Right)
    {
        otherSideCoords.setX(x() + 1);
        otherSideCoords.setY(y());
        otherSideCoords.setZ(z());
        otherSideCoords.setDirection(GridInformation::Left);
    }

    else if(direction() == GridInformation::Back)
    {
        otherSideCoords.setX(x());
        otherSideCoords.setY(y());
        otherSideCoords.setZ(z() - 1);
        otherSideCoords.setDirection(GridInformation::Front);
    }
    else if(direction() == GridInformation::Front)
    {
        otherSideCoords.setX(x());
        otherSideCoords.setY(y());
        otherSideCoords.setZ(z() + 1);
        otherSideCoords.setDirection(GridInformation::Back);
    }
}

CellBorderEntity::CellBorderEntity(sodaLocalGrid *grid, const CellBorderCoordinates &coords) throw(EntityAlreadyExistsException) :
    sodaEntity(),
	rigidBody(0),
    ogreEntity(0),
    ogreNode(0),
    grid(grid),
    coords(coords),
    red(1),
    green(1),
    blue(1),
    alpha(0.2)
{
    Ogre::String name = "cell_id:" + Ogre::StringConverter::toString(grid->getOwnerId()) +
            "x:" + Ogre::StringConverter::toString(coords.x()) +
            "y:" + Ogre::StringConverter::toString(coords.y()) +
            "z:" + Ogre::StringConverter::toString(coords.z()) +
            "d:" + Ogre::StringConverter::toString(coords.direction());

	// If the scene manager doesn't already have an entity with this name
    if(!sodaOgreResources::getSceneManager()->hasEntity(name))
    {
        // Compute the world coordinates of the object according to Cell coordinates and direction
        btVector3 worldCoords = grid->getGridInformation()->toDirectedWorldCoordinates(grid->getResolution(), coords);

        // Compute the shape of the body according to the wanted direction
        btVector3 bodyLen =  grid->getGridInformation()->getGridAtResolution(grid->getResolution())->getCellLength();
        if(coords.direction() == GridInformation::Top || coords.direction() == GridInformation::Bottom)
            bodyLen.setY(0.001);
        if(coords.direction() == GridInformation::Left || coords.direction() == GridInformation::Right)
            bodyLen.setX(0.001);
        if(coords.direction() == GridInformation::Front || coords.direction() == GridInformation::Back)
            bodyLen.setZ(0.001);

        // Create the rigid body
        rigidBody = new sodaRigidBody(this, worldCoords, btQuaternion::getIdentity());
        ogreNode = sodaOgreResources::getSceneManager()->getRootSceneNode()->createChildSceneNode(name, sodaUtils::vectorFromBullet(worldCoords), Ogre::Quaternion::IDENTITY);
//        ogreNode->showBoundingBox(true);
        rigidBody->createBorder(bodyLen, true);

        //TODO: create a sodaOgreResources method to get a plane for a given direction and bodyLen.

        // Create an Ogre plane for the entity
        Ogre::Plane p;
        p.d = 0; // Actually useless according to Ogre documentation

        if(coords.direction() == GridInformation::Top)
        {
            p.normal = Ogre::Vector3(0, 1, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.x(), bodyLen.z(), 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Z);
        }
        else if(coords.direction() == GridInformation::Bottom)
        {
            p.normal = Ogre::Vector3(0, -1, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.x(), bodyLen.z(), 1, 1, true, 1, 1, 1, Ogre::Vector3::NEGATIVE_UNIT_Z);
        }
        else if(coords.direction() == GridInformation::Left)
        {
            p.normal = Ogre::Vector3(-1, 0, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.z(), bodyLen.y(), 1, 1, true, 1, 1, 1, Ogre::Vector3::NEGATIVE_UNIT_Y);
        }
        else if(coords.direction() == GridInformation::Right)
        {
            p.normal = Ogre::Vector3(1, 0, 0);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.z(), bodyLen.y(), 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Y);
        }
        else if(coords.direction() == GridInformation::Back)
        {
            p.normal = Ogre::Vector3(0, 0, -1);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.y(), bodyLen.x(), 1, 1, true, 1, 1, 1, Ogre::Vector3::NEGATIVE_UNIT_X);
        }
        else if(coords.direction() == GridInformation::Front)
        {
            p.normal = Ogre::Vector3(0, 0, 1);
            Ogre::MeshManager::getSingleton().createPlane(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                          p, bodyLen.y(), bodyLen.x(), 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_X);
        }

        ogreEntity = sodaOgreResources::getSceneManager()->createEntity(name, name);
        updateColor();
        ogreNode->attachObject(ogreEntity);

        // Setup a user pointer for later use within the Bullet manager
        // This step is compulsory to be able to retrieve the entity from the broad-phase algorithm.
        rigidBody->getBulletBody()->getCollisionShape()->setUserPointer(this);
	}
    else
    {
        throw EntityAlreadyExistsException(name);
    }
}

CellBorderEntity::~CellBorderEntity()
{
	delete rigidBody;
}

void CellBorderEntity::updateColor()
{
    // Get (or create) the colored version of the default CellBorderEntity material
    ogreEntity->setMaterial(sodaOgreResources::createColoredMaterial(red, green, blue, alpha));
}

void CellBorderEntity::setColor(const float &r, const float &g, const float &b)
{
    red = r;
    green = g;
    blue = b;
    updateColor();
}

void CellBorderEntity::setAlpha(const float &f)
{
    alpha = f;
    updateColor();
}
