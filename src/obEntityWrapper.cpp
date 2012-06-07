/*
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Quentin Avril <quentin.avril@inria.fr>
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

#include <OgreString.h>
#include "obEntityWrapper.h"
#include "physicsworld.h"
#include "Parser/Animation.h"
#include "Parser/Action.h"
#include "utils.h"
#include "ogreresources.h"

unsigned int obEntityWrapper::nextInLine = 0;

obEntityWrapper::obEntityWrapper(const Ogre::String &name, const Ogre::String &meshName, const Ogre::Vector3 &pos, const Ogre::Quaternion &quat, const bool staticMesh, const Ogre::Vector3 &scale, const int mass, btCollisionShape *shape, const bool randomColor)  throw(EntityAlreadyExistsException) :
    obBody(0),
    ogreEntity(0),
    meshName(meshName),
    world(0),
	secWorlds(),
	animation(0),
    staticMesh(staticMesh),
	frameStart(0),
    randomColor(randomColor)

{
    // If the scene manager doesn't already have an entity with this name
    if (!OgreResources::getSceneManager()->hasEntity(name))
    {
        // Create the rigid body and its Ogre entity
        obBody = new obDynamicRigidBody(this, name, pos, quat, scale, mass);
        obBody->createSceneNode();

        ogreEntity = OgreResources::getSceneManager()->createEntity(name, meshName);
        ogreEntity->setCastShadows(true);
        obBody->getSceneNode()->attachObject(ogreEntity);

        // If no shape is defined, create one from the mesh
        if(!shape)
        {
            if (staticMesh)
                obBody->createMeshCollider(ogreEntity->getMesh().getPointer());
            else
                obBody->createBody(ogreEntity->getMesh().getPointer());
        }
        else
            obBody->createBodyWithShape(shape, staticMesh);

        // Setup a user pointer for later use within the Bullet manager
        obBody->getBulletBody()->getCollisionShape()->setUserPointer(this);
    }
    else
    {
        throw EntityAlreadyExistsException(name);
    }

	obBody->getBulletBody()->getWorldTransform().setOrigin(btVector3(pos.x, pos.y, pos.z));
	obBody->getBulletBody()->getWorldTransform().setRotation(Utils::btQuaternionFromOgre(quat));

	// Assign a random color to the object if not static
	if(randomColor && !staticMesh)
		setRandomColor();
}

obEntityWrapper::obEntityWrapper(const obEntityWrapper &other) throw(EntityAlreadyExistsException)
{
    Ogre::String name = other.ogreEntity->getName()+Ogre::StringConverter::toString(nextInLine++);
    obEntityWrapper(name,
                    other.getMeshName(),
                    Utils::vectorFromBullet(other.getPosition()),
                    other.getOrientation(),
                    other.hasStaticMesh(),
                    other.getScale(),
                    other.getMass(),
                    other.getRigidBody()->getShape(),
                    other.hasRandomColor());
}

obEntityWrapper::~obEntityWrapper()
{
	delete obBody;
	delete animation;
}

obEntityWrapper* obEntityWrapper::clone() const throw(EntityAlreadyExistsException)
{
   return new obEntityWrapper(ogreEntity->getName()+Ogre::StringConverter::toString(nextInLine++),
                              meshName,
                              Ogre::Vector3::ZERO,
                              Ogre::Quaternion::IDENTITY,
                              false);
}

short obEntityWrapper::getOwnerId() const
{
	if(world)
		return world->getId();
	else
		return PhysicsWorld::NullWorldId;
}

PhysicsWorld* obEntityWrapper::setOwnerWorld(PhysicsWorld *newWorld)
{
    PhysicsWorld *oldWorld=world;
    world = newWorld;

    return oldWorld;
}

PhysicsWorld* obEntityWrapper::unsetOwnerWorld()
{
    PhysicsWorld *oldWorld=world;
    world = NULL;

    return oldWorld;
}

const QVector<PhysicsWorld*> *obEntityWrapper::getSecondaryWorlds() const
{
    //TODO: implement getSecondaryWorlds
    qDebug("TODO");
	return secWorlds;
}

void obEntityWrapper::addSecondaryWorld(PhysicsWorld *newSecWorld)
{
    //TODO: implement addSecondaryWorld
    qDebug("TODO");
}

void obEntityWrapper::removeSecondaryWorld(PhysicsWorld *oldSecWorld)
{
    //TODO: implement removeSecondaryWorld
    qDebug("TODO");
}

Animation* obEntityWrapper::getAnimation() const
{
	qDebug("TODO");
    if(isStatic())
        return animation;
    else
        return NULL;
}

bool obEntityWrapper::hasAnimation() const
{
	qDebug("TODO");
    return animation!=NULL;
}

void obEntityWrapper::setAnimation(Animation* anim)
{
	qDebug("TODO");
    animation = anim;
}

void obEntityWrapper::setFrameStart(const int newFrameStart)
{
	qDebug("TODO");
	frameStart = newFrameStart;
}

int obEntityWrapper::getFrameStart() const
{
	qDebug("TODO");
	return frameStart;
}

void obEntityWrapper::setColor(const float r, const float g, const float b, const bool randomColorFlag)
{
    // Get the prefix of the entity's material, and then add a suffix for this color
    Ogre::String prefix;
    prefix.append(QString(materialName.c_str()).split(":").first().toAscii());

    materialName = prefix;
    materialName.append(QString("color#%1#%2#%3").arg(r).arg(g).arg(b).toAscii());

    // Get (or create) the corresponding material
    Ogre::MaterialPtr mat = OgreResources::createOrRetrieveMaterial(materialName);

    // Set lighting according to the color
    mat->setAmbient(r,g,b);
    mat->setDiffuse(r,g,b,1);
    mat->setSpecular(0,0,0,1);

    // Have the entity use the new material
    ogreEntity->setMaterial(mat);

    // Update the random color flag
    randomColor = randomColorFlag;
}

void obEntityWrapper::setRandomColor()
{
    // Random color selection
    float r = (float)((float)rand() / ((float)RAND_MAX + 1));
    float g = (float)((float)rand() / ((float)RAND_MAX + 1));
    float b = (float)((float)rand() / ((float)RAND_MAX + 1));

    setColor(r, g, b, true);
}
