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

#include <QSharedPointer>
#include <OgreString.h>
#include "obEntityWrapper.h"
#include "physicsworld.h"
#include "Parser/Animation.h"
#include "Parser/Action.h"
#include "utils.h"
#include "ogreresources.h"

unsigned int obEntityWrapper::nextInLine = 0;

obEntityWrapper::obEntityWrapper(const Ogre::String &name, const Ogre::String &meshName, const Ogre::Vector3 &pos, const Ogre::Quaternion &quat, const bool staticMesh, const Ogre::Vector3 &scale, const int mass, btCollisionShape *shape)  throw(EntityAlreadyExistsException) :
    ogreEntity(0),
    ogreNode(0),
    meshName(meshName),
    shape(shape),
    obBody(0),
    world(0),
    red(0),
    green(0),
    blue(0),
    staticMesh(staticMesh),
    animation(0),
    frameStart(0)
{
    // If the scene manager doesn't already have an entity with this name
    if (!OgreResources::getSceneManager()->hasEntity(name))
    {
        // Create the rigid body
        obBody = new obDynamicRigidBody(this,
                                        Utils::btVectorFromOgre(pos),
                                        Utils::btQuaternionFromOgre(quat),
                                        Utils::btVectorFromOgre(scale),
                                        mass);

        ogreEntity = QSharedPointer<Ogre::Entity>(OgreResources::getSceneManager()->createEntity(name, meshName), Utils::deleteOgreEntity);
        ogreEntity->setCastShadows(true);

        ogreNode = QSharedPointer<Ogre::SceneNode>(OgreResources::getSceneManager()->getRootSceneNode()->createChildSceneNode(name, pos, quat), Utils::deleteSceneNode);
        ogreNode->scale(scale);
        getSceneNode()->attachObject(ogreEntity.data());

        // If no shape is defined, create one from the mesh
        if(!shape)
        {
//            if(meshName == "cube.mesh")
//                obBody->createCube(staticMesh);
//            else if(meshName == "sphere.mesh")
//                obBody->createSphere(staticMesh);
//            else if(meshName == "simplePlane.mesh")
//                obBody->createPlane(staticMesh);
//            else if(meshName == "cylinder.mesh")
//                obBody->createCylinder(staticMesh);
//            else
//            {
                if (staticMesh)
                    obBody->createMeshCollider(ogreEntity->getMesh().getPointer());
                else
                    obBody->createBody(ogreEntity->getMesh().getPointer());
//            }
        }
        else
            obBody->createBodyWithShape(shape, staticMesh);

        // Setup a user pointer for later use within the Bullet manager
        // This step is compulsory to be able to retrieve the entity from the broad-phase algorithm.
        obBody->getBulletBody()->getCollisionShape()->setUserPointer(this);

        obBody->getBulletBody()->getWorldTransform().setOrigin(Utils::btVectorFromOgre(pos));
        obBody->getBulletBody()->getWorldTransform().setRotation(Utils::btQuaternionFromOgre(quat));
    }
    else
    {
        throw EntityAlreadyExistsException(name);
    }
}

obEntityWrapper::obEntityWrapper(const obEntityWrapper &other) throw(EntityAlreadyExistsException) :
    obEntity(),
    ogreEntity(other.ogreEntity),
    ogreNode(other.ogreNode),
    meshName(other.getMeshName()),
    shape(other.shape),
    obBody(0),
    world(0),
    staticMesh(other.hasStaticMesh()),
    animation(other.getAnimation()),
    frameStart(other.getFrameStart())
{
    // Create the rigid body using a copy constructor
    obBody = new obDynamicRigidBody(this, *other.obBody);

    // If no shape is defined, create one from the mesh
    if(!shape)
    {
//            if(meshName == "cube.mesh")
//                obBody->createCube(staticMesh);
//            else if(meshName == "sphere.mesh")
//                obBody->createSphere(staticMesh);
//            else if(meshName == "simplePlane.mesh")
//                obBody->createPlane(staticMesh);
//            else if(meshName == "cylinder.mesh")
//                obBody->createCylinder(staticMesh);
//            else
//            {
                if (staticMesh)
                    obBody->createMeshCollider(ogreEntity->getMesh().getPointer());
                else
                    obBody->createBody(ogreEntity->getMesh().getPointer());
//            }
    }
    else
        obBody->createBodyWithShape(shape, staticMesh);

    // Setup a user pointer for later use within the Bullet manager
    // This step is compulsory to be able to retrieve the entity from the broad-phase algorithm.
    //FIXME: temporary stuff, better to do that in the copy constructor of obBody
    obBody->getBulletBody()->getCollisionShape()->setUserPointer(this);
    obBody->getBulletBody()->setWorldTransform(other.obBody->getBulletBody()->getWorldTransform());
//    obBody->getBulletBody()->setGravity(other.obBody->getBulletBody()->getGravity());
    obBody->getBulletBody()->setAngularVelocity(other.obBody->getBulletBody()->getAngularVelocity());
    obBody->getBulletBody()->setLinearVelocity(other.obBody->getBulletBody()->getLinearVelocity());

//    obBody->getBulletBody()->getWorldTransform().setOrigin(other.obBody->getPosition());
//    obBody->getBulletBody()->getWorldTransform().setRotation(other.obBody->getRotation());
}

obEntityWrapper::~obEntityWrapper()
{
    //FIXME: manage this
//    ogreNode->removeAndDestroyAllChildren();
//    delete ogreNode;
//    delete ogreEntity;

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

void obEntityWrapper::setColor(const float r, const float g, const float b)
{
    // Don't re-set an already set color
    if(r == red && g == green && b == blue)
        return;

    red = r;
    green = g;
    blue = b;


    // Get the prefix of the entity's material, and then add a suffix for this color
    Ogre::String prefix;
    prefix.append(QString(materialName.c_str()).split(":").first().toAscii());

    materialName = prefix;
    materialName.append(QString("color#%1#%2#%3").arg(r).arg(g).arg(b).toAscii());

    // Get (or create) the corresponding material
    Ogre::MaterialPtr mat = OgreResources::createOrRetrieveMaterial(materialName);

    // Set lighting according to the color
    //FIXME: more efficient to just do this if material did NOT exist before (i.e. make a createOrRetrieveColoredFromParent())
    mat->setAmbient(r,g,b);
    mat->setDiffuse(r,g,b,1);
    mat->setSpecular(0,0,0,1);

    // Have the entity use the new material
    ogreEntity->setMaterial(mat);
}

void obEntityWrapper::setRandomColor()
{
    // Random color selection
    float r = (float)((float)rand() / ((float)RAND_MAX + 1));
    float g = (float)((float)rand() / ((float)RAND_MAX + 1));
    float b = (float)((float)rand() / ((float)RAND_MAX + 1));

    setColor(r, g, b);
}
