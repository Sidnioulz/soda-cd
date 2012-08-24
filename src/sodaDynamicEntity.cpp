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
#include "sodaDynamicEntity.h"
#include "sodaLogicWorld.h"
#include "Parser/Animation.h"
#include "Parser/Action.h"
#include "sodaUtils.h"
#include "main.h"
#include "sodaOgreResources.h"

unsigned int sodaDynamicEntity::nextInLine = 0;

namespace {
    //! Deletes an Ogre::Entity properly
    void deleteOgreEntity(Ogre::Entity *ent)
    {
        QMetaObject::invokeMethod(MainPepsiWindow::getInstance()->getOgreWidget(), "onEntityDeletion",
                                  Q_ARG(Ogre::Entity *, ent));
    }

    //! Deletes an Ogre::Entity properly
    void deleteSceneNode(Ogre::SceneNode *node)
    {
        QMetaObject::invokeMethod(MainPepsiWindow::getInstance()->getOgreWidget(), "onSceneNodeDeletion",
                                  Q_ARG(Ogre::SceneNode *, node));
    }
}








sodaDynamicEntity::sodaDynamicEntity(const Ogre::String &name, const Ogre::String &meshName, const Ogre::Vector3 &pos, const Ogre::Quaternion &quat, const bool staticMesh, const Ogre::Vector3 &scale, const int mass, btCollisionShape *shape)  throw(EntityAlreadyExistsException) :
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
    if (!sodaOgreResources::getSceneManager()->hasEntity(name))
    {
        // Create the rigid body
        obBody = new sodaDynamicRigidBody(this,
                                        sodaUtils::btVectorFromOgre(pos),
                                        sodaUtils::btQuaternionFromOgre(quat),
                                        sodaUtils::btVectorFromOgre(scale),
                                        mass);

        ogreEntity = QSharedPointer<Ogre::Entity>(sodaOgreResources::getSceneManager()->createEntity(name, meshName), deleteOgreEntity);
        ogreEntity->setCastShadows(true);

        ogreNode = QSharedPointer<Ogre::SceneNode>(sodaOgreResources::getSceneManager()->getRootSceneNode()->createChildSceneNode(name, pos, quat), deleteSceneNode);
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

        obBody->getBulletBody()->getWorldTransform().setOrigin(sodaUtils::btVectorFromOgre(pos));
        obBody->getBulletBody()->getWorldTransform().setRotation(sodaUtils::btQuaternionFromOgre(quat));
    }
    else
    {
        throw EntityAlreadyExistsException(name);
    }
}

sodaDynamicEntity::sodaDynamicEntity(const sodaDynamicEntity &other) throw(EntityAlreadyExistsException) :
    sodaEntity(),
    ogreEntity(other.ogreEntity),
    ogreNode(other.ogreNode),
    meshName(other.getMeshName()),
    materialName(other.getMaterialName()),
    shape(other.shape),
    obBody(0),
    world(0),
    staticMesh(other.hasStaticMesh()),
    animation(other.getAnimation()),
    frameStart(other.getFrameStart())
{
    // Create the rigid body using a copy constructor
    obBody = new sodaDynamicRigidBody(this, *other.obBody);
}

sodaDynamicEntity::~sodaDynamicEntity()
{
    delete obBody;
	delete animation;
}

sodaDynamicEntity* sodaDynamicEntity::clone() const throw(EntityAlreadyExistsException)
{
   return new sodaDynamicEntity(ogreEntity->getName()+Ogre::StringConverter::toString(nextInLine++),
                              meshName,
                              Ogre::Vector3::ZERO,
                              Ogre::Quaternion::IDENTITY,
                              false);
}

short sodaDynamicEntity::getOwnerId() const
{
	if(world)
		return world->getId();
	else
        return sodaLogicWorld::NullWorldId;
}

sodaLogicWorld* sodaDynamicEntity::setOwnerWorld(sodaLogicWorld *newWorld)
{
    sodaLogicWorld *oldWorld=world;
    world = newWorld;

    return oldWorld;
}

sodaLogicWorld* sodaDynamicEntity::unsetOwnerWorld()
{
    sodaLogicWorld *oldWorld=world;
    world = NULL;

    return oldWorld;
}

Animation* sodaDynamicEntity::getAnimation() const
{
	qDebug("TODO");
    if(isStatic())
        return animation;
    else
        return NULL;
}

bool sodaDynamicEntity::hasAnimation() const
{
	qDebug("TODO");
    return animation!=NULL;
}

void sodaDynamicEntity::setAnimation(Animation* anim)
{
	qDebug("TODO");
    animation = anim;
}

void sodaDynamicEntity::setFrameStart(const int newFrameStart)
{
	qDebug("TODO");
	frameStart = newFrameStart;
}

int sodaDynamicEntity::getFrameStart() const
{
	qDebug("TODO");
	return frameStart;
}

void sodaDynamicEntity::setColor(const float r, const float g, const float b)
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

    materialName.clear();
    materialName.append(prefix);
    materialName.append(QString(":color#%1#%2#%3").arg(r).arg(g).arg(b).toAscii());

    // Get (or create) the corresponding material
    Ogre::MaterialPtr mat = sodaOgreResources::createMaterialFromParent(materialName, prefix);

    // Set lighting according to the color
    //FIXME: more efficient to just do this if material did NOT exist before (i.e. make a createOrRetrieveColoredFromParent())
    mat->setAmbient(r,g,b);
    mat->setDiffuse(r,g,b,1);
    mat->setSpecular(r,g,b,1);

    // Have the entity use the new material
    ogreEntity->setMaterial(mat);
}

void sodaDynamicEntity::setRandomColor()
{
    // Random color selection
    float r = (float)((float)rand() / ((float)RAND_MAX + 1));
    float g = (float)((float)rand() / ((float)RAND_MAX + 1));
    float b = (float)((float)rand() / ((float)RAND_MAX + 1));

    setColor(r, g, b);
}
