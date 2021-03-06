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
#include "randomcubesimulation.h"

#include <limits>
#include "sodaLogicWorld.h"
#include "sodaUtils.h"
#include "sodaLocalGrid.h"
#include "sodaOgreResources.h"

RandomCubeSimulation::RandomCubeSimulation(const btScalar &targetTimeStep, const int &declNumWorlds, const btVector3 &sceneSize, const int &numEntities, const bool &randomScale) :
    Simulation(targetTimeStep, declNumWorlds, sceneSize, numEntities),
    randomScale(randomScale)
{
}

RandomCubeSimulation::~RandomCubeSimulation()
{
}

void RandomCubeSimulation::setupBasic3DEnvironment()
{
    Ogre::Entity *ent;
    Ogre::Plane p;
    p.normal = Ogre::Vector3(0,1,0); p.d = 0;
    Ogre::MeshManager::getSingleton().createPlane("sceneFloor",
                                                  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                  p, sceneSize.x(), sceneSize.z(), 1, 1, true, 1, 1, 1, Ogre::Vector3::UNIT_Z);
//                                                  p, sceneSize.x()*8, sceneSize.z()*8, 32, 32, true, 1, 16, 16, Ogre::Vector3::UNIT_Z);
    ent = sodaOgreResources::getSceneManager()->createEntity("sceneFloor", "sceneFloor");
    ent->setMaterialName("Examples/GrassFloor"); //Surfaces/Lava
    Ogre::SceneNode *node = sodaOgreResources::getSceneManager()->getRootSceneNode()->createChildSceneNode();
    node->attachObject(ent);
    node->setPosition(0, 0, 0);
    node->showBoundingBox(true);

    // Setup some sky
    Ogre::Plane plane;
    plane.d = 1000; plane.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;
    sodaOgreResources::getSceneManager()->setSkyPlane(true, plane, "Examples/CloudySky", 1500, 10, true, 1.5f, 150, 150);

    // Setup some fog
//    Ogre::ColourValue fadeColour(0.75, 0.75, 0.8);
   // sodaOgreResources::getSceneManager()->setFog(Ogre::FOG_EXP2, fadeColour, 0.0001f, 0, 0);

    // Add some light
    sodaOgreResources::getSceneManager()->setAmbientLight(Ogre::ColourValue(1, 1, 1, 1));
}

void RandomCubeSimulation::setupBasicPhysicsEnvironment(sodaLogicWorld *world)
{
    world->createScene();
}

void RandomCubeSimulation::loadEntities()
{
    if(!entitiesWithAssignments.isEmpty())
        qWarning() << "The list of entities is not empty, loaded entities will be appended.";

    // Reserve memory for all simulated entities (performance)
    entitiesWithAssignments.reserve(numEntities);

    int x,y,z;
    btScalar scale;
    btVector3 defaultBoxSize(102, 102, 102); // hard-linked

    // Reference srand for simulations noted as interesting
    srand(1600000);
    //srand(time(NULL));

    for(int i=0; i<numEntities; ++i)
    {
        // Get random coordinates and scale
        scale= randomScale? ((rand() % 11 + 1.9) * 0.1) : 1;
        x=qMax(-sceneSize.x()/2 + defaultBoxSize.x()*scale, qMin(sceneSize.x()/2 - defaultBoxSize.x()*scale -1, rand() % (int)(sceneSize.x()) - sceneSize.x()/2));
        y=qMax((int)(defaultBoxSize.x()*scale), qMin((int)(sceneSize.y() - defaultBoxSize.x()*scale -1), rand() % (int)(sceneSize.y())));
        z=qMax(-sceneSize.z()/2 + defaultBoxSize.z()*scale, qMin(sceneSize.z()/2 - defaultBoxSize.z()*scale -1, rand() % (int)(sceneSize.z()) - sceneSize.z()/2));

        // Create a new entity
        sodaDynamicEntity *obEnt = _createBox(btVector3(x, y, z), btVector3(scale, scale, scale), 10 + rand()%20);

        // Store it in the map that EKMeans will use
        appendEntity(obEnt);
    }
}

sodaDynamicEntity *RandomCubeSimulation::_createBox(const btVector3 &position, const btVector3 &scale, const btScalar &mass)
{
    Ogre::String entityName = "box" + Ogre::StringConverter::toString(++entityIdCounter);

    Ogre::Vector3 ogrePos = sodaUtils::vectorFromBullet(position);
    Ogre::Vector3 ogreScale = sodaUtils::vectorFromBullet(scale);

    sodaDynamicEntity *obEnt = new sodaDynamicEntity(entityName, "cube.mesh", ogrePos, Ogre::Quaternion::IDENTITY, false, ogreScale, mass);
//    sodaDynamicEntity *obEnt = new sodaDynamicEntity(entityName, "penguin.mesh", ogrePos, Ogre::Quaternion::IDENTITY, false, ogreScale, mass);
//    sodaDynamicEntity *obEnt = new sodaDynamicEntity(entityName, "sphere.mesh", ogrePos, Ogre::Quaternion::IDENTITY, false, ogreScale, mass);
//    sodaDynamicEntity *obEnt = new sodaDynamicEntity(entityName, "ogrehead.mesh", ogrePos, Ogre::Quaternion::IDENTITY, false, ogreScale, mass);
    obEnt->setMaterialName("Surfaces/ColoredCube");
//    obEnt->getRigidBody()->setTransformation(Ogre::Vector3(0,4,0), Ogre::Vector3(0, -1000, 0));

    return obEnt;
}

sodaDynamicEntity *RandomCubeSimulation::_createNinja(const btVector3 &position, const btVector3 &scale, const btScalar &mass)
{
    Ogre::String entityName = "ninja" + Ogre::StringConverter::toString(++entityIdCounter);

    Ogre::Vector3 ogrePos = sodaUtils::vectorFromBullet(position);
    Ogre::Vector3 ogreScale = sodaUtils::vectorFromBullet(scale);

    sodaDynamicEntity *myNinja = new sodaDynamicEntity(entityName, "ninja.mesh", ogrePos, Ogre::Quaternion::IDENTITY, true, ogreScale);

    return myNinja;
}
