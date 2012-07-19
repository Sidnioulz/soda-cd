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
#include "robustnessevalsimulation.h"

#include <limits>
#include "physicsworld.h"
#include "utils.h"
#include "ogreresources.h"

RobustnessEvalSimulation::RobustnessEvalSimulation(const btScalar &targetTimeStep, const int &numEntities) :
    Simulation(targetTimeStep, 1, btVector3(10000,10000,10000), numEntities)
{
}


RobustnessEvalSimulation::~RobustnessEvalSimulation()
{
}

void RobustnessEvalSimulation::setupBasic3DEnvironment()
{
    Ogre::Entity *ent;
    Ogre::Plane p;
    p.normal = Ogre::Vector3(0,1,0); p.d = 0;
    Ogre::MeshManager::getSingleton().createPlane("sceneFloor",
                                                  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                  p, sceneSize.x(), sceneSize.z(), 64, 64, true, 1, 1, 1, Ogre::Vector3::UNIT_Z);
//                                                  p, sceneSize.x()*8, sceneSize.z()*8, 32, 32, true, 1, 16, 16, Ogre::Vector3::UNIT_Z);
    ent = OgreResources::getSceneManager()->createEntity("sceneFloor", "sceneFloor");
    ent->setMaterialName("Surfaces/Lava"); //Surfaces/Lava
    Ogre::SceneNode *node = OgreResources::getSceneManager()->getRootSceneNode()->createChildSceneNode();
    node->attachObject(ent);
    node->setPosition(0, 0, 0);

    // Setup some sky
    Ogre::Plane plane;
    plane.d = 1000; plane.normal = Ogre::Vector3::NEGATIVE_UNIT_Y;
    OgreResources::getSceneManager()->setSkyPlane(true, plane, "Examples/EveningSkyBox", 1500, 10, true, 1.5f, 150, 150);

    // Setup some fog
    Ogre::ColourValue fadeColour(0.5, 0.3, 0.3);
    OgreResources::getSceneManager()->setFog(Ogre::FOG_EXP2, fadeColour, 0.0001f, 0, 0);

    // Add some light
    OgreResources::getSceneManager()->setAmbientLight(Ogre::ColourValue(1, 0.8, 0.8, 1));
}

void RobustnessEvalSimulation::setupBasicPhysicsEnvironment(PhysicsWorld *world)
{
    world->createScene();
}

void RobustnessEvalSimulation::tickCallback(PhysicsWorld *world, const btScalar &timeStep)
{
    //static float addRatio = 0.25f;
    static bool init = false;

    QVector<obEntityWrapper*> &ents = world->getEntities(0);

    for(int i=0; i<ents.size(); ++i)
        ents[i]->getRigidBody()->getBulletBody()->setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(0,0,0)));

    if(world->getCurrentTime() >= 1 && !init)
    {
        for(int i=0; i<2; ++i) //######################################## here
            world->addEntity(_createNinja(btVector3(0,0,0), btVector3(10, 10, 10), 50), world->getCurrentTime());
        init = true;
    }
}

void RobustnessEvalSimulation::loadEntities()
{
    if(!entitiesWithAssignments.isEmpty())
        qWarning() << "The list of entities is not empty, loaded entities will be appended.";

    // Reserve memory for all simulated entities (performance)
    entitiesWithAssignments.reserve(numEntities);

    // Reference srand for simulations noted as interesting

    for(int i=0; i<numEntities; ++i)
    {
        appendEntity(_createNinja(btVector3(0, 0, 0), btVector3(10, 10, 10), 50));
    }
}


obEntityWrapper *RobustnessEvalSimulation::_createNinja(const btVector3 &position, const btVector3 &scale, const btScalar &mass)
{
    Ogre::String entityName = "ninja" + Ogre::StringConverter::toString(++entityIdCounter);

    Ogre::Vector3 ogrePos = Utils::vectorFromBullet(position);
    Ogre::Vector3 ogreScale = Utils::vectorFromBullet(scale);

    static obEntityWrapper *modelNinja = new obEntityWrapper("MODELNINJA!", "cube.mesh", Ogre::Vector3(1000, 1000, 1000), Ogre::Quaternion::IDENTITY, false, ogreScale, mass);
    static btCollisionShape *ninjaShape = modelNinja->getRigidBody()->getBulletBody()->getCollisionShape();



    return new obEntityWrapper(entityName, "cube.mesh", ogrePos, Ogre::Quaternion::IDENTITY, false, ogreScale, mass, ninjaShape);
}
