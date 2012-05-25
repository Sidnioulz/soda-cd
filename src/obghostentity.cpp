#include "obghostentity.h"

#include <OgreString.h>
#include "utils.h"
#include "ogreresources.h"

obGhostEntity::obGhostEntity(const Ogre::String &name, const bool &visible, const Ogre::String &meshName, const Ogre::Vector3 &pos, const Ogre::Quaternion &quat, const Ogre::Vector3 &scale, const bool &randomColor)  throw(EntityAlreadyExistsException) :
    position(pos),
    quaternion(quat),
    materialName(),
    randomColor(randomColor),
    ogreEntity(0),
    meshName(meshName),
    ogreSceneManager(0),
    node(0)
{
    // If the scene manager doesn't already have an entity with this name
    if (!OgreResources::getSceneManager()->hasEntity(name))
    {
        // Create the rigid body and its Ogre entity
        node = OgreResources::getSceneManager()->getRootSceneNode()->createChildSceneNode(name, position, quaternion);
        node->scale(scale);

        ogreEntity = OgreResources::getSceneManager()->createEntity(name, meshName);
        ogreEntity->setCastShadows(true);
        ogreEntity->setVisible(visible);
        node->attachObject(ogreEntity);
    }
    else
    {
        throw EntityAlreadyExistsException(name);
    }

    // Assign a random color to the object if not static
    if(randomColor)
        setRandomColor();
}

obGhostEntity::~obGhostEntity()
{
     delete node;
}

void obGhostEntity::setColor(const float r, const float g, const float b, const bool randomColorFlag)
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

void obGhostEntity::setRandomColor()
{
    // Random color selection
    float r = (float)((float)rand() / ((float)RAND_MAX + 1));
    float g = (float)((float)rand() / ((float)RAND_MAX + 1));
    float b = (float)((float)rand() / ((float)RAND_MAX + 1));

    setColor(r, g, b, true);
}
