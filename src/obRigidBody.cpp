/*
 *** Methods createBody, createBodyWithShape and createMeshCollider:
 * Copyright (2000-2009) Torus Knot Software Ltd
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Ogre 3D developers <http://www.ogre3d.org>
 *  Quentin Avril <quentin.avril@irisa.fr>
 *
 * MIT License
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *
 *** Rest of the class
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Quentin Avril <quentin.avril@irisa.fr>
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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include "obRigidBody.h"
#include "utils.h"

#include <QtDebug>
#include "ogreresources.h"

const float obRigidBody::StaticBodyRestitution  = 0.1f;
const float obRigidBody::StaticBodyFriction     = 0.8f;
const float obRigidBody::DynamicBodyRestitution = 0.6f;
const float obRigidBody::DynamicBodyFriction    = 0.6f;

obRigidBody::obRigidBody(const Ogre::String &name, const Ogre::Vector3 &pos, const Ogre::Quaternion &quat, const Ogre::Vector3 &scale, int mass) :
    node(0),
    btBody(0),
	triangleMesh(0),
    btShape(0),
    name(name),
    position(pos),
    scale(scale),
    mass(mass),
    quaternion(quat)
{
}

obRigidBody::~obRigidBody()
{
    delete triangleMesh;
    delete node;

    delete btBody->getMotionState();
    delete btBody;
}

void obRigidBody::createSceneNode()
{
    node = OgreResources::getSceneManager()->getRootSceneNode()->createChildSceneNode(name, position, quaternion);
    node->scale(scale);
}

Ogre::SceneNode* obRigidBody::getSceneNode() const
{
    return node;
}

btRigidBody* obRigidBody::getBulletBody() const
{
    return btBody;
}

btCollisionShape* obRigidBody::getShape() const
{
    return btShape;
}
const btVector3 &obRigidBody::getPosition() const
{
    return btBody->getWorldTransform().getOrigin();
}

void obRigidBody::createCube(const bool staticMesh)
{
    btShape = new btBoxShape(Utils::btVectorFromOgre(scale));
    createBodyWithShape(btShape, staticMesh);
}

void obRigidBody::createSphere(const bool staticMesh)
{
    btShape = new btSphereShape(btScalar(scale.x));
    createBodyWithShape(btShape, staticMesh);
}

void obRigidBody::createCylinder(const bool staticMesh)
{
    btShape = new btCylinderShape(Utils::btVectorFromOgre(scale));
    createBodyWithShape(btShape, staticMesh);
}

void obRigidBody::createPlane(const bool staticMesh)
{
    btShape = new btBoxShape(btVector3(450,0.1,450));
    createBodyWithShape(btShape, staticMesh);
}

void obRigidBody::createBorder(const btVector3 &shapeLen, const bool staticMesh)
{
    btShape = new btBoxShape(shapeLen);
    createBodyWithShape(btShape, staticMesh);
}

void obRigidBody::createBody(Ogre::Mesh *ptr)
{
    btCompoundShape* cShape = new btCompoundShape();

    unsigned short subCount = ptr->getNumSubMeshes();
    for(unsigned short i=0; i<subCount; i++)
    {
        Ogre::SubMesh *pSubMesh = ptr->getSubMesh(i);

        Ogre::uint16 *pVIndices16 = NULL;
        Ogre::uint32 *pVIndices32 = NULL;

        Ogre::IndexData *indexData = pSubMesh->indexData;
        Ogre::HardwareIndexBufferSharedPtr buffIndex = indexData->indexBuffer;

        bool use32bit = false;
        if (buffIndex->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
        {
            pVIndices32 = static_cast<Ogre::uint32*>(buffIndex->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
            use32bit = true;
        }
        else
        {
            pVIndices16 = static_cast<Ogre::uint16*>(buffIndex->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        }
        buffIndex->unlock();
        Ogre::VertexData *usedVertexData;
        usedVertexData = pSubMesh->vertexData;

        Ogre::VertexDeclaration *vDecl = usedVertexData->vertexDeclaration;
        Ogre::VertexBufferBinding *vBind = usedVertexData->vertexBufferBinding;

        const Ogre::VertexElement *elemVPos = vDecl->findElementBySemantic(Ogre::VES_POSITION);
        Ogre::HardwareVertexBufferSharedPtr srcBuf = vBind->getBuffer(elemVPos->getSource());

        unsigned char *pSrcBase = static_cast<unsigned char*>(srcBuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY)); //HBL_NORMAL

        size_t numFaces = indexData->indexCount / 3;

        btVector3 vertexPos[3];
        Ogre::uint32 vertInd[3];
        float *pVPos;

        // Create the convex hull
        btConvexHullShape* hull = new btConvexHullShape();
        btTransform trans;
        for (size_t n = 0; n < numFaces; ++n)
        {
            int i;
            for (i = 0; i < 3; ++i)
            {
                // Get indexes of vertices that form a polygon in the position buffer
                if (use32bit)
                {
                    vertInd[i] = *pVIndices32++;//    dynamicsWorld->removeRigidBody(groundRigidBody);
                    //    delete groundRigidBody->getMotionState();
                    //    delete groundRigidBody;
                }
                else
                {
                    vertInd[i] = *pVIndices16++;
                }
                // Get the vertices positions from the position buffer
                unsigned char* vBase = pSrcBase + (srcBuf->getVertexSize() * vertInd[i]);
                elemVPos->baseVertexPointerToElement(vBase, &pVPos);
                vertexPos[i][0] = pVPos[0];
                vertexPos[i][1] = pVPos[1];
                vertexPos[i][2] = pVPos[2];
            }
            hull->addPoint(vertexPos[0]);
            hull->addPoint(vertexPos[1]);
            hull->addPoint(vertexPos[2]);
        }
        srcBuf->unlock();
        cShape->addChildShape(trans.getIdentity(),hull);
    }

    btShape = cShape;
    btVector3 localInertiaTensor = btVector3(0,0,0);
    btShape->calculateLocalInertia(mass,localInertiaTensor);
    btShape->setLocalScaling(Utils::btVectorFromOgre(scale));

	btBody = new btRigidBody(mass, _createMotionState(), btShape, localInertiaTensor);
    btBody->setRestitution(DynamicBodyRestitution);
    btBody->setFriction(DynamicBodyFriction);
    btBody->setCollisionFlags(btBody->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
}

void obRigidBody::createBodyWithShape(btCollisionShape *shape, const bool staticMesh)
{
    btShape = shape;

    btVector3 localInertiaTensor = btVector3(0,0,0);
    if(!staticMesh)
    {
        btShape->calculateLocalInertia(mass,localInertiaTensor);
    }

	btBody = new btRigidBody(staticMesh ? 0:mass, _createMotionState(), btShape, localInertiaTensor);
    btBody->setRestitution(staticMesh ? StaticBodyRestitution:DynamicBodyRestitution);
    btBody->setFriction(staticMesh ? StaticBodyFriction:DynamicBodyFriction);
}


void obRigidBody::createMeshCollider(Ogre::Mesh *ptr)
{
    btTriangleMesh* triMesh = new btTriangleMesh();

    unsigned short subCount = ptr->getNumSubMeshes();
    for (unsigned short i=0; i<subCount; i++)
    {
        // ripped from OgreMesh.cpp
        Ogre::SubMesh *pSubMesh = ptr->getSubMesh(i);

        Ogre::uint16	*pVIndices16 = NULL;
        Ogre::uint32	*pVIndices32 = NULL;

        Ogre::IndexData *indexData = pSubMesh->indexData;
        Ogre::HardwareIndexBufferSharedPtr buffIndex = indexData->indexBuffer;

        bool use32bit = false;
        if (buffIndex->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
        {
            pVIndices32 = static_cast<Ogre::uint32*>(buffIndex->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
            use32bit = true;
        }
        else
        {
            pVIndices16 = static_cast<Ogre::uint16*>(buffIndex->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        }
        buffIndex->unlock();
        Ogre::VertexData *usedVertexData;
        usedVertexData = pSubMesh->vertexData;

        Ogre::VertexDeclaration *vDecl = usedVertexData->vertexDeclaration;
        Ogre::VertexBufferBinding *vBind = usedVertexData->vertexBufferBinding;

        const Ogre::VertexElement *elemVPos = vDecl->findElementBySemantic(Ogre::VES_POSITION);
        Ogre::HardwareVertexBufferSharedPtr srcBuf = vBind->getBuffer(elemVPos->getSource());

        unsigned char *pSrcBase = static_cast<unsigned char*>(srcBuf->lock(Ogre::HardwareBuffer::HBL_NORMAL));

        size_t numFaces = indexData->indexCount / 3;

        btVector3 vertexPos[3];
        Ogre::uint32 vertInd[3];
        float *pVPos;

        for (size_t n = 0; n < numFaces; ++n)
        {
            int i;
            for (i = 0; i < 3; ++i)
            {
                // get indexes of vertices that form a polygon in the position buffer
                if (use32bit)
                {
                    vertInd[i] = *pVIndices32++;
                }
                else
                {
                    vertInd[i] = *pVIndices16++;
                }
                // get the vertices positions from the position buffer
                unsigned char* vBase = pSrcBase + (srcBuf->getVertexSize() * vertInd[i]);
                elemVPos->baseVertexPointerToElement(vBase, &pVPos);
                vertexPos[i][0] = pVPos[0];
                vertexPos[i][1] = pVPos[1];
                vertexPos[i][2] = pVPos[2];
            }
            triMesh->addTriangle(vertexPos[0], vertexPos[1], vertexPos[2]);
        }
        srcBuf->unlock();
    }
    btShape = new btBvhTriangleMeshShape(triMesh,true);

	btBody = new btRigidBody(0.0, _createMotionState(), btShape);
    btBody->setRestitution(StaticBodyRestitution);
    btBody->setFriction(StaticBodyFriction);
}

void obRigidBody::setTransformation(const Ogre::Vector3 &rotation, const Ogre::Vector3 &direction)
{
    // Computing of the quaternion representing the motion
    Ogre::Radian yRad, pRad, rRad, yDeg, pDeg, rDeg;
    Ogre::Quaternion quat = Ogre::Quaternion::IDENTITY;
    Ogre::Matrix3 mat;
    quat.ToRotationMatrix(mat);
    mat.ToEulerAnglesYXZ(yRad, pRad, rRad);
    yDeg = yRad;
    pDeg = pRad;
    rDeg = rRad;

    yDeg +=Ogre::Degree(rotation.y); //Valeur de la rotation sur l'axe Y
    pDeg +=Ogre::Degree(rotation.x); //Valeur de la rotation sur l'axe X
    rDeg +=Ogre::Degree(rotation.z); //Valeur de la rotation sur l'axe Z

    // Apply the modifications to the rotation matrix
    mat.FromEulerAnglesYXZ(yDeg, pDeg, rDeg);
    // Forward this transformation information to the quaternion
    quat.FromRotationMatrix(mat);

    // Retrieve the btTransform and current quaternion of the Bullet node containing the entity
    btTransform transf = getBulletBody()->getWorldTransform();
    btQuaternion currentQuat = transf.getRotation();

    // Apply the angle modification 'current quat * quat' to the physics engine
    getBulletBody()->setWorldTransform(btTransform(currentQuat*Utils::btQuaternionFromOgre(quat), getPosition()));

    // Apply the rotation to the Ogre scene node
    if(node)
        node->rotate(quat);

    // Apply the direction vector to the btRigidBody and the scene node
    getBulletBody()->translate(Utils::btVectorFromOgre(direction));
    if(node)
        node->translate(direction);
}

btMotionState *obRigidBody::_createMotionState()
{
    return new btDefaultMotionState(btTransform(Utils::btQuaternionFromOgre(quaternion), Utils::btVectorFromOgre(position)));
}
