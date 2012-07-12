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

#include <QtDebug>
#include <btBulletDynamicsCommon.h>
#include <bullet/LinearMath/btSerializer.h>
#include "ogreresources.h"

const float obRigidBody::StaticBodyRestitution  = 0.1f;
const float obRigidBody::StaticBodyFriction     = 0.8f;
const float obRigidBody::DynamicBodyRestitution = 0.6f;
const float obRigidBody::DynamicBodyFriction    = 0.6f;

obRigidBody::obRigidBody(obEntity *parent, const btVector3 &pos, const btQuaternion &quat, const btVector3 &scale, const int mass) :
    parent(parent),
    btBody(0),
	triangleMesh(0),
    btShape(0),
    position(pos),
    scale(scale),
    mass(mass),
    quaternion(quat)
{
}

obRigidBody::obRigidBody(obEntity *parent, const obRigidBody &other) :
    parent(parent),
    btBody(0),
    triangleMesh(0),
    btShape(0),
    position(other.position),
    scale(other.scale),
    mass(other.mass),
    quaternion(other.quaternion)
{
    //TODO: later, shared btShapes with some nice pointers
//    btShape = other.btShape;

//    00368         btCollisionObject::serialize(&rbd->m_collisionObjectData, serializer);
//    00369
//    00370         m_invInertiaTensorWorld.serialize(rbd->m_invInertiaTensorWorld);
//    00371         m_linearVelocity.serialize(rbd->m_linearVelocity);
//    00372         m_angularVelocity.serialize(rbd->m_angularVelocity);
//    00373         rbd->m_inverseMass = m_inverseMass;
//    00374         m_angularFactor.serialize(rbd->m_angularFactor);
//    00375         m_linearFactor.serialize(rbd->m_linearFactor);
//    00376         m_gravity.serialize(rbd->m_gravity);
//    00377         m_gravity_acceleration.serialize(rbd->m_gravity_acceleration);
//    00378         m_invInertiaLocal.serialize(rbd->m_invInertiaLocal);
//    00379         m_totalForce.serialize(rbd->m_totalForce);
//    00380         m_totalTorque.serialize(rbd->m_totalTorque);
//    00381         rbd->m_linearDamping = m_linearDamping;
//    00382         rbd->m_angularDamping = m_angularDamping;
//    00383         rbd->m_additionalDamping = m_additionalDamping;
//    00384         rbd->m_additionalDampingFactor = m_additionalDampingFactor;
//    00385         rbd->m_additionalLinearDampingThresholdSqr = m_additionalLinearDampingThresholdSqr;
//    00386         rbd->m_additionalAngularDampingThresholdSqr = m_additionalAngularDampingThresholdSqr;
//    00387         rbd->m_additionalAngularDampingFactor = m_additionalAngularDampingFactor;
//    00388         rbd->m_linearSleepingThreshold=m_linearSleepingThreshold;
//    00389         rbd->m_angularSleepingThreshold = m_angularSleepingThreshold;
}

obRigidBody::~obRigidBody()
{
    // Check that the shape isn't compound (it is if created in createBody). If it is, delete child shapes
    btCompoundShape *cShape = dynamic_cast<btCompoundShape *>(btShape);
    if(cShape)
    {
        for(int i=0; i<cShape->getNumChildShapes(); ++i)
            delete cShape->getChildShape(i);
    }

    // Delete the shape either way
    delete btShape;

    // Delete any triangle mesh that was used for the shape
    if(triangleMesh)
        delete triangleMesh;

    // Delete the motion state
    delete btBody->getMotionState();

    // Delete the rigid body
    delete btBody;
}

btCollisionShape* obRigidBody::getShape() const
{
    return btShape;
}

const btVector3 &obRigidBody::getPosition() const
{
    return btBody->getWorldTransform().getOrigin();
}

const btQuaternion obRigidBody::getRotation() const
{
    return btBody->getWorldTransform().getRotation();
}

void obRigidBody::createCube(const bool staticMesh)
{
    btShape = new btBoxShape(scale);
    createBodyWithShape(btShape, staticMesh);
}

void obRigidBody::createSphere(const bool staticMesh)
{
    btShape = new btSphereShape(btScalar(scale.x()));
    createBodyWithShape(btShape, staticMesh);
}

void obRigidBody::createCylinder(const bool staticMesh)
{
    btShape = new btCylinderShape(scale);
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
    btShape->setLocalScaling(scale);

	btBody = new btRigidBody(mass, _createMotionState(), btShape, localInertiaTensor);
    btBody->setRestitution(DynamicBodyRestitution);
    btBody->setFriction(DynamicBodyFriction);
    btBody->setUserPointer(parent);
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
    btBody->setUserPointer(parent);
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

    triangleMesh = triMesh;
    btShape = new btBvhTriangleMeshShape(triMesh, true);

	btBody = new btRigidBody(0.0, _createMotionState(), btShape);
    btBody->setRestitution(StaticBodyRestitution);
    btBody->setFriction(StaticBodyFriction);
    btBody->setUserPointer(parent);
}

btMotionState *obRigidBody::_createMotionState()
{
    return new btDefaultMotionState(btTransform(quaternion, position));
}
