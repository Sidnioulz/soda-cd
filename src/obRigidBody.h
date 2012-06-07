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
#ifndef OGRERIGIDOBJECT_H
#define OGRERIGIDOBJECT_H

#include <Ogre.h>
#include <OgreVector3.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include "obentity.h"

/*! \class obRigidBody
  * \brief A dual Ogre-Bullet rigid body
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements rigid bodies that are both Ogre::SceneNode and btRigidBody.
  * It allows synchronous manipulation of rigid body representations in Ogre and Bullet.
  */
class obRigidBody
{
public:
    /*!
      * \brief Default constructor.
      * \param parent the parent entity of this rigid body
      * \param name unique name of the body in the world
      * \param pos the initial position of the body
      * \param quat the initial rotation of the body
      * \param scale the scale of the body with regard to the mesh default size
      * \param mass the mass of the body
      * \return a new obRigidBody
      *
      * This constructor constructs an empty obRigidBody, without btRigidBody and
      * without Ogre::SceneNode. Both objects must be created later using methods
      * of the class.
      */
    obRigidBody(obEntity *parent,
                const Ogre::String &name,
                const Ogre::Vector3 &pos,
                const Ogre::Quaternion &quat,
                const Ogre::Vector3 &scale = Ogre::Vector3(1,1,1),
				const int mass = 0);

    /*!
      * \brief Default destructor.
      */
    virtual ~obRigidBody();

    /*!
      * \brief Creates the Ogre scene node of the rigid body.
      */
    void createSceneNode();

    /*!
      * \brief Gets the Ogre scene node of the rigid body.
      * \return the SceneNode of the body
      */
    Ogre::SceneNode* getSceneNode() const;

    /*!
      * \brief Gets the Bullet version of the rigid body.
      * \return the btRigidBody of the body
      */
    btRigidBody* getBulletBody() const;

    /*!
      * \brief Gets the Bullet collision shape of the body.
      * \return the btCollisionShape of the body
      */
    btCollisionShape* getShape() const;

    /*!
      * \brief Gets the position of the rigid body.
      * \return the position of the body
      */
    const btVector3 &getPosition() const;

    /*!
      * \brief Creates a rigid body with a cube shape.
      * \param staticMesh whether the object shall be static
      */
    void createCube(const bool staticMesh);

    /*!
      * \brief Creates a rigid body with a sphere shape.
      * \param staticMesh whether the object shall be static
      */
    void createSphere(const bool staticMesh);

    /*!
      * \brief Creates a rigid body with a plan shape.
      * \param staticMesh whether the object shall be static
      *
      * \todo Rewrite this function
      * \deprecated This function does not create an actual plan and must be
      * entirely rewritten using SimulationWorld's floor initialization as
      * a base.
      */
    void createPlane(const bool staticMesh);

    /*!
      * \brief Creates a rigid body with a plan shape and a fixed length.
      * \param shapeLen the dimensions of the shape to create
      * \param staticMesh whether the object shall be static
      */
    void createBorder(const btVector3 &shapeLen, const bool staticMesh);

    /*!
      * \brief Creates a rigid body with a cylinder shape.
      * \param staticMesh whether the object shall be static
      */
    void createCylinder(const bool staticMesh);

    /*!
      * \brief Creates a rigid body with a shape extracted from a mesh.
      * \param ptr pointer to the Ogre mesh to use
      */
    void createBody(Ogre::Mesh *ptr);

    /*!
      * \brief Creates a rigid body with an arbitrary shape.
      * \param shape bullet representation of the shape to use
      * \param staticMesh whether the object shall be static
      */
    void createBodyWithShape(btCollisionShape* shape, const bool staticMesh);

    /*!
      * \brief Creates a body using a mesh collider
      * \param ptr pointer to the mesh to use
      *
      * \warning Not sure how this function work or what it does. Use at your own risk.
      */
    void createMeshCollider(Ogre::Mesh *ptr);

    /*!
      * \brief Sets the rotation and motion of a rigid body to be those in parameter
      * \param rotation the rotation to apply as a (x,y,z) vector
      * \param direction the motion to apply as a (x,y,z) vector
      */
    void setTransformation(const Ogre::Vector3 &rotation, const Ogre::Vector3 &direction);

    /*!
      * \brief Returns the parent obEntity of this rigid body
      * \return the parent obEntity of this object
      */
    inline obEntity *getParent() const
    {
        return parent;
    }

	/*! \brief Gets the mass of the body.
	  * \return the mass of the body
	  */
	inline int getMass() const
	{
		return mass;
	}

    //FIXME reimplement with Quentin, using TestBulletOgre's code base.
    //TODO: void setTransformation(Ogre::Real wQuat, Ogre::Real xQuat, Ogre::Real yQuat, Ogre::Real zQuat, Ogre::Real xTrans, Ogre::Real yTrans, Ogre::Real zTrans);

private:
	/*!
	  * \brief Creates a motion state for the Bullet rigid body.
	  * \return a pointer to a new obMotionState for this obRigidBody
	  */
	virtual btMotionState *_createMotionState();

    // Constant values for body friction and restitution
    const static float StaticBodyRestitution;   /*!< Static body restitution (see Bullet documentation) */
    const static float StaticBodyFriction;      /*!< Static body friction (see Bullet documentation) */
    const static float DynamicBodyRestitution;  /*!< Dynamic body restitution (see Bullet documentation) */
    const static float DynamicBodyFriction;     /*!< Dynamic body friction (see Bullet documentation) */

    obEntity            *parent;        /**< Parent entity of this rigid body */
    Ogre::SceneNode     *node;          /**< Ogre scene node of the body */
    btRigidBody         *btBody;        /**< Bullet rigid body */
    btTriangleMesh      *triangleMesh;  /**< Bullet polygonal mesh */
    btCollisionShape    *btShape;       /**< Bullet shape for collisions */
    Ogre::String        name;           /**< Name of the rigid body */
    Ogre::Vector3       position;       /**< Position of the object at creation time */
    Ogre::Vector3       scale;          /**< Object scale compared to original mesh */
	int                 mass;           /**< Mass of the object */
    Ogre::Quaternion    quaternion;     /**< Object rotation at creation time */
};

#endif // OGRERIGIDBODY_H
