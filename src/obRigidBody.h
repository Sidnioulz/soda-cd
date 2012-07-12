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
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include "obentity.h"

/*! \class obRigidBody
  * \brief A Bullet rigid body
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an overlay to btRigidBody, that can be used to create a btRigidBody with various methods.
  *
  * \note For historical reasons, this class is named obRigidBody. It could be named sodaRigidBody.
  */
class obRigidBody
{
public:
    /*!
      * \brief Default constructor.
      * \param parent the parent entity of this rigid body
      * \param pos the initial position of the body
      * \param quat the initial rotation of the body
      * \param scale the scale of the body with regard to the mesh default size
      * \param mass the mass of the body
      * \return a new obRigidBody
      */
    obRigidBody(obEntity *parent,
                const btVector3 &pos,
                const btQuaternion &quat,
                const btVector3 &scale = btVector3(1,1,1),
				const int mass = 0);
    /*!
      * \brief Shallow copy constructor.
      * \param parent the parent obEntity of this instance
      * \param other the obRigidBody whose parameters to copy
      * \return a new obRigidBody
      *
      * \warning This constructor does not copy the other obRigidBody's btRigidBody.
      * Clients still have to create the btRigidBody themselves after constructing the
      * obDynamicRigidBody.
      */
    obRigidBody(obEntity *parent, const obRigidBody &other);

    /*!
      * \brief Default destructor.
      */
    virtual ~obRigidBody();
    /*!
      * \brief Gets the Bullet version of the rigid body.
      * \return the btRigidBody of the body
      */
    inline btRigidBody* getBulletBody() const
    {
        return btBody;
    }

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
      * \brief Gets the orientation of the rigid body.
      * \return the orientation of the body
      */
    const btQuaternion getRotation() const;

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
    btRigidBody         *btBody;        /**< Bullet rigid body */
    btTriangleMesh      *triangleMesh;  /**< Bullet polygonal mesh */
    btCollisionShape    *btShape;       /**< Bullet shape for collisions */
    btVector3           position;       /**< Position of the object at creation time */
    btVector3           scale;          /**< Object scale compared to original mesh */
    int                 mass;           /**< Mass of the object */
    btQuaternion        quaternion;     /**< Object rotation at creation time */
};

#endif // OGRERIGIDBODY_H
