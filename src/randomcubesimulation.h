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
#ifndef RANDOMCUBESIMULATION_H
#define RANDOMCUBESIMULATION_H

#include "simulation.h"
#include "obEntityWrapper.h"
#include "obghostentity.h"

/*! \class RandomCubeSimulation
  * \brief A class for the simulation of randomly placed cubes.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class contains all the code to manage the animation of the world and the synchronization
  * of bullet collision detection/response and Ogre rendering. It also contains the code that
  * defines the initial scene.
  */
class RandomCubeSimulation : public Simulation
{
public:
    /*!
     * \brief Default constructor.
     * \param targetTimeStep the duration of a simulation time step
     * \param numWorlds the number of physics worlds to launch
     * \param numInterfaces the number of rendering interfaces to use
     * \param sceneSize the dimensions of the simulation scene
     * \param numEntities the number of simulated entities
     * \return a new RandomCubeSimulation
     */
    RandomCubeSimulation(const btScalar &targetTimeStep = 1.0f/60,
                         const int &numWorlds = 2, //QThread::idealThreadCount(),
                         const int &numInterfaces = 1,
                         //                         const btVector3 &sceneSize = btVector3(1000, 400, 1000), interesting as a case study where one CPU gets no territory
                         const btVector3 &sceneSize = btVector3(2000, 800, 2000),
                         const int &numEntities = 40);

    /*!
      * \brief Default destructor.
      */
    virtual ~RandomCubeSimulation();

    /*!
     * \overload
     */
    virtual void setupBasic3DEnvironment();

    /*!
     * \overload
     */
    virtual void setupBasicPhysicsEnvironment(PhysicsWorld *world);

    /*!
     * \overload
     */
    virtual void loadEntities();

    /*!
      * \brief Creates a 3D box entity with both an Ogre and a bullet structure.
      * \param position the x,y,z coordinates of the object
      * \param scale the scale of the object in all axes (default 1 for each axis)
      * \param mass the mass of the object (default 1)
      * \return a pointer to the newly created entity
      */
    obEntityWrapper *_createBox(const btVector3 &position, const btVector3 &scale, const btScalar &mass);

    /*!
      * \brief Creates a Ninja. Use with caution.
      * \param position the x,y,z coordinates of the object
      * \param scale the scale of the object in all axes (default 1 for each axis)
      * \param mass the mass of the object (default 1)
      * \return a pointer to the newly created entity
      */
    obGhostEntity *_createNinja(const btVector3 &position, const btVector3 &scale, const btScalar &mass);

};


#endif // RANDOMCUBESIMULATION_H
