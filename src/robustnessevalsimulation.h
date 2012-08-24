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
#ifndef ROBUSTNESSEVALSIMULATION_H
#define ROBUSTNESSEVALSIMULATION_H

#include "simulation.h"

/*! \class RobustnessEvalSimulation
  * \brief A class for the evaluation of the robustness property of SODA.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class allows evaluation of a robustness property by forcing a constant
  * (yet manageable) charge on the collision handling engine and then by suddenly
  * increasing it by a fixed factor and counting how much time it takes before
  * simulation by the engine cannot cope with the application's framerate.
  */
class RobustnessEvalSimulation : public Simulation
{
public:
    /*!
     * \brief Default constructor.
     * \param targetTimeStep the duration of a simulation time step
     * \param numEntities the number of simulated entities
     * \return a new RandomCubeSimulation
     */
    RobustnessEvalSimulation(const btScalar &targetTimeStep, const int &numEntities);

    ~RobustnessEvalSimulation();

    /*!
     * \overload
     */
    void setupBasic3DEnvironment();

    /*!
     * \overload
     */
    void setupBasicPhysicsEnvironment(sodaLogicWorld *world);

    /*!
     * \overload
     */
     void loadEntities();

    /*!
     * \overload
     */
     inline GridInformation::WorldType getWorldType() const
     {
         return GridInformation::ClosedWorld;
     }

     /*!
      * \overload
      */
     inline bool hasTickCallback() const
     {
         return true;
     }

     /*!
      * \brief Repositions simulated entities and adds some if over 1 second of simulation.
      * \param world the simulated sodaLogicWorld
      */
     void tickCallback(sodaLogicWorld *world, const btScalar &);

    /*!
     * \brief Creates a Ninja. Use with caution.
     * \param position the x,y,z coordinates of the object
     * \param scale the scale of the object in all axes (default 1 for each axis)
     * \param mass the mass of the object (default 1)
     * \return a pointer to the newly created entity
     */
    sodaDynamicEntity *_createNinja(const btVector3 &position, const btVector3 &scale, const btScalar &mass);
};

#endif // ROBUSTNESSEVALSIMULATION_H
