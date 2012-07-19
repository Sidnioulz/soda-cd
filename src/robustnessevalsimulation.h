#ifndef ROBUSTNESSEVALSIMULATION_H
#define ROBUSTNESSEVALSIMULATION_H

#include "simulation.h"

class RobustnessEvalSimulation : public Simulation
{
public:
    RobustnessEvalSimulation(const btScalar &targetTimeStep, const int &numEntities);

    ~RobustnessEvalSimulation();

    /*!
     * \overload
     */
    void setupBasic3DEnvironment();

    /*!
     * \overload
     */
    void setupBasicPhysicsEnvironment(PhysicsWorld *world);

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

     //TODO: doc
     inline bool hasTickCallback() const
     {
         return true;
     }

     void tickCallback(PhysicsWorld *world, const btScalar &timeStep);

    /*!
     * \brief Creates a Ninja. Use with caution.
     * \param position the x,y,z coordinates of the object
     * \param scale the scale of the object in all axes (default 1 for each axis)
     * \param mass the mass of the object (default 1)
     * \return a pointer to the newly created entity
     */
    obEntityWrapper *_createNinja(const btVector3 &position, const btVector3 &scale, const btScalar &mass);
};

#endif // ROBUSTNESSEVALSIMULATION_H
