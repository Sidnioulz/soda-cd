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
#ifndef SIMULATION_H
#define SIMULATION_H

#include "EKMeans/libekmeans.h"
#include <btBulletDynamicsCommon.h>
#include <Ogre.h>
#include <QVector>
#include <QMap>

#include "physicsworld.h"
#include "circulartransformbufferinterface.h"
#include "ogrewidget.h"
#include "obEntityWrapper.h"
#include "obghostentity.h"

/*! \class Simulation
  * \brief The virtual class that should be used to describe a Simulation.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class contains all the code to describe and setup a Simulation, with virtual
  * functions that should be implemented for specific instances of Simulations.
  */
class Simulation
{    
public:

	/*!
	  * \brief Default constructor.
	  * \param targetTimeStep the duration of a simulation time step
	  * \param numWorlds the number of PhysicsWorld to run in the Simulation (0 = automatic)
	  * \param numInterfaces the number of rendering interfaces to use (0 = automatic)
	  * \param sceneSize the size of the simulated area
	  * \param numEntities the number of simulated entities
	  * \return a new Simulation
	  */
    Simulation(const btScalar &targetTimeStep = 1.0f/60,
               const int &numWorlds = QThread::idealThreadCount(),
               const int &numInterfaces = 1,
               const btVector3 &sceneSize = btVector3(0, 0, 0),
               const int &numEntities = 0);

	/*!
	  * \brief Default destructor.
	  */
    virtual ~Simulation();

	/*!
	  * \brief Starts the Simulation, initializing it if necessary.
	  */
	virtual void start();

	/*!
	  * \brief Pauses simulation in the PhysicsWorlds.
	  *
	  * \warning This function is not implemented yet.
	  */
	virtual void pause();

	/*!
	  * \brief Stops the Simulation entirely and releases all used resources.
	  *
	  * \warning This function is not implemented yet.
	  */
	virtual void stop();

    /*!
     * \brief Setups the Ogre 3D environment in which the Simulation will occur (terrain, sky, fog, lights).
     */
    virtual void setupBasic3DEnvironment() = 0;

    /*!
     * \brief Setups the physics environment that matches the one defined in setupBasic3DEnvironment() (namely, a plane for the ground).
     * \param world the PhysicsWorld for which to setup the Physics environment
     */
    virtual void setupBasicPhysicsEnvironment(PhysicsWorld *world) = 0;




    // Create the CTB Interface
    //d Create buffers, have them watched by the interface
    //e Create physics worlds
    virtual void createBufferInterfaces();
    virtual void createPhysicsWorlds();

    virtual void loadEntities() = 0;    //pre f/g, fills this->entities

    //f Compute data for GridInformation (size ratio)
    //h Create GridInformation // unlink getGrid from static and link it to Simulation
    virtual void setupGridInformation();

    //g Get objects positions from the Simulation object (random placement in RandomCubeSimulation or read from Parser)
    virtual QVector<btVector3> getEntitiesPositions() const;


    //i Launch EKMeans on the list of centered positions (with random centroids in the space)
//    QVector<Ogre::Vector3> points;//(entities.size());
//    QVector<Ogre::Vector3> centroids;//(scene.size());
    virtual void computeClusterAssignments(const QVector<btVector3> &points);

    //j Get best LocalGrid resolution
    virtual int getBestTerritoryResolution() const;

    //k Compute territory boundaries from EKMeans clusters
    //l Create empty LocalGrids
    virtual void setupLocalGrids(const int &resolution, const QVector<btVector3> &points);


    //m Compute ideal margin for all LocalGrids (based on grid size)
    virtual QVector<int> computeMargin(const int &resolution, const btVector3 &minCoord, const btVector3 &maxCoord) const;

    //n Sort entities per Cell coordinates
    virtual void sortEntitiesPerCellCoordinates(const int &resolution);

    virtual QVector<btVector3> computeCellOwnersAndLocateEmptyCells(const int &resolution);

    virtual void assignSurroundedCellsToOwners(QVector<btVector3> &emptyCells);

    virtual void notifyCellAssignment(LocalGrid *local, const btVector3 &coords, const QVector<obEntityWrapper *> &entities, const short &owner);
    virtual void notifyCellAssignment(LocalGrid *local, const btVector3 &coords, const short &owner);

    void extendLocalGrids(const int &resolution, const QVector<btVector3> &points,const QVector<int> &assignments);

    //q Cleanup holes in all LocalGrids
    //r Use Kmeans to assign global empty Cells (using a bitmap to find them? or browsing thru entities?)
    //s Expand LocalGrids to scene borders

    //# at this point, all physics worlds have their objects with their grids, global scene is covered by all grids
    //# now we need to start the simulations.

    //#t onOgreReady function(3D World manager)
    //u Create 3D objects of all entities
    //v Start simulation.

    /*!
     * \brief Returns the CircularTransformBufferInterfaces defined in this Simulation.
     * \return a copy of the CircularTransformBufferInterface vector of this Simulation
     */
    inline QVector<CircularTransformBufferInterface *> getBufferInterfaces() const
    {
        return bufferInterfaces;
    }

protected:
    /*!
     * \brief Initializes the Simulation so that it can be started.
     */
    virtual void _init();


    void _setCellOwner(const QMap<short, int> &cellOwnerCounter, QVector<btVector3> &emptyCells, const btVector3 &currentCoords, const QVector<obEntityWrapper *> &cellEnts);

    btVector3 _nextOrderedCellCoords(const btVector3 &current);








    struct lessThan {
        lessThan(const int &resolution, GridInformation * grid) :
            resolution(resolution), grid(grid)
        {
        }

        bool operator()(const QPair<obEntityWrapper *, int> &p1, const QPair<obEntityWrapper *, int> &p2)
        {
            const btVector3 &pos1 = grid->toCellCoordinates(resolution, p1.first->getCenteredPosition());
            const btVector3 &pos2 = grid->toCellCoordinates(resolution, p2.first->getCenteredPosition());

            return  (pos1.x() < pos2.x()) ||
                    (pos1.x() == pos2.x() && pos1.y() < pos2.y()) ||
                    (pos1.x() == pos2.x() && pos1.y() == pos2.y() && pos1.z() < pos2.z());
        }

        int resolution;
        GridInformation *grid;
    };





//    virtual bool lessThanPerCoordinates(const QPair<obEntityWrapper *, int> &p1, const QPair<obEntityWrapper *, int> &p2);

    // Internal status
    enum {
        STOPPED = 1,                                                 /*!< the Simulation has not been started and needs initializing */
        PAUSED = 2,                                                  /*!< the physics part of the Simulation is paused */
        RUNNING                                                      /*!< the Simulation is running */
    } status;                                                        /*!< Status of the Simulation */

    // Simulation parameters - changes applied after Simulation restart
    btScalar targetTimeStep;                                         /*!< Framerate wanted for the Simulation */
    int numWorlds;                                                   /*!< Number of different PhysicsWorld instances */
    int numInterfaces;                                               /*!< Number of different CircularTransformBufferInterface instances */

    // Information on the Simulation
    btVector3 sceneSize;                                             /*!< Size of the simulated space */
    int numEntities;                                                 /*!< Number of simulated entities */

    // Data useful to GridInformation setup
    btScalar biggestSizeRatio;                                       /*!< Size ratios between the biggest and smallest entities, on each axis */
    btVector3 highestSizePerAxis;                                    /*!< Size of the biggest entity of the Simulation */
    GridInformation *grid;                                           /*!< Pointer to the GridInformation associated with this Simulation */
    QVector<LocalGrid*> grids;                                       /*!< Vector of LocalGrid pointers used by the PhysicsWorlds */

    // Pointers to simulation objects
    QVector<PhysicsWorld *> worlds;                                  /*!< Physics worlds for this Simulation */
    QVector<CircularTransformBufferInterface *> bufferInterfaces;    /*!< CircularTransformBufferInterface instances for this Simulation */
    QVector<QPair<obEntityWrapper *, int> > entitiesWithAssignments; /*!< List of simulated entities with the world they are assigned to */
    long entityIdCounter;                                            /*!< Number of entities created since the beginning of the Simulation (avoids name conflicts) */
};

#endif // SIMULATION_H
