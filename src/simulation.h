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
#include <QHash>
#include <QTextStream>

#include "sodaLogicWorld.h"
#include "circulartransformbufferinterface.h"
#include "sodaDynamicEntity.h"

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
      * \param declNumWorlds the number of sodaLogicWorld to run in the Simulation (0 = automatic)
	  * \param sceneSize the size of the simulated area
	  * \param numEntities the number of simulated entities
	  * \return a new Simulation
	  */
    Simulation(const btScalar &targetTimeStep = 1.0f/60,
               const int &declNumWorlds = 0,
               const btVector3 &sceneSize = btVector3(0, 0, 0),
               const int &numEntities = 0);

	/*!
	  * \brief Default destructor.
	  */
    virtual ~Simulation();

    /*!
      * \brief Prints some statistics about the Simulation.
      * \param out the stream to write to
      */
    virtual void printStats(QTextStream &out) const;

    /*!
      * \brief Starts the Simulation, initializing it if necessary.
      */
    virtual void start();

	/*!
	  * \brief Pauses simulation in the sodaLogicWorlds.
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
     * \brief Returns the CircularTransformBufferInterface defined in this Simulation.
     * \return a pointer to the CircularTransformBufferInterface of this Simulation
	 */
    inline CircularTransformBufferInterface *getBufferInterface() const
	{
        return bufferInterface;
	}

    /*!
     * \brief Returns the type of world being simulated.
     * \return the WorldType of this Simulation
     */
    virtual GridInformation::WorldType getWorldType() const = 0;

    /*!
     * \brief Retrieves the number of defined sodaLogicWorlds.
     * \return the number of sodaLogicWorlds in the Simulation
     */
    inline int getNumWorlds() const
    {
        return worlds.size();
    }

    /*!
     * \brief Retrieves a sodaLogicWorld from its identifier.
     * \param id the identifier of the wanted sodaLogicWorld
     * \return the corresponding sodaLogicWorld if it exists, or null otherwise
     *
     * \warning This method is temporary and to be supersed. In the future, sodaLogicWorlds should hold pointers to their neighbors.
     */
    //TODO: getWorldFromId(): Implement the neighbor information as a local table in sodaLogicWorlds.
    inline sodaLogicWorld *getWorldFromId(const short id) const
    {
        if(id<=0 || id>worlds.size())
            return 0;
        else
            return worlds[id-1];
    }

    /*!
     * \brief Tells whether this Simulation object has a tickCallback to call after each tick.
     * \return whether the Simulation has a tick callback
     */
    virtual bool hasTickCallback() const = 0;

    /*!
     * \brief Runs custom code after each tick.
     * \param world the sodaLogicWorld where the code will run
     * \param timeStep the timestep of the tick
     */
    virtual void tickCallback(sodaLogicWorld *world, const btScalar &timeStep) = 0;


protected:
	/*!
     * \brief Setups the Ogre 3D environment in which the Simulation will occur (terrain, sky, fog, lights).
     */
    virtual void setupBasic3DEnvironment() = 0;

    /*!
     * \brief Setups the physics environment that matches the one defined in setupBasic3DEnvironment() (namely, a plane for the ground).
     * \param world the sodaLogicWorld for which to setup the Physics environment
     */
    virtual void setupBasicPhysicsEnvironment(sodaLogicWorld *world) = 0;

	/*!
     * \brief Creates the CircularTransformBufferInterface that will be used in the Simulation.
	 *
	 * \note Called by the constructor.
	 */
    virtual void createBufferInterface();

	/*!
	 * \brief Creates the sodaLogicWorlds that will be used in the Simulation.
	 *
	 * \note Called by the constructor.
	 */
    virtual void createsodaLogicWorlds();

	/*!
	  * \brief Loads the entities that will be simulated. Must be defined by inheriting classes.
	  */
    virtual void loadEntities() = 0;

    /*!
      * \brief Computes the parameters that will be needed to compute Grid cell sizes and depth.
      */
    virtual void setupGridInformation();

	/*!
	  * \brief Retrieves the centered positions of all simulated entities. Necessary for the interface to Clustering algorithms.
	  * \return a vector containing the centered positions of all sodaDynamicEntity instances
	  *
	  * \note The position at index i corresponds to the entity at the same index i in entitiesWithAssignments.
	  */
    virtual QVector<btVector3> getEntitiesPositions() const;

	/*!
	  * \brief Computes assignments into as many clusters as there are worlds, for a given set of points.
	  * \param points the points that must be assigned a cluster
	  *
	  * This function computes clusters (as many as there are sodaLogicWorlds in this Simulation), using the
	  * defined Clustering algorithm of the Simulation. This clustering algorithm regroups points by
	  * geographic proximity, into contiguous sets.
	  */
    virtual void computeClusterAssignments(const QVector<btVector3> &points);

	/*!
      * \brief Returns the most appropriate GridInformation resolution at which sodaLocalGrids should be defined.
      * \return the best resolution at which sodaLocalGrids should be defined
	  */
    virtual int getBestTerritoryResolution() const;

	/*!
      * \brief Creates sodaLocalGrids and fills them with the entities that were assigned to their world's id.
      * \param resolution the resolution at which sodaLocalGrids should be created
	  * \param points the positions of the entities
	  *
	  * This function first computes the areas, in Cell coordinates, that entirely cover each cluster that
	  * was computed in computeClusterAssignments(), using points and entitiesWithAssignments. Then, it
      * creates the sodaLocalGrids, giving them an owner Id. The sodaLocalGrids are assigned later to their sodaLogicWorld.
	  */
    void setupLocalGrids(const int &resolution, const QVector<btVector3> &points);

	/*!
      * \brief Computes the margin that should be given to a sodaLocalGrid.
      * \param resolution the resolution of the sodaLocalGrid
	  * \param minCoord the Cell coordinates of its lower-left-back corner
	  * \param maxCoord the Cell coordinates of its top-right-front corner
	  * \return a vector containing margins for all six directions
	  *
	  * \warning This function is not implemented yet and returns a zero margin.
	  */
    QVector<int> computeMargin(const int &resolution, const btVector3 &minCoord, const btVector3 &maxCoord) const;

	/*!
	  * \brief Sorts the entitiesWithAssignments vector by Cell coordinates.
	  * \param resolution the resolution at which entities should be sorted
	  *
	  * This function sorts the entitiesWithAssignments vector by Cell coordinates,
	  * using the lessThan class. The entities vector can then be browsed Cell
	  * by Cell at this resolution.
	  */
    void sortEntitiesPerCellCoordinates(const int &resolution);

	/*!
      * \brief Browses entitiesWithAssignments to find the best owner for each entity-containing Cell, and updates sodaLocalGrids.
      * \param resolution the resolution at which sodaLocalGrids have been defined
	  * \return a list of Cells that are empty and were not assigned
	  */
    QVector<btVector3> computeCellOwnersAndLocateEmptyCells(const int &resolution);

	/*!
      * \brief Tells each sodaLocalGrid to assign to self the Cells that are surrounded by already owned Cells.
	  * \param emptyCells a list of coordinates of Cells that are still unassigned
	  */
    void assignSurroundedCellsToOwners(QVector<btVector3> &emptyCells);

	/*!
      * \brief Uses a clustering algorithm to assign every unassigned empty Cell to a sodaLocalGrid.
      * \param resolution the resolution at which sodaLocalGrids have been defined
	  * \param emptyCells a list of coordinates of Cells that are still unassigned
	  */
    void assignEmptyCells(const int &resolution, const QVector<btVector3> &emptyCells);

	/*!
      * \brief Notifies a sodaLocalGrid about the fact that a Cell is owned by another one.
      * \param local the sodaLocalGrid to notify
	  * \param coords the coordinates of the Cell
      * \param owner the id of the sodaLocalGrid to which this Cell was assigned
	  * \param entities the entities whose coordinates are in the given Cell, if any
	  */
    void notifyCellAssignment(sodaLocalGrid *local, const btVector3 &coords, const short &owner, const QVector<sodaDynamicEntity *> *entities = 0);

	/*!
      * \brief Extends sodaLocalGrids in order for them to be able to contain all the points assigned to them in the parameters.
      * \param resolution the resolution of the sodaLocalGrids
	  * \param points the positions of points assigned to them
	  * \param assignments the assignments of points to each grid
	  */
	void extendLocalGrids(const int &resolution, const QVector<btVector3> &points, const QVector<int> &assignments);

    /*!
      * \brief Appends an entity to the Simulation. For use from loadEntities() in child classes.
      * \param obEnt the sodaDynamicEntity to append
      */
    inline void appendEntity(sodaDynamicEntity *obEnt)
    {
        entitiesWithAssignments.append(QPair<sodaDynamicEntity *, int>(obEnt, sodaLogicWorld::UnknownWorldId));
    }

    /*!
     * \brief Initializes Simulation data (by default, calls setupBasic3DEnvironment(), setupBasicPhysicsEnvironment() and loadEntities())
     */
    virtual void loadSimulationData();
	/*!
	 * \brief Initializes the Simulation so that it can be started.
	 */
    void _init();

private:
	/*!
	  * \brief Computes the owner of a given Cell based on the entities it contains.
	  * \param cellOwnerCounter the number of entities owned for each sodaLogicWorld within this Cell
	  * \param emptyCells a reference to current empty Cells in case the current Cell is to be added to the list
	  * \param currentCoords coordinates of the current Cell
	  * \param cellEnts the entities contained within this Cell
	  */
    void _setCellOwner(const QHash<short, int> &cellOwnerCounter, QVector<btVector3> &emptyCells, const btVector3 &currentCoords, const QVector<sodaDynamicEntity *> &cellEnts);

	/*!
	  * \brief Computes the coordinates of the next Cell for browsing Cells in a given order (same as the one in lessThan).
	  * \param current the coordinates of the current Cell
	  * \return the coordinates of the next Cell
	  */
    btVector3 _nextOrderedCellCoords(const btVector3 &current);

protected:
	/*! \struct lessThan
	  * \brief An implementation of a comparison function for sodaDynamicEntity Cell coordinates.
	  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
	  */
    struct lessThan {
		/*!
		  * \brief Default constructor.
		  * \param resolution the resolution to use for Cell coordinates
		  * \param grid the GridInformation in which the Cell space is defined
		  * \return a new lessThan
		  */
		lessThan(const int &resolution, GridInformation *grid) :
            resolution(resolution), grid(grid)
        {
        }

		/*!
		  * \brief The operator that is called for comparison of two sodaDynamicEntity instances.
		  * \param p1 the first sodaDynamicEntity
		  * \param p2 the second sodaDynamicEntity
		  * \return whether p1's coordinates are lesser than p2's in the order defined by this class.
		  *
		  * \note p1 and p2 are in a QPair format because it is more practical for use with
		  * entitiesWithAssignments.
		  */
        bool operator()(const QPair<sodaDynamicEntity *, int> &p1, const QPair<sodaDynamicEntity *, int> &p2)
        {
            const btVector3 &pos1 = grid->toCellCoordinates(resolution, p1.first->getCenteredPosition());
            const btVector3 &pos2 = grid->toCellCoordinates(resolution, p2.first->getCenteredPosition());

            return  (pos1.x() < pos2.x()) ||
                    (pos1.x() == pos2.x() && pos1.y() < pos2.y()) ||
                    (pos1.x() == pos2.x() && pos1.y() == pos2.y() && pos1.z() < pos2.z());
        }

		int resolution;				/*!< The resolution of this lessThan comparison structure */
		GridInformation *grid;		/*!< The GridInformation from which to read numbers of Cells */
    };

    // Internal status
    enum {
        STOPPED = 1,                                                 /*!< the Simulation has not been started and needs initializing */
        PAUSED = 2,                                                  /*!< the physics part of the Simulation is paused */
        RUNNING                                                      /*!< the Simulation is running */
    } status;                                                        /*!< Status of the Simulation */

    // Simulation parameters - changes applied after Simulation restart
    btScalar targetTimeStep;                                         /*!< Framerate wanted for the Simulation */
    int numWorlds;                                                   /*!< Number of different sodaLogicWorld instances */

    // Information on the Simulation
    btVector3 sceneSize;                                             /*!< Size of the simulated space */
    int numEntities;                                                 /*!< Number of simulated entities */

    // Data useful to GridInformation setup
    btScalar biggestSizeRatio;                                       /*!< Size ratios between the biggest and smallest entities, on each axis */
    btVector3 highestSizePerAxis;                                    /*!< Size of the biggest entity of the Simulation */
    GridInformation *grid;                                           /*!< Pointer to the GridInformation associated with this Simulation */
    QVector<sodaLocalGrid*> grids;                                   /*!< Vector of sodaLocalGrid pointers used by the sodaLogicWorlds */

    // Pointers to simulation objects
    QVector<sodaLogicWorld *> worlds;                                  /*!< Physics worlds for this Simulation */
    CircularTransformBufferInterface *bufferInterface;               /*!< CircularTransformBufferInterface for this Simulation */
    QVector<QPair<sodaDynamicEntity *, int> > entitiesWithAssignments; /*!< List of simulated entities with the world they are assigned to */
    long entityIdCounter;                                            /*!< Number of entities created since the beginning of the Simulation (avoids name conflicts) */
};

#endif // SIMULATION_H
