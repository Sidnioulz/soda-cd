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
#include "simulation.h"
#include "experimenttrackinginterface.h"

Simulation::Simulation(const btScalar &targetTimeStep, const int &declNumWorlds, const btVector3 &sceneSize, const int &numEntities) :
    status(Simulation::STOPPED),
    targetTimeStep(targetTimeStep),
    numWorlds((declNumWorlds == 0 ? QThread::idealThreadCount() : declNumWorlds)),
    sceneSize(sceneSize),
    numEntities(numEntities),
    grid(0),
    grids(numWorlds, 0),
    worlds(numWorlds, 0),
    bufferInterface(),
    entitiesWithAssignments(),
    entityIdCounter(0)
{
    createBufferInterface();
    createsodaLogicWorlds();

    ExperimentTrackingInterface::getInstance()->clearStats();
}

Simulation::~Simulation()
{
    for(int i=0; i<numWorlds; ++i)
        delete worlds[i];

    delete bufferInterface;
}

void Simulation::printStats(QTextStream &out) const
{
    ExperimentTrackingInterface::getInstance()->printSynchronizationTimeStats(*this, out);
}

void Simulation::start()
{
    if(status == STOPPED)
    {
        _init();
        status = PAUSED;
    }

    // Resume the Simulation by telling the sodaLogicWorlds to run
    if(status == PAUSED)
    {
        // Start the simulations
        //TODO: just unlock a WaitCondition that all threads will be waiting for.
        for(int i=0; i<numWorlds; ++i)
        {
            worlds[i]->startSimulation();
        }

        status = RUNNING;
    }
}

void Simulation::pause()
{
    if(status == RUNNING)
    {
        //TODO: pause the sodaLogicWorlds and tell the rendering thread to ask for the latest available past

        status = PAUSED;
    }
}

void Simulation::stop()
{
    if(status != STOPPED)
    {
#ifndef NDEBUG
        qDebug() << "Simulation::stop(); Simulation about to be stopped; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

        for(int i=0; i<numWorlds; ++i)
        {
#ifndef NDEBUG
            qDebug() << "Simulation::stop(); Stopping world " << worlds[i]->getId() << "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
            worlds[i]->stopSimulation();
#ifndef NDEBUG
            qDebug() << "Simulation::stop(); Stopped world " << worlds[i]->getId() << "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        }

        status = STOPPED;
#ifndef NDEBUG
        qDebug() << "Simulation::stop(); Simulation is now stopped; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
    }
}

void Simulation::createBufferInterface()
{
    // Create the rendering thread's interface
    bufferInterface = new CircularTransformBufferInterface();
}

void Simulation::createsodaLogicWorlds()
{
    // Create the physics worlds
    for(int i=0; i<numWorlds; ++i)
    {
        worlds[i] = new sodaLogicWorld(*this, targetTimeStep);
    }

    // Have the CircularTransformBuffer interfaces watch the worlds' buffers.
    // Later, several interfaces can be used and this loop changed
    for(int i=0; i<numWorlds; ++i)
    {
        bufferInterface->watchBuffer(worlds[i]->getCircularBuffer());
    }
}

void Simulation::setupGridInformation()
{
    // Variables used to get the size ratio of simulated objects
    btVector3 lowestSizePerAxis(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    highestSizePerAxis.setValue(0, 0, 0);

    for(int i=0; i<numEntities; ++i)
    {
        // Update the min and max encountered scales for spatial subdivision grid depth
        btVector3 entitySize = entitiesWithAssignments[i].first->getSize();
        lowestSizePerAxis.setMin(entitySize);
        highestSizePerAxis.setMax(entitySize);
    }

    biggestSizeRatio = qMax(highestSizePerAxis.x() / lowestSizePerAxis.x(),
                       qMax(highestSizePerAxis.y() / lowestSizePerAxis.y(),
                            highestSizePerAxis.z() / lowestSizePerAxis.z()));

    grid = GridInformation::getGrid(getWorldType(), biggestSizeRatio, sceneSize, highestSizePerAxis);
}

QVector<btVector3> Simulation::getEntitiesPositions() const
{
    QVector<btVector3> points(entitiesWithAssignments.size());
    for(int i=0; i<entitiesWithAssignments.size(); ++i)
        points[i] = entitiesWithAssignments[i].first->getCenteredPosition();

    return points;
}

void Simulation::computeClusterAssignments(const QVector<btVector3> &points)
{
#ifndef NDEBUG
	QElapsedTimer t;
	t.start();
	qDebug("Clustering algorithm started.");
#endif

    QVector<btVector3> centroids(numWorlds);
    for(int i=0; i<centroids.size(); ++i)
//      centroids[i] = btVector3(rand() % (int)sceneSize.x() - sceneSize.x() / 2,
//                               rand() % (int)sceneSize.y(),
//                               rand() % (int)sceneSize.z() - sceneSize.z() / 2);

    // Vertical clusters
        centroids[i] = btVector3(rand() % (int)sceneSize.x() - sceneSize.x() / 2,
                                 0,
                                 rand() % (int)sceneSize.z() - sceneSize.z() / 2);


	// Call EKMeans on the created objects
    Clustering::EKMeans ekm(centroids, points);
    ekm.setEqual(true);
    ekm.run();

    QVector<int> assignments = ekm.getAssignments();
    for(int i=0; i<assignments.size(); ++i)
        entitiesWithAssignments[i].second = assignments[i];

#ifndef NDEBUG
	qDebug() << "Clustering finished in " << t.elapsed() << "ms.";
#endif
}

int Simulation::getBestTerritoryResolution() const
{
    return grid->getBestTerritoryResolution();
}

void Simulation::setupLocalGrids(const int &resolution, const QVector<btVector3> &points)
{
#ifndef NDEBUG
    QElapsedTimer t;
    t.start();
	qDebug("LocalGrid setup started.");
#endif

    // This vector contains a pair of coordinates used to define the rectangles within which each cluster exists
    QVector<QPair<btVector3, btVector3> > territoryBoundaries(numWorlds,
                QPair<btVector3, btVector3>(sceneSize, btVector3(0, 0, 0) - sceneSize));

    // Go through EKMeans assignments to compute boundaries
    for(int i=0; i<entitiesWithAssignments.size(); ++i)
    {
        // Compute the boundaries of each cluster computed by EKMeans
        QPair <btVector3, btVector3> &boundaries = territoryBoundaries[entitiesWithAssignments[i].second];
        boundaries.first.setMin(points[i]);
        boundaries.second.setMax(points[i]);
    }

    // Go through each cluster boundaries to create appropriate local grids
    for(int i=0; i<numWorlds; ++i)
    {
        try
        {
            btVector3 minCoord = grid->toCellCoordinates(resolution, territoryBoundaries[i].first);
            btVector3 maxCoord = grid->toCellCoordinates(resolution, territoryBoundaries[i].second);

//            qDebug() << "LocalGrid " << i << "has coordinates:";
//            qDebug() << minCoord.x() << minCoord.y() << minCoord.z();
//            qDebug() << maxCoord.x() << maxCoord.y() << maxCoord.z();

            QVector<int> margin = computeMargin(resolution, minCoord, maxCoord);
            grids[i] = new sodaLocalGrid(grid->getGridAtResolution(resolution), worlds[i]->getId(), margin, (maxCoord+btVector3(1,1,1)-minCoord), minCoord);
        }
        catch (exception& e)
        {
            qErrnoWarning("Failed to get grid coordinates for the territory boundaries of world %d (%s)", i, e.what());
        }
    }
#ifndef NDEBUG
	qDebug() << "LocalGrids created in " << t.elapsed() << "ms.";
#endif
}

QVector<int> Simulation::computeMargin(const int &resolution, const btVector3 &minCoord, const btVector3 &maxCoord) const
{
    QVector<int> margins(6, 0);

    return margins;

//    margins[GridInformation::Right] = (grid->getGridAtResolution(resolution)->isWithinWorldCellBounds(maxCoord + btVector3(1,0,0))) ? 1 : 0;
//    margins[GridInformation::Top] = (grid->getGridAtResolution(resolution)->isWithinWorldCellBounds(maxCoord + btVector3(0,1,0))) ? 1 : 0;
//    margins[GridInformation::Front] = (grid->getGridAtResolution(resolution)->isWithinWorldCellBounds(maxCoord + btVector3(0,0,1))) ? 1 : 0;

//    margins[GridInformation::Left] = (grid->getGridAtResolution(resolution)->isWithinWorldCellBounds(minCoord - btVector3(1,0,0))) ? 1 : 0;
//    margins[GridInformation::Bottom] = (grid->getGridAtResolution(resolution)->isWithinWorldCellBounds(minCoord - btVector3(0,1,0))) ? 1 : 0;
//    margins[GridInformation::Back] = (grid->getGridAtResolution(resolution)->isWithinWorldCellBounds(minCoord - btVector3(0,0,1))) ? 1 : 0;

//    return margins;
}

void Simulation::sortEntitiesPerCellCoordinates(const int &resolution)
{
    lessThan lt(resolution, grid);
    qSort(entitiesWithAssignments.begin(), entitiesWithAssignments.end(), lt);
}

void Simulation::_setCellOwner(const QHash<short, int> &cellOwnerCounter, QVector<btVector3> &emptyCells, const btVector3 &currentCoords, const QVector<sodaDynamicEntity *> &cellEnts)
{
    // Find in the owner count for the current Cell which sodaLogicWorld owns most entities
    short maxIndex = sodaLogicWorld::NullWorldId;
    int maxCount = 0;
    QHashIterator<short, int> it(cellOwnerCounter);
    while(it.hasNext())
    {
        it.next();
        if(it.value()>maxCount)
        {
            maxIndex = it.key();
            maxCount = it.value();
        }
    }

    // If the Cell was empty, add it to the list of empty Cells for later assignment
    if(maxIndex == sodaLogicWorld::NullWorldId)
    {
//        qDebug() << "\tEmpty " << currentCoords.x() << currentCoords.y() << currentCoords.z();
        emptyCells.append(currentCoords);
    }
    // Else, notify all sodaLogicWorlds which one the Cell was assigned to
    else
    {
//        qDebug() << "_setCellOwner decision:" << currentCoords.x() << currentCoords.y() << currentCoords.z() << " owned by ID " << worlds[maxIndex]->getId() << " with" << cellEnts.size() << "entities";

        for(int w=0; w<numWorlds; ++w)
            notifyCellAssignment(grids[w], currentCoords, worlds[maxIndex]->getId(), &cellEnts);
    }
}

btVector3 Simulation::_nextOrderedCellCoords(const btVector3 &current)
{
    const btVector3 &nbCells =  grid->getNbCells();
    btVector3 result(current.x(), current.y(), current.z()+1);

    if(result.z() == nbCells.z())
    {
        result.setY(result.y()+1);
        result.setZ(0);

        if(result.y() == nbCells.y())
        {
            result.setX(result.x()+1);
            result.setY(0);

            if(result.x() == nbCells.x())
            {
                return GridInformation::InvalidCellCoordinates;
            }
        }
    }

    return result;
}

QVector<btVector3> Simulation::computeCellOwnersAndLocateEmptyCells(const int &resolution)
{
    // Sort entities based on their Cell coordinates
    sortEntitiesPerCellCoordinates(resolution);

    // A container for empty Cells
    QVector<btVector3> emptyCells;

    // Browse through all entities, to process ownership Cell after Cell
    QHash<short, int> cellOwnerCounter;
    QVector<sodaDynamicEntity *> cellEnts;
    btVector3 currentCoords(grid->toCellCoordinates(resolution, btVector3(-sceneSize.x()/2, 0, -sceneSize.z()/2)));
    for(int i=0; i<entitiesWithAssignments.size(); ++i)
    {
        sodaDynamicEntity *obEnt = entitiesWithAssignments[i].first;
        const btVector3 &coords = grid->toCellCoordinates(resolution, obEnt->getCenteredPosition());

//        qDebug() << "JCoordinates " << coords.x() << coords.y() << coords.z();

        // Different Cells, compute the owner for the current Cell and notify all worlds
        if(coords != currentCoords)
        {
//            qDebug() << "\tCoordinates " << currentCoords.x() << currentCoords.y() << currentCoords.z();
            _setCellOwner(cellOwnerCounter, emptyCells, currentCoords, cellEnts);

            // For all Cells between the one just processed and the next to contain an entity, mark the Cell empty
            btVector3 nextCoords = _nextOrderedCellCoords(currentCoords);
            while (nextCoords != coords)
            {
//                qDebug() << "\tCoordinates " << nextCoords.x() << nextCoords.y() << nextCoords.z();
//                qDebug() << "\tEmpty " << nextCoords.x() << nextCoords.y() << nextCoords.z();
                emptyCells.append(nextCoords);
                nextCoords = _nextOrderedCellCoords(nextCoords);
            }

            // Clear variables for next Cell's processing
            cellOwnerCounter.clear();
            cellEnts.clear();
            currentCoords = coords;
        }

        cellEnts.append(obEnt);
        cellOwnerCounter[entitiesWithAssignments[i].second] = cellOwnerCounter.value(entitiesWithAssignments[i].second, 0) + 1;
    }
    // Process the Cell containing the last entity
    _setCellOwner(cellOwnerCounter, emptyCells, currentCoords, cellEnts);
//    qDebug() << "Cell " << currentCoords.x() << currentCoords.y() << currentCoords.z() << " now owned by a world";

    // Then, mark empty all next Cells
    btVector3 nextCoords = _nextOrderedCellCoords(currentCoords);
    while (nextCoords != GridInformation::InvalidCellCoordinates)
    {
//        qDebug() << "Coordinates " << nextCoords.x() << nextCoords.y() << nextCoords.z();
//        qDebug() << "Empty " << nextCoords.x() << nextCoords.y() << nextCoords.z();
        emptyCells.append(nextCoords);
        nextCoords = _nextOrderedCellCoords(nextCoords);
    }

    return emptyCells;
}

void Simulation::assignSurroundedCellsToOwners(QVector<btVector3> &emptyCells)
{
    for(int i=0; i<grids.size(); ++i)
    {
        // Call the algorithm that will identify such empty Cells and own them
        QVector<btVector3> emptyAssigned = grids[i]->resolveEmptyCellOwnerships();

//      // Remove these now assigned Cells from the list of empty Cells
        for(int j=0; j<emptyAssigned.size(); ++j)
        {
            int index = emptyCells.indexOf(emptyAssigned[j]);
            if(index != -1)
			{
				emptyCells.remove(index);
				for(int w=0; w<numWorlds; ++w)
					notifyCellAssignment(grids[w], emptyAssigned[j], i);
			}
			else
			{
				qWarning() << "assignSurroundedCellsToOwners() assigned to grid" << i << "the Cell"
						<< emptyAssigned[j].x() << emptyAssigned[j].y() << emptyAssigned[j].z()
						<< "while it was apparently not empty.";
			}
        }
    }
}

void Simulation::assignEmptyCells(const int &resolution, const QVector<btVector3> &emptyCells)
{
	QVector<btVector3> points;
	QVector<btVector3> centroids;

	// Include all empty Cells in the set of points
	for(int i=0; i<emptyCells.size(); ++i)
		points.append(grid->toCenteredWorldCoordinates(resolution, emptyCells[i]));

	// Set the centroids to the middle of each array
	for(int i=0; i<numWorlds; ++i)
		centroids.append(grids[i]->getCenteredPosition());

	// Run EKMeans
	Clustering::EKMeans emptyCellsEKM(centroids, points);
	emptyCellsEKM.setFixedCentroids(true);
	emptyCellsEKM.run();

	// Now extend grids for them to can include all their newly assigned Cells
	QVector<int> assignments = emptyCellsEKM.getAssignments();
	extendLocalGrids(resolution, points, assignments);

    // Finally, notify Cell assignments to all sodaLocalGrids
	for(int i=0; i<assignments.size(); ++i)
		for(int w=0; w<numWorlds; ++w)
			notifyCellAssignment(grids[w], emptyCells[i], worlds[assignments[i]]->getId());
}

void Simulation::notifyCellAssignment(sodaLocalGrid *local, const btVector3 &coords, const short &owner, const QVector<sodaDynamicEntity *> *entities)
{
    local->setCellOwnedBy(coords, owner);

	if(entities && local->getOwnerId() == owner)
    {
//        qDebug() << "notifyCellAssignment: confirmed that world" << owner << "owns" << entities->size() << "entities.";

		for(int i=0; i<entities->size(); ++i)
            local->addEntity(entities->at(i));
    }
}

void Simulation::extendLocalGrids(const int &resolution, const QVector<btVector3> &points,const QVector<int> &assignments)
{
#ifndef NDEBUG
    QElapsedTimer t;
    t.start();
	qDebug("LocalGrid extension started.");
#endif

    const btVector3 &nbCells = grid->getGridAtResolution(resolution)->getNbCells();

//    for(int x=0; x<nbCells.x(); ++x)
//    {
//        for(int y=0; y<nbCells.y(); ++y)
//        {
//            for(int z=0; z<nbCells.z(); ++z)
//            {
//                btVector3 test = grid->toCellCoordinates(resolution, grid->toCenteredWorldCoordinates(resolution, btVector3(x,y,z)));
//                qDebug() << x << y << z << " \t ~~~ \t "<< test.x() << test.y() << test.z();
//            }
//        }
//    }

    // This vector contains a pair of coordinates used to define the rectangles within which each cluster exists
    QVector<QPair<btVector3, btVector3> > territoryBoundaries(numWorlds,
                                                              QPair<btVector3, btVector3>(sceneSize + btVector3(1, 1, 1),
                                                                                          btVector3(-1,-1,-1)));

    // Go through EKMeans assignments to compute boundaries
    for(int i=0; i<assignments.size(); ++i)
    {
        // Compute the boundaries of each cluster computed by EKMeans
        QPair <btVector3, btVector3> &boundaries = territoryBoundaries[assignments[i]];
        boundaries.first.setMin(points[i]);
        boundaries.second.setMax(points[i]);
    }

    // Go through each cluster boundaries to create appropriate local grids
    for(int i=0; i<territoryBoundaries.size(); ++i)
    {
        try
        {
            btVector3 minCoord = grid->toCellCoordinates(resolution, territoryBoundaries[i].first);
            btVector3 maxCoord = grid->toCellCoordinates(resolution, territoryBoundaries[i].second);

            qDebug() << "LocalGrid " << i << " being extended to:";
            qDebug() << minCoord.x() << minCoord.y() << minCoord.z();
            qDebug() << maxCoord.x() << maxCoord.y() << maxCoord.z();

            //FIXME: here we resize to a full size because margins are not properly taken into account on all sides of territories
            QVector<int> margin = computeMargin(resolution, minCoord, maxCoord);
//            grids[i]->resize(margin, (maxCoord+btVector3(1,1,1)-minCoord), minCoord);
            grids[i]->resize(margin, nbCells, btVector3(0, 0, 0));
        }
        catch (exception& e)
        {
            qErrnoWarning("Failed to get grid coordinates for the territory boundaries of world %d (%s)", i, e.what());
        }
    }
#ifndef NDEBUG
	qDebug() << "Local grids extended in " << t.elapsed() << "ms.";
#endif
}

void Simulation::loadSimulationData()
 {
     // Setup environments
     setupBasic3DEnvironment();
     for(int i=0; i<numWorlds; ++i)
         setupBasicPhysicsEnvironment(worlds[i]);

     // Load the entities to simulate
     loadEntities();
}

void Simulation::_init()
{
    // Loads all the simulation data
    loadSimulationData();

    // Create the GridInformation
    setupGridInformation();

    // Launch the EKMeans algorithm to get cluster assignments
    QVector<btVector3> points = getEntitiesPositions();
    computeClusterAssignments(points);

    // Compute the best resolution for territories
    int resolution = getBestTerritoryResolution();

    // Create worlds and local grids, using a global assignments vector for sodaDynamicEntity instances
    setupLocalGrids(resolution, points);

    // Browse all sorted entities to assign Cell owners to non-empty Cells, and return the list of empty ones
    QVector<btVector3> emptyCells = computeCellOwnersAndLocateEmptyCells(resolution);

    // Make sure that whenever a Cell is surrounded by Cells from the same sodaLocalGrid, that sodaLocalGrid owns it
//    assignSurroundedCellsToOwners(emptyCells);

    // Assign the Cells that are still empty, using a Clustering algorithm
    assignEmptyCells(resolution, emptyCells);

    // Assign sodaLocalGrids to their respective sodaLogicWorlds, and setup sodaLocalGrid borders if wanted
    for(int i=0; i<numWorlds; ++i)
    {
        worlds[i]->assignLocalGrid(grids[i]);
//        worlds[i]->drawCells();
        worlds[i]->setupLocalGridBorders();
    }
}
