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
    createPhysicsWorlds();
}

Simulation::~Simulation()
{
    for(int i=0; i<numWorlds; ++i)
        delete worlds[i];

    delete bufferInterface;
}


void Simulation::start()
{
    if(status == STOPPED)
    {
        _init();
        status = PAUSED;
    }

    // Resume the Simulation by telling the PhysicsWorlds to run
    if(status == PAUSED)
    {
        // Start the simulations
        //TODO: knowing that Marcel lightthreads will later be used, just unlock a WaitCondition that all threads will be waiting for.
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
        //TODO: pause the PhysicsWorlds and tell the rendering thread to ask for the latest available past

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

void Simulation::createPhysicsWorlds()
{
    // Create the physics worlds
    for(int i=0; i<numWorlds; ++i)
    {
        worlds[i] = new PhysicsWorld(*this, targetTimeStep);
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
        centroids[i] = btVector3(rand() % (int)sceneSize.x() - sceneSize.x() / 2,
                                 rand() % (int)sceneSize.y(),
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

			//FIXME: compute margin using Ceil(Ceil(nbCells/nbProbs) / 20) to have margin >5% of surface on each side?
            //NOTE: Exemple 64Cells, 8Procs, margin=1, Grid=8x8x8, with margin 9x9x9 - Overhead is 36x8 + 100x2, with originally 512 cells, which means 48.8% overhead
//            margin[LocalGrid::Left] =  margin[LocalGrid::Right] = qCeil((float)qCeil((float)grid->getNbCells().x() / numWorlds) / 10);
//            margin[LocalGrid::Top] =  margin[LocalGrid::Bottom] = qCeil((float)qCeil((float)grid->getNbCells().y() / numWorlds) / 10);
//            margin[LocalGrid::Front] =  margin[LocalGrid::Back] = qCeil((float)qCeil((float)grid->getNbCells().z() / numWorlds) / 10);
            QVector<int> margin = computeMargin(resolution, minCoord, maxCoord);
            grids[i] = new LocalGrid(grid->getGridAtResolution(resolution), worlds[i]->getId(), margin, (maxCoord+btVector3(1,1,1)-minCoord), minCoord);
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
	//NOTE: Consider scene borders when computing a margin!
    return QVector<int>(6, 0);
}

void Simulation::sortEntitiesPerCellCoordinates(const int &resolution)
{
    lessThan lt(resolution, grid);
    qSort(entitiesWithAssignments.begin(), entitiesWithAssignments.end(), lt);
}

void Simulation::_setCellOwner(const QMap<short, int> &cellOwnerCounter, QVector<btVector3> &emptyCells, const btVector3 &currentCoords, const QVector<obEntityWrapper *> &cellEnts)
{
    // Find in the owner count for the current Cell which PhysicsWorld owns most entities
    short maxIndex = PhysicsWorld::NullWorldId;
    int maxCount = 0;
    QMapIterator<short, int> it(cellOwnerCounter);
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
    if(maxIndex == PhysicsWorld::NullWorldId)
    {
//        qDebug() << "\tEmpty " << currentCoords.x() << currentCoords.y() << currentCoords.z();
        emptyCells.append(currentCoords);
    }
    // Else, notify all PhysicsWorlds which one the Cell was assigned to
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
    QMap<short, int> cellOwnerCounter;
    QVector<obEntityWrapper *> cellEnts;
    btVector3 currentCoords(grid->toCellCoordinates(resolution, btVector3(-sceneSize.x()/2, 0, -sceneSize.z()/2)));
    for(int i=0; i<entitiesWithAssignments.size(); ++i)
    {
        obEntityWrapper *obEnt = entitiesWithAssignments[i].first;
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

	// Finally, notify Cell assignments to all LocalGrids
	for(int i=0; i<assignments.size(); ++i)
		for(int w=0; w<numWorlds; ++w)
			notifyCellAssignment(grids[w], emptyCells[i], worlds[assignments[i]]->getId());
}

void Simulation::notifyCellAssignment(LocalGrid *local, const btVector3 &coords, const short &owner, const QVector<obEntityWrapper *> *entities)
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

//			qDebug() << "LocalGrid " << i << " being extended to:";
//            qDebug() << minCoord.x() << minCoord.y() << minCoord.z();
//            qDebug() << maxCoord.x() << maxCoord.y() << maxCoord.z();

            QVector<int> margin = computeMargin(resolution, minCoord, maxCoord);
            //FIXME: temporary hack for the extendGrid and resize bug that made some cells unowned.
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

    // Create worlds and local grids, using a global assignments vector for obEntityWrapper instances
    setupLocalGrids(resolution, points);

    // Browse all sorted entities to assign Cell owners to non-empty Cells, and return the list of empty ones
    QVector<btVector3> emptyCells = computeCellOwnersAndLocateEmptyCells(resolution);

//    // Make sure that whenever a Cell is surrounded by Cells from the same LocalGrid, that LocalGrid owns it
    assignSurroundedCellsToOwners(emptyCells);

//	// Assign the Cells that are still empty, using a Clustering algorithm
    assignEmptyCells(resolution, emptyCells);

	// Assign LocalGrids to their respective PhysicsWorlds, and setup LocalGrid borders if wanted
    for(int i=0; i<numWorlds; ++i)
    {
        worlds[i]->assignLocalGrid(grids[i]);
//        worlds[i]->drawCells();
        worlds[i]->setupLocalGridBorders();
    }
}
