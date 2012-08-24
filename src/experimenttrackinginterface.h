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
#ifndef EXPERIMENTTRACKINGINTERFACE_H
#define EXPERIMENTTRACKINGINTERFACE_H

#include <QMap>
#include <QList>
#include <QMutex>
#include <QTextStream>
#include <btBulletDynamicsCommon.h>

// Forward declaration
class Simulation;

/*! \class ExperimentTrackingInterface
  * \brief An interface to track statistics about Simulations.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an interface that retrieves data from Simulations, computes
  * that data into statistics and outputs them to a file. At the moment only
  * time spent synchronizing between sodaLogicWorlds is available.
  */
class ExperimentTrackingInterface : public QObject
{
    Q_OBJECT

public:
    /*!
     * \brief Returns the instance of ExperimentTrackingInterface.
     * \return the instance of ExperimentTrackingInterface
     */
    static ExperimentTrackingInterface *getInstance();

    /*!
     * \brief Sets the duration of an experiment before the interface stops recording.
     * \param newLimit the new limit in seconds
     */
    void setSimulationTimeLimit(const btScalar &newLimit);

    /*!
     * \brief Registers a synchronization between two sodaLogicWorlds.
     * \param simulation a reference to the ongoing Simulation
     * \param world0 the id of the first sodaLogicWorld
     * \param world1 the id of the second sodaLogicWorld
     * \param timestamp the moment at which the synchronization occurs
     */
    void registerSynchronizationEvent(const Simulation &simulation, const short &world0, const short &world1, const btScalar &timestamp);

    /*!
     * \brief Outputs to a stream statistics about synchronization.
     * \param simulation a reference to the ongoing Simulation
     * \param out the stream to which the stats should be written
     */
    void printSynchronizationTimeStats(const Simulation &simulation, QTextStream &out);

    /*!
     * \brief Clears current statistics about synchronization.
     */
    void clearStats();

signals:
    /*!
     * \brief Emitted when the time limit has been reached for the application to print stats and then close or to launch another Simulation.
     * \param timestamp the moment of emitting the signal (might be superior to the time limit by a small value)
     */
    void simulationTimeLimitReached(const btScalar &timestamp);

public slots:
    /*!
     * \brief Slot to indicate the interface that a given timestamp has been rendered by the application.
     * \param timeStamp the rendered timestamp
     */
    void onTimestampRendered(const btScalar &timeStamp);

private:
    /*!
      * \brief Default constructor.
      * \return a new ExperimentTrackingInterface
      */
    explicit ExperimentTrackingInterface();

    QMap<btScalar, QList< QList<short> > > syncGroups;      /*!< The map that contains synchronization groups for all timestamps */
    QMutex syncGroupsMutex;                                 /*!< A mutex to avoid concurrent accesses to the syncGroups map */
    btScalar timeLimit;                                     /*!< The time limit before emitting a signal */
};

#endif // EXPERIMENTTRACKINGINTERFACE_H
