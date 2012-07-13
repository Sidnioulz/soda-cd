#include "experimenttrackinginterface.h"
#include <QtDebug>
#include <QString>
#include "main.h"



ExperimentTrackingInterface *ExperimentTrackingInterface::getInterface()
{
    static ExperimentTrackingInterface *eti = 0;

    if(!eti)
        eti = new ExperimentTrackingInterface;

    return eti;
}

void ExperimentTrackingInterface::setSimulationTimeLimit(const btScalar &newLimit)
{
    timeLimit = newLimit;
}

void ExperimentTrackingInterface::registerSynchronizationEvent(const Simulation &simulation, const short &world0, const short &world1, const btScalar &timestamp)
{
    syncGroupsMutex.lock();

    // Get the group for this timestamp
    QList< QList<short> > groupsAtTime = syncGroups.value(timestamp, QList< QList<short> >());

    // Init groupsAtTime
    if(groupsAtTime.isEmpty())
    {
        for(int i=0; i<simulation.getNumWorlds(); ++i)
        {
            QList<short> oneElemList;
            //FIXME: temporary hack because of deadline, should be simulation.getWorldAt(i)->getId() but segfault (probably called before ctor of last world)
            oneElemList.append(i+1);
            groupsAtTime.append(oneElemList);
        }
    }

    // Lookup for already existing groups
    int index0 = -1, index1 = -1;
    for(int i=0; i<groupsAtTime.size(); ++i)
    {
        for(int j=0; j<groupsAtTime[i].size(); ++j)
        {
            if(groupsAtTime[i][j] == world0)
            {
                index0 = i;
            }
            else if(groupsAtTime[i][j] == world1)
            {
                index1 = i;
            }
        }
    }

    // Both items found, merge lists
    if(index0 != -1 && index1 != -1)
    {
        // Lists not already merged (could happen with duplicate events)
        if(index0 != index1)
        {
            QList<short> senderList = groupsAtTime.at(index0);
            QList<short> syncNeighborList = groupsAtTime.at(index1);

            groupsAtTime.removeAt(qMax(index0, index1));
            groupsAtTime.removeAt(qMin(index0, index1));

            senderList.append(syncNeighborList);
            groupsAtTime.append(senderList);
        }
    }
    else
    {
        // Only sender's list exists
        if(index0 != -1)
        {
            QList<short> senderList = groupsAtTime.takeAt(index0);

            senderList.append(world1);

            groupsAtTime.append(senderList);
        }
        // Only neighbor's list exists
        else if(index1 != -1)
        {
            QList<short> syncNeighborList = groupsAtTime.takeAt(index1);

            syncNeighborList.append(world0);

            groupsAtTime.append(syncNeighborList);
        }
        // If no world was already synchronized
        else
        {
            QList<short> newGroups;
            newGroups.append(world0);
            newGroups.append(world1);
            groupsAtTime.append(newGroups);
        }
    }

    // Save changes
    syncGroups.insert(timestamp, groupsAtTime);
    syncGroupsMutex.unlock();
}

void ExperimentTrackingInterface::onTimestampRendered(const btScalar &timeStamp)
{
    // If reached time limit plus a few frames (needed because
    if(timeStamp >= timeLimit)
        emit simulationTimeLimitReached(timeStamp);
}

void ExperimentTrackingInterface::printSynchronizationTimeStats(const Simulation &simulation, QTextStream &out)
{
    syncGroupsMutex.lock();

    QMap<int, int> cptSumOccurPerGroup;
    QMap<short, int> cptSumSyncPerWorld;

    out << "##### SYNCHRONIZATION TIME STATS #####\n";
    out << "Number of timestamps registered: " << syncGroups.size() << "\n";

    QList<btScalar> timestamps = syncGroups.keys();
    QList<QList<QList<short > > > groups = syncGroups.values();

    // Number of timestamps saved
    for(int ts=0; ts<timestamps.size() && timestamps[ts] <= timeLimit+0.001; ++ts)
    {
        out << "\n# Timestamp " << timestamps[ts] << "\n";

        // Number of groups at timestamp ts
        for(int i=0; i<groups[ts].size(); ++i)
        {
            QString line = QString("Group %1 (size %2): ").arg(i).arg(groups[ts][i].size());
            for(int j=0; j<groups[ts][i].size(); ++j)
            {
                line.append(QString("%1, ").arg(groups[ts][i][j]));

                // For each world, compute the average number of sync'd neighbors
                cptSumSyncPerWorld.insert(groups[ts][i][j], cptSumSyncPerWorld.value(groups[ts][i][j], 0) + groups[ts][i].size()-1);

                // Groups of that size have one more occurence
                cptSumOccurPerGroup.insert(groups[ts][i].size(), cptSumOccurPerGroup.value(groups[ts][i].size(), 0) + 1);
            }

            out << qPrintable(line) << "\n";
        }
    }

    out << "\n\n## Stats per world\n";
    for(int w=0; w<simulation.getNumWorlds(); ++w)
    {
        out << "World " << w+1 << ": " << (float)(cptSumSyncPerWorld.value(w+1, 0)) / (float)(groups.size()) << " sync per time step on average\n";
    }


    out << "\n\n## Stats per group size\n";
    int sumTotOccurPerGroup=0;
    for(int i=0; i<cptSumOccurPerGroup.values().size(); ++i)
        sumTotOccurPerGroup+=cptSumOccurPerGroup.values().at(i);

    QMapIterator<int, int> it(cptSumOccurPerGroup);
    while(it.hasNext())
    {
        it.next();

        out << "Groups of size " << it.key() << " represent " << it.value() << " occurences.\n On average " << (float)100*it.value() / (float)sumTotOccurPerGroup << "% of occurences in this group.\n";
    }

    out << "\n##### end stats #####\n\n";
    syncGroupsMutex.unlock();
}


void ExperimentTrackingInterface::clearStats()
{
    syncGroupsMutex.lock();
    syncGroups.clear();
    syncGroupsMutex.unlock();
}


ExperimentTrackingInterface::ExperimentTrackingInterface() :
    timeLimit(INFINITY)
{
}
