#include "experimenttrackinginterface.h"
#include <QtDebug>
#include <QString>



ExperimentTrackingInterface *ExperimentTrackingInterface::getInterface()
{
    static ExperimentTrackingInterface *eti = 0;

    if(!eti)
        eti = new ExperimentTrackingInterface;

    return eti;
}

void ExperimentTrackingInterface::registerSynchronizationEvent(const short &world0, const btScalar &timestamp, const short &world1)
{
    syncGroupsMutex.lock();

    // Get the group for this timestamp
    QList< QList<short> > groupsAtTime = syncGroups.value(timestamp, QList< QList<short> >());

    // Lookup for already existing groups
    int index0 = 0, index1 = 0;
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
    if(index0 != 0 && index1 != 0)
    {
        // Lists not already merged (could happen with duplicate events)
        if(index0 != index1)
        {
            QList<short> senderList = groupsAtTime.at(index0);
            QList<short> syncNeighborList = groupsAtTime.at(index1);

            groupsAtTime.removeAt(index0);
            groupsAtTime.removeAt(index1);

            senderList.append(syncNeighborList);
            groupsAtTime.append(senderList);
        }
    }
    else
    {
        // Only sender's list exists
        if(index0 != 0)
        {
            groupsAtTime[index0].append(world1);
        }
        // Only neighbor's list exists
        else if(index1 != 0)
        {
            groupsAtTime[index1].append(world0);
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

void ExperimentTrackingInterface::printSynchronizationTimeStats()
{
    syncGroupsMutex.lock();

    qDebug() << "\n\n##### SYNCHRONIZATION TIME STATS #####";
    qDebug() << "Number of timestamps registered:" << syncGroups.size();

    QList<btScalar> timestamps = syncGroups.keys();
    QList<QList<QList<short > > > groups = syncGroups.values();





    for(int ts=0; ts<timestamps.size(); ++ts)
    {
        qDebug() << "\n# Timestamp" << timestamps[ts];

        for(int i=0; i<groups[ts].size(); ++i)
        {
            QString line = QString("Group %1 (size %2): ").arg(i).arg(groups[ts][i].size());
            for(int j=0; j<groups[ts][i].size(); ++j)
                line.append(QString("%1, ").arg(groups[ts][i][j]));

            qDebug(qPrintable(line));
        }
    }


    qDebug() << "##### end stats #####\n";
    syncGroupsMutex.unlock();
}


void ExperimentTrackingInterface::clearStats()
{
    syncGroupsMutex.lock();
    syncGroups.clear();
    syncGroupsMutex.unlock();
}


ExperimentTrackingInterface::ExperimentTrackingInterface()
{
}
