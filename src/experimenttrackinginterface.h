#ifndef EXPERIMENTTRACKINGINTERFACE_H
#define EXPERIMENTTRACKINGINTERFACE_H

#include <QMap>
#include <QList>
#include <QMutex>
#include <btBulletDynamicsCommon.h>

class ExperimentTrackingInterface
{
public:
    static ExperimentTrackingInterface *getInterface();

    void registerSynchronizationEvent(const short &senderId, const btScalar &timestamp, const short &syncNeighbor);
    void printSynchronizationTimeStats();

    void clearStats();

private:
    explicit ExperimentTrackingInterface();

    QMap<btScalar, QList< QList<short> > > syncGroups;
    QMutex syncGroupsMutex;
};

#endif // EXPERIMENTTRACKINGINTERFACE_H
