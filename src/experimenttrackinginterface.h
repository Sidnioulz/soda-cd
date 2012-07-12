#ifndef EXPERIMENTTRACKINGINTERFACE_H
#define EXPERIMENTTRACKINGINTERFACE_H

#include <QMap>
#include <QList>
#include <QMutex>
#include <QTextStream>
#include <btBulletDynamicsCommon.h>

class ExperimentTrackingInterface : public QObject
{
    Q_OBJECT

public:
    static ExperimentTrackingInterface *getInterface();

    void setSimulationTimeLimit(const btScalar &newLimit);

    void registerSynchronizationEvent(const short &senderId, const btScalar &timestamp, const short &syncNeighbor);
    void printSynchronizationTimeStats(QTextStream &out);

    void clearStats();

signals:
    void simulationTimeLimitReached(const btScalar &timestamp);

public slots:
    void onTimestampRendered(const btScalar &timeStamp);

private:
    explicit ExperimentTrackingInterface();

    QMap<btScalar, QList< QList<short> > > syncGroups;
    QMutex syncGroupsMutex;
    btScalar timeLimit;
};

#endif // EXPERIMENTTRACKINGINTERFACE_H
