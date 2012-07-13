#ifndef EXPERIMENTTRACKINGINTERFACE_H
#define EXPERIMENTTRACKINGINTERFACE_H

#include <QMap>
#include <QList>
#include <QMutex>
#include <QTextStream>
#include <btBulletDynamicsCommon.h>

// Forward declaration
class Simulation;

class ExperimentTrackingInterface : public QObject
{
    Q_OBJECT

public:
    static ExperimentTrackingInterface *getInterface();

    void setSimulationTimeLimit(const btScalar &newLimit);

    void registerSynchronizationEvent(const Simulation &simulation, const short &world0, const short &world1, const btScalar &timestamp);
    void printSynchronizationTimeStats(const Simulation &simulation, QTextStream &out);

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
