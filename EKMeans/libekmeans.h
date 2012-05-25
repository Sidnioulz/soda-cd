#ifndef LIBEKMEANS_H
#define LIBEKMEANS_H

#include "EKMeans_global.h"
#include <QVector>
#include <QMap>
#include <QPair>
#include "btBulletDynamicsCommon.h"
#include <obEntityWrapper.h>

/*! \namespace Clustering
  * \brief The namespace for clustering algorithms.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This namespace contains clustering algorithms such as EKMeans.
  */
namespace Clustering {
class EKMEANSSHARED_EXPORT EKMeans {
    struct Listener {
        virtual void iteration(int iteration, int move) = 0;
    };

    struct DistanceFunction {
        virtual btScalar distance(const btVector3 &p1, const  btVector3 &p2) = 0;
    };

    struct EuclidianDistanceFunction : DistanceFunction {
        btScalar distance(const btVector3 &p1, const  btVector3 &p2)
        {
            return p1.distance(p2);
        }
    };

    struct ManhattanDistanceFunction : DistanceFunction {
        btScalar distance(const btVector3 &p1, const  btVector3 &p2)
        {
            return abs(p1.x() - p2.x()) + abs(p1.y() - p2.y()) + abs(p1.z() - p2.z());
        }
    };


public:
    //TODO: parallelize (using Marcel OR TBB)
    EKMeans(const QVector<btVector3> &centroids, const QVector<btVector3> &points);


    QVector<btVector3> getCentroids() const;
    int getCentroidCount() const;
    QVector<btVector3> getPoints() const;
    QHash<QPair<int, int>, btScalar> getDistances() const;
    QVector<int> getAssignments() const;
    QVector<bool> getChanges() const;
    QVector<int> getCounts() const;

    int getIteration() const;
    void setIteration(int iteration);

    bool isEqual() const;
    void setEqual(bool equal);

    bool hasFixedCentroids() const;
    void setFixedCentroids(bool fixed);

    Listener *getListener() const;
    void setListener(Listener *listener);

    void run();

protected:
    QVector<btVector3> centroids;               //!< Centroids
    QVector<btVector3> points;                  //!< Points

    int idealCount;                             //!< Ideal count of points per cluster
    QVector<int> counts;                        //!< Count of points per cluster
    bool equal;                                 //!< Whether to do equal-sized clusters
    bool fixed;                                 //!< Whether centroids are fixed and cannot be changed

    QHash<QPair<int, int>, btScalar> distances; //!< Distances from centroids to points
    QVector<int> assignments;                   //!< Cluster assigned to each point
    QVector<bool> changes;                      //!< Whether a point's cluster was changed
    QVector<bool> dones;                        //!< Whether point is assigned
    int iteration;                              //!< Current iteration number
    ManhattanDistanceFunction distanceFunction; //!< Function to compute the distance between points
    Listener *listener;                         //!< An optional object that listens to the EKMeans progress

    void calculateDistances();
    int makeAssignments();
    int remakeAssignments(int cc);
    int nearestCentroid(int p);
    void moveCentroids();

};
}
#endif // LIBEKMEANS_H
