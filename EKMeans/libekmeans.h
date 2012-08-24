#ifndef LIBEKMEANS_H
#define LIBEKMEANS_H

#include "EKMeans_global.h"
#include <QVector>
#include <QPair>
#include "btBulletDynamicsCommon.h"
#include <sodaDynamicEntity.h>

/*! \namespace Clustering
  * \brief The namespace for clustering algorithms.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This namespace contains clustering algorithms such as EKMeans.
  */
namespace Clustering {

/*! \class EKMeans
  * \brief A poor sequential implementation of equal-size EKMeans.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an EKMeans implementation used to distribute simulated entities
  * into as many equally-sized clusters as there are worlds in a Simulation.
  */
class EKMEANSSHARED_EXPORT EKMeans {

    /*!
     * \brief Virtual distance function with a distance() method used by
     * the EKMeans class.
     */
    struct DistanceFunction {
        /*!
         * \brief Computes the distance between two points.
         * \param p1 the first point
         * \param p2 the second point
         * \return the distance between them
         */
        virtual btScalar distance(const btVector3 &p1, const  btVector3 &p2) = 0;
    };

    /*!
     * \brief Implementation of the Euclidian distance between two vectors.
     */
    struct EuclidianDistanceFunction : DistanceFunction {
        /*!
         * \brief Computes the Euclidian distance between two points.
         * \param p1 the first point
         * \param p2 the second point
         * \return the distance between them
         */
        btScalar distance(const btVector3 &p1, const  btVector3 &p2)
        {
            return p1.distance(p2);
        }
    };

    /*!
     * \brief Implementation of the Manhattan distance between two vectors.
     */
    struct ManhattanDistanceFunction : DistanceFunction {
        /*!
         * \brief Computes the Manhattan distance between two points.
         * \param p1 the first point
         * \param p2 the second point
         * \return the distance between them
         */
        btScalar distance(const btVector3 &p1, const  btVector3 &p2)
        {
            return abs(p1.x() - p2.x()) + abs(p1.y() - p2.y()) + abs(p1.z() - p2.z());
        }
    };


public:
    /*!
     * \brief Default constructor.
     * \param centroids initial cluster centroids
     * \param points points to clusterize
     * \return a new EKMeans
     */
    EKMeans(const QVector<btVector3> &centroids, const QVector<btVector3> &points);

    /*!
     * \brief Returns the current centroids of each cluster.
     * \return a QVector of centroids represented as 3D points
     */
    QVector<btVector3> getCentroids() const;

    /*!
     * \brief Returns the points in this EKMeans.
     * \return a QVector of 3D points in the EKMeans
     */
    QVector<btVector3> getPoints() const;

    /*!
     * \brief Returns the points in this EKMeans.
     * \return a QVector of 3D points in the EKMeans
     */
    QVector<int> getAssignments() const;

    /*!
     * \brief Tells whether this EKMeans creates equal-size clusters.
     * \return whether clusters produced by this EKMeans are equally-sized
     */
    bool isEqual() const;

    /*!
     * \brief Sets whether this EKMeans creates equal-size clusters.
     * \param equal whether clusters produced by this EKMeans are equally-sized
     */
    void setEqual(bool equal);

    /*!
     * \brief Tells whether centroids can still be adjusted to better match clusters.
     * \return whether centroids can still be adjusted to better match clusters
     */
    bool hasFixedCentroids() const;

    /*!
     * \brief Sets whether centroids can still be adjusted to better match clusters.
     * \param fixed whether centroids are fixed and cannot be modified anymore
     */
    void setFixedCentroids(bool fixed);

    /*!
     * \brief Runs the cluster algorithm iteratively.
     */
    void run();

protected:
    QVector<btVector3>                  centroids;          //!< Centroids of all computed clusters
    QVector<btVector3>                  points;             //!< Points within the EKMeans

    int                                 idealCount;         //!< Ideal count of points per cluster
    QVector<int>                        counts;             //!< Count of points per cluster
    bool                                equal;              //!< Whether to do equal-sized clusters
    bool                                fixed;              //!< Whether centroids are fixed and cannot be changed

    QHash<QPair<int, int>, btScalar>    distances;          //!< Distances from centroids to points
    QVector<int>                        assignments;        //!< Cluster assigned to each point
    QVector<bool>                       changes;            //!< Whether a point's cluster was changed
    QVector<bool>                       dones;              //!< Whether point is assigned
    int                                 iteration;          //!< Current iteration number
    ManhattanDistanceFunction           distanceFunction;   //!< Function to compute the distance between points

    /*!
     * \brief Computes distances between all points and all centroids.
     */
    void calculateDistances();

    /*!
     * \brief Assigns points to their closest centroids.
     * \return the number of assignment changes performed
     */
    int makeAssignments();

    /*!
     * \brief Modifies assignments to respect the equal-size constraint.
     * \param cc the index in centroids vector of the oversized cluster
     * \return the number of assignment changes performed
     */
    int remakeAssignments(int cc);

    /*!
     * \brief Returns the nearest centroid for a given point.
     * \param p the queried point
     * \return p's nearest centroid
     */
    int nearestCentroid(int p);

    /*!
     * \brief Updates centroids to make them match the center of their cluster.
     */
    void moveCentroids();
};
}
#endif // LIBEKMEANS_H
