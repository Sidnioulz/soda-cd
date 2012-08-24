#include <QtDebug>
#include <math.h>
#include <string.h>
#include <limits>
#include "libekmeans.h"

using namespace std;

namespace Clustering {
EKMeans::EKMeans(const QVector<btVector3> &centroids, const QVector<btVector3> &points) :
    centroids(centroids),
    points(points),
    equal(false),
    fixed(false),
    iteration(12)
{
    if(centroids.size() > 0)
        idealCount=points.size() / centroids.size();
    else
        idealCount=0;

    assignments.fill(-1, points.size());

    changes.fill(true, centroids.size());

    counts.resize(centroids.size());
    dones.resize(centroids.size());
}

QVector<btVector3> EKMeans::getCentroids() const
{
    return centroids;
}

QVector<btVector3> EKMeans::getPoints() const
{
    return points;
}

QVector<int> EKMeans::getAssignments() const
{
    return assignments;
}

bool EKMeans::isEqual() const
{
    return equal;
}

void EKMeans::setEqual(bool equal)
{
    this->equal=equal;
}

bool EKMeans::hasFixedCentroids() const
{
    return fixed;
}

void EKMeans::setFixedCentroids(bool fixed)
{
    this->fixed=fixed;
}

void EKMeans::run()
{
    calculateDistances();
    int move=makeAssignments();
    int i=0;
    while(move > 0 && i++<iteration)
    {
        if(!hasFixedCentroids())
            moveCentroids();

        calculateDistances();
        move=makeAssignments();
    }

    // Final cleanup step for equal-sized clusters
    bool eqTemp=equal;
    equal=false;
    move=makeAssignments();
    equal=eqTemp;
}

void EKMeans::calculateDistances()
{
    for(int c=0; c<centroids.size(); c++)
    {
        if(changes[c])
        {
            const btVector3 &centroid=centroids[c];
            for(int p=0; p<points.size(); p++)
            {
                const btVector3 &point=points[p];
                distances[QPair<int, int>(c, p)]=distanceFunction.distance(centroid, point);
            }

            changes[c]=false;
        }
    }
}

int EKMeans::makeAssignments()
{
    int move=0;
    counts.fill(0);
    for(int p=0; p<points.size(); p++)
    {
        int nc=nearestCentroid(p);
        if(nc == -1)
        {
            qDebug() << "Point " << p << " has no nearest centroid.";
        }
        else
        {
            if(assignments[p] != nc)
            {
                if(assignments[p] != -1)
                {
                    changes[assignments[p]]=true;
                }
                changes[nc]=true;
                assignments[p]=nc;
                move++;
            }

            counts[nc]++;

            if(equal && counts[nc] > idealCount)
            {
                move += remakeAssignments(nc);
            }
        }
    }
    return move;
}

int EKMeans::remakeAssignments(int cc)
{
    int move=0;
    double md=numeric_limits<double>::max();
    int nc=-1;
    int np=-1;
    for(int p=0; p<points.size(); p++)
    {
        if(assignments[p] == cc)
        {
            for(int c=0; c<centroids.size(); c++)
            {
                if(!(c == cc || dones[c]))
                {
                    double d=distances[QPair<int, int>(c, p)];
                    if(d<md)
                    {
                        md=d;
                        nc=c;
                        np=p;
                    }

                }
            }
        }
    }

    if(nc != -1 && np != -1)
    {
        if(assignments[np] != nc)
        {
            if(assignments[np] != -1)
            {
                changes[assignments[np]]=true;
            }
            changes[nc]=true;
            assignments[np]=nc;
            move++;
        }
        counts[cc]--;
        counts[nc]++;
        if(counts[nc] > idealCount)
        {
            dones[cc]=true;
            move += remakeAssignments(nc);
            dones[cc]=false;
        }
    }
    return move;
}

int EKMeans::nearestCentroid(int p)
{
    double md=numeric_limits<double>::max();
    int nc=-1;
    for(int c=0; c<centroids.size(); c++)
    {
        double d=distances[QPair<int, int>(c, p)];
        if(d<md)
        {
            md=d;
            nc=c;
        }
    }
    return nc;
}

void EKMeans::moveCentroids()
{
    for(int c=0; c<centroids.size(); c++)
    {
        if(changes[c])
        {
            btVector3 centroid=centroids[c];
            int n=0;

            centroid = btVector3(0,0,0);

            for(int p=0; p<points.size(); p++)
            {
                if(assignments[p] == c)
                {
                    btVector3 point=points[p];
                    n++;
                    for(int d=0; d<3; d++)
                    {
                        centroid[d] += point[d];
                    }
                }
            }

            if(n > 0)
            {
                for(int d=0; d<3; d++)
                {
                    centroid[d] /= n;
                }
            }

            centroids[c]=centroid;
        }
    }
}

}

