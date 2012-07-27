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
#include <QtDebug>
#include "circulartransformbufferinterface.h"
#include "physicsworld.h"

CircularTransformBufferInterface::CircularTransformBufferInterface() :
    QVector<CircularTransformBuffer *>()
{
}

QSharedPointer<obEntityTransformRecordList> CircularTransformBufferInterface::processNext(btScalar &targetTime)
{
//    qDebug() << "processNext(" << targetTime <<");";
    btScalar originalTargetTime = targetTime;

    // Update the target time with what's actually possible to get in all worlds
    for(int i=0; i<size(); ++i)
        targetTime = qMin(targetTime, at(i)->getClosestAvailableTime(targetTime));

    // Simulation is late
    if(originalTargetTime > targetTime)
    {
        //qDebug() << "CircularTransformBufferInterface::processNext(" << originalTargetTime << " ); Simulation is late on rendering (can provide only " << targetTime << "); Thread " << QString().sprintf("%p", QThread::currentThread());
    }

    // A map that contains the merged buffer entries. This has to be changed to something more efficient.
    //TODO: mergeMap should be replaced with a list of QSharedPointers to all worlds' maps with a special iterator to browse it transparently
    QSharedPointer<obEntityTransformRecordList> mergeMap = QSharedPointer<obEntityTransformRecordList>(new obEntityTransformRecordList(targetTime));

    // Actually perform querying
    for(int i=0; i<size(); ++i)
    {
        QSharedPointer<obEntityTransformRecordList> ptr = at(i)->processNext(targetTime);
        if(ptr)
            mergeMap->append(*ptr);
    }

    return mergeMap;
}

