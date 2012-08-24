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
#include "circulartransformbuffer.h"
#include "sodaLogicWorld.h"

CircularTransformBuffer::CircularTransformBuffer(sodaLogicWorld *world) :
    QVector<QSharedPointer<obEntityTransformRecordList> >(CircularTransformBuffer::BufferSize),
    latestPastIndex(0),
    currentPhysicsIndex(0),
    bufferType(DiscreteCollisionDetection),
    bufferNotFull(),
    fullBufferMutex(),
    writeMutex(),
    writeAborted(false),
    world(world)
{
}

CircularTransformBuffer::~CircularTransformBuffer()
{
    // Nothing to free, since all shared pointers will be deleted
}

void CircularTransformBuffer::initialize(obEntityTransformRecordList *initialPositions)
{
    this->clear();
    this->resize(CircularTransformBuffer::BufferSize);
    latestPastIndex = 0;
    currentPhysicsIndex = 0;

    appendTimeStep(initialPositions);
}

void CircularTransformBuffer::appendTimeStep(obEntityTransformRecordList *simul)
{
    // We don't really care about this mutex, just make sure it's locked
    // in case we have to wait on the QWaitCondition below.
    fullBufferMutex.tryLock();

    // Make sure that next cell is empty, otherwise wait till it is
    while(getEntryState(nextIndex(currentPhysicsIndex)) != CT_STATE_EMPTY && !writeAborted)
    {
#ifndef NDEBUG
        qDebug() << "CircularTransformBuffer()::appendTimeStep(" << simul->getTimeStep() << "); About to wait; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        bufferNotFull.wait(&fullBufferMutex);
    }

    // Insert the btTransforms computed
    writeMutex.lock();
    if(!writeAborted)
    {
#ifndef NDEBUG
        qDebug() << "CircularTransformBuffer()::appendTimeStep(" << simul->getTimeStep() << "); Writing time step; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        this->operator [](currentPhysicsIndex) = QSharedPointer<obEntityTransformRecordList>(simul);

        // Increase the physics pointer
        currentPhysicsIndex = nextIndex(currentPhysicsIndex);
    }
#ifndef NDEBUG
    else
        qDebug() << "CircularTransformBuffer()::appendTimeStep(" << simul->getTimeStep() << "); Writes are aborted; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    writeMutex.unlock();
}

QSharedPointer<obEntityTransformRecordList> CircularTransformBuffer::processNext(const btScalar &targetTime)
{
    // Continuous collision detection, find closest future positions
//    if(bufferType == ContinuousCollisionDetection)
    {
        // Find closest, already rendered future
        while(getEntryState(nextIndex(latestPastIndex)) != CT_STATE_EMPTY)
        {
            // If still too far behind, move the rendering index
            if(targetTime >= at(nextIndex(latestPastIndex))->getTimeStep())
                 dropPastIndex();
            else
                return at(nextIndex(latestPastIndex));
        }

        // If arriving here, then target time is ahead of the latest available positions
        // Or, the cell next to latestPastIndex is being filled by the physics thread
#ifndef NDEBUG
    //qDebug() << "CircularTransformBuffer()::processNext(" << targetTime << "); Simulation late: latest past is " << (at(latestPastIndex).isNull() ? 0 : at(latestPastIndex)->getTimeStep()) << "; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif
        return at(latestPastIndex);
    }

    // Discrete collision detection, find closest previously computed positions
//    else
//    {
//        // Find closest pas rendered
//        while(getEntryState(nextIndex(latestPastIndex)) != CT_STATE_EMPTY)
//        {
//            // If still too far behind, move the rendering index
//            if(targetTime >= at(nextIndex(latestPastIndex))->getTimeStep())
//                dropPastIndex();
//            else
//            {
////                qDebug("Returned closest past and future is ready for rendering.");
//                return at(latestPastIndex);
//            }

//        }

//        // If arriving here, we already are at the latest available past
////        qDebug("Returned closest past because there was no other choice.");
//        return at(latestPastIndex);
//    }
}

btScalar CircularTransformBuffer::getClosestAvailableTime(const btScalar &targetTime)
{
    btScalar browseIndex = latestPastIndex;

    // Find closest, already rendered future
    while(getEntryState(nextIndex(browseIndex)) != CT_STATE_EMPTY)
    {
        // If still too far behind, move the rendering index
        if(targetTime < at(nextIndex(browseIndex))->getTimeStep())
            return at(nextIndex(browseIndex))->getTimeStep();

        browseIndex = nextIndex(browseIndex);
    }

    // No future entry, use latest past
    if(getEntryState(browseIndex) != CT_STATE_EMPTY)
        return at(browseIndex)->getTimeStep();
    // Case when there is no entry: just ignore this buffer
    else
        return targetTime;
}

void CircularTransformBuffer::abortAllWrites()
{
    writeMutex.lock();

    // Forbid future writes
    writeAborted = true;

    // Make sure to exit any currently writing function
    bufferNotFull.wakeAll();

    writeMutex.unlock();
}

void CircularTransformBuffer::dropPastIndex()
{
    // Insert an empty pointer on the previous cell to mark it available
    this->operator [](latestPastIndex).clear();
    latestPastIndex = nextIndex(latestPastIndex);

    // Wake the physics engine thread if it was waiting for a slot
    bufferNotFull.wakeAll();
}

