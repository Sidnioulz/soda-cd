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

CircularTransformBuffer::CircularTransformBuffer() :
    QVector<QSharedPointer<obEntityTransformRecordList> >(CircularTransformBuffer::BufferSize),
    latestPastIndex(0),
    currentPhysicsIndex(0),
	physicsRefTime(0),
    bufferType(DiscreteCollisionDetection),
    bufferNotFull(),
	fullBufferMutex()
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
    while(getEntryState(nextIndex(currentPhysicsIndex)) != CT_STATE_EMPTY)
        bufferNotFull.wait(&fullBufferMutex);

    // Insert the btTransforms computed
    this->operator [](currentPhysicsIndex) = QSharedPointer<obEntityTransformRecordList>(simul);

    // Update the physics reference time
//    if(physicsRefTime == 0)
//        physicsRefTime = simul->getTimeStep();

//    qDebug() << "Included TSTM with time=" << simul->getTimeStep();

    // Increase the physics pointer
    currentPhysicsIndex = nextIndex(currentPhysicsIndex);
}

QSharedPointer<obEntityTransformRecordList> CircularTransformBuffer::processNext(const btScalar &targetTime)
{
    //TODO its here that we return the physics ref time, it's the last time we let a client process (targetTime or timeStep of the returned data depending on the type of Collision Detection)
    //TODO we can then check that the client asks for future times only

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
    qDebug() << "CircularTransformBuffer()::processNext(" << targetTime << "); Simulation late: latest past is " << (at(latestPastIndex).isNull() ? 0 : at(latestPastIndex)->getTimeStep()) << "; Thread " << QString().sprintf("%p", QThread::currentThread());
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

void CircularTransformBuffer::dropPastIndex()
{
//    QSharedPointer<obEntityTransformRecordList> ptr = at(latestPastIndex);
//    if(!ptr.isNull())
//        physicsRefTime = ptr->getTimeStep();
//    else
//        qWarning() << "About to drop a null pointer. This should not be needed.";

    // Insert an empty pointer on the previous cell to mark it available
    this->operator [](latestPastIndex).clear();
    latestPastIndex = nextIndex(latestPastIndex);

    // Wake the physics engine thread if it was waiting for a slot
    bufferNotFull.wakeAll();
}

