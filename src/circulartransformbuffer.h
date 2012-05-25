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
#ifndef CIRCULARTRANSFORMBUFFER_H
#define CIRCULARTRANSFORMBUFFER_H

#include <QVector>
#include <QMutex>
#include <QWaitCondition>
#include <QSharedPointer>
#include "obentitytransformrecordlist.h"

//TODO: test load bursts of half buffer size, and then of twice buffer size

/*! \class CircularTransformBuffer
  * \brief Implements a circular buffer containing information on transforms for all objects at given time steps.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements a circular buffer with a fixed size, in which every
  * element is a shared pointer to a structure containing a btTransform for each
  * object of the scene.
  *
  * This buffer is to be used with a single writer, and writes may block if the
  * buffer is full. There should be only one reader, and data queries are done by
  * giving a desired time step to the buffer. Reading is non-blocking, and returns
  * the closest entry in time. However, this entry may be old if no writer provides
  * the buffer with fresh btTransform information.
  *
  * \warning Do not share a CircularTransformBuffer object between
  * two writer or reader threads! This class is not thread-safe.
  */
class CircularTransformBuffer : protected QVector<QSharedPointer<obEntityTransformRecordList> >
{
public:
    /*!
      * \brief Default constructor.
      * \return a new CircularTransformBuffer
      */
    CircularTransformBuffer();

    /*!
      * \brief Default destructor.
      */
    ~CircularTransformBuffer();

    /*!
      * \brief Initializes the buffer with the given object positions and time step.
      * \param initialPositions the initial obEntityTransformRecordList of the simulation
      */
    void initialize(obEntityTransformRecordList *initialPositions);

    /*! \enum __CircularTransformEntryState
      * \brief Represents the state of a buffer entry.
      */
    typedef enum __CircularTransformEntryState {
        CT_STATE_EMPTY=0,            //!< This entry contains no obEntityTransformRecord information.
        CT_STATE_READY_RENDER=1      //!< This entry has been filled by the physics engine.
    } CircularTransformEntryState;   //!< the current state of the buffer entry

    /*! \enum __CircularBufferType
      * \brief The type of collision detection for which this buffer is used.
      */
    typedef enum __CircularBufferType {
        DiscreteCollisionDetection=0,   //!< Discrete, stepped collision detection.
        ContinuousCollisionDetection=1  //!< Continuous, TOI-based collision detection.
    } BufferType;                       //!< the type of this CircularTransformBuffer instance

    //! The size of circular buffers. This class is not designed so you can change the buffer size during execution.
    const static int BufferSize = 50;

    /*!
      * \brief Gets the state of the entry at the given index (empty or ready to use).
      * \param index the index of the entry to query
      * \return the CircularTransformEntryState of the queried entry
      */
    inline CircularTransformEntryState getEntryState(int index) const
    {
        if(at(index).isNull())
           return CT_STATE_EMPTY;
        else
            return CT_STATE_READY_RENDER;
    }

    /*!
      * \brief Tells whether the buffer is for continuous or discrete CD.
      * \return the CircularBufferType of this buffer
      */
    inline BufferType getBufferType() const
    {
        return bufferType;
    }

    /*!
      * \brief Returns the reference time (oldest available time step) of this buffer.
      * \return the oldest timeStep of all buffer entries
      */
    inline btScalar getPhysicsRefTime() const
    {
        return physicsRefTime;
    }

    /*!
      * \brief Appends a obEntityTransformRecordList to the buffer.
      * \param simul the data to append
      */
    void appendTimeStep(obEntityTransformRecordList *simul);

    /*!
      * \brief Returns the obEntityTransformRecordList closest to the target time of the caller.
      * \param targetTime the time step for which obEntityTransformRecord objects are wanted.
      * \return the closest obEntityTransformRecord data available
      *
      * \warning When using continuous collision detection, you should not
      * forget to interpolate the returned obEntityTransformRecord with the target time,
      * since it may represent positions in a far away future.
      */
    QSharedPointer<obEntityTransformRecordList> processNext(const btScalar &targetTime);


private:
    /*!
      * \brief Increases the latest past index and frees the previous cell.
      */
    void dropPastIndex();

    /*!
      * \brief Returns the index of the cell following the parameter one.
      * \param index the index for which the next one is wanted
      * \return the index after the one passed as a parameter
      *
      * \warning The index of the cell after the last one is 0, the beginning
      * of the buffer. Call this function instead of managing that yourself,
      * for code clarity.
      */
    static inline int nextIndex(const int index)
    {
        return (index+1) % BufferSize;
    }

    int latestPastIndex;                    //!< Index of the latest data used for rendering
    int currentPhysicsIndex;                //!< Index of the place where to write new data
    btScalar physicsRefTime;                //!< Oldest time step for which the physics engine provided data

    const BufferType bufferType;            //!< Whether the buffer is for continuous or discrete CD

    QWaitCondition bufferNotFull;           //!< An object to wait for "buffer not full" signals
    QMutex fullBufferMutex;                 //!< A mutex for internal use within the wait condition
};

#endif // CIRCULARTRANSFORMBUFFER_H
