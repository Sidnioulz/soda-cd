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
#ifndef CIRCULARTRANSFORMBUFFERINTERFACE_H
#define CIRCULARTRANSFORMBUFFERINTERFACE_H

#include "circulartransformbuffer.h"
#include "obentitytransformrecordlist.h"

/*! \class CircularTransformBufferInterface
  * \brief Implements an interface to use several CircularTransformBuffer instances at the same time.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an interface that queries several watched CircularTransformBuffer instances so that
  * their objects can be queried for the same time unit by a client of the interface.
  *
  * \warning Do not share a CircularTransformBuffer object between several CircularTransformBufferInterface
  * objects and do not have several clients use the same CircularTransformBufferInterface!
  * These classes are not thread-safe.
  */
class CircularTransformBufferInterface : protected QVector<CircularTransformBuffer *>
{
public:
    /*!
      * \brief Default constructor.
      * \return a new CircularTransformBufferInterface
      */
    CircularTransformBufferInterface();

    /*!
      * \brief Adds a buffer to the list managed by this interface.
      * \param buffer the buffer to add to the watch list
      */
    inline void watchBuffer(CircularTransformBuffer *buffer)
    {
        append(buffer);
    }

    /*!
      * \brief Retrieves the positions and rotations of each buffer's objects for a given target time.
      * \param targetTime the time for which transforms are wanted, that will be updated with the actually retrieved time
      * \return a pointer to a list containing the information of all objects within watched buffers
      */
    QSharedPointer<obEntityTransformRecordList> processNext(btScalar &targetTime);

    /*!
      * \brief Tells whether the buffer interface is for continuous or discrete CD.
      * \return the CircularBufferType of this buffer
      *
      * \warning This function just returns the type of the first watched buffer, but no
      * checking is done to ensure that all buffers within an interface are of the same
      * type.
      */
    inline CircularTransformBuffer::BufferType getBufferType() const
    {
        return at(0)->getBufferType();
    }

    /*!
     * \brief Returns the CircularTransformBuffer at a given index.
     * \param i the index for which a CircularTransformBuffer is wanted
     * \return the CircularTransformBuffer pointer stored at index i
     */
    inline CircularTransformBuffer *at(int i) const
    {
        return QVector::at(i);
    }
};

#endif // CIRCULARTRANSFORMBUFFERINTERFACE_H
