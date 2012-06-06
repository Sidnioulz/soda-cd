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
#ifndef OBENTITYTRANSFORMRECORDLIST_H
#define OBENTITYTRANSFORMRECORDLIST_H

#include <QList>
#include "obentitytransformrecord.h"

/*! \class obEntityTransformRecordList
  * \brief A wrapper for a list of obEntityTransformRecord objects, associated with a time step.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a wrapper a list of obEntityTransformRecord objects.
  * It allows specifying the position and orientation of entities at a
  * give time step.
  */
class obEntityTransformRecordList : protected QList<obEntityTransformRecord>
{
    friend class obEntityTransformRecordListIterator;

public:
    /*!
      * \brief Default constructor.
      * \param timeStep the reference time step for this list
      * \return a new obEntityTransformRecordList
      */
    inline obEntityTransformRecordList(const btScalar &timeStep) :
        timeStep(timeStep)
    {
    }

    /*!
      * \brief Adds an obEntityTransformRecord to the list.
      * \param record the obEntityTransformRecord to add
      */
    inline void addRecord(const obEntityTransformRecord &record)
    {
        QList::append(record);
    }

    /*!
      * \brief Appends another obEntityTransformRecordList's values to this one.
      * \param other the obEntityTransformRecordList whose values should be appended
      */
    inline void append(const obEntityTransformRecordList &other)
    {
        QList::append(other);
    }

    /*!
      * \brief Adds an obEntityTransformRecord to the list.
      * \param obEnt the entity for which to add a obEntityTransformRecord
      * \param transform the btTransform of the body
      */
    inline void addTransform(obEntityWrapper *obEnt, const btTransform &transform)
    {
        QList::append(obEntityTransformRecord(obEnt, transform));
    }

    /*!
      * \brief Clears the list.
      */
    inline void clear()
    {
        QList::clear();
    }

    /*!
      * \brief Tells whether an obEntityWrapper has an associated value in the list.
      * \param key the obEntityWrapper for which a record is wanted
      * \return true if the key exists in the list, false otherwise
      */
    inline bool contains(obEntityWrapper *key) const
    {
        return QList::contains(obEntityTransformRecord(key));
    }

    /*!
      * \brief Returns the record associated to a given obEntityWrapper, or a NULL one if it doesn't exist.
      * \param key the obEntityWrapper for which the value must be returned
      * \return the value if it was found, or an empty obEntityTransformRecord otherwise
      */
    const obEntityTransformRecord find(obEntityWrapper *key) const;

    /*!
      * \brief Gets the reference time step of this list.
      * \return the btScalar representing the time step of the list
      */
    inline btScalar getTimeStep() const
    {
        return timeStep;
    }


    /*!
      * \brief Sets the reference time step of this list to a new value.
      * param newStep the new time step of this list
      */
    inline void setTimeStep(const btScalar &newStep)
    {
        timeStep = newStep;
    }

private:
        btScalar timeStep;  //!< the time step of the list
};



//TODO: super iterators, that then can be combined with the interface to transparently link lists without any copy
// For instance, a class that creates obETRL iterators for a list of pointers to obETRL given when constructed.
/*! \class obEntityTransformRecordListIterator
  * \brief An iterator on obEntityTransformRecordList objects.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is an iterator for obEntityTransformRecordList objects.
  */
class obEntityTransformRecordListIterator : public QListIterator<obEntityTransformRecord>
{
public:
    /*!
      * \brief Default constructor.
      * \param list the list to iterate over
      * \return a new obEntityTransformRecordListIterator
      */
    inline obEntityTransformRecordListIterator(const obEntityTransformRecordList &list) :
        QListIterator(list)
    {
    }
};

#endif // OBENTITYTRANSFORMRECORDLIST_H
