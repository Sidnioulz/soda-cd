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
#ifndef GARBAGEWORLD_H
#define GARBAGEWORLD_H

#include <QObject>
#include "physicsworld.h"

// Forward reference
class Simulation;

/*! \class GarbageWorld
  * \brief A garbage world that retrieves obEntityWrappers abandoned by their worlds.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class is a utility to which can be sent obEntityWrapper instances that
  * are not needed anymore in their home PhysicsWorld.
  */
class GarbageWorld : public QObject
{
    Q_OBJECT
public:
	/*!
	  * \brief Default constructor.
	  * \param simulation the Simulation this world belongs to
	  * \param parent the parent of this QObject
	  * \return a new GarbageWorld
	  */
	explicit GarbageWorld(const Simulation &simulation, QObject *parent = 0);

	/*!
	  * \brief Default destructor.
	  */
	virtual ~GarbageWorld();

public slots:
	void onOwnershipTransfer(const PhysicsWorld *&neighbor, const obEntityWrapper *&object, const btScalar &time);

protected:
	const Simulation          &simulation;               /*!< the Simulation this world belongs to */
	QVector<obEntityWrapper*> entities;                  //!< a vector for the existing rigid bodies
};

#endif // GARBAGEWORLD_H
