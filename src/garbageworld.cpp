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
#include "garbageworld.h"
#include "simulation.h"

GarbageWorld::GarbageWorld(const Simulation &simulation, QObject *parent) :
	QObject(parent),
	simulation(simulation),
	entities()
{
}

GarbageWorld::~GarbageWorld()
{
}

void GarbageWorld::onOwnershipTransfer(const PhysicsWorld *&neighbor, const obEntityWrapper *&object, const btScalar &time)
{
#ifndef NDEBUG
	qDebug() << "GarbageWorld()::onOwnershipTransfer(" << neighbor->getId() << ", " << object->getDisplayName() << ", " << time << "); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif


}



