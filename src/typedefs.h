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
#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <QtCore>
#include <QList>
#include <QHash>
#include <QVector>
#include <btBulletCollisionCommon.h>
#include <Ogre.h>

class CellBorderCoordinates;
class sodaLogicWorld;
class sodaDynamicEntity;

//! A map of sodaDynamicEntity objects associated to a set of border coordinates.
typedef QHash<sodaDynamicEntity *, QVector<CellBorderCoordinates> > EntityOverlappedCellsMap;

//! A hashmap of sodaLogicWorld ids associated to EntityOverlappedCellsMap objects.
typedef QHash<short, EntityOverlappedCellsMap> EntityOverlappedCellsPerWorld;

//! A sodaDynamicEntity instance associated with a time unit that represents a temporal event.
typedef QPair<sodaDynamicEntity *, btScalar> TimedEntity;

//! Queue of TimedEntity instances.
typedef QQueue<TimedEntity > TimedEntityQueue;

// IPC type declarations
Q_DECLARE_METATYPE(sodaLogicWorld *)
Q_DECLARE_METATYPE(EntityOverlappedCellsMap)
Q_DECLARE_METATYPE(EntityOverlappedCellsPerWorld)
Q_DECLARE_METATYPE(sodaDynamicEntity *)
Q_DECLARE_METATYPE(btScalar)
Q_DECLARE_METATYPE(Ogre::Entity *)
Q_DECLARE_METATYPE(Ogre::SceneNode *)
Q_DECLARE_METATYPE(QList<short>)


#endif // TYPEDEFS_H
