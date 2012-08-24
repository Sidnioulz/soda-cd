/*
 * Copyright (2012) Inria Rennes - IRISA
 *
 * Contributor(s):
 *  Quentin Avril <quentin.avril@irisa.fr>
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
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#ifndef SODAMESHSHAPECACHE_H
#define SODAMESHSHAPECACHE_H

#include <QHash>
#include <QMutex>
#include <QSharedPointer>
#include <QString>
#include <btBulletCollisionCommon.h>

//TODO: make a btShape cache that stores and reuses btShapes per name and scale, in order to save memory

/*! \class sodaMeshShapeCache
  * \brief A cache that allows mutualizing collision shapes per mesh name. Not used yet.
  * \author Quentin Avril <quentin.avril@irisa.fr>
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * \warning The implementation of this class is not finished and thus it is not being used yet.
  */
class sodaMeshShapeCache : protected QHash<QString, QWeakPointer<btCollisionShape *> >
{
protected:
    /*!
      * \brief Default constructor.
      * \return a new sodaMeshShapeCache
      */
    explicit sodaMeshShapeCache();

public:
    /*!
     * \brief Returns the instance of sodaMeshShapeCache.
     * \return the instance of sodaMeshShapeCache
     */
    static sodaMeshShapeCache *getInstance();

    /*!
     * \brief Deletes the instance of sodaMeshShapeCache.
     */
    static void deleteInstance();

    /*!
     * \brief Adds a btCollisionShape to the sodaMeshShapeCache.
     * \param meshName the mesh that this shape corresponds to
     * \param shape pointer to the actual collision shape
     */
    void addShape(const QString &meshName, const QSharedPointer<btCollisionShape *> &shape);

    /*!
     * \brief Retrieves a btCollisionShape corresponding to a name in the cache.
     * \param meshName the name of the wanted shape's mesh
     * \return a QSharedPointer to the btCollisionShape if it exists, a null QSharedPointer otherwise
     */
    QSharedPointer<btCollisionShape *> getShape(const QString &meshName);

private:
    static sodaMeshShapeCache      *instance;       //!< The unique instance of sodaMeshShapeCache
};

#endif // SODAMESHSHAPECACHE_H
