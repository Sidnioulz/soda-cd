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
#include <QtDebug>
#include <QThread>
#include "sodaMeshShapeCache.h"

sodaMeshShapeCache *sodaMeshShapeCache::instance = 0;

sodaMeshShapeCache::sodaMeshShapeCache()
{
}

sodaMeshShapeCache *sodaMeshShapeCache::getInstance()
{
    if(!instance)
        instance = new sodaMeshShapeCache();

    return instance;
}

void sodaMeshShapeCache::deleteInstance()
{
    if(instance)
    {
        delete instance;
        instance = 0;
    }
}

void sodaMeshShapeCache::addShape(const QString &meshName, const QSharedPointer<btCollisionShape *> &shape)
{
    if(!contains(meshName) || value(meshName).data() == shape.data())
        insert(meshName, QWeakPointer<btCollisionShape *>(shape));
    else
        qWarning() << "sodaMeshShapeCache::addShape(" << meshName << ", " << shape.data() << "); A different btCollisionShape already exists for this name; Thread " << QString().sprintf("%p", QThread::currentThread());
}

QSharedPointer<btCollisionShape *> sodaMeshShapeCache::getShape(const QString &meshName)
{
    return value(meshName).toStrongRef();
}
