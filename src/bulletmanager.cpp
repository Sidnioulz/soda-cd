/*
 *** Some methods of these classes are derived from Bullet Physics:
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2011 Erwin Coumans  http://bulletphysics.org
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *** The code that differs from the original source is licensed as is:
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

#include "bulletmanager.h"

BulletManager::BulletManager(sodaLogicWorld *world) :
    sodaWorld(world),
    broadphase(new sodaLocalGridBroadphase(world)),
    collisionConfig(new btDefaultCollisionConfiguration()),
    dispatcher(new btCollisionDispatcher(collisionConfig)),
    solver(new btSequentialImpulseConstraintSolver),
    dynamicsWorld(new sodaDynamicsWorld(world, dispatcher, broadphase, solver, collisionConfig))
{
    // Set the gravity orientation of the world (here 'y' is upwards)
    dynamicsWorld->setGravity(btVector3(0,-10,0));
}

BulletManager::~BulletManager()
{
    // Cleanup world
    delete dynamicsWorld;
    delete solver;
    delete dispatcher;
    delete collisionConfig;
    delete broadphase;
}
