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
#ifndef OGREWIDGET_H
#define OGREWIDGET_H

#include <QtGui>
#include <QGLWidget>
#include <QTimer>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgrePlugin.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>

#include "mySdkTrays.h"
#include "circulartransformbuffer.h"
#include "circulartransformbufferinterface.h"

//TODO: replace ogre root by OgreResources::root, same for scene manager

/*! \class OgreWidget
  * \brief Displays an Ogre 3D scene within a QGL Widget for Qt GUI applications.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements an Ogre 3D scene within a QGL Widget, for use within Qt applications.
  * It is not actively animated, and the scene is not to be populated within this class. Instead,
  * this class can be used as a passive display scene for a SimulationWorld.
  */
class OgreWidget : public QGLWidget,
                public Ogre::FrameListener, public Ogre::WindowEventListener,
		public OgreBites::SdkTrayListener
{
    Q_OBJECT

public:
    /*!
      * \brief Default constructor.
      * \param targetFPS the wanted max FPS for the simulation
      * \param parent the parent widget, if known
      * \return a new OgreWidget
      */
        OgreWidget(/*CircularTransformBufferInterface *bufferInterface, */int targetFPS, QWidget *parent = 0);

    /*!
      * \brief Destructor.
      */
    ~OgreWidget();

    /*!
      * \brief Changes the position of the camera, and updates.
      * \param pos the new position of the camera
      */
    void setCameraPosition(const Ogre::Vector3 &pos);

    /*!
      * \brief Returns the scene manager used by Ogre, for integration within a SimulationScene.
      * \return the current Ogre SceneManager
      */
    Ogre::SceneManager* getSceneManager() const;

    /*!
      * \brief Returns the root used by Ogre, for integration within a SimulationScene.
      * \return the current Ogre Root
      */
    Ogre::Root* getRoot() const;

    /*!
      * \brief Returns the time step that matches the target FPS of the widget in milliseconds.
      * \return one second divided by the target FPS
      */
    btScalar getTargetTimeStep() const;

    /*!
      * \brief Returns the target FPS of the engine.
      * \return the current target FPS
      */
    int getTargetFPS() const;

    /*!
      * \brief Changes the target maximum FPS rate of the engine.
      * \param newFPS the new FPS that must be reached
      */
    void setTargetFPS(const int newFPS);

    /*!
      * \brief Setups basic rendering-related scene elements such as sky and fog.
      * \param xLimit the left-right size of the environment
      * \param zLimit the front-back size of the environment
      */
    void setupBaseScene(const int xLimit, const int zLimit);

    /*!
     * \brief Registers a new CircularTransformBufferInterface for this OgreWidget to read.
     * \param inter the CircularTransformBufferInterface that this OgreWidget should now use
     */
    inline void registerBufferInterface(CircularTransformBufferInterface *inter)
    {
        bufferInterface = inter;
    }

    /*!
     * \brief Removes any registered CircularTransformBufferInterface for this OgreWidget.
     */
    inline void clearBufferInterface()
    {
        bufferInterface = 0;
    }

    /*!
     * \brief Tells whether a CircularTransformBufferInterface was registered for this OgreWidget.
     * \return true if the OgreWidget has a CircularTransformBufferInterface to read, false otherwise
     */
    inline bool hasBufferInterface() const
    {
        return bufferInterface != 0;
    }

private:
    /*!
      * \brief Initializes the Ogre system, scene manager, camera, viewport, etc.
      *
      * This function initializes the Ogre system, loads the plugins and renderers used
      * by Ogre, creates the scene manager, default viewport, render window and camera,
      *
      */
    void initOgreSystem();

    /*!
      * \brief Setups and loads various resources later used by Ogre and Bullet.
      */
    void setupLoadResources();


protected:
    //! Initializes the Ogre system when the widget needs to be displayed.
    virtual void showEvent(QShowEvent *e);

    //! Tells Ogre to render a new frame.
    virtual void paintEvent(QPaintEvent *e);

    //! Tells the Ogre viewport it has been moved.
    virtual void moveEvent(QMoveEvent *e);

    //! Tells the Ogre viewport it has been resized.
    virtual void resizeEvent(QResizeEvent *e);

    /*!
      * \brief Handler for key press events.
      * \param e the QKeyEvent
      *
      * Handler for key press events.
      * Key pressed     Effect
      * Z               Move fordwards
      * S               Move backwards
      * Q               Move to left
      * D               Move to right
      * Page Up         Move up
      * Page Down       Move down
      */
    virtual void keyPressEvent(QKeyEvent *e);

    /*!
      * \brief Called when the next Ogre frame is about to be rendered, to update SdkTrays.
      * \param evt the Ogre frame event that is being rendered
      * \return true to continue the simulation, false to exit
      */
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    /*!
      * \brief Performs one rendering pass.
      *
      * \todo optimize
      */
    virtual void render();

    // Ogre variables
    Ogre::Root                       *ogreRoot;             //!< Root object of the Ogre engine
    Ogre::SceneManager               *ogreSceneManager;     //!< Object that manages the scene in which entities are rendered
    Ogre::RenderWindow               *ogreRenderWindow;     //!< The displayed and rendered window
    Ogre::Viewport                   *ogreViewport;         //!< An object that defines a point of view of the scene, rendered in the window
    Ogre::Camera                     *ogreCamera;           //!< The camera to navigate through a defined viewport

    // Tray manager
    OgreBites::SdkTrayManager        *mTrayMgr;             //!< An object that manages the panel with rendering information
    OgreBites::ParamsPanel           *mDetailsPanel;        //!< The panel with rendering details

    // Rendering variables
    CircularTransformBufferInterface *bufferInterface;      //!< The interface to read buffers that contain positions to render
    int                              targetFPS;             //!< Maximum FPS that the Ogre engine should reach
    QTimer                           renderingTimer;        //!< Emits ticks when rendering is needed to reach FPS
    QElapsedTimer                    globalTimer;           //!< Global timer that helps measuring the time actually spent (for statistics)
    bool                             currentlyRendering;    //!< Tells whether the rendering function is currently running
    bool                             renderingPassQueued;   //!< Tells whether a tick was emitted during a pass, which means another one is needed to keep up

signals:
    /*!
      * \brief  Indicates that the Ogre system is fully initialized
      */
    void ogreInitialized();

protected slots:
    /*!
      * \brief Calls the rendering function if not already running every time a tick occurs.
      */
    void onTimerTick();

    void onEntityDeletion(const Ogre::Entity *&ent);
    void onSceneNodeDeletion(const Ogre::SceneNode *&node);
};

#endif // OGREWIDGET_H
