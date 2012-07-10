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
#include <QX11Info>
#include <QtDebug>
#include "ogrewidget.h"
#include "ogreresources.h"
#include "utils.h"

OgreWidget::OgreWidget(int targetFPS, QWidget *parent) :
    QGLWidget(parent),
    ogreRoot(0),
    ogreSceneManager(0),
    ogreRenderWindow(0),
    ogreViewport(0),
    ogreCamera(0),
    mTrayMgr(0),
    mDetailsPanel(0),
    bufferInterface(0),
    targetFPS(targetFPS),
    currentlyRendering(false),
    renderingPassQueued(false)
{
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_PaintOnScreen);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(440,240);

    renderingTimer.setInterval(1000*getTargetTimeStep());
    renderingTimer.start();
    connect(&renderingTimer, SIGNAL(timeout()), this, SLOT(onTimerTick()));
}

OgreWidget::~OgreWidget()
{
    if(mTrayMgr)
        delete mTrayMgr;
    windowClosed(ogreRenderWindow);

    OgreResources::deleteRoot();
}

Ogre::SceneManager* OgreWidget::getSceneManager() const
{
    return ogreSceneManager;
}

Ogre::Root* OgreWidget::getRoot() const
{
    return ogreRoot;
}

btScalar OgreWidget::getTargetTimeStep() const
{
    return 1.0f/targetFPS;
}

int OgreWidget::getTargetFPS() const
{
    return targetFPS;
}

void OgreWidget::setTargetFPS(const int newFPS)
{
    qWarning() << "Changing the FPS of the application has not been tested yet.";
    targetFPS = newFPS;
}

void OgreWidget::showEvent(QShowEvent *e)
{
if(!ogreRoot)
    {
        initOgreSystem();
    }

QWidget::showEvent(e);
}

void OgreWidget::paintEvent(QPaintEvent *e)
{
    ogreRoot->_fireFrameStarted();
        ogreRenderWindow->update();
    ogreRoot->_fireFrameEnded();

    e->accept();
}

void OgreWidget::moveEvent(QMoveEvent *e)
{
    QWidget::moveEvent(e);

    if(e->isAccepted() && ogreRenderWindow)
    {
        ogreRenderWindow->windowMovedOrResized();
        update();
    }
}

void OgreWidget::resizeEvent(QResizeEvent *e)
{
    QWidget::resizeEvent(e);

    if(e->isAccepted())
    {
        const QSize &newSize = e->size();
        if(ogreRenderWindow)
        {
            ogreRenderWindow->resize(newSize.width(), newSize.height());
            ogreRenderWindow->windowMovedOrResized();
        }
        if(ogreCamera)
        {
            Ogre::Real aspectRatio = Ogre::Real(newSize.width()) / Ogre::Real(newSize.height());
            ogreCamera->setAspectRatio(aspectRatio);
        }
    }
}

void OgreWidget::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_Print)
    {
//        ogreRenderWindow->writeContentsToTimestampedFile("screenshot", ".jpg");
        e->accept();
    }
    else if(e->key() == Qt::Key_F5)
    {
        Ogre::TextureManager::getSingleton().reloadAll();
        e->accept();
    }
    else if(e->key() == Qt::Key_G)
    {
        if (mDetailsPanel->getTrayLocation() == OgreBites::TL_NONE)
        {
            mTrayMgr->moveWidgetToTray(mDetailsPanel, OgreBites::TL_TOPRIGHT, 0);
            mDetailsPanel->show();
        }
        else
        {
            mTrayMgr->removeWidgetFromTray(mDetailsPanel);
            mDetailsPanel->hide();
        }
        e->accept();
    }
    else if(e->key() == Qt::Key_F)
    {
        mTrayMgr->toggleAdvancedFrameStats();
        e->accept();
    }
    else if(e->key() == Qt::Key_T)
    {
        Ogre::String newVal;
        Ogre::TextureFilterOptions tfo;
        unsigned int aniso;

        switch (mDetailsPanel->getParamValue(9).asUTF8()[0])
        {
        case 'B':
            newVal = "Trilinear";
            tfo = Ogre::TFO_TRILINEAR;
            aniso = 1;
            break;
        case 'T':
            newVal = "Anisotropic";
            tfo = Ogre::TFO_ANISOTROPIC;
            aniso = 8;
            break;
        case 'A':
            newVal = "None";
            tfo = Ogre::TFO_NONE;
            aniso = 1;
            break;
        default:
            newVal = "Bilinear";
            tfo = Ogre::TFO_BILINEAR;
            aniso = 1;
        }

        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(tfo);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(aniso);
        mDetailsPanel->setParamValue(9, newVal);
        e->accept();
    }
    else if(e->key() == Qt::Key_R)
    {
        Ogre::String newVal;
        Ogre::PolygonMode pm;

        switch (ogreCamera->getPolygonMode())
        {
            case Ogre::PM_SOLID:
                  newVal = "Wireframe";
                  pm = Ogre::PM_WIREFRAME;
                  break;
            case Ogre::PM_WIREFRAME:
                  newVal = "Points";
                  pm = Ogre::PM_POINTS;
                  break;
            default:
                  newVal = "Solid";
                  pm = Ogre::PM_SOLID;
        }

        ogreCamera->setPolygonMode(pm);
        mDetailsPanel->setParamValue(10, newVal);
        e->accept();
    }
    else if(e->key() == Qt::Key_A || e->key() == Qt::Key_Left)
    {
        ogreCamera->yaw(Ogre::Degree((e->modifiers() & Qt::ShiftModifier) ? 25 : 3));
        update();
        e->accept();
    }
    else if(e->key() == Qt::Key_E || e->key() == Qt::Key_Right)
    {
        ogreCamera->yaw(Ogre::Degree((e->modifiers() & Qt::ShiftModifier) ? -25 : -3));
        update();
        e->accept();
    }
    else if(e->key() == Qt::Key_Up)
    {
        ogreCamera->setDirection(ogreCamera->getDirection() + Ogre::Vector3(0, 0.07, 0));
        update();
        e->accept();
    }
    else if(e->key() == Qt::Key_Down)
    {
        ogreCamera->setDirection(ogreCamera->getDirection() - Ogre::Vector3(0, 0.07, 0));
        update();
        e->accept();
    }
    else
    {
        Ogre::Vector3 accel = Ogre::Vector3::ZERO;

        // X and Z axis movement
        if (e->key()==Qt::Key_Z) accel += ogreCamera->getDirection();
        if (e->key()==Qt::Key_S) accel -= ogreCamera->getDirection();
        if (e->key()==Qt::Key_D) accel += ogreCamera->getRight();
        if (e->key()==Qt::Key_Q) accel -= ogreCamera->getRight();
        // Cancel any Y axis movement if PageUp and PageDown were untouched.
        accel.y = 0;

        // Y axis movement
        if (e->key()==Qt::Key_PageUp) accel += ogreCamera->getUp();
        if (e->key()==Qt::Key_PageDown) accel -= ogreCamera->getUp();
        // Cancel any X and Z axis movement
        if(accel.y != 0)
        {
            accel.x = 0;
            accel.z = 0;
        }

        if (accel != Ogre::Vector3::ZERO)
        {
            if(e->modifiers() & Qt::ShiftModifier)
                accel *= 100;
            else
                accel *= 6;

            ogreCamera->move(accel);
            update();
            e->accept();
        }
        else
            e->ignore();
    }
}

void OgreWidget::initOgreSystem()
{
    // Create the Ogre root object
    ogreRoot = OgreResources::createRoot();

    // A list of required plugins
    Ogre::StringVector required_plugins;
    required_plugins.push_back("GL RenderSystem");
    required_plugins.push_back("Octree & Terrain Scene Manager");
    required_plugins.push_back("ParticleFX");
//    required_plugins.push_back("Cg Program Manager");

    // List of plugins to load
    Ogre::StringVector plugins_toLoad;
    plugins_toLoad.push_back("RenderSystem_GL");
    plugins_toLoad.push_back("Plugin_OctreeSceneManager");
    plugins_toLoad.push_back("Plugin_ParticleFX");
//    plugins_toLoad.push_back("Plugin_CgProgramManager");

    // Check if plugins are installed and usable
    for (Ogre::StringVector::iterator j = plugins_toLoad.begin(); j != plugins_toLoad.end(); j++)
    {
    #ifdef _DEBUG
        ogreRoot->loadPlugin(*j + Ogre::String("_d"));
    #else
        ogreRoot->loadPlugin(*j);
    #endif
    }

    // Check if the required plugins are installed and ready for use
    // If not: exit the application
    Ogre::Root::PluginInstanceList ip = ogreRoot->getInstalledPlugins();
    for (Ogre::StringVector::iterator j = required_plugins.begin(); j != required_plugins.end(); j++)
    {
        bool found = false;
        // try to find the required plugin in the current installed plugins
        for (Ogre::Root::PluginInstanceList::iterator k = ip.begin(); k != ip.end(); k++)
        {
            if ((*k)->getName() == *j)
            {
                found = true;
                break;
            }
        }
        if (!found)  // exit because a required plugin is not available
        {
            qFatal("Could not load Ogre plugins, see log file.");
            exit(-1);
        }
    }

	Ogre::RenderSystem* rs = ogreRoot->getRenderSystemByName("OpenGL Rendering Subsystem");
	if(!(rs->getName() == "OpenGL Rendering Subsystem"))
    {
        qFatal("Could not find the Ogre rendering system.");
        exit(-2);
    }
    ogreRoot->setRenderSystem(rs);
    ogreRoot->initialise(false);
    ogreRoot->setFrameSmoothingPeriod(0.05);

    // Create the Scene Manager
    ogreSceneManager = OgreResources::createSceneManager();

    // Try to get the right widget handle config - this code works at least under Linux
    Ogre::NameValuePairList viewConfig;
#ifdef Q_WS_WIN
    widgetHandle = (size_t)((HWND)winId());
#else
    QWidget *q_parent = dynamic_cast <QWidget *> (parentWidget()); // use parentWidget() and not parent()
    assert(q_parent);

    QX11Info xInfo = x11Info();

    viewConfig["parentWindowHandle"] =
         Ogre::StringConverter::toString ((unsigned long)xInfo.display()) +
        ":" + Ogre::StringConverter::toString ((unsigned int)xInfo.screen()) +
        ":" + Ogre::StringConverter::toString ((unsigned long)q_parent->winId());
#endif

    // Create the Render Window
    ogreRenderWindow = ogreRoot->createRenderWindow("Ogre rendering window", width(), height(), false, &viewConfig);

    // Create the camera
    ogreCamera = ogreSceneManager->createCamera("myCamera");
    setCameraPosition(Ogre::Vector3(-4000,2000,-4000));
    ogreCamera->lookAt(0,0,0);

    ogreCamera->setNearClipDistance(5);
    ogreCamera->setFarClipDistance(10000);
    if (ogreRoot->getRenderSystem()->getCapabilities()->hasCapability(Ogre::RSC_INFINITE_FAR_PLANE))
        ogreCamera->setFarClipDistance(0);

    // Create the Viewport
    ogreViewport = ogreRenderWindow->addViewport(ogreCamera);
    ogreViewport->setBackgroundColour(Ogre::ColourValue(0.7f,0.7f,0.7f));
    ogreCamera->setAspectRatio(Ogre::Real(ogreViewport->getActualWidth()) / Ogre::Real(ogreViewport->getActualHeight()));

    // Get the ID of Ogre render window
    WId windowId ;
    ogreRenderWindow->getCustomAttribute("WINDOW", &windowId);
    assert(windowId);

    // Take over the ogre created window.
    this->create(windowId);

    // Load various resources
    setupLoadResources();

    // Bad SdkTrays code, FIXME later
    mTrayMgr = new OgreBites::SdkTrayManager("InterfaceName", ogreRenderWindow, this);
    mTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
    mTrayMgr->showLogo(OgreBites::TL_BOTTOMRIGHT);

    // create a params panel for displaying sample details
    Ogre::StringVector items;
    items.push_back("cam.pX");
    items.push_back("cam.pY");
    items.push_back("cam.pZ");
    items.push_back("cam.oW");
    items.push_back("cam.oX");
    items.push_back("cam.oY");
    items.push_back("cam.oZ");
    items.push_back("");
    items.push_back("");
    items.push_back("Filtering");
    items.push_back("Poly Mode");

    mDetailsPanel = mTrayMgr->createParamsPanel(OgreBites::TL_NONE, "DetailsPanel", 200, items);
    mDetailsPanel->setParamValue(9, "Bilinear");
    mDetailsPanel->setParamValue(10, "Solid");
    mDetailsPanel->hide();

    ogreRoot->addFrameListener(this);

    emit(ogreInitialized());
}

void OgreWidget::setupLoadResources()
{
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    #ifdef Q_WS_WIN
        cf.load(QString(QDir::current().absolutePath()+"\\resources.cfg").toStdString());
    #else
        cf.load(QString(QDir::current().absolutePath()+"/resources.cfg").toStdString());
    #endif

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
#if OGRE_PLATFORM == OGRE_PLATFORM_APPLE
            // OS X does not set the working directory relative to the app,
            // In order to make things portable on OS X we need to provide
            // the loading with it's own bundle path location
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            Ogre::String(macBundlePath() + "/" + archName), typeName, secName);
#else
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            archName, typeName, secName);
#endif
        }
    }

    // Initialise, parse scripts etc
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

bool OgreWidget::frameRenderingQueued(const Ogre::FrameEvent &evt)
{
    if(ogreRenderWindow->isClosed())
        return false;

    mTrayMgr->frameRenderingQueued(evt);

    if (!mTrayMgr->isDialogVisible())
    {
        if (mDetailsPanel->isVisible())   // if details panel is visible, then update its contents
        {
            mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(ogreCamera->getDerivedPosition().x));
            mDetailsPanel->setParamValue(1, Ogre::StringConverter::toString(ogreCamera->getDerivedPosition().y));
            mDetailsPanel->setParamValue(2, Ogre::StringConverter::toString(ogreCamera->getDerivedPosition().z));
            mDetailsPanel->setParamValue(3, Ogre::StringConverter::toString(ogreCamera->getDerivedOrientation().w));
            mDetailsPanel->setParamValue(4, Ogre::StringConverter::toString(ogreCamera->getDerivedOrientation().x));
            mDetailsPanel->setParamValue(5, Ogre::StringConverter::toString(ogreCamera->getDerivedOrientation().y));
            mDetailsPanel->setParamValue(6, Ogre::StringConverter::toString(ogreCamera->getDerivedOrientation().z));
        }
    }
    return true;
}

void OgreWidget::render()
{
    // Internal clock of the rendering loop
    static btScalar simulationRunTime = 0;

    // Indicates that rendering started
    currentlyRendering = true;

    // Variable used to store the time that is queried, and actually retrieved
    btScalar currentTimeStep = simulationRunTime;// + getTargetTimeStep()/2.0f;

    // If there is no buffer interface, directly skip to rendering
    if(hasBufferInterface())
    {
        // Get the btTransforms as a pointer. currentTimeStep will indicate the time actually retrieved
        QSharedPointer<obEntityTransformRecordList> ptr = bufferInterface->processNext(currentTimeStep);   //((double)ogreRoot->getNextFrameNumber())*(1000.0f/targetFPS));

        if(!ptr.isNull() && bufferInterface->getBufferType()==CircularTransformBuffer::DiscreteCollisionDetection)
        {
            // Update simulationRunTime for the next query to perform
            simulationRunTime = currentTimeStep + getTargetTimeStep();

            // Update Ogre objects
            obEntityTransformRecordListIterator it(*ptr);
            while(it.hasNext())
            {
                const obEntityTransformRecord &record = it.next();
                Ogre::Node *node = ogreSceneManager->getSceneNode(record.obEnt->getName());






                //        btTransform interpolatedTrans;
        //        btTransformUtil::integrateTransform(myNinjaBody->getWorldTransform(),myNinjaBody->getLinearVelocity(),myNinjaBody->getAngularVelocity(),1/600.f,interpolatedTrans);
        //    //    w->entities["ninja"]->getRigidBody()->setTransformation(interpolatedTrans);

        //        qDebug() << interpolatedTrans.getOrigin().x() << ", "<< interpolatedTrans.getOrigin().y() << ", "<< interpolatedTrans.getOrigin().z();
        //        qDebug() << myNinjaBody->getWorldTransform().getOrigin().x() << ", "<< myNinjaBody->getWorldTransform().getOrigin().y()<< ", "<< myNinjaBody->getWorldTransform().getOrigin().z();





                node->setPosition(Utils::vectorFromBullet(record.transform.getOrigin()));
                node->setOrientation(Utils::quaternionFromBullet(record.transform.getRotation()));

                // Update color according to status for debug
                if(record.status == obEntity::CrossingBorder)
                {
                    record.obEnt->setColor(0, 0, 0);
                }
                else if(record.status == obEntity::OutOfWorld)
                {
                    record.obEnt->setColor(1, 0, 0);
//                    qDebug() << "Deleting eEntity '" << record.obEnt->getDisplayName() << "at time" << ptr->getTimeStep();
//                    delete record.obEnt;

                    //TODO either use a better status system or use smart pointers for deleting obEnts; // or code a garbage collector that is messaged, and that runs in the main thread.
                }
            }
        }
    }

    // Render
    ogreRoot->renderOneFrame();
    update();

    // Render again if we missed a tick. This may cause stack overflow in extreme cases.
    if(renderingPassQueued)
    {
        renderingPassQueued = false;
        qWarning() << "The rendering loop took more than a time step to render.";
        render();
    }
    currentlyRendering = false;
}


void OgreWidget::setCameraPosition(const Ogre::Vector3 &pos)
{
    ogreCamera->setPosition(pos);
    update();
}

void OgreWidget::onTimerTick()
{
    if(currentlyRendering)
        renderingPassQueued = true;
    else
        render();
}
