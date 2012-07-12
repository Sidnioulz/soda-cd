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
#ifndef MAIN_H
#define MAIN_H

#define APP_NAME    "PEPSI's"
#define VERSION     "0.3.2"

#include <QMainWindow>
#include "ogrewidget.h"
#include "simulation.h"

// Doxygen documentation main page, do not remove
/*! \mainpage Welcome to the documentation for SODA CD.
 *
 * \section intro_sec About this application.
 *
 * SODA CD has been developed as part of a research internship in the <a href="http://www.inria.fr/equipes/vr4i">Inria VR4I</a> project team, by Steve Dodier-Lazaro, under the supervision of Valérie Gouranton and Quentin Avril. It is currently not available for download and is the exclusive property of Inria Rennes.
 *
 * \section install_sec Building SODA CD
 *
 * You will need the following dependencies installed: Qt, Bullet Physics, Ogre 3D, Blitz++, Boost. The libraries and include files of all dependencies should be reachable by Qt Creator. If one of them is missing, please edit the .pro file attached to this project, and set INCLUDEPATH and LIBS to appropriate values.
 * Please use qmake and then make to build SODA CD outside of Qt Creator. You may need to modify your LD_LIBRARY_PATH environment variable at runtime to be able to run the application.
 *
 * Note that the Blitz++ Fedora 16 package misses some code headers, and you may need to manually install them to build SODA CD.
 *
 * \section report_bugs Reporting a bug
 * Please email either <a href="mailto:steve.dodier-lazaro@inria.fr">Steve</a> or <a href="mailto:quentin.avril@irisa.fr">Quentin</a>.
 */

//TODO: https://gforge.inria.fr/projects/kaapi/	check if useful
//TODO: http://runtime.bordeaux.inria.fr/marcel/	scheduling and thread placement
//TODO: http://runtime.bordeaux.inria.fr/hwloc/	ideal number of threads, hw ls
//TODO: http://runtime.bordeaux.inria.fr/StarPU/	gpu async computation for cpu's
//TODO: http://runtime.bordeaux.inria.fr/pm2-doc/Section3.html#Section3.1 for multi-pc parallelism

//TODO: inlining
//TODO: assertions on all function headers
//TODO: when objects and cells too big to populate several worlds, don't!

// IPC type declarations
Q_DECLARE_METATYPE(PhysicsWorld *)
Q_DECLARE_METATYPE(QVector<CellBorderCoordinates>)
Q_DECLARE_METATYPE(obEntityWrapper *)
Q_DECLARE_METATYPE(btScalar)
Q_DECLARE_METATYPE(Ogre::Entity *)
Q_DECLARE_METATYPE(Ogre::SceneNode *)

namespace Ui {
    class MainPepsiWindow;
}

/*! \class MainPepsiWindow
  * \brief A Qt window that contains an Ogre+bullet simulator.
  * \author Steve Dodier-Lazaro <steve.dodier-lazaro@inria.fr, sidnioulz@gmail.com>
  *
  * This class implements a simulation application that combines the bullet physics
  * engine and the Ogre 3D engine. It instantiates widgets, the rendering engine and
  * the simulation management object.
  */
class MainPepsiWindow : public QMainWindow
{
    Q_OBJECT

protected:
/*!
  * \brief Default constructor.
  * \param parent the parent widget if it exists
  * \return a new MainPepsiWindow
  */
explicit MainPepsiWindow(QWidget *parent = 0);

public:
    /*!
      * \brief Default destructor.
      */
    ~MainPepsiWindow();

    /*!
     * \brief Permits access to the OgreWidget.
     * \return a pointer to the MainPepsiWindow's OgreWidget
     */
    inline OgreWidget *getOgreWidget() const
    {
        return ogreWidget;
    }

    /*!
     * \brief Permits access to the current Simulation.
     * \return a pointer to the MainPepsiWindow's current Simulation
     */
    inline Simulation *getCurrentSimulation() const
    {
        return simulation;
    }

    /*!
     * \brief Returns the instance of MainPepsiWindow.
     * \return the instance of MainPepsiWindow
     */
    static MainPepsiWindow *getInstance();

private:
    //Temporary placeholder for application parameters
    static QString                      paramLogFilePath;   //!< Path of this application's log file
    static btScalar                     paramTimeLimit;     //!< Time limit for the simulation (used to automate shutdown)
    static bool                         paramAutomatedSimulation;     //!< Whether simulations are automated




    static MainPepsiWindow              *instance;          //!< The unique instance of MainPepsiWindow

    Ui::MainPepsiWindow                 *ui;                //!< Pointer to file-defined Qt GUI widgets
    OgreWidget                          *ogreWidget;        //!< Pointer to the Ogre rendering widget
    Simulation                          *simulation;        //!< A Simulation object

    int                                 targetFPS;          //!< target FPS of the simulation
    QFile                               logFile;            //!< File in which writing the log is performed

private slots:
    //! Used to initialize the Simulation Scene once Ogre is ready to work
    void onOgreReady();
};


#endif // MAIN_H
