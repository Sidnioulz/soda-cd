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
#define VERSION     "0.3.9"

#include <QMainWindow>
#include <QxtCore/QxtCommandOptions>
#include "sodaOgreWidget.h"
#include "simulation.h"
#include "sodaDynamicEntity.h"
#include "cellborderentity.h"
#include "typedefs.h"

// Doxygen documentation main page, do not remove
/*! \mainpage Welcome to the documentation for PEPSI's.
 *
 * \section intro_sec About this application.
 *
 * PEPSI's has been developed as part of a research internship in the <a href="http://www.inria.fr/equipes/vr4i">IRISA-Inria VR4i</a> project team in Rennes, France. The main developer is Steve Dodier-Lazaro, who worked under the supervision of Valérie Gouranton and Quentin Avril. It is currently not available for download and is the exclusive property of Inria Rennes, but a release of a final prototype will be made if given the occasion.
 *
 * \section install_sec Building PEPSI's
 *
 * You will need the following dependencies installed: Qt, Bullet Physics, Ogre 3D, Blitz++, Boost, Qxt. The libraries and include files of all dependencies should be reachable by Qt Creator. If one of them is missing, please edit the .pro file attached to this project, and set INCLUDEPATH and LIBS to appropriate values.
 * Please use qmake and then make to build PEPSI's outside of Qt Creator. You may need to modify your LD_LIBRARY_PATH environment variable at runtime to be able to run the application.
 *
 * Note that the Blitz++ Fedora 16 package misses some code headers, and you may need to manually install them to build PEPSI's.
 *
 * \section status_sec Current status
 * There are currently no active developers for PEPSI's. It is at the moment capable of detecting collisions in a distributed fashion, but constraint solving between objects managed by different processors has not yet been implemented, and a few bugs remain to be fixed. Parallel/Distributed constraint solving may be more efficient/clever with a different granularity than the one chosen in PEPSI's and version 1 of the SODA framework, and a proposal will be issued for the development of this component.
 *
 * \section report_bugs Reporting a bug
 * If you are in-house, please write a bug report in the Inria GForge page of the project (https://gforge.inria.fr/tracker/?atid=13788&group_id=4142&func=browse), or email <a href="mailto:valerie.gouranton@inria.fr">the project head</a> in last resort for her to dispatch the bug report to current developers (specifying [SODA-BUGS] at the start of the mail subject). Steve has the technical know-how about the project and can be reached at <a href="mailto:steve.dodier@ensi-bourges.fr">his school mail address</a>. Same mail subject rules apply.
 */

//TODO: when objects and cells too big to populate several worlds, don't!

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
  * \param opt the object containing user's command-line options
  * \param parent the parent widget if it exists
  * \return a new MainPepsiWindow
  */
explicit MainPepsiWindow(QxtCommandOptions &opt, QWidget *parent = 0);

public:
    /*!
      * \brief Default destructor.
      */
    ~MainPepsiWindow();

    /*!
     * \brief Permits access to the sodaOgreWidget.
     * \return a pointer to the MainPepsiWindow's sodaOgreWidget
     */
    inline sodaOgreWidget *getOgreWidget() const
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
    static MainPepsiWindow *getInstance(QxtCommandOptions &opt);

    /*!
     * \brief Returns the instance of MainPepsiWindow.
     * \return the instance of MainPepsiWindow
     */
    static MainPepsiWindow *getInstance();

    /*!
     * \brief Deletes the instance of MainPepsiWindow.
     */
    static void deleteInstance();

private:
    // Placeholder for application parameters (more readable than direct use of QxtCommandOptions)
    QString                             paramLogFilePath;             //!< Path of this application's log file
    btScalar                            paramTimeLimit;               //!< Time limit for the simulation (used to automate shutdown)
    bool                                paramAutomatedSimulation;     //!< Whether simulations are automated
    btVector3                           paramSpaceLen;                //!< Length of the simulation space, if applicable to chosen Simulation
    int                                 paramNbWorlds;                //!< Number of sodaLogicWorlds to use
    int                                 paramNbEntities;              //!< Number of entities, if applicable to chosen Simulation
    QString                             paramSimulationName;          //!< Simulation name

    static MainPepsiWindow              *instance;          //!< The unique instance of MainPepsiWindow

    Ui::MainPepsiWindow                 *ui;                //!< Pointer to file-defined Qt GUI widgets
    sodaOgreWidget                      *ogreWidget;        //!< Pointer to the Ogre rendering widget
    Simulation                          *simulation;        //!< A Simulation object

    int                                 targetFPS;          //!< target FPS of the simulation
    QFile                               logFile;            //!< File in which writing the log is performed

private slots:
    //! Used to initialize the Simulation Scene once Ogre is ready to work
    void onOgreReady();
};


#endif // MAIN_H
