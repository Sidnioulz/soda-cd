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
#include <QApplication>
#include <QtCore>
#include <iostream>

#include "main.h"
#include "ui_mainwindow.h"
#include "randomcubesimulation.h"
#include "experimenttrackinginterface.h"

MainPepsiWindow *MainPepsiWindow::instance = 0;

MainPepsiWindow::MainPepsiWindow(QxtCommandOptions &opt, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainPepsiWindow),
    targetFPS(60),
    logFile()
{
    // Setup parameters using the QxtCommandOptions object
    paramAutomatedSimulation = opt.parameters().value("automated", QVariant(false)).toBool();
    paramTimeLimit = opt.parameters().value("time-limit", QVariant(10.0f)).toFloat();
    paramLogFilePath = opt.parameters().value("output-path", QVariant("/tmp/pepsis.out")).toString();
    logFile.setFileName(paramLogFilePath);

    paramSpaceLen.setX(opt.parameters().value("x", QVariant(6000)).toInt());
    paramSpaceLen.setY(opt.parameters().value("y", QVariant(2000)).toInt());
    paramSpaceLen.setZ(opt.parameters().value("z", QVariant(6000)).toInt());

    paramNbWorlds = opt.parameters().value("nb-worlds", QVariant(0)).toInt();

    paramNbEntities = opt.parameters().value("nb-entities", QVariant(800)).toInt();
    paramSimulationName = opt.parameters().value("simulation", QVariant("RandomCubeSimulation")).toString();

    // Various UI setups
    ui->setupUi(this);
    this->setWindowTitle("Asynchronous Scalable Anticipative Parallel CD");
    ui->widget->setLayout(new QGridLayout());
    ui->widget->layout()->setMargin(0);

    // IPC setup
    qRegisterMetaType<PhysicsWorld *>("PhysicsWorld *");
    qRegisterMetaType<QVector<CellBorderCoordinates> >("QVector<CellBorderCoordinates>");
    qRegisterMetaType<obEntityWrapper *>("obEntityWrapper *");
    qRegisterMetaType<btScalar>("btScalar");
    qRegisterMetaType<Ogre::Entity *>("Ogre::Entity *");
    qRegisterMetaType<Ogre::SceneNode *>("Ogre::SceneNode *");

    // Create the Ogre Widget
    ogreWidget = new OgreWidget(targetFPS);
    ui->widget->layout()->addWidget(ogreWidget);

    // Connect the Ogre Widget's slots to this interface's signals
    connect(ogreWidget, SIGNAL(ogreInitialized()), this, SLOT(onOgreReady()));

    // Give focus to the Ogre Widget
    ogreWidget->setFocus();
    this->setWindowState(Qt::WindowMaximized);
    this->setFocusPolicy(Qt::StrongFocus);

    // Setup actions for menu entries
    QAction *closeAct = new QAction("&Quit", this);
    connect(closeAct, SIGNAL(triggered()), this, SLOT(close()));
    ui->menuMenu->addAction(closeAct);

    QAction *toggleOgre = new QAction("&Show 3D Widget", this);
    toggleOgre->setCheckable(true);
    toggleOgre->setChecked(true);

    connect(toggleOgre, SIGNAL(triggered(bool)), ogreWidget, SLOT(setShown(bool)));
    ui->menuDebug->addAction(toggleOgre);

    // Setup log file text stream
    logFile.open(QIODevice::WriteOnly | QIODevice::Text);

    // Set a time limit for the simulation, if required
    if(paramAutomatedSimulation)
    {
        ExperimentTrackingInterface::getInterface()->setSimulationTimeLimit(paramTimeLimit);
        connect(ExperimentTrackingInterface::getInterface(), SIGNAL(simulationTimeLimitReached(btScalar)), this, SLOT(close()));
    }
}

MainPepsiWindow::~MainPepsiWindow()
{
#ifndef NDEBUG
        qDebug() << "MainPepsiWindow::~MainPepsiWindow(); Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

    simulation->stop();
    ogreWidget->clearBufferInterface();

    QTextStream out(&logFile);
    simulation->printStats(out);
    out.flush();
    logFile.close();

    if(simulation)
        delete simulation;

    exit(0); //NO TIME TO LOOSE

    delete ogreWidget; // should be done by magic
    delete ui;
#ifndef NDEBUG
        qDebug() << "MainPepsiWindow::~MainPepsiWindow(); Destroyed; Thread " << QString().sprintf("%p", QThread::currentThread());
#endif

}

MainPepsiWindow *MainPepsiWindow::getInstance(QxtCommandOptions &opt)
{
    if(!instance)
        instance = new MainPepsiWindow(opt);

    return getInstance();
}

MainPepsiWindow *MainPepsiWindow::getInstance()
{
    return instance;
}

void MainPepsiWindow::deleteInstance()
{
    if(instance)
    {
        delete instance;
        instance = 0;
    }
}

void MainPepsiWindow::onOgreReady()
{
    // If automated run, hide the widget
    if(paramAutomatedSimulation)
        ui->menuDebug->actions().at(0)->trigger();

    // Create a supported simulation
    //TODO: dynamic loading of plugins
    if(paramSimulationName == "RandomCubeSimulation")
        simulation = new RandomCubeSimulation(1.0f/60, paramNbWorlds, paramSpaceLen, paramNbEntities, false);

    // If a simulation is runnable
    if(simulation)
    {
        ogreWidget->registerBufferInterface(simulation->getBufferInterface());
        simulation->start();
    }
}

int main(int argc, char *argv[])
{
    std::cout << APP_NAME << " version " << VERSION << ", Copyright (C) 2012 Inria Rennes.\n" <<
                 std::endl <<
                 APP_NAME << " comes with ABSOLUTELY NO WARRANTY; for details " <<
                 "read the COPYING file included with the program.  This is " <<
                 "free software, and you are welcome to redistribute it under " <<
                 "certain conditions." << std::endl << std::endl;

   // Create the QApplicationbuffer[
   QApplication app(argc, argv);

   // Setup command-line arguments object
   QxtCommandOptions opt;

   opt.add("simulation", "Simulation name", QxtCommandOptions::ValueRequired);
   opt.alias("simulation", "s");

   opt.addSection("Simulation Specific Parameters");
   opt.add("nb-entities", "Number of entities (if applicable)", QxtCommandOptions::ValueRequired);
   opt.alias("nb-entities", "n");

   opt.add("x", "X dimension of the simulation scene (if applicable)", QxtCommandOptions::ValueRequired);
   opt.alias("x", "x");

   opt.add("y", "Y dimension of the simulation scene (if applicable)", QxtCommandOptions::ValueRequired);
   opt.alias("y", "y");

   opt.add("z", "Z dimension of the simulation scene (if applicable)", QxtCommandOptions::ValueRequired);
   opt.alias("z", "z");

   opt.add("nb-worlds", "Number of worlds in which to run the simulation", QxtCommandOptions::ValueRequired);
   opt.alias("nb-worlds", "w");

   opt.addSection("Debug and Automation Parameters");
   opt.add("output-path", "Path to the log file to create", QxtCommandOptions::ValueRequired);
   opt.alias("output-path", "o");

   opt.add("automated", "Enable automatic simulation and shutdown (requires -t)", QxtCommandOptions::NoValue);
   opt.alias("automated", "a");

   opt.add("time-limit", "Time limit in seconds", QxtCommandOptions::ValueRequired);
   opt.alias("time-limit", "t");


   std::cout << APP_NAME << " options:" << std::endl;
   opt.showUsage();

   // Parse the arguments
   opt.setFlagStyle(QxtCommandOptions::SingleDash);
   opt.setParamStyle(QxtCommandOptions::SpaceAndEquals);
   opt.parse(app.arguments());

   // Initialize the window and Ogre rendering system
   MainPepsiWindow* window = MainPepsiWindow::getInstance(opt);
   window->show();

   // Run the application
   int r = app.exec();

   // Memory cleanup and exit
   MainPepsiWindow::deleteInstance();
   return r;
}
