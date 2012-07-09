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
#include "ui_mainasapcdwindow.h"
#include "randomcubesimulation.h"

MainAsapCdWindow::MainAsapCdWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainAsapCdWindow),
    targetFPS(60)
{
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
}

MainAsapCdWindow::~MainAsapCdWindow()
{
    delete simulation;

    delete ogreWidget;

    for(int i=0; i<bufferInterface.size(); ++i)
        delete bufferInterface[i];

    delete ui;
}

void MainAsapCdWindow::onOgreReady()
{
    simulation = new RandomCubeSimulation();
    ogreWidget->registerBufferInterface(simulation->getBufferInterfaces()[0]);
    simulation->start();
}

int main(int argc, char *argv[])
{
    std::cout << APP_NAME << " version " << VERSION << ", Copyright (C) 2012 Inria Rennes.\n" <<
                 std::endl <<
                 APP_NAME << " comes with ABSOLUTELY NO WARRANTY; for details " <<
                 "read the COPYING file included with the program.  This is " <<
                 "free software, and you are welcome to redistribute it under " <<
                 "certain conditions." << std::endl;

   // Create the QApplicationbuffer[
   QApplication app(argc, argv);

//   qDebug() << "Ideal thread count on this machine: " << QThread::idealThreadCount();

   // Initialize the window and Ogre rendering system
   MainAsapCdWindow* window = new MainAsapCdWindow();
   window->show();

   return app.exec();
}
