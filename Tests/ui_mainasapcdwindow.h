/********************************************************************************
** Form generated from reading UI file 'mainasapcdwindow.ui'
**
** Created: 
**      by: Qt User Interface Compiler version 4.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINASAPCDWINDOW_H
#define UI_MAINASAPCDWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainAsapCdWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QWidget *widget;
    QMenuBar *menubar;
    QMenu *menuMenu;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainAsapCdWindow)
    {
        if (MainAsapCdWindow->objectName().isEmpty())
            MainAsapCdWindow->setObjectName(QString::fromUtf8("MainAsapCdWindow"));
        MainAsapCdWindow->resize(624, 319);
        centralwidget = new QWidget(MainAsapCdWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        widget = new QWidget(centralwidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(widget);

        MainAsapCdWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainAsapCdWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 624, 29));
        menuMenu = new QMenu(menubar);
        menuMenu->setObjectName(QString::fromUtf8("menuMenu"));
        MainAsapCdWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainAsapCdWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainAsapCdWindow->setStatusBar(statusbar);

        menubar->addAction(menuMenu->menuAction());

        retranslateUi(MainAsapCdWindow);

        QMetaObject::connectSlotsByName(MainAsapCdWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainAsapCdWindow)
    {
        MainAsapCdWindow->setWindowTitle(QApplication::translate("MainAsapCdWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        menuMenu->setTitle(QApplication::translate("MainAsapCdWindow", "Menu", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainAsapCdWindow: public Ui_MainAsapCdWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINASAPCDWINDOW_H
