#
# Example unit test module with 2 unit test suites.
#
include(QtTestUtil/QtTestUtil.pri)

QT += qtestlib core qtestlib opengl qtestlib
CONFIG += qtestlib qt qtestlib
CONFIG -= app_bundle

TARGET = Tests

SOURCES += \
    QtTestUtil/SimpleChecker.cpp \
    TestCell.cpp \
    TestGrid.cpp \
    ../src/ogrewidget.cpp \
    ../src/bulletmanager.cpp \
    ../src/grid.cpp \
    ../src/physicsworld.cpp \
    ../src/obEntityWrapper.cpp \
    ../src/obMotionState.cpp \
    ../src/obRigidBody.cpp \
    ../Parser/Action.cpp \
    ../Parser/Animation.cpp \
    ../Parser/AnimationLibrary.cpp \
    ../Parser/DotScene.cpp \
    ../Parser/DotSceneElement.cpp \
    ../Parser/EnvironmentElement.cpp \
    ../Parser/EnvironmentLibrary.cpp \
    ../Parser/EnvironmentLibraryListModel.cpp \
    ../Parser/SceneElement.cpp \
    ../Parser/SceneLibrary.cpp \
    ../Parser/SceneLibraryListModel.cpp \
    ../Parser/UniquenessConstraintException.cpp \
    ../src/circulartransformbuffer.cpp \
    ../src/timesteptransformmap.cpp \
    ../src/circulartransformbufferinterface.cpp \
    ../EKMeans/libekmeans.cpp \
    ../src/cell.cpp \
    ../src/localgrid.cpp \
    ../src/ogreresourcemanager.cpp

HEADERS += \
    QtTestUtil/SimpleChecker.h \
    ../src/ogrewidget.h \
    ../src/bulletmanager.h \
    ../src/grid.h \
    ../src/physicsworld.h \
    ../src/obEntityWrapper.h \
    ../src/obMotionState.h \
    ../src/obRigidBody.h \
    ../Parser/Action.h \
    ../Parser/Animation.h \
    ../Parser/AnimationLibrary.h \
    ../Parser/DotScene.h \
    ../Parser/DotSceneElement.h \
    ../Parser/EnvironmentElement.h \
    ../Parser/EnvironmentLibrary.h \
    ../Parser/EnvironmentLibraryListModel.h \
    ../Parser/SceneElement.h \
    ../Parser/SceneLibrary.h \
    ../Parser/SceneLibraryListModel.h \
    ../Parser/UniquenessConstraintException.h \
    ../src/mySdkTrays.h \
    ../src/circulartransformbuffer.h \
    ../src/timesteptransformmap.h \
    ../src/circulartransformbufferinterface.h \
    ../EKMeans/libekmeans.h \
    ../src/cell.h \
    ../src/localgrid.h \
    ../src/ogreresourcemanager.h

FORMS += \
    ../src/mainasapcdwindow.ui

LIBS += -lOgreMain \
    -lOgreTerrain \
    -lBulletDynamics \
    -lBulletCollision \
    -lBulletSoftBody \
    -lBulletMultiThreaded \
    -lLinearMath \
    -lblitz


DEPENDPATH += ..

win32 {
    INCLUDEPATH += . \
        .. \
        ..\src \
        ..\Parser \
        ..\EKMeans \
        $(OGRE_HOME)\include \
        $(BULLET_HOME)src
}
unix {
    INCLUDEPATH += .. . \
        ../src \
        ../Parser \
        ../EKMeans \
        /usr/local/include/bullet/ \
        /usr/local/include/OGRE/ \
        /usr/include/boost/ \
        /usr/include/blitz/ \
        /usr/include/OGRE/ \
        /usr/include/
}

# Add an extra 'make check' target.
QMAKE_EXTRA_TARGETS = check
check.commands = \$(MAKE) && ./$(QMAKE_TARGET)

# Cleanup the checker on 'make clean'
QMAKE_CLEAN += $(QMAKE_TARGET)
