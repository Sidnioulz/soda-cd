QT += core opengl gui
TARGET = SODA-CD

CONFIG  += qxt
QXT     += core gui

*-g++* {
    QMAKE_CXXFLAGS += -Wno-unused-but-set-parameter -Wno-unused-parameter
#	QMAKE_CXXFLAGS += -std=c++11
#	QMAKE_CXX = clang++
}

#CONFIG(release, debug|release) {
#    DESTDIR = SODA-CD-build-desktop-Qt_4_8_0_in_PATH__System__Release
#} else {
#    DESTDIR = SODA-CD-build-desktop-Qt_4_8_0_in_PATH__System__Debug
#}
DESTDIR = .


win32 {
    #copyfiles.commands += @call copy ..\\$${TARGET}\\Plugins.tmp $${DESTDIR}\\Plugins.tmp
}
unix {
	 copyfiles.commands +=  cp ../$${TARGET}/data/Plugins.cfg ../$${TARGET}/data/resources.cfg  $${DESTDIR}/ &&
	 copyfiles.commands +=  ln -fs  ../$${TARGET}/media $${DESTDIR}/
}

QMAKE_EXTRA_TARGETS += copyfiles
POST_TARGETDEPS += copyfiles

QMAKE_CXXFLAGS_DEBUG += -g -pg
QMAKE_CFLAGS_DEBUG += -g -pg
QMAKE_LFLAGS_DEBUG += -g -pg

CONFIG(debug, debug|release) {
#    LIBS += -lOgreGUIRenderer_d -lOgreMain_d
}
CONFIG(release, debug|release) {
	QMAKE_CXXFLAGS += -O3 -DNDEBUG
}

TEMPLATE = app

HEADERS += \
    src/bulletmanager.h \
    src/main.h \
    src/grid.h \
    Parser/Action.h \
    Parser/Animation.h \
    Parser/AnimationLibrary.h \
    Parser/DotScene.h \
    Parser/DotSceneElement.h \
    Parser/EnvironmentElement.h \
    Parser/EnvironmentLibrary.h \
    Parser/EnvironmentLibraryListModel.h \
    Parser/SceneElement.h \
    Parser/SceneLibrary.h \
    Parser/SceneLibraryListModel.h \
    Parser/UniquenessConstraintException.h \
    src/mySdkTrays.h \
    src/circulartransformbuffer.h \
    src/circulartransformbufferinterface.h \
    EKMeans/EKMeans_global.h \
    EKMeans/libekmeans.h \
    src/cell.h \
    src/obentitytransformrecord.h \
    src/obentitytransformrecordlist.h \
    src/cellborderentity.h \
    src/simulation.h \
    src/experimenttrackinginterface.h \
    src/robustnessevalsimulation.h \
    src/typedefs.h \
    src/sodaSimulationIslandForeignerCache.h \
    src/sodaLocalGridBroadphase.h \
    src/sodaDynamicsWorld.h \
    src/sodaSimulationIslandManager.h \
    src/sodaContactResultCallback.h \
    src/sodaRigidBody.h \
    src/sodaDynamicRigidBody.h \
    src/sodaPersistentForeignerManifold.h \
    src/sodaPersistentForeignerManifoldArray.h \
    src/sodaMeshShapeCache.h \
    src/sodaMotionState.h \
    src/sodaLocalGrid.h \
    src/sodaEntity.h \
    src/sodaOgreResources.h \
    src/sodaOgreWidget.h \
    src/sodaUtils.h \
    src/sodaDynamicEntity.h \
    src/randomcubesimulation.h \
    src/sodaLogicWorld.h

SOURCES += \
    src/bulletmanager.cpp \
    src/main.cpp \
    src/grid.cpp \
    Parser/Action.cpp \
    Parser/Animation.cpp \
    Parser/AnimationLibrary.cpp \
    Parser/DotScene.cpp \
    Parser/DotSceneElement.cpp \
    Parser/EnvironmentElement.cpp \
    Parser/EnvironmentLibrary.cpp \
    Parser/EnvironmentLibraryListModel.cpp \
    Parser/SceneElement.cpp \
    Parser/SceneLibrary.cpp \
    Parser/SceneLibraryListModel.cpp \
    Parser/UniquenessConstraintException.cpp \
    src/circulartransformbuffer.cpp \
    src/circulartransformbufferinterface.cpp \
    EKMeans/libekmeans.cpp \
    src/cell.cpp \
    src/obentitytransformrecord.cpp \
    src/obentitytransformrecordlist.cpp \
    src/cellborderentity.cpp \
    src/simulation.cpp \
    src/randomcubesimulation.cpp \
    src/experimenttrackinginterface.cpp \
    src/robustnessevalsimulation.cpp \
    src/sodaSimulationIslandManager.cpp \
    src/sodaSimulationIslandForeignerCache.cpp \
    src/sodaDynamicsWorld.cpp \
    src/sodaContactResultCallback.cpp \
    src/sodaLocalGridBroadphase.cpp \
    src/sodaRigidBody.cpp \
    src/sodaDynamicRigidBody.cpp \
    src/sodaPersistentForeignerManifoldArray.cpp \
    src/sodaMeshShapeCache.cpp \
    src/sodaMotionState.cpp \
    src/sodaLocalGrid.cpp \
    src/sodaEntity.cpp \
    src/sodaOgreWidget.cpp \
    src/sodaOgreResources.cpp \
    src/sodaDynamicEntity.cpp \
    src/sodaLogicWorld.cpp

LIBS += -lOgreMain \
    -lOgreTerrain \
    -lBulletDynamics \
    -lBulletCollision \
    -lBulletSoftBody \
	-lLinearMath \
    -lblitz

DEPENDPATH += .

win32 {
    INCLUDEPATH += src \
        $(OGRE_HOME)\include \
        $(BULLET_HOME)src
}
unix {
    INCLUDEPATH += src \
        /usr/local/include/bullet/ \
        /usr/include/bullet/ \
        /usr/include/qxt/ \
		/usr/local/include/OGRE/ \
		/usr/include/boost/ \
		/usr/include/blitz/ \
		/usr/include/OGRE/ \
		/usr/include/
}

FORMS += \
    src/mainwindow.ui

OTHER_FILES += \
    data/Plugins.cfg \
    data/resources.cfg
