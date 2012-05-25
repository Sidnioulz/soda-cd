#-------------------------------------------------
#
# Project created by QtCreator 2012-03-08T15:33:36
#
#-------------------------------------------------

QT       -= gui

TARGET = EKMeans
TEMPLATE = lib

DEFINES += EKMEANS_LIBRARY

SOURCES += libekmeans.cpp

HEADERS += libekmeans.h\
        EKMeans_global.h

symbian {
    MMP_RULES += EXPORTUNFROZEN
    TARGET.UID3 = 0xE4A63666
    TARGET.CAPABILITY = 
    TARGET.EPOCALLOWDLLDATA = 1
    addFiles.sources = EKMeans.dll
    addFiles.path = !:/sys/bin
    DEPLOYMENT += addFiles
}

LIBS +=  -lBulletDynamics \
    -lLinearMath

win32 {
    INCLUDEPATH += \
        $(BULLET_HOME)src
}
unix {
    INCLUDEPATH += \
        /usr/local/include/bullet/
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
