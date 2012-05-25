/****************************************************************************
** Meta object code from reading C++ file 'ogrewidget.h'
**
** Created: Wed Apr 4 16:40:58 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../src/ogrewidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ogrewidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_OgreWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      30,   11,   11,   11, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_OgreWidget[] = {
    "OgreWidget\0\0ogreInitialized()\0"
    "onTimerTick()\0"
};

void OgreWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        OgreWidget *_t = static_cast<OgreWidget *>(_o);
        switch (_id) {
        case 0: _t->ogreInitialized(); break;
        case 1: _t->onTimerTick(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData OgreWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject OgreWidget::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_OgreWidget,
      qt_meta_data_OgreWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &OgreWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *OgreWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *OgreWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_OgreWidget))
        return static_cast<void*>(const_cast< OgreWidget*>(this));
    if (!strcmp(_clname, "Ogre::FrameListener"))
        return static_cast< Ogre::FrameListener*>(const_cast< OgreWidget*>(this));
    if (!strcmp(_clname, "Ogre::WindowEventListener"))
        return static_cast< Ogre::WindowEventListener*>(const_cast< OgreWidget*>(this));
    if (!strcmp(_clname, "OgreBites::SdkTrayListener"))
        return static_cast< OgreBites::SdkTrayListener*>(const_cast< OgreWidget*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int OgreWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void OgreWidget::ogreInitialized()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
