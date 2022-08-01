/****************************************************************************
** Meta object code from reading C++ file 'remove_speed_camera.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "remove_speed_camera.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'remove_speed_camera.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_RemoveSpeedCameraWindow_t {
    QByteArrayData data[7];
    char stringdata0[94];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RemoveSpeedCameraWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RemoveSpeedCameraWindow_t qt_meta_stringdata_RemoveSpeedCameraWindow = {
    {
QT_MOC_LITERAL(0, 0, 23), // "RemoveSpeedCameraWindow"
QT_MOC_LITERAL(1, 24, 22), // "closeRemoveSpeedCamera"
QT_MOC_LITERAL(2, 47, 0), // ""
QT_MOC_LITERAL(3, 48, 10), // "MinusLimit"
QT_MOC_LITERAL(4, 59, 9), // "PlusLimit"
QT_MOC_LITERAL(5, 69, 6), // "Cancel"
QT_MOC_LITERAL(6, 76, 17) // "RemoveSpeedCamera"

    },
    "RemoveSpeedCameraWindow\0closeRemoveSpeedCamera\0"
    "\0MinusLimit\0PlusLimit\0Cancel\0"
    "RemoveSpeedCamera"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RemoveSpeedCameraWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   40,    2, 0x0a /* Public */,
       4,    0,   41,    2, 0x0a /* Public */,
       5,    0,   42,    2, 0x0a /* Public */,
       6,    0,   43,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RemoveSpeedCameraWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RemoveSpeedCameraWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->closeRemoveSpeedCamera(); break;
        case 1: _t->MinusLimit(); break;
        case 2: _t->PlusLimit(); break;
        case 3: _t->Cancel(); break;
        case 4: _t->RemoveSpeedCamera(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (RemoveSpeedCameraWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RemoveSpeedCameraWindow::closeRemoveSpeedCamera)) {
                *result = 0;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject RemoveSpeedCameraWindow::staticMetaObject = { {
    &QFrame::staticMetaObject,
    qt_meta_stringdata_RemoveSpeedCameraWindow.data,
    qt_meta_data_RemoveSpeedCameraWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *RemoveSpeedCameraWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RemoveSpeedCameraWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RemoveSpeedCameraWindow.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int RemoveSpeedCameraWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void RemoveSpeedCameraWindow::closeRemoveSpeedCamera()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
