/****************************************************************************
** Meta object code from reading C++ file 'cameraview.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "cameraview.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cameraview.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CameraViewWidget_t {
    QByteArrayData data[12];
    char stringdata0[160];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CameraViewWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CameraViewWidget_t qt_meta_stringdata_CameraViewWidget = {
    {
QT_MOC_LITERAL(0, 0, 16), // "CameraViewWidget"
QT_MOC_LITERAL(1, 17, 7), // "clicked"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 19), // "vipcThreadConnected"
QT_MOC_LITERAL(4, 46, 16), // "VisionIpcClient*"
QT_MOC_LITERAL(5, 63, 23), // "vipcThreadFrameReceived"
QT_MOC_LITERAL(6, 87, 10), // "VisionBuf*"
QT_MOC_LITERAL(7, 98, 13), // "vipcConnected"
QT_MOC_LITERAL(8, 112, 11), // "vipc_client"
QT_MOC_LITERAL(9, 124, 17), // "vipcFrameReceived"
QT_MOC_LITERAL(10, 142, 8), // "uint32_t"
QT_MOC_LITERAL(11, 151, 8) // "frame_id"

    },
    "CameraViewWidget\0clicked\0\0vipcThreadConnected\0"
    "VisionIpcClient*\0vipcThreadFrameReceived\0"
    "VisionBuf*\0vipcConnected\0vipc_client\0"
    "vipcFrameReceived\0uint32_t\0frame_id"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CameraViewWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,
       3,    1,   40,    2, 0x06 /* Public */,
       5,    2,   43,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   48,    2, 0x09 /* Protected */,
       9,    2,   51,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    2,
    QMetaType::Void, 0x80000000 | 6, QMetaType::UInt,    2,    2,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 4,    8,
    QMetaType::Void, 0x80000000 | 6, 0x80000000 | 10,    8,   11,

       0        // eod
};

void CameraViewWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CameraViewWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->clicked(); break;
        case 1: _t->vipcThreadConnected((*reinterpret_cast< VisionIpcClient*(*)>(_a[1]))); break;
        case 2: _t->vipcThreadFrameReceived((*reinterpret_cast< VisionBuf*(*)>(_a[1])),(*reinterpret_cast< quint32(*)>(_a[2]))); break;
        case 3: _t->vipcConnected((*reinterpret_cast< VisionIpcClient*(*)>(_a[1]))); break;
        case 4: _t->vipcFrameReceived((*reinterpret_cast< VisionBuf*(*)>(_a[1])),(*reinterpret_cast< uint32_t(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CameraViewWidget::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraViewWidget::clicked)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CameraViewWidget::*)(VisionIpcClient * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraViewWidget::vipcThreadConnected)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CameraViewWidget::*)(VisionBuf * , quint32 );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraViewWidget::vipcThreadFrameReceived)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CameraViewWidget::staticMetaObject = { {
    &QOpenGLWidget::staticMetaObject,
    qt_meta_stringdata_CameraViewWidget.data,
    qt_meta_data_CameraViewWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CameraViewWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CameraViewWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CameraViewWidget.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "QOpenGLFunctions"))
        return static_cast< QOpenGLFunctions*>(this);
    return QOpenGLWidget::qt_metacast(_clname);
}

int CameraViewWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QOpenGLWidget::qt_metacall(_c, _id, _a);
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
void CameraViewWidget::clicked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void CameraViewWidget::vipcThreadConnected(VisionIpcClient * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CameraViewWidget::vipcThreadFrameReceived(VisionBuf * _t1, quint32 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
