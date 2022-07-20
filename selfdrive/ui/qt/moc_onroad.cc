/****************************************************************************
** Meta object code from reading C++ file 'onroad.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "onroad.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'onroad.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_OnroadAlerts_t {
    QByteArrayData data[1];
    char stringdata0[13];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OnroadAlerts_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OnroadAlerts_t qt_meta_stringdata_OnroadAlerts = {
    {
QT_MOC_LITERAL(0, 0, 12) // "OnroadAlerts"

    },
    "OnroadAlerts"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OnroadAlerts[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void OnroadAlerts::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject OnroadAlerts::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_OnroadAlerts.data,
    qt_meta_data_OnroadAlerts,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *OnroadAlerts::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OnroadAlerts::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OnroadAlerts.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int OnroadAlerts::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_NvgWindow_t {
    QByteArrayData data[13];
    char stringdata0[142];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NvgWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NvgWindow_t qt_meta_stringdata_NvgWindow = {
    {
QT_MOC_LITERAL(0, 0, 9), // "NvgWindow"
QT_MOC_LITERAL(1, 10, 5), // "speed"
QT_MOC_LITERAL(2, 16, 9), // "speedUnit"
QT_MOC_LITERAL(3, 26, 8), // "setSpeed"
QT_MOC_LITERAL(4, 35, 10), // "speedLimit"
QT_MOC_LITERAL(5, 46, 13), // "is_cruise_set"
QT_MOC_LITERAL(6, 60, 18), // "has_eu_speed_limit"
QT_MOC_LITERAL(7, 79, 18), // "has_us_speed_limit"
QT_MOC_LITERAL(8, 98, 9), // "is_metric"
QT_MOC_LITERAL(9, 108, 10), // "engageable"
QT_MOC_LITERAL(10, 119, 8), // "dmActive"
QT_MOC_LITERAL(11, 128, 6), // "hideDM"
QT_MOC_LITERAL(12, 135, 6) // "status"

    },
    "NvgWindow\0speed\0speedUnit\0setSpeed\0"
    "speedLimit\0is_cruise_set\0has_eu_speed_limit\0"
    "has_us_speed_limit\0is_metric\0engageable\0"
    "dmActive\0hideDM\0status"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NvgWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
      12,   14, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // properties: name, type, flags
       1, QMetaType::Float, 0x00095003,
       2, QMetaType::QString, 0x00095003,
       3, QMetaType::Float, 0x00095003,
       4, QMetaType::Float, 0x00095003,
       5, QMetaType::Bool, 0x00095003,
       6, QMetaType::Bool, 0x00095003,
       7, QMetaType::Bool, 0x00095003,
       8, QMetaType::Bool, 0x00095003,
       9, QMetaType::Bool, 0x00095003,
      10, QMetaType::Bool, 0x00095003,
      11, QMetaType::Bool, 0x00095003,
      12, QMetaType::Int, 0x00095003,

       0        // eod
};

void NvgWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{

#ifndef QT_NO_PROPERTIES
    if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<NvgWindow *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< float*>(_v) = _t->speed; break;
        case 1: *reinterpret_cast< QString*>(_v) = _t->speedUnit; break;
        case 2: *reinterpret_cast< float*>(_v) = _t->setSpeed; break;
        case 3: *reinterpret_cast< float*>(_v) = _t->speedLimit; break;
        case 4: *reinterpret_cast< bool*>(_v) = _t->is_cruise_set; break;
        case 5: *reinterpret_cast< bool*>(_v) = _t->has_eu_speed_limit; break;
        case 6: *reinterpret_cast< bool*>(_v) = _t->has_us_speed_limit; break;
        case 7: *reinterpret_cast< bool*>(_v) = _t->is_metric; break;
        case 8: *reinterpret_cast< bool*>(_v) = _t->engageable; break;
        case 9: *reinterpret_cast< bool*>(_v) = _t->dmActive; break;
        case 10: *reinterpret_cast< bool*>(_v) = _t->hideDM; break;
        case 11: *reinterpret_cast< int*>(_v) = _t->status; break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<NvgWindow *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0:
            if (_t->speed != *reinterpret_cast< float*>(_v)) {
                _t->speed = *reinterpret_cast< float*>(_v);
            }
            break;
        case 1:
            if (_t->speedUnit != *reinterpret_cast< QString*>(_v)) {
                _t->speedUnit = *reinterpret_cast< QString*>(_v);
            }
            break;
        case 2:
            if (_t->setSpeed != *reinterpret_cast< float*>(_v)) {
                _t->setSpeed = *reinterpret_cast< float*>(_v);
            }
            break;
        case 3:
            if (_t->speedLimit != *reinterpret_cast< float*>(_v)) {
                _t->speedLimit = *reinterpret_cast< float*>(_v);
            }
            break;
        case 4:
            if (_t->is_cruise_set != *reinterpret_cast< bool*>(_v)) {
                _t->is_cruise_set = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 5:
            if (_t->has_eu_speed_limit != *reinterpret_cast< bool*>(_v)) {
                _t->has_eu_speed_limit = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 6:
            if (_t->has_us_speed_limit != *reinterpret_cast< bool*>(_v)) {
                _t->has_us_speed_limit = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 7:
            if (_t->is_metric != *reinterpret_cast< bool*>(_v)) {
                _t->is_metric = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 8:
            if (_t->engageable != *reinterpret_cast< bool*>(_v)) {
                _t->engageable = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 9:
            if (_t->dmActive != *reinterpret_cast< bool*>(_v)) {
                _t->dmActive = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 10:
            if (_t->hideDM != *reinterpret_cast< bool*>(_v)) {
                _t->hideDM = *reinterpret_cast< bool*>(_v);
            }
            break;
        case 11:
            if (_t->status != *reinterpret_cast< int*>(_v)) {
                _t->status = *reinterpret_cast< int*>(_v);
            }
            break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject NvgWindow::staticMetaObject = { {
    &CameraViewWidget::staticMetaObject,
    qt_meta_stringdata_NvgWindow.data,
    qt_meta_data_NvgWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *NvgWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NvgWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_NvgWindow.stringdata0))
        return static_cast<void*>(this);
    return CameraViewWidget::qt_metacast(_clname);
}

int NvgWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CameraViewWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    
#ifndef QT_NO_PROPERTIES
    if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 12;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 12;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 12;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 12;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 12;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
struct qt_meta_stringdata_OnroadWindow_t {
    QByteArrayData data[7];
    char stringdata0[62];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OnroadWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OnroadWindow_t qt_meta_stringdata_OnroadWindow = {
    {
QT_MOC_LITERAL(0, 0, 12), // "OnroadWindow"
QT_MOC_LITERAL(1, 13, 17), // "offroadTransition"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 7), // "offroad"
QT_MOC_LITERAL(4, 40, 11), // "updateState"
QT_MOC_LITERAL(5, 52, 7), // "UIState"
QT_MOC_LITERAL(6, 60, 1) // "s"

    },
    "OnroadWindow\0offroadTransition\0\0offroad\0"
    "updateState\0UIState\0s"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OnroadWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x08 /* Private */,
       4,    1,   27,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, 0x80000000 | 5,    6,

       0        // eod
};

void OnroadWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<OnroadWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->offroadTransition((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->updateState((*reinterpret_cast< const UIState(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject OnroadWindow::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_OnroadWindow.data,
    qt_meta_data_OnroadWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *OnroadWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OnroadWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OnroadWindow.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int OnroadWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
