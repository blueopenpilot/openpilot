/****************************************************************************
** Meta object code from reading C++ file 'route_engine.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "route_engine.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'route_engine.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_RouteEngine_t {
    QByteArrayData data[16];
    char stringdata0[198];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RouteEngine_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RouteEngine_t qt_meta_stringdata_RouteEngine = {
    {
QT_MOC_LITERAL(0, 0, 11), // "RouteEngine"
QT_MOC_LITERAL(1, 12, 15), // "positionUpdated"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 19), // "QMapbox::Coordinate"
QT_MOC_LITERAL(4, 49, 8), // "position"
QT_MOC_LITERAL(5, 58, 7), // "bearing"
QT_MOC_LITERAL(6, 66, 12), // "routeUpdated"
QT_MOC_LITERAL(7, 79, 21), // "QList<QGeoCoordinate>"
QT_MOC_LITERAL(8, 101, 11), // "coordinates"
QT_MOC_LITERAL(9, 113, 11), // "routeUpdate"
QT_MOC_LITERAL(10, 125, 9), // "msgUpdate"
QT_MOC_LITERAL(11, 135, 15), // "routeCalculated"
QT_MOC_LITERAL(12, 151, 15), // "QGeoRouteReply*"
QT_MOC_LITERAL(13, 167, 5), // "reply"
QT_MOC_LITERAL(14, 173, 14), // "recomputeRoute"
QT_MOC_LITERAL(15, 188, 9) // "sendRoute"

    },
    "RouteEngine\0positionUpdated\0\0"
    "QMapbox::Coordinate\0position\0bearing\0"
    "routeUpdated\0QList<QGeoCoordinate>\0"
    "coordinates\0routeUpdate\0msgUpdate\0"
    "routeCalculated\0QGeoRouteReply*\0reply\0"
    "recomputeRoute\0sendRoute"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RouteEngine[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   49,    2, 0x06 /* Public */,
       6,    1,   54,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    0,   57,    2, 0x08 /* Private */,
      10,    0,   58,    2, 0x08 /* Private */,
      11,    1,   59,    2, 0x08 /* Private */,
      14,    0,   62,    2, 0x08 /* Private */,
      15,    0,   63,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Float,    4,    5,
    QMetaType::Void, 0x80000000 | 7,    8,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RouteEngine::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RouteEngine *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->positionUpdated((*reinterpret_cast< QMapbox::Coordinate(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 1: _t->routeUpdated((*reinterpret_cast< QList<QGeoCoordinate>(*)>(_a[1]))); break;
        case 2: _t->routeUpdate(); break;
        case 3: _t->msgUpdate(); break;
        case 4: _t->routeCalculated((*reinterpret_cast< QGeoRouteReply*(*)>(_a[1]))); break;
        case 5: _t->recomputeRoute(); break;
        case 6: _t->sendRoute(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (RouteEngine::*)(QMapbox::Coordinate , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RouteEngine::positionUpdated)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (RouteEngine::*)(QList<QGeoCoordinate> );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RouteEngine::routeUpdated)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject RouteEngine::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_RouteEngine.data,
    qt_meta_data_RouteEngine,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *RouteEngine::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RouteEngine::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RouteEngine.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int RouteEngine::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void RouteEngine::positionUpdated(QMapbox::Coordinate _t1, float _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void RouteEngine::routeUpdated(QList<QGeoCoordinate> _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
