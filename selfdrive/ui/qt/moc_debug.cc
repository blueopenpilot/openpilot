/****************************************************************************
** Meta object code from reading C++ file 'debug.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "debug.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'debug.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DebugSettingPanel_t {
    QByteArrayData data[8];
    char stringdata0[90];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DebugSettingPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DebugSettingPanel_t qt_meta_stringdata_DebugSettingPanel = {
    {
QT_MOC_LITERAL(0, 0, 17), // "DebugSettingPanel"
QT_MOC_LITERAL(1, 18, 9), // "setVolume"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 6), // "volume"
QT_MOC_LITERAL(4, 36, 12), // "setBacklight"
QT_MOC_LITERAL(5, 49, 9), // "backlight"
QT_MOC_LITERAL(6, 59, 22), // "setAutoShowdownMinutes"
QT_MOC_LITERAL(7, 82, 7) // "minutes"

    },
    "DebugSettingPanel\0setVolume\0\0volume\0"
    "setBacklight\0backlight\0setAutoShowdownMinutes\0"
    "minutes"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugSettingPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x0a /* Public */,
       4,    1,   32,    2, 0x0a /* Public */,
       6,    1,   35,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    7,

       0        // eod
};

void DebugSettingPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DebugSettingPanel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setVolume((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->setBacklight((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->setAutoShowdownMinutes((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject DebugSettingPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_DebugSettingPanel.data,
    qt_meta_data_DebugSettingPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DebugSettingPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugSettingPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DebugSettingPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int DebugSettingPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
struct qt_meta_stringdata_DebugFeaturePanel_t {
    QByteArrayData data[1];
    char stringdata0[18];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DebugFeaturePanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DebugFeaturePanel_t qt_meta_stringdata_DebugFeaturePanel = {
    {
QT_MOC_LITERAL(0, 0, 17) // "DebugFeaturePanel"

    },
    "DebugFeaturePanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugFeaturePanel[] = {

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

void DebugFeaturePanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject DebugFeaturePanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_DebugFeaturePanel.data,
    qt_meta_data_DebugFeaturePanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DebugFeaturePanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugFeaturePanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DebugFeaturePanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int DebugFeaturePanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_DebugTestPanel_t {
    QByteArrayData data[1];
    char stringdata0[15];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DebugTestPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DebugTestPanel_t qt_meta_stringdata_DebugTestPanel = {
    {
QT_MOC_LITERAL(0, 0, 14) // "DebugTestPanel"

    },
    "DebugTestPanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugTestPanel[] = {

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

void DebugTestPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject DebugTestPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_DebugTestPanel.data,
    qt_meta_data_DebugTestPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DebugTestPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugTestPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DebugTestPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int DebugTestPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_DebugGeneralPanel_t {
    QByteArrayData data[1];
    char stringdata0[18];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DebugGeneralPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DebugGeneralPanel_t qt_meta_stringdata_DebugGeneralPanel = {
    {
QT_MOC_LITERAL(0, 0, 17) // "DebugGeneralPanel"

    },
    "DebugGeneralPanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugGeneralPanel[] = {

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

void DebugGeneralPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject DebugGeneralPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_DebugGeneralPanel.data,
    qt_meta_data_DebugGeneralPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DebugGeneralPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugGeneralPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DebugGeneralPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int DebugGeneralPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_DebugOsdPanel_t {
    QByteArrayData data[1];
    char stringdata0[14];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DebugOsdPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DebugOsdPanel_t qt_meta_stringdata_DebugOsdPanel = {
    {
QT_MOC_LITERAL(0, 0, 13) // "DebugOsdPanel"

    },
    "DebugOsdPanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugOsdPanel[] = {

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

void DebugOsdPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject DebugOsdPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_DebugOsdPanel.data,
    qt_meta_data_DebugOsdPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DebugOsdPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugOsdPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DebugOsdPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int DebugOsdPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_DebugLogPanel_t {
    QByteArrayData data[1];
    char stringdata0[14];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DebugLogPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DebugLogPanel_t qt_meta_stringdata_DebugLogPanel = {
    {
QT_MOC_LITERAL(0, 0, 13) // "DebugLogPanel"

    },
    "DebugLogPanel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugLogPanel[] = {

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

void DebugLogPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject DebugLogPanel::staticMetaObject = { {
    &ListWidget::staticMetaObject,
    qt_meta_stringdata_DebugLogPanel.data,
    qt_meta_data_DebugLogPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DebugLogPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugLogPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DebugLogPanel.stringdata0))
        return static_cast<void*>(this);
    return ListWidget::qt_metacast(_clname);
}

int DebugLogPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ListWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_DebugWindow_t {
    QByteArrayData data[3];
    char stringdata0[24];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DebugWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DebugWindow_t qt_meta_stringdata_DebugWindow = {
    {
QT_MOC_LITERAL(0, 0, 11), // "DebugWindow"
QT_MOC_LITERAL(1, 12, 10), // "closeDebug"
QT_MOC_LITERAL(2, 23, 0) // ""

    },
    "DebugWindow\0closeDebug\0"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DebugWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,

       0        // eod
};

void DebugWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DebugWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->closeDebug(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DebugWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DebugWindow::closeDebug)) {
                *result = 0;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject DebugWindow::staticMetaObject = { {
    &QFrame::staticMetaObject,
    qt_meta_stringdata_DebugWindow.data,
    qt_meta_data_DebugWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *DebugWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DebugWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DebugWindow.stringdata0))
        return static_cast<void*>(this);
    return QFrame::qt_metacast(_clname);
}

int DebugWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void DebugWindow::closeDebug()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
