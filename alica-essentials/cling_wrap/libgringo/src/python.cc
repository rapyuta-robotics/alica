#ifdef WITH_PYTHON

#include <Python.h>

#include "gringo/python.hh"
#include "gringo/value.hh"
#include "gringo/locatable.hh"
#include "gringo/logger.hh"
#include "gringo/control.hh"
#include <iostream>
#include <sstream>

namespace Gringo {

namespace {

// {{{ auxiliary functions and objects

struct Object {
    Object() : obj(nullptr)                               { }
    Object(PyObject *obj, bool inc = false) : obj(obj)    { if (inc) { Py_XINCREF(obj); } }
    Object(Object const &other) : Object(other.obj, true) { }
    Object(Object &&other) : Object(other.obj, false)     { other.obj = nullptr; }
    bool none() const                                     { return obj == Py_None; } 
    bool valid() const                                    { return obj; }
    PyObject *get() const                                 { return obj; }
    PyObject *release()                                   { PyObject *ret = obj; obj = nullptr; return ret; }
    PyObject *operator->() const                          { return get(); }
    operator bool() const                                 { return valid(); }
    operator PyObject*() const                            { return get(); }
    Object &operator=(Object const &other)                { Py_XDECREF(obj); obj = other.obj; Py_XINCREF(obj); return *this; }
    ~Object()                                             { Py_XDECREF(obj); }
    PyObject *obj;
};

struct PyUnblock {
    PyUnblock() : state(PyEval_SaveThread()) { }
    ~PyUnblock() { PyEval_RestoreThread(state); }
    PyThreadState *state;
};

struct PyBlock {
    PyBlock() : state(PyGILState_Ensure()) { }
    ~PyBlock() { PyGILState_Release(state); }
    PyGILState_STATE state;
};

Object pyExec(char const *str, char const *filename, PyObject *globals, PyObject *locals = Py_None) {
    if (locals == Py_None) { locals = globals; }
    Object x = Py_CompileString(str, filename, Py_file_input);
    if (!x) { return nullptr; }
    return PyEval_EvalCode((PyCodeObject*)x.get(), globals, locals);
}

bool pyToVal(Object obj, Value &val);
bool pyToVal(PyObject *obj, Value &val) { return pyToVal({obj, true}, val); }
bool pyToVals(Object obj, ValVec &vals);
bool pyToVals(PyObject *obj, ValVec &vals) { return pyToVals({obj, true}, vals); }

PyObject *valToPy(Value v);
template <class T>
PyObject *valsToPy(T const & vals);

template <typename T>
bool protect(T f) {
    try                             { f(); return true; }
    catch (std::bad_alloc const &e) { PyErr_SetString(PyExc_MemoryError, e.what()); }
    catch (std::exception const &e) { PyErr_SetString(PyExc_RuntimeError, e.what()); }
    catch (...)                     { PyErr_SetString(PyExc_RuntimeError, "unknown error"); }
    return false;
}

template <class T>
bool checkCmp(T *self, PyObject *b, int op) {
    if (b->ob_type == self->ob_type) { return true; }
    else {
        const char *ops = "<";
        switch (op) {
            case Py_LT: { ops = "<";  break; }
            case Py_LE: { ops = "<="; break; }
            case Py_EQ: { ops = "=="; break; }
            case Py_NE: { ops = "!="; break; }
            case Py_GT: { ops = ">";  break; }
            case Py_GE: { ops = ">="; break; }
        }
        PyErr_Format(PyExc_TypeError, "unorderable types: %s() %s %s()", self->ob_type->tp_name, ops, b->ob_type->tp_name);
        return false;
    }
}

template <class T>
PyObject *doCmp(T const &a, T const &b, int op) {
    switch (op) {
        case Py_LT: { if (a <  b) { Py_RETURN_TRUE; } else { Py_RETURN_FALSE; } }
        case Py_LE: { if (a <= b) { Py_RETURN_TRUE; } else { Py_RETURN_FALSE; } }
        case Py_EQ: { if (a == b) { Py_RETURN_TRUE; } else { Py_RETURN_FALSE; } }
        case Py_NE: { if (a != b) { Py_RETURN_TRUE; } else { Py_RETURN_FALSE; } }
        case Py_GT: { if (a >  b) { Py_RETURN_TRUE; } else { Py_RETURN_FALSE; } }
        case Py_GE: { if (a >= b) { Py_RETURN_TRUE; } else { Py_RETURN_FALSE; } }
    }
    Py_RETURN_FALSE;
}

std::string errorToString() {
    Object type, value, traceback;
    PyErr_Fetch(&type.obj, &value.obj, &traceback.obj);
    if (!type)        { PyErr_Clear(); return "  error during error handling"; }
    PyErr_NormalizeException(&type.obj, &value.obj, &traceback.obj);
    Object tbModule  = PyImport_ImportModule("traceback");
    if (!tbModule)    { PyErr_Clear(); return "  error during error handling"; }
    PyObject *tbDict = PyModule_GetDict(tbModule);
    if (!tbDict)      { PyErr_Clear(); return "  error during error handling"; }
    PyObject *tbFE   = PyDict_GetItemString(tbDict, "format_exception");
    if (!tbFE)        { PyErr_Clear(); return "  error during error handling"; }
    Object ret       = PyObject_CallFunction(tbFE, const_cast<char*>("OOO"), type.get(), value ? value.get() : Py_None, traceback ? traceback.get() : Py_None);
    if (!ret)         { PyErr_Clear(); return "  error during error handling"; }
    Object it        = PyObject_GetIter(ret);
    if (!it)          { PyErr_Clear(); return "  error during error handling"; }
    std::ostringstream oss;
    while (Object line = PyIter_Next(it)) {
        char const *msg = PyString_AsString(line);
        if (!msg) { break; }
        oss << "  " << msg;
    }
    if (PyErr_Occurred()) { PyErr_Clear(); return "  error during error handling"; }
    PyErr_Clear();
    return oss.str();
}
void handleError(Location const &loc, Warnings w, char const *msg) {
    std::string s = errorToString();
    GRINGO_REPORT(w)
        << loc << ": warning: " << msg << ":\n"
        << s
        ;
}

// }}}
// {{{ wrap Fun

struct Fun {
    PyObject_HEAD
    Value val;
    static PyTypeObject type;
    static PyMethodDef methods[];

    static PyObject *new_(PyTypeObject *type, PyObject *, PyObject *) {
        Fun *self;
        self = reinterpret_cast<Fun*>(type->tp_alloc(type, 0));
        if (!self) { return nullptr; }
        self->val = Value();
        return reinterpret_cast<PyObject*>(self);
    }

    static int init(Fun *self, PyObject *args, PyObject *kwds) {
        static char const *kwlist[] = {"name", "args", nullptr};
        char const *name;
        PyObject   *params = nullptr;
        if (!PyArg_ParseTupleAndKeywords(args, kwds, "s|O", const_cast<char**>(kwlist), &name, &params)) { return -1; }
        if (strcmp(name, "") == 0) {
            PyErr_SetString(PyExc_RuntimeError, "The name of a Fun object must not be empty");
            return -1;
        }
        if (params) {
            ValVec vals;
            if (!pyToVals(params, vals)) { return -1; }
            if (!protect([name, &vals, self]() { self->val = vals.empty() ? Value(name) : Value(name, vals); })) { return -1; }
        }
        else {
            if (!protect([name, self]() { self->val = Value(name); })) { return -1; }
        }
        return 0;
    }

    static PyObject *name(Fun *self) {
        // Note: should not throw
        return PyString_FromString((*FWString(self->val.name())).c_str());
    }

    static PyObject *args(Fun *self) {
        if (self->val.type() == Value::FUNC) {
            // Note: should not throw
            return valsToPy(self->val.args());
        }
        else {
            ValVec vals;
            return valsToPy(vals);
        }
    }

    static PyObject *str(Fun *self) {
        std::string s;
        protect([self, &s]() -> void { std::ostringstream oss; oss << self->val; s = oss.str(); });
        return PyString_FromString(s.c_str());
    }

    static long hash(Fun *self) {
        return self->val.hash();
    }

    static PyObject *cmp(Fun *self, PyObject *b, int op) {
        if (!checkCmp(self, b, op)) { return nullptr; }
        // Note: should not throw
        return doCmp(self->val, reinterpret_cast<Fun*>(b)->val, op);
    }
};

PyMethodDef Fun::methods[] = {
    {"name", (PyCFunction)Fun::name, METH_NOARGS, "name(self) -> string object\n\nReturn the name of the Fun object."},
    {"args", (PyCFunction)Fun::args, METH_NOARGS, "args(self) -> list object\n\nReturn the arguments of the Fun object."},
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject Fun::type = {
    PyObject_HEAD_INIT(nullptr)
    0,                                        // ob_size
    "gringo.Fun",                             // tp_name
    sizeof(Fun),                              // tp_basicsize
    0,                                        // tp_itemsize
    0,                                        // tp_dealloc
    0,                                        // tp_print
    0,                                        // tp_getattr
    0,                                        // tp_setattr
    0,                                        // tp_compare
    (reprfunc)str,                            // tp_repr
    0,                                        // tp_as_number
    0,                                        // tp_as_sequence
    0,                                        // tp_as_mapping
    (hashfunc)hash,                           // tp_hash
    0,                                        // tp_call
    (reprfunc)str,                            // tp_str
    0,                                        // tp_getattro
    0,                                        // tp_setattro
    0,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
R"(Fun(name, args) -> Fun object

Represents a gringo function term.

This also includes symbolic terms, which have to be created by either omitting
the arguments or passing an empty sequence.

Arguments:
name -- string representing the name of the function symbol
        (must follow gringo's identifier syntax)
args -- optional sequence of terms representing the arguments of the function
        symbol (Default: [])

Fun objects are ordered like in gringo and their string representation
corresponds to their gringo representation.)"
    ,                                         // tp_doc
    0,                                        // tp_traverse
    0,                                        // tp_clear
    (richcmpfunc)cmp,                         // tp_richcompare
    0,                                        // tp_weaklistoffset
    0,                                        // tp_iter
    0,                                        // tp_iternext
    methods,                                  // tp_methods
    0,                                        // tp_members
    0,                                        // tp_getset
    0,                                        // tp_base
    0,                                        // tp_dict
    0,                                        // tp_descr_get
    0,                                        // tp_descr_set
    0,                                        // tp_dictoffset
    (initproc)init,                           // tp_init
    0,                                        // tp_alloc
    new_,                                     // tp_new
    0,                                        // tp_free
    0,                                        // tp_is_gc
    0,                                        // tp_bases
    0,                                        // tp_mro
    0,                                        // tp_cache
    0,                                        // tp_subclasses
    0,                                        // tp_weaklist
    0,                                        // tp_del
    0,                                        // tp_version_tag
};

// }}}
// {{{ wrap Sup 

struct Sup {
    PyObject_HEAD
    static PyTypeObject type;

    static PyObject *new_(PyTypeObject *type, PyObject *, PyObject *) {
        return type->tp_alloc(type, 0);
    }

    static int init(Sup *, PyObject *args, PyObject *kwds) {
        static char const *kwlist[] = {nullptr};
        if (!PyArg_ParseTupleAndKeywords(args, kwds, "", const_cast<char **>(kwlist))) { return -1; }
        return 0;
    }

    static PyObject *str(Sup *, PyObject *, PyObject *) {
        return PyString_FromString("#sup");
    }

    static PyObject *cmp(Sup *self, PyObject *b, int op) {
        if (!checkCmp(self, b, op)) { return nullptr; }
        return doCmp(0, 0, op);
    }

    static long hash(Sup *) {
        return Value(false).hash();
    }

    static PyMethodDef methods[];
};

PyMethodDef Sup::methods[] = {
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject Sup::type = {
    PyObject_HEAD_INIT(nullptr)
    0,                                        // ob_size
    "gringo.Sup",                             // tp_name
    sizeof(Sup),                              // tp_basicsize
    0,                                        // tp_itemsize
    0,                                        // tp_dealloc
    0,                                        // tp_print
    0,                                        // tp_getattr
    0,                                        // tp_setattr
    0,                                        // tp_compare
    (reprfunc)str,                            // tp_repr
    0,                                        // tp_as_number
    0,                                        // tp_as_sequence
    0,                                        // tp_as_mapping
    (hashfunc)hash,                           // tp_hash
    0,                                        // tp_call
    (reprfunc)str,                            // tp_str
    0,                                        // tp_getattro
    0,                                        // tp_setattro
    0,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
R"(Sup() -> Sup object

Represents a gringo #sup term.

Sup objects are ordered like in gringo and their string representation
corresponds to their gringo representation.)"
    ,                                         // tp_doc
    0,                                        // tp_traverse
    0,                                        // tp_clear
    (richcmpfunc)cmp,                         // tp_richcompare
    0,                                        // tp_weaklistoffset
    0,                                        // tp_iter
    0,                                        // tp_iternext
    methods,                                  // tp_methods
    0,                                        // tp_members
    0,                                        // tp_getset
    0,                                        // tp_base
    0,                                        // tp_dict
    0,                                        // tp_descr_get
    0,                                        // tp_descr_set
    0,                                        // tp_dictoffset
    (initproc)init,                           // tp_init
    0,                                        // tp_alloc
    new_,                                     // tp_new
    0,                                        // tp_free
    0,                                        // tp_is_gc
    0,                                        // tp_bases
    0,                                        // tp_mro
    0,                                        // tp_cache
    0,                                        // tp_subclasses
    0,                                        // tp_weaklist
    0,                                        // tp_del
    0,                                        // tp_version_tag
};

// }}}
// {{{ wrap Inf

struct Inf {
    PyObject_HEAD
    static PyTypeObject type;

    static PyObject *new_(PyTypeObject *type, PyObject *, PyObject *) {
        return type->tp_alloc(type, 0);
    }

    static int init(Inf *, PyObject *args, PyObject *kwds) {
        static char const *kwlist[] = {nullptr};
        if (!PyArg_ParseTupleAndKeywords(args, kwds, "", const_cast<char **>(kwlist))) { return -1; }
        return 0;
    }

    static PyObject *str(Inf *, PyObject *, PyObject *) {
        return PyString_FromString("#inf");
    }

    static PyObject *cmp(Inf *self, PyObject *b, int op) {
        if (!checkCmp(self, b, op)) { return nullptr; }
        return doCmp(0, 0, op);
    }

    static long hash(Inf *) {
        return Value(true).hash();
    }

    static PyMethodDef methods[];
};

PyMethodDef Inf::methods[] = {
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject Inf::type = {
    PyObject_HEAD_INIT(nullptr)
    0,                                        // ob_size
    "gringo.Inf",                             // tp_name
    sizeof(Inf),                              // tp_basicsize
    0,                                        // tp_itemsize
    0,                                        // tp_dealloc
    0,                                        // tp_print
    0,                                        // tp_getattr
    0,                                        // tp_setattr
    0,                                        // tp_compare
    (reprfunc)str,                            // tp_repr
    0,                                        // tp_as_number
    0,                                        // tp_as_sequence
    0,                                        // tp_as_mapping
    (hashfunc)hash,                           // tp_hash
    0,                                        // tp_call
    (reprfunc)str,                            // tp_str
    0,                                        // tp_getattro
    0,                                        // tp_setattro
    0,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
R"(Inf() -> Inf object

Represents a gringo #inf term.

Inf objects are ordered like in gringo and their string representation
corresponds to their gringo representation.)"
    ,                                         // tp_doc
    0,                                        // tp_traverse
    0,                                        // tp_clear
    (richcmpfunc)cmp,                         // tp_richcompare
    0,                                        // tp_weaklistoffset
    0,                                        // tp_iter
    0,                                        // tp_iternext
    methods,                                  // tp_methods
    0,                                        // tp_members
    0,                                        // tp_getset
    0,                                        // tp_base
    0,                                        // tp_dict
    0,                                        // tp_descr_get
    0,                                        // tp_descr_set
    0,                                        // tp_dictoffset
    (initproc)init,                           // tp_init
    0,                                        // tp_alloc
    new_,                                     // tp_new
    0,                                        // tp_free
    0,                                        // tp_is_gc
    0,                                        // tp_bases
    0,                                        // tp_mro
    0,                                        // tp_cache
    0,                                        // tp_subclasses
    0,                                        // tp_weaklist
    0,                                        // tp_del
    0,                                        // tp_version_tag
};

// }}}
// {{{ wrap SolveResult

struct SolveResult {
    PyObject_HEAD
    Gringo::SolveResult ret;
    static PyTypeObject type;
    static PyMethodDef methods[];

    static PyObject *new_(Gringo::SolveResult ret) {
        SolveResult *self;
        self = reinterpret_cast<SolveResult*>(type.tp_alloc(&type, 0));
        if (!self) { return nullptr; }
        self->ret = ret;
        return reinterpret_cast<PyObject*>(self);
    }

    static PyObject *str(SolveResult *self) {
        switch (self->ret) {
            case Gringo::SolveResult::SAT:     { return PyString_FromString("SAT"); }
            case Gringo::SolveResult::UNSAT:   { return PyString_FromString("UNSAT"); }
            case Gringo::SolveResult::UNKNOWN: { return PyString_FromString("UNKNOWN"); }
        }
        return PyString_FromString("UNKNOWN");
    }

    static PyObject *get(Gringo::SolveResult ret) {
        PyObject *res = nullptr;
        switch (ret) {
            case Gringo::SolveResult::SAT:     { res = PyDict_GetItemString(type.tp_dict, "SAT"); break; }
            case Gringo::SolveResult::UNSAT:   { res = PyDict_GetItemString(type.tp_dict, "UNSAT"); break; }
            case Gringo::SolveResult::UNKNOWN: { res = PyDict_GetItemString(type.tp_dict, "UNKNOWN"); break; }
        }
        Py_XINCREF(res);
        return res;
    }

    static int addAttr() {
        Object sat(new_(Gringo::SolveResult::SAT), true);
        if (!sat) { return -1; }
        if (PyDict_SetItemString(type.tp_dict, "SAT", sat) < 0) { return -1; }
        Object unsat(new_(Gringo::SolveResult::UNSAT), true);
        if (!unsat) { return -1; }
        if (PyDict_SetItemString(type.tp_dict, "UNSAT", unsat) < 0) { return -1; }
        Object unknown(new_(Gringo::SolveResult::UNKNOWN), true);
        if (!unknown) { return -1; }
        if (PyDict_SetItemString(type.tp_dict, "UNKNOWN", unknown) < 0) { return -1; }
        return 0;
    }

    static long hash(SolveResult *self) {
        return static_cast<long>(self->ret);
    }

    static PyObject *cmp(SolveResult *self, PyObject *b, int op) {
        if (!checkCmp(self, b, op)) { return nullptr; }
        return doCmp(self->ret, reinterpret_cast<SolveResult*>(b)->ret, op);
    }
};

PyMethodDef SolveResult::methods[] = {
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject SolveResult::type = {
    PyObject_HEAD_INIT(nullptr)
    0,                                        // ob_size
    "gringo.SolveResult",                     // tp_name
    sizeof(SolveResult),                      // tp_basicsize
    0,                                        // tp_itemsize
    0,                                        // tp_dealloc
    0,                                        // tp_print
    0,                                        // tp_getattr
    0,                                        // tp_setattr
    0,                                        // tp_compare
    (reprfunc)str,                            // tp_repr
    0,                                        // tp_as_number
    0,                                        // tp_as_sequence
    0,                                        // tp_as_mapping
    (hashfunc)hash,                           // tp_hash
    0,                                        // tp_call
    (reprfunc)str,                            // tp_str
    0,                                        // tp_getattro
    0,                                        // tp_setattro
    0,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
R"(Captures the result of solve call.

SolveResult objects cannot be constructed from python. Instead the
preconstructed objects SolveResult.SAT, SolveResult.UNSAT, and
SolveResult.UNKNOWN have to be used.

SolveResult.SAT     -- solve call during which at least one model has been found.
SolveResult.UNSAT   -- solve call during which no model has been found.
SolveResult.UNKNOWN -- an interrupted solve call - e.g. by SolveFuture.interrupt, 
                       or a signal)"
    ,                                         // tp_doc
    0,                                        // tp_traverse
    0,                                        // tp_clear
    (richcmpfunc)cmp,                         // tp_richcompare
    0,                                        // tp_weaklistoffset
    0,                                        // tp_iter
    0,                                        // tp_iternext
    methods,                                  // tp_methods
    0,                                        // tp_members
    0,                                        // tp_getset
    0,                                        // tp_base
    0,                                        // tp_dict
    0,                                        // tp_descr_get
    0,                                        // tp_descr_set
    0,                                        // tp_dictoffset
    0,                                        // tp_init
    0,                                        // tp_alloc
    0,                                        // tp_new
    0,                                        // tp_free
    0,                                        // tp_is_gc
    0,                                        // tp_bases
    0,                                        // tp_mro
    0,                                        // tp_cache
    0,                                        // tp_subclasses
    0,                                        // tp_weaklist
    0,                                        // tp_del
    0,                                        // tp_version_tag
};

// }}}
// {{{ wrap Statistics

PyObject *getStatistics(Statistics const *stats, char const *prefix) {
    Statistics::Quantity ret(0);
    if (!protect([stats, prefix, &ret]{ ret = stats->getStat(prefix); })) { return nullptr; }
    switch (ret.error()) {
        case Statistics::error_none: { 
            double val = ret;
            return val == (int)val ? PyLong_FromDouble(val) : PyFloat_FromDouble(val);
        }
        case Statistics::error_not_available: {
            return PyErr_Format(PyExc_RuntimeError, "error_not_available: %s", prefix);
        }
        case Statistics::error_unknown_quantity: { 
            return PyErr_Format(PyExc_RuntimeError, "error_unknown_quantity: %s", prefix);
        }
        case Statistics::error_ambiguous_quantity: {
            char const *keys;
            if (!protect([stats, prefix, &keys]{ keys = stats->getKeys(prefix); })) { return nullptr; }
            if (!keys) { return PyErr_Format(PyExc_RuntimeError, "error zero keys string: %s", prefix); }
            if (strcmp(keys, "__len") == 0) {
                int len;
                if (!protect([stats, prefix, &len]() -> void {
                    std::string lenPrefix;
                    lenPrefix += prefix;
                    lenPrefix += "__len";
                    len = (int)(double)stats->getStat(lenPrefix.c_str()); 
                })) { return nullptr; }
                Object list = PyList_New(len);
                if (!list) { return nullptr; }
                for (int i = 0; i < len; ++i) {
                    Object objPrefix = PyString_FromFormat("%s%d.", prefix, i);
                    if (!objPrefix) { return nullptr; }
                    char const *subPrefix = PyString_AsString(objPrefix);
                    if (!subPrefix) { return nullptr; }
                    Object subStats = getStatistics(stats, subPrefix);
                    if (!subStats) { return nullptr; }
                    if (PyList_SetItem(list, i, subStats.release()) < 0) { return nullptr; }
                }
                return list.release();
            }
            else {
                Object dict = PyDict_New();
                if (!dict) { return nullptr; }
                for (char const *it = keys; *it; it+= strlen(it) + 1) {
                    int len = strlen(it);
                    Object key = PyString_FromStringAndSize(it, len - (it[len-1] == '.'));
                    if (!key) { return nullptr; }
                    Object objPrefix = PyString_FromFormat("%s%s", prefix, it);
                    if (!objPrefix) { return nullptr; }
                    char const *subPrefix = PyString_AsString(objPrefix);
                    if (!subPrefix) { return nullptr; }
                    Object subStats = getStatistics(stats, subPrefix);
                    if (!subStats) { return nullptr; }
                    if (PyDict_SetItem(dict, key, subStats) < 0) { return nullptr; }
                }
                return dict.release();
            }
        }
    }
    return PyErr_Format(PyExc_RuntimeError, "error unhandled prefix: %s", prefix);
}

// }}}
// {{{ wrap cmp

static PyObject *cmpVal(PyObject *, PyObject *args) {
    PyObject *a, *b;
    if (!PyArg_ParseTuple(args, "OO", &a, &b)) { return nullptr; }
    Value va, vb;
    if (!pyToVal(a, va)) { return nullptr; }
    if (!pyToVal(b, vb)) { return nullptr; }
    int ret;
    if (va == vb)     { ret =  0; }
    else if (va < vb) { ret = -1; }
    else              { ret =  1; }
    return PyInt_FromLong(ret);
}

// }}}
// {{{ wrap Model

struct Model {
    PyObject_HEAD
    Gringo::Model const *model;
    static PyTypeObject type;
    static PyMethodDef methods[];

    static PyObject *new_(Gringo::Model const &model) {
        Model *self;
        self = reinterpret_cast<Model*>(type.tp_alloc(&type, 0));
        if (!self) { return nullptr; }
        self->model = &model;
        return reinterpret_cast<PyObject*>(self);
    }
    static PyObject *contains(Model *self, PyObject *arg) {
        Value val;
        if(!pyToVal(arg, val)) { return nullptr; }
        bool ret;
        if (!protect([self, val, &ret]() { ret = self->model->contains(val); })) { return nullptr; }
        if (ret) { Py_RETURN_TRUE; } 
        else     { Py_RETURN_FALSE; }
    }
    static PyObject *atoms(Model *self, PyObject *args) {
        int atomset = Gringo::Model::SHOWN;
        if (!PyArg_ParseTuple(args, "|i", &atomset)) { return nullptr; }
        ValVec vals;
        if (!protect([self, &vals, atomset]() { vals = self->model->atoms(atomset); })) { return nullptr; }
        Object list = PyList_New(vals.size());
        if (!list) { return nullptr; }
        int i = 0;
        for (auto x : vals) { 
            Object val = valToPy(x);
            if (!val) { return nullptr; }
            if (PyList_SetItem(list, i, val.release()) < 0) { return nullptr; }
            ++i;
        }
        return list.release();
    }
    static PyObject *optimization(Model *self) {
        Int64Vec values(self->model->optimization());
        Object list = PyList_New(values.size());
        if (!list) { return nullptr; }
        int i = 0;
        for (auto x : values) {
            Object val = PyInt_FromLong(x);
            if (!val) { return nullptr; }
            if (PyList_SetItem(list, i, val.release()) < 0) { return nullptr; }
            ++i;
        }
        return list.release();
    }
    static PyObject *str(Model *self, PyObject *) {
        std::string s;
        if (!protect([self, &s]() -> void {
            auto printAtom = [](std::ostream &out, Value val) {
                if (val.type() == Value::FUNC && *val.sig() == Signature("$", 2)) { out << val.args().front() << "=" << val.args().back(); }
                else { out << val; }
            };
            std::ostringstream oss;
            print_comma(oss, self->model->atoms(Gringo::Model::SHOWN), " ", printAtom);
            s = oss.str();
        })) { return nullptr; }
        return PyString_FromString(s.c_str());
    }
    static int addAttr() {
        Object csp(PyInt_FromLong(Gringo::Model::CSP));
        if (!csp) { return -1; }
        if (PyDict_SetItemString(type.tp_dict, "CSP", csp) < 0) { return -1; }
        Object atoms(PyInt_FromLong(Gringo::Model::ATOMS));
        if (!atoms) { return -1; }
        if (PyDict_SetItemString(type.tp_dict, "ATOMS", atoms) < 0) { return -1; }
        Object terms(PyInt_FromLong(Gringo::Model::TERMS));
        if (!terms) { return -1; }
        if (PyDict_SetItemString(type.tp_dict, "TERMS", terms) < 0) { return -1; }
        Object shown(PyInt_FromLong(Gringo::Model::SHOWN));
        if (!shown) { return -1; }
        if (PyDict_SetItemString(type.tp_dict, "SHOWN", shown) < 0) { return -1; }
        return 0;
    }

};

PyMethodDef Model::methods[] = {
    {"atoms",    (PyCFunction)atoms,    METH_VARARGS, 
R"(atoms(self, atomset) -> list of terms
    
Return the list of atoms, terms, or CSP assignments in the model.

Argument atomset is used to select what kind of atoms or terms are returned:
Model.ATOMS -- selects all atoms in the model (independent of #show statements)
Model.TERMS -- selects all terms displayed with #show statements in the model
Model.SHOWN -- selects all atoms and terms as outputted by clingo's default
               output format
Model.CSP   -- selects all csp assignments (independent of #show statements)

The string representation of a model object is similar to the output of models
by clingo using the default output.

Note that atoms are represented using Fun objects, and that CSP assignments are
represented using function symbols with name "$" where the first argument is
the name of the CSP variable and the second its value.)"},
    {"contains", (PyCFunction)contains, METH_O,       
R"(contains(self, a) -> Boolean
    
Returns true if atom a is contained in the model.

Atom a must be represented using a Fun term.)"},
    {"optimization", (PyCFunction)optimization, METH_NOARGS,
R"(contains(self) -> [int]
    
Returns the list of optimization values of the model. This corresponds to
clasp's optimization output.)"},
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject Model::type = {
    PyObject_HEAD_INIT(nullptr)
    0,                                        // ob_size
    "gringo.Model",                           // tp_name
    sizeof(Model),                            // tp_basicsize
    0,                                        // tp_itemsize
    0,                                        // tp_dealloc
    0,                                        // tp_print
    0,                                        // tp_getattr
    0,                                        // tp_setattr
    0,                                        // tp_compare
    (reprfunc)str,                            // tp_repr
    0,                                        // tp_as_number
    0,                                        // tp_as_sequence
    0,                                        // tp_as_mapping
    0,                                        // tp_hash
    0,                                        // tp_call
    (reprfunc)str,                            // tp_str
    0,                                        // tp_getattro
    0,                                        // tp_setattro
    0,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
R"(Provides access to a model during a solve call.

Note that model objects cannot be constructed from python.  Instead they are
passed as argument to a model callback (see Control.solve and Control.asolve).
Furthermore, the lifetime of a model object is limited to the scope of the
callback. They must not be stored for later use in other places like - e.g.,
the main function.)"
    ,                                         // tp_doc
    0,                                        // tp_traverse
    0,                                        // tp_clear
    0,                                        // tp_richcompare
    0,                                        // tp_weaklistoffset
    0,                                        // tp_iter
    0,                                        // tp_iternext
    methods,                                  // tp_methods
    0,                                        // tp_members
    0,                                        // tp_getset
    0,                                        // tp_base
    0,                                        // tp_dict
    0,                                        // tp_descr_get
    0,                                        // tp_descr_set
    0,                                        // tp_dictoffset
    0,                                        // tp_init
    0,                                        // tp_alloc
    0,                                        // tp_new
    0,                                        // tp_free
    0,                                        // tp_is_gc
    0,                                        // tp_bases
    0,                                        // tp_mro
    0,                                        // tp_cache
    0,                                        // tp_subclasses
    0,                                        // tp_weaklist
    0,                                        // tp_del
    0,                                        // tp_version_tag
};

// }}}
// {{{ wrap SolveFuture

struct SolveFuture {
    PyObject_HEAD
    Gringo::SolveFuture *future;
    static PyTypeObject type;
    static PyMethodDef methods[];

    static PyObject *new_(Gringo::SolveFuture &future) {
        SolveFuture *self;
        self = reinterpret_cast<SolveFuture*>(type.tp_alloc(&type, 0));
        if (!self) { return nullptr; }
        self->future = &future;
        return reinterpret_cast<PyObject*>(self);
    }
    static PyObject *get(SolveFuture *self, PyObject *) {
        Gringo::SolveResult ret;
        if (!protect([self, &ret]() -> void { PyUnblock b; (void)b; ret = self->future->get(); })) { return nullptr; }
        return SolveResult::get(ret);
    }
    static PyObject *wait(SolveFuture *self, PyObject *args) {
        PyObject *timeout = nullptr;
        if (!PyArg_ParseTuple(args, "|O", &timeout)) { return nullptr; }
        if (!timeout) {
            if (!protect([self]() -> void { PyUnblock b; (void)b; self->future->wait(); })) { return nullptr; }
            Py_RETURN_NONE;
        }
        else {
            double time = PyFloat_AsDouble(timeout);
            if (PyErr_Occurred()) { return nullptr; }
            bool ret;
            if (!protect([self, time, &ret]() { PyUnblock b; (void)b; ret = self->future->wait(time); })) { return nullptr; }
            if (ret) { Py_RETURN_TRUE; } 
            else     { Py_RETURN_FALSE; }
        }
    }
    static PyObject *interrupt(SolveFuture *self, PyObject *) {
        if (!protect([self]() { PyUnblock b; (void)b; self->future->interrupt(); })) { return nullptr; }
        Py_RETURN_NONE;
    }
};

PyMethodDef SolveFuture::methods[] = {
    {"get",       (PyCFunction)get,       METH_NOARGS,  
R"(get(self) -> SolveResult object

Get the result of an asolve call. If the search is not completed yet, the
function blocks until the result is ready.)"},
    {"wait",      (PyCFunction)wait,      METH_VARARGS, 
R"(wait(self, timeout) -> None or Boolean

Wait for asolve call to finish. If a timeout is given, the function waits at
most timeout seconds and returns a Boolean indicating whether the search has
finished. Otherwise, the function blocks until the search is finished and
returns nothing.

Arguments:
timeout -- optional timeout in seconds 
           (permits floating point values))"},
    {"interrupt", (PyCFunction)interrupt, METH_NOARGS,  
R"(interrupt(self) -> None
    
Interrupts the running search.)"},
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject SolveFuture::type = {
    PyObject_HEAD_INIT(nullptr)
    0,                                        // ob_size
    "gringo.SolveFuture",                     // tp_name
    sizeof(SolveFuture),                      // tp_basicsize
    0,                                        // tp_itemsize
    0,                                        // tp_dealloc
    0,                                        // tp_print
    0,                                        // tp_getattr
    0,                                        // tp_setattr
    0,                                        // tp_compare
    0,                                        // tp_repr
    0,                                        // tp_as_number
    0,                                        // tp_as_sequence
    0,                                        // tp_as_mapping
    0,                                        // tp_hash
    0,                                        // tp_call
    0,                                        // tp_str
    0,                                        // tp_getattro
    0,                                        // tp_setattro
    0,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
R"(Handle for asynchronous solve calls.

SolveFuture objects cannot be created from python. Instead they are returned by
Control.asolve, which performs a search in the background.  A SolveFuture
object can be used to wait for such a background search or interrupt it.

See Control.asolve for an example.)"
    ,                                         // tp_doc
    0,                                        // tp_traverse
    0,                                        // tp_clear
    0,                                        // tp_richcompare
    0,                                        // tp_weaklistoffset
    0,                                        // tp_iter
    0,                                        // tp_iternext
    methods,                                  // tp_methods
    0,                                        // tp_members
    0,                                        // tp_getset
    0,                                        // tp_base
    0,                                        // tp_dict
    0,                                        // tp_descr_get
    0,                                        // tp_descr_set
    0,                                        // tp_dictoffset
    0,                                        // tp_init
    0,                                        // tp_alloc
    0,                                        // tp_new
    0,                                        // tp_free
    0,                                        // tp_is_gc
    0,                                        // tp_bases
    0,                                        // tp_mro
    0,                                        // tp_cache
    0,                                        // tp_subclasses
    0,                                        // tp_weaklist
    0,                                        // tp_del
    0,                                        // tp_version_tag
};

// }}}
// {{{ wrap Control

struct ControlWrap {
    PyObject_HEAD
    Gringo::Control *ctl;
    static PyTypeObject type;
    static PyMethodDef methods[];

    static bool checkBlocked(ControlWrap *self, char const *function) {
        if (self->ctl->blocked()) {
            PyErr_Format(PyExc_RuntimeError, "Control.%s must not be called during solve call", function);
            return false;
        }
        return true;
    }
    static PyObject *new_(Gringo::Control &ctl) {
        ControlWrap *self;
        self = reinterpret_cast<ControlWrap*>(type.tp_alloc(&type, 0));
        if (!self) { return nullptr; }
        self->ctl = &ctl;
        return reinterpret_cast<PyObject*>(self);
    }
    static PyObject *add(ControlWrap *self, PyObject *args) { 
        if (!checkBlocked(self, "add")) { return nullptr; }
        char     *name;
        PyObject *pyParams;
        char     *part;
        if (!PyArg_ParseTuple(args, "sOs", &name, &pyParams, &part)) { return nullptr; }
        FWStringVec params;
        Object it = PyObject_GetIter(pyParams);
        if (!it) { return nullptr; }
        while (Object pyVal = PyIter_Next(it)) {
            char const *val = PyString_AsString(pyVal);
            if (!val) { return nullptr; }
            if (!protect([val, &params]() { params.emplace_back(val); })) { return nullptr; }
        }
        if (!protect([self, name, &params, part]() { self->ctl->add(name, params, part); })) { return nullptr; }
        Py_RETURN_NONE;
    }
    static PyObject *ground(ControlWrap *self, PyObject *args) { 
        if (!checkBlocked(self, "ground")) { return nullptr; }
        char *part;
        PyObject *pyParams;
        if (!PyArg_ParseTuple(args, "sO", &part, &pyParams)) { return nullptr; }
        ValVec params;
        if (!pyToVals(pyParams, params)) { return nullptr; }
        if (!protect([self, part, &params]() { self->ctl->ground(part, params); })) { return nullptr; }
        Py_RETURN_NONE;
    }
    static PyObject *getConst(ControlWrap *self, PyObject *args) { 
        if (!checkBlocked(self, "getConst")) { return nullptr; }
        char *name;
        if (!PyArg_ParseTuple(args, "s", &name)) { return nullptr; }
        Value val;
        if (!protect([self, name, &val]() { val = self->ctl->getConst(name); })) { return nullptr; }
        if (val.type() == Value::SPECIAL) { Py_RETURN_NONE; }
        else { return valToPy(val); }
    }
    static bool onModel(Gringo::Model const &m, Object const &mh) {
        Object model(Model::new_(m), true);
        if (!model) {
            Location loc("<onModel>", 1, 1, "<onModel>", 1, 1);
            handleError(loc, Warnings::W_TERM_UNDEFINED, "error in model callback");
            throw std::runtime_error("error in model callback");
        }
        Object ret = PyObject_CallFunction(mh, const_cast<char*>("O"), model.get());
        if (!ret) {
            Location loc("<onModel>", 1, 1, "<onModel>", 1, 1);
            handleError(loc, Warnings::W_TERM_UNDEFINED, "error in model callback");
            throw std::runtime_error("error in model callback");
        }
        if (ret == Py_None)       { return true; }
        else if (ret == Py_True)  { return true; }
        else if (ret == Py_False) { return false; }
        else {
            PyErr_Format(PyExc_RuntimeError, "unexpected %s() object as result of onModel", ret->ob_type->tp_name);
            Location loc("<onModel>", 1, 1, "<onModel>", 1, 1);
            handleError(loc, Warnings::W_TERM_UNDEFINED, "error in model callback");
            throw std::runtime_error("error in model callback");
        }

    }
    static void onFinish(Gringo::SolveResult ret, bool interrupted, Object const &fh) {
        Object pyRet = SolveResult::get(ret);
        if (!pyRet) {
            Location loc("<onFinish>", 1, 1, "<onFinish>", 1, 1);
            handleError(loc, Warnings::W_TERM_UNDEFINED, "error in finish callback");
            throw std::runtime_error("error in finish callback");
        }
        Object pyInterrupted = PyBool_FromLong(interrupted);
        if (!pyInterrupted) {
            Location loc("<onFinish>", 1, 1, "<onFinish>", 1, 1);
            handleError(loc, Warnings::W_TERM_UNDEFINED, "error in finish callback");
            throw std::runtime_error("error in finish callback");
        }
        Object fhRet = PyObject_CallFunction(fh, const_cast<char*>("OO"), pyRet.get(), pyInterrupted.get());
        if (!fhRet) {
            Location loc("<onFinish>", 1, 1, "<onFinish>", 1, 1);
            handleError(loc, Warnings::W_TERM_UNDEFINED, "error in finish callback");
            throw std::runtime_error("error in finish callback");
        }
    }
    static PyObject *asolve(ControlWrap *self, PyObject *args, PyObject *kwds) {
        if (!checkBlocked(self, "asolve")) { return nullptr; }
        static char const *kwlist[] = {"onModel", "onFinish", nullptr};
        PyObject *mh = Py_None, *fh = Py_None;
        if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OO", const_cast<char **>(kwlist), &mh, &fh)) { return nullptr; }
        Gringo::SolveFuture *future;
        Object omh(mh, true);
        Object ofh(fh, true);
        if (!protect([self, omh, ofh, &future]() {
            future = (self->ctl->asolve(
                omh == Py_None ? Control::ModelHandler(nullptr) : [omh](Gringo::Model const &m) -> bool { PyBlock b; (void)b; return onModel(m, omh); },
                ofh == Py_None ? Control::FinishHandler(nullptr) : [ofh](Gringo::SolveResult ret, bool interrupted) -> void { PyBlock b; (void)b; onFinish(ret, interrupted, ofh); }
            ));
        })) { return nullptr; }
        PyObject *ret = SolveFuture::new_(*future);
        Py_XINCREF(ret);
        return ret;
    }
    static PyObject *solve(ControlWrap *self, PyObject *args, PyObject *kwds) {
        if (!checkBlocked(self, "solve")) { return nullptr; }
        static char const *kwlist[] = {"onModel", nullptr};
        PyObject *mh = Py_None;
        if (!PyArg_ParseTupleAndKeywords(args, kwds, "|O", const_cast<char **>(kwlist), &mh)) { return nullptr; }
        Gringo::SolveResult ret;
        if (!protect([self, mh, &ret]() {
            ret = (self->ctl->solve(
                mh == Py_None ? Control::ModelHandler(nullptr) : [mh](Gringo::Model const &m) { return onModel(m, Object(mh, true)); }
            ));
        })) { return nullptr; }
        return SolveResult::get(ret);
    }
    static PyObject *assignExternal(ControlWrap *self, PyObject *args) {
        if (!checkBlocked(self, "assignExternal")) { return nullptr; }
        PyObject *pyExt, *pyVal;
        if (!PyArg_ParseTuple(args, "OO", &pyExt, &pyVal)) { return nullptr; }
        bool val;
        if (pyVal == Py_True)     { val = true; }
        else if (pyVal == Py_False) { val = false; }
        else {
            PyErr_Format(PyExc_RuntimeError, "unexpected %s() object as second argumet", pyVal->ob_type->tp_name);
            return nullptr;
        }
        Value ext;
        if (!pyToVal(pyExt, ext)) { return nullptr; }
        if (!protect([self, ext, val]() { self->ctl->assignExternal(ext, val); })) { return nullptr; }
        Py_RETURN_NONE;
    }
    static PyObject *releaseExternal(ControlWrap *self, PyObject *args) {
        if (!checkBlocked(self, "releaseExternal")) { return nullptr; }
        PyObject *pyExt;
        if (!PyArg_ParseTuple(args, "O", &pyExt)) { return nullptr; }
        Value ext;
        if (!pyToVal(pyExt, ext)) { return nullptr; }
        if (!protect([self, ext]() { self->ctl->releaseExternal(ext); })) { return nullptr; }
        Py_RETURN_NONE;
    }
    static PyObject *getStats(ControlWrap *self, PyObject *) {
        if (!checkBlocked(self, "getStats")) { return nullptr; }
        Statistics *stats;
        if (!protect([self, &stats]() { stats = self->ctl->getStats(); })) { return nullptr; }
        return getStatistics(stats, "");
    }
    static PyObject *setConf(ControlWrap *self, PyObject *args) {
        if (!checkBlocked(self, "setConf")) { return nullptr; }
        PyObject *pyReplace = nullptr;
        char const *config;
        if (!PyArg_ParseTuple(args, "s|O", &config, &pyReplace)) { return nullptr; }
        bool replace = pyReplace && PyObject_IsTrue(pyReplace);
        if (!protect([self, config, replace]() { self->ctl->setConf(config, replace); })) { return nullptr; }
        Py_RETURN_NONE;
    }
    static PyObject *enableEnumAssumption(ControlWrap *self, PyObject *pyEnable) {
        if (!checkBlocked(self, "enableEnumAssumption")) { return nullptr; }
        bool enable = PyObject_IsTrue(pyEnable);
        if (!protect([self, enable]() { self->ctl->enableEnumAssumption(enable); })) { return nullptr; }
        Py_RETURN_NONE;
    }
};

PyMethodDef ControlWrap::methods[] = {
    // ground
    {"ground",         (PyCFunction)ground,         METH_VARARGS,                  
R"(ground(self, name, args) -> None

Grounds the program with the given name and arguments upon the next solve call.

Arguments:
name -- name of the program to ground
args -- list of terms to substitute program arguments with

Note that parts of a logic program without an explicit #program specification
are by default put into a program called base without arguments. Files included
with #include are treated as if the file was substituted for the include.

Example:

#script (python)
import gringo

def main(prg):
    prg.ground("p", [1])
    prg.ground("p", [2])
    prg.solve()

#end.

#program p(t).
q(t).

Expected Answer Set:
q(1) q(2))"},
    // getConst
    {"getConst",       (PyCFunction)getConst,       METH_VARARGS,                  
R"(getConst(self, name) -> Fun, integer, string, or tuple object

Returns the term for a constant definition of form: #const name = term.

Caveats:
If Control.add is used to extend a logic program, then contained constant
definitions will be made available after the next solve call.)"},
    // add
    {"add",            (PyCFunction)add,            METH_VARARGS,                  
R"(add(self, name, params, program) -> None

Extend the logic program with the given non-ground logic program in string form.

Arguments:
name    -- name of program block to add
params  -- parameters of program block
program -- non-ground program as string

Example:

#script (python)
import gringo

def main(prg):
    prg.add("p", ["t"], "q(t).")
    prg.ground("p", [2])
    prg.solve()

#end.

Expected Answer Set:
q(2))"},
    // asolve
    {"asolve",         (PyCFunction)asolve,         METH_KEYWORDS | METH_VARARGS,  
R"(asolve(self, onModel, onFinish) -> SolveFuture

Start a search process in the background and return a SolveFuture object.

Keyword Arguments:
onModel  -- optional callback for intercepting models
            a Model object is passed to the callback
onFinish -- optional callback once search has finished
            a SolveResult and a Boolean indicating whether the solve call has
            been interrupted is passed to the callback

Note that this function is only available in clingo with thread support
enabled. Both the onModel and the onFinish callbacks are called from another
thread.  To ensure the that the methods can be called, make sure to not use any
functions that block the GIL indefinitely. Furthermore, you might want to start
clingo using the --outf=3 option to disable all output from clingo.

Example:

#script (python)
import gringo

def onModel(model):
    print model

def onFinish(res, interrupted):
    print res, interrupted

def main(prg):
    prg.ground("base", [])
    f = prg.asolve(onModel, onFinish)
    f.wait()

#end.

q.

Expected Output:
q
SAT False)"},
    // solve
    {"solve",          (PyCFunction)solve,          METH_KEYWORDS | METH_VARARGS,  
R"(solve(self, onModel) -> SolveResult

Starts a search process and returns a SolveResult.

Arguments:
onModel -- optional callback for intercepting models
           a Model object is passed to the callback

Note that in gringo or in clingo with lparse or text output enabled this
functions just grounds and returns SolveResult.UNKNOWN. Furthermore, you might
want to start clingo using the --outf=3 option to disable all output from
clingo.

Take a look at Control.asolve for an example on how to use the model callback.)"},
    // assignExternal
    {"assignExternal", (PyCFunction)assignExternal, METH_VARARGS,                  
R"(assignExternal(self, external, truth) -> None

Assign a truth value to an external atom (represented as a term).

The truth value of an external atom can be changed before each solve call. An
atom is treated as external if it has been declared using an #external
directive, and has not been forgotten by calling Control.releaseExternal or
defined in a logic program with some rule. If the given atom is not external,
then the function has no effect.

Arguments:
external -- term representing the external atom
truth    -- Boolean indicating the truth value

See Control.releaseExternal for an example.)"},
    // releaseExternal
    {"releaseExternal", (PyCFunction)releaseExternal, METH_VARARGS,                  
R"(releaseExternal(self, term) -> None

Release an external represented by the given term.

This function causes the corresponding atom to become permanently false if
there is no definition for the atom in the program. In all other cases this
function has no effect.

Example:

#script (python)
from gringo import Fun

def main(prg):
    prg.ground("base", [])
    prg.assignExternal(Fun("b"), True)
    prg.solve()
    prg.releaseExternal(Fun("b"))
    prg.solve()

#end.

a.
#external b.

Expected Answer Sets:
a b
a)"},
    // getStats
    {"getStats",       (PyCFunction)getStats,       METH_NOARGS,
R"(getStats(self) -> dict object

Returns a dictionary containing the statistics of the last Control.solve or
Control.asolve call. The statistics correspond to the --stats output of clingo.
The detail of the statistics depends on what level is requested on the command
line. Furthermore, you might want to start clingo using the --outf=3 option to
disable all output from clingo.

Note that the function is only available in clingo.

Example:
import json
json.dumps(prg.getStats(), sort_keys=True, indent=4, separators=(',', ': ')))"},
    // setConf
    {"setConf",        (PyCFunction)setConf,        METH_VARARGS,                  
R"(setConf(self, options, update) -> None
    
Set the search configuration for the next solve calls.
    
All clasp library related options are supported (a few like --help, --outf, and
--lemma-in/--lemma-out are not). 

Arguments:
options -- string representing the options
update  -- optional Boolean that indicates whether the current configuration 
           should be updated (True) or replaced (False) (default: True)

Caveats: 
If you select a portfolio and afterwards increase the number of solve threads, then 
you have to pass in the portfolio again.)"},
    // enableEnumAssumption
    {"enableEnumAssumption",        (PyCFunction)enableEnumAssumption,        METH_O,                  
R"(enableEnumAssumption(self, enabled) -> None
    
Configures how learnt information from reasoning modes is treated.

If the enumeration assumption is enabled, then all information learnt from
clasp's various enumeration modes is removed after a solve call. This includes
enumeration of cautious or brave consequences, enumeration of answer sets with
or without projection, or finding optimal models.

Arguments:
enabled -- Boolean indicating whether the enumeration assumption is enabled

Note that initially the enumeration assumption is enabled.)"},
    {nullptr, nullptr, 0, nullptr}
};

PyTypeObject ControlWrap::type = {
    PyObject_HEAD_INIT(nullptr)
    0,                                        // ob_size
    "gringo.Control",                         // tp_name
    sizeof(ControlWrap),                      // tp_basicsize
    0,                                        // tp_itemsize
    0,                                        // tp_dealloc
    0,                                        // tp_print
    0,                                        // tp_getattr
    0,                                        // tp_setattr
    0,                                        // tp_compare
    0,                                        // tp_repr
    0,                                        // tp_as_number
    0,                                        // tp_as_sequence
    0,                                        // tp_as_mapping
    0,                                        // tp_hash
    0,                                        // tp_call
    0,                                        // tp_str
    0,                                        // tp_getattro
    0,                                        // tp_setattro
    0,                                        // tp_as_buffer
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, // tp_flags
R"(Control object for the grounding/solving process.

Note that a control object cannot be created from python. Instead it is passed
as argument to the main function. Furthermore, this object is blocked while a
search call is active; you must not call any member function during search.)"
    ,                                         // tp_doc
    0,                                        // tp_traverse
    0,                                        // tp_clear
    0,                                        // tp_richcompare
    0,                                        // tp_weaklistoffset
    0,                                        // tp_iter
    0,                                        // tp_iternext
    methods,                                  // tp_methods
    0,                                        // tp_members
    0,                                        // tp_getset
    0,                                        // tp_base
    0,                                        // tp_dict
    0,                                        // tp_descr_get
    0,                                        // tp_descr_set
    0,                                        // tp_dictoffset
    0,                                        // tp_init
    0,                                        // tp_alloc
    0,                                        // tp_new
    0,                                        // tp_free
    0,                                        // tp_is_gc
    0,                                        // tp_bases
    0,                                        // tp_mro
    0,                                        // tp_cache
    0,                                        // tp_subclasses
    0,                                        // tp_weaklist
    0,                                        // tp_del
    0,                                        // tp_version_tag
};

// }}}
// {{{ gringo module

static PyMethodDef gringoMethods[] = {
    {"cmp",  (PyCFunction)cmpVal, METH_VARARGS, 
R"(cmp(a, b) -> { -1, 0, 1 }

Compare terms a and b using gringo's inbuilt comparison function.

Returns:
    -1 if a < b, 
     0 if a = b, and 
     1 otherwise.)"},
    {nullptr, nullptr, 0, nullptr}
};

void initgringo() {
    if (PyType_Ready(&Sup::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    Py_INCREF(&Sup::type);
    if (PyType_Ready(&Inf::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    Py_INCREF(&Inf::type);
    if (PyType_Ready(&Fun::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    Py_INCREF(&Fun::type);
    if (PyType_Ready(&Model::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    if (Model::addAttr() < 0) { throw std::runtime_error("could not initialize gringo module"); }
    Py_INCREF(&Model::type);
    if (PyType_Ready(&SolveFuture::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    Py_INCREF(&SolveFuture::type);
    if (PyType_Ready(&SolveResult::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    Py_INCREF(&SolveResult::type);
    if (SolveResult::addAttr() < 0) { throw std::runtime_error("could not initialize gringo module"); };
    if (PyType_Ready(&ControlWrap::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    Py_INCREF(&ControlWrap::type);
    char const *strGrMod =
R"(The gringo module.

This module provides functions and classes to work with ground terms and to
control the instantiation process.  In clingo builts, additional functions to
control and inspect the solving process are available.

Functions defined in a python script block are callable during the
instantiation process using @-syntax. The default grounding/solving process can
be customized if a main function is provided.

Note that gringo terms are wrapped in python classes provided in this module.
For string terms, numbers, and tuples the respective inbuilt python classes are
used.  Functions called during the grounding process from the logic program
must either return a term or a sequence of terms.  If a sequence is returned,
the corresponding @-term is successively substituted by the values in the
sequence.

Functions:

cmp(a, b) -- compare terms a and b as gringo would

Classes:

Control     -- control object for the grounding/solving process
Fun         -- capture function terms - e.g., f, f(x), f(g(x)), etc.
Inf         -- capture #inf terms
Model       -- provides access to a model during solve call
SolveFuture -- handle for asynchronous solve calls
SolveResult -- result of a solve call
Sup         -- capture #sup terms

Example:

#script (python)
import gringo
def id(x):
    return x

def seq(x, y):
    return [x, y]

def main(prg):
    prg.ground("base", {})
    prg.solve()

#end.

p(@id(10)).
q(@seq(1,2)).
)";
    PyObject *m = Py_InitModule3("gringo", gringoMethods, strGrMod);
    if (!m) { throw std::runtime_error("could not initialize gringo module"); }
    if (PyModule_AddObject(m, "Sup",         (PyObject*)&Sup::type)         < 0) { throw std::runtime_error("could not initialize gringo module"); }
    if (PyModule_AddObject(m, "Inf",         (PyObject*)&Inf::type)         < 0) { throw std::runtime_error("could not initialize gringo module"); }
    if (PyModule_AddObject(m, "Fun",         (PyObject*)&Fun::type)         < 0) { throw std::runtime_error("could not initialize gringo module"); }
    if (PyModule_AddObject(m, "Model",       (PyObject*)&Model::type)       < 0) { throw std::runtime_error("could not initialize gringo module"); }
    if (PyModule_AddObject(m, "SolveFuture", (PyObject*)&SolveFuture::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    if (PyModule_AddObject(m, "SolveResult", (PyObject*)&SolveResult::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
    if (PyModule_AddObject(m, "Control",     (PyObject*)&ControlWrap::type) < 0) { throw std::runtime_error("could not initialize gringo module"); }
}

// }}}

// {{{ auxiliary functions and objects

PyObject *valToPy(Value v) {
    switch (v.type()) {
        case Value::FUNC: {
            if (*v.name() == "") {
                FWValVec args = v.args();
                Object tuple = PyTuple_New(args.size());
                if (!tuple) { return nullptr; }
                int i = 0;
                for (auto &val : args) {
                    Object pyVal = valToPy(val);
                    if (!pyVal) { return nullptr; }
                    if (PyTuple_SetItem(tuple, i, pyVal.release()) < 0) { return nullptr; }
                    ++i;
                }
                return tuple.release();
            }
        }
        case Value::ID: {
            Object fun(Fun::new_(&Fun::type, nullptr, nullptr), true);
            if (!fun) { return nullptr; }
            reinterpret_cast<Fun*>(fun.get())->val = v;
            return fun.release();
        }
        case Value::SUP: {
            PyObject *ret = Sup::new_(&Sup::type, nullptr, nullptr);
            Py_XINCREF(ret);
            return ret;
        }
        case Value::INF: {
            PyObject *ret = Inf::new_(&Inf::type, nullptr, nullptr);
            Py_XINCREF(ret);
            return ret;
        }
        case Value::NUM: {
            return PyInt_FromLong(v.num());
        }
        case Value::STRING: {
            return PyString_FromString((*v.string()).c_str());
        }
        default: { 
            PyErr_SetString(PyExc_RuntimeError, "cannot happen");
            return nullptr;
        }
    }

}

bool pyToVal(Object obj, Value &val) {
    if (obj->ob_type == &Sup::type) {
        val = Value(false);
    }
    else if (obj->ob_type == &Inf::type) {
        val = Value(true);
    }
    else if (obj->ob_type == &Fun::type) {
        val = reinterpret_cast<Fun*>(obj.get())->val;
    }
    else if (PyTuple_Check(obj)) {
        ValVec vals;
        if (!pyToVals(obj, vals)) { return false; }
        if (vals.size() < 2) {
            PyErr_SetString(PyExc_RuntimeError, "cannot convert to value: tuples need at least two arguments");
        }
        if (!protect([&val, &vals]() { val = Value("", vals); })) { return false; }
    }
    else if (PyInt_Check(obj)) {
        val = Value(int(PyInt_AsLong(obj)));
    }
    else if (PyString_Check(obj)) {
        char const *value = PyString_AsString(obj);
        if (!protect([value, &val]() { val = Value(value, false); })) { return false; }
    }
    else {
        PyErr_Format(PyExc_RuntimeError, "cannot convert to value: unexpected %s() object", obj->ob_type->tp_name);
        return false;
    }
    return true;
}

template <class T>
PyObject *valsToPy(T const &vals) {
    Object list = PyList_New(vals.size());
    if (!list) { return nullptr; }
    int i = 0;
    for (auto &val : vals) {
        Object pyVal = valToPy(val);
        if (!pyVal) { return nullptr; }
        if (PyList_SetItem(list, i, pyVal.release()) < 0) { return nullptr; }
        ++i;
    }
    return list.release();
}

bool pyToVals(Object obj, ValVec &vals) {
    Object it = PyObject_GetIter(obj);
    if (!it) { return false; }
    while (Object pyVal = PyIter_Next(it)) {
        Value val;
        if (!pyToVal(pyVal, val)) { return false; }
        if (!protect([val, &vals]() { vals.emplace_back(val); })) { return false; }
    }
    return true;

}

// }}}

} // namespace

// {{{ definition of PythonImpl

struct PythonInit {
    PythonInit()  { Py_Initialize(); }
    ~PythonInit() { Py_Finalize(); }
};

struct PythonImpl {
    PythonImpl() {
        static char const *argv[] = {""};
        PySys_SetArgvEx(1, const_cast<char**>(argv), 0);
        if (!PyEval_ThreadsInitialized()) { PyEval_InitThreads(); }
        initgringo();
        mainModule = PyImport_ImportModule("__main__");
        if (!mainModule) { throw std::runtime_error("could not initialize python interpreter"); }
        main       = PyModule_GetDict(mainModule);
        if (!main)       { throw std::runtime_error("could not initialize python interpreter"); }
    }
    bool exec(Location const &loc, FWString code) {
        std::ostringstream oss;
        oss << "<" << loc << ">";
        if (!pyExec((*code).c_str(), oss.str().c_str(), main)) { return false; }
        return true;
    }
    bool callable(FWString name) {
        Object fun = PyMapping_GetItemString(main, const_cast<char*>((*name).c_str()));
        PyErr_Clear();
        bool ret = fun && PyCallable_Check(fun);
        return ret;
    }
    bool call(FWString name, ValVec const &args, ValVec &vals) {
        Object params = PyTuple_New(args.size());
        if (!params) { return false; }
        int i = 0;
        for (auto &val : args) {
            Object pyVal = valToPy(val);
            if (!pyVal) { return false; }
            if (PyTuple_SetItem(params, i, pyVal.release()) < 0) { return false; }
            ++i;
        }
        Object fun = PyMapping_GetItemString(main, const_cast<char*>((*name).c_str()));
        if (!fun) { return false; }
        Object ret = PyObject_Call(fun, params, Py_None);
        if (!ret) { return false; }
        if (PyList_Check(ret)) {
            if (!pyToVals(ret, vals)) { return false; }
        }
        else {
            Value val;
            if (!pyToVal(ret, val)) { return false; }
            vals.emplace_back(val);
        }
        return true;
    }
    bool call(Gringo::Control &ctl) {
        Object fun = PyMapping_GetItemString(main, const_cast<char*>("main"));
        if (!fun) { return false; }
        Object params = PyTuple_New(1);
        if (!params) { return false; }
        Object param(ControlWrap::new_(ctl), true);
        if (!param) { return false; }
        if (PyTuple_SetItem(params, 0, param) < 0) { return false; }
        Object ret = PyObject_Call(fun, params, Py_None);
        if (!ret) { return false; }
        return true;
    }
    PythonInit init;
    Object     mainModule;
    PyObject  *main;
};

// }}}
// {{{ definition of Python

std::unique_ptr<PythonImpl> Python::impl = nullptr;

Python::Python() = default;
bool Python::exec(Location const &loc, FWString code) {
    if (!impl) { impl = make_unique<PythonImpl>(); }
    if (!impl->exec(loc, code)) {
        handleError(loc, W_TERM_UNDEFINED, "parsing failed");
        return false;
    }
    return true;
}
bool Python::callable(FWString name) {
    return impl && impl->callable(name);
}
ValVec Python::call(Location const &loc, FWString name, ValVec const &args) {
    assert(impl);
    ValVec vals;    
    if (!impl->call(name, args, vals)) {
        handleError(loc, W_TERM_UNDEFINED, "operation undefined, a zero is substituted");
        return {0};
    }
    return vals;
}
void Python::main(Gringo::Control &ctl) {
    assert(impl);
    if (!impl->call(ctl)) {
        Location loc("<internal>", 1, 1, "<internal>", 1, 1);
        handleError(loc, W_TERM_UNDEFINED, "error while calling main function");
        return;
    }
}
Python::~Python() = default;

// }}}

} // namespace Gringo

#else // WITH_PYTHON

#include "gringo/python.hh"
#include "gringo/value.hh"
#include "gringo/locatable.hh"
#include "gringo/logger.hh"

namespace Gringo {

// {{{ definition of Python

struct PythonImpl { };

std::unique_ptr<PythonImpl> Python::impl = nullptr;

Python::Python() = default;
bool Python::exec(Location const &loc, FWString ) {
    GRINGO_REPORT(W_TERM_UNDEFINED)
        << loc << ": warning: gringo has been build without python support, code is ignored\n"
        ;
    return false;
}
bool Python::callable(FWString ) {
    return false;
}
ValVec Python::call(Location const &, FWString , ValVec const &) {
    return {0};
}
void Python::main(Control &) { }
Python::~Python() = default;

// }}}

} // namespace Gringo

#endif // WITH_PYTHON
