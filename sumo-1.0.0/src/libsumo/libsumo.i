%module libsumo
#pragma SWIG nowarn=511

#ifdef SWIGPYTHON
%rename(edge) Edge;
%rename(inductionloop) InductionLoop;
%rename(junction) Junction;
%rename(lane) Lane;
%rename(lanearea) LaneArea;
%rename(multientryexit) MultiEntryExit;
%rename(person) Person;
%rename(poi) POI;
%rename(polygon) Polygon;
%rename(route) Route;
%rename(simulation) Simulation;
%rename(trafficlight) TrafficLight;
%rename(vehicle) Vehicle;
%rename(vehicletype) VehicleType;

// adding dummy init and close for easier traci -> libsumo transfer
%pythoncode %{
from traci import constants, exceptions, _vehicle, _person

def isLibsumo():
    return True

def init(port):
    print("Warning! To make your code usable with traci and libsumo, please use traci.start instead of traci.init.")

def close():
    simulation.close()

def start(args):
    simulation.load(args[1:])

def simulationStep(step=0):
    simulation.step(step)
%}

/* There is currently no TraCIPosition used as input so this is only for future usage
%typemap(in) const libsumo::TraCIPosition& (libsumo::TraCIPosition pos) {
    const Py_ssize_t size = PySequence_Size($input);
    if (size == 2 || size == 3) {
        pos.x = PyFloat_AsDouble(PySequence_GetItem($input, 0));
        pos.y = PyFloat_AsDouble(PySequence_GetItem($input, 1));
        pos.z = (size == 3 ? PyFloat_AsDouble(PySequence_GetItem($input, 2)) : 0.);
    } else {
    // TODO error handling
    }
    $1 = &pos;
}
*/

%typemap(in) const libsumo::TraCIPositionVector& (libsumo::TraCIPositionVector shape) {
    const Py_ssize_t size = PySequence_Size($input);
    for (Py_ssize_t i = 0; i < size; i++) {
        PyObject* posTuple = PySequence_GetItem($input, i);
        const Py_ssize_t posSize = PySequence_Size(posTuple);
        libsumo::TraCIPosition pos;
        if (posSize == 2 || posSize == 3) {
            PyObject* item = PySequence_GetItem(posTuple, 0);
            pos.x = PyFloat_Check(item) ? PyFloat_AsDouble(item) : PyLong_AsDouble(item);
            item = PySequence_GetItem(posTuple, 1);
            pos.y = PyFloat_Check(item) ? PyFloat_AsDouble(item) : PyLong_AsDouble(item);
			pos.z = 0.;
			if (posSize == 3) {
                item = PySequence_GetItem(posTuple, 2);
                pos.z = PyFloat_Check(item) ? PyFloat_AsDouble(item) : PyLong_AsDouble(item);
			}
        } else {
        // TODO error handling
        }
        shape.push_back(pos);
    }
    $1 = &shape;
}

%typemap(in) const libsumo::TraCIColor& (libsumo::TraCIColor col) {
    const Py_ssize_t size = PySequence_Size($input);
    if (size == 3 || size == 4) {
        col.r = (unsigned char)PyLong_AsLong(PySequence_GetItem($input, 0));
        col.g = (unsigned char)PyLong_AsLong(PySequence_GetItem($input, 1));
        col.b = (unsigned char)PyLong_AsLong(PySequence_GetItem($input, 2));
        col.a = (unsigned char)(size == 4 ? PyLong_AsLong(PySequence_GetItem($input, 3)) : 255);
    } else {
    // TODO error handling
    }
    $1 = &col;
}

%typemap(in) const std::vector<int>& (std::vector<int> vars) {
    const Py_ssize_t size = PySequence_Size($input);
    for (Py_ssize_t i = 0; i < size; i++) {
        vars.push_back(PyLong_AsLong(PySequence_GetItem($input, i)));
    }
    $1 = &vars;
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_INTEGER) const std::vector<int>& {
    $1 = PySequence_Check($input) ? 1 : 0;
}


%typemap(out) std::map<int, std::shared_ptr<libsumo::TraCIResult> > {
    $result = PyDict_New();
    for (auto iter = $1.begin(); iter != $1.end(); ++iter) {
        const int theKey = iter->first;
        const libsumo::TraCIResult* const theVal = iter->second.get();
        const libsumo::TraCIDouble* const theDouble = dynamic_cast<const libsumo::TraCIDouble*>(theVal);
        if (theDouble != nullptr) {
            PyDict_SetItem($result, PyInt_FromLong(theKey), PyFloat_FromDouble(theDouble->value));
            continue;
        }
        const libsumo::TraCIInt* const theInt = dynamic_cast<const libsumo::TraCIInt*>(theVal);
        if (theInt != nullptr) {
            PyDict_SetItem($result, PyInt_FromLong(theKey), PyInt_FromLong(theInt->value));
            continue;
        }
        const libsumo::TraCIString* const theString = dynamic_cast<const libsumo::TraCIString*>(theVal);
        if (theString != nullptr) {
            PyDict_SetItem($result, PyInt_FromLong(theKey), PyUnicode_FromString(theString->value.c_str()));
            continue;
        }
        const libsumo::TraCIStringList* const theStringList = dynamic_cast<const libsumo::TraCIStringList*>(theVal);
        if (theStringList != nullptr) {
            const Py_ssize_t size = theStringList->value.size();
            PyObject* tuple = PyTuple_New(size);
            for (Py_ssize_t i = 0; i < size; i++) {
                PyTuple_SetItem(tuple, i, PyUnicode_FromString(theStringList->value[i].c_str()));
            }
            PyDict_SetItem($result, PyInt_FromLong(theKey), tuple);
            continue;
        }
        PyObject *value = SWIG_NewPointerObj(SWIG_as_voidptr(theVal), SWIGTYPE_p_libsumo__TraCIResult, 0);
        PyDict_SetItem($result, PyInt_FromLong(theKey), value);
    }
};

%typemap(out) libsumo::TraCIPosition {
    if ($1.z != - 1024 * 1024 * 1024) { // see Position::INVALID
        $result = PyTuple_Pack(3, PyFloat_FromDouble($1.x), PyFloat_FromDouble($1.y), PyFloat_FromDouble($1.z));
    } else {
        $result = PyTuple_Pack(2, PyFloat_FromDouble($1.x), PyFloat_FromDouble($1.y));
    }
};

%typemap(out) libsumo::TraCIPositionVector {
    $result = PyTuple_New($1.size());
    int index = 0;
    for (auto iter = $1.begin(); iter != $1.end(); ++iter) {
        PyTuple_SetItem($result, index++, PyTuple_Pack(2, PyFloat_FromDouble(iter->x), PyFloat_FromDouble(iter->y)));
    }
};

%typemap(out) libsumo::TraCIColor {
    $result = PyTuple_Pack(4, PyLong_FromLong($1.r), PyLong_FromLong($1.g), PyLong_FromLong($1.b), PyLong_FromLong($1.a));
};

%typemap(out) libsumo::TraCIRoadPosition {
    $result = PyTuple_Pack(3, PyUnicode_FromString($1.edgeID.c_str()), PyFloat_FromDouble($1.pos), PyLong_FromLong($1.laneIndex));
};

%typemap(out) std::vector<libsumo::TraCIConnection> {
    $result = PyList_New($1.size());
    int index = 0;
    for (auto iter = $1.begin(); iter != $1.end(); ++iter) {
        PyList_SetItem($result, index++, PyTuple_Pack(8, PyUnicode_FromString(iter->approachedLane.c_str()),
                                                         PyBool_FromLong(iter->hasPrio),
                                                         PyBool_FromLong(iter->isOpen),
                                                         PyBool_FromLong(iter->hasFoe),
                                                         PyUnicode_FromString(iter->approachedInternal.c_str()),
                                                         PyUnicode_FromString(iter->state.c_str()),
                                                         PyUnicode_FromString(iter->direction.c_str()),
                                                         PyFloat_FromDouble(iter->length)));
    }
};

%typemap(out) std::vector<libsumo::TraCIVehicleData> {
    $result = PyList_New($1.size());
    int index = 0;
    for (auto iter = $1.begin(); iter != $1.end(); ++iter) {
        PyList_SetItem($result, index++, PyTuple_Pack(5, PyUnicode_FromString(iter->id.c_str()),
                                                         PyFloat_FromDouble(iter->length),
                                                         PyFloat_FromDouble(iter->entryTime),
                                                         PyFloat_FromDouble(iter->leaveTime),
                                                         PyUnicode_FromString(iter->typeID.c_str())));
    }
};

%typemap(out) std::vector<libsumo::TraCIBestLanesData> {
    $result = PyTuple_New($1.size());
    int index = 0;
    for (auto iter = $1.begin(); iter != $1.end(); ++iter) {
	    const int size = (int)iter->continuationLanes.size();
        auto nextLanes = PyTuple_New(size);
        for (int i = 0; i < size; i++) {
            PyTuple_SetItem(nextLanes, i, PyUnicode_FromString(iter->continuationLanes[i].c_str()));
		}
        PyTuple_SetItem($result, index++, PyTuple_Pack(6, PyUnicode_FromString(iter->laneID.c_str()),
                                                          PyFloat_FromDouble(iter->length),
                                                          PyFloat_FromDouble(iter->occupation),
                                                          PyFloat_FromDouble(iter->bestLaneOffset),
                                                          PyBool_FromLong(iter->allowsContinuation),
                                                          nextLanes));
    }
};

%typemap(out) std::vector<libsumo::TraCINextTLSData> {
    $result = PyTuple_New($1.size());
    int index = 0;
    for (auto iter = $1.begin(); iter != $1.end(); ++iter) {
        PyTuple_SetItem($result, index++, PyTuple_Pack(4, PyUnicode_FromString(iter->id.c_str()),
                                                          PyLong_FromLong(iter->tlIndex),
                                                          PyFloat_FromDouble(iter->dist),
                                                          PyUnicode_FromStringAndSize(&iter->state, 1)));
    }
};

%typemap(out) std::pair<int, int> {
    $result = PyTuple_Pack(2, PyLong_FromLong($1.first), PyLong_FromLong($1.second));
};

%typemap(out) std::pair<std::string, double> {
    $result = PyTuple_Pack(2, PyUnicode_FromString($1.first.c_str()), PyFloat_FromDouble($1.second));
};

%exceptionclass libsumo::TraCIException;

#endif

%begin %{
#ifdef _MSC_VER
// ignore constant conditional expression warnings
#pragma warning(disable:4127)
#endif

#include <libsumo/TraCIDefs.h>
%}


// replacing vector instances of standard types, see https://stackoverflow.com/questions/8469138
%include "std_string.i"
%include "std_vector.i"
%template(StringVector) std::vector<std::string>;
%template(TraCIPhaseVector) std::vector<libsumo::TraCIPhase>;
%template(TraCIStageVector) std::vector<libsumo::TraCIStage>;

// exception handling
%include "exception.i"

// taken from here https://stackoverflow.com/questions/1394484/how-do-i-propagate-c-exceptions-to-python-in-a-swig-wrapper-library
%exception { 
    try {
        $action
    } catch (libsumo::TraCIException &e) {
        const std::string s = std::string("Error: ") + e.what();
#ifdef SWIGPYTHON
        PyErr_SetObject(SWIG_Python_ExceptionType(SWIGTYPE_p_libsumo__TraCIException), PyUnicode_FromString(s.c_str()));
        SWIG_fail;
#else
        SWIG_exception(SWIG_ValueError, s.c_str());
#endif
    } catch (std::runtime_error &e) {
        const std::string s = std::string("SUMO error: ") + e.what();
        SWIG_exception(SWIG_RuntimeError, s.c_str());
    } catch (...) {
        SWIG_exception(SWIG_UnknownError, "unknown exception");
    }
}

// %feature("compactdefaultargs") libsumo::Simulation::findRoute;

// Add necessary symbols to generated header
%{
#include <libsumo/Edge.h>
#include <libsumo/InductionLoop.h>
#include <libsumo/Junction.h>
#include <libsumo/LaneArea.h>
#include <libsumo/Lane.h>
#include <libsumo/MultiEntryExit.h>
#include <libsumo/POI.h>
#include <libsumo/Polygon.h>
#include <libsumo/Route.h>
#include <libsumo/Simulation.h>
#include <libsumo/TrafficLight.h>
#include <libsumo/VehicleType.h>
#include <libsumo/Vehicle.h>
#include <libsumo/Person.h>
%}

// Process symbols in header
%include "TraCIDefs.h"
%include "Edge.h"
%include "InductionLoop.h"
%include "Junction.h"
%include "LaneArea.h"
%include "Lane.h"
%include "MultiEntryExit.h"
%include "POI.h"
%include "Polygon.h"
%include "Route.h"
%include "Simulation.h"
%include "TrafficLight.h"
%include "VehicleType.h"
%include "Vehicle.h"
%include "Person.h"

#ifdef SWIGPYTHON
%pythoncode %{
def wrapAsClassMethod(func, module):
    def wrapper(*args, **kwargs):
        return func(module, *args, **kwargs)
    return wrapper

exceptions.TraCIException = TraCIException
trafficlight.Phase = TraCIPhase
trafficlight.Logic = TraCILogic
vehicle.addFull = vehicle.add
vehicle.addLegacy = wrapAsClassMethod(_vehicle.VehicleDomain.addLegacy, vehicle)
vehicle.couldChangeLane = wrapAsClassMethod(_vehicle.VehicleDomain.couldChangeLane, vehicle)
vehicle.wantsAndCouldChangeLane = wrapAsClassMethod(_vehicle.VehicleDomain.wantsAndCouldChangeLane, vehicle)
vehicle.isStopped = wrapAsClassMethod(_vehicle.VehicleDomain.isStopped, vehicle)
vehicle.setBusStop = wrapAsClassMethod(_vehicle.VehicleDomain.setBusStop, vehicle)
vehicle.setParkingAreaStop = wrapAsClassMethod(_vehicle.VehicleDomain.setParkingAreaStop, vehicle)
person.removeStages = wrapAsClassMethod(_person.PersonDomain.removeStages, person)
%}
#endif
