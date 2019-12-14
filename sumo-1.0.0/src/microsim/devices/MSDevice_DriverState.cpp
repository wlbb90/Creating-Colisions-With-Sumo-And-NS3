/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSDevice_DriverState.cpp
/// @author  Leonhard Luecken
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    15.06.2018
/// @version $Id$
///
/// The Driver State Device mainly provides a configuration and interaction interface for the vehicle's driver state.
/// @see microsim/MSDriverState.h
///
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/TplConvert.h>
#include <utils/options/OptionsCont.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/common/WrappingCommand.h>
#include <utils/common/RGBColor.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSRouteHandler.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSDriverState.h>
#include "MSDevice_DriverState.h"


// ===========================================================================
// debug constants
// ===========================================================================
//#define DEBUG_DSDEVICE
//#define DEBUG_COND (myHolder.isSelected())


// ===========================================================================
// parameter defaults
// ===========================================================================

// see DriverStateDefaults in MSDriverState


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_DriverState::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("Driver State Device");
    insertDefaultAssignmentOptions("driverstate", "Driver State Device", oc);
    oc.doRegister("device.driverstate.initialAwareness", new Option_Float(DriverStateDefaults::minAwareness));
    oc.addDescription("device.driverstate.initialAwareness", "Driver State Device", "Initial value assigned to the driver's awareness.");
    oc.doRegister("device.driverstate.errorTimeScaleCoefficient", new Option_Float(DriverStateDefaults::errorTimeScaleCoefficient));
    oc.addDescription("device.driverstate.errorTimeScaleCoefficient", "Driver State Device", "Time scale for the error process.");
    oc.doRegister("device.driverstate.errorNoiseIntensityCoefficient", new Option_Float(DriverStateDefaults::errorNoiseIntensityCoefficient));
    oc.addDescription("device.driverstate.errorNoiseIntensityCoefficient", "Driver State Device", "Noise intensity driving the error process.");
    oc.doRegister("device.driverstate.speedDifferenceErrorCoefficient", new Option_Float(DriverStateDefaults::speedDifferenceErrorCoefficient));
    oc.addDescription("device.driverstate.speedDifferenceErrorCoefficient", "Driver State Device", "General scaling coefficient for applying the error to the perceived speed difference (error also scales with distance).");
    oc.doRegister("device.driverstate.headwayErrorCoefficient", new Option_Float(DriverStateDefaults::headwayErrorCoefficient));
    oc.addDescription("device.driverstate.headwayErrorCoefficient", "Driver State Device", "General scaling coefficient for applying the error to the perceived distance (error also scales with distance).");
    oc.doRegister("device.driverstate.speedDifferenceChangePerceptionThreshold", new Option_Float(DriverStateDefaults::speedDifferenceChangePerceptionThreshold));
    oc.addDescription("device.driverstate.speedDifferenceChangePerceptionThreshold", "Driver State Device", "Base threshold for recognizing changes in the speed difference (threshold also scales with distance).");
    oc.doRegister("device.driverstate.headwayChangePerceptionThreshold", new Option_Float(DriverStateDefaults::headwayChangePerceptionThreshold));
    oc.addDescription("device.driverstate.headwayChangePerceptionThreshold", "Driver State Device", "Base threshold for recognizing changes in the headway (threshold also scales with distance).");
    oc.doRegister("device.driverstate.minAwareness", new Option_Float(DriverStateDefaults::minAwareness));
    oc.addDescription("device.driverstate.minAwareness", "Driver State Device", "Minimal admissible value for the driver's awareness.");
}


void
MSDevice_DriverState::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "driverstate", v, false)) {
        const double minAwareness = getMinAwareness(v, oc);
        const double initialAwareness = getInitialAwareness(v, oc);
        const double errorTimeScaleCoefficient = getErrorTimeScaleCoefficient(v, oc);
        const double errorNoiseIntensityCoefficient = getErrorNoiseIntensityCoefficient(v, oc);
        const double speedDifferenceErrorCoefficient = getSpeedDifferenceErrorCoefficient(v, oc);
        const double speedDifferenceChangePerceptionThreshold = getSpeedDifferenceChangePerceptionThreshold(v, oc);
        const double headwayChangePerceptionThreshold = getHeadwayChangePerceptionThreshold(v, oc);
        const double headwayErrorCoefficient = getHeadwayErrorCoefficient(v, oc);
        // build the device
        MSDevice_DriverState* device = new MSDevice_DriverState(v, "driverstate" + v.getID(),
                minAwareness,
                initialAwareness,
                errorTimeScaleCoefficient,
                errorNoiseIntensityCoefficient,
                speedDifferenceErrorCoefficient,
                speedDifferenceChangePerceptionThreshold,
                headwayChangePerceptionThreshold,
                headwayErrorCoefficient);
        into.push_back(device);
    }
}


double
MSDevice_DriverState::getMinAwareness(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.minAwareness", DriverStateDefaults::minAwareness, false);
}
double
MSDevice_DriverState::getInitialAwareness(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.initialAwareness", DriverStateDefaults::initialAwareness, false);
}
double
MSDevice_DriverState::getErrorTimeScaleCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.errorTimeScaleCoefficient", DriverStateDefaults::errorTimeScaleCoefficient, false);
}
double
MSDevice_DriverState::getErrorNoiseIntensityCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.errorNoiseIntensityCoefficient", DriverStateDefaults::errorNoiseIntensityCoefficient, false);
}
double
MSDevice_DriverState::getSpeedDifferenceErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.speedDifferenceErrorCoefficient", DriverStateDefaults::speedDifferenceErrorCoefficient, false);
}
double
MSDevice_DriverState::getSpeedDifferenceChangePerceptionThreshold(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.speedDifferenceChangePerceptionThreshold", DriverStateDefaults::speedDifferenceChangePerceptionThreshold, false);
}
double
MSDevice_DriverState::getHeadwayChangePerceptionThreshold(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.headwayChangePerceptionThreshold", DriverStateDefaults::headwayChangePerceptionThreshold, false);
}
double
MSDevice_DriverState::getHeadwayErrorCoefficient(const SUMOVehicle& v, const OptionsCont& oc) {
    return getFloatParam(v, oc, "driverstate.headwayErrorCoefficient", DriverStateDefaults::headwayErrorCoefficient, false);
}


// ---------------------------------------------------------------------------
// MSDevice_DriverState-methods
// ---------------------------------------------------------------------------
MSDevice_DriverState::MSDevice_DriverState(SUMOVehicle& holder, const std::string& id,
        double minAwareness,
        double initialAwareness,
        double errorTimeScaleCoefficient,
        double errorNoiseIntensityCoefficient,
        double speedDifferenceErrorCoefficient,
        double speedDifferenceChangePerceptionThreshold,
        double headwayChangePerceptionThreshold,
        double headwayErrorCoefficient) :
    MSDevice(holder, id),
    myMinAwareness(minAwareness),
    myInitialAwareness(initialAwareness),
    myErrorTimeScaleCoefficient(errorTimeScaleCoefficient),
    myErrorNoiseIntensityCoefficient(errorNoiseIntensityCoefficient),
    mySpeedDifferenceErrorCoefficient(speedDifferenceErrorCoefficient),
    mySpeedDifferenceChangePerceptionThreshold(speedDifferenceChangePerceptionThreshold),
    myHeadwayChangePerceptionThreshold(headwayChangePerceptionThreshold),
    myHeadwayErrorCoefficient(headwayErrorCoefficient),
    myDriverState(nullptr) {
    // Take care! Holder is currently being constructed. Cast occurs before completion.
    myHolderMS = static_cast<MSVehicle*>(&holder);
    // Ensure that the holder receives a driver state as soon as it is created (can't be done here, since myHolderMS is incomplete)
    MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(new WrappingCommand<MSDevice_DriverState>(this, &MSDevice_DriverState::createDriverState), SIMSTEP);

#ifdef DEBUG_DSDEVICE
    std::cout << "initialized device '" << id << "' with "
              << "myMinAwareness=" << myMinAwareness << ", "
              << "myInitialAwareness=" << myInitialAwareness << ", "
              << "myErrorTimeScaleCoefficient=" << myErrorTimeScaleCoefficient << ", "
              << "myErrorNoiseIntensityCoefficient=" << myErrorNoiseIntensityCoefficient << ", "
              << "mySpeedDifferenceErrorCoefficient=" << mySpeedDifferenceErrorCoefficient << ", "
              << "mySpeedDifferenceChangePerceptionThreshold=" << mySpeedDifferenceChangePerceptionThreshold << ", "
              << "myHeadwayChangePerceptionThreshold=" << myHeadwayChangePerceptionThreshold << ", "
              << "myHeadwayErrorCoefficient=" << myHeadwayErrorCoefficient << std::endl;
#endif

}

//MSDevice_DriverState::~MSDevice_DriverState() {}

SUMOTime
MSDevice_DriverState::createDriverState(SUMOTime /* t */) {
#ifdef DEBUG_DSDEVICE
    std::cout << SIMTIME << " MSDevice_DriverState::createDriverState() for vehicle '" << myHolder.getID() << "'" << std::endl;
#endif
    assert(myDriverState == nullptr);
    assert(myHolderMS->getDriverState() == nullptr);
    myHolderMS->createDriverState();
    myDriverState = myHolderMS->getDriverState();
    initDriverState();
    return 0;
}

void
MSDevice_DriverState::initDriverState() {
    myDriverState->setMinAwareness(myMinAwareness);
    myDriverState->setInitialAwareness(myInitialAwareness);
    myDriverState->setErrorTimeScaleCoefficient(myErrorTimeScaleCoefficient);
    myDriverState->setErrorNoiseIntensityCoefficient(myErrorNoiseIntensityCoefficient);
    myDriverState->setSpeedDifferenceErrorCoefficient(mySpeedDifferenceErrorCoefficient);
    myDriverState->setHeadwayErrorCoefficient(myHeadwayErrorCoefficient);
    myDriverState->setSpeedDifferenceChangePerceptionThreshold(mySpeedDifferenceChangePerceptionThreshold);
    myDriverState->setHeadwayChangePerceptionThreshold(myHeadwayChangePerceptionThreshold);
    myDriverState->setAwareness(myInitialAwareness);
}

std::string
MSDevice_DriverState::getParameter(const std::string& key) const {
#ifdef DEBUG_DSDEVICE
    std::cout << "MSDevice_DriverState::getParameter(key=" << key << ")" << std::endl;
#endif
    if (key == "awareness") {
        return toString(myDriverState->getAwareness());
    } else if (key == "errorState") {
        return toString(myDriverState->getErrorState());
    } else if (key == "errorTimeScale") {
        return toString(myDriverState->getErrorTimeScale());
    } else if (key == "errorNoiseIntensity") {
        return toString(myDriverState->getErrorNoiseIntensity());
    } else if (key == "errorNoiseIntensity") {
        return toString(myDriverState->getErrorNoiseIntensity());
    } else if (key == "minAwareness") {
        return toString(myDriverState->getMinAwareness());
    } else if (key == "initialAwareness") {
        return toString(myDriverState->getInitialAwareness());
    } else if (key == "errorTimeScaleCoefficient") {
        return toString(myDriverState->getErrorTimeScaleCoefficient());
    } else if (key == "errorNoiseIntensityCoefficient") {
        return toString(myDriverState->getErrorNoiseIntensityCoefficient());
    } else if (key == "speedDifferenceErrorCoefficient") {
        return toString(myDriverState->getSpeedDifferenceErrorCoefficient());
    } else if (key == "headwayErrorCoefficient") {
        return toString(myDriverState->getHeadwayErrorCoefficient());
    } else if (key == "speedDifferenceChangePerceptionThreshold") {
        return toString(myDriverState->getSpeedDifferenceChangePerceptionThreshold());
    } else if (key == "headwayChangePerceptionThreshold") {
        return toString(myDriverState->getHeadwayChangePerceptionThreshold());
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_DriverState::setParameter(const std::string& key, const std::string& value) {
#ifdef DEBUG_DSDEVICE
    std::cout << "MSDevice_DriverState::setParameter(key=" << key << ", value=" << value << ")" << std::endl;
#endif
    if (key == "awareness") {
        myDriverState->setAwareness(TplConvert::_2double(value.c_str()));
    } else if (key == "errorState") {
        myDriverState->setErrorState(TplConvert::_2double(value.c_str()));
    } else if (key == "errorTimeScale") {
        myDriverState->setErrorTimeScale(TplConvert::_2double(value.c_str()));
    } else if (key == "errorNoiseIntensity") {
        myDriverState->setErrorNoiseIntensity(TplConvert::_2double(value.c_str()));
    } else if (key == "minAwareness") {
        myDriverState->setMinAwareness(TplConvert::_2double(value.c_str()));
    } else if (key == "initialAwareness") {
        myDriverState->setInitialAwareness(TplConvert::_2double(value.c_str()));
    } else if (key == "errorTimeScaleCoefficient") {
        myDriverState->setErrorTimeScaleCoefficient(TplConvert::_2double(value.c_str()));
    } else if (key == "errorNoiseIntensityCoefficient") {
        myDriverState->setErrorNoiseIntensityCoefficient(TplConvert::_2double(value.c_str()));
    } else if (key == "speedDifferenceErrorCoefficient") {
        myDriverState->setSpeedDifferenceErrorCoefficient(TplConvert::_2double(value.c_str()));
    } else if (key == "headwayErrorCoefficient") {
        myDriverState->setHeadwayErrorCoefficient(TplConvert::_2double(value.c_str()));
    } else if (key == "speedDifferenceChangePerceptionThreshold") {
        myDriverState->setSpeedDifferenceChangePerceptionThreshold(TplConvert::_2double(value.c_str()));
    } else if (key == "headwayChangePerceptionThreshold") {
        myDriverState->setHeadwayChangePerceptionThreshold(TplConvert::_2double(value.c_str()));
    } else {
        throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}


/****************************************************************************/

