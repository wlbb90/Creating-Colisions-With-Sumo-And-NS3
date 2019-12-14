/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    MSDevice_Bluelight.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @author  Laura Bieker
/// @date    01.06.2017
/// @version $Id$
///
// A device for emergency vehicle. The behaviour of other traffic participants will be triggered with this device.
// For example building a rescue lane.
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/TplConvert.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include "MSDevice_Tripinfo.h"
#include "MSDevice_Bluelight.h"
#include <microsim/MSVehicleControl.h>
#include <microsim/MSVehicleType.h>


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Bluelight::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("Bluelight Device");
    insertDefaultAssignmentOptions("bluelight", "Bluelight Device", oc);

    oc.doRegister("device.bluelight.parameter", new Option_Float(0.0));
    oc.addDescription("device.bluelight.parameter", "Bluelight Device", "An exemplary parameter which can be used by all instances of the example device");

}


void
MSDevice_Bluelight::buildVehicleDevices(SUMOVehicle& v, std::vector<MSDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "bluelight", v, false)) {
        // build the device
        // get custom vehicle parameter
        double customParameter2 = -1;
        if (v.getParameter().knowsParameter("bluelight")) {
            try {
                customParameter2 = TplConvert::_2double(v.getParameter().getParameter("bluelight", "-1").c_str());
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("bluelight", "-1") + "'for vehicle parameter 'example'");
            }

        } else {
            std::cout << "vehicle '" << v.getID() << "' does not supply vehicle parameter 'bluelight'. Using default of " << customParameter2 << "\n";
        }
        // get custom vType parameter
        double customParameter3 = -1;
        if (v.getVehicleType().getParameter().knowsParameter("bluelight")) {
            try {
                customParameter3 = TplConvert::_2double(v.getVehicleType().getParameter().getParameter("bluelight", "-1").c_str());
            } catch (...) {
                WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("bluelight", "-1") + "'for vType parameter 'bluelight'");
            }

        } else {
            std::cout << "vehicle '" << v.getID() << "' does not supply vType parameter 'bluelight'. Using default of " << customParameter3 << "\n";
        }
        MSDevice_Bluelight* device = new MSDevice_Bluelight(v, "bluelight_" + v.getID(),
                oc.getFloat("device.bluelight.parameter"),
                customParameter2,
                customParameter3);
        into.push_back(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_Bluelight-methods
// ---------------------------------------------------------------------------
MSDevice_Bluelight::MSDevice_Bluelight(SUMOVehicle& holder, const std::string& id,
                                       double customValue1, double customValue2, double customValue3) :
    MSDevice(holder, id),
    myCustomValue1(customValue1),
    myCustomValue2(customValue2),
    myCustomValue3(customValue3) {
    std::cout << "initialized device '" << id << "' with myCustomValue1=" << myCustomValue1 << ", myCustomValue2=" << myCustomValue2 << ", myCustomValue3=" << myCustomValue3 << "\n";
}


MSDevice_Bluelight::~MSDevice_Bluelight() {
}


bool
MSDevice_Bluelight::notifyMove(SUMOVehicle& veh, double /* oldPos */,
                               double /* newPos */, double  newSpeed) {
    std::cout << "device '" << getID() << "' notifyMove: newSpeed=" << newSpeed << "\n";
    // check whether another device is present on the vehicle:
    /*MSDevice_Tripinfo* otherDevice = static_cast<MSDevice_Tripinfo*>(veh.getDevice(typeid(MSDevice_Tripinfo)));
    if (otherDevice != 0) {
        std::cout << "  veh '" << veh.getID() << " has device '" << otherDevice->getID() << "'\n";
    }*/
    //violate red lights  this only need to be done once so shift it todo
    MSVehicle::Influencer& redLight = static_cast<MSVehicle&>(veh).getInfluencer();
    redLight.setSpeedMode(7);
    // build a rescue lane for all vehicles on the route of the emergency vehicle within the range of the siren
    MSVehicleType* vt = MSNet::getInstance()->getVehicleControl().getVType(veh.getVehicleType().getID());
    vt->setPreferredLateralAlignment(LATALIGN_ARBITRARY);
    MSVehicleControl& vc = MSNet::getInstance()->getVehicleControl();
    std::string currentEdgeID = veh.getEdge()->getID();
    for (MSVehicleControl::constVehIt it = vc.loadedVehBegin(); it != vc.loadedVehEnd(); ++it) {
        SUMOVehicle* veh2 = it->second;
        //Vehicle only from edge should react
        if (currentEdgeID == veh2->getEdge()->getID()) {
            double distanceDelta = veh.getPosition().distanceTo(veh2->getPosition());
            // the perception of the sound of the siren should be around 25 meters
            // todo only vehicles in front of the emergency vehicle should react
            if (distanceDelta <= 25 && veh.getID() != veh2->getID() && influencedVehicles.count(veh2->getID()) == 0) {
                influencedVehicles.insert(static_cast<std::string>(veh2->getID()));
                influencedTypes.insert(std::make_pair(static_cast<std::string>(veh2->getID()), veh2->getVehicleType().getID()));
                //Vehicle gets a new Vehicletype to change the alignment and the lanechange options
                MSVehicleType& t = static_cast<MSVehicle*>(veh2)->getSingularType();
                MSVehicle::Influencer& lanechange = static_cast<MSVehicle*>(veh2)->getInfluencer();

                //other vehicle should not use the rescue lane so they should not make any lane changes
                lanechange.setLaneChangeMode(1605);
                const int numLanes = (int)veh2->getEdge()->getLanes().size();
                //Setting the lateral alignment to build a rescue lane
                if (veh2->getLane()->getIndex() == numLanes - 1) {
                    t.setPreferredLateralAlignment(LATALIGN_LEFT);
                    // the alignement is changet to left for the vehicle std::cout << "New alignment to left for vehicle: " << veh2->getID() << " " << veh2->getVehicleType().getPreferredLateralAlignment() << "\n";
                } else {
                    t.setPreferredLateralAlignment(LATALIGN_RIGHT);
                    // the alignement is changet to right for the vehicle std::cout << "New alignment to right for vehicle: " << veh2->getID() << " " << veh2->getVehicleType().getPreferredLateralAlignment() << "\n";
                }

            }

        } else { //if vehicle is passed all vehicles which had to react should get their state back after they leave the communication range
            if (influencedVehicles.count(veh2->getID()) > 0) {
                double distanceDelta = veh.getPosition().distanceTo(veh2->getPosition());
                if (distanceDelta > 25 && veh.getID() != veh2->getID()) {
                    influencedVehicles.erase(veh2->getID());
                    std::map<std::string, std::string>::iterator it = influencedTypes.find(veh2->getID());
                    if (it != influencedTypes.end()) {
                        // The vehicle gets back its old VehicleType after the emergency vehicle have passed them
                        MSVehicleType* targetType = MSNet::getInstance()->getVehicleControl().getVType(it->second);
                        static_cast<MSVehicle*>(veh2)->replaceVehicleType(targetType);
                    }
                }
            }
        }
    }
    return true; // keep the device
}


bool
MSDevice_Bluelight::notifyEnter(SUMOVehicle& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    std::cout << "device '" << getID() << "' notifyEnter: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


bool
MSDevice_Bluelight::notifyLeave(SUMOVehicle& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    std::cout << "device '" << getID() << "' notifyLeave: reason=" << reason << " currentEdge=" << veh.getEdge()->getID() << "\n";
    return true; // keep the device
}


void
MSDevice_Bluelight::generateOutput() const {
    if (OptionsCont::getOptions().isSet("tripinfo-output")) {
        OutputDevice& os = OutputDevice::getDeviceByOption("tripinfo-output");
        os.openTag("example_device");
        os.writeAttr("customValue1", toString(myCustomValue1));
        os.writeAttr("customValue2", toString(myCustomValue2));
        os.closeTag();
    }
}

std::string
MSDevice_Bluelight::getParameter(const std::string& key) const {
    if (key == "customValue1") {
        return toString(myCustomValue1);
    } else if (key == "customValue2") {
        return toString(myCustomValue2);
    } else if (key == "meaningOfLife") {
        return "42";
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_Bluelight::setParameter(const std::string& key, const std::string& value) {
    double doubleValue;
    try {
        doubleValue = TplConvert::_2double(value.c_str());
    } catch (NumberFormatException&) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }
    if (key == "customValue1") {
        myCustomValue1 = doubleValue;
    } else {
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}


/****************************************************************************/

