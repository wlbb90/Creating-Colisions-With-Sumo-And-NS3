/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNECalibratorFlow.cpp
/// @author  Pablo Alvarez Lopez
/// @date    March 2016
/// @version $Id$
///
//
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <iostream>
#include <utility>
#include <utils/geom/PositionVector.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/common/ToString.h>
#include <utils/geom/GeomHelper.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>
#include <netedit/dialogs/GNECalibratorDialog.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/GNEUndoList.h>

#include "GNECalibratorFlow.h"
#include "GNECalibratorVehicleType.h"
#include "GNECalibratorRoute.h"
#include "GNECalibrator.h"


// ===========================================================================
// member method definitions
// ===========================================================================


GNECalibratorFlow::GNECalibratorFlow(GNEAdditional* calibratorParent) :
    GNEAdditional(calibratorParent, calibratorParent->getViewNet(), GLO_CALIBRATOR, SUMO_TAG_FLOW, "", false),
    myVehicleType(calibratorParent->getViewNet()->getNet()->retrieveAdditional(SUMO_TAG_VTYPE, DEFAULT_VTYPE_ID)),
    myRoute(calibratorParent->getViewNet()->getNet()->getAdditionalByType(SUMO_TAG_ROUTE).begin()->second) {
    // fill calibrator flows with default values
    setDefaultValues();
}


GNECalibratorFlow::GNECalibratorFlow(GNEAdditional* calibratorParent, GNEAdditional* vehicleType, GNEAdditional* route, const std::string& vehsPerHour, const std::string& speed,
                                     const RGBColor& color, const std::string& departLane, const std::string& departPos, const std::string& departSpeed, const std::string& arrivalLane,
                                     const std::string& arrivalPos, const std::string& arrivalSpeed, const std::string& line, int personNumber, int containerNumber, bool reroute,
                                     const std::string& departPosLat, const std::string& arrivalPosLat, double begin, double end) :
    GNEAdditional(calibratorParent, calibratorParent->getViewNet(), GLO_CALIBRATOR, SUMO_TAG_FLOW, "", false),
    myVehicleType(vehicleType),
    myRoute(route),
    myVehsPerHour(vehsPerHour),
    mySpeed(speed),
    myColor(color),
    myDepartLane(departLane),
    myDepartPos(departPos),
    myDepartSpeed(departSpeed),
    myArrivalLane(arrivalLane),
    myArrivalPos(arrivalPos),
    myArrivalSpeed(arrivalSpeed),
    myLine(line),
    myPersonNumber(personNumber),
    myContainerNumber(containerNumber),
    myReroute(reroute),
    myDepartPosLat(departPosLat),
    myArrivalPosLat(arrivalPosLat),
    myBegin(begin),
    myEnd(end) {
}


GNECalibratorFlow::~GNECalibratorFlow() {}


void
GNECalibratorFlow::moveGeometry(const Position&, const Position&) {
    // This additional cannot be moved
}


void
GNECalibratorFlow::commitGeometryMoving(const Position&, GNEUndoList*) {
    // This additional cannot be moved
}


void
GNECalibratorFlow::updateGeometry(bool /*updateGrid*/) {
    // Currently this additional doesn't own a Geometry
}


Position
GNECalibratorFlow::getPositionInView() const {
    return myFirstAdditionalParent->getPositionInView();
}


std::string
GNECalibratorFlow::getParentName() const {
    return myFirstAdditionalParent->getID();
}


void
GNECalibratorFlow::drawGL(const GUIVisualizationSettings& /* s */) const {
    // Currently This additional isn't drawn
}


std::string
GNECalibratorFlow::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_TYPE:
            return myVehicleType->getID();
        case SUMO_ATTR_ROUTE:
            return myRoute->getID();
        case SUMO_ATTR_VEHSPERHOUR:
            return toString(myVehsPerHour);
        case SUMO_ATTR_SPEED:
            return toString(mySpeed);
        case SUMO_ATTR_COLOR:
            return toString(myColor);
        case SUMO_ATTR_BEGIN:
            return toString(myBegin);
        case SUMO_ATTR_END:
            return toString(myEnd);
        case SUMO_ATTR_DEPARTLANE:
            return myDepartLane;
        case SUMO_ATTR_DEPARTPOS:
            return myDepartPos;
        case SUMO_ATTR_DEPARTSPEED:
            return myDepartSpeed;
        case SUMO_ATTR_ARRIVALLANE:
            return myArrivalLane;
        case SUMO_ATTR_ARRIVALPOS:
            return myArrivalPos;
        case SUMO_ATTR_ARRIVALSPEED:
            return myArrivalSpeed;
        case SUMO_ATTR_LINE:
            return myLine;
        case SUMO_ATTR_PERSON_NUMBER:
            return toString(myPersonNumber);
        case SUMO_ATTR_CONTAINER_NUMBER:
            return toString(myContainerNumber);
        case SUMO_ATTR_REROUTE:
            return toString(myReroute);
        case SUMO_ATTR_DEPARTPOS_LAT:
            return myDepartPosLat;
        case SUMO_ATTR_ARRIVALPOS_LAT:
            return myArrivalPosLat;
        case GNE_ATTR_PARENT:
            return myFirstAdditionalParent->getID();
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNECalibratorFlow::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_TYPE:
        case SUMO_ATTR_ROUTE:
        case SUMO_ATTR_COLOR:
        case SUMO_ATTR_VEHSPERHOUR:
        case SUMO_ATTR_SPEED:
        case SUMO_ATTR_BEGIN:
        case SUMO_ATTR_END:
        case SUMO_ATTR_DEPARTLANE:
        case SUMO_ATTR_DEPARTPOS:
        case SUMO_ATTR_DEPARTSPEED:
        case SUMO_ATTR_ARRIVALLANE:
        case SUMO_ATTR_ARRIVALPOS:
        case SUMO_ATTR_ARRIVALSPEED:
        case SUMO_ATTR_LINE:
        case SUMO_ATTR_PERSON_NUMBER:
        case SUMO_ATTR_CONTAINER_NUMBER:
        case SUMO_ATTR_REROUTE:
        case SUMO_ATTR_DEPARTPOS_LAT:
        case SUMO_ATTR_ARRIVALPOS_LAT:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNECalibratorFlow::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_TYPE:
            return SUMOXMLDefinitions::isValidTypeID(value) && (myViewNet->getNet()->retrieveAdditional(SUMO_TAG_VTYPE, value, false) != nullptr);
        case SUMO_ATTR_ROUTE:
            return SUMOXMLDefinitions::isValidVehicleID(value) && (myViewNet->getNet()->retrieveAdditional(SUMO_TAG_ROUTE, value, false) != nullptr);
        case SUMO_ATTR_VEHSPERHOUR:
            if (value.empty()) {
                // speed and vehsPerHour cannot be empty at the same time
                if (mySpeed.empty()) {
                    return false;
                } else {
                    return true;
                }
            } else if (canParse<double>(value)) {
                return (parse<double>(value) >= 0);
            } else {
                return false;
            }
        case SUMO_ATTR_SPEED:
            if (value.empty()) {
                // speed and vehsPerHour cannot be empty at the same time
                if (myVehsPerHour.empty()) {
                    return false;
                } else {
                    return true;
                }
            } else if (canParse<double>(value)) {
                return (parse<double>(value) >= 0);
            } else {
                return false;
            }
        case SUMO_ATTR_COLOR:
            return canParse<RGBColor>(value);
        case SUMO_ATTR_BEGIN:
            return canParse<double>(value) && (parse<double>(value) >= 0);
        case SUMO_ATTR_END:
            return canParse<double>(value) && (parse<double>(value) >= 0);
        case SUMO_ATTR_DEPARTLANE:
            if ((value == "random") || (value == "free") || (value == "allowed") || (value == "best") || (value == "first")) {
                return true;
            } else {
                return (myViewNet->getNet()->retrieveLane(value, false) != nullptr);
            }
        case SUMO_ATTR_DEPARTPOS:
            if ((value == "random") || (value == "free") || (value == "random_free") || (value == "base") || (value == "last")) {
                return true;
            } else {
                return canParse<double>(value);
            }
        case SUMO_ATTR_DEPARTSPEED:
            if ((value == "random") || (value == "max")) {
                return true;
            } else {
                return canParse<double>(value);
            }
        case SUMO_ATTR_ARRIVALLANE:
            if (value == "current") {
                return true;
            } else {
                return (myViewNet->getNet()->retrieveLane(value, false) != nullptr);
            }
        case SUMO_ATTR_ARRIVALPOS:
            if ((value == "random") || (value == "max")) {
                return true;
            } else {
                return canParse<double>(value);
            }
        case SUMO_ATTR_ARRIVALSPEED:
            if (value == "current") {
                return true;
            } else {
                return canParse<double>(value);
            }
        case SUMO_ATTR_LINE:
            return true;
        case SUMO_ATTR_PERSON_NUMBER:
            return canParse<int>(value) && parse<int>(value) >= 0;
        case SUMO_ATTR_CONTAINER_NUMBER:
            return canParse<int>(value) && parse<int>(value) >= 0;
        case SUMO_ATTR_REROUTE:
            return canParse<bool>(value);
        case SUMO_ATTR_DEPARTPOS_LAT:
            return SUMOXMLDefinitions::LateralAlignments.hasString(value);
        case SUMO_ATTR_ARRIVALPOS_LAT:
            return SUMOXMLDefinitions::LateralAlignments.hasString(value);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNECalibratorFlow::getPopUpID() const {
    return toString(getTag());
}


std::string
GNECalibratorFlow::getHierarchyName() const {
    return toString(getTag()) + ": " + getAttribute(SUMO_ATTR_BEGIN) + " -> " + getAttribute(SUMO_ATTR_END);
}

// ===========================================================================
// private
// ===========================================================================

void
GNECalibratorFlow::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_TYPE:
            myVehicleType = myViewNet->getNet()->retrieveAdditional(SUMO_TAG_VTYPE, value);
            break;
        case SUMO_ATTR_ROUTE:
            myRoute = myViewNet->getNet()->retrieveAdditional(SUMO_TAG_ROUTE, value);
            break;
        case SUMO_ATTR_VEHSPERHOUR:
            myVehsPerHour = value;
            break;
        case SUMO_ATTR_SPEED:
            mySpeed = value;
            break;
        case SUMO_ATTR_COLOR:
            myColor = parse<RGBColor>(value);
            break;
        case SUMO_ATTR_BEGIN:
            myBegin = parse<double>(value);
            break;
        case SUMO_ATTR_END:
            myEnd = parse<double>(value);
            break;
        case SUMO_ATTR_DEPARTLANE:
            myDepartLane = value;
            break;
        case SUMO_ATTR_DEPARTPOS:
            myDepartPos = value;
            break;
        case SUMO_ATTR_DEPARTSPEED:
            myDepartSpeed = value;
            break;
        case SUMO_ATTR_ARRIVALLANE:
            myArrivalLane = value;
            break;
        case SUMO_ATTR_ARRIVALPOS:
            myArrivalPos = value;
            break;
        case SUMO_ATTR_ARRIVALSPEED:
            myArrivalSpeed = value;
            break;
        case SUMO_ATTR_LINE:
            myLine = value;
            break;
        case SUMO_ATTR_PERSON_NUMBER:
            myPersonNumber = parse<int>(value);
            break;
        case SUMO_ATTR_CONTAINER_NUMBER:
            myContainerNumber = parse<int>(value);
            break;
        case SUMO_ATTR_REROUTE:
            myReroute = parse<bool>(value);
            break;
        case SUMO_ATTR_DEPARTPOS_LAT:
            myDepartPosLat = value;
            break;
        case SUMO_ATTR_ARRIVALPOS_LAT:
            myArrivalPosLat = value;
            break;
        case GNE_ATTR_GENERIC:
            setGenericParametersStr(value);
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}

/****************************************************************************/
