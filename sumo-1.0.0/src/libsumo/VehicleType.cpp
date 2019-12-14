/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2017-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    VehicleType.cpp
/// @author  Gregor Laemmel
/// @date    04.04.2017
/// @version $Id$
///
// C++ TraCI client API implementation
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <microsim/MSNet.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSVehicleType.h>
#include <traci-server/TraCIConstants.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/xml/SUMOVehicleParserHelper.h>
#include "Helper.h"
#include "VehicleType.h"


namespace libsumo {
// ===========================================================================
// static member initializations
// ===========================================================================
SubscriptionResults VehicleType::mySubscriptionResults;
ContextSubscriptionResults VehicleType::myContextSubscriptionResults;


// ===========================================================================
// static member definitions
// ===========================================================================
std::vector<std::string>
VehicleType::getIDList() {
    std::vector<std::string> ids;
    MSNet::getInstance()->getVehicleControl().insertVTypeIDs(ids);
    return ids;
}


int
VehicleType::getIDCount() {
    return (int)getIDList().size();
}


double
VehicleType::getLength(const std::string& typeID) {
    return getVType(typeID)->getLength();
}


double
VehicleType::getMaxSpeed(const std::string& typeID) {
    return getVType(typeID)->getMaxSpeed();
}


double
VehicleType::getActionStepLength(const std::string& typeID) {
    return getVType(typeID)->getActionStepLengthSecs();
}


double
VehicleType::getSpeedFactor(const std::string& typeID) {
    return getVType(typeID)->getSpeedFactor().getParameter()[0];
}


double
VehicleType::getSpeedDeviation(const std::string& typeID) {
    return getVType(typeID)->getSpeedFactor().getParameter()[1];
}


double
VehicleType::getAccel(const std::string& typeID) {
    return getVType(typeID)->getCarFollowModel().getMaxAccel();
}


double
VehicleType::getDecel(const std::string& typeID) {
    return getVType(typeID)->getCarFollowModel().getMaxDecel();
}


double
VehicleType::getEmergencyDecel(const std::string& typeID) {
    return getVType(typeID)->getCarFollowModel().getEmergencyDecel();
}


double
VehicleType::getApparentDecel(const std::string& typeID) {
    return getVType(typeID)->getCarFollowModel().getApparentDecel();
}


double
VehicleType::getImperfection(const std::string& typeID) {
    return getVType(typeID)->getCarFollowModel().getImperfection();
}


double
VehicleType::getTau(const std::string& typeID) {
    return getVType(typeID)->getCarFollowModel().getHeadwayTime();
}


std::string
VehicleType::getVehicleClass(const std::string& typeID) {
    return toString(getVType(typeID)->getVehicleClass());
}


std::string
VehicleType::getEmissionClass(const std::string& typeID) {
    return PollutantsInterface::getName(getVType(typeID)->getEmissionClass());
}


std::string
VehicleType::getShapeClass(const std::string& typeID) {
    return getVehicleShapeName(getVType(typeID)->getGuiShape());
}


double
VehicleType::getMinGap(const std::string& typeID) {
    return getVType(typeID)->getMinGap();
}


double
VehicleType::getWidth(const std::string& typeID) {
    return getVType(typeID)->getWidth();
}


double
VehicleType::getHeight(const std::string& typeID) {
    return getVType(typeID)->getHeight();
}


TraCIColor
VehicleType::getColor(const std::string& typeID) {
    return Helper::makeTraCIColor(getVType(typeID)->getColor());
}


double
VehicleType::getMinGapLat(const std::string& typeID) {
    return getVType(typeID)->getMinGapLat();
}


double
VehicleType::getMaxSpeedLat(const std::string& typeID) {
    return getVType(typeID)->getMaxSpeedLat();
}


std::string
VehicleType::getLateralAlignment(const std::string& typeID) {
    return toString(getVType(typeID)->getPreferredLateralAlignment());
}


std::string
VehicleType::getParameter(const std::string& typeID, const std::string& key) {
    return getVType(typeID)->getParameter().getParameter(key, "");
}


void
VehicleType::setLength(const std::string& typeID, double length)  {
    getVType(typeID)->setLength(length);
}


void
VehicleType::setMaxSpeed(const std::string& typeID, double speed)  {
    getVType(typeID)->setMaxSpeed(speed);
}


void
VehicleType::setActionStepLength(const std::string& typeID, double actionStepLength, bool resetActionOffset)  {
    getVType(typeID)->setActionStepLength(SUMOVehicleParserHelper::processActionStepLength(actionStepLength), resetActionOffset);
}


void
VehicleType::setVehicleClass(const std::string& typeID, const std::string& clazz)  {
    getVType(typeID)->setVClass(getVehicleClassID(clazz));
}


void
VehicleType::setSpeedFactor(const std::string& typeID, double factor)  {
    getVType(typeID)->setSpeedFactor(factor);
}


void
VehicleType::setSpeedDeviation(const std::string& typeID, double deviation)  {
    getVType(typeID)->setSpeedDeviation(deviation);
}


void
VehicleType::setEmissionClass(const std::string& typeID, const std::string& clazz)  {
    getVType(typeID)->setEmissionClass(PollutantsInterface::getClassByName(clazz));
}


void
VehicleType::setShapeClass(const std::string& typeID, const std::string& shapeClass)  {
    getVType(typeID)->setShape(getVehicleShapeID(shapeClass));
}


void
VehicleType::setWidth(const std::string& typeID, double width)  {
    getVType(typeID)->setWidth(width);
}


void
VehicleType::setHeight(const std::string& typeID, double height)  {
    getVType(typeID)->setHeight(height);
}


void
VehicleType::setMinGap(const std::string& typeID, double minGap)  {
    getVType(typeID)->setMinGap(minGap);
}


void
VehicleType::setAccel(const std::string& typeID, double accel)  {
    getVType(typeID)->setAccel(accel);
}


void
VehicleType::setDecel(const std::string& typeID, double decel)  {
    MSVehicleType* v = getVType(typeID);
    v->setDecel(decel);
    // automatically raise emergencyDecel to ensure it is at least as high as decel
    if (decel > v->getCarFollowModel().getEmergencyDecel()) {
        if (v->getParameter().cfParameter.count(SUMO_ATTR_EMERGENCYDECEL) > 0) {
            // notify user only if emergencyDecel was previously specified
            WRITE_WARNING("Automatically setting emergencyDecel to " + toString(decel) + " for vType '" + typeID + "' to match decel.");
        }
        v->setEmergencyDecel(decel);
    }
}


void
VehicleType::setEmergencyDecel(const std::string& typeID, double decel)  {
    MSVehicleType* v = getVType(typeID);
    v->setEmergencyDecel(decel);
    if (decel < v->getCarFollowModel().getMaxDecel()) {
        WRITE_WARNING("New value of emergencyDecel (" + toString(decel) + ") is lower than decel (" + toString(v->getCarFollowModel().getMaxDecel()) + ")");
    }
}


void
VehicleType::setApparentDecel(const std::string& typeID, double decel)  {
    getVType(typeID)->setApparentDecel(decel);
}


void
VehicleType::setImperfection(const std::string& typeID, double imperfection)  {
    getVType(typeID)->setImperfection(imperfection);
}


void
VehicleType::setTau(const std::string& typeID, double tau)  {
    getVType(typeID)->setTau(tau);
}


void
VehicleType::setColor(const std::string& typeID, const TraCIColor& c)  {
    getVType(typeID)->setColor(Helper::makeRGBColor(c));
}


void
VehicleType::setMinGapLat(const std::string& typeID, double minGapLat)  {
    getVType(typeID)->setMinGapLat(minGapLat);
}


void
VehicleType::setMaxSpeedLat(const std::string& typeID, double speed)  {
    getVType(typeID)->setMaxSpeedLat(speed);
}


void
VehicleType::setLateralAlignment(const std::string& typeID, const std::string& latAlignment)  {
    getVType(typeID)->setPreferredLateralAlignment(SUMOXMLDefinitions::LateralAlignments.get(latAlignment));
}


void
VehicleType::copy(const std::string& origTypeID, const std::string& newTypeID)  {
    getVType(origTypeID)->duplicateType(newTypeID, true);
}


void
VehicleType::setParameter(const std::string& typeID, const std::string& name, const std::string& value) {
    ((SUMOVTypeParameter&)getVType(typeID)->getParameter()).setParameter(name, value);
}


LIBSUMO_SUBSCRIPTION_IMPLEMENTATION(VehicleType, VEHICLETYPE)


MSVehicleType*
VehicleType::getVType(std::string id) {
    MSVehicleType* t = MSNet::getInstance()->getVehicleControl().getVType(id);
    if (t == 0) {
        throw TraCIException("Vehicle type '" + id + "' is not known");
    }
    return t;
}


std::shared_ptr<VariableWrapper>
VehicleType::makeWrapper() {
    return std::make_shared<Helper::SubscriptionWrapper>(handleVariable, mySubscriptionResults, myContextSubscriptionResults);
}


bool
VehicleType::handleVariable(const std::string& objID, const int variable, VariableWrapper* wrapper) {
    switch (variable) {
        case ID_LIST:
            return wrapper->wrapStringList(objID, variable, getIDList());
        case ID_COUNT:
            return wrapper->wrapInt(objID, variable, getIDCount());
        case VAR_LENGTH:
            return wrapper->wrapDouble(objID, variable, getLength(objID));
        case VAR_HEIGHT:
            return wrapper->wrapDouble(objID, variable, getHeight(objID));
        case VAR_MINGAP:
            return wrapper->wrapDouble(objID, variable, getMinGap(objID));
        case VAR_MAXSPEED:
            return wrapper->wrapDouble(objID, variable, getMaxSpeed(objID));
        case VAR_ACCEL:
            return wrapper->wrapDouble(objID, variable, getAccel(objID));
        case VAR_DECEL:
            return wrapper->wrapDouble(objID, variable, getDecel(objID));
        case VAR_EMERGENCY_DECEL:
            return wrapper->wrapDouble(objID, variable, getEmergencyDecel(objID));
        case VAR_APPARENT_DECEL:
            return wrapper->wrapDouble(objID, variable, getApparentDecel(objID));
        case VAR_ACTIONSTEPLENGTH:
            return wrapper->wrapDouble(objID, variable, getActionStepLength(objID));
        case VAR_IMPERFECTION:
            return wrapper->wrapDouble(objID, variable, getImperfection(objID));
        case VAR_TAU:
            return wrapper->wrapDouble(objID, variable, getTau(objID));
        case VAR_SPEED_FACTOR:
            return wrapper->wrapDouble(objID, variable, getSpeedFactor(objID));
        case VAR_SPEED_DEVIATION:
            return wrapper->wrapDouble(objID, variable, getSpeedDeviation(objID));
        case VAR_VEHICLECLASS:
            return wrapper->wrapString(objID, variable, getVehicleClass(objID));
        case VAR_EMISSIONCLASS:
            return wrapper->wrapString(objID, variable, getEmissionClass(objID));
        case VAR_SHAPECLASS:
            return wrapper->wrapString(objID, variable, getShapeClass(objID));
        case VAR_WIDTH:
            return wrapper->wrapDouble(objID, variable, getWidth(objID));
        case VAR_COLOR:
            return wrapper->wrapColor(objID, variable, getColor(objID));
        case VAR_MINGAP_LAT:
            return wrapper->wrapDouble(objID, variable, getMinGapLat(objID));
        case VAR_MAXSPEED_LAT:
            return wrapper->wrapDouble(objID, variable, getMaxSpeedLat(objID));
        case VAR_LATALIGNMENT:
            return wrapper->wrapString(objID, variable, getLateralAlignment(objID));
        default:
            return false;
    }
}


}


/****************************************************************************/
