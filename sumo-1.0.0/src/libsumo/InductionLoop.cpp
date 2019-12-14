/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2012-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    InductionLoop.cpp
/// @author  Daniel Krajzewicz
/// @author  Mario Krumnow
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    30.05.2012
/// @version $Id$
///
// C++ TraCI client API implementation
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <microsim/output/MSDetectorControl.h>
#include <microsim/output/MSInductLoop.h>
#include <microsim/MSNet.h>
#include <libsumo/TraCIDefs.h>
#include <traci-server/TraCIConstants.h>
#include "InductionLoop.h"


namespace libsumo {
// ===========================================================================
// static member initializations
// ===========================================================================
SubscriptionResults InductionLoop::mySubscriptionResults;
ContextSubscriptionResults InductionLoop::myContextSubscriptionResults;


// ===========================================================================
// member definitions
// ===========================================================================
std::vector<std::string>
InductionLoop::getIDList() {
    std::vector<std::string> ids;
    MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_INDUCTION_LOOP).insertIDs(ids);
    return ids;
}


int
InductionLoop::getIDCount() {
    std::vector<std::string> ids;
    return (int)MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_INDUCTION_LOOP).size();
}


double
InductionLoop::getPosition(const std::string& detID) {
    return getDetector(detID)->getPosition();
}


std::string
InductionLoop::getLaneID(const std::string& detID) {
    return getDetector(detID)->getLane()->getID();
}


int
InductionLoop::getLastStepVehicleNumber(const std::string& detID) {
    return getDetector(detID)->getCurrentPassedNumber();
}


double
InductionLoop::getLastStepMeanSpeed(const std::string& detID) {
    return getDetector(detID)->getCurrentSpeed();
}


std::vector<std::string>
InductionLoop::getLastStepVehicleIDs(const std::string& detID) {
    return getDetector(detID)->getCurrentVehicleIDs();
}


double
InductionLoop::getLastStepOccupancy(const std::string& detID) {
    return getDetector(detID)->getCurrentOccupancy();
}


double
InductionLoop::getLastStepMeanLength(const std::string& detID) {
    return getDetector(detID)->getCurrentLength();
}


double
InductionLoop::getTimeSinceDetection(const std::string& detID) {
    return getDetector(detID)->getTimeSinceLastDetection();
}


std::vector<libsumo::TraCIVehicleData>
InductionLoop::getVehicleData(const std::string& detID) {
    std::vector<MSInductLoop::VehicleData> vd = getDetector(detID)->collectVehiclesOnDet(MSNet::getInstance()->getCurrentTimeStep() - DELTA_T, true);
    std::vector<libsumo::TraCIVehicleData> tvd;
    for (std::vector<MSInductLoop::VehicleData>::const_iterator vdi = vd.begin(); vdi != vd.end(); ++vdi) {
        tvd.push_back(libsumo::TraCIVehicleData());
        tvd.back().id = vdi->idM;
        tvd.back().length = vdi->lengthM;
        tvd.back().entryTime = vdi->entryTimeM;
        tvd.back().leaveTime = vdi->leaveTimeM;
        tvd.back().typeID = vdi->typeIDM;
    }
    return tvd;

}


MSInductLoop*
InductionLoop::getDetector(const std::string& id) {
    MSInductLoop* il = dynamic_cast<MSInductLoop*>(MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_INDUCTION_LOOP).get(id));
    if (il == 0) {
        throw TraCIException("Induction loop '" + id + "' is not known");
    }
    return il;
}


LIBSUMO_SUBSCRIPTION_IMPLEMENTATION(InductionLoop, INDUCTIONLOOP)


NamedRTree*
InductionLoop::getTree() {
    NamedRTree* t = new NamedRTree();
    for (const auto& i : MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_INDUCTION_LOOP)) {
        MSInductLoop* il = static_cast<MSInductLoop*>(i.second);
        Position p = il->getLane()->getShape().positionAtOffset(il->getPosition());
        const float cmin[2] = {(float) p.x(), (float) p.y()};
        const float cmax[2] = {(float) p.x(), (float) p.y()};
        t->Insert(cmin, cmax, il);
    }
    return t;
}


void
InductionLoop::storeShape(const std::string& id, PositionVector& shape) {
    MSInductLoop* const il = getDetector(id);
    shape.push_back(il->getLane()->getShape().positionAtOffset(il->getPosition()));
}


std::shared_ptr<VariableWrapper>
InductionLoop::makeWrapper() {
    return std::make_shared<Helper::SubscriptionWrapper>(handleVariable, mySubscriptionResults, myContextSubscriptionResults);
}


bool
InductionLoop::handleVariable(const std::string& objID, const int variable, VariableWrapper* wrapper) {
    switch (variable) {
        case ID_LIST:
            return wrapper->wrapStringList(objID, variable, getIDList());
        case ID_COUNT:
            return wrapper->wrapInt(objID, variable, getIDCount());
        case VAR_POSITION:
            return wrapper->wrapDouble(objID, variable, getPosition(objID));
        case VAR_LANE_ID:
            return wrapper->wrapString(objID, variable, getLaneID(objID));
        case LAST_STEP_VEHICLE_NUMBER:
            return wrapper->wrapInt(objID, variable, getLastStepVehicleNumber(objID));
        case LAST_STEP_MEAN_SPEED:
            return wrapper->wrapDouble(objID, variable, getLastStepMeanSpeed(objID));
        case LAST_STEP_VEHICLE_ID_LIST:
            return wrapper->wrapStringList(objID, variable, getLastStepVehicleIDs(objID));
        case LAST_STEP_OCCUPANCY:
            return wrapper->wrapDouble(objID, variable, getLastStepOccupancy(objID));
        case LAST_STEP_LENGTH:
            return wrapper->wrapDouble(objID, variable, getLastStepMeanLength(objID));
        case LAST_STEP_TIME_SINCE_DETECTION:
            return wrapper->wrapDouble(objID, variable, getTimeSinceDetection(objID));
        default:
            return false;
    }
}


}

/****************************************************************************/
