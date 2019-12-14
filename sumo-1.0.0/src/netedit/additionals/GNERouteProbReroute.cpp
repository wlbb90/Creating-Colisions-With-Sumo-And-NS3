/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNERouteProbReroute.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jan 2017
/// @version $Id$
///
//
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/ToString.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/dialogs/GNERerouterIntervalDialog.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>

#include "GNERouteProbReroute.h"
#include "GNERerouter.h"

// ===========================================================================
// member method definitions
// ===========================================================================

GNERouteProbReroute::GNERouteProbReroute(GNERerouterIntervalDialog* rerouterIntervalDialog) :
    GNEAdditional(rerouterIntervalDialog->getEditedAdditional(), rerouterIntervalDialog->getEditedAdditional()->getViewNet(), GLO_REROUTER, SUMO_TAG_ROUTE_PROB_REROUTE, "", false) {
    // if exist a reroute, set newRoute ID
    if (rerouterIntervalDialog->getEditedAdditional()->getViewNet()->getNet()->getAdditionalByType(SUMO_TAG_ROUTE).size() > 0) {
        myNewRouteId = rerouterIntervalDialog->getEditedAdditional()->getViewNet()->getNet()->getAdditionalByType(SUMO_TAG_ROUTE).begin()->first;
    }
    // fill route prob reroute interval with default values
    setDefaultValues();
}


GNERouteProbReroute::GNERouteProbReroute(GNEAdditional* rerouterIntervalParent, const std::string& newRouteId, double probability) :
    GNEAdditional(rerouterIntervalParent, rerouterIntervalParent->getViewNet(), GLO_REROUTER, SUMO_TAG_ROUTE_PROB_REROUTE, "", false),
    myNewRouteId(newRouteId),
    myProbability(probability) {
}


GNERouteProbReroute::~GNERouteProbReroute() {}


void
GNERouteProbReroute::moveGeometry(const Position&, const Position&) {
    // This additional cannot be moved
}


void
GNERouteProbReroute::commitGeometryMoving(const Position&, GNEUndoList*) {
    // This additional cannot be moved
}


void
GNERouteProbReroute::updateGeometry(bool /*updateGrid*/) {
    // Currently this additional doesn't own a Geometry
}


Position
GNERouteProbReroute::getPositionInView() const {
    return myFirstAdditionalParent->getPositionInView();
}


std::string
GNERouteProbReroute::getParentName() const {
    return myFirstAdditionalParent->getID();
}


void
GNERouteProbReroute::drawGL(const GUIVisualizationSettings&) const {
    // Currently This additional isn't drawn
}


std::string
GNERouteProbReroute::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_ROUTE:
            return myNewRouteId;
        case SUMO_ATTR_PROB:
            return toString(myProbability);
        case GNE_ATTR_PARENT:
            return myFirstAdditionalParent->getID();
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERouteProbReroute::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_ROUTE:
        case SUMO_ATTR_PROB:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNERouteProbReroute::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_ROUTE:
            return SUMOXMLDefinitions::isValidVehicleID(value);
        case SUMO_ATTR_PROB:
            return canParse<double>(value);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNERouteProbReroute::getPopUpID() const {
    return toString(getTag());
}


std::string
GNERouteProbReroute::getHierarchyName() const {
    return toString(getTag()) + ": " + myNewRouteId;
}

// ===========================================================================
// private
// ===========================================================================

void
GNERouteProbReroute::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_ROUTE:
            myNewRouteId = value;
            break;
        case SUMO_ATTR_PROB:
            myProbability = parse<double>(value);
            break;
        case GNE_ATTR_GENERIC:
            setGenericParametersStr(value);
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}

/****************************************************************************/
