/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEDestProbReroute.cpp
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
#include <netedit/dialogs/GNERerouterIntervalDialog.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>

#include "GNEDestProbReroute.h"
#include "GNERerouter.h"
#include "GNERerouterInterval.h"

// ===========================================================================
// member method definitions
// ===========================================================================

GNEDestProbReroute::GNEDestProbReroute(GNERerouterIntervalDialog* rerouterIntervalDialog) :
    GNEAdditional(rerouterIntervalDialog->getEditedAdditional(), rerouterIntervalDialog->getEditedAdditional()->getViewNet(), GLO_REROUTER, SUMO_TAG_DEST_PROB_REROUTE, "", false),
    myNewEdgeDestination(rerouterIntervalDialog->getEditedAdditional()->getFirstAdditionalParent()->getEdgeChilds().at(0)) {
    // fill dest prob reroute interval with default values
    setDefaultValues();
}


GNEDestProbReroute::GNEDestProbReroute(GNEAdditional* rerouterIntervalParent, GNEEdge* newEdgeDestination, double probability):
    GNEAdditional(rerouterIntervalParent, rerouterIntervalParent->getViewNet(), GLO_REROUTER, SUMO_TAG_DEST_PROB_REROUTE, "", false),
    myNewEdgeDestination(newEdgeDestination),
    myProbability(probability) {
}


GNEDestProbReroute::~GNEDestProbReroute() {}


void
GNEDestProbReroute::moveGeometry(const Position&, const Position&) {
    // This additional cannot be moved
}


void
GNEDestProbReroute::commitGeometryMoving(const Position&, GNEUndoList*) {
    // This additional cannot be moved
}

void
GNEDestProbReroute::updateGeometry(bool /*updateGrid*/) {
    // Currently this additional doesn't own a Geometry
}


Position
GNEDestProbReroute::getPositionInView() const {
    return myFirstAdditionalParent->getPositionInView();
}


std::string
GNEDestProbReroute::getParentName() const {
    return myFirstAdditionalParent->getID();
}


void
GNEDestProbReroute::drawGL(const GUIVisualizationSettings&) const {
    // Currently This additional isn't drawn
}


std::string
GNEDestProbReroute::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_EDGE:
            return myNewEdgeDestination->getID();
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
GNEDestProbReroute::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_EDGE:
        case SUMO_ATTR_PROB:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEDestProbReroute::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_EDGE:
            return (myViewNet->getNet()->retrieveEdge(value, false) != nullptr);
        case SUMO_ATTR_PROB:
            return canParse<double>(value) && parse<double>(value) >= 0 && parse<double>(value) <= 1;
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNEDestProbReroute::getPopUpID() const {
    return toString(getTag());
}


std::string
GNEDestProbReroute::getHierarchyName() const {
    return toString(getTag()) + ": " + myNewEdgeDestination->getID();
}

// ===========================================================================
// private
// ===========================================================================

void
GNEDestProbReroute::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_EDGE:
            myNewEdgeDestination = myViewNet->getNet()->retrieveEdge(value);
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
