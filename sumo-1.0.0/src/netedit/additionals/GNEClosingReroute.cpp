/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEClosingReroute.cpp
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
#include "GNEClosingReroute.h"
#include <netedit/netelements/GNEEdge.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/dialogs/GNERerouterIntervalDialog.h>

#include <netedit/GNEUndoList.h>
#include "GNERerouter.h"
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>
#include "GNERerouterInterval.h"


// ===========================================================================
// member method definitions
// ===========================================================================

GNEClosingReroute::GNEClosingReroute(GNERerouterIntervalDialog* rerouterIntervalDialog) :
    GNEAdditional(rerouterIntervalDialog->getEditedAdditional(), rerouterIntervalDialog->getEditedAdditional()->getViewNet(), GLO_CALIBRATOR, SUMO_TAG_CLOSING_REROUTE, "", false),
    myClosedEdge(rerouterIntervalDialog->getEditedAdditional()->getFirstAdditionalParent()->getEdgeChilds().at(0)) {
    // fill closing reroute interval with default values
    setDefaultValues();
}


GNEClosingReroute::GNEClosingReroute(GNEAdditional* rerouterIntervalParent, GNEEdge* closedEdge, SVCPermissions permissions) :
    GNEAdditional(rerouterIntervalParent, rerouterIntervalParent->getViewNet(), GLO_CALIBRATOR, SUMO_TAG_CLOSING_REROUTE, "", false),
    myClosedEdge(closedEdge),
    myPermissions(permissions) {
}


GNEClosingReroute::~GNEClosingReroute() {}


void
GNEClosingReroute::moveGeometry(const Position&, const Position&) {
    // This additional cannot be moved
}


void
GNEClosingReroute::commitGeometryMoving(const Position&, GNEUndoList*) {
    // This additional cannot be moved
}


void
GNEClosingReroute::updateGeometry(bool /*updateGrid*/) {
    // Currently this additional doesn't own a Geometry
}


Position
GNEClosingReroute::getPositionInView() const {
    return myFirstAdditionalParent->getPositionInView();
}


std::string
GNEClosingReroute::getParentName() const {
    return myFirstAdditionalParent->getID();
}


void
GNEClosingReroute::drawGL(const GUIVisualizationSettings& /* s */) const {
    // Currently this additional isn't drawn
}


std::string
GNEClosingReroute::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_EDGE:
            return myClosedEdge->getID();
        case SUMO_ATTR_ALLOW:
            return getVehicleClassNames(myPermissions);
        case SUMO_ATTR_DISALLOW:
            return getVehicleClassNames(invertPermissions(myPermissions));
        case GNE_ATTR_PARENT:
            return myFirstAdditionalParent->getID();
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEClosingReroute::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_EDGE:
        case SUMO_ATTR_ALLOW:
        case SUMO_ATTR_DISALLOW:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEClosingReroute::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_EDGE:
            return (myViewNet->getNet()->retrieveEdge(value, false) != nullptr);
        case SUMO_ATTR_ALLOW:
            return canParseVehicleClasses(value);
        case SUMO_ATTR_DISALLOW:
            return canParseVehicleClasses(value);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNEClosingReroute::getPopUpID() const {
    return toString(getTag());
}


std::string
GNEClosingReroute::getHierarchyName() const {
    return toString(getTag()) + ": " + myClosedEdge->getID();
}

// ===========================================================================
// private
// ===========================================================================

void
GNEClosingReroute::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_EDGE:
            myClosedEdge = myViewNet->getNet()->retrieveEdge(value);
            break;
        case SUMO_ATTR_ALLOW:
            myPermissions = parseVehicleClasses(value);
            break;
        case SUMO_ATTR_DISALLOW:
            myPermissions = invertPermissions(parseVehicleClasses(value));
            break;
        case GNE_ATTR_GENERIC:
            setGenericParametersStr(value);
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}

/****************************************************************************/
