/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNERerouterInterval.cpp
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
#include <utils/common/MsgHandler.h>
#include "GNERerouterInterval.h"
#include <netedit/netelements/GNEEdge.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/dialogs/GNERerouterDialog.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>

#include "GNERerouter.h"

// ===========================================================================
// member method definitions
// ===========================================================================

GNERerouterInterval::GNERerouterInterval(GNERerouterDialog* rerouterDialog) :
    GNEAdditional(rerouterDialog->getEditedAdditional(), rerouterDialog->getEditedAdditional()->getViewNet(), GLO_REROUTER, SUMO_TAG_INTERVAL, "", false) {
    // fill reroute interval with default values
    setDefaultValues();
}


GNERerouterInterval::GNERerouterInterval(GNEAdditional* rerouterParent, double begin, double end) :
    GNEAdditional(rerouterParent, rerouterParent->getViewNet(), GLO_REROUTER, SUMO_TAG_INTERVAL, "", false),
    myBegin(begin),
    myEnd(end) {
}


GNERerouterInterval::~GNERerouterInterval() {}

void
GNERerouterInterval::moveGeometry(const Position&, const Position&) {
    // This additional cannot be moved
}


void
GNERerouterInterval::commitGeometryMoving(const Position&, GNEUndoList*) {
    // This additional cannot be moved
}


void
GNERerouterInterval::updateGeometry(bool /*updateGrid*/) {
    // Currently this additional doesn't own a Geometry
}


Position
GNERerouterInterval::getPositionInView() const {
    return myFirstAdditionalParent->getPositionInView();
}


std::string
GNERerouterInterval::getParentName() const {
    return myFirstAdditionalParent->getID();
}


void
GNERerouterInterval::drawGL(const GUIVisualizationSettings&) const {
    // Currently This additional isn't drawn
}


std::string
GNERerouterInterval::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_BEGIN:
            return toString(myBegin);
        case SUMO_ATTR_END:
            return toString(myEnd);
        case GNE_ATTR_PARENT:
            return myFirstAdditionalParent->getID();
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERerouterInterval::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID: {
            // change ID of Rerouter Interval
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            // Change Ids of all Rerouter childs
            for (auto i : myAdditionalChilds) {
                i->setAttribute(SUMO_ATTR_ID, generateAdditionalChildID(i->getTag()), undoList);
            }
            break;
        }
        case SUMO_ATTR_BEGIN:
        case SUMO_ATTR_END:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNERerouterInterval::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_BEGIN:
            return canParse<double>(value) && (parse<double>(value) >= 0) && (parse<double>(value) < myEnd);
        case SUMO_ATTR_END:
            return canParse<double>(value) && (parse<double>(value) >= 0) && (parse<double>(value) > myBegin);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNERerouterInterval::getPopUpID() const {
    return toString(getTag());
}


std::string
GNERerouterInterval::getHierarchyName() const {
    return toString(getTag()) + ": " + getAttribute(SUMO_ATTR_BEGIN) + " -> " + getAttribute(SUMO_ATTR_END);
}

// ===========================================================================
// private
// ===========================================================================

void
GNERerouterInterval::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_BEGIN:
            myBegin = parse<double>(value);
            break;
        case SUMO_ATTR_END:
            myEnd = parse<double>(value);
            break;
        case GNE_ATTR_GENERIC:
            setGenericParametersStr(value);
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}

/****************************************************************************/
