/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEVariableSpeedSignStep.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Apr 2017
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
#include <netedit/dialogs/GNEVariableSpeedSignDialog.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/GNEUndoList.h>

#include "GNEVariableSpeedSignStep.h"
#include "GNEVariableSpeedSign.h"


// ===========================================================================
// member method definitions
// ===========================================================================

GNEVariableSpeedSignStep::GNEVariableSpeedSignStep(GNEVariableSpeedSignDialog* variableSpeedSignDialog) :
    GNEAdditional(variableSpeedSignDialog->getEditedAdditional(), variableSpeedSignDialog->getEditedAdditional()->getViewNet(), GLO_VSS, SUMO_TAG_STEP, "", false) {
    // fill VSS Step with default values
    setDefaultValues();
    // set time Attribute manually
    if (myFirstAdditionalParent->getAdditionalChilds().size() > 0) {
        myTime = parse<double>(myFirstAdditionalParent->getAdditionalChilds().back()->getAttribute(SUMO_ATTR_TIME)) + 1;
    } else {
        myTime = 0;
    }
}


GNEVariableSpeedSignStep::GNEVariableSpeedSignStep(GNEAdditional* variableSpeedSignParent, double time, double speed) :
    GNEAdditional(variableSpeedSignParent, variableSpeedSignParent->getViewNet(), GLO_VSS, SUMO_TAG_STEP, "", false),
    myTime(time),
    mySpeed(speed) {
}


GNEVariableSpeedSignStep::~GNEVariableSpeedSignStep() {}


double
GNEVariableSpeedSignStep::getTime() const {
    return myTime;
}


void
GNEVariableSpeedSignStep::moveGeometry(const Position&, const Position&) {
    // This additional cannot be moved
}


void
GNEVariableSpeedSignStep::commitGeometryMoving(const Position&, GNEUndoList*) {
    // This additional cannot be moved
}


void
GNEVariableSpeedSignStep::updateGeometry(bool /*updateGrid*/) {
    // Currently this additional doesn't own a Geometry
}


Position
GNEVariableSpeedSignStep::getPositionInView() const {
    return myFirstAdditionalParent->getPositionInView();
}


std::string
GNEVariableSpeedSignStep::getParentName() const {
    return myFirstAdditionalParent->getID();
}


void
GNEVariableSpeedSignStep::drawGL(const GUIVisualizationSettings&) const {
    // Currently This additional isn't drawn
}


std::string
GNEVariableSpeedSignStep::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_TIME:
            return toString(myTime);
        case SUMO_ATTR_SPEED:
            return toString(mySpeed);
        case GNE_ATTR_PARENT:
            return myFirstAdditionalParent->getID();
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEVariableSpeedSignStep::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_TIME:
        case SUMO_ATTR_SPEED:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEVariableSpeedSignStep::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_TIME:
            if (canParse<double>(value)) {
                // Check that
                double newTime = parse<double>(value);
                // Only allowed positiv times
                if (newTime < 0) {
                    return false;
                }
                // check that there isn't duplicate times
                int counter = 0;
                for (auto i : myFirstAdditionalParent->getAdditionalChilds()) {
                    if (parse<double>(i->getAttribute(SUMO_ATTR_TIME)) == newTime) {
                        counter++;
                    }
                }
                return (counter <= 1);
            } else {
                return false;
            }
        case SUMO_ATTR_SPEED:
            return canParse<double>(value) && (parse<double>(value) >= 0);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNEVariableSpeedSignStep::getPopUpID() const {
    return toString(getTag());
}


std::string
GNEVariableSpeedSignStep::getHierarchyName() const {
    return toString(getTag()) + ": " + getAttribute(SUMO_ATTR_TIME);
}

// ===========================================================================
// private
// ===========================================================================

void
GNEVariableSpeedSignStep::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_TIME:
            myTime = parse<double>(value);
            break;
        case SUMO_ATTR_SPEED:
            mySpeed = parse<double>(value);
            break;
        case GNE_ATTR_GENERIC:
            setGenericParametersStr(value);
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


/****************************************************************************/
