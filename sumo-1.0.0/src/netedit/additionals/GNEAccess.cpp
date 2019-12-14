/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEAccess.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2018
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
#include <netbuild/NBEdge.h>
#include <utils/geom/PositionVector.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/common/ToString.h>
#include <utils/geom/GeomHelper.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITextureSubSys.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/images/GUITexturesHelper.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNENet.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/GNEViewParent.h>

#include "GNEAccess.h"
#include "GNEBusStop.h"
#include "GNEAdditionalHandler.h"

// ===========================================================================
// member method definitions
// ===========================================================================

GNEAccess::GNEAccess(GNEAdditional* busStop, GNELane* lane, GNEViewNet* viewNet, const std::string& pos, const std::string& length, bool friendlyPos, bool blockMovement) :
    GNEAdditional(busStop, viewNet, GLO_ACCESS, SUMO_TAG_ACCESS, "", blockMovement),
    myLane(lane),
    myPositionOverLane(pos),
    myLength(length),
    myFriendlyPosition(friendlyPos) {
}


GNEAccess::~GNEAccess() {
}


void
GNEAccess::moveGeometry(const Position& oldPos, const Position& offset) {
    // Calculate new position using old position
    Position newPosition = oldPos;
    newPosition.add(offset);
    myPositionOverLane = toString(myLane->getShape().nearest_offset_to_point2D(newPosition, false));
    // Update geometry
    updateGeometry(false);
}


void
GNEAccess::commitGeometryMoving(const Position& oldPos, GNEUndoList* undoList) {
    if (!myBlockMovement) {
        // restore old position before commit new position
        double originalPosOverLane = myLane->getShape().nearest_offset_to_point2D(oldPos, false);
        // commit new position allowing undo/redo
        undoList->p_begin("position of " + toString(getTag()));
        undoList->p_add(new GNEChange_Attribute(this, SUMO_ATTR_POSITION, myPositionOverLane, true, toString(originalPosOverLane)));
        undoList->p_end();
    }
}


void
GNEAccess::updateGeometry(bool updateGrid) {
    // first check if object has to be removed from grid (SUMOTree)
    if (updateGrid) {
        myViewNet->getNet()->removeGLObjectFromGrid(this);
    }

    // Clear all containers
    myShapeRotations.clear();
    myShapeLengths.clear();

    // Get shape of lane parent
    myShape = myLane->getShape();

    // set start position
    double fixedPositionOverLane;
    if (!canParse<double>(myPositionOverLane)) {
        fixedPositionOverLane = myLane->getParentEdge().getNBEdge()->getFinalLength();
    } else if (parse<double>(myPositionOverLane) < 0) {
        fixedPositionOverLane = 0;
    } else if (parse<double>(myPositionOverLane) > myLane->getParentEdge().getNBEdge()->getFinalLength()) {
        fixedPositionOverLane = myLane->getParentEdge().getNBEdge()->getFinalLength();
    } else {
        fixedPositionOverLane = parse<double>(myPositionOverLane);
    }
    // obtain position
    myShape[0] = myLane->getShape().positionAtOffset(fixedPositionOverLane * myLane->getLengthGeometryFactor());

    // Save rotation (angle) of the vector constructed by points f and s
    myShapeRotations.push_back(myLane->getShape().rotationDegreeAtOffset(fixedPositionOverLane) * -1);

    // Set block icon position
    myBlockIconPosition = myShape.getLineCenter();

    // Set offset of the block icon
    myBlockIconOffset = Position(-1, 0);

    // Set block icon rotation, and using their rotation for logo
    setBlockIconRotation(myLane);

    // last step is to check if object has to be added into grid (SUMOTree) again
    if (updateGrid) {
        myViewNet->getNet()->addGLObjectIntoGrid(this);
    }
}


Position
GNEAccess::getPositionInView() const {
    if (!canParse<double>(myPositionOverLane)) {
        return myLane->getShape().front();
    } else {
        double posOverLane = parse<double>(myPositionOverLane);
        if (posOverLane < 0) {
            return myLane->getShape().front();
        } else if (posOverLane > myLane->getShape().length()) {
            return myLane->getShape().back();
        } else {
            return myLane->getShape().positionAtOffset(posOverLane);
        }
    }
}


bool
GNEAccess::isAccessPositionFixed() const {
    // with friendly position enabled position are "always fixed"
    if (myFriendlyPosition) {
        return true;
    } else {
        if (canParse<double>(myPositionOverLane)) {
            return (parse<double>(myPositionOverLane) >= 0) && ((parse<double>(myPositionOverLane)) <= myLane->getParentEdge().getNBEdge()->getFinalLength());
        } else {
            return false;
        }
    }
}


GNEEdge&
GNEAccess::getEdge() const {
    return myLane->getParentEdge();
}


std::string
GNEAccess::getParentName() const {
    return myFirstAdditionalParent->getID();
}


void
GNEAccess::drawGL(const GUIVisualizationSettings& s) const {
    // Obtain exaggeration of the draw
    const double exaggeration = s.addSize.getExaggeration(s);
    // Start drawing adding an gl identificator
    glPushName(getGlID());
    // push matrix
    glPushMatrix();
    // set color depending of selection
    if (mySelected) {
        GLHelper::setColor(myViewNet->getNet()->selectedAdditionalColor);
    } else {
        GLHelper::setColor(s.SUMO_color_busStop);
    }
    glTranslated(myShape[0].x(), myShape[0].y(), GLO_ACCESS);
    // draw circle
    if (s.drawForSelecting) {
        GLHelper::drawFilledCircle((double) 0.5 * exaggeration, 8);
    } else {
        std::vector<Position> vertices = GLHelper::drawFilledCircleReturnVertices((double) 0.5 * exaggeration, 16);
        // check if dotted contour has to be drawn
        if (myViewNet->getACUnderCursor() == this) {
            GLHelper::drawShapeDottedContour(getType(), vertices);
        }
    }
    // pop matrix
    glPopMatrix();
    // pop gl identficador
    glPopName();
}


std::string
GNEAccess::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_LANE:
            return myLane->getID();
        case SUMO_ATTR_POSITION:
            return toString(myPositionOverLane);
        case SUMO_ATTR_LENGTH:
            return toString(myLength);
        case SUMO_ATTR_FRIENDLY_POS:
            return toString(myFriendlyPosition);
        case GNE_ATTR_BLOCK_MOVEMENT:
            return toString(myBlockMovement);
        case GNE_ATTR_PARENT:
            return myFirstAdditionalParent->getID();
        case GNE_ATTR_SELECTED:
            return toString(isAttributeCarrierSelected());
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEAccess::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_LANE:
        case SUMO_ATTR_POSITION:
        case SUMO_ATTR_LENGTH:
        case SUMO_ATTR_FRIENDLY_POS:
        case GNE_ATTR_BLOCK_MOVEMENT:
        case GNE_ATTR_SELECTED:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEAccess::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_LANE: {
            GNELane* lane = myViewNet->getNet()->retrieveLane(value, false);
            if (lane != nullptr) {
                if (myLane->getParentEdge().getID() != lane->getParentEdge().getID()) {
                    return GNEAdditionalHandler::accessCanBeCreated(myFirstAdditionalParent, lane->getParentEdge());
                } else {
                    return true;
                }
            } else {
                return false;
            }
        }
        case SUMO_ATTR_POSITION:
            if (value.empty()) {
                return true;
            } else {
                return canParse<double>(value);
            }
        case SUMO_ATTR_LENGTH:
            if (value.empty()) {
                return true;
            } else {
                return (canParse<double>(value) && (parse<double>(value) >= 0));
            }
        case SUMO_ATTR_FRIENDLY_POS:
            return canParse<bool>(value);
        case GNE_ATTR_BLOCK_MOVEMENT:
            return canParse<bool>(value);
        case GNE_ATTR_SELECTED:
            return canParse<bool>(value);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNEAccess::getPopUpID() const {
    return toString(getTag());
}


std::string
GNEAccess::getHierarchyName() const {
    return toString(getTag()) + ": " + myLane->getParentEdge().getID();
}

// ===========================================================================
// private
// ===========================================================================

void
GNEAccess::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_LANE:
            myLane = changeLane(myLane, value);
            break;
        case SUMO_ATTR_POSITION:
            myPositionOverLane = value;
            break;
        case SUMO_ATTR_LENGTH:
            myLength = value;
            break;
        case SUMO_ATTR_FRIENDLY_POS:
            myFriendlyPosition = parse<bool>(value);
            break;
        case GNE_ATTR_BLOCK_MOVEMENT:
            myBlockMovement = parse<bool>(value);
            break;
        case GNE_ATTR_SELECTED:
            if (parse<bool>(value)) {
                selectAttributeCarrier();
            } else {
                unselectAttributeCarrier();
            }
            break;
        case GNE_ATTR_GENERIC:
            setGenericParametersStr(value);
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
    // After setting attribute always update Geometry
    updateGeometry(true);
}

/****************************************************************************/
