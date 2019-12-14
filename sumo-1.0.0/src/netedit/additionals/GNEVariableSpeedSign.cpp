/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEVariableSpeedSign.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
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
#include <utils/geom/GeomConvHelper.h>
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
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/dialogs/GNEVariableSpeedSignDialog.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/GNEViewParent.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNENet.h>

#include "GNEVariableSpeedSign.h"
#include "GNEVariableSpeedSignStep.h"


// ===========================================================================
// member method definitions
// ===========================================================================

GNEVariableSpeedSign::GNEVariableSpeedSign(const std::string& id, GNEViewNet* viewNet, const Position& pos, const std::vector<GNELane*>& lanes, const std::string& name, bool blockMovement) :
    GNEAdditional(id, viewNet, GLO_VSS, SUMO_TAG_VSS, name, blockMovement, lanes),
    myPosition(pos) {
}


GNEVariableSpeedSign::~GNEVariableSpeedSign() {
}


void
GNEVariableSpeedSign::updateGeometry(bool updateGrid) {
    // first check if object has to be removed from grid (SUMOTree)
    if (updateGrid) {
        myViewNet->getNet()->removeGLObjectFromGrid(this);
    }

    // Clear shape
    myShape.clear();

    // Set block icon position
    myBlockIconPosition = myPosition;

    // Set block icon offset
    myBlockIconOffset = Position(-0.5, -0.5);

    // Set block icon rotation, and using their rotation for draw logo
    setBlockIconRotation();

    // Set position
    myShape.push_back(myPosition);

    // clear mySymbolsPositionAndRotation
    mySymbolsPositionAndRotation.clear();

    // update child connections
    updateChildConnections();

    // last step is to check if object has to be added into grid (SUMOTree) again
    if (updateGrid) {
        myViewNet->getNet()->addGLObjectIntoGrid(this);
    }
}


Position
GNEVariableSpeedSign::getPositionInView() const {
    return myPosition;
}


void
GNEVariableSpeedSign::openAdditionalDialog() {
    // Open VSS dialog
    GNEVariableSpeedSignDialog(this);
}


void
GNEVariableSpeedSign::moveGeometry(const Position& oldPos, const Position& offset) {
    // restore old position, apply offset and update Geometry
    myPosition = oldPos;
    myPosition.add(offset);
    updateGeometry(false);
}


void
GNEVariableSpeedSign::commitGeometryMoving(const Position& oldPos, GNEUndoList* undoList) {
    // commit new position allowing undo/redo
    undoList->p_begin("position of " + toString(getTag()));
    undoList->p_add(new GNEChange_Attribute(this, SUMO_ATTR_POSITION, toString(myPosition), true, toString(oldPos)));
    undoList->p_end();
}


std::string
GNEVariableSpeedSign::getParentName() const {
    return myViewNet->getNet()->getMicrosimID();
}


void
GNEVariableSpeedSign::drawGL(const GUIVisualizationSettings& s) const {
    // Start drawing adding an gl identificator
    glPushName(getGlID());

    // Add a draw matrix for drawing logo
    glPushMatrix();
    glTranslated(myShape[0].x(), myShape[0].y(), getType());

    // Draw icon depending of variable speed sign is or if isn't being drawn for selecting
    if (s.drawForSelecting) {
        GLHelper::setColor(RGBColor::WHITE);
        GLHelper::drawBoxLine(Position(0, 1), 0, 2, 1);
    } else {
        glColor3d(1, 1, 1);
        glRotated(180, 0, 0, 1);
        if (isAttributeCarrierSelected()) {
            GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_VARIABLESPEEDSIGNSELECTED), 1);
        } else {
            GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_VARIABLESPEEDSIGN), 1);
        }
    }

    // Pop draw icon matrix
    glPopMatrix();

    // Only lock and childs if isn't being drawn for selecting
    if (!s.drawForSelecting) {

        // Show Lock icon depending of the Edit mode
        drawLockIcon(0.4);

        // obtain exxageration
        const double exaggeration = s.addSize.getExaggeration(s);

        // iterate over symbols and rotation
        for (auto i : mySymbolsPositionAndRotation) {
            glPushMatrix();
            glScaled(exaggeration, exaggeration, 1);
            glTranslated(i.first.x(), i.first.y(), getType());
            glRotated(-1 * i.second, 0, 0, 1);
            glTranslated(0, -1.5, 0);

            int noPoints = 9;
            if (s.scale > 25) {
                noPoints = (int)(9.0 + s.scale / 10.0);
                if (noPoints > 36) {
                    noPoints = 36;
                }
            }
            glColor3d(1, 0, 0);
            GLHelper::drawFilledCircle((double) 1.3, noPoints);
            if (s.scale >= 5) {
                glTranslated(0, 0, .1);
                glColor3d(0, 0, 0);
                GLHelper::drawFilledCircle((double) 1.1, noPoints);
                // draw the speed string
                //draw
                glColor3d(1, 1, 0);
                glTranslated(0, 0, .1);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                // draw last value string
                GLHelper::drawText("S", Position(0, 0), .1, 1.2, RGBColor(255, 255, 0), 180);
            }
            glPopMatrix();
        }

        // Draw connections
        drawChildConnections();
    }

    // Pop symbol matrix
    glPopMatrix();

    // Draw name if isn't being drawn for selecting
    if (!s.drawForSelecting) {
        drawName(getCenteringBoundary().getCenter(), s.scale, s.addName);
    }

    // check if dotted contour has to be drawn
    if (!s.drawForSelecting && (myViewNet->getACUnderCursor() == this)) {
        GLHelper::drawShapeDottedContour(getType(), myPosition, 2, 2);
        // draw shape dotte contour aroud alld connections between child and parents
        for (auto i : myChildConnectionPositions) {
            GLHelper::drawShapeDottedContour(getType(), i, 0);
        }
        // draw rerouter symbol over all lanes
        for (auto i : mySymbolsPositionAndRotation) {
            GLHelper::drawShapeDottedContour(getType(), i.first, 2.6, 2.6, -1 * i.second, 0, -1.5);
        }
    }

    // Pop name
    glPopName();
}


std::string
GNEVariableSpeedSign::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_LANES:
            return parseIDs(myLaneChilds);
        case SUMO_ATTR_POSITION:
            return toString(myPosition);
        case SUMO_ATTR_NAME:
            return myAdditionalName;
        case GNE_ATTR_BLOCK_MOVEMENT:
            return toString(myBlockMovement);
        case GNE_ATTR_SELECTED:
            return toString(isAttributeCarrierSelected());
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEVariableSpeedSign::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID: {
            // change ID of Rerouter Interval
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            // Change Ids of all Variable Speed Sign
            for (auto i : myAdditionalChilds) {
                i->setAttribute(SUMO_ATTR_ID, generateAdditionalChildID(SUMO_TAG_STEP), undoList);
            }
            break;
        }
        case SUMO_ATTR_LANES:
        case SUMO_ATTR_POSITION:
        case SUMO_ATTR_NAME:
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
GNEVariableSpeedSign::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_POSITION:
            return canParse<Position>(value);
        case SUMO_ATTR_LANES:
            if (value.empty()) {
                return false;
            } else {
                return canParse<std::vector<GNELane*> >(myViewNet->getNet(), value, false);
            }
        case SUMO_ATTR_NAME:
            return SUMOXMLDefinitions::isValidAttribute(value);
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
GNEVariableSpeedSign::getPopUpID() const {
    return toString(getTag()) + ": " + getID();
}


std::string
GNEVariableSpeedSign::getHierarchyName() const {
    return toString(getTag());
}

// ===========================================================================
// private
// ===========================================================================

void
GNEVariableSpeedSign::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_LANES:
            myLaneChilds = parse<std::vector<GNELane*> >(myViewNet->getNet(), value);
            break;
        case SUMO_ATTR_POSITION:
            myPosition = parse<Position>(value);
            break;
        case SUMO_ATTR_NAME:
            myAdditionalName = value;
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
