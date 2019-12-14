/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNERouteProbe.cpp
/// @author  Pablo Alvarez Lopez
/// @date    May 2016
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
#include <netedit/GNEViewNet.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNENet.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/GNEViewParent.h>

#include "GNERouteProbe.h"


// ===========================================================================
// member method definitions
// ===========================================================================

GNERouteProbe::GNERouteProbe(const std::string& id, GNEViewNet* viewNet, GNEEdge* edge, const std::string& frequency, const std::string& name, const std::string& filename, double begin) :
    GNEAdditional(id, viewNet, GLO_ROUTEPROBE, SUMO_TAG_ROUTEPROBE, name, false),
    myEdge(edge),
    myFrequency(frequency),
    myFilename(filename),
    myBegin(begin),
    myRelativePositionY(0) {
}


GNERouteProbe::~GNERouteProbe() {
}


void
GNERouteProbe::updateGeometry(bool updateGrid) {
    // first check if object has to be removed from grid (SUMOTree)
    if (updateGrid) {
        myViewNet->getNet()->removeGLObjectFromGrid(this);
    }

    // Clear all containers
    myShapeRotations.clear();
    myShapeLengths.clear();

    // clear Shape
    myShape.clear();

    // obtain relative position of routeProbe in edge
    myRelativePositionY = 2 * myEdge->getRouteProbeRelativePosition(this);

    // get lanes of edge
    GNELane* firstLane = myEdge->getLanes().at(0);

    // Get shape of lane parent
    double offset = firstLane->getShape().length() < 0.5 ? firstLane->getShape().length() : 0.5;
    myShape.push_back(firstLane->getShape().positionAtOffset(offset));

    // Obtain first position
    Position f = myShape[0] - Position(1, 0);

    // Obtain next position
    Position s = myShape[0] + Position(1, 0);

    // Save rotation (angle) of the vector constructed by points f and s
    myShapeRotations.push_back(firstLane->getShape().rotationDegreeAtOffset(offset) * -1);

    // Set block icon position
    myBlockIconPosition = myShape.getLineCenter();

    // Set offset of the block icon
    myBlockIconOffset = Position(1.1, (-3.06) - myRelativePositionY);

    // Set block icon rotation, and using their rotation for logo
    setBlockIconRotation(firstLane);

    // last step is to check if object has to be added into grid (SUMOTree) again
    if (updateGrid) {
        myViewNet->getNet()->addGLObjectIntoGrid(this);
    }
}


Position
GNERouteProbe::getPositionInView() const {
    if (myEdge->getLanes().front()->getShape().length() < 0.5) {
        return myEdge->getLanes().front()->getShape().front();
    } else {
        Position A = myEdge->getLanes().front()->getShape().positionAtOffset(0.5);
        Position B = myEdge->getLanes().back()->getShape().positionAtOffset(0.5);

        // return Middle point
        return Position((A.x() + B.x()) / 2, (A.y() + B.y()) / 2);
    }
}


void
GNERouteProbe::moveGeometry(const Position&, const Position&) {
    // This additional cannot be moved
}


void
GNERouteProbe::commitGeometryMoving(const Position&, GNEUndoList*) {
    // This additional cannot be moved
}


std::string
GNERouteProbe::getParentName() const {
    return myEdge->getMicrosimID();
}


void
GNERouteProbe::drawGL(const GUIVisualizationSettings& s) const {
    // get values
    glPushName(getGlID());
    double width = (double) 2.0 * s.scale;
    glLineWidth(1.0);
    const double exaggeration = s.addSize.getExaggeration(s);
    const int numberOfLanes = int(myEdge->getLanes().size());

    // set color
    if (isAttributeCarrierSelected()) {
        GLHelper::setColor(myViewNet->getNet()->selectedAdditionalColor);
    } else {
        GLHelper::setColor(RGBColor(255, 216, 0));
    }

    // draw shape
    glPushMatrix();
    glTranslated(0, 0, getType());
    glTranslated(myShape[0].x(), myShape[0].y(), 0);
    glRotated(myShapeRotations[0], 0, 0, 1);
    glScaled(exaggeration, exaggeration, 1);
    glTranslated(-1.6, -1.6, 0);
    glBegin(GL_QUADS);
    glVertex2d(0,  0.25);
    glVertex2d(0, -0.25);
    glVertex2d((numberOfLanes * 3.3), -0.25);
    glVertex2d((numberOfLanes * 3.3),  0.25);
    glEnd();
    glTranslated(0, 0, .01);
    glBegin(GL_LINES);
    glVertex2d(0, 0.25 - .1);
    glVertex2d(0, -0.25 + .1);
    glEnd();

    // position indicator (White)
    if ((width * exaggeration > 1) && !s.drawForSelecting) {
        if (isAttributeCarrierSelected()) {
            GLHelper::setColor(myViewNet->getNet()->selectionColor);
        } else {
            GLHelper::setColor(RGBColor::WHITE);
        }
        glRotated(90, 0, 0, -1);
        glBegin(GL_LINES);
        glVertex2d(0, 0);
        glVertex2d(0, (numberOfLanes * 3.3));
        glEnd();
    }

    // Pop shape matrix
    glPopMatrix();

    // Add a draw matrix for drawing logo
    glPushMatrix();
    glTranslated(myShape[0].x(), myShape[0].y(), getType());
    glRotated(myShapeRotations[0], 0, 0, 1);
    glTranslated((-2.56) - myRelativePositionY, (-1.6), 0);

    // Draw icon depending of Route Probe is selected and if isn't being drawn for selecting
    if (s.drawForSelecting) {
        GLHelper::setColor(RGBColor::YELLOW);
        GLHelper::drawBoxLine(Position(0, 1), 0, 2, 1);
    } else {
        glColor3d(1, 1, 1);
        glRotated(-90, 0, 0, 1);
        if (isAttributeCarrierSelected()) {
            GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_ROUTEPROBESELECTED), 1);
        } else {
            GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GNETEXTURE_ROUTEPROBE), 1);
        }
    }

    // Pop logo matrix
    glPopMatrix();

    // Check if the distance is enought to draw details
    if ((s.scale * exaggeration >= 10) && !s.drawForSelecting) {
        // Show Lock icon depending of the Edit mode
        drawLockIcon(0.4);
    }

    // draw name
    drawName(getCenteringBoundary().getCenter(), s.scale, s.addName);

    // check if dotted contour has to be drawn
    if (!s.drawForSelecting && (myViewNet->getACUnderCursor() == this)) {
        GLHelper::drawShapeDottedContour(getType(), myShape[0], 2, 2, myShapeRotations[0], (-2.56) - myRelativePositionY, -1.6);
    }

    // pop name
    glPopName();
}


std::string
GNERouteProbe::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getAdditionalID();
        case SUMO_ATTR_EDGE:
            return myEdge->getID();
        case SUMO_ATTR_NAME:
            return myAdditionalName;
        case SUMO_ATTR_FILE:
            return myFilename;
        case SUMO_ATTR_FREQUENCY:
            return toString(myFrequency);
        case SUMO_ATTR_BEGIN:
            return toString(myBegin);
        case GNE_ATTR_SELECTED:
            return toString(isAttributeCarrierSelected());
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERouteProbe::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_EDGE:
        case SUMO_ATTR_NAME:
        case SUMO_ATTR_FILE:
        case SUMO_ATTR_FREQUENCY:
        case SUMO_ATTR_BEGIN:
        case GNE_ATTR_SELECTED:
        case GNE_ATTR_GENERIC:
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNERouteProbe::getPopUpID() const {
    return toString(getTag());
}


std::string
GNERouteProbe::getHierarchyName() const {
    return toString(getTag()) + ": " + getAttribute(SUMO_ATTR_BEGIN);
}

// ===========================================================================
// private
// ===========================================================================

bool
GNERouteProbe::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_EDGE:
            if (myViewNet->getNet()->retrieveEdge(value, false) != nullptr) {
                return true;
            } else {
                return false;
            }
        case SUMO_ATTR_NAME:
            return SUMOXMLDefinitions::isValidAttribute(value);
        case SUMO_ATTR_FILE:
            return SUMOXMLDefinitions::isValidFilename(value);
        case SUMO_ATTR_FREQUENCY:
            if (value.empty()) {
                return true;
            } else {
                return canParse<double>(value) && (parse<double>(value) >= 0);
            }
        case SUMO_ATTR_BEGIN:
            return canParse<double>(value) && (parse<double>(value) >= 0);
        case GNE_ATTR_SELECTED:
            return canParse<bool>(value);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERouteProbe::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            changeAdditionalID(value);
            break;
        case SUMO_ATTR_EDGE:
            myEdge = changeEdge(myEdge, value);
            break;
        case SUMO_ATTR_NAME:
            myAdditionalName = value;
            break;
        case SUMO_ATTR_FILE:
            myFilename = value;
            break;
        case SUMO_ATTR_FREQUENCY:
            myFrequency = value;
            break;
        case SUMO_ATTR_BEGIN:
            myBegin = parse<double>(value);
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
