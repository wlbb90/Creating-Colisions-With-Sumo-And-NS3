/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEConnection.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2016
/// @version $Id$
///
// A class for visualizing connections between lanes
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <string>
#include <iostream>
#include <utility>
#include <time.h>
#include <utils/common/StringTokenizer.h>
#include <utils/foxtools/MFXUtils.h>
#include <utils/geom/PositionVector.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/gui/windows/GUIMainWindow.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>
#include <utils/common/ToString.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/globjects/GLIncludes.h>
#include <netbuild/NBLoadedSUMOTLDef.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/changes/GNEChange_TLS.h>
#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>

#include "GNEConnection.h"
#include "GNEJunction.h"
#include "GNEEdge.h"
#include "GNELane.h"
#include "GNEInternalLane.h"


// ===========================================================================
// static member definitions
// ===========================================================================
int NUM_POINTS = 5;

// ===========================================================================
// method definitions
// ===========================================================================

GNEConnection::GNEConnection(GNELane* from, GNELane* to) :
    GNENetElement(from->getNet(), "from" + from->getMicrosimID() + "to" + to->getMicrosimID(),
                  GLO_CONNECTION, SUMO_TAG_CONNECTION),
    myFromLane(from),
    myToLane(to),
    myLinkState(LINKSTATE_TL_OFF_NOSIGNAL),
    mySpecialColor(0),
    myShapeDeprecated(true) {
}


GNEConnection::~GNEConnection() {
}


void
GNEConnection::updateGeometry(bool updateGrid) {
    // Get shape of from and to lanes
    NBEdge::Connection& nbCon = getNBEdgeConnection();
    if (myShapeDeprecated) {
        // first check if object has to be removed from grid (SUMOTree)
        if (updateGrid) {
            myNet->removeGLObjectFromGrid(this);
        }
        // Clear containers
        myShape.clear();
        myShapeRotations.clear();
        myShapeLengths.clear();

        PositionVector laneShapeFrom;
        if ((int)getEdgeFrom()->getNBEdge()->getLanes().size() > nbCon.fromLane) {
            laneShapeFrom = getEdgeFrom()->getNBEdge()->getLanes().at(nbCon.fromLane).shape;
        } else {
            return;
        }
        PositionVector laneShapeTo;
        if ((int)nbCon.toEdge->getLanes().size() > nbCon.toLane) {
            laneShapeTo = nbCon.toEdge->getLanes().at(nbCon.toLane).shape;
        } else {
            return;
        }
        // Calculate shape of connection depending of the size of Junction shape
        // value obtanied from GNEJunction::drawgl
        if (nbCon.customShape.size() != 0) {
            myShape = nbCon.customShape;
        } else if (getEdgeFrom()->getNBEdge()->getToNode()->getShape().area() > 4) {
            if (nbCon.shape.size() != 0) {
                myShape = nbCon.shape;
                // only append via shape if it exists
                if (nbCon.haveVia) {
                    myShape.append(nbCon.viaShape);
                }
            } else {
                // Calculate shape so something can be drawn immidiately
                myShape = getEdgeFrom()->getNBEdge()->getToNode()->computeSmoothShape(
                              laneShapeFrom,
                              laneShapeTo,
                              NUM_POINTS, getEdgeFrom()->getNBEdge()->getTurnDestination() == nbCon.toEdge,
                              (double) 5. * (double) getEdgeFrom()->getNBEdge()->getNumLanes(),
                              (double) 5. * (double) nbCon.toEdge->getNumLanes());
            }
        } else {
            myShape.clear();
            myShape.push_back(laneShapeFrom.positionAtOffset(MAX2(0.0, laneShapeFrom.length() - 1)));
            myShape.push_back(laneShapeTo.positionAtOffset(MIN2(1.0, laneShapeFrom.length())));
        }
        // check if internal junction marker must be calculated
        if (nbCon.haveVia && (nbCon.shape.size() != 0)) {
            // create marker for interal junction waiting position (contPos)
            const double orthoLength = 0.5;
            Position pos = nbCon.shape.back();
            myInternalJunctionMarker = nbCon.shape.getOrthogonal(pos, 10, true, 0.1);
            if (myInternalJunctionMarker.length() < orthoLength) {
                myInternalJunctionMarker.extrapolate(orthoLength - myInternalJunctionMarker.length());
            }
        } else {
            myInternalJunctionMarker.clear();
        }
        // Obtain lengths and shape rotations
        int segments = (int) myShape.size() - 1;
        if (segments >= 0) {
            myShapeRotations.reserve(segments);
            myShapeLengths.reserve(segments);
            for (int i = 0; i < segments; ++i) {
                const Position& f = myShape[i];
                const Position& s = myShape[i + 1];
                myShapeLengths.push_back(f.distanceTo2D(s));
                myShapeRotations.push_back((double) atan2((s.x() - f.x()), (f.y() - s.y())) * (double) 180.0 / (double)M_PI);
            }
        }

        // mark connection as non-deprecated
        myShapeDeprecated = false;

        // last step is to check if object has to be added into grid (SUMOTree) again
        if (updateGrid) {
            myNet->addGLObjectIntoGrid(this);
        }
    }
}


Boundary
GNEConnection::getBoundary() const {
    if (myShape.size() == 0) {
        // we need to use the center of junction parent as boundary if shape is empty
        Position junctionParentPosition = myFromLane->getParentEdge().getGNEJunctionDestiny()->getPositionInView();
        return Boundary(junctionParentPosition.x() - 0.1, junctionParentPosition.y() - 0.1,
                        junctionParentPosition.x() + 0.1, junctionParentPosition.x() + 0.1);
    } else {
        return myShape.getBoxBoundary();
    }
}


GNEEdge*
GNEConnection::getEdgeFrom() const {
    return &(myFromLane->getParentEdge());
}


GNEEdge*
GNEConnection::getEdgeTo() const {
    return &(myToLane->getParentEdge());
}


GNELane*
GNEConnection::getLaneFrom() const {
    return myFromLane;
}


GNELane*
GNEConnection::getLaneTo() const {
    return myToLane;
}


int
GNEConnection::getFromLaneIndex() const {
    return myFromLane->getIndex();
}


int
GNEConnection::getToLaneIndex() const {
    return myToLane->getIndex();
}


NBEdge::Connection&
GNEConnection::getNBEdgeConnection() const {
    return getEdgeFrom()->getNBEdge()->getConnectionRef(getFromLaneIndex(), getEdgeTo()->getNBEdge(), getToLaneIndex());
}


NBConnection
GNEConnection::getNBConnection() const {
    return NBConnection(getEdgeFrom()->getNBEdge(), getFromLaneIndex(),
                        getEdgeTo()->getNBEdge(), getToLaneIndex(),
                        (int)getNBEdgeConnection().tlLinkIndex);
}


void
GNEConnection::updateID() {
    setMicrosimID(myFromLane->getMicrosimID() + " -> " + myToLane->getMicrosimID());
}


LinkState
GNEConnection::getLinkState() const {
    return myLinkState;
}


const PositionVector&
GNEConnection::getShape() const {
    if (myShape.size() > 0) {
        return myShape;
    } else {
        return getNBEdgeConnection().customShape;
    }
}


void
GNEConnection::markConnectionGeometryDeprecated() {
    myShapeDeprecated = true;
}


void
GNEConnection::updateLinkState() {
    NBEdge::Connection& nbCon = getNBEdgeConnection();
    myLinkState = getEdgeFrom()->getNBEdge()->getToNode()->getLinkState(getEdgeFrom()->getNBEdge(),
                  nbCon.toEdge,
                  nbCon.fromLane,
                  nbCon.toLane,
                  nbCon.mayDefinitelyPass,
                  nbCon.tlID);
}


GUIGLObjectPopupMenu*
GNEConnection::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
    buildPopupHeader(ret, app);
    buildCenterPopupEntry(ret);
    buildNameCopyPopupEntry(ret);
    // build selection and show parameters menu
    buildSelectionPopupEntry(ret);
    buildShowParamsPopupEntry(ret);
    // build position copy entry
    buildPositionCopyEntry(ret, false);
    // create menu commands
    FXMenuCommand* mcCustomShape = new FXMenuCommand(ret, "Set custom connection shape", 0, &parent, MID_GNE_CONNECTION_EDIT_SHAPE);
    // check if menu commands has to be disabled
    EditMode editMode = myNet->getViewNet()->getCurrentEditMode();
    const bool wrongMode = (editMode == GNE_MODE_CONNECT || editMode == GNE_MODE_TLS || editMode == GNE_MODE_CREATE_EDGE);
    if (wrongMode) {
        mcCustomShape->disable();
    }
    return ret;
}


Boundary
GNEConnection::getCenteringBoundary() const {
    Boundary b = getBoundary();
    b.grow(20);
    return b;
}


void
GNEConnection::drawGL(const GUIVisualizationSettings& s) const {
    // Check if connection must be drawed
    if (!myShapeDeprecated && myNet->getViewNet()->showConnections()) {
        // Push draw matrix 1
        glPushMatrix();
        // Push name
        glPushName(getGlID());
        // Traslate matrix
        glTranslated(0, 0, GLO_JUNCTION + 0.1); // must draw on top of junction
        // Set color
        if (isAttributeCarrierSelected() && s.junctionColorer.getActive() != 1) {
            // override with special colors (unless the color scheme is based on selection)
            GLHelper::setColor(GNENet::selectedConnectionColor);
        } else if (mySpecialColor != 0) {
            GLHelper::setColor(*mySpecialColor);
        } else {
            // Set color depending of the link state
            GLHelper::setColor(GNEInternalLane::colorForLinksState(getLinkState()));
        }
        // draw connection checking whether it is not too small if isn't being drawn for selecting
        if ((s.scale < 1.) && !s.drawForSelecting) {
            // If it's small, dra a simple line
            GLHelper::drawLine(myShape);
        } else {
            // draw a list of lines
            GLHelper::drawBoxLines(myShape, myShapeRotations, myShapeLengths, 0.2);
            glTranslated(0, 0, 0.1);
            GLHelper::setColor(GLHelper::getColor().changedBrightness(51));
            // check if internal junction marker has to be drawn
            if (myInternalJunctionMarker.size() > 0) {
                GLHelper::drawLine(myInternalJunctionMarker);
            }
        }
        // check if dotted contour has to be drawn
        if (!s.drawForSelecting && (myNet->getViewNet()->getACUnderCursor() == this)) {
            GLHelper::drawShapeDottedContour(getType(), myShape, 0.25);
        }
        // Pop name
        glPopName();
        // Pop draw matrix 1
        glPopMatrix();
    }
}


void
GNEConnection::setSpecialColor(const RGBColor* color) {
    mySpecialColor = color;
}

std::string
GNEConnection::getAttribute(SumoXMLAttr key) const {
    if (key == SUMO_ATTR_ID) {
        // used by GNEReferenceCounter
        // @note: may be called for connections without a valid nbCon reference
        return getMicrosimID();
    }
    NBEdge::Connection& nbCon = getNBEdgeConnection();
    switch (key) {
        case SUMO_ATTR_FROM:
            return getEdgeFrom()->getID();
        case SUMO_ATTR_TO:
            return nbCon.toEdge->getID();
        case SUMO_ATTR_FROM_LANE:
            return toString(nbCon.toLane);
        case SUMO_ATTR_TO_LANE:
            return toString(nbCon.toLane);
        case SUMO_ATTR_PASS:
            return toString(nbCon.mayDefinitelyPass);
        case SUMO_ATTR_KEEP_CLEAR:
            return toString(nbCon.keepClear);
        case SUMO_ATTR_CONTPOS:
            return toString(nbCon.contPos);
        case SUMO_ATTR_UNCONTROLLED:
            return toString(nbCon.uncontrolled);
        case SUMO_ATTR_VISIBILITY_DISTANCE:
            return toString(nbCon.visibility);
        case SUMO_ATTR_TLLINKINDEX:
            return toString(nbCon.tlLinkIndex);
        case SUMO_ATTR_SPEED:
            return toString(nbCon.speed);
        case SUMO_ATTR_CUSTOMSHAPE:
            return toString(nbCon.customShape);
        case GNE_ATTR_SELECTED:
            return toString(isAttributeCarrierSelected());
        case GNE_ATTR_GENERIC:
            return getGenericParametersStr();
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEConnection::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    switch (key) {
        case SUMO_ATTR_FROM:
        case SUMO_ATTR_TO:
        case SUMO_ATTR_FROM_LANE:
        case SUMO_ATTR_TO_LANE:
        case SUMO_ATTR_PASS:
        case SUMO_ATTR_KEEP_CLEAR:
        case SUMO_ATTR_CONTPOS:
        case SUMO_ATTR_UNCONTROLLED:
        case SUMO_ATTR_VISIBILITY_DISTANCE:
        case SUMO_ATTR_SPEED:
        case SUMO_ATTR_CUSTOMSHAPE:
        case GNE_ATTR_SELECTED:
        case GNE_ATTR_GENERIC:
            // no special handling
            undoList->p_add(new GNEChange_Attribute(this, key, value));
            break;
        case SUMO_ATTR_TLLINKINDEX:
            if (value != getAttribute(key)) {
                // trigger GNEChange_TLS
                undoList->p_begin("change tls linkIndex for connection");
                // make a copy
                std::set<NBTrafficLightDefinition*> defs = getEdgeFrom()->getNBEdge()->getToNode()->getControllingTLS();
                for (NBTrafficLightDefinition* tlDef : defs) {
                    NBLoadedSUMOTLDef* sumoDef = dynamic_cast<NBLoadedSUMOTLDef*>(tlDef);
                    NBTrafficLightLogic* tllogic = sumoDef ? sumoDef->getLogic() : tlDef->compute(OptionsCont::getOptions());
                    if (tllogic != 0) {
                        NBLoadedSUMOTLDef* newDef = new NBLoadedSUMOTLDef(tlDef, tllogic);
                        newDef->addConnection(getEdgeFrom()->getNBEdge(), getEdgeTo()->getNBEdge(),
                                              getLaneFrom()->getIndex(), getLaneTo()->getIndex(), parse<int>(value), false);
                        std::vector<NBNode*> nodes = tlDef->getNodes();
                        for (NBNode* node : nodes) {
                            GNEJunction* junction = getNet()->retrieveJunction(node->getID());
                            undoList->add(new GNEChange_TLS(junction, tlDef, false), true);
                            undoList->add(new GNEChange_TLS(junction, newDef, true), true);
                        }
                    } else {
                        WRITE_ERROR("Could not set attribute '" + toString(key) + "' (tls is broken)");
                    }
                }
                undoList->p_end();
            }
            break;
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEConnection::isValid(SumoXMLAttr key, const std::string& value) {
    // Currently ignored before implementation to avoid warnings
    switch (key) {
        case SUMO_ATTR_FROM:
        case SUMO_ATTR_TO:
        case SUMO_ATTR_FROM_LANE:
        case SUMO_ATTR_TO_LANE:
            return false;
        case SUMO_ATTR_PASS:
            return canParse<bool>(value);
        case SUMO_ATTR_KEEP_CLEAR:
            return canParse<bool>(value);
        case SUMO_ATTR_CONTPOS:
            return canParse<double>(value);
        case SUMO_ATTR_UNCONTROLLED:
            return canParse<bool>(value);
        case SUMO_ATTR_VISIBILITY_DISTANCE:
            return canParse<double>(value) && (parse<double>(value) > 0);
        case SUMO_ATTR_TLLINKINDEX:
            if (getNBEdgeConnection().uncontrolled == false
                    && getEdgeFrom()->getNBEdge()->getToNode()->getControllingTLS().size() > 0
                    && canParse<int>(value)
                    && parse<int>(value) >= 0) {
                NBTrafficLightDefinition* def = *getEdgeFrom()->getNBEdge()->getToNode()->getControllingTLS().begin();
                return def->getMaxValidIndex() >= parse<int>(value);
            } else {
                return false;
            }
        case SUMO_ATTR_SPEED:
            return canParse<double>(value) && (parse<double>(value) > 0);
        case SUMO_ATTR_CUSTOMSHAPE: {
            bool ok = true;
            PositionVector shape = GeomConvHelper::parseShapeReporting(value, "user-supplied shape", 0, ok, true);
            return ok;
        }
        case GNE_ATTR_SELECTED:
            return canParse<bool>(value);
        case GNE_ATTR_GENERIC:
            return isGenericParametersValid(value);
        default:
            throw InvalidArgument(toString(getTag()) + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEConnection::addGenericParameter(const std::string& key, const std::string& value) {
    if (!getNBEdgeConnection().knowsParameter(key)) {
        getNBEdgeConnection().setParameter(key, value);
        return true;
    } else {
        return false;
    }
}


bool
GNEConnection::removeGenericParameter(const std::string& key) {
    if (getNBEdgeConnection().knowsParameter(key)) {
        getNBEdgeConnection().unsetParameter(key);
        return true;
    } else {
        return false;
    }
}


bool
GNEConnection::updateGenericParameter(const std::string& oldKey, const std::string& newKey) {
    if (getNBEdgeConnection().knowsParameter(oldKey) && !getNBEdgeConnection().knowsParameter(newKey)) {
        std::string value = getNBEdgeConnection().getParameter(oldKey);
        getNBEdgeConnection().unsetParameter(oldKey);
        getNBEdgeConnection().setParameter(newKey, value);
        return true;
    } else {
        return false;
    }
}


bool
GNEConnection::updateGenericParameterValue(const std::string& key, const std::string& newValue) {
    if (getNBEdgeConnection().knowsParameter(key)) {
        getNBEdgeConnection().setParameter(key, newValue);
        return true;
    } else {
        return false;
    }
}


std::string
GNEConnection::getGenericParametersStr() const {
    std::string result;
    // Generate an string using the following structure: "key1=value1|key2=value2|...
    for (auto i : getNBEdgeConnection().getParametersMap()) {
        result += i.first + "=" + i.second + "|";
    }
    // remove the last "|"
    if (!result.empty()) {
        result.pop_back();
    }
    return result;
}


std::vector<std::pair<std::string, std::string> >
GNEConnection::getGenericParameters() const {
    std::vector<std::pair<std::string, std::string> >  result;
    // iterate over parameters map and fill result
    for (auto i : getNBEdgeConnection().getParametersMap()) {
        result.push_back(std::make_pair(i.first, i.second));
    }
    return result;
}


void
GNEConnection::setGenericParametersStr(const std::string& value) {
    // clear parameters
    getNBEdgeConnection().clearParameter();
    // separate value in a vector of string using | as separator
    std::vector<std::string> parsedValues;
    StringTokenizer stValues(value, "|", true);
    while (stValues.hasNext()) {
        parsedValues.push_back(stValues.next());
    }
    // check that parsed values (A=B)can be parsed in generic parameters
    for (auto i : parsedValues) {
        std::vector<std::string> parsedParameters;
        StringTokenizer stParam(i, "=", true);
        while (stParam.hasNext()) {
            parsedParameters.push_back(stParam.next());
        }
        // Check that parsed parameters are exactly two and contains valid chracters
        if (parsedParameters.size() == 2 && SUMOXMLDefinitions::isValidGenericParameterKey(parsedParameters.front()) && SUMOXMLDefinitions::isValidGenericParameterValue(parsedParameters.back())) {
            getNBEdgeConnection().setParameter(parsedParameters.front(), parsedParameters.back());
        }
    }
}

// ===========================================================================
// private
// ===========================================================================

void
GNEConnection::setAttribute(SumoXMLAttr key, const std::string& value) {
    NBEdge::Connection& nbCon = getNBEdgeConnection();
    switch (key) {
        case SUMO_ATTR_PASS:
            nbCon.mayDefinitelyPass = parse<bool>(value);
            break;
        case SUMO_ATTR_KEEP_CLEAR:
            nbCon.keepClear = parse<bool>(value);
            break;
        case SUMO_ATTR_UNCONTROLLED:
            nbCon.uncontrolled = parse<bool>(value);
            break;
        case SUMO_ATTR_CONTPOS:
            nbCon.contPos = parse<double>(value);
            break;
        case SUMO_ATTR_VISIBILITY_DISTANCE:
            nbCon.visibility = parse<double>(value);
            break;
        case SUMO_ATTR_SPEED:
            nbCon.speed = parse<double>(value);
            break;
        case SUMO_ATTR_CUSTOMSHAPE: {
            bool ok;
            const bool init = (myShape.size() == 0);
            if (!init) {
                // first remove object from net grid
                myNet->removeGLObjectFromGrid(this);
            }
            nbCon.customShape = GeomConvHelper::parseShapeReporting(value, "user-supplied shape", 0, ok, true);
            if (!init) {
                // add object into net again
                myNet->addGLObjectIntoGrid(this);
            }
            break;
        }
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


void
GNEConnection::mouseOverObject(const GUIVisualizationSettings&) const {
}


/****************************************************************************/
