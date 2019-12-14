/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEChange_Shape.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Oct 2017
/// @version $Id$
///
// A network change in which a single poly is created or deleted
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/common/RGBColor.h>
#include <utils/geom/PositionVector.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNENet.h>
#include <netedit/additionals/GNEShape.h>
#include <netedit/frames/GNEInspectorFrame.h>
#include <netedit/GNEViewParent.h>

#include "GNEChange_Shape.h"

// ===========================================================================
// FOX-declarations
// ===========================================================================
FXIMPLEMENT_ABSTRACT(GNEChange_Shape, GNEChange, nullptr, 0)

// ===========================================================================
// member method definitions
// ===========================================================================

GNEChange_Shape::GNEChange_Shape(GNEShape* shape, bool forward) :
    GNEChange(shape->getNet(), forward),
    myShape(shape) {
    assert(myNet);
    myShape->incRef("GNEChange_Shape");
}


GNEChange_Shape::~GNEChange_Shape() {
    assert(myShape);
    myShape->decRef("GNEChange_Shape");
    if (myShape->unreferenced()) {
        // make sure that shape are removed of ShapeContainer (net) AND grid
        if (myNet->retrievePolygon(myShape->getID(), false) != nullptr) {
            // show extra information for tests
            WRITE_DEBUG("Removing " + toString(myShape->getTag()) + " '" + myShape->getID() + "' from net in ~GNEChange_Shape()");
            myNet->removeGLObjectFromGrid(dynamic_cast<GUIGlObject*>(myShape));
            myNet->myPolygons.remove(myShape->getID(), false);
        } else if (myNet->retrievePOI(myShape->getID(), false) != nullptr) {
            // show extra information for tests
            WRITE_DEBUG("Removing " + toString(myShape->getTag()) + " '" + myShape->getID() + "' from net in ~GNEChange_Shape()");
            myNet->removeGLObjectFromGrid(dynamic_cast<GUIGlObject*>(myShape));
            myNet->myPOIs.remove(myShape->getID(), false);
        }
        // show extra information for tests
        WRITE_DEBUG("delete " + toString(myShape->getTag()) + " '" + myShape->getID() + "' in ~GNEChange_Shape()");
        delete myShape;
    }
}


void
GNEChange_Shape::undo() {
    if (myForward) {
        // show extra information for tests
        WRITE_DEBUG("Removing " + toString(myShape->getTag()) + " '" + myShape->getID() + "' from viewNet");
        // remove shape from net
        myNet->removeShape(myShape);
    } else {
        // show extra information for tests
        WRITE_DEBUG("Adding " + toString(myShape->getTag()) + " '" + myShape->getID() + "' into viewNet");
        // Add shape in net
        myNet->insertShape(myShape);
    }
    // check if inspector frame has to be updated
    if (myNet->getViewNet()->getViewParent()->getInspectorFrame()->shown()) {
        myNet->getViewNet()->getViewParent()->getInspectorFrame()->getACHierarchy()->refreshACHierarchy();
    }
}


void
GNEChange_Shape::redo() {
    if (myForward) {
        // show extra information for tests
        WRITE_DEBUG("Adding " + toString(myShape->getTag()) + " '" + myShape->getID() + "' into viewNet");
        // Add shape in net
        myNet->insertShape(myShape);
    } else {
        // show extra information for tests
        WRITE_DEBUG("Removing " + toString(myShape->getTag()) + " '" + myShape->getID() + "' from viewNet");
        // remove shape from net
        myNet->removeShape(myShape);
    }
    // check if inspector frame has to be updated
    if (myNet->getViewNet()->getViewParent()->getInspectorFrame()->shown()) {
        myNet->getViewNet()->getViewParent()->getInspectorFrame()->getACHierarchy()->refreshACHierarchy();
    }
}


FXString
GNEChange_Shape::undoName() const {
    if (myForward) {
        return ("Undo create " + toString(myShape->getTag())).c_str();
    } else {
        return ("Undo delete " + toString(myShape->getTag())).c_str();
    }
}


FXString
GNEChange_Shape::redoName() const {
    if (myForward) {
        return ("Redo create " + toString(myShape->getTag())).c_str();
    } else {
        return ("Redo delete " + toString(myShape->getTag())).c_str();
    }
}
