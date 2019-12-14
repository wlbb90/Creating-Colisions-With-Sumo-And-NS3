/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNEChange_Lane.cpp
/// @author  Jakob Erdmann
/// @date    April 2011
/// @version $Id$
///
// A network change in which a single lane is created or deleted
/****************************************************************************/

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <netedit/GNENet.h>
#include <netedit/GNEViewNet.h>
#include <netedit/netelements/GNEEdge.h>
#include <netedit/netelements/GNELane.h>
#include <netedit/frames/GNEInspectorFrame.h>
#include <netedit/GNEViewParent.h>
#include <cassert>

#include "GNEChange_Lane.h"


// ===========================================================================
// FOX-declarations
// ===========================================================================
FXIMPLEMENT_ABSTRACT(GNEChange_Lane, GNEChange, nullptr, 0)

// ===========================================================================
// member method definitions
// ===========================================================================


/// @brief constructor for creating an edge
GNEChange_Lane::GNEChange_Lane(GNEEdge* edge, GNELane* lane, const NBEdge::Lane& laneAttrs, bool forward, bool recomputeConnections):
    GNEChange(edge->getNet(), forward),
    myEdge(edge),
    myLane(lane),
    myLaneAttrs(laneAttrs),
    myRecomputeConnections(recomputeConnections) {
    assert(myNet);
    myEdge->incRef("GNEChange_Lane");
    if (myLane) {
        // non-zero pointer is passsed in case of removal or duplication
        myLane->incRef("GNEChange_Lane");
        // Save additionals of lane
        myAdditionalChilds = myLane->getAdditionalChilds();
        // Save POILanes of lane
        myShapeChilds = myLane->getShapeChilds();
    } else {
        assert(forward);
    }
}


GNEChange_Lane::~GNEChange_Lane() {
    assert(myEdge);
    myEdge->decRef("GNEChange_Lane");
    if (myEdge->unreferenced()) {
        // show extra information for tests
        WRITE_DEBUG("Deleting unreferenced " + toString(myEdge->getTag()) + " '" + myEdge->getID() + "' in GNEChange_Lane");
        delete myEdge;
    }
    if (myLane) {
        myLane->decRef("GNEChange_Lane");
        if (myLane->unreferenced()) {
            // show extra information for tests
            WRITE_DEBUG("Deleting unreferenced " + toString(myLane->getTag()) + " '" + myLane->getID() + "' in GNEChange_Lane");
            delete myLane;
        }
    }
}


void
GNEChange_Lane::undo() {
    if (myForward) {
        // show extra information for tests
        if (myLane != nullptr) {
            WRITE_DEBUG("Removing " + toString(myLane->getTag()) + " '" + myLane->getID() + "' from " + toString(SUMO_TAG_EDGE));
        } else {
            WRITE_DEBUG("Removing nullptr " + toString(SUMO_TAG_LANE) + " from " + toString(SUMO_TAG_EDGE));
        }
        // remove lane from edge
        myEdge->removeLane(myLane, false);
        // Remove additionals vinculated with this lane
        for (auto i : myAdditionalChilds) {
            myNet->deleteAdditional(i);
        }
        // Remove Shapes vinculated with this lane of net
        for (auto i : myShapeChilds) {
            myNet->removeShape(i);
        }
    } else {
        // show extra information for tests
        if (myLane != nullptr) {
            WRITE_DEBUG("Adding " + toString(myLane->getTag()) + " '" + myLane->getID() + "' into " + toString(SUMO_TAG_EDGE));
        } else {
            WRITE_DEBUG("Adding nullptr " + toString(SUMO_TAG_LANE) + " into " + toString(SUMO_TAG_EDGE));
        }
        // add lane and their attributes to edge
        // (lane removal is reverted, no need to recompute connections)
        myEdge->addLane(myLane, myLaneAttrs, false);
        // add additional sets vinculated with this lane of net
        for (auto i : myAdditionalChilds) {
            myNet->insertAdditional(i);
        }
        // add Shapes vinculated with this lane in net
        for (auto i : myShapeChilds) {
            myNet->removeShape(i);
        }
    }
    // check if inspector frame has to be updated
    if (myNet->getViewNet()->getViewParent()->getInspectorFrame()->shown()) {
        myNet->getViewNet()->getViewParent()->getInspectorFrame()->getACHierarchy()->refreshACHierarchy();
    }
    // enable save netElements
    myNet->requiereSaveNet(true);
}


void
GNEChange_Lane::redo() {
    if (myForward) {
        // show extra information for tests
        if (myLane != nullptr) {
            WRITE_DEBUG("Adding " + toString(myLane->getTag()) + " '" + myLane->getID() + "' into " + toString(SUMO_TAG_EDGE));
        } else {
            WRITE_DEBUG("Adding nullptr " + toString(SUMO_TAG_LANE) + " into " + toString(SUMO_TAG_EDGE));
        }
        // add lane and their attributes to edge
        myEdge->addLane(myLane, myLaneAttrs, myRecomputeConnections);
        // add additional vinculated with this lane of net
        for (auto i : myAdditionalChilds) {
            myNet->insertAdditional(i);
        }
        // add shapes vinculated with this lane in net
        for (auto i : myShapeChilds) {
            myNet->removeShape(i);
        }
    } else {
        // show extra information for tests
        if (myLane != nullptr) {
            WRITE_DEBUG("Removing " + toString(myLane->getTag()) + " '" + myLane->getID() + "' from " + toString(SUMO_TAG_EDGE));
        } else {
            WRITE_DEBUG("Removing nullptr " + toString(SUMO_TAG_LANE) + " from " + toString(SUMO_TAG_EDGE));
        }
        // remove lane from edge
        myEdge->removeLane(myLane, myRecomputeConnections);
        // Remove additional vinculated with this lane of net
        for (auto i : myAdditionalChilds) {
            myNet->deleteAdditional(i);
        }
        // Remove shapes vinculated with this lane of net
        for (auto i : myShapeChilds) {
            myNet->removeShape(i);
        }
    }
    // check if inspector frame has to be updated
    if (myNet->getViewNet()->getViewParent()->getInspectorFrame()->shown()) {
        myNet->getViewNet()->getViewParent()->getInspectorFrame()->getACHierarchy()->refreshACHierarchy();
    }
    // enable save netElements
    myNet->requiereSaveNet(true);
}


FXString
GNEChange_Lane::undoName() const {
    if (myForward) {
        return ("Undo create " + toString(SUMO_TAG_LANE)).c_str();
    } else {
        return ("Undo delete " + toString(SUMO_TAG_LANE)).c_str();
    }
}


FXString
GNEChange_Lane::redoName() const {
    if (myForward) {
        return ("Redo create " + toString(SUMO_TAG_LANE)).c_str();
    } else {
        return ("Redo delete " + toString(SUMO_TAG_LANE)).c_str();
    }
}
