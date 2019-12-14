/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    GNENetElement.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2016
/// @version $Id$
///
// A abstract class for netElements
/****************************************************************************/


// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <netedit/additionals/GNEAdditional.h>
#include <netedit/GNENet.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>

#include "GNENetElement.h"


// ===========================================================================
// method definitions
// ===========================================================================


GNENetElement::GNENetElement(GNENet* net, const std::string& id, GUIGlObjectType type, SumoXMLTag tag) :
    GUIGlObject(type, id),
    GNEAttributeCarrier(tag),
    myNet(net),
    myMovingGeometryBoundary() {}


GNENetElement::~GNENetElement() {}


GNENet*
GNENetElement::getNet() const {
    return myNet;
}


void
GNENetElement::addAdditionalParent(GNEAdditional* additional) {
    // First check that additional wasn't already inserted
    if (std::find(myFirstAdditionalParents.begin(), myFirstAdditionalParents.end(), additional) != myFirstAdditionalParents.end()) {
        throw ProcessError(toString(additional->getTag()) + " with ID='" + additional->getID() + "' was already inserted in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else {
        myFirstAdditionalParents.push_back(additional);
        // update geometry is needed for stacked additionals (routeProbes and Vaporicers)
        updateGeometry(true);
    }
}


void
GNENetElement::removeAdditionalParent(GNEAdditional* additional) {
    // First check that additional was already inserted
    auto it = std::find(myFirstAdditionalParents.begin(), myFirstAdditionalParents.end(), additional);
    if (it == myFirstAdditionalParents.end()) {
        throw ProcessError(toString(additional->getTag()) + " with ID='" + additional->getID() + "' doesn't exist in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else {
        myFirstAdditionalParents.erase(it);
        // update geometry is needed for stacked additionals (routeProbes and Vaporizers)
        updateGeometry(true);
    }
}


void
GNENetElement::addAdditionalChild(GNEAdditional* additional) {
    // First check that additional wasn't already inserted
    if (std::find(myAdditionalChilds.begin(), myAdditionalChilds.end(), additional) != myAdditionalChilds.end()) {
        throw ProcessError(toString(additional->getTag()) + " with ID='" + additional->getID() + "' was already inserted in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else {
        myAdditionalChilds.push_back(additional);
        // update geometry is needed for stacked additionals (routeProbes and Vaporicers)
        updateGeometry(true);
    }
}


void
GNENetElement::removeAdditionalChild(GNEAdditional* additional) {
    // First check that additional was already inserted
    auto it = std::find(myAdditionalChilds.begin(), myAdditionalChilds.end(), additional);
    if (it == myAdditionalChilds.end()) {
        throw ProcessError(toString(additional->getTag()) + " with ID='" + additional->getID() + "' doesn't exist in " + toString(getTag()) + " with ID='" + getID() + "'");
    } else {
        myAdditionalChilds.erase(it);
        // update geometry is needed for stacked additionals (routeProbes and Vaporizers)
        updateGeometry(true);
    }
}


const std::vector<GNEAdditional*>&
GNENetElement::getAdditionalParents() const {
    return myFirstAdditionalParents;
}


const std::vector<GNEAdditional*>&
GNENetElement::getAdditionalChilds() const {
    return myAdditionalChilds;
}


GUIParameterTableWindow*
GNENetElement::getParameterWindow(GUIMainWindow& app, GUISUMOAbstractView&) {
    // Create table
    GUIParameterTableWindow* ret = new GUIParameterTableWindow(app, *this, getTagProperties(getTag()).getNumberOfAttributes());
    // Iterate over attributes
    for (auto i : getTagProperties(getTag())) {
        // Add attribute and set it dynamic if aren't unique
        if (i.second.isUnique()) {
            ret->mkItem(toString(i.first).c_str(), false, getAttribute(i.first));
        } else {
            ret->mkItem(toString(i.first).c_str(), true, getAttribute(i.first));
        }
    }
    // close building
    ret->closeBuilding();
    return ret;
}


void
GNENetElement::selectAttributeCarrier(bool changeFlag) {
    if (!myNet) {
        throw ProcessError("Net cannot be nullptr");
    } else {
        gSelected.select(dynamic_cast<GUIGlObject*>(this)->getGlID());
        if (changeFlag) {
            mySelected = true;
        }
    }
}


void
GNENetElement::unselectAttributeCarrier(bool changeFlag) {
    if (!myNet) {
        throw ProcessError("Net cannot be nullptr");
    } else {
        gSelected.deselect(dynamic_cast<GUIGlObject*>(this)->getGlID());
        if (changeFlag) {
            mySelected = false;
        }
    }
}


bool
GNENetElement::isAttributeCarrierSelected() const {
    return mySelected;
}


std::string
GNENetElement::getPopUpID() const {
    if (getTag() == SUMO_TAG_CONNECTION) {
        return getAttribute(SUMO_ATTR_FROM) + "_" + getAttribute(SUMO_ATTR_FROM_LANE) + " -> " + getAttribute(SUMO_ATTR_TO) + "_" + getAttribute(SUMO_ATTR_TO_LANE);
    } else {
        return toString(getTag()) + ": " + getID();
    }
}


std::string
GNENetElement::getHierarchyName() const {
    if (getTag() == SUMO_TAG_LANE) {
        return toString(SUMO_TAG_LANE) + " " + getAttribute(SUMO_ATTR_INDEX);
    } else if (getTag() == SUMO_TAG_CONNECTION) {
        return getAttribute(SUMO_ATTR_FROM_LANE) + " -> " + getAttribute(SUMO_ATTR_TO_LANE);
    } else if (getTag() == SUMO_TAG_CROSSING) {
        return toString(SUMO_TAG_CROSSING) + " " + getAttribute(SUMO_ATTR_ID);
    } else {
        return toString(getTag());
    }
}

/****************************************************************************/
